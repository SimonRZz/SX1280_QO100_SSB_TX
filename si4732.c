#include "si4732.h"

#include <stdarg.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// The SSB patch is a third-party blob, kept out of the repository. Without it
// the sideband modes are unavailable; AM still works.
#if defined(__has_include)
#  if __has_include("ssb_patch.h")
#    include "ssb_patch.h"
#    define SI4732_SSB_PATCH 1
#  endif
#endif

// ---------------------------------------------------------------------------
// Bus and pins. I2C0 is brought up by gpsdo_init() for the SI5351; we share it.
// ---------------------------------------------------------------------------
#define SI_I2C          i2c0
#define SI_PIN_SCL      1u     // needed during reset, see si_hw_reset()
#define SI_PIN_RST      13u

// Both documented control-bus addresses. The SENB polarity differs between the
// Si4732-A10 and the Si4735-D60 that most documentation describes, so rather
// than trust either datasheet reading we probe both and report what answered.
#define SI_ADDR_A       0x11u
#define SI_ADDR_B       0x63u

// ---------------------------------------------------------------------------
// Commands and properties (verified against the PU2CLR SI4735 library headers,
// which in turn cite AN332).
// ---------------------------------------------------------------------------
#define CMD_POWER_UP        0x01u
#define CMD_GET_REV         0x10u
#define CMD_POWER_DOWN      0x11u
#define CMD_SET_PROPERTY    0x12u
#define CMD_TUNE_FREQ       0x40u   // AM_TUNE_FREQ and SSB_TUNE_FREQ are the same
#define CMD_RSQ_STATUS      0x43u

#define PROP_SSB_BFO             0x0100u
#define PROP_SSB_MODE            0x0101u
#define PROP_REFCLK_FREQ         0x0201u
#define PROP_REFCLK_PRESCALE     0x0202u
#define PROP_AM_CHANNEL_FILTER   0x3102u
#define PROP_AM_SOFT_MUTE_MAX_AT 0x3302u
#define PROP_RX_VOLUME           0x4000u
#define PROP_RX_HARD_MUTE        0x4001u

// POWER_UP ARG1 bits
#define PU_PATCH            0x20u   // expect a patch download to follow
#define PU_XOSCEN           0x10u   // 1 = internal crystal oscillator; we want 0
#define PU_FUNC_AM          0x01u   // AM/SW/LW — also the host for SSB
// POWER_UP ARG2
#define PU_OPMODE_ANALOG    0x05u

// TUNE_FREQ ARG1 bits: bit0 FAST, bit1 FREEZE, bits2-5 reserved, bits6-7 sideband
#define TUNE_FAST           0x01u
#define TUNE_SB_LSB         (1u << 6)
#define TUNE_SB_USB         (2u << 6)

// ---------------------------------------------------------------------------
// Reference clock. 25 MHz from the second GPSDO, divided down into the window
// the chip accepts. 25e6 / 800 = 31250 Hz exactly — no rounding error to carry
// into the tuning.
// ---------------------------------------------------------------------------
#define SI_RCLK_HZ          25000000u
#define SI_RCLK_PRESCALE    800u
#define SI_REFCLK_FREQ      (SI_RCLK_HZ / SI_RCLK_PRESCALE)

// AN332 gives the usable REFCLK_FREQ window as 31130..34406 Hz and caps the
// prescaler at 4095. Catch a bad edit here rather than on the bench.
_Static_assert(SI_RCLK_HZ % SI_RCLK_PRESCALE == 0,
               "RCLK must divide exactly, otherwise the tuning carries an offset");
_Static_assert(SI_REFCLK_FREQ >= 31130u && SI_REFCLK_FREQ <= 34406u,
               "REFCLK_FREQ outside the window the Si473x accepts");
_Static_assert(SI_RCLK_PRESCALE >= 1u && SI_RCLK_PRESCALE <= 4095u,
               "REFCLK_PRESCALE is a 12-bit field");

#define SI_KHZ_MIN          2300u    // SW band, per the Si4732-A10 data sheet
#define SI_KHZ_MAX          26100u

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static uint8_t       s_addr      = 0x00u;
static bool          s_present   = false;
static bool          s_on        = false;
static si4732_mode_t s_mode      = SI4732_MODE_USB;
static uint32_t      s_hz        = 19500000u; // middle of the planned IF
static uint32_t      s_khz       = 19500u;    // what the tuner was actually given
static int32_t       s_cal_hz    = 0;         // LNB LO error, see si4732.h
static uint8_t       s_volume    = 32u;
static uint16_t      s_bw_hz     = 2200u;
static int16_t       s_bfo_hz    = 0;
static bool          s_mute      = false;
static uint8_t       s_rev_pn    = 0u;
static uint8_t       s_rev_fw[2] = { 0u, 0u };
static uint8_t       s_rev_raw[9] = { 0 };   // last GET_REV response, for diag
static bool          s_rev_valid = false;
static const char   *s_last_err  = "none";

static si4732_log_fn s_log       = NULL;

void si4732_set_log(si4732_log_fn fn) { s_log = fn; }

// Formats one line and hands it to the sink. Silently dropped when no sink is
// registered, which keeps the driver usable before the console exists.
static void si_log(const char *fmt, ...)
{
    if (!s_log) return;
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0) s_log(buf);
}

// ---------------------------------------------------------------------------
// Low-level bus helpers
// ---------------------------------------------------------------------------

// The chip raises CTS in bit 7 of its status byte when it has finished the
// previous command. Every command must be followed by this wait; the patch
// upload in particular overruns silently without it.
static bool si_wait_cts(uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    for (;;) {
        uint8_t st = 0u;
        if (i2c_read_blocking(SI_I2C, s_addr, &st, 1, false) == 1) {
            // 0xFF is what an idle bus reads: SDA held high by the pull-up
            // because nothing is driving it. Bit 7 is set in that pattern, so
            // a plain "CTS is bit 7" test passes on a chip that is not talking
            // at all — which is how a patch upload once reported 1979
            // successful blocks into a silent device. Bit 6 is ERR, and no real
            // status has both CTS and ERR plus every other bit set, so treat
            // 0xFF as "no answer".
            if (st == 0xFFu) {
                s_last_err = "no answer (bus reads 0xFF)";
            } else if (st & 0x80u) {
                return true;
            }
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            if (s_last_err != (const char *)0 && s_last_err[0] == 'n')
                return false;              // keep the more specific message
            s_last_err = "CTS timeout";
            return false;
        }
        sleep_us(150);
    }
}

static bool si_cmd(const uint8_t *buf, size_t len)
{
    if (i2c_write_blocking(SI_I2C, s_addr, buf, len, false) != (int)len) {
        s_last_err = "I2C write failed";
        return false;
    }
    return si_wait_cts(100u);
}

static bool si_set_prop(uint16_t prop, uint16_t val)
{
    const uint8_t b[6] = {
        CMD_SET_PROPERTY, 0x00u,
        (uint8_t)(prop >> 8), (uint8_t)prop,
        (uint8_t)(val >> 8),  (uint8_t)val
    };
    return si_cmd(b, sizeof b);
}

// Hold RST low, then release it with SCL parked high.
//
// Two things the modules handle for you and a bare chip does not. First, the
// bus mode is latched from GPO1/GPO2 on the RISING edge of RST: left open,
// their internal pull-up and pull-down select 2-wire mode, but that only
// settles if RST stays low for at least 100 us. Second, SCLK must be high
// across that edge and must not see a start condition in the 300 ns before it.
// The SI5351 shares this bus, so a transfer in flight would leave SCL low at
// exactly the wrong moment — we take the pin away from the I2C block for the
// duration and drive it high by hand.
static void si_hw_reset(void)
{
    gpio_set_function(SI_PIN_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(SI_PIN_SCL, GPIO_OUT);
    gpio_put(SI_PIN_SCL, 1);

    gpio_init(SI_PIN_RST);
    gpio_set_dir(SI_PIN_RST, GPIO_OUT);
    gpio_put(SI_PIN_RST, 0);
    sleep_ms(5);              // well past the 100 us minimum
    gpio_put(SI_PIN_RST, 1);
    sleep_us(500);            // GPO1/GPO2 are sampled and released here

    gpio_set_function(SI_PIN_SCL, GPIO_FUNC_I2C);
    sleep_ms(5);
}

static bool si_probe(uint8_t addr)
{
    uint8_t dummy;
    return i2c_read_blocking(SI_I2C, addr, &dummy, 1, false) == 1;
}

// ---------------------------------------------------------------------------
// Bandwidth tables. The chip offers a fixed set of filters, different ones in
// SSB and in AM, so a requested width is snapped to the nearest available.
// ---------------------------------------------------------------------------
typedef struct { uint16_t hz; uint8_t code; } bw_entry_t;

// SSB_MODE AUDIOBW field
static const bw_entry_t SSB_BW[] = {
    {  500u, 4u }, { 1000u, 5u }, { 1200u, 0u },
    { 2200u, 1u }, { 3000u, 2u }, { 4000u, 3u },
};
// AM_CHANNEL_FILTER AMCHFLT field
static const bw_entry_t AM_BW[] = {
    { 1000u, 4u }, { 1800u, 5u }, { 2000u, 3u }, { 2500u, 6u },
    { 3000u, 2u }, { 4000u, 1u }, { 6000u, 0u },
};

static const bw_entry_t *bw_nearest(uint16_t want, bool ssb)
{
    const bw_entry_t *tab = ssb ? SSB_BW : AM_BW;
    size_t n = ssb ? (sizeof SSB_BW / sizeof SSB_BW[0])
                   : (sizeof AM_BW  / sizeof AM_BW[0]);
    const bw_entry_t *best = &tab[0];
    uint32_t best_d = UINT32_MAX;
    for (size_t i = 0; i < n; i++) {
        uint32_t d = (tab[i].hz > want) ? (uint32_t)(tab[i].hz - want)
                                        : (uint32_t)(want - tab[i].hz);
        if (d < best_d) { best_d = d; best = &tab[i]; }
    }
    return best;
}

static bool si_apply_bandwidth(void)
{
    bool ssb = (s_mode != SI4732_MODE_AM);
    const bw_entry_t *e = bw_nearest(s_bw_hz, ssb);
    s_bw_hz = e->hz;

    if (ssb) {
        // SSB_MODE: AUDIOBW in bits 3:0, SBCUTFLT 7:4, then AVC_DIVIDER 11:8,
        // AVCEN 12, SMUTESEL 13, DSP_AFCDIS 15.
        // AVC on, AFC off (AFC belongs to SYNC mode and would drag the tuning).
        uint16_t val = (uint16_t)(0x9000u | e->code);
        return si_set_prop(PROP_SSB_MODE, val);
    }
    return si_set_prop(PROP_AM_CHANNEL_FILTER, e->code);
}

// ---------------------------------------------------------------------------
// Patch upload
// ---------------------------------------------------------------------------
#ifdef SI4732_SSB_PATCH
// The patch is a stream of 8-byte commands; the first byte of each group is
// the opcode, so it goes out verbatim with no register address in front.
static bool si_download_patch(void)
{
    absolute_time_t t0 = get_absolute_time();

    for (size_t off = 0; off < SSB_PATCH_SIZE; off += 8u) {
        if (i2c_write_blocking(SI_I2C, s_addr, &ssb_patch_content[off], 8, false) != 8) {
            s_last_err = "patch write failed";
            return false;
        }
        if (!si_wait_cts(50u)) {
            s_last_err = "patch stalled";
            return false;
        }
    }

    int ms = (int)(absolute_time_diff_us(t0, get_absolute_time()) / 1000);
    si_log("[SI4732] SSB patch uploaded: %u bytes in %d ms",
           (unsigned)SSB_PATCH_SIZE, ms);
    return true;
}
#endif

// ---------------------------------------------------------------------------
// Power up / tune
// ---------------------------------------------------------------------------
static bool si_power_up(bool with_patch)
{
    // ARG1: CTSIEN=0, GPO2OEN=0, PATCH as asked, XOSCEN=0 (external RCLK),
    // FUNC=1 (AM/SSB). ARG2: analog audio out.
    const uint8_t b[3] = {
        CMD_POWER_UP,
        (uint8_t)((with_patch ? PU_PATCH : 0u) | PU_FUNC_AM),
        PU_OPMODE_ANALOG
    };
    if (i2c_write_blocking(SI_I2C, s_addr, b, sizeof b, false) != (int)sizeof b) {
        s_last_err = "POWER_UP write failed";
        return false;
    }
    sleep_ms(10);                 // chip boot
    return si_wait_cts(500u);
}

static bool si_tune(void)
{
    uint8_t arg1 = 0u;
    if (s_mode == SI4732_MODE_LSB)      arg1 = TUNE_SB_LSB | TUNE_FAST;
    else if (s_mode == SI4732_MODE_USB) arg1 = TUNE_SB_USB | TUNE_FAST;

    uint16_t khz = (uint16_t)s_khz;
    const uint8_t b[6] = {
        CMD_TUNE_FREQ, arg1,
        (uint8_t)(khz >> 8), (uint8_t)khz,
        0x00u, 0x00u              // ANTCAP = 0: automatic, irrelevant on a
    };                            // 50 ohm driven input
    if (!si_cmd(b, sizeof b)) return false;
    sleep_ms(30);                 // let the tune settle before RSQ is read
    return true;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool si4732_init(void)
{
    s_present = false;
    s_on      = false;
    s_addr    = 0x00u;

    si_hw_reset();

    if (si_probe(SI_ADDR_A))      s_addr = SI_ADDR_A;
    else if (si_probe(SI_ADDR_B)) s_addr = SI_ADDR_B;
    else {
        s_last_err = "no answer at 0x11 or 0x63";
        si_log("[SI4732] not found (tried 0x11 and 0x63)");
        return false;
    }

    // GET_REV also proves the bus mode latched correctly: in SPI or 3-wire mode
    // this read returns nothing sensible.
    const uint8_t rev_cmd[1] = { CMD_GET_REV };
    if (!si_cmd(rev_cmd, 1)) {
        si_log("[SI4732] found at 0x%02X but GET_REV failed (%s)",
               s_addr, s_last_err);
        return false;
    }
    int got = i2c_read_blocking(SI_I2C, s_addr, s_rev_raw, sizeof s_rev_raw, false);
    s_rev_valid = (got == (int)sizeof s_rev_raw);
    if (s_rev_valid) {
        s_rev_pn    = s_rev_raw[1];
        s_rev_fw[0] = s_rev_raw[2];
        s_rev_fw[1] = s_rev_raw[3];
    }

    // A real part reports a part number. All zeros means the read came back
    // empty, all ones means nothing was driving the bus — in both cases the
    // address acknowledged but the device is not actually answering, and
    // carrying on would only produce writes that fail later with no clue why.
    //
    // The usual cause is a missing reference clock: POWER_UP runs with
    // XOSCEN = 0, so without the 25 MHz on pin 13 the chip has no clock for its
    // digital core. After that, reset and the GPO1/GPO2 bus-mode latch.
    if (!s_rev_valid || s_rev_pn == 0x00u || s_rev_pn == 0xFFu) {
        s_last_err = "GET_REV returned no part number";
        si_log("[SI4732] 0x%02X acknowledges but does not answer: GET_REV gave "
               "part=%u (%s)", s_addr, s_rev_pn,
               s_rev_valid ? "read ok, data empty" : "read failed");
        si_log("[SI4732]   check in this order: 25 MHz on pin 13 ('rclk'), "
               "3V3 on pin 14, RST on GP13, GPO1+GPO2 left open");
        return false;
    }

    s_present  = true;
    s_last_err = "none";
    si_log("[SI4732] found at 0x%02X, part %u, fw %c.%c, SSB patch %s",
           s_addr, s_rev_pn,
           s_rev_fw[0] ? s_rev_fw[0] : '?', s_rev_fw[1] ? s_rev_fw[1] : '?',
#ifdef SI4732_SSB_PATCH
           "compiled in");
#else
           "MISSING - run tools/fetch_ssb_patch.sh for SSB");
#endif
    return true;
}

bool si4732_present(void)  { return s_present; }
uint8_t si4732_i2c_addr(void) { return s_addr; }
bool si4732_is_on(void)    { return s_on; }
si4732_mode_t si4732_get_mode(void) { return s_mode; }
uint32_t si4732_get_khz(void)       { return s_khz; }
uint8_t  si4732_get_volume(void)    { return s_volume; }
uint16_t si4732_get_bandwidth_hz(void) { return s_bw_hz; }
int16_t  si4732_get_bfo_hz(void)    { return s_bfo_hz; }
bool     si4732_get_mute(void)      { return s_mute; }

bool si4732_has_ssb(void)
{
#ifdef SI4732_SSB_PATCH
    return true;
#else
    return false;
#endif
}

const char *si4732_mode_name(si4732_mode_t mode)
{
    switch (mode) {
        case SI4732_MODE_LSB: return "lsb";
        case SI4732_MODE_USB: return "usb";
        default:              return "am";
    }
}

bool si4732_set_mode(si4732_mode_t mode)
{
    if (!s_present) return false;

    if (mode != SI4732_MODE_AM && !si4732_has_ssb()) {
        s_last_err = "no SSB patch in this build";
        return false;
    }

    // The patch lives in volatile memory, so switching into a sideband mode
    // means a fresh reset and a fresh upload every time.
    si_hw_reset();
    if (!si_probe(s_addr)) { s_last_err = "gone after reset"; s_on = false; return false; }

    bool want_patch = (mode != SI4732_MODE_AM);
    if (!si_power_up(want_patch)) { s_on = false; return false; }

#ifdef SI4732_SSB_PATCH
    if (want_patch && !si_download_patch()) { s_on = false; return false; }
#endif

    s_mode = mode;
    s_on   = true;

    // Reference clock first: everything downstream derives its frequency from
    // it, so setting it after tuning would leave the first tune offset.
    bool ok = si_set_prop(PROP_REFCLK_PRESCALE, SI_RCLK_PRESCALE);
    ok = si_set_prop(PROP_REFCLK_FREQ, SI_REFCLK_FREQ) && ok;

    // Soft mute fights weak SSB signals by ducking exactly when you want to
    // hear them. Off.
    ok = si_set_prop(PROP_AM_SOFT_MUTE_MAX_AT, 0u) && ok;

    ok = si_apply_bandwidth() && ok;
    ok = si4732_set_volume(s_volume) && ok;
    ok = si4732_set_mute(s_mute) && ok;
    if (want_patch) ok = si4732_set_bfo_hz(s_bfo_hz) && ok;
    ok = si_tune() && ok;

    return ok;
}

void si4732_power_down(void)
{
    if (!s_present || !s_on) { s_on = false; return; }
    const uint8_t b[1] = { CMD_POWER_DOWN };
    (void)si_cmd(b, 1);
    s_on = false;
}

bool si4732_tune_khz(uint32_t khz)
{
    return si4732_tune_hz(khz * 1000u);
}

uint32_t si4732_get_hz(void) { return s_hz; }

// Splits the wanted frequency into what the tuner can take and what the BFO
// has to make up. The calibration offset is added here, in one place, so
// everything above this layer can work in true downlink terms and stay unaware
// of the LNB's error.
//
// SIGN CONVENTION OF THE BFO IS ASSUMED, NOT VERIFIED: positive BFO is taken to
// shift the received frequency up, the same direction as the tuner. Check it on
// the beacon — tune to it, then "rx bfo 500" and listen whether the tone rises
// or falls. If it falls, the one negation below is wrong and nothing else is.
bool si4732_tune_hz(uint32_t hz)
{
    if (!s_present) return false;

    int64_t want = (int64_t)hz + (int64_t)s_cal_hz;
    if (want < (int64_t)SI_KHZ_MIN * 1000 || want > (int64_t)SI_KHZ_MAX * 1000) {
        s_last_err = "frequency outside 2300..26100 kHz after calibration";
        return false;
    }

    uint32_t khz = (uint32_t)((want + 500) / 1000);          // nearest kHz
    int32_t  rem = (int32_t)(want - (int64_t)khz * 1000);    // -500..+499 Hz

    s_hz     = hz;
    s_khz    = khz;
    s_bfo_hz = (int16_t)rem;

    if (!s_on) return true;       // remembered, applied at power-up

    bool ok = si_tune();
    if (s_mode != SI4732_MODE_AM)
        ok = si_set_prop(PROP_SSB_BFO, (uint16_t)(int16_t)s_bfo_hz) && ok;
    return ok;
}

bool si4732_set_cal_hz(int32_t hz)
{
    // Wider than any LNB should ever be off, but narrow enough that a typo
    // cannot push the tuning out of the band.
    if (hz < -200000 || hz > 200000) {
        s_last_err = "calibration outside +-200 kHz";
        return false;
    }
    s_cal_hz = hz;
    return si4732_tune_hz(s_hz);   // re-apply at the new offset
}

int32_t si4732_get_cal_hz(void) { return s_cal_hz; }

void si4732_restore(si4732_mode_t mode, uint32_t hz, uint8_t vol,
                    uint16_t bw_hz, int32_t cal_hz)
{
    s_mode = (mode == SI4732_MODE_LSB || mode == SI4732_MODE_USB ||
              mode == SI4732_MODE_AM) ? mode : SI4732_MODE_USB;

    s_cal_hz = (cal_hz >= -200000 && cal_hz <= 200000) ? cal_hz : 0;

    s_volume = (vol <= 63u) ? vol : 32u;

    s_bw_hz = bw_nearest((bw_hz >= 100u && bw_hz <= 8000u) ? bw_hz : 2200u,
                         s_mode != SI4732_MODE_AM)->hz;

    // Checked against the band after calibration, exactly as tuning would.
    int64_t want = (int64_t)hz + (int64_t)s_cal_hz;
    if (want < (int64_t)SI_KHZ_MIN * 1000 || want > (int64_t)SI_KHZ_MAX * 1000)
        hz = 19500000u;
    s_hz  = hz;
    s_khz = (uint32_t)(((int64_t)hz + s_cal_hz + 500) / 1000);
}

bool si4732_set_bfo_hz(int16_t hz)
{
    if (!s_present) return false;
    s_bfo_hz = hz;
    if (!s_on || s_mode == SI4732_MODE_AM) return true;
    return si_set_prop(PROP_SSB_BFO, (uint16_t)hz);
}

bool si4732_set_volume(uint8_t vol)
{
    if (vol > 63u) vol = 63u;
    s_volume = vol;
    if (!s_present || !s_on) return s_present;
    return si_set_prop(PROP_RX_VOLUME, vol);
}

bool si4732_set_bandwidth_hz(uint16_t hz)
{
    if (!s_present) return false;
    s_bw_hz = hz;
    if (!s_on) { s_bw_hz = bw_nearest(hz, s_mode != SI4732_MODE_AM)->hz; return true; }
    return si_apply_bandwidth();
}

bool si4732_set_mute(bool mute)
{
    s_mute = mute;
    if (!s_present || !s_on) return s_present;
    return si_set_prop(PROP_RX_HARD_MUTE, mute ? 0x03u : 0x00u);
}

bool si4732_read_rsq(uint8_t *rssi_dbuv, uint8_t *snr_db)
{
    if (!s_present || !s_on) return false;

    const uint8_t b[2] = { CMD_RSQ_STATUS, 0x00u };
    if (!si_cmd(b, sizeof b)) return false;

    uint8_t r[6] = { 0 };
    if (i2c_read_blocking(SI_I2C, s_addr, r, sizeof r, false) != (int)sizeof r) {
        s_last_err = "RSQ read failed";
        return false;
    }
    if (rssi_dbuv) *rssi_dbuv = r[4];
    if (snr_db)    *snr_db    = r[5];
    return true;
}

void si4732_diag_print(void)
{
    si_log("[SI4732] present=%d addr=0x%02X on=%d patch=%d",
           (int)s_present, s_addr, (int)s_on, (int)si4732_has_ssb());
    if (!s_present) {
        si_log("[SI4732]   last error: %s", s_last_err);
        si_log("[SI4732]   GET_REV raw%s: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
               s_rev_valid ? "" : " (READ FAILED)",
               s_rev_raw[0], s_rev_raw[1], s_rev_raw[2], s_rev_raw[3], s_rev_raw[4],
               s_rev_raw[5], s_rev_raw[6], s_rev_raw[7], s_rev_raw[8]);
        si_log("[SI4732]   check in this order:");
        si_log("[SI4732]     1. 25 MHz on pin 13 - 'rclk' must say 1, and POWER_UP");
        si_log("[SI4732]        runs with XOSCEN=0, so without it the chip has no clock");
        si_log("[SI4732]     2. 3V3 on pin 14, GND on pin 15, 100 nF close by");
        si_log("[SI4732]     3. RST on GP13");
        si_log("[SI4732]     4. GPO1 (pin 4) and GPO2 (pin 3) left OPEN - bus mode");
        si_log("[SI4732]     5. SENB (pin 10) tied to a level, not floating");
        return;
    }
    si_log("[SI4732]   part=%u fw=%c.%c", s_rev_pn,
           s_rev_fw[0] ? s_rev_fw[0] : '?', s_rev_fw[1] ? s_rev_fw[1] : '?');
    // The raw GET_REV bytes say more than the decoded fields when something is
    // wrong: all 00 means the chip answered with nothing, all FF means nothing
    // was driving the bus at all.
    si_log("[SI4732]   GET_REV raw%s: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
           s_rev_valid ? "" : " (READ FAILED)",
           s_rev_raw[0], s_rev_raw[1], s_rev_raw[2], s_rev_raw[3], s_rev_raw[4],
           s_rev_raw[5], s_rev_raw[6], s_rev_raw[7], s_rev_raw[8]);
    si_log("[SI4732]   mode=%s freq=%lu kHz bw=%u Hz bfo=%d Hz vol=%u mute=%d",
           si4732_mode_name(s_mode), (unsigned long)s_khz, s_bw_hz,
           (int)s_bfo_hz, s_volume, (int)s_mute);
    si_log("[SI4732]   rclk=%u Hz / %u = refclk %u Hz",
           (unsigned)SI_RCLK_HZ, (unsigned)SI_RCLK_PRESCALE, (unsigned)SI_REFCLK_FREQ);

    uint8_t rssi = 0u, snr = 0u;
    if (si4732_read_rsq(&rssi, &snr))
        si_log("[SI4732]   rssi=%u dBuV snr=%u dB", rssi, snr);
    else
        si_log("[SI4732]   rsq unavailable (%s)", s_last_err);
    si_log("[SI4732]   last error: %s", s_last_err);
}
