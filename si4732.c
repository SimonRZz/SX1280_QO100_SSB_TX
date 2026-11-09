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
#define CMD_GET_INT_STATUS  0x14u
#define CMD_TUNE_STATUS     0x42u   // AM_TUNE_STATUS == SSB_TUNE_STATUS
#define CMD_AGC_STATUS      0x47u   // AM_AGC_STATUS
#define CMD_AGC_OVERRIDE    0x48u   // AM_AGC_OVERRIDE

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
// AN332 gives the usable REFCLK_FREQ window as 31130..34406 Hz and caps the
// prescaler at 4095 (it is a 12-bit field).
#define SI_REFCLK_MIN       31130u
#define SI_REFCLK_MAX       34406u

// The reference is switchable at runtime because 25 MHz turned out to be a
// poor choice on this board: it leaks into the AM input at 65 dBuV, five
// megahertz from a 19.5 MHz IF, and holds the front-end AGC down. 36 MHz is
// the same PLL B through an even integer divider and puts the leakage 16 MHz
// away instead. See gpsdo_set_rclk_mhz(), which moves the other end.
static uint32_t s_rclk_hz    = 25000000u;
static uint16_t s_rclk_presc = 800u;        // 25e6 / 800 = 31250 Hz exactly

// Smallest exact divisor that lands REFCLK inside the window. Exactness
// matters: a remainder here is not a rounding detail, it becomes a fixed
// multiplicative offset on every frequency the chip tunes.
static uint16_t si_presc_for(uint32_t rclk_hz)
{
    uint32_t d = (rclk_hz + SI_REFCLK_MAX - 1u) / SI_REFCLK_MAX;
    if (d < 1u) d = 1u;
    for (; d <= 4095u; d++) {
        if (rclk_hz / d < SI_REFCLK_MIN) break;
        if (rclk_hz % d == 0u) return (uint16_t)d;
    }
    return 0u;
}

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
static uint16_t      s_patch_id  = 0u;    // PATCHH/PATCHL from GET_REV
static bool          s_rsq_valid = false; // RESP2: chip considers this a channel
static bool          s_rsq_afcrl = false; // RESP2: AFC railed
static bool          s_rsq_smute = false; // RESP2: soft mute engaged
static int8_t        s_rsq_off   = 0;     // RESP7: frequency offset, kHz
static uint16_t      s_read_khz  = 0u;    // frequency the tuner reports back
static bool          s_stc_ok    = false; // last tune reported seek/tune complete
// XOSCEN in POWER_UP. 0 means "external clock on RCLK, crystal amplifier off",
// which is what this design wants and what AM tunes happily with. The reference
// library uses 1 and calls it "a condition for loading the patch", so while SSB
// refuses to tune this stays switchable rather than asserted.
static bool          s_xoscen    = false;
// Antenna tuning capacitor sent with every tune. 0 = the chip tunes its own
// varactor, which is what Silicon Labs recommends and what a medium-wave
// loopstick wants. A short wire or a 50 ohm feed from a mixer has no
// inductance to resonate against, so a fixed small value may suit better:
// the capacitance is 95 fF x ANTCAP + 7 pF, range 1..6143.
static uint16_t      s_antcap    = 0u;
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
        // SSB_MODE: AUDIOBW bits 3:0, SBCUTFLT 7:4, AVC_DIVIDER 11:8,
        // AVCEN 12, SMUTESEL 13, DSP_AFCDIS 15.
        //
        // Values taken from the reference library's setSSBConfig(bw, 1, 0, 0,
        // 0, 1) rather than chosen: side-band cutoff filter on, AVC off,
        // soft-mute on SNR, AFC disabled (AFC belongs to SYNC mode and would
        // drag the tuning). An earlier version had the cutoff filter off and
        // AVC on, which is not what the parts this patch was written for use.
        uint16_t val = (uint16_t)(0x8010u | e->code);
        if (!si_set_prop(PROP_SSB_MODE, val)) return false;
        sleep_ms(25);          // the library waits here too
        return true;
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

    sleep_ms(50);   // let the chip digest it before the next command

    int ms = (int)(absolute_time_diff_us(t0, get_absolute_time()) / 1000);
    si_log("[SI4732] SSB patch uploaded: %u bytes in %d ms",
           (unsigned)SSB_PATCH_SIZE, ms);
    // Fifteen kilobytes of firmware have just been pushed into a running DSP.
    // The first tune after it was coming back as 0 kHz; give the part a
    // moment before asking it to do anything.
    sleep_ms(100);
    return true;
}
#endif

// ---------------------------------------------------------------------------
// Power up / tune
// ---------------------------------------------------------------------------
// Reads GET_REV and checks it. Only meaningful once the chip is powered up:
// before that the payload comes back empty. After a patch upload the PATCHH and
// PATCHL bytes carry the patch id, which is how we know the upload actually
// landed rather than merely that 1979 writes were acknowledged.
#define SI_PATCH_ID_FULL 0x9D29u   // amrx_6_0_1_ssbrx_patch_full_0x9D29

static bool si_read_rev(bool expect_patch)
{
    const uint8_t cmd[1] = { CMD_GET_REV };
    if (!si_cmd(cmd, 1)) return false;

    int got = i2c_read_blocking(SI_I2C, s_addr, s_rev_raw, sizeof s_rev_raw, false);
    s_rev_valid = (got == (int)sizeof s_rev_raw);
    if (!s_rev_valid) { s_last_err = "GET_REV read failed"; return false; }

    s_rev_pn    = s_rev_raw[1];
    s_rev_fw[0] = s_rev_raw[2];
    s_rev_fw[1] = s_rev_raw[3];
    s_patch_id  = (uint16_t)((s_rev_raw[4] << 8) | s_rev_raw[5]);

    if (s_rev_pn == 0x00u || s_rev_pn == 0xFFu) {
        s_last_err = "no part number after power-up";
        si_log("[SI4732] powered up but reports no part number — raw "
               "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
               s_rev_raw[0], s_rev_raw[1], s_rev_raw[2], s_rev_raw[3],
               s_rev_raw[4], s_rev_raw[5], s_rev_raw[6], s_rev_raw[7],
               s_rev_raw[8]);
        si_log("[SI4732]   most likely no 25 MHz on pin 13: POWER_UP runs with "
               "XOSCEN=0, so without it the chip has no clock. Check 'rclk'.");
        return false;
    }

    si_log("[SI4732] part %u, fw %c.%c, chiprev 0x%02X, patch id 0x%04X",
           s_rev_pn, s_rev_fw[0] ? s_rev_fw[0] : '?',
           s_rev_fw[1] ? s_rev_fw[1] : '?', s_rev_raw[8], s_patch_id);

    if (expect_patch && s_patch_id != SI_PATCH_ID_FULL)
        si_log("[SI4732] WARNING: patch id is 0x%04X, expected 0x%04X — the "
               "upload may not have taken; SSB will not work", s_patch_id,
               SI_PATCH_ID_FULL);
    return true;
}

static bool si_power_up(bool with_patch)
{
    // ARG1: CTSIEN=0, GPO2OEN=0, PATCH as asked, XOSCEN=0 (external RCLK),
    // FUNC=1 (AM/SSB). ARG2: analog audio out.
    const uint8_t b[3] = {
        CMD_POWER_UP,
        (uint8_t)((with_patch ? PU_PATCH : 0u)
                  | (s_xoscen ? PU_XOSCEN : 0u)
                  | PU_FUNC_AM),
        PU_OPMODE_ANALOG
    };
    if (i2c_write_blocking(SI_I2C, s_addr, b, sizeof b, false) != (int)sizeof b) {
        s_last_err = "POWER_UP write failed";
        return false;
    }
    // The library waits 50 ms here before touching the chip again, and the
    // patch upload that follows is unforgiving about being rushed.
    sleep_ms(50);
    return si_wait_cts(500u);
}

// Waits for the seek/tune-complete flag. The tune command is acknowledged long
// before the tuner has settled, and until then every quality figure reads as
// zero — which is indistinguishable from "no signal" and sent us looking at the
// antenna for something that was a missing wait.
static bool si_wait_stc(uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    const uint8_t cmd[1] = { CMD_GET_INT_STATUS };
    for (;;) {
        if (si_cmd(cmd, 1)) {
            uint8_t st = 0u;
            if (i2c_read_blocking(SI_I2C, s_addr, &st, 1, false) == 1
                && st != 0xFFu && (st & 0x01u))
                return true;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) return false;
        sleep_ms(2);
    }
}

// Reads back what the tuner actually did, which is the one figure that tells a
// tune that silently went nowhere from one that worked. INTACK clears the STC
// flag so the next tune can be detected.
// Reads back what the tuner actually did. On ANY failure the read-back
// frequency is cleared: leaving the previous tune's value in place once had
// an AM result standing in for an SSB tune that never answered, and the
// wrong conclusion survived two rounds of measurements.
static bool si_read_tune_status(void)
{
    const uint8_t cmd[2] = { CMD_TUNE_STATUS, 0x01u };
    if (!si_cmd(cmd, sizeof cmd)) { s_read_khz = 0u; return false; }

    uint8_t r[8] = { 0 };
    if (i2c_read_blocking(SI_I2C, s_addr, r, sizeof r, false) != (int)sizeof r) {
        s_read_khz = 0u;        // never let the previous tune's answer stand in
        s_last_err = "TUNE_STATUS read failed";
        return false;
    }
    s_rsq_valid = (r[1] & 0x01u) != 0u;
    s_rsq_afcrl = (r[1] & 0x02u) != 0u;
    s_read_khz  = (uint16_t)((r[2] << 8) | r[3]);
    return true;
}

static bool si_tune(void)
{
    // No FAST bit. The data sheet says of it: "executes fast and invalidated
    // tune. The tune status will not be accurate." That is precisely the
    // symptom we were chasing — seek/tune-complete never arriving and the
    // read-back frequency coming out as zero. A tune that takes a few more
    // milliseconds and reports honestly is worth more here.
    //
    // How the result is checked then differs by mode, see below: AM has the
    // flag, SSB does not.
    uint8_t arg1 = 0u;
    if (s_mode == SI4732_MODE_LSB)      arg1 = TUNE_SB_LSB;
    else if (s_mode == SI4732_MODE_USB) arg1 = TUNE_SB_USB;

    uint16_t khz = (uint16_t)s_khz;
    const uint8_t b[6] = {
        CMD_TUNE_FREQ, arg1,
        (uint8_t)(khz >> 8), (uint8_t)khz,
        (uint8_t)(s_antcap >> 8), (uint8_t)s_antcap
    };
    if (!si_cmd(b, sizeof b)) return false;

    const char *err_before = s_last_err;
    const bool ssb = (s_mode != SI4732_MODE_AM);

    // In SSB the chip does not raise seek/tune-complete, and polling for it
    // does active harm. The reference library never waits for it there — it
    // sends the tune and delays — and now we know why: 150 GET_INT_STATUS
    // reads in 300 ms leave the part unable to answer CTS afterwards, so the
    // very next RSQ read times out. The tune itself was fine all along; the
    // read-back frequency was correct while the flag said it had failed.
    //
    // So: AM waits for the flag, SSB waits on the clock.
    if (ssb) {
        // SSB wants time, not questions. One read 40 ms after the command is
        // routinely too early — straight after a 2.9 s patch upload it comes
        // back as 0 kHz — and that is exactly what the old 300 ms of polling
        // was accidentally providing. So wait the same total, but ask eight
        // times instead of a hundred and fifty.
        s_stc_ok = false;
        for (int attempt = 0; attempt < 8 && !s_stc_ok; attempt++) {
            sleep_ms(40);
            if (si_read_tune_status() && s_read_khz == (uint16_t)s_khz)
                s_stc_ok = true;
        }
        if (s_stc_ok) {
            s_last_err = err_before;
        } else {
            s_last_err = "SSB tuner never reported the frequency back";
            si_log("[SI4732] WARNING: asked for %u kHz, tuner reports %u kHz "
                   "after 320 ms", (unsigned)s_khz, (unsigned)s_read_khz);
        }
        return true;
    }

    // Advisory only: a tune that does not announce itself may still have
    // worked, and failing the whole call here would hide the rest of the
    // diagnosis behind one flag.
    s_stc_ok = si_wait_stc(300u);
    (void)si_read_tune_status();
    if (s_stc_ok) {
        s_last_err = err_before;
    } else {
        // Name the overwhelmingly likely cause. The chip has no crystal, so
        // with SI5351 CLK2 off it still answers every command and still
        // acknowledges the tune — it simply never completes one, and then
        // every level reads 0. That is identical to a dead antenna from the
        // outside, and reporting "none" here once cost a whole bench session
        // and the wrong conclusion that the part was deaf.
        s_last_err = "tune never completed — no reference clock on pin 13? ('rclk')";
        si_log("[SI4732] note: no seek/tune-complete within 300 ms");
        si_log("[SI4732]       this chip has no crystal: check 'rclk' is 1");
    }

    if (s_stc_ok && s_read_khz != (uint16_t)s_khz)
        si_log("[SI4732] WARNING: asked for %u kHz, tuner reports %u kHz",
               (unsigned)s_khz, (unsigned)s_read_khz);
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

    // No GET_REV here. In the powered-down state the chip answers it with a
    // valid status byte and an empty payload — observed as
    // "80 00 00 00 00 00 00 00 10" — because its internal controller is not
    // running yet. Treating that as a fault, as an earlier version did, declares
    // a perfectly good chip dead. The real check happens after POWER_UP, in
    // si_read_rev(), where the part number and the patch id mean something.
    s_present  = true;
    s_last_err = "none";
    si_log("[SI4732] 0x%02X answers; revision read deferred until power-up. "
           "SSB patch %s", s_addr,
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

    // Now the revision read means something: the chip is running, and after a
    // patch upload the patch id proves it landed.
    if (!si_read_rev(want_patch)) { s_on = false; return false; }

    s_mode = mode;
    s_on   = true;

    // Reference clock first: everything downstream derives its frequency from
    // it, so setting it after tuning would leave the first tune offset.
    bool ok = si_set_prop(PROP_REFCLK_PRESCALE, s_rclk_presc);
    ok = si_set_prop(PROP_REFCLK_FREQ, (uint16_t)(s_rclk_hz / s_rclk_presc)) && ok;

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

const char *si4732_last_error(void) { return s_last_err ? s_last_err : "none"; }

// Tunes and averages several quality readings.
//
// A single RSQ read is not usable for searching: on the bench the same
// frequency gave 64 and then 0 in consecutive reads seconds apart, because with
// no carrier the AGC is chasing noise. Averaging a handful of reads, and
// reporting the peak alongside, turns that into something an eye can compare
// between frequencies.
bool si4732_probe_khz(uint32_t khz, uint8_t samples,
                      uint8_t *rssi_avg, uint8_t *rssi_max, uint8_t *snr_max)
{
    if (rssi_avg) *rssi_avg = 0u;
    if (rssi_max) *rssi_max = 0u;
    if (snr_max)  *snr_max  = 0u;
    if (!s_present || !s_on) return false;
    if (samples == 0u) samples = 1u;

    if (!si4732_tune_hz(khz * 1000u)) return false;
    sleep_ms(40);                      // let the AGC settle after the tune

    uint32_t sum = 0u;
    uint8_t  n = 0u, rmax = 0u, smax = 0u;
    for (uint8_t i = 0; i < samples; i++) {
        uint8_t r = 0u, sn = 0u;
        if (si4732_read_rsq(&r, &sn)) {
            sum += r; n++;
            if (r  > rmax) rmax = r;
            if (sn > smax) smax = sn;
        }
        sleep_ms(10);
    }
    if (n == 0u) return false;

    if (rssi_avg) *rssi_avg = (uint8_t)(sum / n);
    if (rssi_max) *rssi_max = rmax;
    if (snr_max)  *snr_max  = smax;
    return true;
}

bool si4732_get_xoscen(void) { return s_xoscen; }

uint16_t si4732_get_antcap(void) { return s_antcap; }

bool si4732_set_antcap(uint16_t cap)
{
    if (cap > 6143u) return false;
    s_antcap = cap;
    if (!s_present || !s_on) return true;
    return si_tune();              // ANTCAP travels with the tune command
}

// Takes effect on the next power-up, so re-select the mode afterwards.
void si4732_set_xoscen(bool on) { s_xoscen = on; }

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
    if (!s_present) { s_last_err = "no Si4732 on the bus";   return false; }
    if (!s_on)      { s_last_err = "receiver is powered down"; return false; }

    const uint8_t b[2] = { CMD_RSQ_STATUS, 0x00u };
    if (!si_cmd(b, sizeof b)) return false;

    // Eight bytes, not six: RESP2 carries VALID and AFCRL, and RESP7 a signed
    // frequency offset in kHz. Those three say whether the chip thinks it is on
    // a real channel and how far off it is — worth far more while hunting for a
    // beacon than the two level figures alone.
    uint8_t r[8] = { 0 };
    if (i2c_read_blocking(SI_I2C, s_addr, r, sizeof r, false) != (int)sizeof r) {
        s_last_err = "RSQ read failed";
        return false;
    }
    s_rsq_valid = (r[2] & 0x01u) != 0u;
    s_rsq_afcrl = (r[2] & 0x02u) != 0u;
    s_rsq_smute = (r[2] & 0x08u) != 0u;
    s_rsq_off   = (int8_t)r[7];
    if (rssi_dbuv) *rssi_dbuv = r[4];
    if (snr_db)    *snr_db    = r[5];
    return true;
}

// ---------------------------------------------------------------------------
// Front-end AGC
//
// The index is an attenuation, not a gain: 0 means the input stage is wide
// open. The AM AGC watches the whole input, not the channel you tuned, so one
// strong carrier anywhere in band turns the front end down for everything —
// which is exactly the failure mode to rule out before blaming the antenna.
// Reading it costs one command and settles the question.
// ---------------------------------------------------------------------------
bool si4732_read_agc(bool *disabled, uint8_t *index)
{
    if (!s_present || !s_on) { s_last_err = "receiver is not powered up"; return false; }

    const uint8_t b[1] = { CMD_AGC_STATUS };
    if (!si_cmd(b, sizeof b)) return false;

    uint8_t r[3] = { 0 };
    if (i2c_read_blocking(SI_I2C, s_addr, r, sizeof r, false) != (int)sizeof r) {
        s_last_err = "AGC status read failed";
        return false;
    }
    if (disabled) *disabled = (r[1] & 0x01u) != 0u;
    if (index)    *index    = r[2];
    return true;
}

bool si4732_set_agc(bool disable, uint8_t index)
{
    if (!s_present || !s_on) { s_last_err = "receiver is not powered up"; return false; }
    const uint8_t b[3] = { CMD_AGC_OVERRIDE, (uint8_t)(disable ? 0x01u : 0x00u), index };
    return si_cmd(b, sizeof b);
}

// ---------------------------------------------------------------------------
// Reference clock
// ---------------------------------------------------------------------------
uint32_t si4732_rclk_hz(void)       { return s_rclk_hz; }
uint16_t si4732_rclk_prescale(void) { return s_rclk_presc; }

bool si4732_set_rclk_hz(uint32_t hz)
{
    uint16_t presc = si_presc_for(hz);
    if (presc == 0u) {
        s_last_err = "no exact prescaler puts that clock in the 31130..34406 Hz window";
        return false;
    }
    s_rclk_hz    = hz;
    s_rclk_presc = presc;

    if (!s_present || !s_on) return true;   // adopted; applied at the next power-up

    // Order matters: the prescaler divides the pin, REFCLK_FREQ tells the chip
    // what came out. Setting the frequency first would briefly describe a
    // reference that is not there. Then retune, because every frequency the
    // tuner holds was derived from the old reference.
    bool ok = si_set_prop(PROP_REFCLK_PRESCALE, s_rclk_presc);
    ok = si_set_prop(PROP_REFCLK_FREQ, (uint16_t)(s_rclk_hz / s_rclk_presc)) && ok;
    if (!ok) return false;
    return si4732_tune_hz(si4732_get_hz());
}

// ---------------------------------------------------------------------------
// Step-by-step SSB bring-up, to find which command stops the answers
// ---------------------------------------------------------------------------
// After an SSB tune this chip stops answering CTS, while AM on the same
// hardware is fine and a plain power-up brings it straight back without a
// reset. That rules out a crash and makes it a state that something in this
// one sequence puts it into. So walk the sequence and read the status byte
// after every single command.
//
// The status byte is worth more than CTS alone: bit 6 is ERR, which the part
// sets when it rejects a command or a property. A property that is valid in
// AM and not in SSB would show up there and nowhere else.
static bool si_cts_now(const char *what)
{
    uint8_t st = 0xFFu;
    const bool got = (i2c_read_blocking(SI_I2C, s_addr, &st, 1, false) == 1);
    const char *verdict;
    if (!got)              verdict = "NO ACK - chip is not on the bus";
    else if (st == 0xFFu)  verdict = "0xFF - bus idle, nothing answered";
    else if (!(st & 0x80u))verdict = "busy, CTS still low";
    else if (st & 0x40u)   verdict = "CTS, but ERR SET - command refused";
    else                   verdict = "ok";
    si_log("[SSBTEST] %-20s status=0x%02X  %s", what, st, verdict);
    return got && st != 0xFFu && (st & 0x80u) && !(st & 0x40u);
}

void si4732_ssb_bringup_test(void)
{
    if (!s_present) { si_log("[SSBTEST] no Si4732 on the bus"); return; }
#ifndef SI4732_SSB_PATCH
    si_log("[SSBTEST] no SSB patch compiled in - run tools/fetch_ssb_patch.sh");
#else
    si_log("[SSBTEST] --- step-by-step SSB bring-up, %lu kHz ---",
           (unsigned long)s_khz);

    si_hw_reset();
    si_log("[SSBTEST] probe after reset: %d", (int)si_probe(s_addr));

    si_log("[SSBTEST] POWER_UP(patch): %d", (int)si_power_up(true));
    si_cts_now("after POWER_UP");

    si_log("[SSBTEST] patch upload: %d", (int)si_download_patch());
    si_cts_now("after patch");

    s_on   = true;
    s_mode = SI4732_MODE_USB;

    si_log("[SSBTEST] GET_REV: %d", (int)si_read_rev(true));
    si_cts_now("after GET_REV");

    si_log("[SSBTEST] REFCLK_PRESCALE: %d",
           (int)si_set_prop(PROP_REFCLK_PRESCALE, s_rclk_presc));
    si_cts_now("after PRESCALE");

    si_log("[SSBTEST] REFCLK_FREQ: %d",
           (int)si_set_prop(PROP_REFCLK_FREQ, (uint16_t)(s_rclk_hz / s_rclk_presc)));
    si_cts_now("after REFCLK_FREQ");

    si_log("[SSBTEST] AM_SOFT_MUTE_MAX: %d",
           (int)si_set_prop(PROP_AM_SOFT_MUTE_MAX_AT, 0u));
    si_cts_now("after SOFT_MUTE");

    si_log("[SSBTEST] SSB_MODE+bandwidth: %d", (int)si_apply_bandwidth());
    si_cts_now("after SSB_MODE");

    si_log("[SSBTEST] RX_VOLUME: %d", (int)si_set_prop(PROP_RX_VOLUME, s_volume));
    si_cts_now("after VOLUME");

    si_log("[SSBTEST] SSB_BFO: %d", (int)si_set_prop(PROP_SSB_BFO, 0u));
    si_cts_now("after BFO");

    // The tune itself, by hand, so a CTS timeout inside si_cmd() cannot hide
    // what the chip did with it.
    {
        const uint16_t khz = (uint16_t)s_khz;
        const uint8_t tf[6] = { CMD_TUNE_FREQ, TUNE_SB_USB,
                                (uint8_t)(khz >> 8), (uint8_t)khz, 0u, 0u };
        const int w = i2c_write_blocking(SI_I2C, s_addr, tf, sizeof tf, false);
        si_log("[SSBTEST] SSB_TUNE_FREQ written: %d of %u bytes",
               w, (unsigned)sizeof tf);
    }
    sleep_ms(10);  si_cts_now("10 ms after tune");
    sleep_ms(90);  si_cts_now("100 ms after tune");
    sleep_ms(400); si_cts_now("500 ms after tune");

    si_log("[SSBTEST] TUNE_STATUS: %d  -> %u kHz",
           (int)si_read_tune_status(), (unsigned)s_read_khz);
    si_cts_now("after TUNE_STATUS");

    uint8_t rssi = 0u, snr = 0u;
    si_log("[SSBTEST] RSQ: %d  rssi=%u snr=%u",
           (int)si4732_read_rsq(&rssi, &snr), rssi, snr);

    si_log("[SSBTEST] --- the first line that is not ok is the culprit ---");
#endif
}

void si4732_diag_print(void)
{
    si_log("[SI4732] present=%d addr=0x%02X on=%d patch=%d",
           (int)s_present, s_addr, (int)s_on, (int)si4732_has_ssb());
    if (!s_present) {
        si_log("[SI4732]   last error: %s", s_last_err);
        si_log("[SI4732]   note: before power-up the chip answers GET_REV with a");
        si_log("[SI4732]         valid status and an empty payload. That is normal.");
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
    // Before power-up there is nothing to report: the revision read comes back
    // empty by design, and printing it next to "READ FAILED" and a part number
    // of zero reads as a broken chip. It has misled us twice.
    if (!s_on) {
        si_log("[SI4732]   not powered up yet — run 'rx on' (or 'rx am' / 'rx usb')");
        si_log("[SI4732]   xoscen=%d antcap=%u  rclk=%lu Hz / %u = refclk %lu Hz",
               (int)s_xoscen, s_antcap, (unsigned long)s_rclk_hz,
               s_rclk_presc, (unsigned long)(s_rclk_hz / s_rclk_presc));
        si_log("[SI4732]   stored settings: mode=%s freq=%lu kHz bw=%u Hz vol=%u",
               si4732_mode_name(s_mode), (unsigned long)s_khz, s_bw_hz, s_volume);
        return;
    }

    si_log("[SI4732]   part=%u fw=%c.%c chiprev=0x%02X patch_id=0x%04X%s",
           s_rev_pn, s_rev_fw[0] ? s_rev_fw[0] : '?',
           s_rev_fw[1] ? s_rev_fw[1] : '?', s_rev_raw[8], s_patch_id,
           (s_patch_id == SI_PATCH_ID_FULL) ? " (SSB patch confirmed)" : "");
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
    si_log("[SI4732]   xoscen=%d (0 = external clock on RCLK)  antcap=%u%s",
           (int)s_xoscen, s_antcap,
           s_antcap ? "" : " (chip tunes its own varactor)");
    si_log("[SI4732]   rclk=%lu Hz / %u = refclk %lu Hz",
           (unsigned long)s_rclk_hz, s_rclk_presc,
           (unsigned long)(s_rclk_hz / s_rclk_presc));

    bool agc_off = false;
    uint8_t agc_idx = 0u;
    if (si4732_read_agc(&agc_off, &agc_idx)) {
        si_log("[SI4732]   agc %s, attenuation index %u%s",
               agc_off ? "DISABLED (forced)" : "automatic", agc_idx,
               (!agc_off && agc_idx > 8u)
                   ? "  <-- front end is being turned down by something strong"
                   : "");
    }

    uint8_t rssi = 0u, snr = 0u;
    if (si4732_read_rsq(&rssi, &snr)) {
        si_log("[SI4732]   rssi=%u dBuV snr=%u dB  valid=%d afc_railed=%d "
               "softmute=%d offset=%d kHz", rssi, snr, (int)s_rsq_valid,
               (int)s_rsq_afcrl, (int)s_rsq_smute, (int)s_rsq_off);
    } else {
        si_log("[SI4732]   rsq unavailable (%s)", s_last_err);
    }
    // "tune complete" is the chip's seek/tune flag, which only AM raises. In
    // SSB the same column reports whether the read-back frequency matches
    // what was asked for, which is the honest equivalent.
    si_log("[SI4732]   tuner reports %u kHz, tune %s  (asked for %lu kHz)",
           (unsigned)s_read_khz,
           s_stc_ok ? "ok" : "DID NOT COMPLETE",
           (unsigned long)s_khz);
    if (!s_stc_ok && s_read_khz == 0u)
        si_log("[SI4732]   0 kHz here means the chip did not answer, not that it "
               "tuned to zero");
    si_log("[SI4732]   last error: %s", s_last_err);
}
