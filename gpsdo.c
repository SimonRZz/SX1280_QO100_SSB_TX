// gpsdo.c — GPSDO integration for SX1280_QO100_SSB_TX (Pico SDK C)
//
// Ported from gpsdo_rp2350.ino (Arduino / CT2GQV).
// All Arduino-specific APIs replaced with Pico SDK equivalents.
//
// Hardware:
//   NEO-7M GPS  → UART1   GP4 = TX (→ GPS RX), GP5 = RX (← GPS TX)
//   SI5351      → I2C0    GP0 = SDA, GP1 = SCL
//   TCXO_EN (GP22) and SX1280 NRESET (GP20) are managed by main.c.
//
// NOTE: GP0/GP1 are used here for I2C.  CMakeLists.txt must therefore
//       set pico_enable_stdio_uart to 0 (UART0 defaults to GP0/GP1).

#include "gpsdo.h"

#include <string.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
#define GPSDO_I2C           i2c0
#define GPSDO_I2C_SDA       0u
#define GPSDO_I2C_SCL       1u
#define GPSDO_I2C_BAUD      100000u   // 100 kHz — reliable with internal pull-ups

// UART1 pin assignment.
// If uart_rx stays 0, try swapping these two lines (swap GP4 and GP5).
#define GPSDO_UART          uart1
#define GPSDO_UART_TX       4u        // Pico TX (output) → NEO-7M RX
#define GPSDO_UART_RX       5u        // Pico RX (input)  ← NEO-7M TX
#define GPSDO_GPS_BAUD      9600u

#define GPSDO_PRINT_MS      10000u    // status line interval (10 s — keeps RF quiet)
#define GPSDO_NMEA_STALE_MS 3000u
#define GPSDO_GGA_STALE_MS  5000u
#define GPSDO_GSV_FRESH_MS  10000u
#define GPSDO_SI5351_POLL_MS 30000u   // SI5351 reg-0 re-read interval once locked
#define GPSDO_SI5351_FAST_POLL_MS 500u // ... while still waiting for PLL lock at boot
#define GPSDO_LOST_MS       10000u    // no fix for this long after having had one = signal lost
#define GPSDO_NMEA_BUF      128u
#define GPSDO_BOOT_DELAY_MS 300u      // wait before sending UBX commands

// ---------------------------------------------------------------------------
// SI5351 minimal driver (I2C0)
// Two outputs, both from the 24 MHz GPS TIMEPULSE reference:
//
//   CLK1  52 MHz  → SX1280 XTA        PLL A: 24 MHz × (36 + 5/6) = 884 MHz
//                                     MS1:   884 ÷ 17 = 52 MHz
//   CLK2 144 MHz  → receive mixer LO  PLL B: 24 MHz × 36        = 864 MHz
//                 (off at boot, enable with "lo 1")
//                                     MS2:   864 ÷ 6  = 144 MHz
//
// CLK0 is left powered down. CLK2 carries the mixer LO because that is where
// the filter is wired; electrically either output would do, both sit next to
// CLK1 on the package and either can be driven from PLL B.
//
// The mixer does not run on 144 MHz itself: its local oscillator is the 5th
// harmonic at 720 MHz, picked out by a high-pass and lifted by an SPF5189Z at
// the receiver. That is why CLK2 runs at 8 mA — the harmonic lives on the edge
// steepness — and why the divider is a plain even integer. Multiplying by 5
// also multiplies the phase noise by 20·log10(5) = 14 dB, so there is no room
// for a fractional divider here. 144 MHz is the only frequency near the target
// that PLL B reaches with an integer divider (720 ÷ 144 = 5 is not a legal
// MultiSynth value, 864 ÷ 6 is).
//
// PLL A stays fractional, as it always was: 52 MHz needs a 624 MHz VCO for an
// integer path and there are only two PLLs. It has been on the air like this.
//
// Register values:
//   PLL A: p1=4202 (0x106A), p2=4, p3=6
//   MS1:   p1=1664 (0x0680), p2=0, p3=1
//   PLL B: p1=4096 (0x1000), p2=0, p3=1
//   MS2:   p1=256  (0x0100), p2=0, p3=1
// ---------------------------------------------------------------------------
// SI5351 I2C address — auto-detected in gpsdo_init().
// Common values: 0x60 (SDO/AD0=GND), 0x61 (SDO/AD0=VCC).
// Some boards use 0x62 or 0x63; all four are tried.
// 0x00 means "not found yet".
static uint8_t s_si_addr = 0x00u;

// Whether CLK2 (144 MHz, the receive mixer LO) is currently enabled.
static bool s_loOn = false;

#define SI_REG_STATUS      0u    // Device status (read-only)
#define SI_STATUS_SYS_INIT (1u << 7) // 1 = calibrating (not ready)
#define SI_STATUS_LOL_A    (1u << 5) // 1 = PLL A loss-of-lock
#define SI_STATUS_LOS_XTAL (1u << 3) // 1 = no signal on XA pin
#define SI_REG_OEB         3u    // Output Enable (active LOW per bit)
#define SI_REG_CLK0_CTRL   16u
#define SI_REG_CLK1_CTRL   17u
#define SI_REG_CLK2_CTRL   18u
#define SI_REG_PLLA_BASE   26u   // 8 bytes: regs 26-33
#define SI_REG_PLLB_BASE   34u   // 8 bytes: regs 34-41
#define SI_REG_MS0_BASE    42u   // 8 bytes: regs 42-49  (CLK0, unused here)
#define SI_REG_MS1_BASE    50u   // 8 bytes: regs 50-57
#define SI_REG_MS2_BASE    58u   // 8 bytes: regs 58-65
#define SI_REG_PLL_RESET   177u
#define SI_REG_XTAL_LOAD   183u

// Output enable is active LOW, one bit per clock: bit 0 = CLK0, 1 = CLK1,
// 2 = CLK2. CLK1 must never be disabled — it is the SX1280's only clock.
#define SI_OEB_ALL_OFF     0xFFu
#define SI_OEB_CLK1        0xFDu  // CLK1 on, CLK0 and CLK2 off
#define SI_OEB_CLK1_CLK2   0xF9u  // CLK1 and CLK2 on, CLK0 off

// Scan 0x60–0x63 (all valid SI5351 I2C addresses).
// Retries up to 3 times with a short delay between attempts.
// Sets s_si_addr on success; leaves it 0x00 on failure.
static bool si5351_scan(void)
{
    static const uint8_t candidates[] = { 0x60u, 0x61u, 0x62u, 0x63u };
    for (int attempt = 0; attempt < 3; attempt++) {
        for (size_t i = 0; i < sizeof(candidates); i++) {
            uint8_t dummy;
            int ret = i2c_read_blocking(GPSDO_I2C, candidates[i], &dummy, 1, false);
            if (ret == 1) {
                s_si_addr = candidates[i];
                return true;
            }
        }
        sleep_ms(20);  // brief pause before retry
    }
    s_si_addr = 0x00u;  // explicit "not found"
    return false;
}

static bool si_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(GPSDO_I2C, s_si_addr, buf, 2, false) == 2;
}

static bool si_write_regs(uint8_t base, const uint8_t *data, size_t len)
{
    // max needed: 1 address byte + 8 data bytes = 9
    uint8_t buf[9];
    if (len == 0u || len > 8u) return false;
    buf[0] = base;
    memcpy(buf + 1, data, len);
    return i2c_write_blocking(GPSDO_I2C, s_si_addr, buf, len + 1u, false)
           == (int)(len + 1u);
}

static bool si_read_reg(uint8_t reg, uint8_t *val)
{
    if (i2c_write_blocking(GPSDO_I2C, s_si_addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(GPSDO_I2C, s_si_addr, val, 1, false) == 1;
}

static bool si5351_init_clocks(void)
{
    // Scan for SI5351 at 0x60–0x63 before writing any registers.
    if (!si5351_scan()) return false;

    // Disable all outputs while configuring.
    if (!si_write_reg(SI_REG_OEB, SI_OEB_ALL_OFF)) return false;

    // Power down unused CLK0.
    si_write_reg(SI_REG_CLK0_CTRL, 0x80u);

    // CLK1: PDN=0, INT=1 (integer mode), MS_SRC=PLLA, INV=0,
    //        CLK_SRC=MS1(self)=0b11, IDRV=4mA=0b01
    //   Byte: 0b 0 1 0 0 11 01 = 0x4D
    si_write_reg(SI_REG_CLK1_CTRL, 0x4Du);

    // CLK2: PDN=0, INT=1, MS_SRC=PLLB, INV=0, CLK_SRC=MS2=0b11, IDRV=8mA=0b11
    //   Byte: 0b 0 1 1 0 11 11 = 0x6F
    // 8 mA is deliberate: the wanted signal is the 5th harmonic, which only
    // exists because the edges are steep.
    si_write_reg(SI_REG_CLK2_CTRL, 0x6Fu);

    // Crystal load capacitance: 8 pF (bits[7:6]=0b10) + reserved 0x12.
    si_write_reg(SI_REG_XTAL_LOAD, 0x92u);

    // PLL A: p1=4202=0x106A, p2=4, p3=6
    //   Regs 26-33: [p3_hi, p3_lo, p1[17:16], p1_hi, p1_lo,
    //                p3[19:16]|p2[19:16], p2_hi, p2_lo]
    static const uint8_t plla[8] = {
        0x00u, 0x06u,   // p3=6
        0x00u,          // p1[17:16]=0
        0x10u, 0x6Au,   // p1[15:8]=0x10, p1[7:0]=0x6A  (4202)
        0x00u,          // upper nibbles of p3/p2 = 0
        0x00u, 0x04u    // p2=4
    };
    if (!si_write_regs(SI_REG_PLLA_BASE, plla, 8u)) return false;

    // MS1 (CLK1): p1=1664=0x0680, p2=0, p3=1
    //   Regs 50-57
    static const uint8_t ms1[8] = {
        0x00u, 0x01u,   // p3=1
        0x00u,          // R_DIV=/1, DIVBY4=0, p1[17:16]=0
        0x06u, 0x80u,   // p1[15:8]=0x06, p1[7:0]=0x80  (1664)
        0x00u,          // upper nibbles = 0
        0x00u, 0x00u    // p2=0
    };
    if (!si_write_regs(SI_REG_MS1_BASE, ms1, 8u)) return false;

    // PLL B: p1=4096=0x1000, p2=0, p3=1  →  24 MHz × 36 = 864 MHz
    static const uint8_t pllb[8] = {
        0x00u, 0x01u,   // p3=1
        0x00u,          // p1[17:16]=0
        0x10u, 0x00u,   // p1[15:8]=0x10, p1[7:0]=0x00  (4096)
        0x00u,          // upper nibbles of p3/p2 = 0
        0x00u, 0x00u    // p2=0
    };
    if (!si_write_regs(SI_REG_PLLB_BASE, pllb, 8u)) return false;

    // MS2 (CLK2): p1=256=0x0100, p2=0, p3=1  →  864 ÷ 6 = 144 MHz
    static const uint8_t ms2[8] = {
        0x00u, 0x01u,   // p3=1
        0x00u,          // R_DIV=/1, DIVBY4=0, p1[17:16]=0
        0x01u, 0x00u,   // p1[15:8]=0x01, p1[7:0]=0x00  (256)
        0x00u,          // upper nibbles = 0
        0x00u, 0x00u    // p2=0
    };
    if (!si_write_regs(SI_REG_MS2_BASE, ms2, 8u)) return false;

    // Reset both PLLs to lock them to the new multisynth settings. Safe here
    // because nothing depends on either clock yet — the SX1280 is still held in
    // reset. At runtime PLL A must never be reset: the SX1280 would lose its
    // clock for milliseconds. Reset B alone with 0x80 if that is ever needed.
    si_write_reg(SI_REG_PLL_RESET, 0xA0u);

    // CLK1 only. The mixer LO stays OFF until asked for with "lo 1".
    //
    // It used to come up enabled, which was a mistake: gpsdo_init() runs before
    // the boot gate that waits for a locked 52 MHz, so anything wrong on the
    // CLK2 line — stray DC from an amplifier module, a supply droop from the
    // extra 8 mA, 144 MHz coupling into the 24 MHz reference on XA — could stop
    // the transmitter from starting at all. The receive side must never be able
    // to do that. MS2 and CLK2_CTRL are configured above, so "lo 1" only has to
    // flip the output-enable bit; PLL B is already locked by then.
    si_write_reg(SI_REG_OEB, SI_OEB_CLK1);
    s_loOn = false;

    return true;
}

// Switch the 144 MHz mixer LO without touching CLK1. Useful for measuring the
// LO branch against a quiet baseline, and for silencing it during transmit.
bool gpsdo_set_lo_enabled(bool on)
{
    if (s_si_addr == 0x00u) return false;
    if (!si_write_reg(SI_REG_OEB, on ? SI_OEB_CLK1_CLK2 : SI_OEB_CLK1))
        return false;
    s_loOn = on;
    return true;
}

bool gpsdo_lo_enabled(void) { return s_loOn; }

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
static int      s_fixQuality      = 0;
static int      s_satsUsed        = 0;
static char     s_utc[7]          = "------";
static char     s_date[7]         = "------"; // ddmmyy from RMC
static bool     s_clk1Ok          = false;   // I2C init succeeded
static uint8_t  s_si5351_status   = 0xFFu;   // SI5351 reg 0; 0xFF = not yet read
static uint32_t s_si5351_pollMs   = 0u;
static bool     s_gpsdoReady      = false;

// Position & locator (updated from GGA when fixQuality > 0)
static float    s_lat             = 0.0f;    // decimal degrees, positive = N
static float    s_lon             = 0.0f;    // decimal degrees, positive = E
static int16_t  s_alt_m           = 0;       // altitude in metres
static bool     s_has_position    = false;
static char     s_locator[7]      = "------"; // 6-char Maidenhead

// Returns the clk1= status string based on SI5351 register 0.
// "ok"  – PLL A locked, XA signal present
// "lol" – PLL A loss of lock (XA seen but PLL not locked)
// "los" – loss of signal on XA (GPS 24 MHz not reaching SI5351)
// "fail"– I2C init failed (SI5351 not found or bus error)
static const char *si5351_clk_status(void)
{
    if (!s_clk1Ok)                              return "fail";
    if (s_si5351_status == 0xFFu)               return "ok";   // not yet polled
    if (s_si5351_status & SI_STATUS_LOS_XTAL)   return "los";
    if (s_si5351_status & SI_STATUS_LOL_A)      return "lol";
    return "ok";
}

static char     s_nmeaBuf[GPSDO_NMEA_BUF];
static size_t   s_nmeaIdx         = 0;
static bool     s_collecting      = false;
static bool     s_nmeaOverflow    = false;

static uint32_t s_lastNmeaMs      = 0;
static uint32_t s_lastTimeMs      = 0;
static uint32_t s_lastGgaMs       = 0;
static uint32_t s_lastGoodMs      = 0;       // last moment with SI5351 lock AND a position fix
static uint32_t s_lastPrintMs     = 0;
static uint32_t s_uartBytesRx     = 0;   // raw bytes received from GPS UART
static uint32_t s_nmeaCount       = 0;   // valid NMEA sentences parsed
static uint32_t s_detected_baud   = 0;   // baud rate confirmed during auto-detect

// GSV buckets: GP=0, GL=1, GA=2, GB/BD=3, GQ=4, GI=5, GN=6
#define GSV_BUCKETS 7u
static uint8_t  s_gsvVisible[GSV_BUCKETS];
static uint32_t s_gsvStampMs[GSV_BUCKETS];

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
static inline uint32_t gpsdo_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline bool is_digit6(const char *p)
{
    if (!p) return false;
    for (uint8_t i = 0u; i < 6u; i++) {
        if (p[i] < '0' || p[i] > '9') return false;
    }
    return true;
}

static uint16_t parse_uint(const char *p)
{
    if (!p) return 0u;
    uint16_t v = 0u; bool any = false;
    while (*p >= '0' && *p <= '9') {
        any = true;
        v = (uint16_t)(v * 10u + (uint16_t)(*p - '0'));
        ++p;
    }
    return any ? v : 0u;
}

static int8_t hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

static const char *nmea_field(const char *s, uint8_t n)
{
    uint8_t cnt = 0u;
    while (*s) {
        if (cnt == n) return s;
        if (*s == '*') break;
        if (*s == ',') cnt++;
        s++;
    }
    return (cnt == n) ? s : NULL;
}

static bool nmea_checksum_ok(const char *s)
{
    if (!s || s[0] != '$') return false;
    uint8_t cs = 0u;
    const char *p = s + 1;
    while (*p && *p != '*') { cs ^= (uint8_t)(*p); ++p; }
    if (*p != '*' || !p[1] || !p[2]) return false;
    int8_t hi = hex_nibble(p[1]);
    int8_t lo = hex_nibble(p[2]);
    if (hi < 0 || lo < 0) return false;
    return cs == (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
}

static int8_t talker_to_gsv_bucket(char t1, char t2)
{
    if (t1 == 'G' && t2 == 'P') return 0;
    if (t1 == 'G' && t2 == 'L') return 1;
    if (t1 == 'G' && t2 == 'A') return 2;
    if ((t1 == 'G' && t2 == 'B') || (t1 == 'B' && t2 == 'D')) return 3;
    if (t1 == 'G' && t2 == 'Q') return 4;
    if (t1 == 'G' && t2 == 'I') return 5;
    if (t1 == 'G' && t2 == 'N') return 6;
    return -1;
}

static uint16_t get_visible_sats(void)
{
    const uint32_t now = gpsdo_ms();
    // Prefer combined GN bucket if fresh.
    if (s_gsvStampMs[6] != 0u &&
        (now - s_gsvStampMs[6]) <= GPSDO_GSV_FRESH_MS) {
        return s_gsvVisible[6];
    }
    uint16_t sum = 0u;
    for (uint8_t i = 0u; i < 6u; i++) {
        if (s_gsvStampMs[i] != 0u &&
            (now - s_gsvStampMs[i]) <= GPSDO_GSV_FRESH_MS) {
            sum += s_gsvVisible[i];
        }
    }
    return sum;
}

// ---------------------------------------------------------------------------
// Coordinate helpers
// ---------------------------------------------------------------------------

// Parse "DDMM.MMMM" (lat, deg_digits=2) or "DDDMM.MMMM" (lon, deg_digits=3)
// into decimal degrees. Returns 0 on malformed input. Apply N/S/E/W sign outside.
static float parse_nmea_coord(const char *p, int deg_digits)
{
    if (!p || *p < '0' || *p > '9') return 0.0f;
    int32_t deg = 0;
    for (int i = 0; i < deg_digits; i++) {
        if (p[i] < '0' || p[i] > '9') return 0.0f;
        deg = deg * 10 + (p[i] - '0');
    }
    const char *mp = p + deg_digits;
    float min = 0.0f, frac = 0.1f;
    bool dot = false;
    while (*mp && *mp != ',' && *mp != '*') {
        if (*mp == '.') { dot = true; mp++; continue; }
        if (*mp < '0' || *mp > '9') break;
        if (!dot) { min = min * 10.0f + (float)(*mp - '0'); }
        else      { min += (float)(*mp - '0') * frac; frac *= 0.1f; }
        mp++;
    }
    return (float)deg + min / 60.0f;
}

// Parse altitude field (metres, may be fractional — truncate to integer).
static int16_t parse_alt(const char *p)
{
    if (!p || (*p != '-' && (*p < '0' || *p > '9'))) return 0;
    bool neg = (*p == '-'); if (neg) p++;
    int32_t v = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (*p - '0'); p++; }
    if (v > 32767) v = 32767;
    return (int16_t)(neg ? -v : v);
}

// Compute 6-character Maidenhead locator from decimal degrees.
static void compute_maidenhead(float lat, float lon, char *loc6)
{
    lon += 180.0f;  lat += 90.0f;           // shift to positive range
    int fl = (int)(lon / 20.0f);  if (fl > 17) fl = 17;
    int fa = (int)(lat / 10.0f);  if (fa > 17) fa = 17;
    lon -= fl * 20.0f;  lat -= fa * 10.0f;
    int sl = (int)(lon / 2.0f);   if (sl > 9) sl = 9;
    int sa = (int)(lat);          if (sa > 9) sa = 9;
    lon -= sl * 2.0f;  lat -= (float)sa;
    int ssl = (int)(lon * 12.0f); if (ssl > 23) ssl = 23;
    int ssa = (int)(lat * 24.0f); if (ssa > 23) ssa = 23;
    loc6[0] = (char)('A' + fl);
    loc6[1] = (char)('A' + fa);
    loc6[2] = (char)('0' + sl);
    loc6[3] = (char)('0' + sa);
    loc6[4] = (char)('a' + ssl);
    loc6[5] = (char)('a' + ssa);
    loc6[6] = '\0';
}

// ---------------------------------------------------------------------------
// NMEA parsers
// ---------------------------------------------------------------------------
static void parse_time(const char *s, uint8_t fn)
{
    const char *f = nmea_field(s, fn);
    if (f && is_digit6(f)) {
        for (uint8_t i = 0u; i < 6u; i++) s_utc[i] = f[i];
        s_utc[6] = '\0';
        s_lastTimeMs = gpsdo_ms();
    }
}

static void parse_gga(const char *s)
{
    s_lastGgaMs  = gpsdo_ms();
    parse_time(s, 1u);
    s_fixQuality = (int)parse_uint(nmea_field(s, 6u));
    s_satsUsed   = (int)parse_uint(nmea_field(s, 7u));
    if (s_fixQuality > 0) {
        const char *lat_f = nmea_field(s, 2u);
        const char *ns_f  = nmea_field(s, 3u);
        const char *lon_f = nmea_field(s, 4u);
        const char *ew_f  = nmea_field(s, 5u);
        const char *alt_f = nmea_field(s, 9u);
        if (lat_f && ns_f && lon_f && ew_f &&
            lat_f[0] >= '0' && lat_f[0] <= '9') {
            float lat = parse_nmea_coord(lat_f, 2);
            float lon = parse_nmea_coord(lon_f, 3);
            if (ns_f[0] == 'S') lat = -lat;
            if (ew_f[0] == 'W') lon = -lon;
            s_lat = lat; s_lon = lon;
            s_has_position = true;
            compute_maidenhead(lat, lon, s_locator);
        }
        if (alt_f) s_alt_m = parse_alt(alt_f);
    }
}

static void parse_rmc(const char *s)
{
    parse_time(s, 1u);
    const char *d = nmea_field(s, 9u);          // ddmmyy
    if (d && is_digit6(d)) {
        for (uint8_t i = 0u; i < 6u; i++) s_date[i] = d[i];
        s_date[6] = '\0';
    }
}

static void parse_gsv(const char *s)
{
    if (parse_uint(nmea_field(s, 2u)) != 1u) return; // only first message
    uint16_t visible = parse_uint(nmea_field(s, 3u));
    int8_t bucket    = talker_to_gsv_bucket(s[1], s[2]);
    if (bucket < 0) return;
    s_gsvVisible[(uint8_t)bucket] = (visible > 255u) ? 255u : (uint8_t)visible;
    s_gsvStampMs[(uint8_t)bucket] = gpsdo_ms();
}

static void process_nmea_sentence(const char *s)
{
    if (s[0] != '$' || s[1] == '\0' || s[2] == '\0' ||
        s[3] == '\0' || s[4] == '\0' || s[5] == '\0') return;
    if (!nmea_checksum_ok(s)) return;
    s_lastNmeaMs = gpsdo_ms();
    s_nmeaCount++;
    char c4 = s[3], c5 = s[4], c6 = s[5];
    if (c4 == 'G' && c5 == 'G' && c6 == 'A') { parse_gga(s); return; }
    if (c4 == 'R' && c5 == 'M' && c6 == 'C') { parse_rmc(s); return; }
    if (c4 == 'G' && c5 == 'S' && c6 == 'V') { parse_gsv(s); return; }
}

static void process_incoming_gps(void)
{
    while (uart_is_readable(GPSDO_UART)) {
        char c = (char)uart_getc(GPSDO_UART);
        s_uartBytesRx++;
        if (c == '$') {
            s_collecting   = true;
            s_nmeaOverflow = false;
            s_nmeaIdx      = 0u;
            s_nmeaBuf[s_nmeaIdx++] = c;
            continue;
        }
        if (!s_collecting) continue;
        if (c == '\r' || c == '\n') {
            if (!s_nmeaOverflow && s_nmeaIdx > 6u) {
                s_nmeaBuf[s_nmeaIdx] = '\0';
                process_nmea_sentence(s_nmeaBuf);
            }
            s_collecting   = false;
            s_nmeaOverflow = false;
            s_nmeaIdx      = 0u;
            continue;
        }
        if (!s_nmeaOverflow) {
            if (s_nmeaIdx < (GPSDO_NMEA_BUF - 1u))
                s_nmeaBuf[s_nmeaIdx++] = c;
            else
                s_nmeaOverflow = true;
        }
    }
}

// ---------------------------------------------------------------------------
// UBX configuration packets
// ---------------------------------------------------------------------------

// CFG-TP5: TIMEPULSE2 = 24 MHz continuous (drives SI5351 reference input).
static const uint8_t UBX_CFG_TP5_24MHZ[] = {
    0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36,
    0x6E, 0x01, 0x00, 0x36, 0x6E, 0x01, 0x00, 0x00,
    0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x11, 0xD8
};

// CFG-NAV5: Stationary dynamic model.
static const uint8_t UBX_CFG_NAV5_STATIONARY[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
    0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
    0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
    0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x4E, 0x60
};

static void send_ubx_packet(const uint8_t *pkt, size_t len)
{
    for (size_t i = 0u; i < len; i++) {
        uart_putc_raw(GPSDO_UART, pkt[i]);
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void gpsdo_init(void)
{
    memset(s_gsvVisible, 0, sizeof(s_gsvVisible));
    memset(s_gsvStampMs, 0, sizeof(s_gsvStampMs));

    // ── I2C0 for SI5351 ──────────────────────────────────────────────────────
    i2c_init(GPSDO_I2C, GPSDO_I2C_BAUD);
    gpio_set_function(GPSDO_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPSDO_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(GPSDO_I2C_SDA);
    gpio_pull_up(GPSDO_I2C_SCL);

    sleep_ms(10u); // SI5351 power-on stabilisation

    s_clk1Ok = si5351_init_clocks();
    if (s_clk1Ok) {
        sleep_ms(50u);  // give PLL time to lock before reading status
        si_read_reg(SI_REG_STATUS, &s_si5351_status);
        s_si5351_pollMs = to_ms_since_boot(get_absolute_time());
        printf("[GPSDO] SI5351 reg0=0x%02X: clk1=%s  "
               "(LOL_A=%d LOS_XTAL=%d SYS_INIT=%d)\n",
               (unsigned)s_si5351_status, si5351_clk_status(),
               !!(s_si5351_status & SI_STATUS_LOL_A),
               !!(s_si5351_status & SI_STATUS_LOS_XTAL),
               !!(s_si5351_status & SI_STATUS_SYS_INIT));
    } else {
        printf("[GPSDO] SI5351 not found or I2C error\n");
    }

    // ── UART1 for NEO-7M GPS ─────────────────────────────────────────────────
    gpio_set_function(GPSDO_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(GPSDO_UART_RX, GPIO_FUNC_UART);

    // Auto-detect GPS baud rate.
    // NEO-7M default is 9600, but u-center or prior config may have changed it.
    // We try each rate for 400 ms and look for a '$' (NMEA sentence start).
    static const uint32_t baud_candidates[] = { 9600u, 38400u, 57600u, 115200u };
    uint32_t detected_baud = 9600u; // fallback
    for (size_t bi = 0u; bi < sizeof(baud_candidates)/sizeof(baud_candidates[0]); bi++) {
        uart_init(GPSDO_UART, baud_candidates[bi]);
        uart_set_format(GPSDO_UART, 8, 1, UART_PARITY_NONE);
        // Drain stale bytes from previous attempt.
        while (uart_is_readable(GPSDO_UART)) { uart_getc(GPSDO_UART); }
        uint32_t t_end = to_ms_since_boot(get_absolute_time()) + 400u;
        uint32_t rx_count = 0u;
        bool found_dollar = false;
        while ((int32_t)(to_ms_since_boot(get_absolute_time()) - t_end) < 0) {
            if (uart_is_readable(GPSDO_UART)) {
                char c = (char)uart_getc(GPSDO_UART);
                rx_count++;
                if (c == '$') { found_dollar = true; }
            }
        }
        if (found_dollar && rx_count >= 10u) {
            // Enough data with a proper NMEA start — baud rate confirmed.
            detected_baud = baud_candidates[bi];
            s_uartBytesRx = rx_count;
            break;
        }
    }
    // Re-init at confirmed baud rate (already set in loop, but make it explicit).
    uart_init(GPSDO_UART, detected_baud);
    uart_set_format(GPSDO_UART, 8, 1, UART_PARITY_NONE);
    while (uart_is_readable(GPSDO_UART)) { uart_getc(GPSDO_UART); }

    // Store detected baud for status reporting.
    s_detected_baud = detected_baud;

    send_ubx_packet(UBX_CFG_TP5_24MHZ,       sizeof(UBX_CFG_TP5_24MHZ));
    sleep_ms(50u);
    send_ubx_packet(UBX_CFG_NAV5_STATIONARY, sizeof(UBX_CFG_NAV5_STATIONARY));
    sleep_ms(50u);

    printf("[GPSDO] UBX config sent — waiting for GPS lock...\n");
}

static bool si_locked_now(void)
{
    return s_clk1Ok &&
           !(s_si5351_status & SI_STATUS_LOS_XTAL) &&
           !(s_si5351_status & SI_STATUS_LOL_A);
}

void gpsdo_task(void)
{
    process_incoming_gps();

    // Re-read the SI5351 status register. Fast while waiting for the PLL at
    // boot (the GPS TIMEPULSE may appear a moment after UBX config); slow once
    // locked, to keep I2C traffic away from an active transmission.
    if (s_clk1Ok) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        uint32_t interval = si_locked_now() ? GPSDO_SI5351_POLL_MS
                                            : GPSDO_SI5351_FAST_POLL_MS;
        if ((now_ms - s_si5351_pollMs) >= interval) {
            s_si5351_pollMs = now_ms;
            si_read_reg(SI_REG_STATUS, &s_si5351_status);
        }
    }

    // READY when SI5351 is locked AND GPS has provided at least one UTC timestamp.
    // UTC appearing means the GPS module has disciplined its oscillator to at least
    // one satellite — the TIMEPULSE 24 MHz is now frequency-accurate.
    // Without UTC the GPS runs on its free-running TCXO (±2.5 ppm = ±6 kHz at 2.4 GHz).
    const bool utc_valid = (s_utc[0] != '-');
    // Remember the last moment everything was actually good, so a lock that
    // is lost later can be reported (s_gpsdoReady itself never falls back:
    // the SX1280 keeps running on the free-wheeling TIMEPULSE).
    if (si_locked_now() && s_fixQuality > 0 && s_satsUsed >= 3) {
        s_lastGoodMs = gpsdo_ms();
    }

    if (!s_gpsdoReady && si_locked_now() && utc_valid) {
        s_gpsdoReady = true;
        printf("[GPSDO] READY — SI5351 locked + GPS UTC received, TX enabled\n");
    }
}

bool gpsdo_clock_ok(void)
{
    return si_locked_now();
}

bool gpsdo_signal_lost(void)
{
    if (s_lastGoodMs == 0u) return false;                     // never had a fix — not "lost"
    return (gpsdo_ms() - s_lastGoodMs) >= GPSDO_LOST_MS;
}

bool gpsdo_is_ready(void)
{
    return s_gpsdoReady;
}

bool gpsdo_si5351_ok(void)
{
    return s_clk1Ok;
}

int gpsdo_format_status(char *buf, size_t size)
{
    const uint32_t now = gpsdo_ms();

    // sig: GPS time-discipline status
    //   "stable" – UTC received, GGA fresh, satellites tracked → 24 MHz disciplined
    //   "lost"   – had a fix, none for GPSDO_LOST_MS           → drifting on the GPS TCXO
    //   "stale"  – had UTC but no GGA for >60 s                → receiver silent
    //   "wait"   – no UTC ever received                        → cold start
    // NOTE: the NEO-7M keeps emitting GGA without a fix, so sentence arrival
    // alone does not mean the reference is disciplined — hence the "lost" case.
    const bool utc_valid  = (s_utc[0] != '-');
    const bool gga_recent = (s_lastGgaMs != 0u) && ((now - s_lastGgaMs) <= 60000u);
    const bool lost       = gpsdo_signal_lost();
    const char *sig = !utc_valid       ? "wait"
                    : !gga_recent      ? "stale"
                    : lost             ? "lost"
                    :                    "stable";

    // fix: position fix (needs ≥3 sats — consistent with Arduino "LCK" = sats≥3)
    //   "locked" – position fix active, locator valid
    //   "wait"   – no fix yet
    const char *fix = (s_fixQuality > 0 && s_satsUsed >= 3) ? "locked" : "wait";

    // Always use last known sats/vis — do NOT zero on stale (robustness against
    // brief interruptions, same approach as the reference Arduino sketch).
    const uint16_t vis = get_visible_sats();

    // Format UTC as HH:MM:SS (last known value, never zeroed)
    char utc_str[9];
    if (utc_valid) {
        snprintf(utc_str, sizeof(utc_str), "%c%c:%c%c:%c%c",
                 s_utc[0], s_utc[1], s_utc[2], s_utc[3], s_utc[4], s_utc[5]);
    } else {
        snprintf(utc_str, sizeof(utc_str), "--:--:--");
    }

    return snprintf(buf, size,
                    "GPSDO: sig=%s fix=%s sats=%d vis=%d clk1=%s"
                    " utc=%s loc=%s alt=%dm lost=%d\r\n",
                    sig, fix,
                    s_satsUsed,   // last known — not zeroed on stale
                    (int)vis,
                    si5351_clk_status(),
                    utc_str,
                    s_has_position ? s_locator : "------",
                    (int)s_alt_m,
                    lost ? 1 : 0);
}

void gpsdo_get_info(gpsdo_info_t *o)
{
    const bool utc_valid  = (s_utc[0] != '-');
    const bool date_valid = (s_date[0] != '-');
    if (utc_valid) snprintf(o->utc_hhmm, sizeof(o->utc_hhmm), "%c%c:%c%c", s_utc[0], s_utc[1], s_utc[2], s_utc[3]);
    else           snprintf(o->utc_hhmm, sizeof(o->utc_hhmm), "--:--");
    if (date_valid) snprintf(o->date, sizeof(o->date), "%c%c.%c%c.20%c%c",
                             s_date[0], s_date[1], s_date[2], s_date[3], s_date[4], s_date[5]);
    else            snprintf(o->date, sizeof(o->date), "--.--.----");
    snprintf(o->locator, sizeof(o->locator), "%s", s_has_position ? s_locator : "------");
    o->utc_valid   = utc_valid;
    o->fix         = (s_fixQuality > 0 && s_satsUsed >= 3);
    o->sats_used   = (uint8_t)(s_satsUsed < 0 ? 0 : (s_satsUsed > 255 ? 255 : s_satsUsed));
    o->sats_vis    = (uint8_t)(get_visible_sats() > 255u ? 255u : get_visible_sats());
    o->alt_m       = s_alt_m;
}

bool gpsdo_status_due(void)
{
    const uint32_t now = gpsdo_ms();
    if ((now - s_lastPrintMs) >= GPSDO_PRINT_MS) {
        s_lastPrintMs = now;
        return true;
    }
    return false;
}
