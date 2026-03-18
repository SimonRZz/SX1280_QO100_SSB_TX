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
#define GPSDO_I2C_BAUD      400000u   // 400 kHz fast-mode

#define GPSDO_UART          uart1
#define GPSDO_UART_TX       4u        // GP4 → NEO-7M RX
#define GPSDO_UART_RX       5u        // GP5 ← NEO-7M TX
#define GPSDO_GPS_BAUD      9600u

#define GPSDO_PRINT_MS      2000u     // status line interval
#define GPSDO_NMEA_STALE_MS 3000u
#define GPSDO_GGA_STALE_MS  3000u
#define GPSDO_GSV_FRESH_MS  5000u
#define GPSDO_NMEA_BUF      128u
#define GPSDO_BOOT_DELAY_MS 300u      // wait before sending UBX commands

// ---------------------------------------------------------------------------
// SI5351 minimal driver (I2C0)
// Configures 52 MHz on CLK1 from a 24 MHz GPS TIMEPULSE reference.
//
// VCO:    24 MHz × (36 + 5/6) = 884 MHz   (PLL A)
// CLK1:   884 MHz ÷ 17        = 52 MHz    (integer divider)
//
// Register values:
//   PLL A: p1=4202 (0x106A), p2=4, p3=6
//   MS1:   p1=1664 (0x0680), p2=0, p3=1
// ---------------------------------------------------------------------------
#define SI_ADDR            0x60u

#define SI_REG_OEB         3u    // Output Enable (active LOW per bit)
#define SI_REG_CLK0_CTRL   16u
#define SI_REG_CLK1_CTRL   17u
#define SI_REG_CLK2_CTRL   18u
#define SI_REG_PLLA_BASE   26u   // 8 bytes: regs 26-33
#define SI_REG_MS1_BASE    58u   // 8 bytes: regs 58-65
#define SI_REG_PLL_RESET   177u
#define SI_REG_XTAL_LOAD   183u

static bool si_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_write_blocking(GPSDO_I2C, SI_ADDR, buf, 2, false) == 2;
}

static bool si_write_regs(uint8_t base, const uint8_t *data, size_t len)
{
    // max needed: 1 address byte + 8 data bytes = 9
    uint8_t buf[9];
    if (len == 0u || len > 8u) return false;
    buf[0] = base;
    memcpy(buf + 1, data, len);
    return i2c_write_blocking(GPSDO_I2C, SI_ADDR, buf, len + 1u, false)
           == (int)(len + 1u);
}

static bool si5351_init_52mhz(void)
{
    // Disable all outputs while configuring.
    if (!si_write_reg(SI_REG_OEB, 0xFFu)) return false;

    // Power down unused CLK0 and CLK2.
    si_write_reg(SI_REG_CLK0_CTRL, 0x80u);
    si_write_reg(SI_REG_CLK2_CTRL, 0x80u);

    // CLK1: PDN=0, INT=1 (integer mode), MS_SRC=PLLA, INV=0,
    //        CLK_SRC=MS1(self)=0b11, IDRV=2mA=0b00
    //   Byte: 0b 0 1 0 0 11 00 = 0x4C
    si_write_reg(SI_REG_CLK1_CTRL, 0x4Cu);

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
    //   Regs 58-65
    static const uint8_t ms1[8] = {
        0x00u, 0x01u,   // p3=1
        0x00u,          // R_DIV=/1, DIVBY4=0, p1[17:16]=0
        0x06u, 0x80u,   // p1[15:8]=0x06, p1[7:0]=0x80  (1664)
        0x00u,          // upper nibbles = 0
        0x00u, 0x00u    // p2=0
    };
    if (!si_write_regs(SI_REG_MS1_BASE, ms1, 8u)) return false;

    // Reset PLLs to lock them to the new multisynth settings.
    si_write_reg(SI_REG_PLL_RESET, 0xA0u);

    // Enable CLK1 only (bit 1 = 0 → enabled).
    si_write_reg(SI_REG_OEB, 0xFDu);

    return true;
}

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
static int      s_fixQuality      = 0;
static int      s_satsUsed        = 0;
static char     s_utc[7]          = "------";
static bool     s_clk1Ok          = false;
static bool     s_gpsdoReady      = false;

static char     s_nmeaBuf[GPSDO_NMEA_BUF];
static size_t   s_nmeaIdx         = 0;
static bool     s_collecting      = false;
static bool     s_nmeaOverflow    = false;

static uint32_t s_lastNmeaMs      = 0;
static uint32_t s_lastTimeMs      = 0;
static uint32_t s_lastGgaMs       = 0;
static uint32_t s_lastPrintMs     = 0;

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
}

static void parse_rmc(const char *s) { parse_time(s, 1u); }

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
    char c4 = s[3], c5 = s[4], c6 = s[5];
    if (c4 == 'G' && c5 == 'G' && c6 == 'A') { parse_gga(s); return; }
    if (c4 == 'R' && c5 == 'M' && c6 == 'C') { parse_rmc(s); return; }
    if (c4 == 'G' && c5 == 'S' && c6 == 'V') { parse_gsv(s); return; }
}

static void process_incoming_gps(void)
{
    while (uart_is_readable(GPSDO_UART)) {
        char c = (char)uart_getc(GPSDO_UART);
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

    s_clk1Ok = si5351_init_52mhz();
    printf("[GPSDO] SI5351 52MHz CLK1: %s\n", s_clk1Ok ? "OK" : "FAIL");

    // ── UART1 for NEO-7M GPS ─────────────────────────────────────────────────
    uart_init(GPSDO_UART, GPSDO_GPS_BAUD);
    gpio_set_function(GPSDO_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(GPSDO_UART_RX, GPIO_FUNC_UART);

    // Allow the NEO-7M to boot before sending UBX commands.
    sleep_ms(GPSDO_BOOT_DELAY_MS);

    // Drain any stale bytes.
    while (uart_is_readable(GPSDO_UART)) { uart_getc(GPSDO_UART); }

    send_ubx_packet(UBX_CFG_TP5_24MHZ,       sizeof(UBX_CFG_TP5_24MHZ));
    sleep_ms(50u);
    send_ubx_packet(UBX_CFG_NAV5_STATIONARY, sizeof(UBX_CFG_NAV5_STATIONARY));
    sleep_ms(50u);

    printf("[GPSDO] UBX config sent — waiting for GPS lock...\n");
}

void gpsdo_task(void)
{
    process_incoming_gps();

    if (!s_gpsdoReady && s_clk1Ok && s_fixQuality > 0 && s_satsUsed >= 1) {
        s_gpsdoReady = true;
        printf("[GPSDO] READY — satellite lock acquired\n");
    }
}

bool gpsdo_is_ready(void)
{
    return s_gpsdoReady;
}

int gpsdo_format_status(char *buf, size_t size)
{
    const bool    locked   = s_clk1Ok && (s_fixQuality > 0) && (s_satsUsed >= 1);
    const uint32_t now     = gpsdo_ms();
    const bool    ggaFresh = (s_lastGgaMs != 0u) &&
                             ((now - s_lastGgaMs) <= GPSDO_GGA_STALE_MS);
    const int     sats     = ggaFresh ? s_satsUsed : 0;
    (void)get_visible_sats(); // keep function referenced
    return snprintf(buf, size, "GPSDO: lock=%d sats=%d clk1=%s\r\n",
                    locked ? 1 : 0, sats, s_clk1Ok ? "ok" : "fail");
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
