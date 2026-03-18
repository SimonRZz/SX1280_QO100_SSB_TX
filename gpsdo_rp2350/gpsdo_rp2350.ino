// CT2GQV GPSDO for QO-100 — Raspberry Pi Pico 2W (RP2350) Port
// Original sketch: CT2GQV_GPSDO_52MHz.ino (Arduino Nano / CT2GQV)
// Ported to RP2350 with the arduino-pico framework.
//
// Hardware wiring:
//   u-blox NEO-7M GPS  → UART1  GP5 = RX (← GPS TX), GP4 = TX (→ GPS RX)
//   SI5351 clock gen   → I2C0   GP2 = SDA, GP3 = SCL
//   TCXOEN             → GP22   permanently HIGH (first GPIO action in setup)
//   SX1280 NRESET      → GP15   held LOW until GPSDO declares READY
//   CW key input       → GP10   (active LOW, internal pull-up; unused in Step 1)
//
// Boot sequence (Step 1 — GPSDO only, SX1280 not initialised yet):
//   1. GP22 HIGH immediately.
//   2. GP15 (SX1280 NRESET) LOW — SX1280 stays in reset.
//   3. I2C0 and SI5351 initialised → 52 MHz on CLK1.
//   4. UART1 opened at 9600 baud.
//   5. UBX-CFG-TP5 (24 MHz continuous) and UBX-CFG-NAV5 (stationary) sent.
//   6. NMEA parsed until fixQuality > 0 && satsUsed >= 1.
//   7. "GPSDO: READY" printed — future Step 2 releases SX1280 NRESET.
//
// Periodic USB-Serial status (machine-parseable for the Python GUI):
//   GPSDO: lock=<0|1> sats=<N> clk1=<ok|fail>
//
// Key changes from the Nano original:
//   • SoftwareSerial   → Hardware UART1 (Serial2) with setRX/setTX
//   • Wire.begin()     → Wire.setSDA(2) / Wire.setSCL(3) before begin()
//   • gpsSerial.listen() removed (UART1 is always active on RP2350)
//   • SI5351 init failure no longer halts — clk1=fail is reported instead
//   • gpsdoReady flag gates the SX1280 release point (future integration)
//   • GPSDO status line added for Python GUI parsing
//   • avr/pgmspace.h   → arduino-pico pgmspace.h (PROGMEM is transparent on RP2350)

#include <Arduino.h>
#include <Wire.h>
#include "si5351.h"

#if defined(ARDUINO_ARCH_RP2040)
#  include <pgmspace.h>      // provided by arduino-pico; no-op on RP2350
#elif defined(ARDUINO_ARCH_AVR)
#  include <avr/pgmspace.h>
#else
#  include <pgmspace.h>
#endif

// ---------------------------------------------------------------------------
// Pin assignments — RP2350 / Pico 2W
// ---------------------------------------------------------------------------
static const uint8_t PIN_TCXOEN   = 22;  // TCXO enable — permanently HIGH
static const uint8_t PIN_SX_RESET = 15;  // SX1280 NRESET — LOW until GPSDO ready
static const uint8_t PIN_GPS_TX   =  4;  // UART1 TX → NEO-7M RX
static const uint8_t PIN_GPS_RX   =  5;  // UART1 RX ← NEO-7M TX
static const uint8_t PIN_SI_SDA   =  2;  // I2C0 SDA → SI5351
static const uint8_t PIN_SI_SCL   =  3;  // I2C0 SCL → SI5351

// ---------------------------------------------------------------------------
// User configuration
// ---------------------------------------------------------------------------
static const uint32_t GPS_BAUD   = 9600UL;
static const uint32_t DEBUG_BAUD = 115200UL;

// Set SEND_UBX 0 only if you want to skip GPS receiver configuration entirely.
#define SEND_UBX 1

// Waiting for UBX ACK/NAK is useful for diagnostics but adds startup latency.
// Default off — GPSDO time is the priority.
static const bool WAIT_FOR_UBX_ACK = false;

// When true, GSV sentences are parsed and satellite visibility is tracked.
static const bool PARSE_GSV = true;

// Time to let the NEO-7M boot before sending UBX commands.
// SI5351 init is overlapped with this delay to keep startup fast.
static const unsigned long GPS_BOOT_DELAY_MS = 300UL;

// How often a human-readable + machine-parseable status line is printed.
static const unsigned long PRINT_INTERVAL_MS = 2000UL;

// Freshness thresholds — only affect the status display, not operation.
static const unsigned long NMEA_STALE_MS = 3000UL;
static const unsigned long TIME_STALE_MS = 3000UL;
static const unsigned long GGA_STALE_MS  = 3000UL;
static const unsigned long GSV_FRESH_MS  = 5000UL;

// Used only when WAIT_FOR_UBX_ACK is true.
static const unsigned long UBX_ACK_TIMEOUT_MS = 1200UL;

// NMEA line buffer. 128 bytes gives headroom above the classic 82-byte limit.
static const size_t NMEA_BUF_SIZE = 128U;

// ---------------------------------------------------------------------------
// UBX configuration packets (stored in flash via PROGMEM)
// ---------------------------------------------------------------------------
#if SEND_UBX

// CFG-TP5: TIMEPULSE output = 24 MHz continuous.
// This drives the SI5351 reference input.
// NOTE: this is NOT a 1 PPS packet.
const uint8_t UBX_CFG_TP5_24MHZ[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x36,
  0x6E, 0x01, 0x00, 0x36, 0x6E, 0x01, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x11, 0xD8
};

// CFG-NAV5: Stationary dynamic model.
const uint8_t UBX_CFG_NAV5_STATIONARY[] PROGMEM = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF,
  0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27,
  0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00,
  0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x4E, 0x60
};

#endif  // SEND_UBX

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------
Si5351 si5351;

// Serial  = USB CDC (debug + GUI output)
// Serial2 = UART1, configured with setRX/setTX in setup()

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
char   nmeaBuf[NMEA_BUF_SIZE];
size_t nmeaIdx            = 0;
bool   collectingSentence = false;
bool   nmeaOverflow       = false;

int  fixQuality = 0;       // GGA field 6 (0 = no fix)
int  satsUsed   = 0;       // GGA field 7
char utc[7]     = "------"; // "HHMMSS\0"

unsigned long lastNmeaMs  = 0;
unsigned long lastTimeMs  = 0;
unsigned long lastGgaMs   = 0;
unsigned long lastPrintMs = 0;

unsigned long cntGGA          = 0;
unsigned long cntRMC          = 0;
unsigned long cntGSV          = 0;
unsigned long cntOther        = 0;
unsigned long cntChecksumFail = 0;
unsigned long cntOverflow     = 0;
unsigned long cntUbxAck       = 0;
unsigned long cntUbxNak       = 0;
unsigned long cntUbxTimeout   = 0;

// true once SI5351 CLK1 has been configured successfully.
bool clk1Ok = false;

// true once the GPSDO has acquired at least one satellite and CLK1 is stable.
// In the full integrated sketch this flag gates the SX1280 initialisation.
bool gpsdoReady = false;

// ---------------------------------------------------------------------------
// GSV constellation buckets
// ---------------------------------------------------------------------------
enum GsvBucket : uint8_t {
  GSV_GP = 0,   // GPS
  GSV_GL,       // GLONASS
  GSV_GA,       // Galileo
  GSV_GB,       // BeiDou (also "BD")
  GSV_GQ,       // QZSS
  GSV_GI,       // NavIC / IRNSS
  GSV_GN,       // Combined multi-GNSS
  GSV_BUCKET_COUNT
};

uint8_t       gsvVisible[GSV_BUCKET_COUNT] = {0};
unsigned long gsvStampMs[GSV_BUCKET_COUNT] = {0};

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static inline bool isDigit6(const char* p);
static uint16_t    parseUInt(const char* p);
static int8_t      hexNibble(char c);
static const char* field(const char* s, uint8_t n);
static bool        validNmeaChecksum(const char* s);

#if SEND_UBX
static void   ubxChecksumStep(uint8_t b, uint8_t& ckA, uint8_t& ckB);
static int8_t waitForUbxAck(uint8_t cls, uint8_t id, unsigned long tms);
static void   sendUbxPacket_P(const uint8_t* pkt, size_t len);
static void   sendUbxPacketWithOptionalAck(const uint8_t* pkt, size_t len,
                                           uint8_t cls, uint8_t id,
                                           const __FlashStringHelper* lbl);
#endif

static int8_t   talkerToGsvBucket(char t1, char t2);
static uint16_t getVisibleSatCount(void);
static void     parseTime(const char* s, uint8_t fn);
static void     parseGGA(const char* s);
static void     parseRMC(const char* s);
static void     parseGSV(const char* s);
static void     processSentence(const char* s);
static void     processIncomingGps(void);
static void     printUtc(void);
static void     printStatusLine(void);
static void     initSi5351Clock(void);

// ---------------------------------------------------------------------------
// Small helpers (logic unchanged from original Nano sketch)
// ---------------------------------------------------------------------------

static inline bool isDigit6(const char* p)
{
  if (!p) return false;
  for (uint8_t i = 0; i < 6; i++) {
    if (p[i] < '0' || p[i] > '9') return false;
  }
  return true;
}

static uint16_t parseUInt(const char* p)
{
  if (!p) return 0;
  uint16_t value = 0;
  bool any = false;
  while (*p >= '0' && *p <= '9') {
    any = true;
    value = (uint16_t)(value * 10U + (uint16_t)(*p - '0'));
    ++p;
  }
  return any ? value : 0;
}

static int8_t hexNibble(char c)
{
  if (c >= '0' && c <= '9') return (int8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
  return -1;
}

static const char* field(const char* s, uint8_t n)
{
  uint8_t commaCount = 0;
  while (*s) {
    if (commaCount == n) return s;
    if (*s == '*') break;
    if (*s == ',') commaCount++;
    s++;
  }
  return (commaCount == n) ? s : nullptr;
}

static bool validNmeaChecksum(const char* s)
{
  if (!s || s[0] != '$') return false;
  uint8_t cs = 0;
  const char* p = s + 1;
  while (*p && *p != '*') { cs ^= (uint8_t)(*p); ++p; }
  if (*p != '*' || !p[1] || !p[2]) return false;
  const int8_t hi = hexNibble(p[1]);
  const int8_t lo = hexNibble(p[2]);
  if (hi < 0 || lo < 0) return false;
  return cs == (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
}

// ---------------------------------------------------------------------------
// UBX helpers
// ---------------------------------------------------------------------------
#if SEND_UBX

enum : int8_t {
  UBX_ACK_TIMEOUT_VAL = -1,
  UBX_ACK_NAK_VAL     =  0,
  UBX_ACK_ACK_VAL     =  1
};

static void ubxChecksumStep(uint8_t b, uint8_t& ckA, uint8_t& ckB)
{
  ckA = (uint8_t)(ckA + b);
  ckB = (uint8_t)(ckB + ckA);
}

static int8_t waitForUbxAck(uint8_t expectedClass, uint8_t expectedId,
                             unsigned long timeoutMs)
{
  uint8_t state    = 0;
  uint8_t msgClass = 0, msgId = 0;
  uint8_t lenL     = 0, lenH = 0;
  uint8_t payload0 = 0, payload1 = 0;
  uint8_t rxCkA    = 0, rxCkB = 0;
  uint8_t ckA      = 0, ckB   = 0;

  const unsigned long startMs = millis();
  while ((unsigned long)(millis() - startMs) < timeoutMs) {
    if (!Serial2.available()) continue;
    const uint8_t b = (uint8_t)Serial2.read();
    switch (state) {
      case 0: if (b == 0xB5) state = 1; break;
      case 1:
        if      (b == 0x62) { state = 2; ckA = 0; ckB = 0; }
        else if (b != 0xB5) { state = 0; }
        break;
      case 2: msgClass = b; ubxChecksumStep(b, ckA, ckB); state = 3; break;
      case 3: msgId    = b; ubxChecksumStep(b, ckA, ckB); state = 4; break;
      case 4: lenL     = b; ubxChecksumStep(b, ckA, ckB); state = 5; break;
      case 5:
        lenH = b; ubxChecksumStep(b, ckA, ckB);
        state = (lenL == 2 && lenH == 0) ? 6 : 0;
        break;
      case 6: payload0 = b; ubxChecksumStep(b, ckA, ckB); state = 7; break;
      case 7: payload1 = b; ubxChecksumStep(b, ckA, ckB); state = 8; break;
      case 8: rxCkA    = b; state = 9; break;
      case 9:
        rxCkB = b;
        if (rxCkA == ckA && rxCkB == ckB &&
            msgClass == 0x05 && (msgId == 0x01 || msgId == 0x00) &&
            payload0 == expectedClass && payload1 == expectedId) {
          return (msgId == 0x01) ? UBX_ACK_ACK_VAL : UBX_ACK_NAK_VAL;
        }
        state = 0;
        break;
    }
  }
  return UBX_ACK_TIMEOUT_VAL;
}

static void sendUbxPacket_P(const uint8_t* packet, size_t len)
{
  for (size_t i = 0; i < len; ++i) {
    Serial2.write((uint8_t)pgm_read_byte(packet + i));
  }
}

static void sendUbxPacketWithOptionalAck(const uint8_t* packet, size_t len,
                                          uint8_t expectedClass, uint8_t expectedId,
                                          const __FlashStringHelper* label)
{
  while (Serial2.available()) Serial2.read();   // drain stale bytes
  sendUbxPacket_P(packet, len);
  Serial.print(F("Sent ")); Serial.print(label);
  if (WAIT_FOR_UBX_ACK) {
    const int8_t ack = waitForUbxAck(expectedClass, expectedId, UBX_ACK_TIMEOUT_MS);
    if      (ack == UBX_ACK_ACK_VAL) { cntUbxAck++;     Serial.println(F(" -> ACK")); }
    else if (ack == UBX_ACK_NAK_VAL) { cntUbxNak++;     Serial.println(F(" -> NAK")); }
    else                              { cntUbxTimeout++; Serial.println(F(" -> timeout")); }
  } else {
    Serial.println(F(" (ACK wait disabled)"));
  }
}

#endif  // SEND_UBX

// ---------------------------------------------------------------------------
// GSV helpers
// ---------------------------------------------------------------------------

static int8_t talkerToGsvBucket(char t1, char t2)
{
  if (t1 == 'G' && t2 == 'P') return GSV_GP;
  if (t1 == 'G' && t2 == 'L') return GSV_GL;
  if (t1 == 'G' && t2 == 'A') return GSV_GA;
  if ((t1 == 'G' && t2 == 'B') || (t1 == 'B' && t2 == 'D')) return GSV_GB;
  if (t1 == 'G' && t2 == 'Q') return GSV_GQ;
  if (t1 == 'G' && t2 == 'I') return GSV_GI;
  if (t1 == 'G' && t2 == 'N') return GSV_GN;
  return -1;
}

// Returns the best available satellite-visible count.
// Prefers a fresh combined GN group; falls back to summing individual groups.
static uint16_t getVisibleSatCount(void)
{
  const unsigned long now = millis();
  if (gsvStampMs[GSV_GN] != 0UL &&
      (unsigned long)(now - gsvStampMs[GSV_GN]) <= GSV_FRESH_MS) {
    return gsvVisible[GSV_GN];
  }
  uint16_t sum = 0;
  for (uint8_t i = 0; i < GSV_BUCKET_COUNT; ++i) {
    if (i == GSV_GN) continue;
    if (gsvStampMs[i] != 0UL &&
        (unsigned long)(now - gsvStampMs[i]) <= GSV_FRESH_MS) {
      sum += gsvVisible[i];
    }
  }
  return sum;
}

// ---------------------------------------------------------------------------
// NMEA parsing
// ---------------------------------------------------------------------------

// Accept UTC time as soon as six consecutive digits appear in the expected
// field. No software "lock" state is required — time visibility is the goal.
static void parseTime(const char* s, uint8_t fieldNum)
{
  const char* f = field(s, fieldNum);
  if (f && isDigit6(f)) {
    utc[0] = f[0]; utc[1] = f[1]; utc[2] = f[2];
    utc[3] = f[3]; utc[4] = f[4]; utc[5] = f[5]; utc[6] = '\0';
    lastTimeMs = millis();
  }
}

static void parseGGA(const char* s)
{
  lastGgaMs  = millis();
  parseTime(s, 1);
  fixQuality = (int)parseUInt(field(s, 6));
  satsUsed   = (int)parseUInt(field(s, 7));
}

static void parseRMC(const char* s) { parseTime(s, 1); }

static void parseGSV(const char* s)
{
  if (!PARSE_GSV) return;
  const uint16_t msgNum  = parseUInt(field(s, 2));
  const uint16_t visible = parseUInt(field(s, 3));
  if (msgNum != 1U) return;   // total is repeated in message 1 — no need for others
  const int8_t bucket = talkerToGsvBucket(s[1], s[2]);
  if (bucket < 0) return;
  gsvVisible[(uint8_t)bucket] = (visible > 255U) ? 255U : (uint8_t)visible;
  gsvStampMs[(uint8_t)bucket] = millis();
}

static void processSentence(const char* s)
{
  if (s[0] != '$' || s[1] == '\0' || s[2] == '\0' ||
      s[3] == '\0' || s[4] == '\0' || s[5] == '\0') return;
  if (!validNmeaChecksum(s)) { cntChecksumFail++; return; }
  lastNmeaMs = millis();
  const char c4 = s[3], c5 = s[4], c6 = s[5];
  if (c4 == 'G' && c5 == 'G' && c6 == 'A') { cntGGA++; parseGGA(s); return; }
  if (c4 == 'R' && c5 == 'M' && c6 == 'C') { cntRMC++; parseRMC(s); return; }
  if (c4 == 'G' && c5 == 'S' && c6 == 'V') { cntGSV++; parseGSV(s); return; }
  cntOther++;
}

// Read all available GPS UART bytes and assemble NMEA lines.
// '$' resynchronises immediately; overlong lines are discarded cleanly.
static void processIncomingGps(void)
{
  while (Serial2.available()) {
    const char c = (char)Serial2.read();
    if (c == '$') {
      collectingSentence = true;
      nmeaOverflow = false;
      nmeaIdx = 0;
      nmeaBuf[nmeaIdx++] = c;
      continue;
    }
    if (!collectingSentence) continue;
    if (c == '\r' || c == '\n') {
      if (!nmeaOverflow && nmeaIdx > 6U) {
        nmeaBuf[nmeaIdx] = '\0';
        processSentence(nmeaBuf);
      }
      collectingSentence = false;
      nmeaOverflow = false;
      nmeaIdx = 0;
      continue;
    }
    if (!nmeaOverflow) {
      if (nmeaIdx < (NMEA_BUF_SIZE - 1U)) {
        nmeaBuf[nmeaIdx++] = c;
      } else {
        nmeaOverflow = true;
        cntOverflow++;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------

static void printUtc(void)
{
  Serial.print(utc[0]); Serial.print(utc[1]); Serial.print(':');
  Serial.print(utc[2]); Serial.print(utc[3]); Serial.print(':');
  Serial.print(utc[4]); Serial.print(utc[5]);
}

static void printStatusLine(void)
{
  const unsigned long now = millis();
  const bool rxFresh   = (lastNmeaMs != 0UL) && ((unsigned long)(now - lastNmeaMs) <= NMEA_STALE_MS);
  const bool timeFresh = (lastTimeMs != 0UL) && ((unsigned long)(now - lastTimeMs) <= TIME_STALE_MS);
  const bool ggaFresh  = (lastGgaMs  != 0UL) && ((unsigned long)(now - lastGgaMs)  <= GGA_STALE_MS);
  const uint16_t satsVis = getVisibleSatCount();

  // Human-readable diagnostic line (same format as Nano original).
  Serial.print(rxFresh   ? F("RX:OK ")    : F("RX:STALE "));
  Serial.print(timeFresh ? F("TIME:OK ")  : F("TIME:STALE "));
  Serial.print(F("FixQ:"));
  if (ggaFresh) { Serial.print(fixQuality); } else { Serial.print(F("-")); }
  Serial.print(F(" Used/Vis:"));
  if (ggaFresh) { Serial.print(satsUsed); } else { Serial.print(F("-")); }
  Serial.print('/');
  Serial.print(satsVis);
  Serial.print(F(" UTC:")); printUtc();
  Serial.print(F(" Cnt GGA/RMC/GSV/O/BAD/OVF:"));
  Serial.print(cntGGA);          Serial.print('/');
  Serial.print(cntRMC);          Serial.print('/');
  Serial.print(cntGSV);          Serial.print('/');
  Serial.print(cntOther);        Serial.print('/');
  Serial.print(cntChecksumFail); Serial.print('/');
  Serial.print(cntOverflow);
  if (WAIT_FOR_UBX_ACK) {
    Serial.print(F(" UBX A/N/T:"));
    Serial.print(cntUbxAck);     Serial.print('/');
    Serial.print(cntUbxNak);     Serial.print('/');
    Serial.print(cntUbxTimeout);
  }
  Serial.println();

  // Machine-parseable GPSDO status line for the Python GUI tab.
  // Format: "GPSDO: lock=<0|1> sats=<N> clk1=<ok|fail>"
  const bool locked = clk1Ok && (fixQuality > 0) && (satsUsed >= 1);
  Serial.print(F("GPSDO: lock="));
  Serial.print(locked ? 1 : 0);
  Serial.print(F(" sats="));
  Serial.print(ggaFresh ? satsUsed : 0);
  Serial.print(F(" clk1="));
  Serial.println(clk1Ok ? F("ok") : F("fail"));
}

// ---------------------------------------------------------------------------
// SI5351 initialisation
// ---------------------------------------------------------------------------

// Sets up the SI5351 and enables 52 MHz on CLK1.
//
// IMPORTANT: The second argument to si5351.init() must match the actual
// reference frequency applied to the SI5351 XTAL / CLK_IN pin.
//
// 24 MHz is correct when the GPS TIMEPULSE output (configured via CFG-TP5
// above) drives the SI5351 reference input — as in the original design.
//
// If your SI5351 board uses its own 25 MHz crystal (no GPS reference),
// change 24000000UL to 25000000UL here.
static void initSi5351Clock(void)
{
  // Wire must already be started with the correct SDA/SCL pins before this.
  const bool ok = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 24000000UL, 0);
  clk1Ok = ok;
  if (!ok) {
    // Do not halt — report clk1=fail in the status line and continue parsing.
    Serial.println(F("SI5351 init FAILED (check I2C wiring and reference freq)"));
    return;
  }

  // Etherkit library uses 1/100 Hz units: 52 MHz → 52 000 000 * 100
  si5351.set_freq(5200000000ULL, SI5351_CLK1);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_2MA);

  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 1);
  si5351.output_enable(SI5351_CLK2, 0);
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

void setup()
{
  // ── 1. TCXOEN HIGH — absolute first GPIO action ───────────────────────────
  // GP22 must go HIGH before anything else, including Serial.begin().
  // The TCXO on the LoRa1280F27-TCXO module needs ~5 ms to stabilise;
  // the time spent initialising Serial and SI5351 covers this.
  pinMode(PIN_TCXOEN, OUTPUT);
  digitalWrite(PIN_TCXOEN, HIGH);

  // ── 2. Hold SX1280 in reset (Step 1 — SX1280 not yet initialised) ────────
  // GP15 LOW keeps the SX1280 in hardware reset until gpsdoReady is set.
  // In Step 2 this line will be released: digitalWrite(PIN_SX_RESET, HIGH).
  pinMode(PIN_SX_RESET, OUTPUT);
  digitalWrite(PIN_SX_RESET, LOW);

  // ── 3. USB CDC serial for debug and GUI ───────────────────────────────────
  Serial.begin(DEBUG_BAUD);
  delay(50);
  Serial.println();
  Serial.println(F("CT2GQV GPSDO | RP2350 port | CLK1 always ON"));

  // ── 4. I2C0 with explicit pin assignment ──────────────────────────────────
  Wire.setSDA(PIN_SI_SDA);   // GP2
  Wire.setSCL(PIN_SI_SCL);   // GP3
  Wire.begin();

  // ── 5. SI5351 clock — initialised before GPS loop starts ─────────────────
  // Record when GPS startup observation begins so we can overlap delays.
  const unsigned long gpsStartMs = millis();

  initSi5351Clock();
  if (clk1Ok) {
    Serial.println(F("SI5351 ready | 52 MHz on CLK1"));
  }

  // ── 6. GPS hardware UART (UART1 = Serial2) ───────────────────────────────
  // setRX / setTX must be called before begin() in arduino-pico.
  // GP5 is UART1 RX, GP4 is UART1 TX.
  Serial2.setRX(PIN_GPS_RX);   // GP5 ← NEO-7M TX
  Serial2.setTX(PIN_GPS_TX);   // GP4 → NEO-7M RX
  Serial2.begin(GPS_BAUD);
  // No gpsSerial.listen() needed — UART1 is always active on RP2350.

#if SEND_UBX
  // Overlap any remaining GPS boot time with the SI5351 init time above.
  const unsigned long elapsed = (unsigned long)(millis() - gpsStartMs);
  if (elapsed < GPS_BOOT_DELAY_MS) {
    delay(GPS_BOOT_DELAY_MS - elapsed);
  }

  // ── 7. Send GPS receiver configuration ───────────────────────────────────
  sendUbxPacketWithOptionalAck(
    UBX_CFG_TP5_24MHZ, sizeof(UBX_CFG_TP5_24MHZ),
    0x06, 0x31, F("CFG-TP5 24MHz"));
  sendUbxPacketWithOptionalAck(
    UBX_CFG_NAV5_STATIONARY, sizeof(UBX_CFG_NAV5_STATIONARY),
    0x06, 0x24, F("CFG-NAV5 stationary"));
#else
  Serial.println(F("UBX init skipped (SEND_UBX=0)"));
#endif

  Serial.println(F("Parser ready — waiting for GPS lock..."));
}

void loop()
{
  // Service GPS UART first — keeps sentence assembly responsive.
  processIncomingGps();

  // Transition to READY once CLK1 is stable and ≥1 satellite is in use.
  if (!gpsdoReady && clk1Ok && fixQuality > 0 && satsUsed >= 1) {
    gpsdoReady = true;
    Serial.println(F("GPSDO: READY"));
    // Future Step 2: release SX1280 here.
    // digitalWrite(PIN_SX_RESET, HIGH);
  }

  // Periodic status line (human-readable + machine-parseable).
  const unsigned long now = millis();
  if ((unsigned long)(now - lastPrintMs) >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printStatusLine();
  }
}
