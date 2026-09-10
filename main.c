#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/flash.h"
#include "hardware/flash.h"
#include <stddef.h>

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

// TinyUSB
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"

// OLED display
#include "ssd1306.h"

// GPSDO (SI5351 52 MHz clock gen + NEO-7M GPS)
#include "gpsdo.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================== MODE ==================
#define FIXED_POWER_CW_MODE     0
#define FIXED_TX_POWER_DBM      (13)
// ==========================================

// ================== TEST MODE ==================
#define USE_TEST_TONE       0
#define USE_TWO_TONE_TEST   1

#define TEST_TONE_HZ        1000.0f
#define TEST_TONE2_HZ       1900.0f

#define TEST_TONE_AMPL      0.35f
#define TEST_BLOCK_SAMPLES  8000u
// ===============================================

// ================== DITHER SPEED-UP ==================
#define DITHER_SUBSTEPS     4
// ===============================================

// ================== BUFFERING ==================
#define BLOCK_SAMPLES       256u
#define NUM_BLOCKS          8u   // increased from 2 to prevent underruns
// ===============================================

// ================== UNDERRUN DIAGNOSTICS ==================
#define UNDERRUN_LED_ENABLE   1
#define UNDERRUN_LED_PULSE_MS 20u
// ===============================================

// ================== AUDIO SHAPING (default values) ==================
#define AUDIO_ENABLE_BANDPASS       1
#define AUDIO_BP_LO_HZ              50.0f
#define AUDIO_BP_HI_HZ              2700.0f
#define AUDIO_BP_MAX_STAGES         10      // Max stages (compile-time allocation)
#define AUDIO_BP_DEFAULT_STAGES     7       // Default stages (runtime adjustable, 1-10)
// Each stage = 12 dB/octave, so 7 stages = 84 dB/oct, 10 stages = 120 dB/oct

#define AUDIO_ENABLE_EQ             1
#define EQ_LOW_SHELF_HZ             190.0f
#define EQ_LOW_SHELF_DB             (-2.0f)
#define EQ_HIGH_SHELF_HZ            1700.0f
#define EQ_HIGH_SHELF_DB            (13.5f)
// ===============================================

// ================== COMPRESSION (default values) ==================
#define AUDIO_ENABLE_COMPRESSOR     1
#define COMP_THRESHOLD_DB           (-2.5f)
#define COMP_RATIO                  (6.1f)
#define COMP_ATTACK_MS              (41.1f)
#define COMP_RELEASE_MS             (1595.0f)
#define COMP_MAKEUP_DB              (0.0f)
#define COMP_KNEE_DB                (16.5f)
#define COMP_OUTPUT_LIMIT           (0.940f)
// ===============================================

// ================== FIRMWARE VERSION ==================
#define FW_VERSION      "2.2.0-dev"
#define FW_BUILD        __DATE__ " " __TIME__
// ======================================================

// ================== MODULE VARIANT ==================
// Set to 1 if using LoRa1280F27-TCXO module
// Set to 0 if using SX1280 V1.0 (no PA) with SI5351 52 MHz on XTA
#define USE_TCXO_MODULE     0
// ====================================================

// ---------------- Pin mapping ----------------
static const uint32_t PIN_MISO  = 16;
static const uint32_t PIN_MOSI  = 19;
static const uint32_t PIN_SCK   = 18;
static const uint32_t PIN_NSS   = 17;

static const uint32_t PIN_RX_EN = 14;
static const uint32_t PIN_TX_EN = 15;

static const uint32_t PIN_RESET = 20;
static const uint32_t PIN_BUSY  = 21;

#if USE_TCXO_MODULE
static const uint32_t PIN_TCXO_EN = 22;
#endif

// ---------------- OLED I2C pins ----------------
#define OLED_I2C       i2c1
#define OLED_I2C_BAUD  100000  // 100 kHz — reliable with Pico internal pull-ups (~50 kΩ)
static const uint32_t PIN_OLED_SDA = 6;
static const uint32_t PIN_OLED_SCL = 7;

// ---------------- Encoder + buttons ----------------
static const uint32_t PIN_ENC_A   = 2;   // Encoder phase A
static const uint32_t PIN_ENC_B   = 3;   // Encoder phase B
static const uint32_t PIN_ENC_OK  = 10;  // Encoder push button (GP4 occupied by GPSDO UART1 TX)
static const uint32_t PIN_KEY_DIT = 9;   // CW dit paddle (active LOW, internal pull-up)
static const uint32_t PIN_KEY_DAH = 11;  // CW dah paddle (active LOW, internal pull-up; GP10 used by ENC_OK)

// ---------------- SPI config ----------------
#define SX_SPI spi0
static const uint32_t SX_SPI_BAUD = 18000000;

// ---------------- SX1280 opcodes ----------------
#define OPCODE_SET_STANDBY         0x80
#define OPCODE_SET_FS              0xC1   // Lock PLL without starting TX
#define OPCODE_SET_PACKET_TYPE     0x8A
#define OPCODE_SET_RF_FREQUENCY    0x86
#define OPCODE_SET_TX_PARAMS       0x8E
#define OPCODE_SET_TX_CW           0xD1

// ---------------- RF/audio params ----------------
#define BASE_FREQ_HZ        2400400000u
#define WAV_SAMPLE_RATE     8000u

#define PWR_MAX_DBM         (13)
#define PWR_MIN_DBM         (-18)

#define AMP_GAIN            2.9f
#define AMP_MIN_A           0.000002f

#define RAMP_TIME           0xE0     // 20 us

// --- Runtime RF config (adjustable via CDC) ---
// Frequency stored as double for sub-Hz precision; automatically split into PLL steps + fine DSP offset
static volatile double g_target_freq_hz = (double)BASE_FREQ_HZ;
static volatile float g_ppm_correction = 0.0f;
static volatile uint8_t g_cw_test_mode = 0;  // 1 = CW test active (blocks normal Core1 operation)
static volatile int8_t g_tx_power_max_dbm = PWR_MAX_DBM;  // Runtime TX power limit
static volatile uint8_t g_tx_enabled = 1;  // TX enable flag (for GUI TX button)
static volatile uint8_t g_tx_mode = 0;     // 0 = USB (SSB), 1 = CW
static volatile uint8_t g_tune_active = 0; // 1 = TUNE carrier active
static volatile uint8_t g_ptt_key = 0;     // 1 = dit or dah paddle pressed (live)
static volatile uint8_t g_key_dit = 0;     // dit paddle (GP9), debounced
static volatile uint8_t g_key_dah = 0;     // dah paddle (GP11), debounced
static volatile uint8_t g_soft_ptt_key = 0; // Software PTT/KEY via CDC "key 0|1" (GUI CW keyer)

// --- On-device CW keyer (paddles GP9/GP11 → element timing in firmware) ---
enum { KEYER_STRAIGHT = 0, KEYER_IAMBIC_A = 1, KEYER_IAMBIC_B = 2 };
static volatile uint8_t g_cw_mode   = KEYER_IAMBIC_B;
static volatile uint8_t g_cw_wpm    = 18;      // 5..60
static volatile float   g_cw_ratio  = 3.0f;    // dah length in dits, 2.0..5.0
static volatile uint8_t g_keyer_key = 0;       // keyer output: 1 = key down
static void keyer_reset(void);
static const char *keyer_mode_name(uint8_t m);
static void keyer_diag_print(void);

// --- Sidetone (PWM audio on GP12, follows keyer output and TUNE) ---
static volatile uint16_t g_st_hz   = 700;    // 300..1200
static volatile uint8_t  g_st_vol  = 30;     // 0..100
static volatile uint8_t  g_st_test = 0;      // 1 = continuous test tone (not persisted)
static void sidetone_update_params(void);

// --- Audio source: USB from the PC, or the MAX4466 microphone on GP26 (ADC0) ---
enum { AUDIO_SRC_PC = 0, AUDIO_SRC_MIC = 1 };
static volatile uint8_t g_audio_src = AUDIO_SRC_PC;
static volatile float   g_mic_gain  = 10.0f;   // 1..50, applied after DC removal
static volatile float   g_mic_gate  = 0.02f;   // 0..0.5, noise gate threshold on the gained signal
static const char *audio_src_name(uint8_t s);
static void mic_diag_print(void);
// GPS gate: 1 = TX only when gpsdo_is_ready() (default), 0 = override for bench tests.
// Deliberately not persisted — resets to enforced on every boot.
static volatile uint8_t g_gps_gate = 1;

static inline bool tx_allowed(void) {
    return gpsdo_is_ready() || !g_gps_gate;
}

// --- Persistent config (last flash sector) ---
static volatile uint8_t  g_persist_dirty       = 0;  // something worth saving changed
static volatile uint32_t g_persist_dirty_since = 0;  // ms of last change
static uint8_t           g_persist_loaded      = 0;  // 1 = boot config came from flash

// The main loop autosaves after an idle period, so knob turns don't erase flash repeatedly.
static inline void persist_mark_dirty(void) {
    g_persist_dirty = 1;
    g_persist_dirty_since = to_ms_since_boot(get_absolute_time());
}

// --- Encoder UI state ---
// The frequency stays on top; the lower half shows one of three pages.
// Pages 0 and 1 hold 2x3 tiles (short click = browse), page 2 is GPS info.
// A long press (500 ms) switches to the next page.
typedef enum {
    UI_PARAM_MODE = 0,   // USB PC / USB MIC / CW
    UI_PARAM_TUNE,       // TUNE on/off
    UI_PARAM_WPM,        // keyer speed
    UI_PARAM_TX,         // TX on/off
    UI_PARAM_VOL,        // sidetone volume
    UI_PARAM_PWR,        // TX power dBm
    UI_PARAM_KEYER,      // straight / iambic A / iambic B
    UI_PARAM_RATIO,      // dah length in dits
    UI_PARAM_TONE,       // sidetone pitch
    UI_PARAM_MICGAIN,
    UI_PARAM_MICGATE
} ui_param_t;

typedef enum {
    UI_STATE_IDLE = 0,   // encoder = frequency; click = browse tiles; hold = next page
    UI_STATE_BROWSE,     // frame around a tile, rotate moves cursor; hold = back
    UI_STATE_EDITING     // tile inverted, rotate adjusts value; hold = back
} ui_state_t;

#define UI_PAGES         3
#define UI_TILES         6                    // 2 columns x 3 rows on tile pages
#define UI_TIMEOUT_MS    5000                 // BROWSE/EDITING fall back to IDLE
#define UI_LONG_PRESS_MS 500

static const ui_param_t k_page_tiles[2][UI_TILES] = {
    { UI_PARAM_MODE,  UI_PARAM_TUNE,  UI_PARAM_WPM, UI_PARAM_TX,   UI_PARAM_VOL,     UI_PARAM_PWR     },
    { UI_PARAM_KEYER, UI_PARAM_RATIO, UI_PARAM_WPM, UI_PARAM_TONE, UI_PARAM_MICGAIN, UI_PARAM_MICGATE },
};

static volatile ui_state_t g_ui_state = UI_STATE_IDLE;
static volatile uint8_t    g_ui_page = 0;                  // 0 operating, 1 CW/audio, 2 GPS info
static volatile uint8_t    g_ui_cursor_idx = 0;            // tile index on the current page
static volatile ui_param_t g_ui_cursor = UI_PARAM_MODE;    // param under the cursor (derived)
static volatile ui_param_t g_ui_editing = UI_PARAM_MODE;   // which param is being edited
static volatile uint32_t   g_ui_last_activity_ms = 0;      // for auto-timeout

// --- Hilbert ---
#define HILBERT_TAPS        247

// --- PLL step ---
static const float PLL_STEP_HZ =
    (float)(52000000.0 / (double)(1u << 18)); // ~198.364 Hz

#define F_OFF_LIMIT_HZ      3500.0f
#define SILENCE_SECONDS     2u

#define GATE_A_REF          0.01f   // Noise gate threshold - higher with compressor
#define GATE_SHAPE          1

#define IQ_GAIN_CORR        1.00f
#define IQ_PHASE_CORR_DEG   0.0f

// ---------------- Command buffer ----------------
typedef struct {
    int32_t  freq_steps;
    int8_t   p_dbm;
    uint8_t  tx_on;
} sample_cmd_t;

static sample_cmd_t g_blocks[NUM_BLOCKS][BLOCK_SAMPLES];

static volatile uint32_t g_prod_block = 0;
static volatile uint32_t g_cons_block = 0;
static volatile uint8_t  g_block_ready[NUM_BLOCKS] = {0};
static volatile uint32_t g_underruns = 0;
static volatile uint8_t  g_core1_start = 0; 

// ==========================================================
// USB AUDIO IN (from PC) -> ringbuffer -> resampler to 8k mono
// ==========================================================

typedef struct { int16_t l, r; } stereo16_t;

// Power-of-two
#ifndef USB_RB_FRAMES
#define USB_RB_FRAMES 8192u
#endif

#if (USB_RB_FRAMES & (USB_RB_FRAMES - 1u)) != 0
#error "USB_RB_FRAMES must be power-of-two"
#endif

static stereo16_t g_usb_rb[USB_RB_FRAMES];
static volatile uint32_t g_usb_w = 0;
static volatile uint32_t g_usb_r = 0;

static volatile uint32_t g_usb_sample_rate_hz = 48000u; // current host SR

static inline uint32_t usb_rb_next(uint32_t x) { return (x + 1u) & (USB_RB_FRAMES - 1u); }

static inline bool usb_rb_push(stereo16_t s) {
    uint32_t w = g_usb_w;
    uint32_t n = usb_rb_next(w);
    if (n == g_usb_r) return false; // full
    g_usb_rb[w] = s;
    g_usb_w = n;
    return true;
}

static inline bool usb_rb_pop(stereo16_t *out) {
    uint32_t r = g_usb_r;
    if (r == g_usb_w) return false; // empty
    *out = g_usb_rb[r];
    g_usb_r = usb_rb_next(r);
    return true;
}

static inline int16_t clamp16(int32_t x) {
    if (x > 32767) return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

// Read host audio (PCM16LE stereo), push to ringbuffer
static void usb_audio_pump(void);

// Resampler: host SR stereo -> 8 kHz mono
// With smoothed adaptive rate to prevent buffer overflow and pitch artifacts
static int16_t usb_audio_get_mono_8k(void) {
    static uint32_t src_rate = 48000u;
    static uint32_t base_step_q16 = 0;
    static uint32_t smooth_step_q16 = 0;  // Smoothed step for gradual changes
    static uint32_t phase_q16 = 0;

    static stereo16_t s0 = {0,0};
    static stereo16_t s1 = {0,0};
    static stereo16_t s2 = {0,0};  // Extra sample for cubic interpolation
    static stereo16_t sm1 = {0,0}; // s[-1] for cubic
    static bool primed = false;

    uint32_t sr = g_usb_sample_rate_hz;
    if (!sr) sr = 48000u;

    if (sr != src_rate || base_step_q16 == 0) {
        src_rate = sr;
        base_step_q16 = (uint32_t)(((uint64_t)src_rate << 16) / (uint32_t)WAV_SAMPLE_RATE);
        smooth_step_q16 = base_step_q16;
    }

    // *** Adaptive rate with heavy smoothing ***
    uint32_t usb_w = g_usb_w;
    uint32_t usb_r = g_usb_r;
    uint32_t fill = (usb_w >= usb_r) ? (usb_w - usb_r) : (USB_RB_FRAMES - usb_r + usb_w);
    
    const uint32_t target_fill = USB_RB_FRAMES / 2;
    uint32_t target_step = base_step_q16;
    
    if (fill > target_fill) {
        uint32_t excess = fill - target_fill;
        uint32_t adj = (base_step_q16 * excess) / (USB_RB_FRAMES * 10);
        target_step = base_step_q16 + adj;
    } else if (fill < target_fill) {
        uint32_t deficit = target_fill - fill;
        uint32_t adj = (base_step_q16 * deficit) / (USB_RB_FRAMES * 10);
        target_step = base_step_q16 - adj;
    }
    
    // Heavy smoothing: move only 1/256 of the way to target each sample
    // This prevents audible pitch wobble
    if (smooth_step_q16 < target_step) {
        uint32_t diff = target_step - smooth_step_q16;
        smooth_step_q16 += (diff >> 8) + 1;
        if (smooth_step_q16 > target_step) smooth_step_q16 = target_step;
    } else if (smooth_step_q16 > target_step) {
        uint32_t diff = smooth_step_q16 - target_step;
        smooth_step_q16 -= (diff >> 8) + 1;
        if (smooth_step_q16 < target_step) smooth_step_q16 = target_step;
    }

    if (!primed) {
        if (!usb_rb_pop(&sm1)) sm1 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s0)) s0 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s1)) s1 = (stereo16_t){0,0};
        if (!usb_rb_pop(&s2)) s2 = (stereo16_t){0,0};
        phase_q16 = 0;
        primed = true;
    }

    phase_q16 += smooth_step_q16;
    while (phase_q16 >= (1u << 16)) {
        phase_q16 -= (1u << 16);
        sm1 = s0;
        s0 = s1;
        s1 = s2;
        if (!usb_rb_pop(&s2)) s2 = s1;  // Hold last value if empty
    }

    // Cubic Hermite interpolation for smoother audio
    float t = (float)phase_q16 / 65536.0f;
    float t2 = t * t;
    float t3 = t2 * t;
    
    // Hermite basis functions
    float h00 = 2*t3 - 3*t2 + 1;
    float h10 = t3 - 2*t2 + t;
    float h01 = -2*t3 + 3*t2;
    float h11 = t3 - t2;
    
    // Left channel
    float m0_l = (float)(s1.l - sm1.l) * 0.5f;
    float m1_l = (float)(s2.l - s0.l) * 0.5f;
    float l = h00 * s0.l + h10 * m0_l + h01 * s1.l + h11 * m1_l;
    
    // Right channel
    float m0_r = (float)(s1.r - sm1.r) * 0.5f;
    float m1_r = (float)(s2.r - s0.r) * 0.5f;
    float r = h00 * s0.r + h10 * m0_r + h01 * s1.r + h11 * m1_r;

    float mono = (l + r) * 0.5f;
    return clamp16((int32_t)mono);
}

// ==========================================================
// TinyUSB UAC1 control callbacks (sampling freq + feature unit)
// ==========================================================

// Minimal feature unit state (żeby Windows nie marudził)
static uint8_t  g_mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];
static int16_t  g_volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1]; // 1/256 dB

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    (void)rhport;
    (void)p_request;
    return true;
}

// EP sampling freq (UAC1)
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff) {
    (void)rhport;
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ &&
        p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
        TU_VERIFY(p_request->wLength == 3);
        uint32_t sr = tu_unaligned_read32(pBuff) & 0x00FFFFFF;
        if (sr) g_usb_sample_rate_hz = sr;
        return true;
    }
    return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);

    if (ctrlSel == AUDIO10_EP_CTRL_SAMPLING_FREQ &&
        p_request->bRequest == AUDIO10_CS_REQ_GET_CUR) {
        uint32_t sr = g_usb_sample_rate_hz;
        uint8_t freq[3] = {
            (uint8_t)(sr & 0xFF),
            (uint8_t)((sr >> 8) & 0xFF),
            (uint8_t)((sr >> 16) & 0xFF)
        };
        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, freq, sizeof(freq));
    }
    return false;
}

// Feature Unit (mute/volume) – UAC1 entity callbacks
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
    (void)rhport;

    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;

    switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
                TU_VERIFY(p_request->wLength == 1);
                g_mute[channelNum] = buf[0];
                return true;
            }
            return false;

        case AUDIO10_FU_CTRL_VOLUME:
            if (p_request->bRequest == AUDIO10_CS_REQ_SET_CUR) {
                TU_VERIFY(p_request->wLength == 2);
                g_volume[channelNum] = (int16_t) tu_unaligned_read16(buf); // 1/256 dB
                return true;
            }
            return false;

        default:
            return false;
    }
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel    = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID   = TU_U16_HIGH(p_request->wIndex);

    if (entityID != UAC1_ENTITY_FEATURE_UNIT) return false;

    switch (ctrlSel) {
        case AUDIO10_FU_CTRL_MUTE:
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &g_mute[channelNum], 1);

        case AUDIO10_FU_CTRL_VOLUME:
            switch (p_request->bRequest) {
                case AUDIO10_CS_REQ_GET_CUR:
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &g_volume[channelNum], sizeof(g_volume[channelNum]));
                case AUDIO10_CS_REQ_GET_MIN: {
                    int16_t min = (int16_t)(-90 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &min, sizeof(min));
                }
                case AUDIO10_CS_REQ_GET_MAX: {
                    int16_t max = (int16_t)(0 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &max, sizeof(max));
                }
                case AUDIO10_CS_REQ_GET_RES: {
                    int16_t res = (int16_t)(1 * 256);
                    return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &res, sizeof(res));
                }
                default:
                    return false;
            }

        default:
            return false;
    }
}

#if CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
// Feedback method (dla UAC1 speaker+feedback)
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param) {
    (void)func_id;
    (void)alt_itf;
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = g_usb_sample_rate_hz ? g_usb_sample_rate_hz : 48000u;
}
#endif

// ==========================================================
// SX1280 low-level radio I/O (CORE1 ONLY)
// ==========================================================
static inline void cs_select(void)   { gpio_put(PIN_NSS, 0); }
static inline void cs_deselect(void) { gpio_put(PIN_NSS, 1); }

static inline void sx_wait_busy(void) {
    while (gpio_get(PIN_BUSY)) { tight_loop_contents(); }
}

// Forward declarations for CDC functions
static void cdc_printf(const char *fmt, ...);
static void cdc_write_str(const char *s);

static void sx_write_cmd(uint8_t opcode, const uint8_t *params, size_t len) {
    sx_wait_busy();
    cs_select();
    spi_write_blocking(SX_SPI, &opcode, 1);
    if (len && params) {
        spi_write_blocking(SX_SPI, params, len);
    }
    cs_deselect();
    sx_wait_busy();
}

static inline void sx_set_standby_rc(void) {
    uint8_t cfg = 0x00;  // STDBY_RC
    sx_write_cmd(OPCODE_SET_STANDBY, &cfg, 1);
}

static inline void sx_set_standby_xosc(void) {
    uint8_t cfg = 0x01;  // STDBY_XOSC - required for TCXO module
    sx_write_cmd(OPCODE_SET_STANDBY, &cfg, 1);
}

static inline void sx_set_packet_type_gfsk(void) {
    uint8_t pt = 0x00; // GFSK
    sx_write_cmd(OPCODE_SET_PACKET_TYPE, &pt, 1);
}

static inline uint8_t sx_encode_power_dbm(int32_t dbm) {
    if (dbm > 13)  dbm = 13;
    if (dbm < -18) dbm = -18;
    return (uint8_t)(dbm + 18); // 0..31
}

static inline void sx_set_tx_params_dbm(int32_t power_dbm) {
    uint8_t pwr = sx_encode_power_dbm(power_dbm);
    uint8_t p[2] = { pwr, RAMP_TIME };
    sx_write_cmd(OPCODE_SET_TX_PARAMS, p, 2);
}

static inline void sx_start_tx_continuous_wave(void) {
    sx_write_cmd(OPCODE_SET_TX_CW, NULL, 0);
}

static inline void sx_set_rf_frequency_steps(uint32_t steps) {
    uint8_t p[3] = {
        (uint8_t)(steps >> 16),
        (uint8_t)(steps >> 8),
        (uint8_t)(steps)
    };
    sx_write_cmd(OPCODE_SET_RF_FREQUENCY, p, 3);
}

static inline uint32_t hz_to_steps(uint32_t freq_hz) {
    return (uint32_t)((double)freq_hz / (double)PLL_STEP_HZ);
}

// Calculate corrected frequency with PPM
static inline double get_corrected_freq_hz(void) {
    return g_target_freq_hz * (1.0 + (double)g_ppm_correction / 1000000.0);
}

// Get base PLL steps (integer part)
static inline uint32_t get_base_steps(void) {
    double corrected_hz = get_corrected_freq_hz();
    return (uint32_t)(corrected_hz / (double)PLL_STEP_HZ);
}

// Get fine tune offset in Hz (fractional part that PLL can't reach)
static inline float get_fine_tune_hz(void) {
    double corrected_hz = get_corrected_freq_hz();
    uint32_t base_steps = (uint32_t)(corrected_hz / (double)PLL_STEP_HZ);
    double base_hz = (double)base_steps * (double)PLL_STEP_HZ;
    return (float)(corrected_hz - base_hz);
}

// Get SX1280 status byte
static uint8_t sx_get_status(void) {
    sx_wait_busy();
    cs_select();
    uint8_t cmd = 0xC0;  // GetStatus opcode
    uint8_t status;
    spi_write_blocking(SX_SPI, &cmd, 1);
    spi_read_blocking(SX_SPI, 0x00, &status, 1);
    cs_deselect();
    return status;
}

// Diagnostic: print SX1280 state via CDC
static void sx_print_diag(void) {
#if CFG_TUD_CDC
    uint8_t status = sx_get_status();
    uint8_t mode = (status >> 5) & 0x07;
    
    const char *mode_str = "UNKNOWN";
    switch (mode) {
        case 2: mode_str = "STDBY_RC"; break;
        case 3: mode_str = "STDBY_XOSC"; break;
        case 4: mode_str = "FS"; break;
        case 5: mode_str = "RX"; break;
        case 6: mode_str = "TX"; break;
    }
    
    cdc_printf("\r\n=== SX1280 Diagnostics ===\r\n");
    cdc_printf("Status: 0x%02X (mode=%d: %s)\r\n", status, mode, mode_str);
    cdc_printf("BUSY pin: %d\r\n", gpio_get(PIN_BUSY));
    cdc_printf("TX_EN pin: %d\r\n", gpio_get(PIN_TX_EN));
    cdc_printf("RX_EN pin: %d\r\n", gpio_get(PIN_RX_EN));
#if USE_TCXO_MODULE
    cdc_printf("TCXO_EN pin: %d\r\n", gpio_get(PIN_TCXO_EN));
#endif
    cdc_printf("Base freq: %lu Hz\r\n", (unsigned long)BASE_FREQ_HZ);
    cdc_printf("TX power max: %d dBm\r\n", g_tx_power_max_dbm);
    
    // Buffer diagnostics
    uint32_t prod = g_prod_block;
    uint32_t cons = g_cons_block;
    uint32_t ready_count = 0;
    for (uint32_t i = 0; i < NUM_BLOCKS; i++) {
        if (g_block_ready[i]) ready_count++;
    }
    cdc_printf("Blocks: prod=%lu cons=%lu ready=%lu/%lu\r\n", 
               (unsigned long)prod, (unsigned long)cons, 
               (unsigned long)ready_count, (unsigned long)NUM_BLOCKS);
    cdc_printf("Underruns: %lu\r\n", (unsigned long)g_underruns);
    
    // USB audio buffer
    uint32_t usb_w = g_usb_w;
    uint32_t usb_r = g_usb_r;
    uint32_t usb_fill = (usb_w >= usb_r) ? (usb_w - usb_r) : (USB_RB_FRAMES - usb_r + usb_w);
    cdc_printf("USB ringbuf: %lu/%lu frames\r\n", (unsigned long)usb_fill, (unsigned long)USB_RB_FRAMES);
    keyer_diag_print();
    cdc_printf("==========================\r\n");
#endif
}

// Apply current freq + power to SX1280 while in CW/TUNE mode.
// Uses fractional-step dithering for sub-PLL-step precision (same as SSB).
// Call repeatedly from carrier_poll() — each call advances the dither accumulator.
static float g_cw_freq_acc = 0.0f;  // Fractional step accumulator for CW dithering
static int8_t g_cr_pwr = PWR_MIN_DBM; // Power currently applied to the carrier (ramped by carrier_poll)

static void tune_apply_settings(void) {
    uint32_t base = get_base_steps();
    float fine_hz = get_fine_tune_hz();

    // Dither: accumulate fractional PLL step, toggle +1 when it overflows
    float want_frac = fine_hz / PLL_STEP_HZ;
    int32_t Nf = (int32_t)floorf(want_frac);
    float ffrac = want_frac - (float)Nf;

    g_cw_freq_acc += ffrac;
    int32_t chosen = Nf;
    if (g_cw_freq_acc >= 1.0f) {
        chosen = Nf + 1;
        g_cw_freq_acc -= 1.0f;
    }

    sx_set_rf_frequency_steps((uint32_t)((int32_t)base + chosen));
    sx_set_tx_params_dbm(g_cr_pwr);
}

// Pump USB while waiting (keep USB alive during short delays)
static void usb_aware_delay_ms(uint32_t ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < ms) {
        tud_task();
    }
}

// Start CW/TUNE carrier (Core0 only).
// PRECONDITION: g_cw_test_mode must already be 1 and Core1 must be idle.
static void sx_start_carrier(void) {
    // Ensure TCXO is on (pump USB while waiting)
#if USE_TCXO_MODULE
    gpio_put(PIN_TCXO_EN, 1);
    usb_aware_delay_ms(5);
#endif

    // Full clean init: standby -> packet type -> freq -> power -> CW
#if USE_TCXO_MODULE
    sx_set_standby_xosc();
#else
    sx_set_standby_rc();
#endif

    sx_set_packet_type_gfsk();

    uint32_t steps = get_base_steps();
    sx_set_rf_frequency_steps(steps);
    // Start at minimum power; carrier_poll() ramps up in 1 dB steps (click-free keying).
    g_cr_pwr = PWR_MIN_DBM;
    sx_set_tx_params_dbm(g_cr_pwr);

    gpio_put(PIN_TX_EN, 1);
    gpio_put(PIN_RX_EN, 0);

    sx_start_tx_continuous_wave();
    usb_aware_delay_ms(2);

    // Safety: re-apply frequency AFTER CW start
    sx_set_rf_frequency_steps(steps);
}

// Stop CW carrier and restore for SSB (Core0 only)
static void sx_stop_carrier(void) {
#if USE_TCXO_MODULE
    sx_set_standby_xosc();
#else
    sx_set_standby_rc();
#endif

    // Re-initialize radio for normal SSB operation
    sx_set_packet_type_gfsk();
    sx_set_rf_frequency_steps(get_base_steps());
    sx_set_tx_params_dbm((int32_t)g_tx_power_max_dbm);

    gpio_put(PIN_TX_EN, 1);
    gpio_put(PIN_RX_EN, 0);

    // Resume normal Core1 operation
    g_cw_test_mode = 0;
    __compiler_memory_barrier();
}

#define CW_ARM_WAIT_MS  35  // Time to wait for Core1 to finish SPI (> 1 block = 32ms)
// Key-click suppression: power ramps 1 dB per step, 31 steps (-18..+13 dBm) ≈ 5 ms.
#define CW_RAMP_STEP_US 160

// Start CW/TUNE transmission (legacy wrapper with CDC logging)
// This is a blocking call (used by CDC "cw" command and TUNE encoder action).
// It idles Core1, waits (USB-aware), then starts carrier.
static void sx_test_cw(void) {
#if CFG_TUD_CDC
    cdc_printf("\r\n*** Starting CW/TUNE ***\r\n");
#endif
    // Idle Core1 and wait (USB-aware, not blocking)
    if (!g_cw_test_mode) {
        g_cw_test_mode = 1;
        __compiler_memory_barrier();
        usb_aware_delay_ms(CW_ARM_WAIT_MS);
    }
    sx_start_carrier();
#if CFG_TUD_CDC
    uint8_t status = sx_get_status();
    cdc_printf("TUNE: freq=%.1f Hz, steps=%lu, pwr=%d dBm, status=0x%02X\r\n",
               get_corrected_freq_hz(), (unsigned long)get_base_steps(),
               g_tx_power_max_dbm, status);
#endif
}

// Stop CW transmission and restore normal SSB mode (legacy wrapper)
static void sx_stop_cw(void) {
    sx_stop_carrier();
#if CFG_TUD_CDC
    cdc_printf("TX stopped, radio re-initialized for SSB\r\n");
#endif
}

// ==========================================================
// Unified carrier state machine (runs on Core0 in polling loop)
//
// Inputs:  g_tx_mode (0=USB, 1=CW), g_tune_active, g_ptt_key
// Outputs: g_cw_test_mode, SPI carrier on/off
//
// Logic:
//   need_idle  = (g_tx_mode==1) || g_tune_active     → Core1 must idle
//   need_carrier = g_tune_active || (g_tx_mode==1 && g_ptt_key)  → CW on
//
// States:
//   IDLE    → need_idle? set g_cw_test_mode=1, go ARMING
//   ARMING  → wait 35ms for Core1, go ARMED
//   ARMED   → need_carrier? sx_start_carrier → CARRIER_ON
//   CARRIER_ON → !need_carrier? standby → ARMED
//   any     → !need_idle? stop carrier if on, clear g_cw_test_mode → IDLE
// ==========================================================
typedef enum {
    CR_ST_IDLE = 0,     // Normal SSB mode — Core1 owns SPI
    CR_ST_ARMING,       // g_cw_test_mode=1 set, waiting for Core1 to idle
    CR_ST_ARMED,        // Core1 is idle, no carrier (standby)
    CR_ST_CARRIER_ON    // CW carrier active
} carrier_state_t;

static carrier_state_t g_cr_state = CR_ST_IDLE;
static uint32_t        g_cr_arm_start_ms = 0;
static uint64_t        g_cr_ramp_next_us = 0;

static void carrier_poll(void) {
    bool mode_cw     = (g_tx_mode == 1);
    bool tune        = (bool)g_tune_active;
    if (!mode_cw && g_keyer_key) keyer_reset();   // never leave a key hanging after a mode switch
    bool key         = (bool)(g_keyer_key | g_soft_ptt_key);

    bool need_idle    = mode_cw || tune;
    bool need_carrier = tx_allowed() && (tune || (mode_cw && key));

    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (g_cr_state) {

    case CR_ST_IDLE:
        if (need_idle) {
            g_cw_test_mode = 1;
            __compiler_memory_barrier();
            g_cr_arm_start_ms = now;
            g_cr_state = CR_ST_ARMING;
        }
        break;

    case CR_ST_ARMING:
        if (!need_idle) {
            // No longer need idle — cancel
            g_cw_test_mode = 0;
            __compiler_memory_barrier();
            g_cr_state = CR_ST_IDLE;
            break;
        }
        if ((now - g_cr_arm_start_ms) >= CW_ARM_WAIT_MS) {
            // Core1 is now idle — force radio to known standby state
            // (Core1 may have left it in TX from SSB)
#if USE_TCXO_MODULE
            sx_set_standby_xosc();
#else
            sx_set_standby_rc();
#endif
            g_cr_state = CR_ST_ARMED;
        }
        break;

    case CR_ST_ARMED:
        if (!need_idle) {
            // Return to SSB — restore Core1
            g_cw_test_mode = 0;
            __compiler_memory_barrier();
            g_cr_state = CR_ST_IDLE;
            break;
        }
        if (need_carrier) {
            g_cw_freq_acc = 0.0f;  // Reset dither accumulator
            sx_start_carrier();    // starts at PWR_MIN_DBM
            g_cr_ramp_next_us = 0; // first ramp step immediately
            g_cr_state = CR_ST_CARRIER_ON;
        }
        break;

    case CR_ST_CARRIER_ON: {
        if (!need_idle) {
            // Leaving CW/TUNE entirely — full stop + restore SSB
            sx_stop_carrier();   // clears g_cw_test_mode
            g_cr_state = CR_ST_IDLE;
            break;
        }
        uint64_t now_us = time_us_64();

        // Power envelope: ramp toward max while keyed, toward min on key-up.
        // A key-down during the tail simply reverses direction — no re-init.
        int8_t target = need_carrier ? g_tx_power_max_dbm : (int8_t)PWR_MIN_DBM;
        if (g_cr_pwr != target && now_us >= g_cr_ramp_next_us) {
            g_cr_pwr += (target > g_cr_pwr) ? 1 : -1;
            sx_set_tx_params_dbm(g_cr_pwr);
            g_cr_ramp_next_us = now_us + CW_RAMP_STEP_US;
        }
        if (!need_carrier && g_cr_pwr <= PWR_MIN_DBM) {
            // Tail finished — carrier off but stay armed (key released, or TUNE off in CW mode)
#if USE_TCXO_MODULE
            sx_set_standby_xosc();
#else
            sx_set_standby_rc();
#endif
            g_cr_state = CR_ST_ARMED;
            break;
        }
        // Carrier on — dither freq at ~8 kHz (125 µs), same rate as SSB
        static uint64_t next_dither_us = 0;
        if (now_us >= next_dither_us) {
            tune_apply_settings();
            next_dither_us = now_us + 125;  // 125 µs = 8 kHz
        }
        break;
    }
    }
}

// ==========================================================
// DSP blocks (biquads + compressor + hilbert)
// ==========================================================
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} biquad_t;

static inline void biquad_reset(biquad_t *q) { q->z1 = 0.0f; q->z2 = 0.0f; }

static inline float biquad_process(biquad_t *q, float x) {
    float y = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * y + q->z2;
    q->z2 = q->b2 * x - q->a2 * y;
    return y;
}

static void biquad_init_lowpass_bw2(biquad_t *q, float fc, float fs) {
    const float K  = tanf((float)M_PI * fc / fs);
    const float K2 = K * K;
    const float s2 = 1.41421356f;
    const float norm = 1.0f / (1.0f + s2 * K + K2);

    q->b0 = K2 * norm;
    q->b1 = 2.0f * q->b0;
    q->b2 = q->b0;

    q->a1 = 2.0f * (K2 - 1.0f) * norm;
    q->a2 = (1.0f - s2 * K + K2) * norm;

    biquad_reset(q);
}

static void biquad_init_highpass_bw2(biquad_t *q, float fc, float fs) {
    const float K  = tanf((float)M_PI * fc / fs);
    const float K2 = K * K;
    const float s2 = 1.41421356f;
    const float norm = 1.0f / (1.0f + s2 * K + K2);

    q->b0 = 1.0f * norm;
    q->b1 = -2.0f * q->b0;
    q->b2 = q->b0;

    q->a1 = 2.0f * (K2 - 1.0f) * norm;
    q->a2 = (1.0f - s2 * K + K2) * norm;

    biquad_reset(q);
}

static void biquad_init_low_shelf(biquad_t *q, float fc, float fs, float gain_db) {
    const float A = powf(10.0f, gain_db / 40.0f);
    const float w0 = 2.0f * (float)M_PI * fc / fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw * 0.5f * 1.41421356f;

    float b0 =    A*((A+1.0f) - (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha);
    float b1 =  2.0f*A*((A-1.0f) - (A+1.0f)*cw);
    float b2 =    A*((A+1.0f) - (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha);
    float a0 =        (A+1.0f) + (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha;
    float a1 =   -2.0f*((A-1.0f) + (A+1.0f)*cw);
    float a2 =        (A+1.0f) + (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha;

    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    biquad_reset(q);
}

static void biquad_init_high_shelf(biquad_t *q, float fc, float fs, float gain_db) {
    const float A = powf(10.0f, gain_db / 40.0f);
    const float w0 = 2.0f * (float)M_PI * fc / fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw * 0.5f * 1.41421356f;

    float b0 =    A*((A+1.0f) + (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha);
    float b1 = -2.0f*A*((A-1.0f) + (A+1.0f)*cw);
    float b2 =    A*((A+1.0f) + (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha);
    float a0 =        (A+1.0f) - (A-1.0f)*cw + 2.0f*sqrtf(A)*alpha;
    float a1 =    2.0f*((A-1.0f) - (A+1.0f)*cw);
    float a2 =        (A+1.0f) - (A-1.0f)*cw - 2.0f*sqrtf(A)*alpha;

    q->b0 = b0 / a0; q->b1 = b1 / a0; q->b2 = b2 / a0;
    q->a1 = a1 / a0; q->a2 = a2 / a0;
    biquad_reset(q);
}

typedef struct {
    float env;
    float a_att, a_rel;
    float thr_db, ratio, makeup_lin, knee_db;
} compressor_t;

static inline float compressor_gain_db(const compressor_t *c, float in_db) {
    const float thr = c->thr_db;
    const float r = c->ratio;

    if (c->knee_db <= 0.0f) {
        if (in_db <= thr) return 0.0f;
        float out_db = thr + (in_db - thr) / r;
        return out_db - in_db;
    }

    const float k = c->knee_db;
    const float x0 = thr - k * 0.5f;
    const float x1 = thr + k * 0.5f;

    if (in_db <= x0) return 0.0f;
    if (in_db >= x1) {
        float out_db = thr + (in_db - thr) / r;
        return out_db - in_db;
    }

    const float t = (in_db - x0) / (x1 - x0);
    float out1 = thr + (x1 - thr) / r;
    float g1 = out1 - x1;
    return g1 * t * t;
}

static inline float compressor_process(compressor_t *c, float x) {
    float ax = fabsf(x);
    if (ax > c->env) c->env = c->a_att * c->env + (1.0f - c->a_att) * ax;
    else            c->env = c->a_rel * c->env + (1.0f - c->a_rel) * ax;

    float env = fmaxf(c->env, 1e-8f);
    float in_db = 20.0f * log10f(env);

    float g_db = compressor_gain_db(c, in_db);
    float g_lin = powf(10.0f, g_db / 20.0f) * c->makeup_lin;

    return x * g_lin;
}

// ==========================================================
// Runtime-configurable DSP settings (over USB CDC)
// ==========================================================
typedef struct {
    uint8_t enable_bandpass;
    uint8_t enable_eq;
    uint8_t enable_comp;

    float bp_lo_hz;
    float bp_hi_hz;
    uint8_t bp_stages;      // 1-10, each stage = 12 dB/octave

    float eq_low_hz;
    float eq_low_db;
    float eq_high_hz;
    float eq_high_db;

    float comp_thr_db;
    float comp_ratio;
    float comp_attack_ms;
    float comp_release_ms;
    float comp_makeup_db;
    float comp_knee_db;
    float comp_out_limit;

    float amp_gain;
    float amp_min_a;
} audio_cfg_t;

#define AUDIO_CFG_DEFAULTS { \
    .enable_bandpass = AUDIO_ENABLE_BANDPASS, \
    .enable_eq       = AUDIO_ENABLE_EQ, \
    .enable_comp     = AUDIO_ENABLE_COMPRESSOR, \
    .bp_lo_hz  = AUDIO_BP_LO_HZ, \
    .bp_hi_hz  = AUDIO_BP_HI_HZ, \
    .bp_stages = AUDIO_BP_DEFAULT_STAGES, \
    .eq_low_hz  = EQ_LOW_SHELF_HZ, \
    .eq_low_db  = EQ_LOW_SHELF_DB, \
    .eq_high_hz = EQ_HIGH_SHELF_HZ, \
    .eq_high_db = EQ_HIGH_SHELF_DB, \
    .comp_thr_db     = COMP_THRESHOLD_DB, \
    .comp_ratio      = COMP_RATIO, \
    .comp_attack_ms  = COMP_ATTACK_MS, \
    .comp_release_ms = COMP_RELEASE_MS, \
    .comp_makeup_db  = COMP_MAKEUP_DB, \
    .comp_knee_db    = COMP_KNEE_DB, \
    .comp_out_limit  = COMP_OUTPUT_LIMIT, \
    .amp_gain  = AMP_GAIN, \
    .amp_min_a = AMP_MIN_A, \
}

static const audio_cfg_t k_cfg_defaults = AUDIO_CFG_DEFAULTS;
static volatile audio_cfg_t g_cfg = AUDIO_CFG_DEFAULTS;
static volatile uint8_t g_cfg_dirty = 1;

static void compressor_reconfig(compressor_t *c, float fs, const audio_cfg_t *cfg) {
    c->env = 0.0f;

    const float att_s = cfg->comp_attack_ms  * 0.001f;
    const float rel_s = cfg->comp_release_ms * 0.001f;
    c->a_att = expf(-1.0f / (fmaxf(att_s, 1e-4f) * fs));
    c->a_rel = expf(-1.0f / (fmaxf(rel_s, 1e-4f) * fs));

    c->thr_db = cfg->comp_thr_db;
    c->ratio  = fmaxf(cfg->comp_ratio, 1.0f);
    c->makeup_lin = powf(10.0f, cfg->comp_makeup_db / 20.0f);
    c->knee_db    = fmaxf(cfg->comp_knee_db, 0.0f);
}

static void cfg_sanitize(audio_cfg_t *c, float fs) {
    if (c->bp_lo_hz < 50.0f) c->bp_lo_hz = 50.0f;
    float max_hi = fs * 0.45f;
    if (c->bp_hi_hz > max_hi) c->bp_hi_hz = max_hi;
    if (c->bp_hi_hz <= c->bp_lo_hz + 50.0f) c->bp_hi_hz = c->bp_lo_hz + 50.0f;

    if (c->eq_low_hz < 50.0f) c->eq_low_hz = 50.0f;
    if (c->eq_low_hz > fs * 0.45f) c->eq_low_hz = fs * 0.45f;

    if (c->eq_high_hz < 50.0f) c->eq_high_hz = 50.0f;
    if (c->eq_high_hz > fs * 0.45f) c->eq_high_hz = fs * 0.45f;

    if (c->comp_ratio < 1.0f) c->comp_ratio = 1.0f;
    if (c->comp_attack_ms < 0.1f) c->comp_attack_ms = 0.1f;
    if (c->comp_release_ms < 1.0f) c->comp_release_ms = 1.0f;

    if (c->comp_out_limit < 0.05f) c->comp_out_limit = 0.05f;
    if (c->comp_out_limit > 0.999f) c->comp_out_limit = 0.999f;

    if (c->amp_gain < 0.01f) c->amp_gain = 0.01f;
    if (c->amp_min_a < 1e-9f) c->amp_min_a = 1e-9f;

    // Clamp bp_stages to valid range
    if (c->bp_stages < 1) c->bp_stages = 1;
    if (c->bp_stages > AUDIO_BP_MAX_STAGES) c->bp_stages = AUDIO_BP_MAX_STAGES;
}

static void apply_cfg_if_dirty(float Fs,
                              biquad_t *bp_hpf, biquad_t *bp_lpf,
                              biquad_t *eq_low, biquad_t *eq_high,
                              compressor_t *comp,
                              audio_cfg_t *out_cfg)
{
    if (!g_cfg_dirty) return;

    audio_cfg_t tmp;
    __compiler_memory_barrier();
    memcpy(&tmp, (const void*)&g_cfg, sizeof(tmp));
    __compiler_memory_barrier();

    cfg_sanitize(&tmp, Fs);

#if AUDIO_BP_MAX_STAGES
    for (int i = 0; i < AUDIO_BP_MAX_STAGES; i++) {
        biquad_init_highpass_bw2(&bp_hpf[i], tmp.bp_lo_hz, Fs);
        biquad_init_lowpass_bw2 (&bp_lpf[i], tmp.bp_hi_hz, Fs);
    }
#else
    (void)bp_hpf; (void)bp_lpf;
#endif

    biquad_init_low_shelf (eq_low,  tmp.eq_low_hz,  Fs, tmp.eq_low_db);
    biquad_init_high_shelf(eq_high, tmp.eq_high_hz, Fs, tmp.eq_high_db);

    compressor_reconfig(comp, Fs, &tmp);

    *out_cfg = tmp;

    __compiler_memory_barrier();
    g_cfg_dirty = 0;
    __compiler_memory_barrier();
}
// ==========================================================
// Simple USB CDC command interface (enabled only if CDC exists)
// ==========================================================
static int streqi(const char *a, const char *b) {
    if (!a || !b) return 0;
    while (*a && *b) {
        char ca = *a++, cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
        if (ca != cb) return 0;
    }
    return (*a == 0 && *b == 0);
}

static void cdc_write_str(const char *s) {
#if CFG_TUD_CDC
    if (!tud_cdc_connected()) return;
    tud_cdc_write_str(s);
    tud_cdc_write_flush();
#else
    (void)s;
#endif
}

static void cdc_printf(const char *fmt, ...) {
#if CFG_TUD_CDC
    if (!tud_cdc_connected()) return;
    char b[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    tud_cdc_write_str(b);
    tud_cdc_write_flush();
#else
    (void)fmt;
#endif
}

static bool parse_bool(const char *s, uint8_t *out) {
    if (!s) return false;
    if (streqi(s, "1") || streqi(s, "on") || streqi(s, "true"))  { *out = 1; return true; }
    if (streqi(s, "0") || streqi(s, "off")|| streqi(s, "false")) { *out = 0; return true; }
    return false;
}

static bool parse_f(const char *s, float *out) {
    if (!s) return false;
    char *e = NULL;
    float v = strtof(s, &e);
    if (e == s) return false;
    *out = v;
    return true;
}

// Format large Hz value as "XXXXXXXXXX.X" into caller's buffer.
// Needed because newlib-nano's printf uses scientific notation for big doubles.
static void fmt_freq(char *buf, size_t len, double hz) {
    uint64_t i = (uint64_t)hz;
    uint32_t f = (uint32_t)((hz - (double)i) * 10.0 + 0.5);
    if (f >= 10) { i++; f = 0; }
    snprintf(buf, len, "%llu.%u", (unsigned long long)i, (unsigned)f);
}

static void cfg_print(void) {
    audio_cfg_t c;
    __compiler_memory_barrier();
    memcpy(&c, (const void*)&g_cfg, sizeof(c));
    __compiler_memory_barrier();

    double corrected = get_corrected_freq_hz();
    float fine = get_fine_tune_hz();

    char freq_str[24], corr_str[24];
    fmt_freq(freq_str, sizeof(freq_str), g_target_freq_hz);
    fmt_freq(corr_str, sizeof(corr_str), corrected);

    cdc_printf(
        "CFG:\r\n"
        "  fw=" FW_VERSION "  built=" FW_BUILD "\r\n"
        "  freq=%s Hz (target)  ppm=%.3f  tx=%s  txpwr=%d dBm\r\n"
        "  mode=%s  tune=%s  gps=%s  gpsgate=%s  config=%s\r\n"
        "  keyer=%s  wpm=%u  ratio=%.1f  sidetone=%u Hz  vol=%u%%\r\n"
        "  src=%s  mic_gain=%.1f  mic_gate=%.3f\r\n"
        "  corrected=%s Hz  base_steps=%lu  fine=%.1f Hz (auto)\r\n",
        freq_str, g_ppm_correction, g_tx_enabled ? "ON" : "OFF", g_tx_power_max_dbm,
        g_tx_mode ? "CW" : "USB", g_tune_active ? "ON" : "OFF",
        gpsdo_is_ready() ? "ready" : "wait", g_gps_gate ? "ON" : "OFF",
        g_persist_dirty ? "unsaved" : (g_persist_loaded ? "flash" : "defaults"),
        keyer_mode_name(g_cw_mode), (unsigned)g_cw_wpm, (double)g_cw_ratio,
        (unsigned)g_st_hz, (unsigned)g_st_vol,
        audio_src_name(g_audio_src), (double)g_mic_gain, (double)g_mic_gate,
        corr_str, (unsigned long)get_base_steps(), fine);
    cdc_printf(
        "  enable bp=%u eq=%u comp=%u\r\n"
        "  bp_lo=%.1f bp_hi=%.1f bp_stages=%u (%u dB/oct)\r\n"
        "  eq_low_hz=%.1f eq_low_db=%.1f\r\n"
        "  eq_high_hz=%.1f eq_high_db=%.1f\r\n"
        "  comp_thr=%.1f ratio=%.2f att=%.2fms rel=%.2fms makeup=%.1f knee=%.1f outlim=%.3f\r\n"
        "  amp_gain=%.3f amp_min_a=%.9f\r\n",
        c.enable_bandpass, c.enable_eq, c.enable_comp,
        c.bp_lo_hz, c.bp_hi_hz, c.bp_stages, c.bp_stages * 12,
        c.eq_low_hz, c.eq_low_db,
        c.eq_high_hz, c.eq_high_db,
        c.comp_thr_db, c.comp_ratio, c.comp_attack_ms, c.comp_release_ms, c.comp_makeup_db, c.comp_knee_db, c.comp_out_limit,
        c.amp_gain, c.amp_min_a
    );
}

static void cmd_help(void) {
    cdc_write_str(
        "Commands:\r\n"
        "  help\r\n"
        "  get\r\n"
        "  version       - firmware version and build date\r\n"
        "  diag          - show SX1280 status\r\n"
        "  gpsdo         - GPSDO status line\r\n"
        "  gpsgate 0|1   - 1: TX only with GPS UTC (default); 0: override for bench tests\r\n"
        "  save          - write freq/ppm/txpwr/mode/DSP to flash now (autosaves 5 s after idle)\r\n"
        "  defaults      - restore compile-time defaults and save\r\n"
        "  src pc|mic    - audio source: USB from PC, or MAX4466 mic on GP26\r\n"
        "  mic [gain <1..50> | gate <0..0.5>] - mic gain and noise-gate threshold\r\n"
        "  keyer [wpm <5..60> | mode <straight|a|b> | ratio <2..5>] - on-device paddle keyer\r\n"
        "        [tone <300..1200> | vol <0..100> | test 0|1]      - sidetone on GP12 (PWM)\r\n"
        "                  decoder events: !K e=.|-  !K c=<char>  !K w\r\n"
        "  tx 0|1        - enable/disable TX (SSB modulation)\r\n"
        "  mode usb|cw   - set modulation mode\r\n"
        "  tune 0|1      - toggle TUNE carrier\r\n"
        "  cw            - start CW test transmission (carrier)\r\n"
        "  stop          - stop CW/tune transmission\r\n"
        "  key 0|1       - software PTT/KEY for GUI CW keyer\r\n"
        "  cwkey          - enter CW keyer mode (GUI use)\r\n"
        "  hang <ms>      - hang time (compatibility, no-op in v2)\r\n"
        "  freq <Hz>     - set frequency with sub-Hz precision (e.g. freq 2400100050.5)\r\n"
        "  ppm <value>   - set PPM correction (e.g. ppm -0.5)\r\n"
        "  txpwr <-18..13> - set max TX power in dBm\r\n"
        "  enable <bp|eq|comp> <0|1|on|off>\r\n"
        "  set bp_lo <Hz>\r\n"
        "  set bp_hi <Hz>\r\n"
        "  set bp_stages <1-10>  (filter steepness: 12dB/oct per stage)\r\n"
        "  set eq_low_hz <Hz>\r\n"
        "  set eq_low_db <dB>\r\n"
        "  set eq_high_hz <Hz>\r\n"
        "  set eq_high_db <dB>\r\n"
        "  set comp_thr <dB>\r\n"
        "  set comp_ratio <R>\r\n"
        "  set comp_att <ms>\r\n"
        "  set comp_rel <ms>\r\n"
        "  set comp_makeup <dB>\r\n"
        "  set comp_knee <dB>\r\n"
        "  set comp_outlim <0..1>\r\n"
        "  set amp_gain <float>\r\n"
        "  set amp_min_a <float>\r\n"
        "\r\n"
        "Frequency is automatically split into PLL steps + fine DSP offset.\r\n"
    );
}

static void cfg_commit(const audio_cfg_t *c) {
    __compiler_memory_barrier();
    memcpy((void*)&g_cfg, c, sizeof(*c));
    g_cfg_dirty = 1;
    __compiler_memory_barrier();
    persist_mark_dirty();
}

// ==========================================================
// Persistent configuration in the last 4 KB flash sector
// ==========================================================
// Layout: one flash page. Anything not listed here (TX enable, TUNE,
// GPS gate, soft key) is deliberately volatile and resets at boot.
#define CFG_FLASH_OFFSET   (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define CFG_MAGIC          0x51303130u   // 'Q010'
#define CFG_VERSION        3u            // bump whenever persist_cfg_t layout changes

typedef struct __attribute__((packed)) {
    uint32_t    magic;
    uint32_t    version;
    double      freq_hz;
    float       ppm_correction;
    int8_t      tx_power_dbm;
    uint8_t     tx_mode;         // 0 = USB, 1 = CW
    uint8_t     cw_mode;         // KEYER_*
    uint8_t     cw_wpm;
    float       cw_ratio;
    uint16_t    st_hz;           // sidetone
    uint8_t     st_vol;
    uint8_t     audio_src;       // AUDIO_SRC_*
    float       mic_gain;
    float       mic_gate;
    audio_cfg_t dsp;
    uint32_t    crc32;           // over everything above
} persist_cfg_t;

_Static_assert(sizeof(persist_cfg_t) <= FLASH_PAGE_SIZE,
               "persist_cfg_t must fit in one flash page");

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc ^= 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static inline uint32_t persist_cfg_crc(const persist_cfg_t *c) {
    return crc32_update(0, (const uint8_t *)c, offsetof(persist_cfg_t, crc32));
}

static void persist_collect(persist_cfg_t *c) {
    memset(c, 0, sizeof(*c));
    c->magic          = CFG_MAGIC;
    c->version        = CFG_VERSION;
    c->freq_hz        = g_target_freq_hz;
    c->ppm_correction = g_ppm_correction;
    c->tx_power_dbm   = g_tx_power_max_dbm;
    c->tx_mode        = g_tx_mode;
    c->cw_mode        = g_cw_mode;
    c->cw_wpm         = g_cw_wpm;
    c->cw_ratio       = g_cw_ratio;
    c->st_hz          = g_st_hz;
    c->st_vol         = g_st_vol;
    c->audio_src      = g_audio_src;
    c->mic_gain       = g_mic_gain;
    c->mic_gate       = g_mic_gate;
    __compiler_memory_barrier();
    memcpy(&c->dsp, (const void *)&g_cfg, sizeof(c->dsp));
    __compiler_memory_barrier();
    c->crc32          = persist_cfg_crc(c);
}

static void persist_apply(const persist_cfg_t *c) {
    double f = c->freq_hz;
    if (!(f >= 2400000000.0 && f <= 2500000000.0)) f = (double)BASE_FREQ_HZ;  // also rejects NaN
    g_target_freq_hz = f;

    float ppm = c->ppm_correction;
    if (!(ppm >= -100.0f && ppm <= 100.0f)) ppm = 0.0f;
    g_ppm_correction = ppm;

    int8_t p = c->tx_power_dbm;
    if (p < PWR_MIN_DBM) p = PWR_MIN_DBM;
    if (p > PWR_MAX_DBM) p = PWR_MAX_DBM;
    g_tx_power_max_dbm = p;

    g_tx_mode = (c->tx_mode == 1) ? 1 : 0;

    g_cw_mode = (c->cw_mode <= KEYER_IAMBIC_B) ? c->cw_mode : KEYER_IAMBIC_B;
    g_cw_wpm  = (c->cw_wpm >= 5 && c->cw_wpm <= 60) ? c->cw_wpm : 18;
    float r = c->cw_ratio;
    g_cw_ratio = (r >= 2.0f && r <= 5.0f) ? r : 3.0f;   // also rejects NaN

    g_st_hz  = (c->st_hz >= 300 && c->st_hz <= 1200) ? c->st_hz : 700;
    g_st_vol = (c->st_vol <= 100) ? c->st_vol : 30;
    sidetone_update_params();

    g_audio_src = (c->audio_src == AUDIO_SRC_MIC) ? AUDIO_SRC_MIC : AUDIO_SRC_PC;
    float mg = c->mic_gain, mt = c->mic_gate;
    g_mic_gain = (mg >= 1.0f && mg <= 50.0f) ? mg : 10.0f;
    g_mic_gate = (mt >= 0.0f && mt <= 0.5f)  ? mt : 0.02f;

    audio_cfg_t d = c->dsp;
    cfg_sanitize(&d, (float)WAV_SAMPLE_RATE);
    cfg_commit(&d);
}

static bool persist_load(void) {
    const persist_cfg_t *p = (const persist_cfg_t *)(XIP_BASE + CFG_FLASH_OFFSET);
    if (p->magic != CFG_MAGIC)          return false;
    if (p->version != CFG_VERSION)      return false;
    if (persist_cfg_crc(p) != p->crc32) return false;
    persist_apply(p);
    g_persist_dirty  = 0;   // cfg_commit() marked dirty; a fresh load is clean by definition
    g_persist_loaded = 1;
    return true;
}

// Runs with IRQs off on Core0 and Core1 parked by flash_safe_execute().
// Must stay in RAM: XIP is unavailable while flash is being erased.
static void __not_in_flash_func(persist_flash_op)(void *param) {
    flash_range_erase(CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CFG_FLASH_OFFSET, (const uint8_t *)param, FLASH_PAGE_SIZE);
}

// ~20 ms erase + ~1 ms program. Core1's SPI loop is stalled meanwhile, so the
// SX1280 holds its last state for that long — hence autosave only when idle.
static bool persist_save_now(void) {
    persist_cfg_t c;
    persist_collect(&c);

    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &c, sizeof(c));

    int r = flash_safe_execute(persist_flash_op, page, 100);
    if (r == PICO_OK) {
        g_persist_dirty = 0;
        return true;
    }
    // Don't retry back-to-back: each failed attempt costs up to 100 ms.
    g_persist_dirty_since = to_ms_since_boot(get_absolute_time());
    return false;
}

// Periodic status push to CDC for GUI synchronization.
// Sends a compact line that the GUI can parse to update its widgets.
// Only sends when something changed, at most every 250 ms.
#if CFG_TUD_CDC
static void cdc_status_push_ex(bool force) {
    static uint32_t last_push_ms = 0;
    static uint8_t  last_mode = 0xFF;
    static uint8_t  last_tune = 0xFF;
    static uint8_t  last_tx   = 0xFF;
    static int8_t   last_pwr  = 127;
    static float    last_ppm  = 9999.0f;
    static double   last_freq = 0.0;
    static uint8_t  last_gps  = 0xFF;
    static uint8_t  last_gate = 0xFF;
    static uint8_t  last_dirty = 0xFF;
    static uint8_t  last_kwpm  = 0;
    static uint8_t  last_kmode = 0xFF;
    static uint8_t  last_key   = 0xFF;
    static uint8_t  last_pdl   = 0xFF;
    static uint16_t last_sthz  = 0;
    static uint8_t  last_stvol = 0xFF;
    static uint8_t  last_src   = 0xFF;

    if (!tud_cdc_connected()) return;

    uint8_t  cur_mode = g_tx_mode;
    uint8_t  cur_tune = g_tune_active;
    uint8_t  cur_tx   = g_tx_enabled;
    int8_t   cur_pwr  = g_tx_power_max_dbm;
    float    cur_ppm  = g_ppm_correction;
    double   cur_freq = (double)g_target_freq_hz;
    uint8_t  cur_gps  = gpsdo_is_ready() ? 1 : 0;
    uint8_t  cur_gate = g_gps_gate;
    uint8_t  cur_dirty = g_persist_dirty;
    uint8_t  cur_kwpm  = g_cw_wpm;
    uint8_t  cur_kmode = g_cw_mode;
    uint8_t  cur_key   = (g_keyer_key | g_soft_ptt_key) ? 1 : 0;
    uint8_t  cur_pdl   = (uint8_t)(g_key_dit | (g_key_dah << 1));
    uint16_t cur_sthz  = g_st_hz;
    uint8_t  cur_stvol = g_st_vol;
    uint8_t  cur_src   = g_audio_src;

    if (!force) {
        // Check if anything changed
        bool changed = (cur_mode != last_mode) || (cur_tune != last_tune) ||
                       (cur_tx != last_tx) || (cur_pwr != last_pwr) ||
                       (cur_ppm != last_ppm) || (cur_freq != last_freq) ||
                       (cur_gps != last_gps) || (cur_gate != last_gate) ||
                       (cur_dirty != last_dirty) ||
                       (cur_kwpm != last_kwpm) || (cur_kmode != last_kmode) ||
                       (cur_key != last_key) || (cur_pdl != last_pdl) ||
                       (cur_sthz != last_sthz) || (cur_stvol != last_stvol) ||
                       (cur_src != last_src);

        if (!changed) return;

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - last_push_ms) < 100) return;
    }

    // Format freq with fmt_freq to avoid newlib-nano scientific notation
    char freq_str[24];
    fmt_freq(freq_str, sizeof(freq_str), cur_freq);

    // PPM: format as fixed-point (4 decimals) to avoid float formatting issues
    int ppm_neg = (cur_ppm < 0.0f);
    float ppm_abs = ppm_neg ? -cur_ppm : cur_ppm;
    uint32_t ppm_int = (uint32_t)ppm_abs;
    uint32_t ppm_frac = (uint32_t)((ppm_abs - (float)ppm_int) * 10000.0f + 0.5f);
    if (ppm_frac >= 10000) { ppm_int++; ppm_frac = 0; }

    char status_buf[160];
    snprintf(status_buf, sizeof(status_buf),
             "!S mode=%u tune=%u tx=%u pwr=%d ppm=%s%lu.%04lu freq=%s gps=%u gate=%u dirty=%u kwpm=%u kmode=%u key=%u pdl=%u sthz=%u stvol=%u src=%u\r\n",
             cur_mode, cur_tune, cur_tx, cur_pwr,
             ppm_neg ? "-" : "", (unsigned long)ppm_int, (unsigned long)ppm_frac,
             freq_str, cur_gps, cur_gate, cur_dirty, cur_kwpm, cur_kmode, cur_key, cur_pdl,
             (unsigned)cur_sthz, (unsigned)cur_stvol, (unsigned)cur_src);
    cdc_write_str(status_buf);

    last_mode = cur_mode;
    last_tune = cur_tune;
    last_tx   = cur_tx;
    last_pwr  = cur_pwr;
    last_ppm  = cur_ppm;
    last_freq = cur_freq;
    last_gps  = cur_gps;
    last_gate = cur_gate;
    last_dirty = cur_dirty;
    last_kwpm  = cur_kwpm;
    last_kmode = cur_kmode;
    last_key   = cur_key;
    last_pdl   = cur_pdl;
    last_sthz  = cur_sthz;
    last_stvol = cur_stvol;
    last_src   = cur_src;
    last_push_ms = to_ms_since_boot(get_absolute_time());
}

static inline void cdc_status_push(void) { cdc_status_push_ex(false); }
#endif

static void cdc_handle_line(char *line) {
    char *argv[6] = {0};
    int argc = 0;

    for (char *t = strtok(line, " \t\r\n"); t && argc < 6; t = strtok(NULL, " \t\r\n")) {
        argv[argc++] = t;
    }
    if (argc == 0) return;

    if (streqi(argv[0], "help")) { cmd_help(); return; }
    if (streqi(argv[0], "get"))  { cfg_print(); return; }
    if (streqi(argv[0], "status")) { cdc_status_push_ex(true); return; }
    if (streqi(argv[0], "diag")) { sx_print_diag(); return; }
    if (streqi(argv[0], "gpsdo")) {
        char gbuf[128];
        gpsdo_format_status(gbuf, sizeof(gbuf));
        cdc_write_str(gbuf);
        return;
    }
    if (streqi(argv[0], "save")) {
        if (persist_save_now()) cdc_write_str("OK saved to flash\r\n");
        else                    cdc_write_str("ERR: flash save failed (core lockout timeout?)\r\n");
        return;
    }
    if (streqi(argv[0], "defaults")) {
        g_target_freq_hz   = (double)BASE_FREQ_HZ;
        g_ppm_correction   = 0.0f;
        g_tx_power_max_dbm = PWR_MAX_DBM;
        g_tx_mode          = 0;
        g_cw_mode          = KEYER_IAMBIC_B;
        g_cw_wpm           = 18;
        g_cw_ratio         = 3.0f;
        g_st_hz            = 700;
        g_st_vol           = 30;
        sidetone_update_params();
        g_audio_src        = AUDIO_SRC_PC;
        g_mic_gain         = 10.0f;
        g_mic_gate         = 0.02f;
        keyer_reset();
        cfg_commit(&k_cfg_defaults);
        if (g_tune_active) tune_apply_settings();
        if (persist_save_now()) cdc_write_str("OK defaults restored and saved\r\n");
        else                    cdc_write_str("ERR: defaults restored in RAM, flash save failed\r\n");
        cfg_print();
        return;
    }
    // Audio source: src pc|mic
    if (streqi(argv[0], "src")) {
        if (argc >= 2) {
            uint8_t s;
            if      (streqi(argv[1], "pc") || streqi(argv[1], "usb")) s = AUDIO_SRC_PC;
            else if (streqi(argv[1], "mic") || streqi(argv[1], "adc")) s = AUDIO_SRC_MIC;
            else { cdc_write_str("ERR: src pc|mic\r\n"); return; }
            if (s != g_audio_src) { g_audio_src = s; persist_mark_dirty(); }
        }
        cdc_printf("OK src=%s\r\n", audio_src_name(g_audio_src));
        return;
    }
    // Microphone: mic [gain <1..50> | gate <0..0.5>]
    if (streqi(argv[0], "mic")) {
        if (argc >= 3 && streqi(argv[1], "gain")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 1.0f || v > 50.0f) { cdc_write_str("ERR: mic gain 1..50\r\n"); return; }
            g_mic_gain = v; persist_mark_dirty();
        } else if (argc >= 3 && streqi(argv[1], "gate")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 0.0f || v > 0.5f) { cdc_write_str("ERR: mic gate 0..0.5\r\n"); return; }
            g_mic_gate = v; persist_mark_dirty();
        } else if (argc >= 2) {
            cdc_write_str("ERR: mic [gain <1..50> | gate <0..0.5>]\r\n");
            return;
        }
        cdc_printf("OK mic gain=%.1f gate=%.3f\r\n", (double)g_mic_gain, (double)g_mic_gate);
        mic_diag_print();
        return;
    }
    if (streqi(argv[0], "version") || streqi(argv[0], "ver")) {
        cdc_write_str("FW: " FW_VERSION " built " FW_BUILD "\r\n");
        return;
    }
    if (streqi(argv[0], "gpsgate") && argc >= 2) {
        uint8_t v;
        if (!parse_bool(argv[1], &v)) { cdc_write_str("ERR: gpsgate 0|1\r\n"); return; }
        g_gps_gate = v;
        if (v) cdc_write_str("OK gpsgate=ON (TX requires GPS UTC)\r\n");
        else   cdc_write_str("OK gpsgate=OFF — WARNING: TX without GPS discipline, "
                             "up to +/-6 kHz error at 2.4 GHz. Not persisted.\r\n");
        return;
    }
    if (streqi(argv[0], "cw")) {
        g_tune_active = 1;
        cdc_printf("OK tune=ON (carrier_poll handles SPI)\r\n");
        if (!tx_allowed()) cdc_write_str("WARN: carrier blocked until GPS UTC valid (gpsgate 0 to override)\r\n");
        return;
    }
    if (streqi(argv[0], "stop")) { g_tune_active = 0; g_soft_ptt_key = 0; keyer_reset(); cdc_printf("OK tune=OFF\r\n"); return; }

    // On-device keyer: keyer [wpm <5..60> | mode <straight|a|b> | ratio <2.0..5.0>]
    if (streqi(argv[0], "keyer")) {
        if (argc >= 3 && streqi(argv[1], "wpm")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 5.0f || v > 60.0f) { cdc_write_str("ERR: keyer wpm 5..60\r\n"); return; }
            g_cw_wpm = (uint8_t)v;
            persist_mark_dirty();
        } else if (argc >= 3 && streqi(argv[1], "mode")) {
            uint8_t m;
            if      (streqi(argv[2], "straight") || streqi(argv[2], "0")) m = KEYER_STRAIGHT;
            else if (streqi(argv[2], "a")        || streqi(argv[2], "1")) m = KEYER_IAMBIC_A;
            else if (streqi(argv[2], "b")        || streqi(argv[2], "2")) m = KEYER_IAMBIC_B;
            else { cdc_write_str("ERR: keyer mode straight|a|b\r\n"); return; }
            if (m != g_cw_mode) { g_cw_mode = m; keyer_reset(); persist_mark_dirty(); }
        } else if (argc >= 3 && streqi(argv[1], "ratio")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 2.0f || v > 5.0f) { cdc_write_str("ERR: keyer ratio 2.0..5.0\r\n"); return; }
            g_cw_ratio = v;
            persist_mark_dirty();
        } else if (argc >= 3 && streqi(argv[1], "tone")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 300.0f || v > 1200.0f) { cdc_write_str("ERR: keyer tone 300..1200 Hz\r\n"); return; }
            g_st_hz = (uint16_t)v;
            sidetone_update_params();
            persist_mark_dirty();
        } else if (argc >= 3 && streqi(argv[1], "vol")) {
            float v;
            if (!parse_f(argv[2], &v) || v < 0.0f || v > 100.0f) { cdc_write_str("ERR: keyer vol 0..100\r\n"); return; }
            g_st_vol = (uint8_t)v;
            sidetone_update_params();
            persist_mark_dirty();
        } else if (argc >= 3 && streqi(argv[1], "test")) {
            uint8_t v;
            if (!parse_bool(argv[2], &v)) { cdc_write_str("ERR: keyer test 0|1\r\n"); return; }
            g_st_test = v;   // continuous sidetone for checking the headphone wiring
        } else if (argc >= 2) {
            cdc_write_str("ERR: keyer [wpm <n> | mode <straight|a|b> | ratio <r> | tone <Hz> | vol <0..100> | test 0|1]\r\n");
            return;
        }
        cdc_printf("OK keyer mode=%s wpm=%u ratio=%.1f tone=%u vol=%u%s\r\n",
                   keyer_mode_name(g_cw_mode), (unsigned)g_cw_wpm, (double)g_cw_ratio,
                   (unsigned)g_st_hz, (unsigned)g_st_vol, g_st_test ? " test=ON" : "");
        return;
    }

    // Software PTT/KEY for GUI CW keyer: key 0|1
    if (streqi(argv[0], "key") && argc >= 2) {
        g_soft_ptt_key = (argv[1][0] == '1') ? 1 : 0;
        cdc_printf("OK key=%u\r\n", (unsigned)g_soft_ptt_key);
        return;
    }
    // cwkey: enter CW mode and arm keyer (alias for mode cw, used by GUI keyer)
    if (streqi(argv[0], "cwkey")) {
        if (g_tx_mode != 1) persist_mark_dirty();
        g_tx_mode = 1;
        g_soft_ptt_key = 0;
        cdc_printf("OK cwkey mode=CW\r\n");
        return;
    }
    // hang <ms>: accepted for GUI compatibility (hang managed by carrier state machine)
    if (streqi(argv[0], "hang") && argc >= 2) {
        cdc_printf("OK hang accepted\r\n");
        return;
    }

    // Mode: mode usb|cw
    if (streqi(argv[0], "mode") && argc >= 2) {
        if (streqi(argv[1], "usb") || streqi(argv[1], "ssb")) {
            if (g_tx_mode != 0) persist_mark_dirty();
            g_tx_mode = 0;
            cdc_printf("OK mode=USB\r\n");
        } else if (streqi(argv[1], "cw")) {
            if (g_tx_mode != 1) persist_mark_dirty();
            g_tx_mode = 1;
            cdc_printf("OK mode=CW\r\n");
        } else {
            cdc_write_str("ERR: mode usb|cw\r\n");
        }
        return;
    }

    // Tune carrier: tune 0|1
    if (streqi(argv[0], "tune") && argc >= 2) {
        uint8_t v;
        if (!parse_bool(argv[1], &v)) {
            cdc_write_str("ERR: tune 0|1|on|off\r\n");
            return;
        }
        g_tune_active = v;
        // carrier_poll() will handle SPI transitions
        cdc_printf("OK tune=%s\r\n", g_tune_active ? "ON" : "OFF");
        if (v && !tx_allowed()) cdc_write_str("WARN: carrier blocked until GPS UTC valid (gpsgate 0 to override)\r\n");
        return;
    }

    // TX enable/disable: tx 0|1
    if (streqi(argv[0], "tx") && argc >= 2) {
        uint8_t v;
        if (!parse_bool(argv[1], &v)) { 
            cdc_write_str("ERR: tx 0|1|on|off\r\n"); 
            return; 
        }
        g_tx_enabled = v;
        cdc_printf("OK tx=%s\r\n", g_tx_enabled ? "ON" : "OFF");
        if (v && !tx_allowed()) cdc_write_str("WARN: TX blocked until GPS UTC valid (gpsgate 0 to override)\r\n");
        return;
    }

    // Frequency command: freq <Hz> (supports decimal for sub-Hz precision)
    if (streqi(argv[0], "freq") && argc >= 2) {
        char *e = NULL;
        double f = strtod(argv[1], &e);
        if (e == argv[1] || f < 2400000000.0 || f > 2500000000.0) {
            cdc_write_str("ERR: freq must be 2400000000-2500000000 Hz\r\n");
            return;
        }
        g_target_freq_hz = f;
        persist_mark_dirty();
        double corrected = get_corrected_freq_hz();
        float fine = get_fine_tune_hz();
        cdc_printf("OK freq=%.1f Hz (corrected=%.1f, steps=%lu, fine=%.1f Hz)\r\n", 
                   g_target_freq_hz, corrected,
                   (unsigned long)get_base_steps(), fine);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    // PPM correction command: ppm <value>
    if (streqi(argv[0], "ppm") && argc >= 2) {
        float ppm;
        if (!parse_f(argv[1], &ppm)) { 
            cdc_write_str("ERR: bad PPM value\r\n"); 
            return; 
        }
        if (ppm < -100.0f || ppm > 100.0f) {
            cdc_write_str("ERR: ppm must be -100 to +100\r\n");
            return;
        }
        g_ppm_correction = ppm;
        persist_mark_dirty();
        double corrected = get_corrected_freq_hz();
        float fine = get_fine_tune_hz();
        cdc_printf("OK ppm=%.3f (corrected=%.1f Hz, steps=%lu, fine=%.1f Hz)\r\n", 
                   g_ppm_correction, corrected,
                   (unsigned long)get_base_steps(), fine);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    // TX power command: txpwr <-18..13>
    if (streqi(argv[0], "txpwr") && argc >= 2) {
        float pwr;
        if (!parse_f(argv[1], &pwr)) { 
            cdc_write_str("ERR: bad txpwr value\r\n"); 
            return; 
        }
        if (pwr < (float)PWR_MIN_DBM) pwr = (float)PWR_MIN_DBM;
        if (pwr > (float)PWR_MAX_DBM) pwr = (float)PWR_MAX_DBM;
        g_tx_power_max_dbm = (int8_t)pwr;
        persist_mark_dirty();
        cdc_printf("OK txpwr=%d dBm\r\n", g_tx_power_max_dbm);
        if (g_tune_active) tune_apply_settings();
        return;
    }

    audio_cfg_t c;
    __compiler_memory_barrier();
    memcpy(&c, (const void*)&g_cfg, sizeof(c));
    __compiler_memory_barrier();

    if (streqi(argv[0], "enable") && argc >= 3) {
        uint8_t v;
        if (!parse_bool(argv[2], &v)) { cdc_write_str("ERR: bad bool\r\n"); return; }

        if (streqi(argv[1], "bp")) c.enable_bandpass = v;
        else if (streqi(argv[1], "eq")) c.enable_eq = v;
        else if (streqi(argv[1], "comp")) c.enable_comp = v;
        else { cdc_write_str("ERR: enable bp|eq|comp\r\n"); return; }

        cfg_commit(&c);
        cdc_write_str("OK\r\n");
        return;
    }

    if (streqi(argv[0], "set") && argc >= 3) {
        float f;
        if (!parse_f(argv[2], &f)) { cdc_write_str("ERR: bad number\r\n"); return; }

        if      (streqi(argv[1], "bp_lo"))       c.bp_lo_hz = f;
        else if (streqi(argv[1], "bp_hi"))       c.bp_hi_hz = f;
        else if (streqi(argv[1], "bp_stages"))   c.bp_stages = (uint8_t)f;
        else if (streqi(argv[1], "eq_low_hz"))   c.eq_low_hz = f;
        else if (streqi(argv[1], "eq_low_db"))   c.eq_low_db = f;
        else if (streqi(argv[1], "eq_high_hz"))  c.eq_high_hz = f;
        else if (streqi(argv[1], "eq_high_db"))  c.eq_high_db = f;
        else if (streqi(argv[1], "comp_thr"))    c.comp_thr_db = f;
        else if (streqi(argv[1], "comp_ratio"))  c.comp_ratio = f;
        else if (streqi(argv[1], "comp_att"))    c.comp_attack_ms = f;
        else if (streqi(argv[1], "comp_rel"))    c.comp_release_ms = f;
        else if (streqi(argv[1], "comp_makeup")) c.comp_makeup_db = f;
        else if (streqi(argv[1], "comp_knee"))   c.comp_knee_db = f;
        else if (streqi(argv[1], "comp_outlim")) c.comp_out_limit = f;
        else if (streqi(argv[1], "amp_gain"))    c.amp_gain = f;
        else if (streqi(argv[1], "amp_min_a"))   c.amp_min_a = f;
        else { cdc_write_str("ERR: unknown key\r\n"); return; }

        cfg_commit(&c);
        cdc_write_str("OK\r\n");
        return;
    }

    cdc_write_str("ERR: unknown command (type 'help')\r\n");
}

static void cdc_task(void) {
#if CFG_TUD_CDC
    static char line[128];
    static uint32_t pos = 0;

    if (!tud_cdc_connected()) return;

    while (tud_cdc_available()) {
        char ch = (char)tud_cdc_read_char();

        if (ch == '\r' || ch == '\n') {
            if (pos > 0) {
                line[pos] = 0;
                cdc_handle_line(line);
                pos = 0;
            }
        } else {
            if (pos < sizeof(line) - 1) {
                line[pos++] = ch;
            }
        }
    }
#endif
}

// ==========================================================
// OLED display — simple text, DMA transfer
// ==========================================================
#define QO100_DOWNLINK_OFFSET_HZ  8089500000.0

// Helper: right-justify a 1x string ending at x_right edge
static void draw_string_right(int x_right, int page, const char *str) {
    int len = 0;
    const char *p = str;
    while (*p++) len++;
    int x = x_right - len * 6;
    ssd1306_draw_string(x, page, str);
}

// Draw a 2x-height arrow-up glyph into framebuf at (x, page..page+1)
static void draw_arrow_up_2x(int x, int page) {
    static const uint8_t src[] = {0x04, 0x02, 0x7F, 0x02, 0x04};
    uint8_t *fb = ssd1306_get_framebuf();
    for (int c = 0; c < 5; c++) {
        uint16_t expanded = 0;
        for (int b = 0; b < 7; b++)
            if (src[c] & (1 << b)) expanded |= (3u << (b * 2));
        uint8_t lo = (uint8_t)(expanded & 0xFF);
        uint8_t hi = (uint8_t)((expanded >> 8) & 0xFF);
        for (int dx = 0; dx < 2; dx++) {
            int px = x + c * 2 + dx;
            if (px >= 0 && px < SSD1306_WIDTH) {
                fb[page * SSD1306_WIDTH + px] = lo;
                fb[(page + 1) * SSD1306_WIDTH + px] = hi;
            }
        }
    }
}

// Draw a 2x-height arrow-down glyph into framebuf at (x, page..page+1)
static void draw_arrow_down_2x(int x, int page) {
    static const uint8_t src[] = {0x10, 0x20, 0x7F, 0x20, 0x10};
    uint8_t *fb = ssd1306_get_framebuf();
    for (int c = 0; c < 5; c++) {
        uint16_t expanded = 0;
        for (int b = 0; b < 7; b++)
            if (src[c] & (1 << b)) expanded |= (3u << (b * 2));
        uint8_t lo = (uint8_t)(expanded & 0xFF);
        uint8_t hi = (uint8_t)((expanded >> 8) & 0xFF);
        for (int dx = 0; dx < 2; dx++) {
            int px = x + c * 2 + dx;
            if (px >= 0 && px < SSD1306_WIDTH) {
                fb[page * SSD1306_WIDTH + px] = lo;
                fb[(page + 1) * SSD1306_WIDTH + px] = hi;
            }
        }
    }
}

// Draw radio-on icon at pixel (x, y): dot + 2 arcs, 7px tall, ~9px wide
static void draw_radio_on_xy(int x, int y) {
    // Dot 2x2 at center height (y+2, y+3)
    ssd1306_set_pixel(x,   y+2, true); ssd1306_set_pixel(x,   y+3, true);
    ssd1306_set_pixel(x+1, y+2, true); ssd1306_set_pixel(x+1, y+3, true);
    // Arc 1
    ssd1306_set_pixel(x+3, y+1, true);
    ssd1306_set_pixel(x+3, y+2, true); ssd1306_set_pixel(x+3, y+3, true);
    ssd1306_set_pixel(x+3, y+4, true);
    // Arc 2
    ssd1306_set_pixel(x+5, y+0, true);
    ssd1306_set_pixel(x+5, y+1, true);
    ssd1306_set_pixel(x+5, y+2, true); ssd1306_set_pixel(x+5, y+3, true);
    ssd1306_set_pixel(x+5, y+4, true);
    ssd1306_set_pixel(x+5, y+5, true);
}

// Draw radio-off icon at pixel (x, y): just a dot, 7px tall, 2px wide
static void draw_radio_off_xy(int x, int y) {
    ssd1306_set_pixel(x,   y+2, true); ssd1306_set_pixel(x,   y+3, true);
    ssd1306_set_pixel(x+1, y+2, true); ssd1306_set_pixel(x+1, y+3, true);
}

// Prepare framebuffer content (fast, no I2C)
static void oled_prepare_frame(void) {
    double freq = g_target_freq_hz;
    double downlink = freq + QO100_DOWNLINK_OFFSET_HZ;

    ssd1306_clear();

    // --- Row 0 (pages 0-1): ↑ arrow + right-justified uplink freq ---
    draw_arrow_up_2x(0, 0);
    {
        char buf[20];
        uint32_t khz_total = (uint32_t)(freq / 1000.0);
        uint32_t frac = (uint32_t)((freq - (double)khz_total * 1000.0) / 100.0 + 0.5);
        if (frac >= 10) { frac -= 10; khz_total++; }
        snprintf(buf, sizeof(buf), "%lu.%lu", (unsigned long)khz_total, (unsigned long)frac);
        // Right-justify 2x text: each char = 12px
        int len = 0; { const char *p = buf; while (*p++) len++; }
        int x_start = 128 - len * 12;
        ssd1306_draw_string_2x(x_start, 0, buf);
    }

    // --- Row 1 (pages 2-3): ↓ arrow + right-justified downlink freq ---
    draw_arrow_down_2x(0, 2);
    {
        char buf[20];
        uint32_t khz_total = (uint32_t)(downlink / 1000.0);
        uint32_t frac = (uint32_t)((downlink - (double)khz_total * 1000.0) / 100.0 + 0.5);
        if (frac >= 10) { frac -= 10; khz_total++; }
        snprintf(buf, sizeof(buf), "%lu.%lu", (unsigned long)khz_total, (unsigned long)frac);
        int len = 0; { const char *p = buf; while (*p++) len++; }
        int x_start = 128 - len * 12;
        ssd1306_draw_string_2x(x_start, 2, buf);
    }

    // --- Separator line at y=32 ---
    ssd1306_hline(0, 127, 32);

    // --- Page 2: GPS / time (display only) ---
    if (g_ui_page == 2) {
        gpsdo_info_t gi;
        gpsdo_get_info(&gi);
        char line[24];
        snprintf(line, sizeof(line), "%-10s    %6s", gi.date, gi.locator);
        ssd1306_draw_string_bold_y(0, 34, line);
        snprintf(line, sizeof(line), "%s UTC", gi.utc_hhmm);
        ssd1306_draw_string_2x(0, 5, line);                       // y 40..55
        snprintf(line, sizeof(line), "sat %u/%u %s %dm",
                 (unsigned)gi.sats_used, (unsigned)gi.sats_vis, gi.fix ? "fix" : "nofix", (int)gi.alt_m);
        ssd1306_draw_string_bold_y(0, 57, line);
        return;
    }

    // --- Pages 0/1: 2 columns × 3 rows, all navigable ---
    // Vertical divider at x=63
    ssd1306_vline(63, 33, 63);

    // Column positions and widths
    const int COL0_X = 2;    // left column text start
    const int COL0_W = 60;   // left column field width for inverted rendering
    const int COL0_R = 61;   // left column right edge for frame
    const int COL1_X = 66;   // right column text start
    const int COL1_W = 60;   // right column field width
    const int COL1_R = 126;  // right column right edge for frame
    // Row Y positions (stride 10: 7px text + 3px gap)
    const int ROW0_Y = 34;
    const int ROW1_Y = 44;
    const int ROW2_Y = 54;

    // Helper macro: draw a navigable item in either column
    #define DRAW_ITEM(col_x, col_w, col_l, col_r, y_pos, text, param_id) do { \
        if (g_ui_state == UI_STATE_EDITING && g_ui_editing == (param_id)) { \
            /* EDITING: inverted (white bg, black text) */ \
            ssd1306_draw_string_bold_y_inv((col_x), (y_pos), (text), (col_w)); \
        } else if (g_ui_state == UI_STATE_BROWSE && g_ui_cursor == (param_id)) { \
            /* BROWSE: frame around this item */ \
            ssd1306_draw_string_bold_y((col_x), (y_pos), (text)); \
            ssd1306_hline((col_l), (col_r), (y_pos)-1); \
            ssd1306_hline((col_l), (col_r), (y_pos)+8); \
            ssd1306_vline((col_l), (y_pos)-1, (y_pos)+8); \
            ssd1306_vline((col_r), (y_pos)-1, (y_pos)+8); \
        } else { \
            /* IDLE or non-selected: normal text */ \
            ssd1306_draw_string_bold_y((col_x), (y_pos), (text)); \
        } \
    } while(0)

    // Shorthand for left/right columns
    #define DRAW_L(y, text, pid) DRAW_ITEM(COL0_X, COL0_W, 0,  COL0_R, y, text, pid)
    #define DRAW_R(y, text, pid) DRAW_ITEM(COL1_X, COL1_W, 64, COL1_R, y, text, pid)

    char wpm_buf[12];
    snprintf(wpm_buf, sizeof(wpm_buf), "%u WPM", (unsigned)g_cw_wpm);

    if (g_ui_page == 0) {
        // --- Page 0: operating ---
        DRAW_L(ROW0_Y, g_tx_mode ? "CW" : (g_audio_src == AUDIO_SRC_MIC ? "USB MIC" : "USB PC"), UI_PARAM_MODE);
        DRAW_L(ROW1_Y, g_tune_active ? "TUNE *" : "TUNE", UI_PARAM_TUNE);
        DRAW_L(ROW2_Y, wpm_buf, UI_PARAM_WPM);

        const char *tx_label;
        if (!tx_allowed()) {
            tx_label = "WAIT GPS";
        } else if (g_tx_mode == 1 && (g_keyer_key | g_soft_ptt_key)) {
            tx_label = "KEY";
        } else {
            tx_label = g_tx_enabled ? "TX ON" : "TX OFF";
        }
        DRAW_R(ROW0_Y, tx_label, UI_PARAM_TX);

        char vol_buf[12];
        snprintf(vol_buf, sizeof(vol_buf), "VOL %u%%", (unsigned)g_st_vol);
        DRAW_R(ROW1_Y, vol_buf, UI_PARAM_VOL);

        char pwr_buf[12];
        snprintf(pwr_buf, sizeof(pwr_buf), "%+ddBm", g_tx_power_max_dbm);
        DRAW_R(ROW2_Y, pwr_buf, UI_PARAM_PWR);
    } else {
        // --- Page 1: CW & audio ---
        DRAW_L(ROW0_Y, g_cw_mode == KEYER_STRAIGHT ? "STRAIGHT" : g_cw_mode == KEYER_IAMBIC_A ? "IAMBIC A" : "IAMBIC B",
               UI_PARAM_KEYER);

        char ratio_buf[12];
        snprintf(ratio_buf, sizeof(ratio_buf), "RATIO %.1f", (double)g_cw_ratio);
        DRAW_L(ROW1_Y, ratio_buf, UI_PARAM_RATIO);
        DRAW_L(ROW2_Y, wpm_buf, UI_PARAM_WPM);

        char tone_buf[12];
        snprintf(tone_buf, sizeof(tone_buf), "TONE %u", (unsigned)g_st_hz);
        DRAW_R(ROW0_Y, tone_buf, UI_PARAM_TONE);

        char gain_buf[12];
        snprintf(gain_buf, sizeof(gain_buf), "MIC G %.0f", (double)g_mic_gain);
        DRAW_R(ROW1_Y, gain_buf, UI_PARAM_MICGAIN);

        char gate_buf[12];
        snprintf(gate_buf, sizeof(gate_buf), "GATE %.3f", (double)g_mic_gate);
        DRAW_R(ROW2_Y, gate_buf, UI_PARAM_MICGATE);
    }

    #undef DRAW_ITEM
    #undef DRAW_L
    #undef DRAW_R
}

// ==========================================================
// Encoder + Button polling (Core0, called from idle loop)
// ==========================================================

// Encoder state machine (Gray code quadrature)
static uint8_t enc_last_ab = 0;
static int8_t  enc_accum = 0;

// ==========================================================
// On-device iambic keyer
// ==========================================================
// Port of the GUI's Keyer class (uSDX/WB4VVF behaviour):
//  - pending latches are set only on rising edges of the debounced paddles
//  - both latches are cleared at element START (uSDX KEYED_PREP)
//  - same-paddle level is re-latched at element END (hold-to-repeat)
//  - Iambic B additionally latches the opposite paddle during an element
//  - Iambic A drops latches for paddles already released at element end
// ICH/IWD only serve the decoder (character / word gap detection); keying
// itself is finished once IEL has elapsed.
typedef enum { KS_IDLE = 0, KS_DIT, KS_DAH, KS_IEL, KS_ICH, KS_IWD } keyer_state_t;

static struct {
    keyer_state_t state;
    uint64_t      t0_us;
    uint8_t       pend_dit, pend_dah, was_dit;
    uint8_t       dit_prev, dah_prev;
    char          sym[8];        // elements of the character being keyed
    uint8_t       sym_len;
} s_kyr;

static const struct { const char *code; char ch; } k_morse[] = {
    {".-",'A'},   {"-...",'B'}, {"-.-.",'C'}, {"-..",'D'},  {".",'E'},    {"..-.",'F'},
    {"--.",'G'},  {"....",'H'}, {"..",'I'},   {".---",'J'}, {"-.-",'K'},  {".-..",'L'},
    {"--",'M'},   {"-.",'N'},   {"---",'O'},  {".--.",'P'}, {"--.-",'Q'}, {".-.",'R'},
    {"...",'S'},  {"-",'T'},    {"..-",'U'},  {"...-",'V'}, {".--",'W'},  {"-..-",'X'},
    {"-.--",'Y'}, {"--..",'Z'},
    {".----",'1'},{"..---",'2'},{"...--",'3'},{"....-",'4'},{".....",'5'},
    {"-....",'6'},{"--...",'7'},{"---..",'8'},{"----.",'9'},{"-----",'0'},
    {".-.-.-",'.'},{"--..--",','},{"..--..",'?'},{".----.",'\''},{"-..-.",'/'},
    {"---...",':'},{"-.--.",'('},{"-.--.-",')'},{".-...",'&'},{"-...-",'='},
    {".-.-.",'+'},{"-....-",'-'},{".-..-.",'"'},{".--.-.",'@'},
};

// Decoder events for the GUI: "!K e=." / "!K e=-" per element,
// "!K c=<char>" per decoded character, "!K w" per word gap.
static inline void keyer_emit(const char *s) { cdc_write_str(s); }

static void keyer_sym_push(char c) {
    if (s_kyr.sym_len < sizeof(s_kyr.sym) - 1) s_kyr.sym[s_kyr.sym_len++] = c;
    keyer_emit(c == '.' ? "!K e=.\r\n" : "!K e=-\r\n");
}

static void keyer_decode_char(void) {
    if (!s_kyr.sym_len) return;
    s_kyr.sym[s_kyr.sym_len] = '\0';
    char ch = '?';
    for (size_t i = 0; i < sizeof(k_morse) / sizeof(k_morse[0]); i++) {
        if (strcmp(k_morse[i].code, s_kyr.sym) == 0) { ch = k_morse[i].ch; break; }
    }
    s_kyr.sym_len = 0;
    char line[12];
    snprintf(line, sizeof(line), "!K c=%c\r\n", ch);
    keyer_emit(line);
}

static const char *keyer_mode_name(uint8_t m) {
    return (m == KEYER_STRAIGHT) ? "straight" : (m == KEYER_IAMBIC_A) ? "iambic-a" : "iambic-b";
}

static void keyer_reset(void) {
    s_kyr.state = KS_IDLE;
    s_kyr.pend_dit = s_kyr.pend_dah = 0;
    s_kyr.sym_len = 0;
    g_keyer_key = 0;
}

static inline void keyer_send_dit(uint64_t now) {
    s_kyr.pend_dit = s_kyr.pend_dah = 0;
    s_kyr.state = KS_DIT; s_kyr.t0_us = now;
    g_keyer_key = 1;
}

static inline void keyer_send_dah(uint64_t now) {
    s_kyr.pend_dit = s_kyr.pend_dah = 0;
    s_kyr.state = KS_DAH; s_kyr.t0_us = now;
    g_keyer_key = 1;
}

// Called from button_poll() with the debounced paddle levels.
static void keyer_poll(void) {
    uint8_t dit = g_key_dit, dah = g_key_dah;
    if (g_tx_mode != 1) {
        // Keyer is only live in CW mode; track levels so no stale edge fires on entry.
        s_kyr.dit_prev = dit; s_kyr.dah_prev = dah;
        if (g_keyer_key || s_kyr.state != KS_IDLE || s_kyr.pend_dit || s_kyr.pend_dah) keyer_reset();
        return;
    }
    if (dit && !s_kyr.dit_prev) s_kyr.pend_dit = 1;
    if (dah && !s_kyr.dah_prev) s_kyr.pend_dah = 1;
    s_kyr.dit_prev = dit;
    s_kyr.dah_prev = dah;

    const uint64_t now    = time_us_64();
    const uint64_t el     = now - s_kyr.t0_us;
    const uint32_t dit_us = 1200000u / (g_cw_wpm ? g_cw_wpm : 18u);
    const uint32_t dah_us = (uint32_t)((float)dit_us * g_cw_ratio);
    const bool     mode_a = (g_cw_mode == KEYER_IAMBIC_A);

    if (g_cw_mode == KEYER_STRAIGHT) {
        // Either contact is the key: works for a straight key on tip and for paddles alike.
        // Elements are classified by hold time for the decoder only.
        const uint8_t k = dit | dah;
        s_kyr.pend_dit = s_kyr.pend_dah = 0;
        if (k && !g_keyer_key) {
            g_keyer_key = 1; s_kyr.t0_us = now; s_kyr.state = KS_DIT;
        } else if (!k && g_keyer_key) {
            g_keyer_key = 0;
            keyer_sym_push(el < (uint64_t)dit_us * 2u ? '.' : '-');
            s_kyr.t0_us = now; s_kyr.state = KS_ICH;
        } else if (!k) {
            if (s_kyr.state == KS_ICH && el >= (uint64_t)dit_us * 3u) {
                keyer_decode_char();
                s_kyr.state = KS_IWD; s_kyr.t0_us = now;
            } else if (s_kyr.state == KS_IWD && el >= (uint64_t)dit_us * 4u) {
                keyer_emit("!K w\r\n");
                s_kyr.state = KS_IDLE;
            }
        }
        return;
    }

    switch (s_kyr.state) {
    case KS_IDLE:
        if (s_kyr.pend_dit && !s_kyr.pend_dah)      keyer_send_dit(now);
        else if (s_kyr.pend_dah && !s_kyr.pend_dit) keyer_send_dah(now);
        else if (s_kyr.pend_dit && s_kyr.pend_dah) {
            if (s_kyr.was_dit) keyer_send_dah(now); else keyer_send_dit(now);
        }
        break;

    case KS_DIT:
        if (el >= dit_us) {
            s_kyr.was_dit = 1;
            g_keyer_key = 0;
            keyer_sym_push('.');
            if (dit) s_kyr.pend_dit = 1;
            if (mode_a) { if (!dit) s_kyr.pend_dit = 0; if (!dah) s_kyr.pend_dah = 0; }
            s_kyr.state = KS_IEL; s_kyr.t0_us = now;
        } else if (!mode_a && dah) {
            s_kyr.pend_dah = 1;   // Iambic B squeeze latch
        }
        break;

    case KS_DAH:
        if (el >= dah_us) {
            s_kyr.was_dit = 0;
            g_keyer_key = 0;
            keyer_sym_push('-');
            if (dah) s_kyr.pend_dah = 1;
            if (mode_a) { if (!dit) s_kyr.pend_dit = 0; if (!dah) s_kyr.pend_dah = 0; }
            s_kyr.state = KS_IEL; s_kyr.t0_us = now;
        } else if (!mode_a && dit) {
            s_kyr.pend_dit = 1;   // Iambic B squeeze latch
        }
        break;

    case KS_IEL:
        if (el >= dit_us) {
            if (s_kyr.pend_dit && s_kyr.pend_dah) {
                if (s_kyr.was_dit) keyer_send_dah(now); else keyer_send_dit(now);
            } else if (s_kyr.pend_dah) keyer_send_dah(now);
            else if (s_kyr.pend_dit)   keyer_send_dit(now);
            else { s_kyr.state = KS_ICH; s_kyr.t0_us = now; }
        }
        break;

    case KS_ICH:
        // Character gap running (decoder only). A new element continues the same character.
        if (s_kyr.pend_dit || s_kyr.pend_dah) {
            if (s_kyr.pend_dah) keyer_send_dah(now); else keyer_send_dit(now);
        } else if (el >= (uint64_t)dit_us * 3u) {
            keyer_decode_char();
            s_kyr.state = KS_IWD; s_kyr.t0_us = now;
        }
        break;

    case KS_IWD:
        if (s_kyr.pend_dit || s_kyr.pend_dah) {
            s_kyr.state = KS_IDLE;   // IDLE sends it on the next poll
        } else if (el >= (uint64_t)dit_us * 4u) {
            keyer_emit("!K w\r\n");
            s_kyr.state = KS_IDLE;
        }
        break;
    }
}

static void keyer_diag_print(void) {
    static const char *cr_names[] = { "IDLE", "ARMING", "ARMED", "CARRIER_ON" };
    static const char *ks_names[] = { "IDLE", "DIT", "DAH", "IEL", "ICH", "IWD", "?", "?" };
    static const char *ui_names[] = { "IDLE", "BROWSE", "EDITING", "?" };
    cdc_printf("Encoder: GP2(A)=%u GP3(B)=%u GP10(SW)=%u  ui=%s cursor=%u  (1 = pin at GND)\r\n",
               (unsigned)!gpio_get(PIN_ENC_A), (unsigned)!gpio_get(PIN_ENC_B), (unsigned)!gpio_get(PIN_ENC_OK),
               ui_names[g_ui_state & 3], (unsigned)g_ui_cursor);
    cdc_printf("Paddles: GP9(dit) raw=%u db=%u  GP11(dah) raw=%u db=%u  (1 = pressed, pin at GND)\r\n",
               (unsigned)!gpio_get(PIN_KEY_DIT), (unsigned)g_key_dit,
               (unsigned)!gpio_get(PIN_KEY_DAH), (unsigned)g_key_dah);
    cdc_printf("Keyer: mode=%s wpm=%u ratio=%.1f state=%s pend=%u/%u out=%u softkey=%u\r\n",
               keyer_mode_name(g_cw_mode), (unsigned)g_cw_wpm, (double)g_cw_ratio,
               ks_names[s_kyr.state & 7], (unsigned)s_kyr.pend_dit, (unsigned)s_kyr.pend_dah,
               (unsigned)g_keyer_key, (unsigned)g_soft_ptt_key);
    cdc_printf("Sidetone: GP12 PWM  tone=%u Hz vol=%u%% test=%u  (audible while keyer out=1 or tune=1)\r\n",
               (unsigned)g_st_hz, (unsigned)g_st_vol, (unsigned)g_st_test);
    mic_diag_print();
    cdc_printf("Carrier: txmode=%s tune=%u state=%s cw_test_mode=%u pwr_now=%d dBm  tx_allowed=%u (gps=%u gate=%u)\r\n",
               g_tx_mode ? "CW" : "USB", (unsigned)g_tune_active,
               cr_names[g_cr_state & 3], (unsigned)g_cw_test_mode, (int)g_cr_pwr,
               (unsigned)tx_allowed(), (unsigned)gpsdo_is_ready(), (unsigned)g_gps_gate);
}

// ==========================================================
// Sidetone: PWM audio on GP12 → RC low-pass → headphones
// ==========================================================
// 10-bit PWM at sysclk/1024 (244 kHz @ 250 MHz). Samples are delivered
// by DMA at 48 kHz, paced by the wrap DREQ of a spare PWM slice used as a
// timebase — no per-sample interrupt, no jitter from USB IRQs. Two
// ping-pong buffers of 2.7 ms are refilled from the DMA completion IRQ.
// Pure sine, 5 ms raised-cosine envelope on each key edge (same rise
// time as the RF power ramp), first-order error feedback so the 10-bit
// PWM does not hiss at low volume.
#define PIN_AUDIO_OUT    12
#define ST_PWM_WRAP      1023u
#define ST_FS_HZ         48000u
#define ST_TB_SLICE      7        // timebase slice; GP14/15 are plain GPIO, so its counter is free
#define ST_BUF           128      // samples per half-buffer (2.7 ms → key latency ≤ 5.3 ms)
#define ST_RAMP_SAMPLES  240      // 5 ms

static int16_t           s_st_sin[256];
static uint16_t          s_st_rc[257];            // raised cosine 0..32767
static uint32_t          s_st_buf[2][ST_BUF];     // PWM CC values (channel A in the low half)
static int               s_st_dma[2] = { -1, -1 };
static uint              s_st_slice;
static volatile uint32_t s_st_inc  = 0;           // phase increment per sample
static volatile int32_t  s_st_gain = 0;           // Q15 volume
static uint32_t          s_st_phase   = 0;
static uint32_t          s_st_env_pos = 0;        // 0..ST_RAMP_SAMPLES on the envelope
static int32_t           s_st_err     = 0;        // quantisation error carried to the next sample
static uint32_t          s_st_fs      = ST_FS_HZ;

static void sidetone_update_params(void) {
    s_st_inc  = (uint32_t)((double)g_st_hz * 4294967296.0 / (double)s_st_fs);
    s_st_gain = (int32_t)g_st_vol * 32767 / 100;
}

static void __not_in_flash_func(sidetone_fill)(uint32_t *dst) {
    const uint32_t inc  = s_st_inc;
    const int32_t  gain = s_st_gain;
    // Also sound while the volume is being edited on the OLED, so the level can be heard.
    const bool on = g_keyer_key || g_tune_active || g_st_test ||
                    (g_ui_state == UI_STATE_EDITING &&
                     (g_ui_editing == UI_PARAM_VOL || g_ui_editing == UI_PARAM_TONE));

    for (uint32_t i = 0; i < ST_BUF; i++) {
        if (on)  { if (s_st_env_pos < ST_RAMP_SAMPLES) s_st_env_pos++; }
        else     { if (s_st_env_pos > 0)               s_st_env_pos--; }
        const int32_t env = s_st_rc[(s_st_env_pos * 256u) / ST_RAMP_SAMPLES];

        s_st_phase += inc;
        const int32_t s   = s_st_sin[s_st_phase >> 24];
        const int32_t amp = (env * gain) >> 15;              // Q15
        const int32_t v   = (s * amp) >> 15;                 // ±32767 full scale

        // 16-bit sample → 10-bit PWM level with the rounding error fed back
        // into the next sample (pushes quantisation noise above the RC corner).
        const int32_t want = (v + 32768) * (int32_t)(ST_PWM_WRAP + 1u) + s_st_err;
        int32_t lvl = want >> 16;
        s_st_err = want - (lvl << 16);
        if (lvl < 0) lvl = 0;
        if (lvl > (int32_t)ST_PWM_WRAP) lvl = (int32_t)ST_PWM_WRAP;
        dst[i] = (uint32_t)lvl;
    }
}

static void __not_in_flash_func(sidetone_dma_irq)(void) {
    for (int b = 0; b < 2; b++) {
        if (s_st_dma[b] >= 0 && dma_channel_get_irq0_status((uint)s_st_dma[b])) {
            dma_channel_acknowledge_irq0((uint)s_st_dma[b]);
            sidetone_fill(s_st_buf[b]);
            // Re-arm for the next chain trigger; TRANS_COUNT reloads automatically.
            dma_channel_set_read_addr((uint)s_st_dma[b], s_st_buf[b], false);
        }
    }
}

static void sidetone_init(void) {
    for (int i = 0; i < 256; i++) {
        s_st_sin[i] = (int16_t)lrintf(sinf(2.0f * (float)M_PI * (float)i / 256.0f) * 32767.0f);
    }
    for (int i = 0; i <= 256; i++) {
        s_st_rc[i] = (uint16_t)lrintf((1.0f - cosf((float)M_PI * (float)i / 256.0f)) * 0.5f * 32767.0f);
    }

    // Audio PWM on GP12
    gpio_set_function(PIN_AUDIO_OUT, GPIO_FUNC_PWM);
    s_st_slice = pwm_gpio_to_slice_num(PIN_AUDIO_OUT);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, 1.0f);
    pwm_config_set_wrap(&c, ST_PWM_WRAP);
    pwm_init(s_st_slice, &c, true);
    pwm_set_gpio_level(PIN_AUDIO_OUT, 512);   // silence = mid-scale (DC blocked by the output cap)

    // Timebase: spare slice wrapping at the sample rate; its DREQ paces the DMA.
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    const uint32_t tb_wrap = sys_hz / ST_FS_HZ - 1u;
    s_st_fs = sys_hz / (tb_wrap + 1u);
    pwm_config tb = pwm_get_default_config();
    pwm_config_set_clkdiv(&tb, 1.0f);
    pwm_config_set_wrap(&tb, (uint16_t)tb_wrap);
    pwm_init(ST_TB_SLICE, &tb, true);

    sidetone_update_params();
    sidetone_fill(s_st_buf[0]);
    sidetone_fill(s_st_buf[1]);

    // Two channels chained to each other; each raises IRQ0 when its half is done.
    s_st_dma[0] = dma_claim_unused_channel(true);
    s_st_dma[1] = dma_claim_unused_channel(true);
    for (int b = 0; b < 2; b++) {
        dma_channel_config dc = dma_channel_get_default_config((uint)s_st_dma[b]);
        channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
        channel_config_set_read_increment(&dc, true);
        channel_config_set_write_increment(&dc, false);
        channel_config_set_dreq(&dc, pwm_get_dreq(ST_TB_SLICE));
        channel_config_set_chain_to(&dc, (uint)s_st_dma[b ^ 1]);
        dma_channel_configure((uint)s_st_dma[b], &dc,
                              &pwm_hw->slice[s_st_slice].cc,   // write: CC register (A = low 16 bits)
                              s_st_buf[b], ST_BUF, false);
        dma_channel_set_irq0_enabled((uint)s_st_dma[b], true);
    }
    irq_add_shared_handler(DMA_IRQ_0, sidetone_dma_irq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_start((uint)s_st_dma[0]);
}

// ==========================================================
// Microphone: MAX4466 on GP26 (ADC0), free-running 8 kHz → DMA ring
// ==========================================================
// The ADC samples continuously at exactly WAV_SAMPLE_RATE (48 MHz ADC
// clock / 6000) and DMA drops the 12-bit values into a 4 KB ring with
// address wrap. Core0 reads behind the DMA write pointer, so the mic
// paces the producer at the same rate Core1 consumes — both clocks come
// from the same crystal, no resampler needed.
#define PIN_MIC          26
#define MIC_RB_SAMPLES   2048u
#define MIC_RB_MASK      (MIC_RB_SAMPLES - 1u)
#define MIC_DMA_COUNT    0x0FFFFFFFu          // ~9 h at 8 kHz, then re-armed

static uint16_t __attribute__((aligned(4096))) s_mic_rb[MIC_RB_SAMPLES];
static int      s_mic_dma  = -1;
static uint32_t s_mic_rd   = 0;
static float    s_mic_dc   = 0.0f;   // DC estimate (MAX4466 idles at VCC/2)
static float    s_mic_env  = 0.0f;   // gate envelope
static float    s_mic_ggain = 0.0f;  // smoothed gate gain 0..1
static uint32_t s_mic_hold = 0;

static inline uint32_t mic_wr_index(void) {
    uint32_t addr = dma_channel_hw_addr((uint)s_mic_dma)->write_addr;
    return ((addr - (uint32_t)(uintptr_t)s_mic_rb) / 2u) & MIC_RB_MASK;
}
static inline uint32_t mic_available(void) { return (mic_wr_index() - s_mic_rd) & MIC_RB_MASK; }
static inline void     mic_flush(void)     { s_mic_rd = mic_wr_index(); }

static void mic_dma_rearm_if_done(void) {
    if (s_mic_dma >= 0 && !dma_channel_is_busy((uint)s_mic_dma)) {
        dma_channel_set_trans_count((uint)s_mic_dma, MIC_DMA_COUNT, true);
    }
}

// One sample, normalised to ±1: DC removal → gain → noise gate (hysteresis,
// 300 ms hold, smoothed so it does not click). Caller checks availability.
static float mic_get_sample(void) {
    uint16_t raw = s_mic_rb[s_mic_rd];
    s_mic_rd = (s_mic_rd + 1u) & MIC_RB_MASK;

    float x = ((float)(raw & 0x0FFFu) - 2048.0f) / 2048.0f;
    s_mic_dc += 0.005f * (x - s_mic_dc);           // one-pole HPF, fc ≈ 6 Hz
    x = (x - s_mic_dc) * g_mic_gain;

    float ax = fabsf(x);
    s_mic_env += (ax > s_mic_env ? 0.2f : 0.002f) * (ax - s_mic_env);
    const float thr = g_mic_gate;
    bool open;
    if (s_mic_env > thr)             { open = true;  s_mic_hold = 2400u; }
    else if (s_mic_hold)             { open = true;  s_mic_hold--; }
    else                             { open = (s_mic_env > thr * 0.5f) && (s_mic_ggain > 0.5f); }
    s_mic_ggain += open ? 0.05f : -0.002f;
    if (s_mic_ggain > 1.0f) s_mic_ggain = 1.0f;
    if (s_mic_ggain < 0.0f) s_mic_ggain = 0.0f;
    x *= s_mic_ggain;

    if (x >  1.0f) x =  1.0f;
    if (x < -1.0f) x = -1.0f;
    return x;
}

static void mic_init(void) {
    adc_init();
    adc_gpio_init(PIN_MIC);
    adc_select_input(0);
    adc_set_clkdiv(48000000.0f / (float)WAV_SAMPLE_RATE - 1.0f);   // one conversion per 1/8000 s
    adc_fifo_setup(true, true, 1, false, false);

    s_mic_dma = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config((uint)s_mic_dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 12);          // wrap the write address every 4 KB
    channel_config_set_dreq(&c, DREQ_ADC);
    dma_channel_configure((uint)s_mic_dma, &c, s_mic_rb, &adc_hw->fifo, MIC_DMA_COUNT, true);

    adc_run(true);
    mic_flush();
}

static const char *audio_src_name(uint8_t s) { return s == AUDIO_SRC_MIC ? "mic" : "pc"; }

static void mic_diag_print(void) {
    uint32_t wi  = mic_wr_index();
    uint32_t raw = s_mic_rb[wi ? wi - 1u : MIC_RB_MASK] & 0x0FFFu;   // newest ADC value (2048 = idle)
    cdc_printf("Audio: src=%s  mic GP26: ring=%lu/%u raw=%lu gain=%.1f gate=%.3f env=%.3f gate=%s\r\n",
               audio_src_name(g_audio_src), (unsigned long)mic_available(), (unsigned)MIC_RB_SAMPLES,
               (unsigned long)raw, (double)g_mic_gain, (double)g_mic_gate, (double)s_mic_env,
               s_mic_ggain > 0.5f ? "open" : "closed");
}

// Button debounce state
static uint8_t  ok_was_pressed = 0;
static uint32_t ok_debounce_ms = 0;
static uint32_t ok_press_start_ms = 0;
static uint8_t  ok_long_fired = 0;
static uint32_t dit_debounce_ms = 0, dah_debounce_ms = 0;
static uint8_t  dit_last_state = 0,  dah_last_state = 0;

#define DEBOUNCE_MS         5

// Frequency step for encoder (Hz)
#define ENC_FREQ_STEP_HZ   100.0

static inline void ui_touch(void) {
    g_ui_last_activity_ms = to_ms_since_boot(get_absolute_time());
}

static void encoder_poll(void) {
    // Read encoder pins (active LOW with pull-up)
    uint8_t a = gpio_get(PIN_ENC_A) ? 0 : 1;
    uint8_t b = gpio_get(PIN_ENC_B) ? 0 : 1;
    uint8_t ab = (a << 1) | b;

    if (ab == enc_last_ab) return;

    // Quadrature decode via state transition table
    static const int8_t enc_table[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0
    };
    int8_t dir = enc_table[(enc_last_ab << 2) | ab];
    enc_last_ab = ab;

    if (dir == 0) return;

    // Accumulate — only act on full detent (4 edges per click)
    enc_accum += dir;
    int step = 0;
    if (enc_accum >= 4) {
        step = +1;
        enc_accum = 0;
    } else if (enc_accum <= -4) {
        step = -1;
        enc_accum = 0;
    } else {
        return;  // Not a full step yet
    }

    ui_touch();

    switch (g_ui_state) {
        case UI_STATE_IDLE:
            // Default: encoder adjusts frequency
            {
                double f = g_target_freq_hz + step * ENC_FREQ_STEP_HZ;
                if (f < 2400000000.0) f = 2400000000.0;
                if (f > 2500000000.0) f = 2500000000.0;
                g_target_freq_hz = f;
                persist_mark_dirty();
                if (g_tune_active) tune_apply_settings();
            }
            break;

        case UI_STATE_BROWSE:
            // Move cursor between the tiles of the current page
            {
                int c = (int)g_ui_cursor_idx + step;
                if (c < 0) c = UI_TILES - 1;
                if (c >= UI_TILES) c = 0;
                g_ui_cursor_idx = (uint8_t)c;
                g_ui_cursor = k_page_tiles[g_ui_page < 2 ? g_ui_page : 0][c];
            }
            break;

        case UI_STATE_EDITING:
            // Adjust value of selected param
            switch (g_ui_editing) {
                case UI_PARAM_MODE:
                    {
                        // Cycle USB PC → USB MIC → CW (both directions)
                        int idx = g_tx_mode ? 2 : (g_audio_src == AUDIO_SRC_MIC ? 1 : 0);
                        idx = (idx + step + 3) % 3;
                        g_tx_mode   = (idx == 2) ? 1 : 0;
                        if (idx < 2) g_audio_src = (idx == 1) ? AUDIO_SRC_MIC : AUDIO_SRC_PC;
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_TUNE:
                    if (step > 0 && !g_tune_active) {
                        g_tune_active = 1;
                        // carrier_poll() will arm Core1 idle + start carrier
                    } else if (step < 0 && g_tune_active) {
                        g_tune_active = 0;
                        // carrier_poll() will stop carrier (or keep idle if CW mode)
                    }
                    break;
                case UI_PARAM_WPM:
                    {
                        int w = (int)g_cw_wpm + step;
                        if (w < 5)  w = 5;
                        if (w > 60) w = 60;
                        g_cw_wpm = (uint8_t)w;
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_TX:
                    g_tx_enabled = g_tx_enabled ? 0 : 1;
                    break;
                case UI_PARAM_VOL:
                    {
                        int v = (int)g_st_vol + step * 5;
                        if (v < 0)   v = 0;
                        if (v > 100) v = 100;
                        g_st_vol = (uint8_t)v;
                        sidetone_update_params();
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_PWR:
                    {
                        int8_t p = g_tx_power_max_dbm + (int8_t)step;
                        if (p < PWR_MIN_DBM) p = PWR_MIN_DBM;
                        if (p > PWR_MAX_DBM) p = PWR_MAX_DBM;
                        g_tx_power_max_dbm = p;
                        persist_mark_dirty();
                        if (g_tune_active) tune_apply_settings();
                    }
                    break;
                case UI_PARAM_KEYER:
                    g_cw_mode = (uint8_t)(((int)g_cw_mode + step + 3) % 3);
                    keyer_reset();
                    persist_mark_dirty();
                    break;
                case UI_PARAM_RATIO:
                    {
                        float r = g_cw_ratio + 0.1f * (float)step;
                        if (r < 2.0f) r = 2.0f;
                        if (r > 5.0f) r = 5.0f;
                        g_cw_ratio = r;
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_TONE:
                    {
                        int t = (int)g_st_hz + 50 * step;
                        if (t < 300)  t = 300;
                        if (t > 1200) t = 1200;
                        g_st_hz = (uint16_t)t;
                        sidetone_update_params();
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_MICGAIN:
                    {
                        float g = g_mic_gain + (float)step;
                        if (g < 1.0f)  g = 1.0f;
                        if (g > 50.0f) g = 50.0f;
                        g_mic_gain = g;
                        persist_mark_dirty();
                    }
                    break;
                case UI_PARAM_MICGATE:
                    {
                        float t = g_mic_gate + 0.005f * (float)step;
                        if (t < 0.0f) t = 0.0f;
                        if (t > 0.5f) t = 0.5f;
                        g_mic_gate = t;
                        persist_mark_dirty();
                    }
                    break;
                default: break;
            }
            break;
    }
}

static void button_poll(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    // --- Timeout: return to IDLE after 5s ---
    if (g_ui_state != UI_STATE_IDLE) {
        if ((now_ms - g_ui_last_activity_ms) >= UI_TIMEOUT_MS) {
            g_ui_state = UI_STATE_IDLE;
        }
    }

    // --- OK button (encoder push): short click acts on release, long press fires while held ---
    uint8_t ok_raw = gpio_get(PIN_ENC_OK) ? 0 : 1;  // Active LOW

    if (ok_raw != ok_was_pressed && (now_ms - ok_debounce_ms) >= DEBOUNCE_MS) {
        ok_debounce_ms = now_ms;
        ok_was_pressed = ok_raw;

        if (ok_raw) {
            ok_press_start_ms = now_ms;
            ok_long_fired = 0;
        } else if (!ok_long_fired) {
            // Short click
            ui_touch();
            switch (g_ui_state) {
                case UI_STATE_IDLE:
                    if (g_ui_page == 2) {
                        g_ui_page = 0;            // GPS page has no tiles — click returns to page 0
                    } else {
                        g_ui_state = UI_STATE_BROWSE;
                        g_ui_cursor_idx = 0;
                        g_ui_cursor = k_page_tiles[g_ui_page][0];
                    }
                    break;
                case UI_STATE_BROWSE:
                    g_ui_editing = g_ui_cursor;
                    g_ui_state = UI_STATE_EDITING;
                    break;
                case UI_STATE_EDITING:
                    g_ui_state = UI_STATE_BROWSE;
                    break;
            }
        }
    }
    if (ok_was_pressed && !ok_long_fired && (now_ms - ok_press_start_ms) >= UI_LONG_PRESS_MS) {
        // Long press: next page from IDLE, otherwise back to IDLE
        ok_long_fired = 1;
        ui_touch();
        if (g_ui_state == UI_STATE_IDLE) g_ui_page = (uint8_t)((g_ui_page + 1u) % UI_PAGES);
        else                             g_ui_state = UI_STATE_IDLE;
    }

    // --- CW dit/dah paddles (GP9 / GP11, active LOW), debounced separately ---
    uint8_t dit_raw = !gpio_get(PIN_KEY_DIT);
    uint8_t dah_raw = !gpio_get(PIN_KEY_DAH);

    if (dit_raw != dit_last_state && (now_ms - dit_debounce_ms) >= DEBOUNCE_MS) {
        dit_debounce_ms = now_ms;
        dit_last_state  = dit_raw;
        g_key_dit = dit_raw;
    }
    if (dah_raw != dah_last_state && (now_ms - dah_debounce_ms) >= DEBOUNCE_MS) {
        dah_debounce_ms = now_ms;
        dah_last_state  = dah_raw;
        g_key_dah = dah_raw;
    }
    g_ptt_key = g_key_dit | g_key_dah;

    keyer_poll();
}

// ==========================================================
// Hilbert
// ==========================================================
static float hilb_h[HILBERT_TAPS];
static float hilb_buf[HILBERT_TAPS];
static uint32_t hilb_idx = 0;

static void hilbert_reset(void) {
    for (int i = 0; i < HILBERT_TAPS; i++) hilb_buf[i] = 0.0f;
    hilb_idx = 0;
}

static void hilbert_init(void) {
    const int M = (HILBERT_TAPS - 1) / 2;

    for (int n = 0; n < HILBERT_TAPS; n++) {
        int k = n - M;

        float h = 0.0f;
        if (k != 0 && (k & 1)) h = 2.0f / ((float)M_PI * (float)k);

        float w = 0.54f - 0.46f *
                  cosf(2.0f * (float)M_PI * (float)n /
                       (float)(HILBERT_TAPS - 1));

        hilb_h[n] = h * w;
        hilb_buf[n] = 0.0f;
    }
}

static inline float hilbert_process(float x, float *i_delayed) {
    const int M = (HILBERT_TAPS - 1) / 2;

    hilb_buf[hilb_idx] = x;

    float y = 0.0f;
    uint32_t idx = hilb_idx;
    for (int n = 0; n < HILBERT_TAPS; n++) {
        y += hilb_h[n] * hilb_buf[idx];
        if (idx == 0) idx = HILBERT_TAPS - 1;
        else idx--;
    }

    uint32_t id = (hilb_idx + HILBERT_TAPS - (uint32_t)M) % HILBERT_TAPS;
    *i_delayed = hilb_buf[id];

    hilb_idx++;
    if (hilb_idx >= HILBERT_TAPS) hilb_idx = 0;

    return y;
}

static inline float duty_from_A(float A) {
    if (A <= 0.0f) return 0.0f;
    float r = A / GATE_A_REF;
    if (r >= 1.0f) return 1.0f;
#if GATE_SHAPE == 2
    return r * r;
#else
    return r;
#endif
}

// ==========================================================
// CORE1: timed radio apply loop
// ==========================================================
static void core1_radio_apply_loop(void) {
    // Lets Core0's flash_safe_execute() park this core during flash writes.
    flash_safe_execute_core_init();

    const uint32_t sample_period_us = 1000000u / WAV_SAMPLE_RATE;
    const uint32_t substeps = (DITHER_SUBSTEPS <= 1) ? 1u : (uint32_t)DITHER_SUBSTEPS;
    const uint32_t sub_period_us = (substeps == 1) ? sample_period_us : (sample_period_us / substeps);

#if UNDERRUN_LED_ENABLE
    const uint led_pin = PICO_DEFAULT_LED_PIN;
    if (led_pin != (uint)-1) {
        gpio_init(led_pin);
        gpio_set_dir(led_pin, GPIO_OUT);
        gpio_put(led_pin, 0);
    }
    uint32_t last_und = 0;
    absolute_time_t led_off_time = get_absolute_time();
#endif

    int32_t last_steps = 0x7FFFFFFF;
    int32_t last_p_dbm = 9999;
    bool last_tx_on = false;  // Start with TX off
    bool tx_en_activated = false;  // Track if we've enabled the PA

    while (true) {
        // === CW test / CW mode / TUNE: Core0 owns SPI, Core1 idles ===
        if (g_cw_test_mode) {
            last_tx_on = false;
            last_steps = 0x7FFFFFFF;
            last_p_dbm = 9999;
            // Drain all blocks so Core0 doesn't stall
            for (;;) {
                uint32_t b = g_cons_block;
                if (!g_block_ready[b]) break;
                __compiler_memory_barrier();
                g_block_ready[b] = 0;
                __compiler_memory_barrier();
                g_cons_block = (b + 1u) % NUM_BLOCKS;
            }
            sleep_ms(10);
            continue;
        }

        // === SSB MODE: normal audio processing ===
        // Wait for Core0 to pre-buffer before processing SSB audio
        if (!g_core1_start) {
            sleep_ms(1);
            continue;
        }

        uint32_t b = g_cons_block;

        if (!g_block_ready[b]) {
            g_underruns++;

#if UNDERRUN_LED_ENABLE
            uint32_t und = g_underruns;
            if (und != last_und) {
                last_und = und;
                if (PICO_DEFAULT_LED_PIN != (uint)-1) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 1);
                    led_off_time = make_timeout_time_ms(UNDERRUN_LED_PULSE_MS);
                }
            }
#endif

            uint64_t t0 = time_us_64();
            while (time_us_64() - t0 < sample_period_us) tight_loop_contents();
            continue;
        }

        // Enable TX_EN on first valid block (USB is now stable)
        if (!tx_en_activated) {
            gpio_put(PIN_TX_EN, 1);
            tx_en_activated = true;
            sleep_ms(1);  // Short delay for PA to stabilize
        }

        sample_cmd_t *blk = g_blocks[b];
        uint64_t next_us = time_us_64();

        for (uint32_t i = 0; i < BLOCK_SAMPLES; i++) {
            next_us += sample_period_us;

            for (uint32_t k = 0; k < substeps; k++) {
                sample_cmd_t c = blk[i];

                if ((bool)c.tx_on != last_tx_on) {
                    if (c.tx_on) sx_start_tx_continuous_wave();
#if USE_TCXO_MODULE
                    else         sx_set_standby_xosc();
#else
                    else         sx_set_standby_rc();
#endif
                    last_tx_on = (bool)c.tx_on;
                }

                if (c.freq_steps != last_steps) {
                    sx_set_rf_frequency_steps((uint32_t)c.freq_steps);
                    last_steps = c.freq_steps;
                }

                if ((int32_t)c.p_dbm != last_p_dbm) {
                    sx_set_tx_params_dbm((int32_t)c.p_dbm);
                    last_p_dbm = (int32_t)c.p_dbm;
                }

                if (sub_period_us > 0) {
                    uint64_t target = next_us - (uint64_t)(sample_period_us - (k + 1u) * sub_period_us);
                    while (time_us_64() < target) tight_loop_contents();
                }
            }

            while (time_us_64() < next_us) tight_loop_contents();

#if UNDERRUN_LED_ENABLE
            if (PICO_DEFAULT_LED_PIN != (uint)-1) {
                if (absolute_time_diff_us(get_absolute_time(), led_off_time) <= 0) {
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                }
            }
#endif
        }

        __compiler_memory_barrier();
        g_block_ready[b] = 0;
        __compiler_memory_barrier();

        g_cons_block = (b + 1u) % NUM_BLOCKS;
    }
}

// ==========================================================
// USB audio pump (TinyUSB task + UAC RX read)
// ==========================================================
static void usb_audio_pump(void) {
    // utrzymuj TinyUSB
    tud_task();
    // CDC command handling (if present)
    cdc_task();

    const uint32_t frame_bytes =
        (uint32_t)CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX *
        (uint32_t)CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX;

    if (!frame_bytes) return;

    uint32_t avail = tud_audio_available();
    if (avail < frame_bytes) return;

    static uint8_t tmp[512];

    uint32_t to_read = avail;
    if (to_read > sizeof(tmp)) to_read = sizeof(tmp);
    to_read = (to_read / frame_bytes) * frame_bytes;
    if (!to_read) return;

    uint32_t got = tud_audio_read(tmp, (uint16_t)to_read);
    if (!got) return;

    uint32_t frames = got / frame_bytes;
    const uint8_t *p = tmp;

    for (uint32_t i = 0; i < frames; i++) {
        int16_t l = (int16_t)(p[0] | (p[1] << 8));
        int16_t r = l;
        if (CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX >= 2) {
            r = (int16_t)(p[2] | (p[3] << 8));
        }
        p += frame_bytes;
        (void) usb_rb_push((stereo16_t){ .l = l, .r = r }); // drop if full
    }
}

// Autosave 5 s after the last change, but only while nothing is on the air:
// the ~20 ms flash erase parks Core1, which would freeze the SX1280 mid-word.
static void persist_maybe_autosave(void) {
    if (!g_persist_dirty) return;
    if (g_ui_state != UI_STATE_IDLE) return;                     // still in the menu
    if (g_tune_active || g_ptt_key || g_soft_ptt_key) return;   // carrier / key down
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if ((now - g_persist_dirty_since) < 5000u) return;
    persist_save_now();
}

// ==========================================================
// Boot screen while waiting for the SI5351 clock
// ==========================================================
static void oled_draw_boot_wait(uint32_t elapsed_ms) {
    char line[22];
    ssd1306_clear();
    ssd1306_draw_string(0, 0, "QO-100 TX " FW_VERSION);
    ssd1306_draw_string(0, 1, FW_BUILD);
    ssd1306_draw_string(0, 3, "Waiting for clock");
    ssd1306_draw_string(0, 4, gpsdo_si5351_ok() ? "SI5351: PLL unlocked"
                                                : "SI5351: NOT FOUND");
    snprintf(line, sizeof(line), "%lus", (unsigned long)(elapsed_ms / 1000u));
    ssd1306_draw_string(0, 6, line);
    ssd1306_display(OLED_I2C);
}

// ==========================================================
// MAIN (CORE0): init + DSP producer
// ==========================================================
int main(void) {
    bool ok = set_sys_clock_khz(250000, false);
    if (!ok) set_sys_clock_khz(200000, true);

    // Restore saved config before anything displays or uses it.
    persist_load();

    // ---- USB device init ----
    board_init();
    tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL  // UAC1-only: Full-Speed dla Windows/Linux
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    board_init_after_tusb();

    // ---- SX1280 GPIO/SPI init ----
    // CRITICAL FOR TCXO MODULE: Enable TCXO FIRST, before any SPI/reset!
#if USE_TCXO_MODULE
    gpio_init(PIN_TCXO_EN);
    gpio_set_dir(PIN_TCXO_EN, GPIO_OUT);
    gpio_put(PIN_TCXO_EN, 1);  // Enable TCXO FIRST!
    sleep_ms(5);               // Wait for TCXO to stabilize (min 3ms)
    printf("[SX1280] TCXO enabled (GPIO%d=HIGH)\n", PIN_TCXO_EN);
#endif

    gpio_init(PIN_NSS);   gpio_set_dir(PIN_NSS, GPIO_OUT);   gpio_put(PIN_NSS, 1);
    gpio_init(PIN_RX_EN); gpio_set_dir(PIN_RX_EN, GPIO_OUT); gpio_put(PIN_RX_EN, 0);
    gpio_init(PIN_TX_EN); gpio_set_dir(PIN_TX_EN, GPIO_OUT); gpio_put(PIN_TX_EN, 0);  // Start with TX disabled!
    // SX1280 held in reset until GPSDO is ready (released after gpsdo_is_ready()).
    gpio_init(PIN_RESET); gpio_set_dir(PIN_RESET, GPIO_OUT); gpio_put(PIN_RESET, 0);
    gpio_init(PIN_BUSY);  gpio_set_dir(PIN_BUSY, GPIO_IN);

    // --- Encoder + button GPIO init (input with pull-up, active LOW) ---
    gpio_init(PIN_ENC_A);   gpio_set_dir(PIN_ENC_A, GPIO_IN);   gpio_pull_up(PIN_ENC_A);
    gpio_init(PIN_ENC_B);   gpio_set_dir(PIN_ENC_B, GPIO_IN);   gpio_pull_up(PIN_ENC_B);
    gpio_init(PIN_ENC_OK);  gpio_set_dir(PIN_ENC_OK, GPIO_IN);  gpio_pull_up(PIN_ENC_OK);
    gpio_init(PIN_KEY_DIT); gpio_set_dir(PIN_KEY_DIT, GPIO_IN); gpio_pull_up(PIN_KEY_DIT);
    gpio_init(PIN_KEY_DAH); gpio_set_dir(PIN_KEY_DAH, GPIO_IN); gpio_pull_up(PIN_KEY_DAH);
    // Initialize encoder last state
    enc_last_ab = ((gpio_get(PIN_ENC_A) ? 0 : 1) << 1) | (gpio_get(PIN_ENC_B) ? 0 : 1);

    // --- Sidetone PWM (GP12) — starts silent at mid-scale ---
    sidetone_init();
    // --- Microphone ADC (GP26) — runs continuously, used only when src=mic ---
    mic_init();

    spi_init(SX_SPI, SX_SPI_BAUD);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);

    // --- OLED I2C init (early — before GPSDO wait so display is always on) ---
    sleep_ms(20);  // Allow display VCC to stabilise before sending I2C init sequence
    i2c_init(OLED_I2C, OLED_I2C_BAUD);
    gpio_set_function(PIN_OLED_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_OLED_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_OLED_SDA);
    gpio_pull_up(PIN_OLED_SCL);
    ssd1306_init(OLED_I2C);
    oled_draw_boot_wait(0u);

    // ---- GPSDO: SI5351 52 MHz (I2C0, GP0/GP1) + NEO-7M GPS (UART1, GP4/GP5) ----
    // Stage 1 (blocking): wait for a locked 52 MHz clock from the SI5351. The
    // SX1280 has no other clock source, so NRESET stays LOW until then.
    // Stage 2 (non-blocking): GPS discipline (UTC) only gates TX via
    // tx_allowed() — the UI, CDC and OLED run normally while GPS acquires.
    gpsdo_init();
    {
        const uint32_t boot_ms = to_ms_since_boot(get_absolute_time());
        uint32_t last_status_ms  = 0u;
        uint32_t last_oled_ms    = 0u;
        uint8_t  si_warn_printed = 0u;
        while (!gpsdo_clock_ok()) {
            gpsdo_task();
            tud_task();
            cdc_task();   // allow 'gpsdo' / 'version' while waiting
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            // 1 s cadence: ssd1306_display() blocks ~100 ms on I2C at 100 kHz,
            // and only the seconds counter changes anyway.
            if ((now_ms - last_oled_ms) >= 1000u) {
                last_oled_ms = now_ms;
                oled_draw_boot_wait(now_ms - boot_ms);
            }
            if (tud_cdc_connected()) {
                if (!si_warn_printed && !gpsdo_si5351_ok()) {
                    si_warn_printed = 1u;
                    cdc_write_str("GPSDO: WARNING — SI5351 not found (GP0/GP1).\r\n"
                                  "GPSDO: SX1280 has no clock and will NOT start until SI5351 is connected.\r\n");
                }
                if ((now_ms - last_status_ms) >= 2000u) {
                    last_status_ms = now_ms;
                    char gbuf[128];
                    gpsdo_format_status(gbuf, sizeof(gbuf));
                    tud_cdc_write(gbuf, strlen(gbuf));
                    tud_cdc_write_flush();
                }
            }
        }
        if (tud_cdc_connected()) {
            cdc_write_str("GPSDO: 52 MHz clock locked — starting SX1280. "
                          "TX stays blocked until GPS UTC is valid (see 'gpsgate').\r\n");
        }
    }

    // Release SX1280 from reset (was held LOW during GPSDO init) and initialise
    printf("[SX1280] Resetting...\n");
    gpio_put(PIN_RESET, 0); sleep_ms(2);
    gpio_put(PIN_RESET, 1); sleep_ms(10);
    printf("[SX1280] Reset complete, BUSY=%d\n", gpio_get(PIN_BUSY));

#if USE_TCXO_MODULE
    // For TCXO module, use STDBY_XOSC mode
    sx_set_standby_xosc();
    printf("[SX1280] Set STDBY_XOSC mode (for TCXO)\n");
#else
    sx_set_standby_rc();
    printf("[SX1280] Set STDBY_RC mode\n");
#endif

    sx_set_packet_type_gfsk();

    // Use runtime-configurable frequency with PPM correction
    uint32_t init_base_steps = get_base_steps();
    sx_set_rf_frequency_steps(init_base_steps);

    sx_set_tx_params_dbm((int32_t)PWR_MIN_DBM);  // Start with minimum power
    // TX_EN stays LOW - will be controlled by Core1 or CDC commands
    // Use 'cw' command via CDC to test transmission
    // DON'T start TX yet - wait for USB to enumerate and audio to start
    // sx_start_tx_continuous_wave();  -- moved to core1 when audio starts

#if FIXED_POWER_CW_MODE
    // For CW mode, wait for USB then start TX
    while (!tud_ready()) { tud_task(); sleep_ms(10); }
    sleep_ms(500);  // Extra delay for USB stability
    gpio_put(PIN_TX_EN, 1);
    sx_start_tx_continuous_wave();
    while (true) { tud_task(); tight_loop_contents(); }
#endif

    // *** Start Core1 early so it can idle and drain blocks immediately ***
    multicore_launch_core1(core1_radio_apply_loop);

    // Wait for USB — meanwhile handle encoder, buttons, OLED, CW keying
    {
        printf("[BOOT] Waiting for USB connection (encoder+CW active)...\n");
        absolute_time_t oled_next = {0};
        // Powerbank / standalone: no host within 3 s → carry on without one.
        // TinyUSB keeps running, so a PC plugged in later still enumerates.
        absolute_time_t usb_deadline = make_timeout_time_ms(3000);

        while (!tud_ready()) {
            if (absolute_time_diff_us(get_absolute_time(), usb_deadline) <= 0) {
                printf("[BOOT] No USB host — standalone operation\n");
                break;
            }
            tud_task();
            encoder_poll();
            button_poll();
            carrier_poll();

            // OLED refresh during wait
            if (!ssd1306_dma_busy() &&
                absolute_time_diff_us(get_absolute_time(), oled_next) <= 0) {
                oled_prepare_frame();
                ssd1306_display_dma(OLED_I2C);
                oled_next = make_timeout_time_ms(200);
            }

            sleep_ms(1);
        }
        printf("[BOOT] USB connected, starting normal SSB mode\n");
    }

    hilbert_init();

    const float Fs = (float)WAV_SAMPLE_RATE;

    float theta_prev = 0.0f;
    float f_acc = 0.0f;
    float fine_tune_phase = 0.0f;  // Phase accumulator for fine frequency tuning

    float p_acc = 0.0f;
    float tx_acc = 0.0f;

    const float phi = (float)IQ_PHASE_CORR_DEG * (float)M_PI / 180.0f;
    const float cphi = cosf(phi);
    const float sphi = sinf(phi);

#if AUDIO_ENABLE_BANDPASS
    biquad_t bp_hpf[AUDIO_BP_MAX_STAGES];
    biquad_t bp_lpf[AUDIO_BP_MAX_STAGES];
    // init via apply_cfg_if_dirty()
#endif

#if AUDIO_ENABLE_EQ
    biquad_t eq_low, eq_high;
    // init via apply_cfg_if_dirty()
#endif

#if AUDIO_ENABLE_COMPRESSOR
    compressor_t comp;
    // init via apply_cfg_if_dirty()
#endif

#if USE_TEST_TONE
    float sine_phase1 = 0.0f;
    const float sine_inc1 = 2.0f * (float)M_PI * (float)TEST_TONE_HZ / Fs;
#if USE_TWO_TONE_TEST
    float sine_phase2 = 0.0f;
    const float sine_inc2 = 2.0f * (float)M_PI * (float)TEST_TONE2_HZ / Fs;
#endif
#endif

    audio_cfg_t cfg_local;
    memset(&cfg_local, 0, sizeof(cfg_local));

    // PRE-BUFFERING: fill half of the blocks before starting consumer
    // This prevents underruns at startup
    const uint32_t prebuf_blocks = NUM_BLOCKS / 2;
    
    // silence reset counter
    const uint32_t silence_samples = WAV_SAMPLE_RATE * SILENCE_SECONDS;
    uint32_t silence_ctr = 0;

    // greet once if CDC is connected later
    uint8_t greeted = 0;

    // *** Pre-fill some blocks before signaling Core1 to start ***
    const uint32_t prebuf_target = NUM_BLOCKS / 2;  // Fill half the buffer
    uint32_t prebuf_count = 0;

    while (true) {
        uint32_t b = g_prod_block;

        // === CW/TUNE active: Core1 just drains blocks, no DSP needed ===
        // Run a tight poll loop instead of producing audio blocks.
        // Without this, Core1 drains so fast that the wait loop below
        // never executes, starving encoder/button/cw_keying polls.
        if (g_cw_test_mode) {
            usb_audio_pump();

            {
                static absolute_time_t oled_next_cw = {0};
                if (!ssd1306_dma_busy() &&
                    absolute_time_diff_us(get_absolute_time(), oled_next_cw) <= 0) {
                    oled_prepare_frame();
                    ssd1306_display_dma(OLED_I2C);
                    oled_next_cw = make_timeout_time_ms(200);
                }
            }

            encoder_poll();
            button_poll();
            carrier_poll();
            persist_maybe_autosave();

#if CFG_TUD_CDC
            cdc_task();
            cdc_status_push();
#endif
            tight_loop_contents();
            continue;   // Skip block production entirely
        }

        while (g_block_ready[b]) {
            usb_audio_pump();

            // --- OLED update via DMA (zero CPU during transfer) ---
            // Render + kick DMA during idle time. DMA feeds I2C TX FIFO
            // in background — CPU is completely free for USB/DSP.
            {
                static absolute_time_t oled_next = {0};

                if (!ssd1306_dma_busy() &&
                    absolute_time_diff_us(get_absolute_time(), oled_next) <= 0) {
                    oled_prepare_frame();
                    ssd1306_display_dma(OLED_I2C);
                    oled_next = make_timeout_time_ms(200);  // ~5 fps
                }
            }

            // Poll encoder + buttons + carrier state machine
            encoder_poll();
            button_poll();
            carrier_poll();
            persist_maybe_autosave();

#if CFG_TUD_CDC
            cdc_status_push();
#endif

            tight_loop_contents();
        }

#if CFG_TUD_CDC
        if (!greeted && tud_cdc_connected()) {
            greeted = 1;
            cdc_write_str("\r\nSX1280_SDR control ready. FW " FW_VERSION " built " FW_BUILD ". Type 'help'.\r\n");
            cdc_write_str(g_persist_loaded ? "Config: restored from flash.\r\n"
                                           : "Config: compile-time defaults (nothing valid in flash).\r\n");
            cfg_print();
        }

        // GPSDO: service GPS UART and send periodic status (every 10 s).
        gpsdo_task();
        if (gpsdo_status_due()) {
            char gbuf[128];
            gpsdo_format_status(gbuf, sizeof(gbuf));
            cdc_write_str(gbuf);
        }
#endif

        // Apply pending cfg on block boundary
#if AUDIO_ENABLE_BANDPASS || AUDIO_ENABLE_EQ || AUDIO_ENABLE_COMPRESSOR
        apply_cfg_if_dirty(
            Fs,
#if AUDIO_ENABLE_BANDPASS
            bp_hpf, bp_lpf,
#else
            NULL, NULL,
#endif
#if AUDIO_ENABLE_EQ
            &eq_low, &eq_high,
#else
            NULL, NULL,
#endif
#if AUDIO_ENABLE_COMPRESSOR
            &comp,
#else
            NULL,
#endif
            &cfg_local
        );
#else
        // still snapshot cfg (amp, etc.)
        __compiler_memory_barrier();
        memcpy(&cfg_local, (const void*)&g_cfg, sizeof(cfg_local));
        __compiler_memory_barrier();
#endif

        // Get current base steps (with freq and PPM correction) at block boundary
        int32_t base_steps = (int32_t)get_base_steps();

        // --- Audio source for this block ---
        const uint8_t src = g_audio_src;
        {
            static uint8_t last_src = AUDIO_SRC_PC;
            if (src != last_src) {
                if (src == AUDIO_SRC_MIC) mic_flush();   // drop whatever piled up while unused
                last_src = src;
            }
        }
        if (src == AUDIO_SRC_MIC) {
            // Paced by the ADC: wait for a full block of mic samples, keeping
            // USB/UI/keyer alive meanwhile. Bail out if the mode or source changes.
            mic_dma_rearm_if_done();
            bool bail = false;
            while (mic_available() < BLOCK_SAMPLES) {
                usb_audio_pump();
                encoder_poll();
                button_poll();
                carrier_poll();
                persist_maybe_autosave();
                if (g_cw_test_mode || g_audio_src != AUDIO_SRC_MIC) { bail = true; break; }
                tight_loop_contents();
            }
            if (bail) continue;
        }

        sample_cmd_t *blk = g_blocks[b];

        for (uint32_t n = 0; n < BLOCK_SAMPLES; n++) {
            if ((n & 0x07u) == 0u) usb_audio_pump();

            float x = 0.0f;

#if USE_TEST_TONE
#if USE_TWO_TONE_TEST
            // Two-tone test: sum of two sinusoids
            x = TEST_TONE_AMPL * (sinf(sine_phase1) + sinf(sine_phase2));
            sine_phase1 += sine_inc1;
            sine_phase2 += sine_inc2;
            if (sine_phase1 > 2.0f * (float)M_PI) sine_phase1 -= 2.0f * (float)M_PI;
            if (sine_phase2 > 2.0f * (float)M_PI) sine_phase2 -= 2.0f * (float)M_PI;
#else
            x = TEST_TONE_AMPL * sinf(sine_phase1);
            sine_phase1 += sine_inc1;
            if (sine_phase1 > 2.0f * (float)M_PI) sine_phase1 -= 2.0f * (float)M_PI;
#endif
#else
            if (src == AUDIO_SRC_MIC) {
                x = mic_get_sample();
            } else {
                int16_t s = usb_audio_get_mono_8k();
                x = (float)s / 32768.0f;
            }

            if (fabsf(x) < 1e-5f) {
                if (silence_ctr < silence_samples) silence_ctr++;
            } else {
                silence_ctr = 0;
            }

            if (silence_ctr == silence_samples) {
                hilbert_reset();
                theta_prev = 0.0f;
                f_acc = 0.0f;
                fine_tune_phase = 0.0f;
                p_acc = 0.0f;
                tx_acc = 0.0f;

#if AUDIO_ENABLE_BANDPASS
                for (int i = 0; i < AUDIO_BP_MAX_STAGES; i++) {
                    biquad_reset(&bp_hpf[i]);
                    biquad_reset(&bp_lpf[i]);
                }
#endif
#if AUDIO_ENABLE_EQ
                biquad_reset(&eq_low);
                biquad_reset(&eq_high);
#endif
#if AUDIO_ENABLE_COMPRESSOR
                comp.env = 0.0f;
#endif
                silence_ctr = silence_samples + 1u;
            }
#endif

#if AUDIO_ENABLE_EQ
            if (cfg_local.enable_eq) {
                x = biquad_process(&eq_low,  x);
                x = biquad_process(&eq_high, x);
            }
#endif

#if AUDIO_ENABLE_COMPRESSOR
            if (cfg_local.enable_comp) {
                x = compressor_process(&comp, x);
                // output limiter
                if (x > cfg_local.comp_out_limit) x = cfg_local.comp_out_limit;
                if (x < -cfg_local.comp_out_limit) x = -cfg_local.comp_out_limit;
            }
#endif

#if AUDIO_ENABLE_BANDPASS
            if (cfg_local.enable_bandpass) {
                for (int i = 0; i < cfg_local.bp_stages; i++) x = biquad_process(&bp_hpf[i], x);
                for (int i = 0; i < cfg_local.bp_stages; i++) x = biquad_process(&bp_lpf[i], x);
            }
#endif

            float I;
            float Q = hilbert_process(x, &I);

            float Iq = I;
            float Qq = Q * (float)IQ_GAIN_CORR;

            float I2 = Iq * cphi - Qq * sphi;
            float Q2 = Iq * sphi + Qq * cphi;

            // Apply fine frequency tuning via complex carrier multiplication
            // Fine tune is calculated automatically from fractional Hz that PLL can't reach
            float fine_hz = get_fine_tune_hz();  // Auto-calculated from target freq + PPM
            if (fine_hz != 0.0f) {
                float fine_cos = cosf(fine_tune_phase);
                float fine_sin = sinf(fine_tune_phase);
                float I3 = I2 * fine_cos - Q2 * fine_sin;
                float Q3 = I2 * fine_sin + Q2 * fine_cos;
                I2 = I3;
                Q2 = Q3;
                fine_tune_phase += 2.0f * (float)M_PI * fine_hz / Fs;
                // Keep phase in [-π, π] to avoid precision loss
                if (fine_tune_phase > (float)M_PI)   fine_tune_phase -= 2.0f * (float)M_PI;
                if (fine_tune_phase < -(float)M_PI) fine_tune_phase += 2.0f * (float)M_PI;
            }

            float A = sqrtf(I2 * I2 + Q2 * Q2);

            float theta = atan2f(Q2, I2);

            float dtheta = theta - theta_prev;
            if (dtheta > (float)M_PI)   dtheta -= 2.0f * (float)M_PI;
            if (dtheta < -(float)M_PI) dtheta += 2.0f * (float)M_PI;
            theta_prev = theta;

            float f_off = dtheta * Fs / (2.0f * (float)M_PI);
            if (f_off > (float)F_OFF_LIMIT_HZ)  f_off = (float)F_OFF_LIMIT_HZ;
            if (f_off < -(float)F_OFF_LIMIT_HZ) f_off = -(float)F_OFF_LIMIT_HZ;

            float want_steps = f_off / PLL_STEP_HZ;
            int32_t Nf = (int32_t)floorf(want_steps);
            float ffrac = want_steps - (float)Nf;

            f_acc += ffrac;
            int32_t f_chosen = Nf;
            if (f_acc >= 1.0f) { f_chosen = Nf + 1; f_acc -= 1.0f; }

            int32_t cur_steps = base_steps + f_chosen;

            float duty = duty_from_A(A);

            int32_t p_chosen = PWR_MIN_DBM;
            uint8_t tx_on = 1;

            if (duty < 1.0f) {
                p_chosen = PWR_MIN_DBM;
                tx_acc += duty;
                if (tx_acc >= 1.0f) { tx_on = 1; tx_acc -= 1.0f; }
                else                { tx_on = 0; }
            } else {
                tx_on = 1;

                int8_t pwr_max = g_tx_power_max_dbm;  // Local copy for this sample
                float Aeff = A * cfg_local.amp_gain;
                if (Aeff < cfg_local.amp_min_a) Aeff = cfg_local.amp_min_a;

                float p_raw = (float)pwr_max + 20.0f * log10f(Aeff);

                float p_des = p_raw;
                if (p_des > (float)pwr_max) p_des = (float)pwr_max;
                if (p_des < (float)PWR_MIN_DBM) p_des = (float)PWR_MIN_DBM;

                int32_t p_low  = (int32_t)floorf(p_des);
                int32_t p_high = p_low + 1;

                if (p_low  < PWR_MIN_DBM) p_low  = PWR_MIN_DBM;
                if (p_high > pwr_max) p_high = pwr_max;

                float frac = p_des - (float)p_low;
                if (frac < 0.0f) frac = 0.0f;
                if (frac > 1.0f) frac = 1.0f;

                p_acc += frac;
                p_chosen = p_low;
                if (p_acc >= 1.0f && p_high != p_low) { p_chosen = p_high; p_acc -= 1.0f; }
            }

            // Global TX enable (GUI TX button) and GPS gate
            if (!g_tx_enabled || !tx_allowed()) {
                tx_on = 0;
            }

            blk[n].freq_steps = cur_steps;
            blk[n].p_dbm      = (int8_t)p_chosen;
            blk[n].tx_on      = tx_on;
        }

        __compiler_memory_barrier();
        g_block_ready[b] = 1;
        __compiler_memory_barrier();

        g_prod_block = (b + 1u) % NUM_BLOCKS;

        // *** Signal Core1 to start after pre-buffering ***
        if (!g_core1_start) {
            prebuf_count++;
            if (prebuf_count >= prebuf_target) {
                __compiler_memory_barrier();
                g_core1_start = 1;
                __compiler_memory_barrier();
            }
        }
    }
}

// ==========================================================
// TinyUSB HID callbacks (stubs) – only compiled when HID is enabled
// ==========================================================
#if CFG_TUD_HID
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t* buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}
#endif /* CFG_TUD_HID */
