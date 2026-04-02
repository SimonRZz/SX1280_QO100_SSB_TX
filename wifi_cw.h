/* wifi_cw.h  –  WiFi (STA/AP), UDP cwdaemon :6789, HTTP control UI :80
 *
 * Core 0: wifi_cw_init(), wifi_cw_task()
 * Core 1: wifi_cw_core1_keyer_tick()   (called from Core1 CW idle path)
 *
 * Board requirement: pico2_w (CYW43439 WiFi).
 * CMake: pico_cyw43_arch_lwip_threadsafe_background, hardware_flash,
 *        hardware_watchdog
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/util/queue.h"

/* ---- Flash layout -------------------------------------------------------
 * Credentials are stored in the second-to-last 4 KB sector of flash so they
 * survive normal firmware updates (which erase from address 0 upward).
 * RP2350 / Pico2W: 2 MB flash = 0x200000; FLASH_SECTOR_SIZE = 4096.
 */
#define WIFI_CREDS_MAGIC        0x57494649u   /* "WIFI" */
#define WIFI_CREDS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    char     ssid[64];
    char     password[64];
    uint8_t  _pad[FLASH_SECTOR_SIZE - 4 - 64 - 64];
} wifi_creds_t;

/* ---- CW queue depth (chars) -------------------------------------------- */
#define WIFI_CW_QUEUE_DEPTH  128

/* ---- UDP cwdaemon port -------------------------------------------------- */
#define WIFI_CW_UDP_PORT     6789

/* ---- HTTP port ---------------------------------------------------------- */
#define WIFI_HTTP_PORT       80

/* =========================================================================
 * Public API
 * =========================================================================
 *
 * Call wifi_cw_init() once from Core 0 after all hardware init is done
 * (after SX1280 and GPSDO are initialised, before multicore_launch_core1).
 *
 * cw_queue        – queue_t (sizeof element = 1 byte) for CW characters
 * p_soft_ptt_key  – &g_soft_ptt_key   (volatile uint8_t)
 * p_tx_mode       – &g_tx_mode        (0=USB, 1=CW)
 * p_cw_test_mode  – &g_cw_test_mode   (set by carrier_poll when Core1 idles)
 * p_tx_power_dbm  – &g_tx_power_max_dbm
 * p_target_freq   – &g_target_freq_hz  (Hz, double)
 * p_tx_enabled    – &g_tx_enabled
 * p_ppm           – &g_ppm_correction
 */
void wifi_cw_init(queue_t          *cw_queue,
                  volatile uint8_t *p_soft_ptt_key,
                  volatile uint8_t *p_tx_mode,
                  volatile uint8_t *p_cw_test_mode,
                  volatile int8_t  *p_tx_power_dbm,
                  volatile double  *p_target_freq,
                  volatile uint8_t *p_tx_enabled,
                  volatile float   *p_ppm);

/* Call from Core 0 main loop (all polling paths).
 * Processes deferred actions: flash save → reboot, /set parameter changes. */
void wifi_cw_task(void);

/* Call from Core 1 CW idle path (inside the g_cw_test_mode == 1 branch).
 * Non-blocking state machine: reads queue, times elements, sets soft PTT. */
void wifi_cw_core1_keyer_tick(void);

/* Returns the current IP string:
 *   client mode → "1.2.3.4"
 *   AP mode     → "AP:192.168.4.1"
 *   not init    → ""
 */
const char *wifi_cw_ip_str(void);
