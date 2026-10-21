#ifndef SI4732_H
#define SI4732_H

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Si4732-A10 receive demodulator, on the I2C0 bus shared with the SI5351.
//
// The chip is a broadcast AM/FM/SW receiver. SSB is not in its ROM: about 15 KB
// of firmware has to be pushed in over I2C after every power-up, which is what
// tools/fetch_ssb_patch.sh provides. Without that file the driver still builds
// and still receives AM, but si4732_has_ssb() returns false and the sideband
// modes are refused.
//
// Its reference clock is the GPS-disciplined 25 MHz from the second GPSDO on
// pin 13 (RCLK), so the receive tuning carries the same accuracy as the
// transmit side — that is the whole reason for not using the usual 32.768 kHz
// watch crystal, which would be ±20 ppm, or ±390 Hz at a 19.5 MHz IF.
//
// si4732_init() must run AFTER gpsdo_init(), which brings up I2C0.
// ---------------------------------------------------------------------------

typedef enum {
    SI4732_MODE_AM = 0,
    SI4732_MODE_LSB,
    SI4732_MODE_USB,
} si4732_mode_t;

// Probe the bus, reset the chip, read its revision. Does not power up the
// receiver — call si4732_set_mode() for that. Returns false if nothing
// answered, in which case every other call is a no-op and the rest of the
// firmware carries on unaffected.
bool     si4732_init(void);

bool     si4732_present(void);
uint8_t  si4732_i2c_addr(void);      // 0x11 or 0x63; 0 when absent
bool     si4732_has_ssb(void);       // patch compiled in

// Powers up in the requested mode, uploads the patch for the sideband modes,
// restores volume, bandwidth and BFO, and retunes.
bool     si4732_set_mode(si4732_mode_t mode);
si4732_mode_t si4732_get_mode(void);

void     si4732_power_down(void);
bool     si4732_is_on(void);

bool     si4732_tune_khz(uint32_t khz);   // 2300..26100 (SW band)
uint32_t si4732_get_khz(void);

bool     si4732_set_bfo_hz(int16_t hz);   // fine tuning, 8 Hz steps
int16_t  si4732_get_bfo_hz(void);

bool     si4732_set_volume(uint8_t vol);  // 0..63
uint8_t  si4732_get_volume(void);

// Nearest supported filter is chosen; si4732_get_bandwidth_hz() reports what
// was actually set, not what was asked for.
bool     si4732_set_bandwidth_hz(uint16_t hz);
uint16_t si4732_get_bandwidth_hz(void);

bool     si4732_set_mute(bool mute);
bool     si4732_get_mute(void);

// Received signal quality. rssi is in dBµV, snr in dB. Either may be NULL.
bool     si4732_read_rsq(uint8_t *rssi_dbuv, uint8_t *snr_db);

const char *si4732_mode_name(si4732_mode_t mode);

// Where the driver's messages go. Both stdio backends are disabled in this
// build (see CMakeLists.txt), so printf goes nowhere — the caller hands us a
// sink that reaches the console instead. Without one, messages are dropped.
typedef void (*si4732_log_fn)(const char *line);
void     si4732_set_log(si4732_log_fn fn);

// Multi-line report through the log sink.
void     si4732_diag_print(void);

#endif // SI4732_H
