#ifndef SI4732_H
#define SI4732_H

#include <stdbool.h>
#include <stddef.h>
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

// Tuning. The chip itself only accepts whole kilohertz, which is useless for
// SSB — a kilohertz of error puts every voice a kilohertz off pitch. So the
// Hz-resolution entry point rounds to the nearest kHz for the tuner and puts
// the remainder into the BFO, which steps in 8 Hz. Prefer it over the kHz form.
bool     si4732_tune_hz(uint32_t hz);
uint32_t si4732_get_hz(void);

bool     si4732_tune_khz(uint32_t khz);   // 2300..26100 (SW band)
uint32_t si4732_get_khz(void);

// The LNB's local-oscillator error, in Hz, as the amount by which everything
// arrives HIGH of where it should. Measure it on the QO-100 beacon: the lower
// CW beacon at 10489.500 MHz should land at 739.500 MHz out of a 9750 MHz LNB,
// so a beacon seen at 739.512 MHz means +12000.
//
// The Bullseye LNB cannot take an external reference — its 25 MHz connector is
// an output — so this offset is what stands in for GPS discipline on the
// receive side. Its TCXO is ±2 ppm, roughly ±10 kHz at 23 °C and ±30 kHz over
// the full temperature range, so one calibration holds for a session.
// Tunes to a frequency and averages `samples` quality readings, reporting the
// mean RSSI plus the peak RSSI and SNR. A single read swings far too much to
// compare frequencies with: the same spot gave 64 and then 0 seconds apart.
bool     si4732_probe_khz(uint32_t khz, uint8_t samples,
                          uint8_t *rssi_avg, uint8_t *rssi_max, uint8_t *snr_max);

// Why the last call failed. Never NULL.
const char *si4732_last_error(void);

bool     si4732_set_cal_hz(int32_t hz);
int32_t  si4732_get_cal_hz(void);

// XOSCEN bit of POWER_UP. false (the default) means "external clock on RCLK,
// crystal amplifier off", which is what this design uses and what AM tunes
// with. The reference library passes true and calls it a condition for loading
// the SSB patch, so this is switchable while that is unresolved. Takes effect
// on the next power-up.
void     si4732_set_xoscen(bool on);
bool     si4732_get_xoscen(void);

// Antenna tuning capacitor, sent with every tune. Defaults to 1, as the
// reference library does for every AM and SSB tune.
//
// 0 means "tune the varactor yourself". The ROM firmware can; the SSB patch
// cannot, and asking it to leaves the chip unable to finish another command
// until the next power-up - which is what "SSB does not tune" was, for weeks.
// Leave it alone unless you are measuring: in AM it makes no difference worth
// having on a 50 ohm feed from a mixer, which has no inductance to resonate
// against anyway. The capacitance is 95 fF x ANTCAP + 7 pF, so 1 is about
// 7.1 pF. Range 0..6143.
bool     si4732_set_antcap(uint16_t cap);
uint16_t si4732_get_antcap(void);

// Adopt saved settings without touching the chip. Called at boot, before the
// receiver has even been probed — the first si4732_set_mode() then applies all
// of it at once. Values are range-checked and silently replaced by the
// defaults where they are not plausible.
void     si4732_restore(si4732_mode_t mode, uint32_t hz, uint8_t vol,
                        uint16_t bw_hz, int32_t cal_hz);

// Raw access to the beat frequency offset. Normally you do not want this:
// si4732_tune_hz() owns the BFO and uses it for the sub-kilohertz part of the
// tuning, so the next tune overwrites whatever is set here. It is kept for
// checking the sign convention on the beacon.
bool     si4732_set_bfo_hz(int16_t hz);
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

// Front-end AGC. The index is an ATTENUATION: 0 is maximum gain, and the AM
// AGC responds to the whole input rather than the tuned channel, so a single
// strong carrier anywhere in band turns the front end down for everything.
// Reading it separates "nothing is arriving" from "the receiver has been made
// deaf", which look identical in the RSSI alone.
bool si4732_read_agc(bool *disabled, uint8_t *index);

// Override it. disable=true freezes the attenuation at `index`; disable=false
// hands control back to the chip and ignores the index.
bool si4732_set_agc(bool disable, uint8_t index);

// Reference clock on RCLK, in Hz. The prescaler is chosen for an exact
// division into the 31130..34406 Hz window the chip accepts — inexact would
// not be a rounding detail, it would scale every tuned frequency. Rejected,
// leaving the old value in place, when no such divisor exists.
//
// Changing this does NOT change what the SI5351 puts on the pin. The caller
// moves both ends; gpsdo_set_rclk_mhz() is the other one.
bool     si4732_set_rclk_hz(uint32_t hz);
uint32_t si4732_rclk_hz(void);
uint16_t si4732_rclk_prescale(void);

const char *si4732_mode_name(si4732_mode_t mode);

// Where the driver's messages go. Both stdio backends are disabled in this
// build (see CMakeLists.txt), so printf goes nowhere — the caller hands us a
// sink that reaches the console instead. Without one, messages are dropped.
typedef void (*si4732_log_fn)(const char *line);
void     si4732_set_log(si4732_log_fn fn);

// Multi-line report through the log sink.
void     si4732_diag_print(void);

// Walks the SSB bring-up one command at a time and reports the chip's status
// byte after each, ERR bit included. For the open question of why the part
// stops answering after an SSB tune while AM is fine on the same hardware.
// Leaves the receiver powered up in USB, however far it got.
void     si4732_ssb_bringup_test(void);

// Send an arbitrary command and dump the response. For the bench, when the
// question is "does this command want a different argument or a different
// response length" - which is not a question worth a build and a flash each
// time. rlen 0 sends the command and reads nothing.
void     si4732_raw(const uint8_t *cmd, size_t clen, size_t rlen);

#endif // SI4732_H
