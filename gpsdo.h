// gpsdo.h — GPSDO integration header for SX1280_QO100_SSB_TX
//
// Provides GPS-disciplined 52 MHz reference via SI5351 clock generator
// locked to a u-blox NEO-7M GPS module.
//
// Hardware (all pins free in the SX1280 master design):
//   SI5351  → I2C0   GP0 = SDA, GP1 = SCL
//   NEO-7M  → UART1  GP4 = TX (→ GPS RX), GP5 = RX (← GPS TX)
//   SX1280 NRESET (GP20) is held LOW by main.c until gpsdo_is_ready().

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Initialise I2C0, SI5351 (52 MHz on CLK1) and UART1 (GPS at 9600 baud).
// Sends UBX-CFG-TP5 and UBX-CFG-NAV5 to the NEO-7M.
// Returns immediately after hardware setup — call gpsdo_task() in a loop
// until gpsdo_is_ready().
void gpsdo_init(void);

// Service the GPS UART: drain incoming bytes, parse NMEA sentences,
// update internal state. Call this frequently (every few ms is fine).
void gpsdo_task(void);

// Returns true while the SI5351 CLK1 (52 MHz) is running and its PLL is
// locked to the GPS TIMEPULSE — i.e. the SX1280 has a usable clock.
// This is the condition for releasing the SX1280 reset.  It does NOT
// imply GPS discipline: without a satellite the NEO-7M free-runs at
// ±2.5 ppm (≈ ±6 kHz at 2.4 GHz).
bool gpsdo_clock_ok(void);

// Returns true once gpsdo_clock_ok() AND the GPS has delivered a UTC
// timestamp, meaning the 24 MHz TIMEPULSE is satellite-disciplined.
// This is the condition for allowing transmission.
bool gpsdo_is_ready(void);

// Returns true if the SI5351 was found and CLK1 is running.
// False means SI5351 is not connected or not responding.
bool gpsdo_si5351_ok(void);

// Fill buf with the machine-parseable status line:
//   "GPSDO: lock=<0|1> sats=<N> clk1=<ok|fail>\r\n"
// Returns the number of characters written (excluding the NUL terminator).
int gpsdo_format_status(char *buf, size_t size);

// Returns true once every GPSDO_PRINT_MS milliseconds (2 s default).
// Use this to rate-limit periodic status output.
bool gpsdo_status_due(void);
