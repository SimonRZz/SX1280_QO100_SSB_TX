# SX1280 QO-100 SSB TX

**SSB/CW transmitter for the QO-100 satellite (Es'hail 2) — Raspberry Pi Pico 2 + SX1280, with GPS-disciplined reference and standalone operation**

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

## Demo

📺 **Video test:** [YouTube Short - SSB TX Test](https://www.youtube.com/shorts/xTy9VHoNrlg)

![Waterfall signal from SX1280 with external PA](img/waterfall1.png)
*SSB signal on QO-100 waterfall (with external amplifier)*

## Project Description

SSB (Single Sideband) and CW transmitter for the 2.4 GHz band, designed for the narrowband
transponder of the geostationary satellite **QO-100 (Es'hail 2)**.

A **dual-core architecture** splits the work: Core0 handles USB audio or the microphone,
the DSP chain and the user interface, while Core1 replays each block to the SX1280 over SPI
at an exact 8 kHz rate — a polar modulator that FMs the carrier with the phase derivative and
AMs it through the power register.

This fork extends SP8ESA's original design towards a **transmitter that works with or without
a computer**: a GPS-disciplined 52 MHz reference, an iambic keyer running in the firmware,
a microphone input, a sidetone on headphones, and an OLED with rotary-encoder control.

### Features

**Transmit**
- **SSB (USB) and CW**, up to +27 dBm with the PA in the LoRa1280F27 module
- **Sub-Hz frequency precision** — automatic split into PLL steps + DSP fine offset, no 198 Hz quantization
- **Real-time DSP** — bandpass filter, shelving equalizer, compressor with limiter, power shaping
- **Click-free CW keying** — the carrier ramps 1 dB per 160 µs instead of switching hard

**Reference**
- **GPSDO** — SI5351 generates 52 MHz for the SX1280, locked to the 24 MHz TIMEPULSE of a u-blox NEO-7M
- **Two-stage gating** — the radio starts as soon as the 52 MHz clock is locked; transmission is
  released only once GPS delivers UTC, because an undisciplined NEO-7M drifts ±2.5 ppm (≈ ±6 kHz at 2.4 GHz)
- **Lost-fix warning** — display and GUI flag a fix that was present and is gone

**Audio**
- **USB Audio** — the Pico acts as a USB sound card for the PC
- **Microphone input** — MAX4466 on ADC0, sampled at exactly 8 kHz by the ADC and moved by DMA;
  DC removal, gain and a noise gate with hold
- **Source switching** — PC (USB) or MIC, from the GUI, the OLED menu, or `src pc|mic`
- **Sidetone on headphones** — PWM on GP12, DMA-fed at 48 kHz with a 5 ms raised-cosine envelope
  matching the RF ramp; adjustable pitch and volume

**Standalone operation**
- **On-device iambic keyer** — Straight / Iambic A / Iambic B, 5–60 WPM, adjustable dah ratio,
  contact-bounce filter, and a Morse decoder that reports what was keyed
- **OLED display (SSD1306)** — three pages: operating, CW/audio, GPS/time; rotatable 180°
- **Rotary encoder** — frequency tuning plus six navigable tiles per page
- **Settings survive power-off** — stored in flash with CRC32, autosaved once the radio is idle
- **Boots without a PC** — no waiting for a USB host

**Control**
- **Python GUI** — RF/DSP, TX control, CW keyer with PC-side sidetone, GPSDO status, and a
  debugging console with timestamps, filters and log export
- **USB CDC** — full command set over a serial port

## Author and credits

**Original project: Kacper Kidała SP8ESA** — the SSB/DSP core, the dual-core architecture and
the SX1280 driver are his work. Upstream: [SP8ESA/QOkka-DSP](https://github.com/SP8ESA/QOkka-DSP).

**GPSDO principle: CT2GQV** — based on
[SI5351 GPS disciplined oscillator](https://speakyssb.blogspot.com/2019/10/si5351-gps-disciplined-oscillator-with.html).
The Arduino sketch was ported to the Pico SDK and extended with SI5351 address auto-detection,
GPS baud auto-detection, PLL status monitoring, multi-constellation GSV parsing, NMEA checksum
validation, a Maidenhead locator and structured status output.

This fork: **DL1OKE**. Code written with assistance from Claude.

## Hardware

### Required

| Component | Description |
|-----------|-------------|
| Raspberry Pi Pico 2 | RP2350 microcontroller (dual Cortex-M33) |
| LoRa1280F27-TCXO | SX1280 module with PA (+27 dBm) |
| SI5351 breakout | Clock generator, 52 MHz reference for the SX1280 |
| u-blox NEO-7M | GPS module with configurable TIMEPULSE output |
| 2.4 GHz antenna | SMA or u.FL |

### Optional

| Component | Description |
|-----------|-------------|
| SSD1306 OLED 128×64 | I²C display, 0.96" (1.3" panels usually carry an SH1106 and will not work) |
| Rotary encoder | KY-040 or similar, with push button |
| CW paddle / straight key | 3.5 mm stereo jack |
| MAX4466 microphone module | Electret mic with amplifier, for standalone SSB |
| Headphones | 3.5 mm jack with a passive RC network — no amplifier needed |

> **Note on the reference:** this build does **not** use the module's TCXO
> (`USE_TCXO_MODULE 0`). The SX1280 is clocked from the SI5351, which in turn is referenced
> to the GPS TIMEPULSE. Without a working SI5351 the SX1280 has no clock at all and will not start.

### Prototype

![Prototype transmitter](img/prototype.png)

*Prototype transmitter used for QO-100 tests - quick and dirty but it works!*

## Wiring

See [WIRING.txt](WIRING.txt) for detailed diagrams, including the headphone RC network
and the paddle jack.

```
Raspberry Pi Pico 2          LoRa1280F27-TCXO Module
===================          =======================
GPIO 16 (SPI0 RX)  ───────── MISO
GPIO 17            ───────── NSS (CS)
GPIO 18 (SPI0 SCK) ───────── SCK
GPIO 19 (SPI0 TX)  ───────── MOSI
GPIO 20            ───────── RESET
GPIO 21            ───────── BUSY
GPIO 14            ───────── RX_EN
GPIO 15            ───────── TX_EN

VBUS (5V)          ───────── VCC
GND                ───────── GND
USB                ───────── To computer (Audio + CDC)
```

> **Do not connect DIO1.** Upstream wires it to GPIO 5; in this fork GPIO 5 is the GPS UART.
> The firmware never uses DIO1 — the SX1280 is driven purely by BUSY polling.

### GPIO map

| GPIO | Use | | GPIO | Use |
|------|-----|---|------|-----|
| 0 | SI5351 SDA (I²C0) | | 14 | RX_EN |
| 1 | SI5351 SCL (I²C0) | | 15 | TX_EN |
| 2 | Encoder A | | 16 | SPI0 MISO |
| 3 | Encoder B | | 17 | SPI0 NSS |
| 4 | GPS TX (UART1) | | 18 | SPI0 SCK |
| 5 | GPS RX (UART1) | | 19 | SPI0 MOSI |
| 6 | OLED SDA (I²C1) | | 20 | NRESET |
| 7 | OLED SCL (I²C1) | | 21 | BUSY |
| 8 | free | | 22 | TCXO_EN (unused) |
| 9 | CW dit | | 25 | LED (underrun) |
| 10 | Encoder push | | 26 | Microphone (ADC0) |
| 11 | CW dah | | 27 | free (ADC1) |
| 12 | Sidetone PWM | | 28 | free (ADC2) |
| 13 | free | | | |

### Headphone output

No amplifier required — headphones need microwatts, and the GPIO pin supplies far more than that.

```
GPIO12 ──[ 100 Ω ]──┬──[ 10 µF ]──[ 220 Ω ]──► 3.5 mm jack, tip + ring
                    │   + to GPIO side
                 [ 100 nF ]
                    │
                   GND ─────────────────────► 3.5 mm jack, sleeve
```

The 100 Ω / 100 nF pair filters the 244 kHz PWM carrier, the 10 µF blocks the 1.65 V idle
offset, and the 220 Ω sets the level and protects against a shorted jack. Start at
`keyer vol 15` — 30 % is already loud on 32 Ω headphones.

## Building

### Requirements
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.0+ (or the VS Code Pico extension)
- CMake 3.13+
- ARM GCC toolchain

### Clone with submodules
```bash
git clone --recurse-submodules https://github.com/SimonRZz/SX1280_QO100_SSB_TX.git
cd SX1280_QO100_SSB_TX
```

Or if already cloned:
```bash
git submodule update --init
```

### Build
```bash
mkdir build && cd build
cmake ..
make -j4
```

### Flash
```bash
# Hold BOOTSEL and connect USB
cp SX1280SDR.uf2 /media/$USER/RP2350/
```

The running build identifies itself on the OLED boot screen, in the CDC greeting, and via
`version` — useful when several builds are in circulation.

## Usage

### With a computer

1. Connect the Pico and select **"SX1280 QO-100 SSB TX"** as the audio output device
2. Transmit from any software (SDR, WSJT-X, fldigi, …)
3. Run the GUI for parameter control:

```bash
pip install pyserial
python3 gui.py
```

![GUI Control Panel](img/gui.png)

### Standalone

The device boots without a host and is fully operable from the encoder:

- **Turn** — frequency in 100 Hz steps
- **Short press** — browse the six tiles of the current page; press again to edit a value,
  turn to change it, press to leave
- **Long press (½ s)** — next page: operating → CW/audio → GPS/time

Page 1 covers keyer mode, dah ratio, WPM, sidetone pitch, microphone gain and gate.
Page 2 shows UTC, date, Maidenhead locator, satellites and altitude. Sidetone pitch and
volume are audible while being edited.

### CW

Connect a paddle to GP9 (dit) and GP11 (dah), or a straight key to GP9 with
`keyer mode straight`. Element timing runs in the firmware, so no PC is involved.
The firmware also decodes what it keys and reports it over CDC, which the GUI displays —
handy for checking speed and timing.

## CDC commands

### General

| Command | Description |
|---------|-------------|
| `help` | List commands |
| `get` | Show current configuration |
| `version` | Firmware version and build date |
| `diag` | SX1280, buffers, paddles, keyer, sidetone and carrier state |
| `save` | Write settings to flash now |
| `defaults` | Restore compile-time defaults and save |

### GPSDO

| Command | Description |
|---------|-------------|
| `gpsdo` | Status line: signal, fix, satellites, CLK1, UTC, locator, altitude |
| `gpsgate 0/1` | 1: transmit only with GPS UTC (default). 0: bench-test override, resets at boot |

### Transmit

| Command | Description |
|---------|-------------|
| `tx 0/1` | Enable/disable TX (SSB modulation) |
| `mode usb/cw` | Set modulation mode |
| `tune 0/1` | Toggle TUNE carrier |
| `cw` / `stop` | Start / stop the test carrier |
| `key 0/1` | Software PTT/KEY for the PC-side keyer |
| `freq <Hz>` | Frequency with sub-Hz precision (e.g. `freq 2400100050.5`) |
| `ppm <value>` | Oscillator PPM correction |
| `txpwr <-18..13>` | Max TX power on the SX1280 in dBm |

### Keyer and sidetone

| Command | Description |
|---------|-------------|
| `keyer mode <straight\|a\|b>` | Keyer mode |
| `keyer wpm <5..60>` | Speed |
| `keyer ratio <2..5>` | Dah length in dits |
| `keyer tone <300..1200>` | Sidetone pitch in Hz |
| `keyer vol <0..100>` | Sidetone volume |
| `keyer test 0/1` | Continuous tone for checking the headphone wiring |

The keyer emits decoder events: `!K e=.` / `!K e=-` per element, `!K c=<char>` per
character, `!K w` per word gap.

### Audio and display

| Command | Description |
|---------|-------------|
| `src pc/mic` | Audio source: USB from the PC, or the MAX4466 on GP26 |
| `mic gain <1..50>` | Microphone gain |
| `mic gate <0..0.5>` | Noise gate threshold |
| `oled flip 0/1` | Rotate the display by 180° |

### DSP

| Command | Description |
|---------|-------------|
| `enable bp\|eq\|comp 0/1` | Enable/disable a DSP block |
| `set bp_lo <Hz>` / `set bp_hi <Hz>` | Bandpass corner frequencies |
| `set bp_stages <1-10>` | Filter steepness (12 dB/oct per stage) |
| `set eq_low_hz <Hz>` / `set eq_low_db <dB>` | Low shelf |
| `set eq_high_hz <Hz>` / `set eq_high_db <dB>` | High shelf |
| `set comp_thr <dB>` / `set comp_ratio <n>` | Compressor threshold and ratio |
| `set comp_att <ms>` / `set comp_rel <ms>` | Attack and release |
| `set comp_makeup <dB>` / `set comp_knee <dB>` | Makeup gain and knee |
| `set comp_outlim <0..1>` | Output limiter |
| `set amp_gain <float>` / `set amp_min_a <float>` | Power shaping |

## Technical specifications

| Parameter | Value |
|-----------|-------|
| Frequency range | 2400.000 – 2500.000 MHz (QO-100 NB: 2400.000 – 2400.500) |
| Output power | up to +27 dBm (adjustable −18…+13 dBm on the chip) |
| Modulation | SSB (USB), CW |
| Reference | SI5351 52 MHz, GPS-disciplined via NEO-7M TIMEPULSE (24 MHz) |
| Frequency resolution | sub-Hz (PLL steps + DSP fine offset) |
| Audio input | USB 48 kHz, or ADC 8 kHz (MAX4466) |
| DSP sample rate | 8 kHz |
| Sidetone | PWM 10 bit, 244 kHz carrier, 48 kHz sample rate via DMA |
| SPI clock | 18 MHz |
| OLED | SSD1306 128×64, I²C1 @ 100 kHz, DMA transfer |

## QO-100 uplink

QO-100 narrowband transponder:
- **Uplink:** 2400.000 – 2400.500 MHz
- **Downlink:** 10489.500 – 10490.000 MHz

## Changelog

### v2.2.0 — standalone operation

Everything below is new in this fork relative to SP8ESA's v2.0 base.

**GPS-disciplined reference**
- SI5351 52 MHz reference locked to a NEO-7M, replacing the module TCXO
- Two-stage gating: the radio starts on clock lock, transmission waits for GPS UTC
- `gpsgate 0` overrides the TX gate for bench tests; never persisted
- A fix that is lost after being acquired now blinks on the display and is reported over CDC
- GPS UART is serviced in every wait loop, including CW mode

**CW**
- Iambic keyer in the firmware: Straight / Iambic A / Iambic B, 5–60 WPM, adjustable dah ratio
- Hold-to-repeat is decided at the end of the inter-element space, not at element end —
  a normal tap no longer produces double dits
- Contact-bounce stability filter replaces the previous lockout filter
- Click-free keying: the carrier ramps 1 dB per 160 µs
- Morse decoder reports elements, characters and word gaps over CDC
- 3.5 mm paddle jack on GP9/GP11, straight key supported

**Audio**
- Sidetone on GP12: PWM audio, DMA-fed at 48 kHz, raised-cosine envelope, interpolated sine table
- Microphone input on GP26 (MAX4466) with DC removal, gain and a noise gate
- Audio source switchable between PC and microphone

**User interface**
- SSD1306 OLED with three pages and rotary-encoder control; `oled flip` rotates it 180°
- Settings persist in flash with CRC32; autosave waits until the radio is off the air
- Boots without a USB host
- 120 ms TX guard after a mode change
- GUI: debugging console with timestamps, filters and log export; GPSDO tab; on-device keyer panel;
  downlink frequency slider

### v1.5.0
- **New feature:** Sub-Hz frequency precision via automatic PLL + DSP fine tuning
  - Frequency stored as double - no more 198 Hz PLL quantization visible to user
  - Firmware automatically splits frequency into PLL steps + DSP complex carrier rotation
- **GUI improvements:**
  - **TX ON/OFF button** with green color when transmitting
  - **PPM slider** (-2 to +2 ppm) with immediate response
  - **Scroll wheel tuning** (50 Hz per step, toggleable checkbox)
  - **QO-100 downlink frequency** display (uplink → downlink conversion)
  - All sliders now respond immediately (no delay)
- Added `tx 0/1` command for TX enable/disable
- **Updated default DSP values** (optimized for voice):
  - Bandpass: 50-2700 Hz
  - EQ low shelf: -2.0 dB (was -9.5)
  - Compressor threshold: -2.5 dB (was -12.5)
  - Output limit: 0.940 (was 0.312)
  - Amp gain: 2.9 (was 4.36)

### v1.4.1
- **CRITICAL FIX:** Fixed NaN bug in shelf filter that caused continuous carrier instead of SSB modulation
- **CRITICAL FIX:** Reverted unstable DSP parameter changes from v1.4.0 that caused distorted audio
- Removed experimental features (timing jitter, EQ slope) that caused instability
- Restored proven default DSP values from v1.3.0
- Kept only stable new features: TX power control (`txpwr`) and BP stages (`bp_stages`)
- GUI updated to match firmware capabilities

### v1.4.0 (DEPRECATED - DO NOT USE)
- ⚠️ This version has critical bugs causing continuous carrier and distorted audio
- Added adjustable bandpass filter stages (`set bp_stages 1-10`)
- Added EQ slope parameter (removed in v1.4.1)
- Added timing jitter dithering (removed in v1.4.1)
- Added TX power control (`txpwr -18..13`)

### v1.3.0
- DSP chain reordered: EQ → Compressor → BPF
- Added Python GUI for CDC control

### v1.2.0
- Fixed USB Audio compatibility on Windows
- Added volume control support

### v1.1.0
- Initial release with basic SSB TX functionality

## TODO

- [ ] Headphone mixer: sidetone + local microphone monitoring + received audio
- [ ] Feed QO-100 receive audio (LNB → downconverter → handheld) into ADC1, with a level-shifting network
- [ ] Full-screen value display while turning the encoder
- [ ] Keyer timing above 32 WPM — decouple element timing from the polling loop
- [ ] Full band 2300–2450 MHz
- [ ] FM mode (for 2.4 GHz simplex only — **never** on the QO-100 narrowband transponder)

## Warning

**Transmission on 2.4 GHz requires appropriate radio license!**

Make sure you have a valid amateur radio license and comply with regulations in your country.

A continuous carrier at 2400.4 MHz sits at the bottom edge of WLAN channel 1 and will disrupt
2.4 GHz WLAN and Bluetooth nearby. Use a dummy load for bench testing.

## License

This project is licensed under **CC BY-NC 4.0** (Creative Commons Attribution-NonCommercial).

- Non-commercial use (including amateur radio) - OK
- Modifications allowed - OK
- Commercial use requires author's permission

This project uses:
- [TinyUSB](https://github.com/hathach/tinyusb) - MIT License
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) - BSD-3-Clause

---

73 de SP8ESA · fork by DL1OKE
