# SX1280 QO-100 SSB/CW Transmitter with GPS-Disciplined Oscillator

> **Work in progress.** It is not yet clear whether this project is useful to anyone
> beyond my own shack. The README is incomplete, photos will be added later, and some
> sections may still be inaccurate. If you find it useful or have questions, feel free
> to open an issue. This is my first attempt at working with Github, Claude and code. No guarantee for anything.

A 2.4 GHz uplink transmitter for the QO-100 geostationary amateur radio satellite,
built around the Semtech SX1280 LoRa chip and a Raspberry Pi Pico 2.

This is a fork of [SP8ESA's SX1280_QO100_SSB_TX](https://github.com/SP8ESA/SX1280_QO100_SSB_TX).
The README has not yet been fully updated — the relevant work is in the code.

---

## What this project adds

- **GPS-disciplined oscillator (GPSDO)** integrated directly into the Pico firmware —
  no separate Arduino Nano required
- **Transmit lock on GPS fix**: the transmitter does not start until the NEO-7M has
  acquired enough satellites and the timepulse is stable
- **Rock-stable frequency**: GPS-locked reference eliminates the thermal drift of the
  SX1280's internal TCXO, making SSB fully usable
- **CW keyer** with iambic A/B and straight key support — connect a key via a
  TTL-to-USB adapter and operate directly from the GUI
- **GPSDO status tab** in the Python GUI showing satellite count, lock status and
  timepulse frequency

---

## Background

The original SP8ESA project demonstrated that the SX1280 LoRa chip can be used as a
direct IQ-modulated 2.4 GHz SSB transmitter. The chip accepts raw IQ samples over SPI,
which the Pico writes at audio rate to produce SSB modulation.

The main problem encountered during testing was **frequency instability**: the SX1280F27
module (which includes an integrated 500 mW PA) runs its PA continuously, causing
significant thermal drift. SP8ESA has demonstrated that SSB is possible despite this,
and CW certainly works too — but in my tests the drift was severe enough to make
narrow digital modes (such as FT8) impossible to decode. The TCXO on the module cannot
compensate for it.

The solution is to replace the SX1280's internal reference entirely with a
GPS-disciplined SI5351 synthesizer — a technique originally described by
[CT2GQV](https://speakyssb.blogspot.com/2019/10/si5351-gps-disciplined-oscillator-with.html).

In this build the bare SX1280 module (~20 mW) is used instead of the SX1280F27.
The reason is simple: the external PA used here (SG Labs PA2400 V3) is fully driven
by 20 mW, so the integrated PA of the F27 is not needed. Whether the F27 produces
relevant spurious emissions compared to the bare module has not been systematically tested. 
You can also use the SX1280f27, no problem. 

---

## How the GPSDO works

```
u-blox NEO-7M  →  24 MHz Timepulse  →  SI5351 XA input  →  CLK1: 52 MHz  →  SX1280 XTA
```

1. The NEO-7M is configured via UBX command `UBX-CFG-TP5` to output an exact **24 MHz
   timepulse** on its TIMEPULSE pin. 24 MHz = 48 MHz ÷ 2 — an exact integer divisor,
   no pulse-swallowing, no jitter.
2. The **quartz crystal on the SI5351 breakout board is desoldered**. The 24 MHz GPS
   signal is fed directly into the XA pin of the SI5351, replacing the crystal.
3. The SI5351 synthesizes **52 MHz** from this GPS-locked reference via its PLL.
4. The 52 MHz signal is fed into the **XTA pin of the SX1280**, replacing its internal
   TCXO (which must also be desoldered; TCXO mode is kept permanently enabled via GP22 HIGH).

The GPSDO logic (UBX configuration, satellite count polling, timepulse validation) runs
directly on the Pico — the Arduino Nano from the original CT2GQV design is not needed.

**The transmitter will not activate until the GPS module reports a valid fix with
sufficient satellites.** The GPSDO tab in the GUI shows live lock status.

![Keyer Tab](img/GPSDO%20Tab.jpg)

---

## CW Keyer

A CW keyer is built into the firmware and exposed in the Python GUI. To use it:

1. Connect a straight key or iambic paddle to a **TTL-to-USB serial adapter**
   (e.g. CH340, CP2102). The key contacts connect to the DTR/RTS lines of the adapter.
2. Plug the adapter into the PC running the GUI.
3. Open the **CW Keyer tab**, select the correct serial port, and choose keyer mode
   (Straight / Iambic A / Iambic B) and speed (WPM).
4. The GUI reads the key state and controls the SX1280 carrier accordingly via the Pico.

Sidetone is generated in software through the PC audio output.

![Keyer Tab](img/CW%20Keyer%20Tab.jpg)


---

## Hardware

### Bill of materials

| Part | Notes |
|---|---|
| Raspberry Pi Pico 2 (or Pico 2W) | Pico 2W recommended for future WiFi support |
| SX1280 module (no internal PA) | ~20 mW output |
| u-blox NEO-7M GPS module | Must support UBX protocol — see note below |
| SI5351 breakout board | Crystal will be removed |
| Helix antenna (3D printed) | See below; or use a dish |
| External PA (optional) | Needed without a large dish |

**Why the NEO-7M specifically?** Cheap GPS modules output only NMEA sentences and do
not support the UBX binary protocol. The NEO-7M supports `UBX-CFG-TP5`, which allows
configuring the timepulse output to exactly 24 MHz. This is essential for the GPSDO —
no other commonly available module supports this out of the box.

---

## Wiring diagram

```
Raspberry Pi Pico 2          SX1280 Module
===================          =============
GP16 (SPI0 MISO)   ───────── MISO
GP17 (SPI0 CS)     ───────── NSS / CS
GP18 (SPI0 SCK)    ───────── SCK
GP19 (SPI0 MOSI)   ───────── MOSI
GP20               ───────── NRESET
GP21               ───────── BUSY
GP22               ───────── TCXO_EN (permanently HIGH)
GP14               ───────── RX_EN
GP15               ───────── TX_EN
3V3                ───────── VCC
GND                ───────── GND

SI5351 (I2C0)                u-blox NEO-7M (UART1)
=============                =====================
GP0  (I2C0 SDA)    ── SDA   GP4  (UART1 TX)   ── RX
GP1  (I2C0 SCL)    ── SCL   GP5  (UART1 RX)   ── TX
CLK1               ── SX1280 XTA (keep wire short!)
3V3                ── VCC
GND                ── GND
Crystal: DESOLDER

Optional: Decoupling
====================
SI5351  VCC:   100 nF ceramic directly at VCC pin
NEO-7M  VCC:   220 µF electrolytic + 100 nF ceramic close to module
SPI lines:     33 Ω series resistors on SCK/MOSI (reduce ringing)
I2C lines:     4.7 kΩ pull-up resistors on SDA/SCL (if not on breakout)
CLK1 → XTA:   shield wire or short coax run recommended
```

---

### Antenna

A 3D-printed helix antenna is a practical option if you do not have a dish.
The design used here is based on
[this Thingiverse model](https://www.thingiverse.com/thing:4980180), modified as follows:

- 8 turns
- Narrower wire feed holes (2 mm) for the antenna element
- M8 mounting holes
- Guide rail on the side — allows printing in two halves and gluing back together accurately

---

### Critical assembly notes

- **Desolder the crystal from the SI5351 board.** Without this, the XA input will not
  accept the external GPS signal.
- **Desolder the TCXO from the SX1280 module.** Keep GP22 permanently HIGH to hold
  the SX1280 in TCXO mode. Never issue a `tcxo 0` command.
- **Decoupling on NEO-7M VCC:** 220 µF electrolytic + 100 nF ceramic, placed close
  to the module. The NEO-7M draws current spikes that cause GPS lock loss without this.
- **Decoupling on SI5351 VCC:** 100 nF ceramic directly at the VCC pin. A missing cap
  here produces visible ~1024 Hz spurs in the output spectrum (SI5351 PLL artifacts).
- Build the GPSDO section in a metal enclosure if possible to reduce interference.

---

## Software

### Requirements

- Python 3.10+
- Git

### Installation

```bash
git clone https://github.com/SimonRZz/SX1280_QO100_SSB_TX
cd SX1280_QO100_SSB_TX
pip install -r requirements.txt
```

### Build the Pico firmware

```bash
mkdir build && cd build
cmake ..
make -j4
```

This produces a `.uf2` file in the `build/` directory.

### Flash the Pico firmware

1. Hold the **BOOTSEL** button on the Pico and connect it via USB — it appears as a
   mass storage device.
2. Copy the `.uf2` file from the `/firmware/` directory onto it.
3. The Pico reboots and starts running immediately.

### Start the GUI

```bash
python main.py
```

The GUI provides:

- **Waterfall display** and frequency tuning
- **SSB transmit** via PC microphone
- **GPSDO tab**: live satellite count, lock status, timepulse frequency.
  Transmit is blocked until a valid GPS fix is confirmed.
- **CW keyer tab**: select serial port, keyer mode (Straight / Iambic A / Iambic B),
  speed in WPM. Connect a paddle or straight key via any TTL-to-USB adapter.

---

## Transponder frequency calibration

The QO-100 narrowband transponder LO is not exactly at its nominal value.
Measure your signal on the [WebSDR](https://eshail.batc.org.uk/nb/) and adjust
the LO calibration value in the GUI. The value used in this build:
**8089.4972 MHz** (nominal is 8089.5 MHz).

---

## Planned / maybe someday

- **WiFi remote operation** using the Pico 2W's CYW43439 radio:
  cwdaemon-compatible UDP server on port 6789, web interface for frequency/speed/power,
  WiFi client or AP mode with onboarding page.
  Reference implementation for the concept: [ok1cdj/SX1281_QO100_TX](https://github.com/ok1cdj/SX1281_QO100_TX)

---

## Credits

- [SP8ESA](https://github.com/SP8ESA/SX1280_QO100_SSB_TX) — original SX1280 QO-100 SSB TX project
- [CT2GQV](https://speakyssb.blogspot.com/2019/10/si5351-gps-disciplined-oscillator-with.html) — SI5351 GPSDO concept
- [Thingiverse / original helix design](https://www.thingiverse.com/thing:4980180)

---

## License

GPL-3.0 — see [LICENSE](LICENSE)
