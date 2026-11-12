# Host regression checks

Run `sh tests/run.sh` with a native C11 compiler (`CC` defaults to `cc`).
The tests compile the real Si4732 driver against minimal GPIO, I2C and time
stubs. They check first-boot calibration, valid/invalid saved settings,
calibration sign, sub-kHz boundaries, BFO writes, raw overrides, rejected
requests and AM rounding. They use no Pico SDK, radio or dynamic allocation.

The I2C stub acknowledges commands and supplies a fixed revision/status. These
checks do not validate RF reception, the real SSB patch, flash programming,
reference-clock quality, or the physical/audio sign convention on a chip.
A test-only eight-byte patch fallback allows these checks on a clean checkout
without downloading the real third-party SSB patch.

Run `python tests/test_gui_rx.py` for headless GUI/serial regression checks.
Python's Tkinter module must be available, but no display or serial/audio
hardware is used. The checks cover reader failures and reconnect isolation,
legacy/Git version messages, visible command replies, live polling after OFF,
meter freshness, and receive-timer recovery after malformed status messages.
