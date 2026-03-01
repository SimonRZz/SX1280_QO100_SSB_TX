#!/usr/bin/env python3
"""Command-line CW helper for SX1280 CDC interface.

Supports:
- carrier mode (`cw` / `stop`)
- plain text to Morse keying via `key 1` / `key 0`

This keeps all existing firmware features intact and adds a lightweight
terminal workflow for CW without the GUI.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:  # runtime dependency hint
    raise SystemExit("pyserial missing. Install with: pip install pyserial") from exc


MORSE = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".", "F": "..-.",
    "G": "--.", "H": "....", "I": "..", "J": ".---", "K": "-.-", "L": ".-..",
    "M": "--", "N": "-.", "O": "---", "P": ".--.", "Q": "--.-", "R": ".-.",
    "S": "...", "T": "-", "U": "..-", "V": "...-", "W": ".--", "X": "-..-",
    "Y": "-.--", "Z": "--..",
    "0": "-----", "1": ".----", "2": "..---", "3": "...--", "4": "....-",
    "5": ".....", "6": "-....", "7": "--...", "8": "---..", "9": "----.",
    ".": ".-.-.-", ",": "--..--", "?": "..--..", "/": "-..-.", "=": "-...-",
    "+": ".-.-.", "-": "-....-", "(": "-.--.", ")": "-.--.-", "@": ".--.-.",
}


def send_line(ser: serial.Serial, cmd: str) -> None:
    ser.write((cmd + "\r\n").encode("utf-8"))
    ser.flush()


def key_for(ser: serial.Serial, down: bool, seconds: float) -> None:
    send_line(ser, "key 1" if down else "key 0")
    if seconds > 0:
        time.sleep(seconds)


def send_text(ser: serial.Serial, text: str, wpm: int, sidetone: int) -> None:
    dot = 1.2 / max(5, min(60, int(wpm)))
    send_line(ser, "mode cw")
    send_line(ser, f"wpm {int(wpm)}")
    send_line(ser, f"sidetone {int(sidetone)}")

    words = text.upper().split(" ")
    for wi, word in enumerate(words):
        for ci, ch in enumerate(word):
            pattern = MORSE.get(ch)
            if not pattern:
                continue
            for ei, elem in enumerate(pattern):
                key_for(ser, True, dot if elem == "." else 3.0 * dot)
                key_for(ser, False, dot if ei < len(pattern) - 1 else 0.0)
            if ci < len(word) - 1:
                time.sleep(2.0 * dot)  # total letter gap = 3 dots incl. element gap
        if wi < len(words) - 1:
            time.sleep(4.0 * dot)      # total word gap = 7 dots incl. letter gap

    send_line(ser, "key 0")


def run() -> int:
    parser = argparse.ArgumentParser(description="SX1280 CW command-line helper")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200)

    sub = parser.add_subparsers(dest="mode", required=True)

    sub.add_parser("carrier", help="Start continuous carrier CW until Ctrl+C")

    text_p = sub.add_parser("text", help="Send plain text as Morse using key 1/0")
    text_p.add_argument("message", help="Text to send")
    text_p.add_argument("--wpm", type=int, default=18)
    text_p.add_argument("--sidetone", type=int, default=700)

    args = parser.parse_args()

    with serial.Serial(port=args.port, baudrate=args.baud, timeout=0.2, write_timeout=1.0) as ser:
        if args.mode == "carrier":
            send_line(ser, "cw")
            print("CW carrier active. Ctrl+C to stop.")
            try:
                while True:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                send_line(ser, "stop")
                send_line(ser, "mode ssb")
            return 0

        if args.mode == "text":
            try:
                send_text(ser, args.message, args.wpm, args.sidetone)
            finally:
                send_line(ser, "mode ssb")
                send_line(ser, "stop")
            return 0

    return 1


if __name__ == "__main__":
    sys.exit(run())
