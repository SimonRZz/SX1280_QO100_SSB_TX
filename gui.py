#!/usr/bin/env python3
"""SX1280 GUI with external CW keying (Straight/Iambic A/Iambic B)."""

import math
import os
import queue
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from typing import Callable, Optional

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False

try:
    import evdev
    HAS_EVDEV = True
except ImportError:
    evdev = None
    HAS_EVDEV = False

try:
    import simpleaudio as sa
    HAS_SIMPLEAUDIO = True
except ImportError:
    sa = None
    HAS_SIMPLEAUDIO = False


class SerialWorker:
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser = None
        self.stop_evt = threading.Event()
        self.thread = None
        self.lock = threading.Lock()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int = 115200):
        if not HAS_SERIAL:
            raise RuntimeError("pyserial missing (pip install pyserial)")
        with self.lock:
            if self.is_connected():
                return
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, write_timeout=0.5)
            self.stop_evt.clear()
            self.thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.thread.start()

    def disconnect(self):
        with self.lock:
            self.stop_evt.set()
            s = self.ser
            self.ser = None
        if s:
            try:
                s.close()
            except Exception:
                pass

    def send_line(self, line: str):
        line = line.strip()
        if not line:
            return
        with self.lock:
            if not self.is_connected():
                raise RuntimeError("Not connected")
            self.ser.write((line + "\r\n").encode("utf-8", errors="replace"))
            self.ser.flush()

    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_evt.is_set():
            with self.lock:
                s = self.ser
            if s is None:
                break
            try:
                data = s.read(256)
                if not data:
                    time.sleep(0.01)
                    continue
                buf.extend(data)
                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    self.rx_queue.put(line.decode("utf-8", errors="replace").rstrip("\r"))
            except Exception as exc:
                self.rx_queue.put(f"[SERIAL ERROR] {exc}")
                break


class SidetonePlayer:
    def __init__(self):
        self.enabled = False
        self.freq_hz = 700
        self.sample_rate = 22050
        self.wave_obj = None
        self.play_obj = None
        self._rebuild_wave()

    def available(self):
        return HAS_SIMPLEAUDIO

    def set_freq(self, hz: int):
        self.freq_hz = max(300, min(1200, int(hz)))
        self._rebuild_wave()

    def _rebuild_wave(self):
        if not HAS_SIMPLEAUDIO:
            return
        duration = 0.1
        total = int(self.sample_rate * duration)
        pcm = bytearray()
        for n in range(total):
            v = int(18000 * math.sin(2.0 * math.pi * self.freq_hz * n / self.sample_rate))
            pcm.extend(struct.pack('<h', v))
        self.wave_obj = sa.WaveObject(bytes(pcm), 1, 2, self.sample_rate)

    def key_down(self):
        if not HAS_SIMPLEAUDIO or not self.enabled:
            return
        if self.play_obj and self.play_obj.is_playing():
            return
        self.play_obj = self.wave_obj.play()

    def key_up(self):
        if not HAS_SIMPLEAUDIO:
            return
        if self.play_obj:
            self.play_obj.stop()
            self.play_obj = None


class ExternalKeyer:
    STRAIGHT = "Straight"
    IAMBIC_A = "Iambic A"
    IAMBIC_B = "Iambic B"

    def __init__(self, send_cmd: Callable[[str], None], log: Callable[[str, str], None], sidetone: SidetonePlayer):
        self.send_cmd = send_cmd
        self.log = log
        self.sidetone = sidetone

        self.enabled = False
        self.mode = self.STRAIGHT
        self.wpm = 18
        self.device_path = "Keyboard (Space)"

        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._run_keyer_loop, daemon=True)
        self._thread.start()

        self.dit_pressed = False
        self.dah_pressed = False
        self.dit_latch = False
        self.dah_latch = False

        self.key_is_down = False
        self.element_active = False
        self.element_end = 0.0
        self.gap_active = False
        self.gap_end = 0.0
        self.last_element = None
        self.squeeze = False
        self.iambic_b_extra = False

        self.dev_thread = None
        self.dev_stop_evt = threading.Event()
        self.keyboard_space = False

    def stop(self):
        self.disable()
        self._stop_evt.set()

    def set_mode(self, mode: str):
        with self._lock:
            self.mode = mode

    def set_wpm(self, wpm: int):
        with self._lock:
            self.wpm = max(5, min(60, int(wpm)))

    def set_device(self, device_path: str):
        with self._lock:
            self.device_path = device_path
        if self.enabled:
            self._restart_device_listener()

    def enable(self):
        if self.enabled:
            return
        self.enabled = True
        self.reset_state()
        self._restart_device_listener()

    def disable(self):
        if not self.enabled:
            return
        self.enabled = False
        self._stop_device_listener()
        self._set_key(False)
        self.send_cmd("key 0")
        self.send_cmd("stop")

    def reset_state(self):
        with self._lock:
            self.dit_pressed = False
            self.dah_pressed = False
            self.dit_latch = False
            self.dah_latch = False
            self.element_active = False
            self.gap_active = False
            self.last_element = None
            self.squeeze = False
            self.iambic_b_extra = False

    def keyboard_event(self, keysym: str, pressed: bool):
        if not self.enabled:
            return
        if keysym == "space":
            self.keyboard_space = pressed
            if self.mode == self.STRAIGHT:
                self._set_key(pressed)
            return
        if keysym in ("z", "Z"):
            self._update_paddle("dit", pressed)
        if keysym in ("x", "X"):
            self._update_paddle("dah", pressed)

    def _restart_device_listener(self):
        self._stop_device_listener()
        path = self.device_path
        if path == "Keyboard (Space)" or not HAS_EVDEV:
            return
        self.dev_stop_evt.clear()
        self.dev_thread = threading.Thread(target=self._evdev_loop, args=(path,), daemon=True)
        self.dev_thread.start()

    def _stop_device_listener(self):
        self.dev_stop_evt.set()
        if self.dev_thread and self.dev_thread.is_alive():
            self.dev_thread.join(timeout=0.3)
        self.dev_thread = None

    def _evdev_loop(self, dev_path: str):
        try:
            dev = evdev.InputDevice(dev_path)
            for event in dev.read_loop():
                if self.dev_stop_evt.is_set() or not self.enabled:
                    break
                if event.type != evdev.ecodes.EV_KEY:
                    continue
                if event.value == 2:
                    continue
                key_event = evdev.categorize(event)
                code = key_event.keycode
                if isinstance(code, list):
                    code = code[0]
                pressed = bool(event.value)
                self._handle_evdev_key(code, pressed)
        except Exception as exc:
            self.log(f"[External keying] Device error: {exc}", "error")

    def _handle_evdev_key(self, code: str, pressed: bool):
        dit_keys = {"KEY_LEFT", "KEY_Z", "BTN_TRIGGER_HAPPY1", "BTN_TOP"}
        dah_keys = {"KEY_RIGHT", "KEY_X", "BTN_TRIGGER_HAPPY2", "BTN_THUMB"}
        straight_keys = {"KEY_SPACE", "BTN_0", "BTN_TRIGGER", "KEY_ENTER"}

        if code in straight_keys and self.mode == self.STRAIGHT:
            self._set_key(pressed)
            return
        if code in dit_keys:
            self._update_paddle("dit", pressed)
        elif code in dah_keys:
            self._update_paddle("dah", pressed)

    def _update_paddle(self, which: str, pressed: bool):
        with self._lock:
            if which == "dit":
                self.dit_pressed = pressed
                if pressed:
                    self.dit_latch = True
            else:
                self.dah_pressed = pressed
                if pressed:
                    self.dah_latch = True

            if self.dit_pressed and self.dah_pressed:
                self.squeeze = True

            if self.mode == self.STRAIGHT:
                self._set_key(self.dit_pressed or self.dah_pressed)

    def _set_key(self, down: bool):
        if down == self.key_is_down:
            return
        self.key_is_down = down
        self.send_cmd("key 1" if down else "key 0")
        if down:
            self.sidetone.key_down()
        else:
            self.sidetone.key_up()

    def _select_next_element(self):
        both = (self.dit_pressed or self.dit_latch) and (self.dah_pressed or self.dah_latch)
        if self.iambic_b_extra:
            self.iambic_b_extra = False
            return "dah" if self.last_element == "dit" else "dit"
        if both:
            if self.last_element == "dit":
                return "dah"
            return "dit"
        if self.dit_pressed or self.dit_latch:
            return "dit"
        if self.dah_pressed or self.dah_latch:
            return "dah"
        return None

    def _run_keyer_loop(self):
        while not self._stop_evt.is_set():
            if not self.enabled:
                time.sleep(0.01)
                continue

            with self._lock:
                mode = self.mode
                dit_time = 1.2 / max(5, self.wpm)
                both_pressed = self.dit_pressed and self.dah_pressed

                now = time.monotonic()
                if mode == self.STRAIGHT:
                    self._set_key(self.dit_pressed or self.dah_pressed or self.keyboard_space)
                    time.sleep(0.002)
                    continue

                if self.element_active and now >= self.element_end:
                    self.element_active = False
                    self._set_key(False)
                    self.gap_active = True
                    self.gap_end = now + dit_time

                    if mode == self.IAMBIC_A and not both_pressed:
                        self.dit_latch = self.dit_pressed
                        self.dah_latch = self.dah_pressed
                        self.squeeze = False

                if self.gap_active and now >= self.gap_end:
                    self.gap_active = False
                    if mode == self.IAMBIC_B and not self.dit_pressed and not self.dah_pressed and self.squeeze:
                        self.iambic_b_extra = True
                        self.squeeze = False

                if not self.element_active and not self.gap_active:
                    nxt = self._select_next_element()
                    if nxt:
                        self.last_element = nxt
                        if nxt == "dit":
                            self.dit_latch = False
                            dur = dit_time
                        else:
                            self.dah_latch = False
                            dur = 3.0 * dit_time
                        self._set_key(True)
                        self.element_active = True
                        self.element_end = now + dur

            time.sleep(0.001)


def list_serial_ports():
    if not HAS_SERIAL:
        return []
    out = []
    for p in serial.tools.list_ports.comports():
        label = f"{p.device} ({p.description})"
        if "SX1280" in p.description:
            label = f"★ {label}"
        out.append((p.device, label))
    return out


def list_input_devices():
    items = [("Keyboard (Space)", "Keyboard (Space)")]
    if not HAS_EVDEV:
        return items
    for path in sorted([f"/dev/input/{x}" for x in os.listdir('/dev/input') if x.startswith('event')]):
        try:
            dev = evdev.InputDevice(path)
            items.append((path, f"{dev.name} [{path}]"))
            dev.close()
        except Exception:
            items.append((path, f"Unknown [{path}]") )
    return items


class App(ttk.Frame):
    def __init__(self, root: tk.Tk):
        super().__init__(root)
        self.root = root
        self.root.title("SX1280 QO-100 Control + External CW Keying")
        self.root.geometry("980x720")

        self.rx_queue = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self.sidetone = SidetonePlayer()
        self.keyer = ExternalKeyer(self._send_cmd_safe, self._log, self.sidetone)

        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="⚫ Disconnected")
        self.keying_enabled_var = tk.BooleanVar(value=False)
        self.keyer_mode_var = tk.StringVar(value=ExternalKeyer.STRAIGHT)
        self.wpm_var = tk.IntVar(value=18)
        self.tone_var = tk.IntVar(value=700)
        self.local_sidetone_var = tk.BooleanVar(value=False)
        self.device_label_var = tk.StringVar(value="Keyboard (Space)")

        self._build_ui()
        self._refresh_ports()
        self._refresh_input_devices()

        self.pack(fill="both", expand=True)
        self._poll_rx()

        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=8)
        ttk.Label(top, text="Serial Port:").pack(side="left")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=45)
        self.port_combo.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        ttk.Button(top, text="Connect", command=self._connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side="left")
        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        key = ttk.LabelFrame(self, text="External Keying", padding=10)
        key.pack(fill="x", padx=8, pady=8)

        ttk.Checkbutton(key, text="External Keying On", variable=self.keying_enabled_var,
                        command=self._toggle_external_keying).grid(row=0, column=0, sticky="w")
        ttk.Label(key, text="Input Device:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.device_combo = ttk.Combobox(key, textvariable=self.device_label_var, state="readonly", width=58)
        self.device_combo.grid(row=1, column=1, sticky="ew", pady=(8, 0))
        ttk.Button(key, text="Refresh", command=self._refresh_input_devices).grid(row=1, column=2, padx=4, pady=(8, 0))

        ttk.Label(key, text="Keyer Mode:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        mode_combo = ttk.Combobox(key, textvariable=self.keyer_mode_var, state="readonly",
                                  values=[ExternalKeyer.STRAIGHT, ExternalKeyer.IAMBIC_A, ExternalKeyer.IAMBIC_B], width=20)
        mode_combo.grid(row=2, column=1, sticky="w", pady=(8, 0))
        mode_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_keyer_params_changed())

        ttk.Label(key, text="WPM:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        wpm_spin = ttk.Spinbox(key, from_=5, to=60, increment=1, textvariable=self.wpm_var, width=8,
                               command=self._on_keyer_params_changed)
        wpm_spin.grid(row=3, column=1, sticky="w", pady=(8, 0))

        ttk.Label(key, text="Tone (Hz):").grid(row=4, column=0, sticky="w", pady=(8, 0))
        tone_spin = ttk.Spinbox(key, from_=300, to=1200, increment=10, textvariable=self.tone_var, width=8,
                                command=self._on_keyer_params_changed)
        tone_spin.grid(row=4, column=1, sticky="w", pady=(8, 0))

        state = "normal" if self.sidetone.available() else "disabled"
        ttk.Checkbutton(key, text="Local PC Sidetone", variable=self.local_sidetone_var,
                        command=self._on_keyer_params_changed, state=state).grid(row=5, column=0, sticky="w", pady=(8, 0))
        if not self.sidetone.available():
            ttk.Label(key, text="(Install simpleaudio for local sidetone)").grid(row=5, column=1, sticky="w", pady=(8, 0))

        key.columnconfigure(1, weight=1)

        cmd = ttk.LabelFrame(self, text="Manual Commands", padding=10)
        cmd.pack(fill="x", padx=8, pady=8)
        self.manual_var = tk.StringVar()
        ttk.Entry(cmd, textvariable=self.manual_var).pack(side="left", fill="x", expand=True)
        ttk.Button(cmd, text="Send", command=self._send_manual).pack(side="left", padx=6)
        ttk.Button(cmd, text="STOP", command=lambda: self._send_cmd_safe("stop")).pack(side="left")

        logf = ttk.LabelFrame(self, text="Console", padding=8)
        logf.pack(fill="both", expand=True, padx=8, pady=8)
        self.log_text = tk.Text(logf, height=20)
        self.log_text.pack(fill="both", expand=True)

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_combo["values"] = labels
        self.port_var.set(labels[0])

    def _refresh_input_devices(self):
        devices = list_input_devices()
        self.device_map = {label: path for path, label in devices}
        labels = list(self.device_map.keys())
        self.device_combo["values"] = labels
        if labels:
            self.device_label_var.set(labels[0])
        if HAS_EVDEV:
            self._log("Input devices refreshed (/dev/input/eventX)", "info")
        else:
            self._log("python-evdev missing: keyboard fallback only", "error")

    def _connect(self):
        label = self.port_var.get()
        if not label or label.startswith("(no ports"):
            messagebox.showerror("Error", "No serial port selected")
            return
        try:
            self.worker.connect(self.port_map[label])
            self.status_var.set(f"🟢 Connected: {self.port_map[label]}")
            self._log(f"Connected to {self.port_map[label]}", "info")
            self._on_keyer_params_changed()
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))
            self._log(f"Connect failed: {exc}", "error")

    def _disconnect(self):
        self.keyer.disable()
        self.worker.disconnect()
        self.status_var.set("⚫ Disconnected")

    def _send_cmd_safe(self, cmd: str):
        try:
            if not self.worker.is_connected():
                self._log(f"[NOT CONNECTED] {cmd}", "error")
                return
            self.worker.send_line(cmd)
            self._log(f"> {cmd}", "sent")
        except Exception as exc:
            self._log(f"[SEND ERROR] {exc}", "error")

    def _toggle_external_keying(self):
        if self.keying_enabled_var.get():
            self._on_keyer_params_changed()
            self.keyer.enable()
            self._send_cmd_safe("mode cw")
            self._log("External keying enabled", "info")
        else:
            self.keyer.disable()
            self._send_cmd_safe("mode ssb")
            self._log("External keying disabled", "info")

    def _on_keyer_params_changed(self):
        mode = self.keyer_mode_var.get()
        wpm = max(5, min(60, int(self.wpm_var.get())))
        tone = max(300, min(1200, int(self.tone_var.get())))
        self.wpm_var.set(wpm)
        self.tone_var.set(tone)

        self.sidetone.enabled = bool(self.local_sidetone_var.get())
        self.sidetone.set_freq(tone)

        self.keyer.set_mode(mode)
        self.keyer.set_wpm(wpm)
        device_path = self.device_map.get(self.device_label_var.get(), "Keyboard (Space)")
        self.keyer.set_device(device_path)

        self._send_cmd_safe(f"sidetone {tone}")
        self._send_cmd_safe(f"wpm {wpm}")

    def _send_manual(self):
        cmd = self.manual_var.get().strip()
        if cmd:
            self._send_cmd_safe(cmd)
            self.manual_var.set("")

    def _on_key_press(self, event):
        self.keyer.keyboard_event(event.keysym, True)

    def _on_key_release(self, event):
        self.keyer.keyboard_event(event.keysym, False)

    def _log(self, msg: str, tag: str = "recv"):
        del tag
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")

    def _poll_rx(self):
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._log(line, "recv")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_rx)

    def _on_close(self):
        self.keyer.disable()
        self._send_cmd_safe("key 0")
        self._send_cmd_safe("stop")
        self.keyer.stop()
        self.worker.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
