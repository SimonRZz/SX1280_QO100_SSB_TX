#!/usr/bin/env python3
"""SX1280 Control + External CW Keying GUI."""

import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
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


@dataclass
class KeyerConfig:
    enabled: bool = False
    mode: str = "Straight"   # Straight / Iambic A / Iambic B
    wpm: int = 18
    tone_hz: int = 700
    local_sidetone: bool = False
    device_name: str = "Keyboard (Space)"


class SerialWorker:
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser = None
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.thread = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int = 115200):
        if not HAS_SERIAL:
            raise RuntimeError("pyserial fehlt (pip install pyserial)")
        with self.lock:
            if self.is_connected():
                return
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=0.5)
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
        data = (line.strip() + "\r\n").encode("utf-8", errors="replace")
        with self.lock:
            if not self.is_connected():
                raise RuntimeError("Nicht verbunden")
            self.ser.write(data)
            self.ser.flush()

    def get_modem_states(self):
        with self.lock:
            if not self.is_connected():
                return None
            return {
                "CTS": bool(self.ser.cts),
                "DSR": bool(self.ser.dsr),
                "RI": bool(getattr(self.ser, "ri", False)),
                "CD": bool(getattr(self.ser, "cd", False)),
            }

    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_evt.is_set():
            with self.lock:
                s = self.ser
            if not s:
                break
            try:
                chunk = s.read(256)
                if not chunk:
                    time.sleep(0.01)
                    continue
                buf.extend(chunk)
                while b"\n" in buf:
                    line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    self.rx_queue.put(line.decode("utf-8", errors="replace").rstrip("\r"))
            except Exception as e:
                self.rx_queue.put(f"[SERIAL ERROR] {e}")
                break


class SideTone:
    def __init__(self):
        self.play_obj = None

    def key_down(self, freq_hz: int):
        if not HAS_SIMPLEAUDIO:
            return
        import math
        import array
        sr = 24000
        duration = 0.25
        samples = int(sr * duration)
        amp = 8000
        data = array.array("h", (int(amp * math.sin(2.0 * math.pi * freq_hz * i / sr)) for i in range(samples)))
        self.key_up()
        self.play_obj = sa.play_buffer(data.tobytes(), 1, 2, sr)

    def key_up(self):
        if self.play_obj:
            try:
                self.play_obj.stop()
            except Exception:
                pass
            self.play_obj = None


class ExternalKeyerEngine:
    def __init__(self, send_key: Callable[[bool], None], sidetone: SideTone):
        self.send_key = send_key
        self.sidetone = sidetone
        self.cfg = KeyerConfig()
        self._thread = None
        self._stop_evt = threading.Event()
        self._lock = threading.Lock()
        self._dit_pressed = False
        self._dah_pressed = False
        self._last_element = None

    def set_paddles(self, dit: bool, dah: bool):
        with self._lock:
            self._dit_pressed = dit
            self._dah_pressed = dah

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        self.send_key(False)
        self.sidetone.key_up()

    def _key(self, down: bool):
        self.send_key(down)
        if self.cfg.local_sidetone:
            if down:
                self.sidetone.key_down(self.cfg.tone_hz)
            else:
                self.sidetone.key_up()

    def _loop(self):
        while not self._stop_evt.is_set():
            if not self.cfg.enabled:
                time.sleep(0.01)
                continue

            dit_t = 1.2 / max(5, min(60, int(self.cfg.wpm)))
            with self._lock:
                dit = self._dit_pressed
                dah = self._dah_pressed

            if self.cfg.mode == "Straight":
                want_down = dit or dah
                self._key(want_down)
                time.sleep(0.005)
                continue

            if not dit and not dah:
                self._key(False)
                time.sleep(0.002)
                continue

            next_elem = None
            if dit and dah:
                next_elem = "dah" if self._last_element == "dit" else "dit"
            elif dit:
                next_elem = "dit"
            elif dah:
                next_elem = "dah"

            self._last_element = next_elem
            on_t = dit_t if next_elem == "dit" else dit_t * 3.0

            self._key(True)
            t0 = time.monotonic()
            while time.monotonic() - t0 < on_t and not self._stop_evt.is_set():
                time.sleep(0.001)

            with self._lock:
                dit2 = self._dit_pressed
                dah2 = self._dah_pressed

            self._key(False)
            gap0 = time.monotonic()
            while time.monotonic() - gap0 < dit_t and not self._stop_evt.is_set():
                time.sleep(0.001)

            if self.cfg.mode == "Iambic B" and dit2 and dah2:
                self._last_element = "dit" if next_elem == "dah" else "dah"


class App(ttk.Frame):
    def __init__(self, root: tk.Tk):
        super().__init__(root)
        self.root = root
        self.root.title("Control + External CW Keying")
        self.root.geometry("900x700")

        self.rx_queue = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self.sidetone = SideTone()
        self.keyer = ExternalKeyerEngine(self._send_key_state, self.sidetone)

        self.evdev_stop = threading.Event()
        self.ftdi_stop = threading.Event()
        self.ftdi_map = {"dit": "CTS", "dah": "DSR"}

        self._build_vars()
        self._build_ui()
        self.keyer.start()
        self._poll_rx()

    def _build_vars(self):
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="⚫ Disconnected")
        self.enable_keying_var = tk.BooleanVar(value=False)
        self.device_var = tk.StringVar(value="Keyboard (Space)")
        self.mode_var = tk.StringVar(value="Straight")
        self.wpm_var = tk.IntVar(value=18)
        self.tone_var = tk.IntVar(value=700)
        self.local_tone_var = tk.BooleanVar(value=False)
        self.manual_var = tk.StringVar()
        self.dit_down = False
        self.dah_down = False

    def _build_ui(self):
        self.pack(fill="both", expand=True)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)

        conn = ttk.LabelFrame(self, text="Serial", padding=8)
        conn.grid(row=0, column=0, sticky="ew", padx=8, pady=6)
        conn.columnconfigure(1, weight=1)

        ttk.Label(conn, text="Port:").grid(row=0, column=0)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Button(conn, text="Connect", command=self._connect).grid(row=0, column=3, padx=3)
        ttk.Button(conn, text="Disconnect", command=self._disconnect).grid(row=0, column=4, padx=3)
        ttk.Label(conn, textvariable=self.status_var).grid(row=0, column=5, padx=8)

        keyp = ttk.LabelFrame(self, text="External Keying", padding=8)
        keyp.grid(row=1, column=0, sticky="ew", padx=8, pady=6)
        keyp.columnconfigure(1, weight=1)

        ttk.Checkbutton(keyp, text="Enable External Keying", variable=self.enable_keying_var,
                        command=self._toggle_keying).grid(row=0, column=0, sticky="w")

        ttk.Label(keyp, text="Device:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        devs = ["Keyboard (Space)"]
        if HAS_EVDEV:
            devs.extend([f"evdev: {d.path}" for d in evdev.list_devices()])
        devs.append("FTDI modem lines")
        self.dev_combo = ttk.Combobox(keyp, textvariable=self.device_var, values=devs, state="readonly")
        self.dev_combo.grid(row=1, column=1, sticky="ew", pady=(8, 0))
        self.dev_combo.bind("<<ComboboxSelected>>", lambda _e: self._device_changed())

        ttk.Label(keyp, text="Mode:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(keyp, textvariable=self.mode_var, state="readonly",
                     values=["Straight", "Iambic A", "Iambic B"]).grid(row=2, column=1, sticky="ew", pady=(8, 0))

        ttk.Label(keyp, text="WPM:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Scale(keyp, from_=5, to=60, variable=self.wpm_var, orient="horizontal",
                  command=lambda _v: self._send_keyer_params()).grid(row=3, column=1, sticky="ew", pady=(8, 0))

        ttk.Label(keyp, text="Tone (Hz):").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Scale(keyp, from_=300, to=1200, variable=self.tone_var, orient="horizontal",
                  command=lambda _v: self._send_keyer_params()).grid(row=4, column=1, sticky="ew", pady=(8, 0))

        ttk.Checkbutton(keyp, text="Local PC sidetone (simpleaudio)", variable=self.local_tone_var,
                        command=self._sync_keyer_cfg).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))

        mbar = tk.Menu(self.root)
        tools = tk.Menu(mbar, tearoff=0)
        tools.add_command(label="FTDI Mapping...", command=self._open_ftdi_mapping)
        mbar.add_cascade(label="Tools", menu=tools)
        self.root.config(menu=mbar)

        manual = ttk.LabelFrame(self, text="Manual Commands", padding=8)
        manual.grid(row=2, column=0, sticky="ew", padx=8, pady=6)
        manual.columnconfigure(0, weight=1)
        entry = ttk.Entry(manual, textvariable=self.manual_var)
        entry.grid(row=0, column=0, sticky="ew")
        entry.bind("<Return>", lambda _e: self._send_manual())
        ttk.Button(manual, text="Send", command=self._send_manual).grid(row=0, column=1, padx=6)

        console = ttk.LabelFrame(self, text="Console", padding=8)
        console.grid(row=3, column=0, sticky="nsew", padx=8, pady=6)
        console.columnconfigure(0, weight=1)
        console.rowconfigure(0, weight=1)
        self.log = tk.Text(console, font=("Consolas", 9))
        self.log.grid(row=0, column=0, sticky="nsew")
        ttk.Scrollbar(console, command=self.log.yview).grid(row=0, column=1, sticky="ns")

        self.root.bind("<KeyPress-space>", lambda _e: self._set_keyboard(True))
        self.root.bind("<KeyRelease-space>", lambda _e: self._set_keyboard(False))
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_ports()

    def _log(self, text: str):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    def _refresh_ports(self):
        ports = []
        if HAS_SERIAL:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _connect(self):
        try:
            self.worker.connect(self.port_var.get())
            self.status_var.set("🟢 Connected")
            self._log(f"Connected: {self.port_var.get()}")
        except Exception as e:
            messagebox.showerror("Connect", str(e))

    def _disconnect(self):
        self.worker.disconnect()
        self.status_var.set("⚫ Disconnected")
        self._log("Disconnected")

    def _send_cmd(self, cmd: str):
        try:
            self.worker.send_line(cmd)
            self._log(f"> {cmd}")
        except Exception as e:
            self._log(f"[ERR] {e}")

    def _send_manual(self):
        cmd = self.manual_var.get().strip()
        if cmd:
            self._send_cmd(cmd)
            self.manual_var.set("")

    def _send_key_state(self, down: bool):
        self._send_cmd(f"key {1 if down else 0}")

    def _sync_keyer_cfg(self):
        self.keyer.cfg.mode = self.mode_var.get()
        self.keyer.cfg.wpm = max(5, min(60, int(self.wpm_var.get())))
        self.keyer.cfg.tone_hz = max(300, min(1200, int(self.tone_var.get())))
        self.keyer.cfg.local_sidetone = bool(self.local_tone_var.get())

    def _send_keyer_params(self):
        self._sync_keyer_cfg()
        self._send_cmd(f"sidetone {self.keyer.cfg.tone_hz}")
        self._send_cmd(f"wpm {self.keyer.cfg.wpm}")

    def _toggle_keying(self):
        self._sync_keyer_cfg()
        enabled = bool(self.enable_keying_var.get())
        self.keyer.cfg.enabled = enabled
        if enabled:
            self._send_cmd("mode cw")
            self._send_keyer_params()
        else:
            self._send_cmd("mode ssb")
            self._send_key_state(False)

    def _device_changed(self):
        self._stop_input_threads()
        dev = self.device_var.get()
        if dev.startswith("evdev:"):
            self._start_evdev(dev.split("evdev:", 1)[1].strip())
        elif dev == "FTDI modem lines":
            self._start_ftdi_poll()

    def _set_keyboard(self, down: bool):
        if self.device_var.get() != "Keyboard (Space)":
            return
        self.dit_down = down
        self.keyer.set_paddles(self.dit_down, self.dah_down)

    def _start_evdev(self, path: str):
        if not HAS_EVDEV:
            return
        self.evdev_stop.clear()

        def run():
            try:
                dev = evdev.InputDevice(path)
                for e in dev.read_loop():
                    if self.evdev_stop.is_set():
                        break
                    if e.type == evdev.ecodes.EV_KEY and e.code == evdev.ecodes.KEY_SPACE:
                        self.dit_down = e.value != 0
                        self.keyer.set_paddles(self.dit_down, self.dah_down)
            except Exception as ex:
                self._log(f"[evdev] {ex}")

        threading.Thread(target=run, daemon=True).start()

    def _start_ftdi_poll(self):
        self.ftdi_stop.clear()

        def run():
            while not self.ftdi_stop.is_set():
                st = self.worker.get_modem_states()
                if st:
                    self.dit_down = st.get(self.ftdi_map["dit"], False)
                    self.dah_down = st.get(self.ftdi_map["dah"], False)
                    self.keyer.set_paddles(self.dit_down, self.dah_down)
                time.sleep(0.01)

        threading.Thread(target=run, daemon=True).start()

    def _stop_input_threads(self):
        self.evdev_stop.set()
        self.ftdi_stop.set()

    def _open_ftdi_mapping(self):
        dlg = tk.Toplevel(self.root)
        dlg.title("FTDI Mapping")
        dlg.transient(self.root)
        lines = ["CTS", "DSR", "RI", "CD"]
        dit_var = tk.StringVar(value=self.ftdi_map["dit"])
        dah_var = tk.StringVar(value=self.ftdi_map["dah"])
        ttk.Label(dlg, text="DIT line").grid(row=0, column=0, padx=8, pady=6)
        ttk.Combobox(dlg, values=lines, textvariable=dit_var, state="readonly").grid(row=0, column=1, padx=8)
        ttk.Label(dlg, text="DAH line").grid(row=1, column=0, padx=8, pady=6)
        ttk.Combobox(dlg, values=lines, textvariable=dah_var, state="readonly").grid(row=1, column=1, padx=8)

        def apply_map():
            self.ftdi_map = {"dit": dit_var.get(), "dah": dah_var.get()}
            self._log(f"FTDI mapping: DIT={self.ftdi_map['dit']} DAH={self.ftdi_map['dah']}")
            dlg.destroy()

        ttk.Button(dlg, text="Apply", command=apply_map).grid(row=2, column=0, columnspan=2, pady=8)

    def _poll_rx(self):
        try:
            while True:
                self._log(self.rx_queue.get_nowait())
        except queue.Empty:
            pass
        self.root.after(50, self._poll_rx)

    def _on_close(self):
        self._send_cmd("key 0")
        self._send_cmd("stop")
        self.keyer.stop()
        self._stop_input_threads()
        self.worker.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
