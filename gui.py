#!/usr/bin/env python3
"""
SX1280 QO-100 SSB TX Control GUI
Original: SP8ESA  |  CW Keyer Tab: erweitert  |  v2.0.0 Status Sync
pip install pyserial pyaudio numpy
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import ctypes
import ctypes.util
import sys
from dataclasses import dataclass
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False

try:
    import pyaudio
    import numpy as np
    HAS_AUDIO = True
except ImportError:
    HAS_AUDIO = False


# ALSA-Fehlermeldungen unterdrücken
def _suppress_alsa_errors():
    try:
        lib = ctypes.util.find_library('asound')
        if not lib:
            return
        asound = ctypes.cdll.LoadLibrary(lib)
        _CB_TYPE = ctypes.CFUNCTYPE(
            None,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_char_p,
        )
        sys.modules[__name__]._alsa_error_cb = _CB_TYPE(lambda *a: None)
        asound.snd_lib_error_set_handler(sys.modules[__name__]._alsa_error_cb)
    except Exception:
        pass

_suppress_alsa_errors()


MORSE_TABLE = {
    '.-':'A',    '-...':'B',  '-.-.':'C',  '-..':'D',
    '.':'E',     '..-.':'F',  '--.':'G',   '....':'H',
    '..':'I',    '.---':'J',  '-.-':'K',   '.-..':'L',
    '--':'M',    '-.':'N',    '---':'O',   '.--.':'P',
    '--.-':'Q',  '.-.':'R',   '...':'S',   '-':'T',
    '..-':'U',   '...-':'V',  '.--':'W',   '-..-':'X',
    '-.--':'Y',  '--..':'Z',
    '.----':'1', '..---':'2', '...--':'3', '....-':'4',
    '.....':'5', '-....':'6', '--...':'7', '---..':'8',
    '----.':'9', '-----':'0',
    '.-.-.-':'.', '--..--':',', '..--..':'?', '.----.': "'",
    '-..-.':'/',  '---...':':',  '-.--.':'(',  '-.--.-':')',
    '.-...':'&',  '-...-':'=',   '.-.-.':'+',  '-....-':'-',
    '.-..-.':'"', '.--.-.':'@',
}

# Inverse table: char → morse code string (for text-to-CW sending)
MORSE_CODE = {v: k for k, v in MORSE_TABLE.items()}
MORSE_CODE[' '] = ' '  # word space

READABLE_PINS = ['CTS', 'DSR', 'RI', 'CD']

QO100_LO_HZ = 8_089_500_100  # Measured QO-100 transponder LO offset, nominal 8089.5 MHz


@dataclass
class TxConfig:
    freq_hz: float = 2_400_400_000.0
    ppm: float = 0.0
    tx_power_dbm: int = 4
    tx_enabled: bool = True
    enable_bp: bool = True
    enable_eq: bool = True
    enable_comp: bool = True
    bp_lo_hz: float = 50.0
    bp_hi_hz: float = 2700.0
    bp_stages: int = 7
    eq_low_hz: float = 190.0
    eq_low_db: float = -2.0
    eq_high_hz: float = 1700.0
    eq_high_db: float = 13.5
    comp_thr_db: float = -2.5
    comp_ratio: float = 6.1
    comp_attack_ms: float = 41.1
    comp_release_ms: float = 1595.0
    comp_makeup_db: float = 0.0
    comp_knee_db: float = 16.5
    comp_out_limit: float = 0.940
    amp_gain: float = 2.9
    amp_min_a: float = 0.000002


class SerialWorker:
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser = None
        self.thread = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud=115200):
        if not HAS_SERIAL:
            raise RuntimeError("pyserial not installed")
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
            try: s.close()
            except: pass

    def send_line(self, line):
        line = line.strip()
        if not line:
            return
        data = (line + "\r\n").encode("utf-8", errors="replace")
        with self.lock:
            if not self.is_connected():
                raise RuntimeError("Not connected")
            self.ser.write(data)
            self.ser.flush()

    def _rx_loop(self):
        buf = bytearray()
        while not self.stop_evt.is_set():
            with self.lock:
                s = self.ser
            if not s:
                break
            try:
                chunk = s.read(256)
                if chunk:
                    buf.extend(chunk)
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        txt = line.decode("utf-8", errors="replace").rstrip("\r")
                        self.rx_queue.put(txt)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.rx_queue.put(f"[SERIAL ERROR] {e}")
                break


class AudioEngine:
    def __init__(self):
        self.sample_rate = 44100
        self.tone_freq   = 700
        self.volume      = 0.5
        self._playing    = False
        self._lock       = threading.Lock()
        self._running    = False
        self._thread     = None
        self.pa          = None
        self.available   = False

        if not HAS_AUDIO:
            return
        try:
            self.pa = pyaudio.PyAudio()
            self._running = True
            self.available = True
            self._thread = threading.Thread(target=self._watchdog, daemon=True)
            self._thread.start()
        except Exception as e:
            print(f"Audio init: {e}")

    _CHUNK    = 256          # samples per buffer (≈5.8 ms @ 44100 Hz)
    _RAMP_MS  = 8            # attack/release ramp length in milliseconds

    def _open_stream(self):
        try:
            return self.pa.open(
                format=pyaudio.paInt16, channels=1,
                rate=self.sample_rate, output=True,
                frames_per_buffer=self._CHUNK)
        except Exception:
            # pa selbst neu initialisieren
            try: self.pa.terminate()
            except: pass
            try:
                self.pa = pyaudio.PyAudio()
                return self.pa.open(
                    format=pyaudio.paInt16, channels=1,
                    rate=self.sample_rate, output=True,
                    frames_per_buffer=self._CHUNK)
            except Exception:
                return None

    def _watchdog(self):
        while self._running:
            stream = self._open_stream()
            if stream is None:
                time.sleep(2.0)
                continue
            self._stream_loop(stream)
            if self._running:
                time.sleep(0.5)

    def _stream_loop(self, stream):
        chunk        = self._CHUNK
        ramp_samples = max(1, int(self.sample_rate * self._RAMP_MS / 1000))
        max_step     = chunk / ramp_samples   # max volume change per chunk
        current_vol  = 0.0
        phase        = 0
        try:
            while self._running:
                with self._lock:
                    target = self.volume if self._playing else 0.0
                    freq   = self.tone_freq
                # Linear ramp: spread the volume change evenly across the chunk
                # so there are no amplitude steps at chunk boundaries → no clicks.
                delta   = max(min(target - current_vol, max_step), -max_step)
                env     = np.linspace(current_vol, current_vol + delta,
                                      chunk, endpoint=False, dtype=np.float32)
                current_vol += delta
                t = (np.arange(chunk, dtype=np.float32) + phase) / self.sample_rate
                s = (np.sin(2 * np.pi * freq * t) * env * 32767).astype(np.int16)
                phase = (phase + chunk) % self.sample_rate
                try:
                    stream.write(s.tobytes(), exception_on_underflow=False)
                except Exception:
                    return
        except Exception:
            return
        finally:
            try: stream.stop_stream()
            except: pass
            try: stream.close()
            except: pass

    def on(self):
        with self._lock: self._playing = True

    def off(self):
        with self._lock: self._playing = False

    def set_freq(self, f):
        with self._lock: self.tone_freq = max(200, min(2000, int(f)))

    def set_vol(self, v):
        with self._lock: self.volume = max(0.0, min(1.0, float(v)))

    def close(self):
        self._running = False
        self._playing = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.pa:
            try: self.pa.terminate()
            except: pass


class KeyReader:
    def __init__(self, port, baud, dit_pin, dah_pin, active_low):
        self.port       = port
        self.baud       = baud
        self.dit_pin    = dit_pin.upper()
        self.dah_pin    = dah_pin.upper() if dah_pin else None
        self.active_low = active_low
        self.ser        = None

    def connect(self):
        try:
            self.ser = serial.Serial(
                self.port, self.baud,
                timeout=0, dsrdtr=False, rtscts=False)
            self.ser.dtr = True
            self.ser.rts = True
            return True, ""
        except Exception as e:
            self.ser = None
            return False, str(e)

    def _pin(self, name):
        if not self.ser or name not in READABLE_PINS:
            return False
        try:
            raw = {'CTS': self.ser.cts, 'DSR': self.ser.dsr,
                   'RI':  self.ser.ri,  'CD':  self.ser.cd}[name]
            return (not raw) if self.active_low else raw
        except:
            return False

    def read(self):
        return self._pin(self.dit_pin), \
               (self._pin(self.dah_pin) if self.dah_pin else False)

    def disconnect(self):
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None


class Keyer:
    STRAIGHT = 0
    IAMBIC_A = 1
    IAMBIC_B = 2

    # Contact-bounce filter: input must be stable for this many ticks (2 ms each)
    # before a state change is accepted.  3 ticks = 6 ms covers typical paddle bounce.
    _DEBOUNCE_TICKS = 3

    def __init__(self):
        self.mode       = self.IAMBIC_A
        self.wpm        = 18
        self.weight     = 3.0
        self._update_timing()
        self._state     = 'IDLE'
        self._t0        = 0.0
        self._sym_buf   = ''
        self._pend_dit  = False
        self._pend_dah  = False
        self._was_dit   = False
        # Debounced paddle state (what _iambic sees)
        self._dit_stable = False
        self._dah_stable = False
        # How many consecutive ticks the raw input has differed from stable state
        self._dit_db_cnt = 0
        self._dah_db_cnt = 0
        self.cb_key_on  = None
        self.cb_key_off = None
        self.cb_char    = None
        self.cb_word_sp = None
        self._lock      = threading.Lock()

    def _update_timing(self):
        self.dit_ms = 1200.0 / self.wpm
        self.dah_ms = self.dit_ms * self.weight
        self.iel_ms = self.dit_ms
        self.ich_ms = self.dit_ms * 3.0
        self.iwd_ms = self.dit_ms * 7.0

    def set_wpm(self, w):
        self.wpm = max(5, min(60, int(w)))
        self._update_timing()

    def set_weight(self, w):
        self.weight = max(2.0, min(5.0, float(w)))
        self._update_timing()

    def set_mode(self, m):
        self.mode = m

    def tick(self, dit_in, dah_in):
        try:
            with self._lock:
                now = time.monotonic() * 1000.0

                # --- debounce dit ---
                if dit_in != self._dit_stable:
                    self._dit_db_cnt += 1
                    if self._dit_db_cnt >= self._DEBOUNCE_TICKS:
                        self._dit_stable = dit_in
                        self._dit_db_cnt = 0
                        if dit_in:          # rising edge on debounced signal
                            self._pend_dit = True
                else:
                    self._dit_db_cnt = 0

                # --- debounce dah ---
                if dah_in != self._dah_stable:
                    self._dah_db_cnt += 1
                    if self._dah_db_cnt >= self._DEBOUNCE_TICKS:
                        self._dah_stable = dah_in
                        self._dah_db_cnt = 0
                        if dah_in:          # rising edge on debounced signal
                            self._pend_dah = True
                else:
                    self._dah_db_cnt = 0

                if self.mode == self.STRAIGHT:
                    return self._straight(self._dit_stable, now)
                return self._iambic(self._dit_stable, self._dah_stable, now)
        except:
            return False

    def _straight(self, key, now):
        if key:
            if self._state == 'IDLE':
                self._state = 'DOWN'
                self._t0 = now
                self._sym_buf = ''
                if self.cb_key_on: self.cb_key_on()
            return True
        else:
            if self._state == 'DOWN':
                dur = now - self._t0
                self._state = 'IDLE'
                self._t0 = now
                if self.cb_key_off: self.cb_key_off()
                self._sym_buf += '.' if dur < self.dit_ms * 2.0 else '-'
            elif self._state == 'IDLE' and self._sym_buf:
                if now - self._t0 >= self.ich_ms:
                    ch = MORSE_TABLE.get(self._sym_buf, '?')
                    self._sym_buf = ''
                    if self.cb_char: self.cb_char(ch)
            return False

    def _iambic(self, dit, dah, now):
        # FIX A: removed level-triggered "if dit: self._pend_dit = True" lines.
        # Pending flags are now set exclusively by rising-edge detection in tick().
        el = now - self._t0

        if self._state == 'IDLE':
            if self._pend_dit and not self._pend_dah:
                self._send_dit(now); self._pend_dit = False
            elif self._pend_dah and not self._pend_dit:
                self._send_dah(now); self._pend_dah = False
            elif self._pend_dit and self._pend_dah:
                if self._was_dit:
                    self._send_dah(now); self._pend_dah = False
                else:
                    self._send_dit(now); self._pend_dit = False
            return False

        elif self._state == 'DIT':
            if el >= self.dit_ms:
                self._sym_buf += '.'
                self._was_dit = True
                if self.cb_key_off: self.cb_key_off()
                # Capture same-paddle level at element end for hold-to-repeat.
                # This mirrors the WB4VVF/uSDX sticky-latch approach: latch is
                # cleared at element START (_send_dit clears _pend_dit), then
                # re-latched here if the paddle is still held.
                if dit: self._pend_dit = True
                if self.mode == self.IAMBIC_A:
                    # A: clear memory for any paddle released at element end
                    if not dit: self._pend_dit = False
                    if not dah: self._pend_dah = False
                self._state = 'IEL'; self._t0 = now
            elif self.mode == self.IAMBIC_B and dah:
                # Iambic B squeeze latch: opposite paddle held during element
                self._pend_dah = True
            return True

        elif self._state == 'DAH':
            if el >= self.dah_ms:
                self._sym_buf += '-'
                self._was_dit = False
                if self.cb_key_off: self.cb_key_off()
                # Capture same-paddle level at element end for hold-to-repeat
                if dah: self._pend_dah = True
                if self.mode == self.IAMBIC_A:
                    if not dit: self._pend_dit = False
                    if not dah: self._pend_dah = False
                self._state = 'IEL'; self._t0 = now
            elif self.mode == self.IAMBIC_B and dit:
                # Iambic B squeeze latch (symmetric for dit side)
                self._pend_dit = True
            return True

        elif self._state == 'IEL':
            if el >= self.iel_ms:
                have_dit = self._pend_dit
                have_dah = self._pend_dah
                if have_dit and have_dah:
                    # Squeeze: alternate based on what was last sent
                    if self._was_dit:
                        self._send_dah(now); self._pend_dah = False
                    else:
                        self._send_dit(now); self._pend_dit = False
                elif have_dah:
                    self._send_dah(now); self._pend_dah = False
                elif have_dit:
                    self._send_dit(now); self._pend_dit = False
                else:
                    self._state = 'ICH'; self._t0 = now
            return False

        elif self._state == 'ICH':
            if el >= self.ich_ms:
                # Complete character
                ch = MORSE_TABLE.get(self._sym_buf, '?')
                self._sym_buf = ''
                if self.cb_char: self.cb_char(ch)
                # FIX A: do NOT copy current key level into pending here.
                # Rising-edge detection in tick() already set _pend_* if user
                # pressed a key during ICH – no need to re-read the level.
                if self._pend_dit or self._pend_dah:
                    self._state = 'IDLE'; self._t0 = now
                else:
                    self._state = 'IWD'; self._t0 = now
            # FIX A: only check _pend_* (edge-based), not raw dit/dah level
            elif self._pend_dit or self._pend_dah:
                if self._pend_dah:
                    self._send_dah(now); self._pend_dah = False
                else:
                    self._send_dit(now); self._pend_dit = False
            return False

        elif self._state == 'IWD':
            if el >= self.iwd_ms - self.ich_ms:
                if self.cb_word_sp: self.cb_word_sp()
                self._state = 'IDLE'; self._t0 = now
            # FIX A: only check _pend_* (edge-based), not raw dit/dah level
            elif self._pend_dit or self._pend_dah:
                self._state = 'IDLE'; self._t0 = now
            return False

        return False

    def _send_dit(self, now):
        self._pend_dit = False   # clear BOTH latches at element start
        self._pend_dah = False   # mirrors uSDX KEYED_PREP: keyerControl &= ~(DIT_L|DAH_L)
        self._state = 'DIT'; self._t0 = now
        if self.cb_key_on: self.cb_key_on()

    def _send_dah(self, now):
        self._pend_dit = False   # clear BOTH latches at element start
        self._pend_dah = False   # mirrors uSDX KEYED_PREP: keyerControl &= ~(DIT_L|DAH_L)
        self._state = 'DAH'; self._t0 = now
        if self.cb_key_on: self.cb_key_on()

    def get_sym_buf(self):
        try:
            with self._lock: return self._sym_buf
        except:
            return ''

    def reset(self):
        try:
            with self._lock:
                self._state = 'IDLE'
                self._sym_buf = ''
                self._pend_dit = self._pend_dah = False
                self._dit_stable = self._dah_stable = False
                self._dit_db_cnt = self._dah_db_cnt = 0
                if self.cb_key_off: self.cb_key_off()
        except:
            pass


def list_serial_ports():
    if not HAS_SERIAL:
        return []
    ports = []
    for p in serial.tools.list_ports.comports():
        if "SX1280" in p.description or "cafe:4073" in str(p.hwid).lower():
            ports.insert(0, (p.device, f"★ {p.device} ({p.description})"))
        else:
            ports.append((p.device, f"{p.device} ({p.description})"))
    return ports


class Debouncer:
    def __init__(self, tk_root, delay_ms, fn):
        self.root = tk_root
        self.delay_ms = delay_ms
        self.fn = fn
        self._after_id = None
        self._last_args = None
        self._last_kwargs = None

    def call(self, *args, **kwargs):
        self._last_args = args
        self._last_kwargs = kwargs
        if self._after_id is not None:
            try: self.root.after_cancel(self._after_id)
            except: pass
        self._after_id = self.root.after(self.delay_ms, self._fire)

    def _fire(self):
        self._after_id = None
        if self._last_args is not None:
            self.fn(*self._last_args, **(self._last_kwargs or {}))


class LabeledScale(ttk.Frame):
    def __init__(self, parent, label, var, from_, to, resolution, on_change, format_str="{:.1f}"):
        super().__init__(parent)
        self.var = var
        self.resolution = resolution
        self.on_change = on_change
        self.format_str = format_str
        self.columnconfigure(1, weight=1)
        ttk.Label(self, text=label, width=16, anchor="w").grid(row=0, column=0, sticky="w")
        self.scale = ttk.Scale(self, from_=from_, to=to, orient=tk.HORIZONTAL,
                               variable=var, command=self._on_scale)
        self.scale.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        self.value_label = ttk.Label(self, width=10, anchor="e")
        self.value_label.grid(row=0, column=2, sticky="e")
        self._update_value_label()
        self.scale.bind("<ButtonRelease-1>", self._on_release)
        # Auto-update label on programmatic .set() (e.g. from !S status push)
        var.trace_add("write", lambda *_: self._update_value_label())

    def _on_scale(self, _val):
        self._update_value_label()

    def _on_release(self, _event):
        v = float(self.var.get())
        v = round(v / self.resolution) * self.resolution
        self.var.set(v)
        self._update_value_label()
        self.on_change(v)

    def _update_value_label(self):
        v = float(self.var.get())
        text = self.format_str(v) if callable(self.format_str) else self.format_str.format(v)
        self.value_label.config(text=text)


class ScrollableFrame(ttk.Frame):
    """Frame with vertical scrollbar – used for the RF & DSP tab."""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.inner = ttk.Frame(self.canvas)
        self.inner.bind("<Configure>",   self._on_inner_cfg)
        self.canvas.bind("<Configure>",  self._on_canvas_cfg)
        self._win = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")
        self.canvas.bind("<Enter>", lambda _: self._bind_wheel())
        self.canvas.bind("<Leave>", lambda _: self._unbind_wheel())

    def _on_inner_cfg(self, _e):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_cfg(self, e):
        self.canvas.itemconfig(self._win, width=e.width)

    def _bind_wheel(self):
        self.canvas.bind_all("<Button-4>",   self._scroll)
        self.canvas.bind_all("<Button-5>",   self._scroll)
        self.canvas.bind_all("<MouseWheel>", self._scroll)

    def _unbind_wheel(self):
        for ev in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            self.canvas.unbind_all(ev)

    def _scroll(self, e):
        if   e.num == 4: self.canvas.yview_scroll(-3, "units")
        elif e.num == 5: self.canvas.yview_scroll( 3, "units")
        elif hasattr(e, 'delta'): self.canvas.yview_scroll(-1*(e.delta//120), "units")


class SX1280ControlApp(ttk.Frame):
    FREQ_MIN_HZ = 2_400_000_000
    FREQ_MAX_HZ = 2_400_500_000

    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.config  = TxConfig()
        self.rx_queue = queue.Queue()
        self.worker  = SerialWorker(self.rx_queue)
        self.debounced_send = Debouncer(master, 150, self._send_cmd_safe)
        self.freq_debouncer = Debouncer(master, 200, self._send_freq)

        self._status_updating = False  # guard against feedback loops during !S sync
        self._heartbeat_id   = None    # after() id for periodic status requests

        self.audio       = AudioEngine()
        self.keyer       = Keyer()
        self.key_reader  = None
        self._cw_running      = False
        self._cw_thread       = None
        self._cw_text_thread  = None
        self._cw_stop_evt     = threading.Event()
        # FIX B: shared key state written only by _cw_loop (background thread),
        # read only by _cw_gui_update (GUI thread) – eliminates serial port race.
        self._cw_key_state = (False, False)
        self._cw_tx_active        = False  # True while firmware is in CW keyer mode
        self._cw_test_stop_pending = False  # True when test carrier stop triggered (not keyer stop)

        # Keyer callbacks: sidetone + firmware keying (key 1 / key 0)
        def _on_key_on():
            self.audio.on()
            if self._cw_tx_active:
                try: self.worker.send_line("key 1")
                except: pass
        def _on_key_off():
            self.audio.off()
            if self._cw_tx_active:
                try: self.worker.send_line("key 0")
                except: pass
        self.keyer.cb_key_on  = _on_key_on
        self.keyer.cb_key_off = _on_key_off

        self._create_variables()
        self._build_ui()
        self._update_freq_display()

        master.bind_all("<Button-4>",   self._on_global_scroll)
        master.bind_all("<Button-5>",   self._on_global_scroll)
        master.bind_all("<MouseWheel>", self._on_global_scroll)
        master.bind_all("<Escape>",     self._cw_on_esc)

        self._poll_rx()
        self._cw_gui_update()
        self.pack(fill="both", expand=True)

    def _create_variables(self):
        self.port_var    = tk.StringVar()
        self.status_var  = tk.StringVar(value="⚫ Disconnected")
        self.mode_var    = tk.StringVar(value="usb")
        self.tune_var    = tk.BooleanVar(value=False)
        self.freq_mhz_var  = tk.DoubleVar(value=self.config.freq_hz / 1_000_000)
        self.freq_khz_var  = tk.StringVar(value=f"{self.config.freq_hz / 1000:.1f}")
        _dl0 = self.config.freq_hz + QO100_LO_HZ
        self.dl_mhz_var    = tk.DoubleVar(value=_dl0 / 1_000_000)
        self.dl_khz_var    = tk.StringVar(value=f"{_dl0 / 1000:.1f}")
        self.ppm_var      = tk.DoubleVar(value=0.0)
        self.txpwr_var    = tk.IntVar(value=self.config.tx_power_dbm)
        self.tx_enabled_var          = tk.BooleanVar(value=True)
        self.scroll_tune_enabled_var = tk.BooleanVar(value=False)
        self.en_bp_var    = tk.BooleanVar(value=self.config.enable_bp)
        self.en_eq_var    = tk.BooleanVar(value=self.config.enable_eq)
        self.en_comp_var  = tk.BooleanVar(value=self.config.enable_comp)
        self.bp_lo_var    = tk.DoubleVar(value=self.config.bp_lo_hz)
        self.bp_hi_var    = tk.DoubleVar(value=self.config.bp_hi_hz)
        self.bp_stages_var = tk.IntVar(value=self.config.bp_stages)
        self.eq_low_hz_var  = tk.DoubleVar(value=self.config.eq_low_hz)
        self.eq_low_db_var  = tk.DoubleVar(value=self.config.eq_low_db)
        self.eq_high_hz_var = tk.DoubleVar(value=self.config.eq_high_hz)
        self.eq_high_db_var = tk.DoubleVar(value=self.config.eq_high_db)
        self.comp_thr_var    = tk.DoubleVar(value=self.config.comp_thr_db)
        self.comp_ratio_var  = tk.DoubleVar(value=self.config.comp_ratio)
        self.comp_att_var    = tk.DoubleVar(value=self.config.comp_attack_ms)
        self.comp_rel_var    = tk.DoubleVar(value=self.config.comp_release_ms)
        self.comp_makeup_var = tk.DoubleVar(value=self.config.comp_makeup_db)
        self.comp_knee_var   = tk.DoubleVar(value=self.config.comp_knee_db)
        self.comp_outlim_var = tk.DoubleVar(value=self.config.comp_out_limit)
        self.amp_gain_var    = tk.DoubleVar(value=self.config.amp_gain)
        self.amp_min_a_var   = tk.StringVar(value=f"{self.config.amp_min_a:.9f}")
        self.cw_port_var        = tk.StringVar()
        self.cw_baud_var        = tk.StringVar(value='9600')
        self.cw_mode_var        = tk.StringVar(value='Iambic B')
        self.cw_dit_var         = tk.StringVar(value='CTS')
        self.cw_dah_var         = tk.StringVar(value='CD')
        self.cw_active_low_var  = tk.BooleanVar(value=False)
        self.cw_wpm_var         = tk.DoubleVar(value=18)
        self.cw_tone_var        = tk.DoubleVar(value=700)
        self.cw_vol_var         = tk.DoubleVar(value=70)
        self.cw_conn_status_var = tk.StringVar(value='● DISCONNECTED')
        self.cw_text_var        = tk.StringVar(value='CQ CQ DE SX1280')
        self.cw_weight_var      = tk.DoubleVar(value=3.0)
        # GPSDO state
        self.gpsdo_sig_var  = tk.StringVar(value="--")
        self.gpsdo_fix_var  = tk.StringVar(value="--")
        self.gpsdo_sats_var = tk.StringVar(value="--")
        self.gpsdo_vis_var  = tk.StringVar(value="--")
        self.gpsdo_clk1_var = tk.StringVar(value="--")
        self.gpsdo_utc_var  = tk.StringVar(value="--:--:--")
        self.gpsdo_loc_var  = tk.StringVar(value="------")
        self.gpsdo_alt_var  = tk.StringVar(value="--")

    def _build_ui(self):
        self.master.title("SX1280 QO-100 SSB TX Control")
        self.master.geometry("900x820")
        self.master.minsize(600, 500)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self._build_connection_bar()
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self._build_dsp_tab()
        self._build_tx_tab()
        self._build_cw_tab()
        self._build_gpsdo_tab()
        self._build_console_tab()

    def _build_connection_bar(self):
        f = ttk.Frame(self)
        f.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        f.columnconfigure(1, weight=1)
        ttk.Label(f, text="Port:").grid(row=0, column=0, padx=(0, 5))
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_var.set(labels[0] if labels else "")
        self.port_combo = ttk.Combobox(f, textvariable=self.port_var,
                                        values=labels, state="readonly", width=45)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5)
        bf = ttk.Frame(f)
        bf.grid(row=0, column=2)
        ttk.Button(bf, text="🔄", width=3, command=self._refresh_ports).pack(side="left", padx=2)
        ttk.Button(bf, text="Connect",    command=self._connect).pack(side="left", padx=2)
        ttk.Button(bf, text="Disconnect", command=self._disconnect).pack(side="left", padx=2)
        ttk.Label(f, textvariable=self.status_var).grid(row=0, column=3, padx=(10, 0))

    def _build_dsp_tab(self):
        sc = ScrollableFrame(self.notebook)
        self.notebook.add(sc, text="RF & DSP")
        tab = sc.inner
        tab.columnconfigure(0, weight=1)

        rf = ttk.LabelFrame(tab, text="RF / Frequency", padding=10)
        rf.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        rf.columnconfigure(1, weight=1)

        tbf = ttk.Frame(rf)
        tbf.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        # Mode (USB/CW) radio buttons
        ttk.Label(tbf, text="Mode:").pack(side="left", padx=(0, 4))
        ttk.Radiobutton(tbf, text="USB (SSB)", variable=self.mode_var,
                        value="usb", command=self._on_mode_change).pack(side="left", padx=3)
        ttk.Radiobutton(tbf, text="CW", variable=self.mode_var,
                        value="cw",  command=self._on_mode_change).pack(side="left", padx=3)
        ttk.Separator(tbf, orient="vertical").pack(side="left", fill="y", padx=12, pady=2)
        # TUNE carrier button
        self.tune_button = tk.Button(tbf, text="TUNE OFF", width=11,
                                      font=("TkDefaultFont", 10, "bold"),
                                      command=self._toggle_tune, relief="raised", bd=3,
                                      bg="#cccccc", fg="black")
        self.tune_button.pack(side="left", padx=5)
        ttk.Separator(tbf, orient="vertical").pack(side="left", fill="y", padx=12, pady=2)
        self.tx_button = tk.Button(tbf, text="TX OFF", width=12,
                                    font=("TkDefaultFont", 11, "bold"),
                                    command=self._toggle_tx, relief="raised", bd=3)
        self.tx_button.pack(side="left", padx=5)
        self._update_tx_button()
        ttk.Checkbutton(tbf, text="🖱️ Scroll Tune (50 Hz/step)",
                        variable=self.scroll_tune_enabled_var).pack(side="left", padx=20)

        ttk.Label(rf, text="Uplink:").grid(row=1, column=0, sticky="w")
        fsf = ttk.Frame(rf)
        fsf.grid(row=1, column=1, sticky="ew", padx=5)
        fsf.columnconfigure(0, weight=1)
        self.freq_scale = ttk.Scale(fsf, from_=self.FREQ_MIN_HZ/1e6, to=self.FREQ_MAX_HZ/1e6,
                                     orient=tk.HORIZONTAL, variable=self.freq_mhz_var,
                                     command=self._on_freq_slider)
        self.freq_scale.grid(row=0, column=0, sticky="ew")
        fef = ttk.Frame(rf)
        fef.grid(row=1, column=2)
        self.freq_entry = ttk.Entry(fef, textvariable=self.freq_khz_var, width=14)
        self.freq_entry.pack(side="left")
        self.freq_entry.bind("<Return>", lambda e: self._send_freq_from_entry())
        ttk.Label(fef, text=" kHz").pack(side="left")

        ttk.Label(rf, text="Downlink:").grid(row=2, column=0, sticky="w")
        dlf = ttk.Frame(rf)
        dlf.grid(row=2, column=1, sticky="ew", padx=5)
        dlf.columnconfigure(0, weight=1)
        self.dl_scale = ttk.Scale(dlf,
                                   from_=(self.FREQ_MIN_HZ + QO100_LO_HZ) / 1e6,
                                   to=(self.FREQ_MAX_HZ + QO100_LO_HZ) / 1e6,
                                   orient=tk.HORIZONTAL, variable=self.dl_mhz_var,
                                   command=self._on_dl_slider)
        self.dl_scale.grid(row=0, column=0, sticky="ew")
        dlef = ttk.Frame(rf)
        dlef.grid(row=2, column=2)
        self.dl_entry = ttk.Entry(dlef, textvariable=self.dl_khz_var, width=14)
        self.dl_entry.pack(side="left")
        self.dl_entry.bind("<Return>", lambda e: self._send_dl_from_entry())
        ttk.Label(dlef, text=" kHz").pack(side="left")

        fdf = ttk.Frame(rf)
        fdf.grid(row=3, column=1, columnspan=2, sticky="w", padx=5)
        self.freq_mhz_label = ttk.Label(fdf, text="2400.4000 MHz ↑",
                                         font=("TkDefaultFont", 12, "bold"))
        self.freq_mhz_label.pack(side="left")
        ttk.Label(fdf, text="   →   ").pack(side="left")
        self.downlink_label = ttk.Label(fdf, text="10489.9000 MHz ↓",
                                         font=("TkDefaultFont", 12, "bold"), foreground="blue")
        self.downlink_label.pack(side="left")

        ttk.Label(rf, text="PPM:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        pf = ttk.Frame(rf)
        pf.grid(row=4, column=1, columnspan=2, sticky="ew", padx=5, pady=(10, 0))
        pf.columnconfigure(0, weight=1)
        ttk.Scale(pf, from_=-2.0, to=2.0, orient=tk.HORIZONTAL,
                  variable=self.ppm_var, command=self._on_ppm_slider).grid(row=0, column=0, sticky="ew")
        self.ppm_label = ttk.Label(pf, text="0.000 ppm", width=12)
        self.ppm_label.grid(row=0, column=1, padx=5)

        ttk.Label(rf, text="TX Power:").grid(row=5, column=0, sticky="w", pady=(10, 0))
        tpf = ttk.Frame(rf)
        tpf.grid(row=5, column=1, sticky="ew", padx=5, pady=(10, 0))
        tpf.columnconfigure(0, weight=1)
        LabeledScale(tpf, "", self.txpwr_var, -18, 13, 1,
                     lambda v: self._send_cmd_safe(f"txpwr {int(v)}"),
                     lambda v: f"{int(v)} dBm").pack(fill="x")

        ef = ttk.LabelFrame(tab, text="DSP Modules", padding=10)
        ef.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        ttk.Checkbutton(ef, text="Bandpass Filter", variable=self.en_bp_var,
                        command=lambda: self._send_enable("bp",   self.en_bp_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(ef, text="Equalizer",       variable=self.en_eq_var,
                        command=lambda: self._send_enable("eq",   self.en_eq_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(ef, text="Compressor",      variable=self.en_comp_var,
                        command=lambda: self._send_enable("comp", self.en_comp_var.get())).pack(side="left", padx=20)

        bpf = ttk.LabelFrame(tab, text="Bandpass Filter", padding=10)
        bpf.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        bpf.columnconfigure(0, weight=1)
        LabeledScale(bpf, "Low cutoff (Hz)",    self.bp_lo_var,     50, 1500, 10,
                     lambda v: self.debounced_send.call(f"set bp_lo {v:.0f}"), "{:.0f}").pack(fill="x")
        LabeledScale(bpf, "High cutoff (Hz)",   self.bp_hi_var,    500, 3600, 10,
                     lambda v: self.debounced_send.call(f"set bp_hi {v:.0f}"), "{:.0f}").pack(fill="x")
        LabeledScale(bpf, "Steepness (stages)", self.bp_stages_var,  1,   10,  1,
                     lambda v: self.debounced_send.call(f"set bp_stages {int(v)}"),
                     lambda v: f"{int(v)} ({int(v)*12} dB/oct)").pack(fill="x")

        eqf = ttk.LabelFrame(tab, text="Equalizer (Shelving)", padding=10)
        eqf.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        eqf.columnconfigure(0, weight=1)
        LabeledScale(eqf, "Low shelf freq (Hz)",  self.eq_low_hz_var,   50, 1000, 10,
                     lambda v: self.debounced_send.call(f"set eq_low_hz {v:.0f}"),  "{:.0f}").pack(fill="x")
        LabeledScale(eqf, "Low shelf gain (dB)",  self.eq_low_db_var,  -24,   24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_low_db {v:.1f}"),  "{:.1f}").pack(fill="x")
        LabeledScale(eqf, "High shelf freq (Hz)", self.eq_high_hz_var, 500, 3500,  10,
                     lambda v: self.debounced_send.call(f"set eq_high_hz {v:.0f}"), "{:.0f}").pack(fill="x")
        LabeledScale(eqf, "High shelf gain (dB)", self.eq_high_db_var, -24,   24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_high_db {v:.1f}"), "{:.1f}").pack(fill="x")

        cf = ttk.LabelFrame(tab, text="Compressor", padding=10)
        cf.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        cf.columnconfigure(0, weight=1)
        LabeledScale(cf, "Threshold (dB)",   self.comp_thr_var,   -60,    0, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_thr {v:.1f}"),    "{:.1f}").pack(fill="x")
        LabeledScale(cf, "Ratio",            self.comp_ratio_var,   1,   20, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_ratio {v:.1f}"),  "{:.1f}:1").pack(fill="x")
        LabeledScale(cf, "Attack (ms)",      self.comp_att_var,   0.1,  200, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_att {v:.1f}"),    "{:.1f}").pack(fill="x")
        LabeledScale(cf, "Release (ms)",     self.comp_rel_var,    10, 2000,   1,
                     lambda v: self.debounced_send.call(f"set comp_rel {v:.0f}"),    "{:.0f}").pack(fill="x")
        LabeledScale(cf, "Makeup gain (dB)", self.comp_makeup_var,  0,   40, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_makeup {v:.1f}"), "{:.1f}").pack(fill="x")
        LabeledScale(cf, "Knee (dB)",        self.comp_knee_var,    0,   24, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_knee {v:.1f}"),   "{:.1f}").pack(fill="x")
        LabeledScale(cf, "Output limit",     self.comp_outlim_var, 0.01, 0.999, 0.001,
                     lambda v: self.debounced_send.call(f"set comp_outlim {v:.3f}"), "{:.3f}").pack(fill="x")

        pwf = ttk.LabelFrame(tab, text="Power Shaping", padding=10)
        pwf.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        pwf.columnconfigure(0, weight=1)
        LabeledScale(pwf, "Amp gain", self.amp_gain_var, 0.01, 5.0, 0.01,
                     lambda v: self.debounced_send.call(f"set amp_gain {v:.3f}"),
                     "{:.3f}").pack(fill="x")
        amf = ttk.Frame(pwf)
        amf.pack(fill="x", pady=(5, 0))
        ttk.Label(amf, text="Amp min A:", width=16).pack(side="left")
        ttk.Entry(amf, textvariable=self.amp_min_a_var, width=16).pack(side="left", padx=5)
        ttk.Button(amf, text="Set",
                   command=lambda: self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
                   ).pack(side="left")

    def _build_tx_tab(self):
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="TX Control")
        tab.columnconfigure(0, weight=1)

        cwf = ttk.LabelFrame(tab, text="CW Test Mode", padding=20)
        cwf.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Label(cwf, text="Transmit continuous carrier for testing:").pack(anchor="w")
        bf = ttk.Frame(cwf)
        bf.pack(pady=10)
        ttk.Button(bf, text="▶ Start CW", command=self._start_cw, width=15).pack(side="left", padx=10)
        ttk.Button(bf, text="⏹ Stop",     command=self._stop_cw,  width=15).pack(side="left", padx=10)

        qf = ttk.LabelFrame(tab, text="Quick Commands", padding=20)
        qf.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        qb = ttk.Frame(qf)
        qb.pack()
        ttk.Button(qb, text="GET Config", command=lambda: self._send_cmd_safe("get"),  width=15).pack(side="left", padx=5)
        ttk.Button(qb, text="DIAG",       command=lambda: self._send_cmd_safe("diag"), width=15).pack(side="left", padx=5)
        ttk.Button(qb, text="HELP",       command=lambda: self._send_cmd_safe("help"), width=15).pack(side="left", padx=5)

        mf = ttk.LabelFrame(tab, text="Manual Command", padding=10)
        mf.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        mf.columnconfigure(0, weight=1)
        self.manual_cmd_var = tk.StringVar()
        ce = ttk.Entry(mf, textvariable=self.manual_cmd_var)
        ce.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        ce.bind("<Return>", lambda e: self._send_manual_cmd())
        ttk.Button(mf, text="Send", command=self._send_manual_cmd).grid(row=0, column=1)

        inf = ttk.LabelFrame(tab, text="Device Info", padding=10)
        inf.grid(row=3, column=0, sticky="nsew", pady=(0, 10))
        tab.rowconfigure(3, weight=1)
        self.info_text = tk.Text(inf, height=10, wrap="word", state="disabled",
                                  bg="#f5f5f5", font=("Consolas", 10))
        self.info_text.pack(fill="both", expand=True)

    def _build_cw_tab(self):
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="⚡ CW Keyer")
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)
        tab.rowconfigure(3, weight=1)

        pf = ttk.LabelFrame(tab, text="FTDI Adapter", padding=10)
        pf.grid(row=0, column=0, sticky="ew", padx=(0, 5), pady=(0, 8))
        pf.columnconfigure(1, weight=1)
        ttk.Label(pf, text="Port:").grid(row=0, column=0, sticky="w")
        all_ports = [p.device for p in serial.tools.list_ports.comports()] if HAS_SERIAL else []
        self.cw_port_combo = ttk.Combobox(pf, textvariable=self.cw_port_var,
                                           values=all_ports, width=18)
        if all_ports:
            self.cw_port_var.set(all_ports[0])
        self.cw_port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(pf, text="↻", width=3, command=self._cw_refresh_ports).grid(row=0, column=2)
        ttk.Label(pf, text="Baud rate:").grid(row=1, column=0, sticky="w", pady=3)
        self.cw_baud_combo = ttk.Combobox(pf, textvariable=self.cw_baud_var, width=18,
                                           values=['1200','2400','4800','9600','19200','38400','115200'])
        self.cw_baud_combo.grid(row=1, column=1, padx=4)

        mf = ttk.LabelFrame(tab, text="Mode & Pins", padding=10)
        mf.grid(row=1, column=0, sticky="ew", padx=(0, 5), pady=(0, 8))
        ttk.Label(mf, text="Mode:").grid(row=0, column=0, sticky="w")
        self.cw_mode_combo = ttk.Combobox(mf, textvariable=self.cw_mode_var, width=14,
                                           values=['Straight', 'Iambic A', 'Iambic B'])
        self.cw_mode_combo.grid(row=0, column=1, columnspan=2, padx=4, pady=2)
        self.cw_mode_combo.bind('<<ComboboxSelected>>', self._cw_mode_changed)
        ttk.Label(mf, text="Dit / Key:").grid(row=1, column=0, sticky="w")
        self.cw_dit_combo = ttk.Combobox(mf, textvariable=self.cw_dit_var,
                                          width=7, values=READABLE_PINS)
        self.cw_dit_combo.grid(row=1, column=1, padx=4, pady=2)
        self._cw_dah_lbl = ttk.Label(mf, text="Dah:")
        self._cw_dah_lbl.grid(row=2, column=0, sticky="w")
        self._cw_dah_cb = ttk.Combobox(mf, textvariable=self.cw_dah_var,
                                        width=7, values=READABLE_PINS)
        self._cw_dah_cb.grid(row=2, column=1, padx=4, pady=2)
        self.cw_active_low_cb = ttk.Checkbutton(mf, text="Active Low (key→GND)",
                                                  variable=self.cw_active_low_var)
        self.cw_active_low_cb.grid(row=3, column=0, columnspan=3, sticky="w", pady=2)

        self.cw_conn_btn = ttk.Button(tab, text="▶  CONNECT", command=self._cw_toggle)
        self.cw_conn_btn.grid(row=2, column=0, sticky="ew", padx=(0, 5), pady=(0, 8))

        cpf = ttk.LabelFrame(tab, text="CW Parameters", padding=10)
        cpf.grid(row=0, column=1, rowspan=3, sticky="nsew", pady=(0, 8))
        cpf.columnconfigure(0, weight=1)

        def cw_slider(label, var, from_, to, fmt, cb):
            r = ttk.Frame(cpf)
            r.pack(fill='x', pady=2)
            ttk.Label(r, text=label, width=16, anchor='w').pack(side='left')
            lbl = ttk.Label(r, text=fmt(var.get()), width=10, anchor='e')
            lbl.pack(side='right')
            def _cb(v):
                lbl.config(text=fmt(float(v)))
                cb(float(v))
            ttk.Scale(cpf, from_=from_, to=to, orient='horizontal',
                      variable=var, command=_cb).pack(fill='x')

        cw_slider("Speed (WPM)",   self.cw_wpm_var,    5,   60,
                  lambda v: f"{int(v)} WPM", lambda v: self.keyer.set_wpm(v))
        cw_slider("Weight (Dah)",  self.cw_weight_var, 2.0, 5.0,
                  lambda v: f"{v:.2f}×", lambda v: self.keyer.set_weight(v))
        cw_slider("Sidetone (Hz)", self.cw_tone_var,  400, 1000,
                  lambda v: f"{int(v)} Hz",  lambda v: self.audio.set_freq(v))
        cw_slider("Volume",        self.cw_vol_var,    0,  100,
                  lambda v: f"{int(v)} %",   lambda v: self.audio.set_vol(v / 100))

        if not HAS_AUDIO:
            ttk.Label(cpf, text="pyaudio missing\npip install pyaudio numpy",
                      foreground="red").pack(pady=5)

        # === Send Text as CW ===
        tf = ttk.LabelFrame(tab, text="Send Text as CW", padding=8)
        tf.grid(row=3, column=1, sticky="nsew", pady=(0, 8))
        tf.columnconfigure(0, weight=1)
        self.cw_text_entry = ttk.Entry(tf, textvariable=self.cw_text_var,
                                       font=("TkDefaultFont", 13))
        self.cw_text_entry.grid(row=0, column=0, sticky="ew", ipady=6)
        self.cw_text_entry.bind("<Return>", lambda _: self._start_cw_text())
        tbf2 = ttk.Frame(tf)
        tbf2.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self.cw_text_btn = ttk.Button(tbf2, text="▶ Send", command=self._start_cw_text, width=10)
        self.cw_text_btn.pack(side="left", padx=(0, 4))
        ttk.Button(tbf2, text="⛔ Stop", command=self._abort_cw_text, width=8).pack(side="left")

        sf = ttk.LabelFrame(tab, text="Status", padding=8)
        sf.grid(row=3, column=0, sticky="nsew", padx=(0, 5), pady=(0, 8))
        self.cw_conn_lbl = ttk.Label(sf, textvariable=self.cw_conn_status_var,
                                      font=("TkDefaultFont", 11, "bold"), foreground="red")
        self.cw_conn_lbl.pack()
        self.cw_key_lbl = ttk.Label(sf, text="KEY: OPEN",
                                     font=("Consolas", 10), foreground="gray")
        self.cw_key_lbl.pack(pady=2)
        self.cw_sym_lbl = ttk.Label(sf, text="",
                                     font=("Consolas", 20, "bold"), foreground="#cc8800")
        self.cw_sym_lbl.pack(pady=4)
        self.cw_tx_btn = ttk.Button(sf, text="⬛  TX OFF",
                                    command=self._cw_toggle_tx)
        self.cw_tx_btn.pack(fill='x', pady=(4, 0))

        ttk.Label(tab, text="ESC = Abort  |  Pin → Key → GND  |  Active Low",
                  foreground="gray").grid(row=4, column=0, columnspan=2, pady=2)

    def _build_gpsdo_tab(self):
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="GPSDO")
        tab.columnconfigure(0, weight=1)

        # Two big status indicators
        sf = ttk.LabelFrame(tab, text="Status", padding=10)
        sf.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        sf.columnconfigure(0, weight=1)
        sf.columnconfigure(1, weight=1)

        ttk.Label(sf, text="GPS Signal:", anchor="w").grid(
            row=0, column=0, sticky="w", padx=(0, 4))
        self.gpsdo_sig_label = ttk.Label(
            sf, textvariable=self.gpsdo_sig_var,
            font=("TkDefaultFont", 11, "bold"), anchor="w")
        self.gpsdo_sig_label.grid(row=0, column=1, sticky="w", pady=2)

        ttk.Label(sf, text="GPS Position:", anchor="w").grid(
            row=1, column=0, sticky="w", padx=(0, 4))
        self.gpsdo_fix_label = ttk.Label(
            sf, textvariable=self.gpsdo_fix_var,
            font=("TkDefaultFont", 11, "bold"), anchor="w")
        self.gpsdo_fix_label.grid(row=1, column=1, sticky="w", pady=2)

        # GPS detail fields
        df = ttk.LabelFrame(tab, text="GPS", padding=10)
        df.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        df.columnconfigure(1, weight=1)

        ttk.Label(df, text="UTC time:").grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_utc_var, anchor="w",
                  font=("Consolas", 11)).grid(row=0, column=1, sticky="w")

        ttk.Label(df, text="Locator:").grid(row=1, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_loc_var, anchor="w",
                  font=("Consolas", 11)).grid(row=1, column=1, sticky="w")

        ttk.Label(df, text="Satellites used:").grid(row=2, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_sats_var, anchor="w").grid(row=2, column=1, sticky="w")

        ttk.Label(df, text="Satellites visible:").grid(row=3, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_vis_var, anchor="w").grid(row=3, column=1, sticky="w")

        ttk.Label(df, text="Altitude:").grid(row=4, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_alt_var, anchor="w").grid(row=4, column=1, sticky="w")

        ttk.Label(df, text="CLK1 (52 MHz):").grid(row=5, column=0, sticky="w", padx=(0, 8))
        ttk.Label(df, textvariable=self.gpsdo_clk1_var, anchor="w").grid(row=5, column=1, sticky="w")

        # Manual refresh button
        bf = ttk.Frame(tab)
        bf.grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Button(bf, text="Refresh now",
                   command=lambda: self._send_cmd_safe("gpsdo")).pack(side="left")
        ttk.Label(bf, text="  (requests immediate status update)",
                  foreground="gray").pack(side="left")

    def _build_console_tab(self):
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Console")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        lf = ttk.Frame(tab)
        lf.grid(row=0, column=0, sticky="nsew")
        lf.columnconfigure(0, weight=1)
        lf.rowconfigure(0, weight=1)
        self.log_text = tk.Text(lf, wrap="word", font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(lf, orient="vertical", command=self.log_text.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=sb.set)
        self.log_text.tag_configure("sent",  foreground="#0066cc")
        self.log_text.tag_configure("recv",  foreground="#006600")
        self.log_text.tag_configure("error", foreground="#cc0000")
        self.log_text.tag_configure("info",  foreground="#666666")
        bf = ttk.Frame(tab)
        bf.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        ttk.Button(bf, text="Clear Log",         command=self._clear_log).pack(side="left")
        ttk.Button(bf, text="Send All Settings", command=self._send_all).pack(side="right")

    # CW KEYER
    def _cw_refresh_ports(self):
        if not HAS_SERIAL: return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cw_port_combo['values'] = ports
        if ports: self.cw_port_var.set(ports[0])

    def _cw_mode_changed(self, _=None):
        is_paddle = self.cw_mode_var.get() != 'Straight'
        self._cw_dah_cb.config(state='normal' if is_paddle else 'disabled')

    def _cw_set_widgets_state(self, state):
        for w in [self.cw_port_combo, self.cw_baud_combo, self.cw_mode_combo,
                  self.cw_dit_combo,  self._cw_dah_cb,    self.cw_active_low_cb]:
            try: w.config(state=state)
            except: pass
        if state == 'normal':
            self._cw_mode_changed()

    def _cw_toggle(self):
        if self._cw_running: self._cw_stop()
        else:                self._cw_start()

    def _cw_start(self):
        if not HAS_SERIAL:
            messagebox.showerror("Fehler", "pip install pyserial")
            return
        port = self.cw_port_var.get()
        if not port:
            messagebox.showerror("Fehler", "Keinen FTDI-Port ausgewaehlt!")
            return
        mode_map = {'Straight': Keyer.STRAIGHT,
                    'Iambic A': Keyer.IAMBIC_A,
                    'Iambic B': Keyer.IAMBIC_B}
        self.keyer.set_mode(mode_map[self.cw_mode_var.get()])
        self.keyer.set_wpm(self.cw_wpm_var.get())
        self.audio.set_freq(self.cw_tone_var.get())
        self.audio.set_vol(self.cw_vol_var.get() / 100)
        is_paddle = self.cw_mode_var.get() != 'Straight'
        dah_pin = self.cw_dah_var.get() if is_paddle else None
        self.key_reader = KeyReader(
            port=port, baud=int(self.cw_baud_var.get()),
            dit_pin=self.cw_dit_var.get(), dah_pin=dah_pin,
            active_low=self.cw_active_low_var.get())
        ok, err = self.key_reader.connect()
        if not ok:
            messagebox.showerror("FTDI Fehler", f"Verbindung fehlgeschlagen:\n{err}")
            self.key_reader = None
            return
        self._cw_running = True
        self._cw_thread  = threading.Thread(target=self._cw_loop, daemon=True)
        self._cw_thread.start()
        self.cw_conn_btn.config(text="DISCONNECT")
        self.cw_conn_status_var.set("● CONNECTED")
        self.cw_conn_lbl.config(foreground="green")
        self._cw_set_widgets_state('disabled')

    def _cw_stop(self):
        self._cw_running = False
        self.keyer.reset()
        if self.key_reader:
            self.key_reader.disconnect()
            self.key_reader = None
        # FIX B: reset shared key state so GUI shows "OFFEN" immediately
        self._cw_key_state = (False, False)
        self.cw_conn_btn.config(text="▶  CONNECT")
        self.cw_conn_status_var.set("● DISCONNECTED")
        self.cw_conn_lbl.config(foreground="red")
        self.cw_key_lbl.config(text="KEY: OPEN", foreground="gray")
        self._cw_set_widgets_state('normal')

    def _cw_loop(self):
        # FIX B: this is the ONLY place that calls key_reader.read() – eliminates
        # the race condition with _cw_gui_update() that caused GUI freezes.
        while self._cw_running:
            try:
                dit, dah = self.key_reader.read()
                self._cw_key_state = (dit, dah)
            except Exception:
                self.master.after(0, self._cw_stop)
                break
            self.keyer.tick(dit, dah)
            time.sleep(0.002)

    def _cw_gui_update(self):
        if self._cw_running:
            # FIX B: read from shared variable instead of calling key_reader.read()
            dit, dah = self._cw_key_state
            down = dit or dah
            self.cw_key_lbl.config(
                text="KEY: PRESSED" if down else "KEY: OPEN",
                foreground="green" if down else "gray")
        sym = self.keyer.get_sym_buf()
        self.cw_sym_lbl.config(text=sym)
        self.master.after(50, self._cw_gui_update)

    def _cw_on_esc(self, _=None):
        self._abort_cw_text()
        if self._cw_running:
            self._cw_stop()

    def _start_cw_text(self):
        text = self.cw_text_var.get().strip().upper()
        if not text:
            return
        unsupported = sorted({c for c in text if c != ' ' and c not in MORSE_CODE})
        if unsupported:
            messagebox.showerror("CW Text", f"Unsupported characters: {' '.join(unsupported)}")
            return
        if self._cw_text_thread and self._cw_text_thread.is_alive():
            messagebox.showinfo("CW Text", "CW text is already being sent.")
            return
        if not self.worker.is_connected():
            messagebox.showwarning("Not connected", "Please connect to the SX1280 first!")
            return
        self._cw_stop_evt.clear()
        wpm = max(5, min(60, int(self.cw_wpm_var.get())))
        self._cw_text_thread = threading.Thread(
            target=self._cw_text_worker, args=(text, wpm), daemon=True)
        self._cw_text_thread.start()

    def _abort_cw_text(self):
        self._cw_stop_evt.set()
        try: self._send_cmd_safe("stop")
        except: pass

    def _cw_text_worker(self, text, wpm):
        unit_s = 1.2 / float(wpm)
        weight = self.cw_weight_var.get()

        def sleep_ok(dur):
            end = time.monotonic() + dur
            while time.monotonic() < end:
                if self._cw_stop_evt.is_set():
                    return False
                time.sleep(min(0.02, end - time.monotonic()))
            return True

        self.master.after(0, lambda: self.cw_text_btn.config(state='disabled'))
        try:
            for ch in text:
                if self._cw_stop_evt.is_set():
                    break
                if ch == ' ':
                    self.master.after(0, lambda: self.cw_sym_lbl.config(text='_'))
                    if not sleep_ok(unit_s * 7): break
                    continue
                code = MORSE_CODE[ch]
                for j, sym in enumerate(code):
                    if self._cw_stop_evt.is_set(): break
                    self.master.after(0, lambda s=sym: self.cw_sym_lbl.config(text=s))
                    try: self.worker.send_line("cw")
                    except: break
                    dur = unit_s if sym == '.' else unit_s * weight
                    if not sleep_ok(dur): break
                    try: self.worker.send_line("stop")
                    except: break
                    if j < len(code) - 1 and not sleep_ok(unit_s): break
                if self._cw_stop_evt.is_set(): break
                # inter-character gap (3 units total, 1 already done)
                if not sleep_ok(unit_s * 2): break
        finally:
            try: self.worker.send_line("stop")
            except: pass
            self.master.after(0, lambda: self.cw_sym_lbl.config(text=''))
            self.master.after(0, lambda: self.cw_text_btn.config(state='normal'))

    def _handle_status_push(self, line):
        """Parse firmware !S status push and sync GUI widgets.

        Format: '!S mode=0 tune=0 tx=1 pwr=13 ppm=0.0000 freq=2400400000.0'
        Sent by firmware on state change or heartbeat.
        """
        try:
            parts = line.strip().split()
            kv = {}
            for p in parts[1:]:
                if "=" in p:
                    k, v = p.split("=", 1)
                    kv[k] = v

            self._status_updating = True

            if "mode" in kv:
                new_mode = "cw" if kv["mode"] == "1" else "usb"
                if self.mode_var.get() != new_mode:
                    self.mode_var.set(new_mode)

            if "tune" in kv:
                new_tune = kv["tune"] == "1"
                if self.tune_var.get() != new_tune:
                    self.tune_var.set(new_tune)
                    self._update_tune_button()

            if "tx" in kv:
                new_tx = kv["tx"] == "1"
                if self.tx_enabled_var.get() != new_tx:
                    self.tx_enabled_var.set(new_tx)
                    self._update_tx_button()

            if "pwr" in kv:
                new_pwr = int(kv["pwr"])
                if self.txpwr_var.get() != new_pwr:
                    self.txpwr_var.set(new_pwr)

            if "ppm" in kv:
                new_ppm = float(kv["ppm"])
                if abs(self.ppm_var.get() - new_ppm) > 0.0001:
                    self.ppm_var.set(new_ppm)
                    self.ppm_label.config(text=f"{new_ppm:.3f} ppm")

            if "freq" in kv:
                new_freq = float(kv["freq"])
                if abs(self.config.freq_hz - new_freq) > 0.5:
                    self.config.freq_hz = new_freq
                    self.freq_mhz_var.set(new_freq / 1_000_000)
                    self.freq_khz_var.set(f"{new_freq / 1000:.1f}")
                    self._update_freq_display()

            self._status_updating = False
        except Exception as e:
            self._status_updating = False
            self._log(f"[STATUS PARSE ERROR] {e}: {line!r}", "error")

    def _on_mode_change(self):
        if self._status_updating:
            return
        self._send_cmd_safe(f"mode {self.mode_var.get()}")

    def _toggle_tune(self):
        new_state = not self.tune_var.get()
        self.tune_var.set(new_state)
        self._update_tune_button()
        self._send_cmd_safe(f"tune {'1' if new_state else '0'}")

    def _update_tune_button(self):
        if self.tune_var.get():
            self.tune_button.config(text="TUNE ON",  bg="#ff8800", fg="white",
                                     activebackground="#ffaa00", activeforeground="white")
        else:
            self.tune_button.config(text="TUNE OFF", bg="#cccccc", fg="black",
                                     activebackground="#dddddd", activeforeground="black")

    # ORIGINAL
    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_combo["values"] = labels
        if labels: self.port_var.set(labels[0])
        self._log("Ports refreshed", "info")

    def _connect(self):
        if not HAS_SERIAL:
            messagebox.showerror("Missing dependency", "pip install pyserial")
            return
        label = self.port_var.get()
        port  = self.port_map.get(label)
        if not port or "no ports" in label.lower():
            messagebox.showerror("No port", "No serial port selected.")
            return
        try:
            self.worker.connect(port)
            self.status_var.set(f"🟢 Connected: {port}")
            self._log(f"Connected to {port}", "info")
            self.master.after(500, lambda: self._send_cmd_safe("get"))
            self.master.after(800, lambda: self._send_cmd_safe("status"))
            self._start_heartbeat()
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set("🔴 Connection failed")

    def _disconnect(self):
        if self._heartbeat_id is not None:
            self.master.after_cancel(self._heartbeat_id)
            self._heartbeat_id = None
        self.worker.disconnect()
        self.status_var.set("⚫ Disconnected")
        self._log("Disconnected", "info")

    def _start_heartbeat(self):
        """Request !S status every 2 s so GUI stays in sync with hardware encoder."""
        if not self.worker.is_connected():
            self._heartbeat_id = None
            return
        try:
            self.worker.send_line("status")
        except Exception:
            pass
        self._heartbeat_id = self.master.after(2000, self._start_heartbeat)

    def _send_cmd_safe(self, cmd):
        try:
            if not self.worker.is_connected():
                self._log(f"[NOT CONNECTED] {cmd}", "error")
                return
            self.worker.send_line(cmd)
            self._log(f"> {cmd}", "sent")
        except Exception as e:
            self._log(f"[SEND ERROR] {e}", "error")

    def _send_enable(self, which, enabled):
        self._send_cmd_safe(f"enable {which} {'1' if enabled else '0'}")

    def _toggle_tx(self):
        new_state = not self.tx_enabled_var.get()
        self.tx_enabled_var.set(new_state)
        self._update_tx_button()
        self._send_cmd_safe(f"tx {'1' if new_state else '0'}")

    def _update_tx_button(self):
        if self.tx_enabled_var.get():
            self.tx_button.config(text="TX ON",  bg="#00cc00", fg="white",
                                   activebackground="#00ff00", activeforeground="white")
        else:
            self.tx_button.config(text="TX OFF", bg="#cccccc", fg="black",
                                   activebackground="#dddddd", activeforeground="black")

    def _on_ppm_slider(self, _val):
        if self._status_updating:
            return
        ppm = self.ppm_var.get()
        self.ppm_label.config(text=f"{ppm:.3f} ppm")
        self._update_freq_display()
        self._send_cmd_safe(f"ppm {ppm:.4f}")

    def _update_freq_display(self):
        try:    hz = float(self.freq_khz_var.get()) * 1000
        except: hz = self.config.freq_hz
        dl_hz = hz + QO100_LO_HZ
        self.freq_mhz_label.config(text=f"{hz/1_000_000:.4f} MHz ↑")
        self.downlink_label.config(text=f"{dl_hz/1_000_000:.4f} MHz ↓")
        self.dl_mhz_var.set(dl_hz / 1_000_000)
        self.dl_khz_var.set(f"{dl_hz / 1000:.1f}")

    def _on_global_scroll(self, event):
        if not self.scroll_tune_enabled_var.get(): return
        try:
            if self.notebook.index(self.notebook.select()) != 0: return
        except: return
        return self._on_freq_scroll(event)

    def _on_freq_scroll(self, event):
        if not self.scroll_tune_enabled_var.get(): return
        if   event.num == 4: delta = 50
        elif event.num == 5: delta = -50
        elif hasattr(event, 'delta'): delta = 50 if event.delta > 0 else -50
        else: return
        try:    hz = float(self.freq_khz_var.get()) * 1000
        except: hz = self.config.freq_hz
        new_hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ, hz + delta))
        self.freq_khz_var.set(f"{new_hz / 1000:.1f}")
        self.freq_mhz_var.set(new_hz / 1_000_000)
        self._update_freq_display()
        self._send_cmd_safe(f"freq {new_hz:.1f}")
        return "break"

    def _on_freq_slider(self, _val):
        if self._status_updating:
            return
        hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ,
                 int(round(self.freq_mhz_var.get() * 1_000_000 / 100)) * 100))
        self.freq_khz_var.set(f"{hz / 1000:.1f}")
        self._update_freq_display()
        self._send_cmd_safe(f"freq {hz}")

    def _on_dl_slider(self, _val):
        if self._status_updating:
            return
        dl_hz = max(self.FREQ_MIN_HZ + QO100_LO_HZ,
                    min(self.FREQ_MAX_HZ + QO100_LO_HZ,
                        int(round(self.dl_mhz_var.get() * 1_000_000 / 100)) * 100))
        ul_hz = dl_hz - QO100_LO_HZ
        self.dl_mhz_var.set(dl_hz / 1_000_000)
        self.dl_khz_var.set(f"{dl_hz / 1000:.1f}")
        self.freq_mhz_var.set(ul_hz / 1_000_000)
        self.freq_khz_var.set(f"{ul_hz / 1000:.1f}")
        self.freq_mhz_label.config(text=f"{ul_hz/1_000_000:.4f} MHz ↑")
        self.downlink_label.config(text=f"{dl_hz/1_000_000:.4f} MHz ↓")
        self._send_cmd_safe(f"freq {ul_hz}")

    def _send_freq(self, hz):
        self._send_cmd_safe(f"freq {hz:.1f}")

    def _send_freq_from_entry(self):
        try:
            khz = float(self.freq_khz_var.get().replace(",", "."))
            hz = round(khz * 1000 / 100) * 100  # snap to 0.1 kHz
            hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ, hz))
            self.freq_khz_var.set(f"{hz / 1000:.1f}")
            self.freq_mhz_var.set(hz / 1_000_000)
            self._update_freq_display()
            self._send_cmd_safe(f"freq {hz:.1f}")
        except ValueError:
            messagebox.showerror("Invalid frequency", "Frequency must be a number in kHz")

    def _send_dl_from_entry(self):
        try:
            khz = float(self.dl_khz_var.get().replace(",", "."))
            dl_hz = round(khz * 1000 / 100) * 100
            dl_hz = max(self.FREQ_MIN_HZ + QO100_LO_HZ,
                        min(self.FREQ_MAX_HZ + QO100_LO_HZ, dl_hz))
            ul_hz = dl_hz - QO100_LO_HZ
            self.dl_khz_var.set(f"{dl_hz / 1000:.1f}")
            self.dl_mhz_var.set(dl_hz / 1_000_000)
            self.freq_khz_var.set(f"{ul_hz / 1000:.1f}")
            self.freq_mhz_var.set(ul_hz / 1_000_000)
            self.freq_mhz_label.config(text=f"{ul_hz/1_000_000:.4f} MHz ↑")
            self.downlink_label.config(text=f"{dl_hz/1_000_000:.4f} MHz ↓")
            self._send_cmd_safe(f"freq {ul_hz:.1f}")
        except ValueError:
            messagebox.showerror("Invalid frequency", "Frequency must be a number in kHz")

    def _start_cw(self): self._send_cmd_safe("cw")
    def _stop_cw(self):
        self._cw_test_stop_pending = True   # suppress keyer TX-button reset in _poll_rx
        self._send_cmd_safe("stop")

    def _cw_toggle_tx(self):
        if not self.worker.is_connected():
            messagebox.showwarning("Not connected",
                                   "Please connect to the SX1280 first!")
            return
        self._cw_tx_active = not self._cw_tx_active
        if self._cw_tx_active:
            # Enter CW keyer mode (no carrier until first key press)
            try:
                self.worker.send_line("mode cw")  # switch firmware to CW mode
            except Exception: pass
            self.cw_tx_btn.config(text="🔴  TX ON")
        else:
            try:
                self.worker.send_line("stop")      # stop any active carrier
                self.worker.send_line("mode usb")  # return to SSB mode
            except Exception: pass
            self.cw_tx_btn.config(text="⬛  TX OFF")

    def _send_manual_cmd(self):
        cmd = self.manual_cmd_var.get().strip()
        if cmd:
            self._send_cmd_safe(cmd)
            self.manual_cmd_var.set("")

    def _send_all(self):
        self._send_cmd_safe(f"mode {self.mode_var.get()}")
        try:
            hz = round(float(self.freq_khz_var.get()) * 1000 / 100) * 100
            hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ, hz))
        except ValueError:
            hz = int(self.config.freq_hz)
        self._send_cmd_safe(f"freq {hz}")
        try:
            ppm = float(str(self.ppm_var.get()).replace(",", "."))
            if -100 <= ppm <= 100: self._send_cmd_safe(f"ppm {ppm}")
        except: pass
        self._send_cmd_safe(f"txpwr {int(self.txpwr_var.get())}")
        self._send_cmd_safe(f"enable bp   {'1' if self.en_bp_var.get()   else '0'}")
        self._send_cmd_safe(f"enable eq   {'1' if self.en_eq_var.get()   else '0'}")
        self._send_cmd_safe(f"enable comp {'1' if self.en_comp_var.get() else '0'}")
        self._send_cmd_safe(f"set bp_lo {self.bp_lo_var.get():.0f}")
        self._send_cmd_safe(f"set bp_hi {self.bp_hi_var.get():.0f}")
        self._send_cmd_safe(f"set bp_stages {int(self.bp_stages_var.get())}")
        self._send_cmd_safe(f"set eq_low_hz {self.eq_low_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_low_db {self.eq_low_db_var.get():.1f}")
        self._send_cmd_safe(f"set eq_high_hz {self.eq_high_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_high_db {self.eq_high_db_var.get():.1f}")
        self._send_cmd_safe(f"set comp_thr {self.comp_thr_var.get():.1f}")
        self._send_cmd_safe(f"set comp_ratio {self.comp_ratio_var.get():.1f}")
        self._send_cmd_safe(f"set comp_att {self.comp_att_var.get():.1f}")
        self._send_cmd_safe(f"set comp_rel {self.comp_rel_var.get():.0f}")
        self._send_cmd_safe(f"set comp_makeup {self.comp_makeup_var.get():.1f}")
        self._send_cmd_safe(f"set comp_knee {self.comp_knee_var.get():.1f}")
        self._send_cmd_safe(f"set comp_outlim {self.comp_outlim_var.get():.3f}")
        self._send_cmd_safe(f"set amp_gain {self.amp_gain_var.get():.3f}")
        self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
        self._log("All settings sent", "info")

    def _log(self, msg, tag="recv"):
        self.log_text.insert("end", msg + "\n", tag)
        self.log_text.see("end")
        if "CFG:" in msg or "===" in msg or "Status:" in msg:
            self.info_text.config(state="normal")
            self.info_text.insert("end", msg + "\n")
            self.info_text.see("end")
            self.info_text.config(state="disabled")

    def _clear_log(self):
        self.log_text.delete("1.0", "end")
        self.info_text.config(state="normal")
        self.info_text.delete("1.0", "end")
        self.info_text.config(state="disabled")

    def _parse_gpsdo_line(self, line):
        """Parse a GPSDO status line and update the GPSDO tab."""
        import re
        m = re.search(
            r'sig=(\S+)\s+fix=(\S+)\s+sats=(\d+)\s+vis=(\d+)\s+clk1=(\S+)'
            r'(?:\s+utc=(\S+))?'
            r'(?:\s+loc=(\S+))?'
            r'(?:\s+alt=(-?\d+)m)?',
            line)
        if not m:
            return
        sig  = m.group(1)
        fix  = m.group(2)
        sats = m.group(3)
        vis  = m.group(4)
        clk1 = m.group(5)
        utc  = m.group(6) or "--:--:--"
        loc  = m.group(7) or "------"
        alt  = (m.group(8) + " m") if m.group(8) else "--"

        if clk1 == "fail":
            self.gpsdo_sig_var.set("SI5351 not found")
            self.gpsdo_sig_label.config(foreground="#cc0000")
        elif clk1 in ("lol", "los"):
            self.gpsdo_sig_var.set(f"CLK1 error: {clk1.upper()}")
            self.gpsdo_sig_label.config(foreground="#cc0000")
        elif sig == "stable":
            self.gpsdo_sig_var.set("GPS signal stable")
            self.gpsdo_sig_label.config(foreground="#007700")
        elif sig == "stale":
            self.gpsdo_sig_var.set("GPS signal lost (stale)")
            self.gpsdo_sig_label.config(foreground="#cc4400")
        else:
            self.gpsdo_sig_var.set("Waiting for satellite...")
            self.gpsdo_sig_label.config(foreground="#cc4400")

        if fix == "locked":
            self.gpsdo_fix_var.set("Position locked")
            self.gpsdo_fix_label.config(foreground="#007700")
        else:
            self.gpsdo_fix_var.set("No position fix")
            self.gpsdo_fix_label.config(foreground="#888888")

        self.gpsdo_sats_var.set(sats)
        self.gpsdo_vis_var.set(vis)
        self.gpsdo_clk1_var.set(clk1)
        self.gpsdo_utc_var.set(utc)
        self.gpsdo_loc_var.set(loc)
        self.gpsdo_alt_var.set(alt)

    def _poll_rx(self):
        processed = 0
        latest_status = None
        try:
            while processed < 20:
                line = self.rx_queue.get_nowait()
                processed += 1
                if line.startswith("!S "):
                    latest_status = line  # keep only the newest status push
                else:
                    self._log(line, "recv")
                    # Sync CW keyer TX button when firmware reports tune/CW stopped.
                    # _cw_test_stop_pending suppresses this for test-carrier stops.
                    if "OK tune=OFF" in line or "TX stopped" in line:
                        if self._cw_test_stop_pending:
                            self._cw_test_stop_pending = False
                        elif self._cw_tx_active:
                            self._cw_tx_active = False
                            self.cw_tx_btn.config(text="⬛  TX OFF")
                    if line.startswith("GPSDO:"):
                        self._parse_gpsdo_line(line)
        except queue.Empty:
            pass
        if latest_status is not None:
            self._handle_status_push(latest_status)
        delay = 10 if processed >= 20 else 50
        self.master.after(delay, self._poll_rx)


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    for theme in ["clam", "alt", "default"]:
        if theme in style.theme_names():
            style.theme_use(theme)
            break
    style.configure("TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))
    app = SX1280ControlApp(root)

    def on_close():
        app._cw_stop()
        app.audio.close()
        app._disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
