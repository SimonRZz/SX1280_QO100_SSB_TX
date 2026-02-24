#!/usr/bin/env python3
"""
SX1280 QO-100 SSB TX Control GUI
================================
Modern, responsive GUI for controlling the SX1280 SDR transmitter.
Supports all CDC commands and real-time parameter adjustment.

Author: SP8ESA
License: CC BY-NC 4.0
"""

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import re
from dataclasses import dataclass
from typing import Optional, Callable


MORSE_CODE_MAP = {
    "A": ".-", "B": "-...", "C": "-.-.", "D": "-..", "E": ".",
    "F": "..-.", "G": "--.", "H": "....", "I": "..", "J": ".---",
    "K": "-.-", "L": ".-..", "M": "--", "N": "-.", "O": "---",
    "P": ".--.", "Q": "--.-", "R": ".-.", "S": "...", "T": "-",
    "U": "..-", "V": "...-", "W": ".--", "X": "-..-", "Y": "-.--",
    "Z": "--..", "0": "-----", "1": ".----", "2": "..---", "3": "...--",
    "4": "....-", "5": ".....", "6": "-....", "7": "--...", "8": "---..",
    "9": "----.", "/": "-..-.", "?": "..--..", ".": ".-.-.-", ",": "--..--",
    "=": "-...-", "+": ".-.-.", "-": "-....-", "(": "-.--.", ")": "-.--.-",
}

# ---- serial
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False


# ============================================================
# CONFIGURATION DATACLASS
# ============================================================

@dataclass
class TxConfig:
    """Mirrors the firmware's audio_cfg_t structure"""
    # RF
    freq_hz: float = 2_400_400_000.0  # Now supports sub-Hz precision
    ppm: float = 0.0
    tx_power_dbm: int = 13  # Max TX power on SX1280 chip (-18 to +13 dBm)
    tx_enabled: bool = True
    
    # Enables
    enable_bp: bool = True
    enable_eq: bool = True
    enable_comp: bool = True
    
    # Bandpass
    bp_lo_hz: float = 50.0
    bp_hi_hz: float = 2700.0
    bp_stages: int = 7  # 1-10, each stage = 12 dB/oct (7 = 84 dB/oct)
    
    # EQ (Shelving)
    eq_low_hz: float = 190.0
    eq_low_db: float = -2.0
    eq_high_hz: float = 1700.0
    eq_high_db: float = 13.5
    
    # Compressor
    comp_thr_db: float = -2.5
    comp_ratio: float = 6.1
    comp_attack_ms: float = 41.1
    comp_release_ms: float = 1595.0
    comp_makeup_db: float = 0.0
    comp_knee_db: float = 16.5
    comp_out_limit: float = 0.940
    
    # Power shaping
    amp_gain: float = 2.9
    amp_min_a: float = 0.000002
    vox_level_a: float = 0.0100

# ============================================================
# SERIAL BACKEND (CDC)
# ============================================================

class SerialWorker:
    """Thread-safe serial communication handler"""
    
    def __init__(self, rx_queue: queue.Queue):
        self.rx_queue = rx_queue
        self.ser: Optional[serial.Serial] = None
        self.thread: Optional[threading.Thread] = None
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()

    def is_connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self, port: str, baud: int = 115200):
        if not HAS_SERIAL:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
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


# ============================================================
# UI HELPERS
# ============================================================

def list_serial_ports():
    """Get list of available serial ports"""
    if not HAS_SERIAL:
        return []
    ports = []
    for p in serial.tools.list_ports.comports():
        # Prioritize SX1280 device
        if "SX1280" in p.description or "cafe:4073" in str(p.hwid).lower():
            ports.insert(0, (p.device, f"★ {p.device} ({p.description})"))
        else:
            ports.append((p.device, f"{p.device} ({p.description})"))
    return ports


class Debouncer:
    """Debounce rapid function calls"""
    
    def __init__(self, tk_root: tk.Tk, delay_ms: int, fn: Callable):
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
            try:
                self.root.after_cancel(self._after_id)
            except Exception:
                pass
        self._after_id = self.root.after(self.delay_ms, self._fire)

    def _fire(self):
        self._after_id = None
        if self._last_args is not None:
            self.fn(*self._last_args, **(self._last_kwargs or {}))


class LabeledScale(ttk.Frame):
    """Reusable labeled scale widget with value display"""
    
    def __init__(self, parent, label: str, var: tk.Variable, 
                 from_: float, to: float, resolution: float,
                 on_change: Callable, format_str: str = "{:.1f}"):
        super().__init__(parent)
        
        self.var = var
        self.resolution = resolution
        self.on_change = on_change
        self.format_str = format_str
        
        self.columnconfigure(1, weight=1)
        
        # Label
        ttk.Label(self, text=label, width=16, anchor="w").grid(row=0, column=0, sticky="w")
        
        # Scale
        self.scale = ttk.Scale(self, from_=from_, to=to, orient=tk.HORIZONTAL, 
                               variable=var, command=self._on_scale)
        self.scale.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        
        # Value display
        self.value_label = ttk.Label(self, width=10, anchor="e")
        self.value_label.grid(row=0, column=2, sticky="e")
        self._update_value_label()
        
        # Bind for continuous updates
        self.scale.bind("<ButtonRelease-1>", self._on_release)
        
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
        if callable(self.format_str):
            text = self.format_str(v)
        else:
            text = self.format_str.format(v)
        self.value_label.config(text=text)


# ============================================================
# MAIN APPLICATION
# ============================================================

class SX1280ControlApp(ttk.Frame):
    """Main application window"""
    
    # RF limits for QO-100 uplink
    FREQ_MIN_HZ = 2_400_000_000
    FREQ_MAX_HZ = 2_400_500_000
    FREQ_STEP_HZ = 100

    def __init__(self, master: tk.Tk):
        super().__init__(master)
        self.master = master
        
        # State
        self.config = TxConfig()
        self.rx_queue: queue.Queue = queue.Queue()
        self.worker = SerialWorker(self.rx_queue)
        self.cw_stop_evt = threading.Event()
        self.cw_thread: Optional[threading.Thread] = None
        
        # Debouncers (kept for other sliders, but freq/ppm are now immediate)
        self.debounced_send = Debouncer(master, 150, self._send_cmd_safe)
        self.freq_debouncer = Debouncer(master, 200, self._send_freq)
        
        # Build UI
        self._create_variables()
        self._build_ui()
        
        # Initialize displays
        self._update_freq_display()
        
        # Global scroll binding for frequency tuning (works anywhere in window)
        master.bind_all("<Button-4>", self._on_global_scroll)  # Linux scroll up
        master.bind_all("<Button-5>", self._on_global_scroll)  # Linux scroll down
        master.bind_all("<MouseWheel>", self._on_global_scroll)  # Windows/macOS
        master.bind_all("<Escape>", self._on_escape_stop)
        
        self._poll_rx()
        
        # Pack main frame
        self.pack(fill="both", expand=True)

    def _create_variables(self):
        """Create all Tk variables"""
        # Connection
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="⚫ Disconnected")
        
        # RF
        self.freq_mhz_var = tk.DoubleVar(value=self.config.freq_hz / 1_000_000)
        self.freq_hz_var = tk.StringVar(value=str(self.config.freq_hz))
        self.ppm_var = tk.DoubleVar(value=0.0)
        self.txpwr_var = tk.IntVar(value=self.config.tx_power_dbm)
        self.tx_enabled_var = tk.BooleanVar(value=True)
        self.scroll_tune_enabled_var = tk.BooleanVar(value=False)  # Scroll tuning on/off
        
        # Enables
        self.en_bp_var = tk.BooleanVar(value=self.config.enable_bp)
        self.en_eq_var = tk.BooleanVar(value=self.config.enable_eq)
        self.en_comp_var = tk.BooleanVar(value=self.config.enable_comp)
        
        # Bandpass
        self.bp_lo_var = tk.DoubleVar(value=self.config.bp_lo_hz)
        self.bp_hi_var = tk.DoubleVar(value=self.config.bp_hi_hz)
        self.bp_stages_var = tk.IntVar(value=self.config.bp_stages)
        
        # EQ
        self.eq_low_hz_var = tk.DoubleVar(value=self.config.eq_low_hz)
        self.eq_low_db_var = tk.DoubleVar(value=self.config.eq_low_db)
        self.eq_high_hz_var = tk.DoubleVar(value=self.config.eq_high_hz)
        self.eq_high_db_var = tk.DoubleVar(value=self.config.eq_high_db)
        
        # Compressor
        self.comp_thr_var = tk.DoubleVar(value=self.config.comp_thr_db)
        self.comp_ratio_var = tk.DoubleVar(value=self.config.comp_ratio)
        self.comp_att_var = tk.DoubleVar(value=self.config.comp_attack_ms)
        self.comp_rel_var = tk.DoubleVar(value=self.config.comp_release_ms)
        self.comp_makeup_var = tk.DoubleVar(value=self.config.comp_makeup_db)
        self.comp_knee_var = tk.DoubleVar(value=self.config.comp_knee_db)
        self.comp_outlim_var = tk.DoubleVar(value=self.config.comp_out_limit)
        
        # Power shaping
        self.amp_gain_var = tk.DoubleVar(value=self.config.amp_gain)
        self.amp_min_a_var = tk.StringVar(value=f"{self.config.amp_min_a:.9f}")
        self.vox_level_var = tk.DoubleVar(value=self.config.vox_level_a)

        # CW text mode
        self.cw_text_var = tk.StringVar(value="CQ CQ DE SX1280")
        self.cw_wpm_var = tk.IntVar(value=18)
        self.cw_preview_char_var = tk.StringVar(value="-")
        self.cw_preview_symbol_var = tk.StringVar(value="-")
        self.cw_preview_state_var = tk.StringVar(value="Idle")

    def _build_ui(self):
        """Build the user interface"""
        self.master.title("SX1280 QO-100 SSB TX Control")
        self.master.geometry("900x800")
        self.master.minsize(600, 500)
        
        # Configure grid weights for responsive layout
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        
        # === Connection Bar ===
        self._build_connection_bar()
        
        # === Main Content (Notebook) ===
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        
        # Tab 1: RF & DSP
        self._build_dsp_tab()
        
        # Tab 2: TX Control
        self._build_tx_tab()
        
        # Tab 3: Console
        self._build_console_tab()

    def _build_connection_bar(self):
        """Build the connection toolbar"""
        conn_frame = ttk.Frame(self)
        conn_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        conn_frame.columnconfigure(1, weight=1)
        
        # Port selection
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=(0, 5))
        
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_var.set(labels[0] if labels else "")
        
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, 
                                        values=labels, state="readonly", width=45)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5)
        
        # Buttons
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=0, column=2)
        
        ttk.Button(btn_frame, text="🔄", width=3, command=self._refresh_ports).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Connect", command=self._connect).pack(side="left", padx=2)
        ttk.Button(btn_frame, text="Disconnect", command=self._disconnect).pack(side="left", padx=2)
        
        # Status
        ttk.Label(conn_frame, textvariable=self.status_var).grid(row=0, column=3, padx=(10, 0))

    def _build_dsp_tab(self):
        """Build the DSP control tab"""
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="RF & DSP")
        
        tab.columnconfigure(0, weight=1)
        
        # === RF Section ===
        rf_frame = ttk.LabelFrame(tab, text="RF / Frequency", padding=10)
        rf_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        rf_frame.columnconfigure(1, weight=1)
        
        # Bind scroll to entire RF frame
        rf_frame.bind("<MouseWheel>", self._on_freq_scroll)
        rf_frame.bind("<Button-4>", self._on_freq_scroll)
        rf_frame.bind("<Button-5>", self._on_freq_scroll)
        
        # === TX ON/OFF Button (prominent, with color) ===
        tx_btn_frame = ttk.Frame(rf_frame)
        tx_btn_frame.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0, 10))
        
        # Use regular tk.Button for color support
        self.tx_button = tk.Button(tx_btn_frame, text="TX OFF", width=12, font=("TkDefaultFont", 11, "bold"),
                                    command=self._toggle_tx, relief="raised", bd=3)
        self.tx_button.pack(side="left", padx=5)
        self._update_tx_button()
        
        # Scroll tuning toggle
        self.scroll_tune_cb = ttk.Checkbutton(tx_btn_frame, text="🖱️ Scroll Tune (50 Hz/step)", 
                                               variable=self.scroll_tune_enabled_var)
        self.scroll_tune_cb.pack(side="left", padx=20)
        
        # Frequency slider
        ttk.Label(rf_frame, text="Frequency:").grid(row=1, column=0, sticky="w")
        
        freq_slider_frame = ttk.Frame(rf_frame)
        freq_slider_frame.grid(row=1, column=1, sticky="ew", padx=5)
        freq_slider_frame.columnconfigure(0, weight=1)
        
        self.freq_scale = ttk.Scale(freq_slider_frame, 
                                     from_=self.FREQ_MIN_HZ / 1_000_000,
                                     to=self.FREQ_MAX_HZ / 1_000_000,
                                     orient=tk.HORIZONTAL,
                                     variable=self.freq_mhz_var,
                                     command=self._on_freq_slider)
        self.freq_scale.grid(row=0, column=0, sticky="ew")
        
        # Frequency entry
        freq_entry_frame = ttk.Frame(rf_frame)
        freq_entry_frame.grid(row=1, column=2)
        
        self.freq_entry = ttk.Entry(freq_entry_frame, textvariable=self.freq_hz_var, width=14)
        self.freq_entry.pack(side="left")
        self.freq_entry.bind("<Return>", lambda e: self._send_freq_from_entry())
        self.freq_entry.bind("<MouseWheel>", self._on_freq_scroll)
        self.freq_entry.bind("<Button-4>", self._on_freq_scroll)
        self.freq_entry.bind("<Button-5>", self._on_freq_scroll)
        ttk.Label(freq_entry_frame, text=" Hz").pack(side="left")
        
        # Frequency display frame (uplink + downlink)
        freq_display_frame = ttk.Frame(rf_frame)
        freq_display_frame.grid(row=2, column=1, columnspan=2, sticky="w", padx=5)
        
        # Uplink MHz display
        self.freq_mhz_label = ttk.Label(freq_display_frame, text="2400.1000 MHz ↑", 
                                         font=("TkDefaultFont", 12, "bold"))
        self.freq_mhz_label.pack(side="left")
        self.freq_mhz_label.bind("<MouseWheel>", self._on_freq_scroll)
        self.freq_mhz_label.bind("<Button-4>", self._on_freq_scroll)
        self.freq_mhz_label.bind("<Button-5>", self._on_freq_scroll)
        
        # Downlink display (QO-100: uplink 2400.xxx -> downlink 10489.xxx)
        ttk.Label(freq_display_frame, text="   →   ").pack(side="left")
        self.downlink_label = ttk.Label(freq_display_frame, text="10489.6000 MHz ↓", 
                                         font=("TkDefaultFont", 12, "bold"), foreground="blue")
        self.downlink_label.pack(side="left")
        
        # PPM slider (fine adjustment -2 to +2 ppm) - IMMEDIATE response
        ttk.Label(rf_frame, text="PPM:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ppm_frame = ttk.Frame(rf_frame)
        ppm_frame.grid(row=3, column=1, columnspan=2, sticky="ew", padx=5, pady=(10, 0))
        ppm_frame.columnconfigure(0, weight=1)
        
        self.ppm_scale = ttk.Scale(ppm_frame, from_=-2.0, to=2.0, orient=tk.HORIZONTAL,
                                    variable=self.ppm_var, command=self._on_ppm_slider)
        self.ppm_scale.grid(row=0, column=0, sticky="ew")
        
        self.ppm_label = ttk.Label(ppm_frame, text="0.000 ppm", width=12)
        self.ppm_label.grid(row=0, column=1, padx=5)
        
        # TX Power - IMMEDIATE response
        ttk.Label(rf_frame, text="TX Power:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        txpwr_frame = ttk.Frame(rf_frame)
        txpwr_frame.grid(row=4, column=1, sticky="ew", padx=5, pady=(10, 0))
        txpwr_frame.columnconfigure(0, weight=1)
        
        LabeledScale(txpwr_frame, "", self.txpwr_var, -18, 13, 1,
                     lambda v: self._send_cmd_safe(f"txpwr {int(v)}"),
                     lambda v: f"{int(v)} dBm").pack(fill="x")
        
        # === Enable Checkboxes ===
        enable_frame = ttk.LabelFrame(tab, text="DSP Modules", padding=10)
        enable_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Checkbutton(enable_frame, text="Bandpass Filter", variable=self.en_bp_var,
                        command=lambda: self._send_enable("bp", self.en_bp_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(enable_frame, text="Equalizer", variable=self.en_eq_var,
                        command=lambda: self._send_enable("eq", self.en_eq_var.get())).pack(side="left", padx=20)
        ttk.Checkbutton(enable_frame, text="Compressor", variable=self.en_comp_var,
                        command=lambda: self._send_enable("comp", self.en_comp_var.get())).pack(side="left", padx=20)
        
        # === Bandpass ===
        bp_frame = ttk.LabelFrame(tab, text="Bandpass Filter", padding=10)
        bp_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        bp_frame.columnconfigure(0, weight=1)
        
        LabeledScale(bp_frame, "Low cutoff (Hz)", self.bp_lo_var, 50, 1500, 10,
                     lambda v: self.debounced_send.call(f"set bp_lo {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(bp_frame, "High cutoff (Hz)", self.bp_hi_var, 500, 3600, 10,
                     lambda v: self.debounced_send.call(f"set bp_hi {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(bp_frame, "Steepness (stages)", self.bp_stages_var, 1, 10, 1,
                     lambda v: self.debounced_send.call(f"set bp_stages {int(v)}"),
                     lambda v: f"{int(v)} ({int(v)*12} dB/oct)").pack(fill="x")
        
        # === EQ ===
        eq_frame = ttk.LabelFrame(tab, text="Equalizer (Shelving)", padding=10)
        eq_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        eq_frame.columnconfigure(0, weight=1)
        
        LabeledScale(eq_frame, "Low shelf freq (Hz)", self.eq_low_hz_var, 50, 1000, 10,
                     lambda v: self.debounced_send.call(f"set eq_low_hz {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(eq_frame, "Low shelf gain (dB)", self.eq_low_db_var, -24, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_low_db {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(eq_frame, "High shelf freq (Hz)", self.eq_high_hz_var, 500, 3500, 10,
                     lambda v: self.debounced_send.call(f"set eq_high_hz {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(eq_frame, "High shelf gain (dB)", self.eq_high_db_var, -24, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set eq_high_db {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        
        # === Compressor ===
        comp_frame = ttk.LabelFrame(tab, text="Compressor", padding=10)
        comp_frame.grid(row=4, column=0, sticky="ew", pady=(0, 10))
        comp_frame.columnconfigure(0, weight=1)
        
        LabeledScale(comp_frame, "Threshold (dB)", self.comp_thr_var, -60, 0, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_thr {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Ratio", self.comp_ratio_var, 1, 20, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_ratio {v:.1f}"),
                     "{:.1f}:1").pack(fill="x")
        LabeledScale(comp_frame, "Attack (ms)", self.comp_att_var, 0.1, 200, 0.1,
                     lambda v: self.debounced_send.call(f"set comp_att {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Release (ms)", self.comp_rel_var, 10, 2000, 1,
                     lambda v: self.debounced_send.call(f"set comp_rel {v:.0f}"),
                     "{:.0f}").pack(fill="x")
        LabeledScale(comp_frame, "Makeup gain (dB)", self.comp_makeup_var, 0, 40, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_makeup {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Knee (dB)", self.comp_knee_var, 0, 24, 0.5,
                     lambda v: self.debounced_send.call(f"set comp_knee {v:.1f}"),
                     "{:.1f}").pack(fill="x")
        LabeledScale(comp_frame, "Output limit", self.comp_outlim_var, 0.01, 0.999, 0.001,
                     lambda v: self.debounced_send.call(f"set comp_outlim {v:.3f}"),
                     "{:.3f}").pack(fill="x")
        
        # === Power Shaping ===
        pwr_frame = ttk.LabelFrame(tab, text="Power Shaping", padding=10)
        pwr_frame.grid(row=5, column=0, sticky="ew", pady=(0, 10))
        pwr_frame.columnconfigure(0, weight=1)
        
        LabeledScale(pwr_frame, "Amp gain", self.amp_gain_var, 0.01, 5.0, 0.01,
                     lambda v: self.debounced_send.call(f"set amp_gain {v:.3f}"),
                     "{:.3f}").pack(fill="x")
        
        amp_min_frame = ttk.Frame(pwr_frame)
        amp_min_frame.pack(fill="x", pady=(5, 0))
        ttk.Label(amp_min_frame, text="Amp min A:", width=16).pack(side="left")
        ttk.Entry(amp_min_frame, textvariable=self.amp_min_a_var, width=16).pack(side="left", padx=5)
        ttk.Button(amp_min_frame, text="Set", 
                   command=lambda: self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
                  ).pack(side="left")

        LabeledScale(pwr_frame, "VOX / Gate level", self.vox_level_var, 0.0005, 0.0500, 0.0005,
                     lambda v: self.debounced_send.call(f"set vox {v:.4f}"),
                     "{:.4f}").pack(fill="x")

    def _build_tx_tab(self):
        """Build the TX control tab"""
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="TX Control")
        
        tab.columnconfigure(0, weight=1)
        
        # === CW Test ===
        cw_frame = ttk.LabelFrame(tab, text="CW Test Mode", padding=20)
        cw_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(cw_frame, text="Transmit continuous carrier for testing:").pack(anchor="w")
        
        btn_frame = ttk.Frame(cw_frame)
        btn_frame.pack(pady=10)
        
        self.cw_btn = ttk.Button(btn_frame, text="▶ Start CW", command=self._start_cw, width=15)
        self.cw_btn.pack(side="left", padx=10)
        
        self.stop_btn = ttk.Button(btn_frame, text="⏹ Stop", command=self._stop_cw, width=15)
        self.stop_btn.pack(side="left", padx=10)

        ttk.Separator(cw_frame, orient="horizontal").pack(fill="x", pady=10)

        ttk.Label(cw_frame, text="Send CW text (Morse code):").pack(anchor="w", pady=(0, 5))

        cw_text_entry = ttk.Entry(cw_frame, textvariable=self.cw_text_var)
        cw_text_entry.pack(fill="x", pady=(0, 8))
        cw_text_entry.bind("<Return>", lambda _e: self._start_cw_text())

        wpm_frame = ttk.Frame(cw_frame)
        wpm_frame.pack(fill="x", pady=(0, 8))
        ttk.Label(wpm_frame, text="WPM:").pack(side="left")
        ttk.Scale(wpm_frame, from_=5, to=40, orient=tk.HORIZONTAL,
                  variable=self.cw_wpm_var, command=self._on_cw_wpm_slider).pack(side="left", fill="x", expand=True, padx=8)
        ttk.Label(wpm_frame, textvariable=self.cw_wpm_var, width=4).pack(side="left")

        self.cw_text_btn = ttk.Button(cw_frame, text="📡 Send CW Text", command=self._start_cw_text)
        self.cw_text_btn.pack(anchor="w")

        self.cw_abort_btn = ttk.Button(cw_frame, text="⛔ Abort CW (Esc)", command=self._abort_cw_text)
        self.cw_abort_btn.pack(anchor="w", pady=(6, 0))

        preview_frame = ttk.LabelFrame(cw_frame, text="CW Live Preview", padding=8)
        preview_frame.pack(fill="x", pady=(10, 0))
        ttk.Label(preview_frame, text="Character:").grid(row=0, column=0, sticky="w")
        ttk.Label(preview_frame, textvariable=self.cw_preview_char_var, width=20).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(preview_frame, text="Symbol:").grid(row=1, column=0, sticky="w")
        ttk.Label(preview_frame, textvariable=self.cw_preview_symbol_var, width=20).grid(row=1, column=1, sticky="w", padx=(8, 0))
        ttk.Label(preview_frame, text="State:").grid(row=2, column=0, sticky="w")
        ttk.Label(preview_frame, textvariable=self.cw_preview_state_var, width=20).grid(row=2, column=1, sticky="w", padx=(8, 0))
        
        # === Quick Commands ===
        cmd_frame = ttk.LabelFrame(tab, text="Quick Commands", padding=20)
        cmd_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        
        quick_btns = ttk.Frame(cmd_frame)
        quick_btns.pack()
        
        ttk.Button(quick_btns, text="GET Config", command=lambda: self._send_cmd_safe("get"), 
                   width=15).pack(side="left", padx=5)
        ttk.Button(quick_btns, text="DIAG", command=lambda: self._send_cmd_safe("diag"),
                   width=15).pack(side="left", padx=5)
        ttk.Button(quick_btns, text="HELP", command=lambda: self._send_cmd_safe("help"),
                   width=15).pack(side="left", padx=5)
        
        # === Manual Command ===
        manual_frame = ttk.LabelFrame(tab, text="Manual Command", padding=10)
        manual_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        manual_frame.columnconfigure(0, weight=1)
        
        self.manual_cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(manual_frame, textvariable=self.manual_cmd_var)
        cmd_entry.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        cmd_entry.bind("<Return>", lambda e: self._send_manual_cmd())
        
        ttk.Button(manual_frame, text="Send", command=self._send_manual_cmd).grid(row=0, column=1)
        
        # === Status Info ===
        info_frame = ttk.LabelFrame(tab, text="Device Info", padding=10)
        info_frame.grid(row=3, column=0, sticky="nsew", pady=(0, 10))
        tab.rowconfigure(3, weight=1)
        
        self.info_text = tk.Text(info_frame, height=10, wrap="word", state="disabled",
                                  bg="#f5f5f5", font=("Consolas", 10))
        self.info_text.pack(fill="both", expand=True)

    def _build_console_tab(self):
        """Build the console/log tab"""
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Console")
        
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        
        # Log text
        log_frame = ttk.Frame(tab)
        log_frame.grid(row=0, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        self.log_text = tk.Text(log_frame, wrap="word", font=("Consolas", 9))
        self.log_text.grid(row=0, column=0, sticky="nsew")
        
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        # Tag for different message types
        self.log_text.tag_configure("sent", foreground="#0066cc")
        self.log_text.tag_configure("recv", foreground="#006600")
        self.log_text.tag_configure("error", foreground="#cc0000")
        self.log_text.tag_configure("info", foreground="#666666")
        
        # Buttons
        btn_frame = ttk.Frame(tab)
        btn_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        
        ttk.Button(btn_frame, text="Clear Log", command=self._clear_log).pack(side="left")
        ttk.Button(btn_frame, text="Send All Settings", command=self._send_all).pack(side="right")

    # === Connection Methods ===
    
    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_map = {label: dev for dev, label in ports}
        labels = list(self.port_map.keys()) or ["(no ports found)"]
        self.port_combo["values"] = labels
        if labels:
            self.port_var.set(labels[0])
        self._log("Ports refreshed", "info")

    def _connect(self):
        if not HAS_SERIAL:
            messagebox.showerror("Missing dependency", 
                                "pyserial is required.\nInstall with: pip install pyserial")
            return
        
        label = self.port_var.get()
        port = self.port_map.get(label)
        if not port or "no ports" in label.lower():
            messagebox.showerror("No port", "No serial port selected.")
            return
        
        try:
            self.worker.connect(port)
            self.status_var.set(f"🟢 Connected: {port}")
            self._log(f"Connected to {port}", "info")
            # Request current config
            self.master.after(500, lambda: self._send_cmd_safe("get"))
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            self.status_var.set("🔴 Connection failed")

    def _disconnect(self):
        self.worker.disconnect()
        self.status_var.set("⚫ Disconnected")
        self._log("Disconnected", "info")

    # === Command Methods ===
    
    def _send_cmd_safe(self, cmd: str):
        try:
            if not self.worker.is_connected():
                if threading.current_thread() is threading.main_thread():
                    self._log(f"[NOT CONNECTED] {cmd}", "error")
                else:
                    self.master.after(0, lambda: self._log(f"[NOT CONNECTED] {cmd}", "error"))
                return
            self.worker.send_line(cmd)
            if threading.current_thread() is threading.main_thread():
                self._log(f"> {cmd}", "sent")
            else:
                self.master.after(0, lambda: self._log(f"> {cmd}", "sent"))
        except Exception as e:
            if threading.current_thread() is threading.main_thread():
                self._log(f"[SEND ERROR] {e}", "error")
            else:
                self.master.after(0, lambda: self._log(f"[SEND ERROR] {e}", "error"))

    def _send_enable(self, which: str, enabled: bool):
        v = "1" if enabled else "0"
        self._send_cmd_safe(f"enable {which} {v}")

    def _toggle_tx(self):
        """Toggle TX on/off"""
        current = self.tx_enabled_var.get()
        new_state = not current
        self.tx_enabled_var.set(new_state)
        self._update_tx_button()
        self._send_cmd_safe(f"tx {'1' if new_state else '0'}")

    def _update_tx_button(self):
        """Update TX button appearance based on state"""
        if self.tx_enabled_var.get():
            self.tx_button.config(text="TX ON", bg="#00cc00", fg="white", 
                                   activebackground="#00ff00", activeforeground="white")
        else:
            self.tx_button.config(text="TX OFF", bg="#cccccc", fg="black",
                                   activebackground="#dddddd", activeforeground="black")

    def _on_ppm_slider(self, _val):
        """Handle PPM slider change - IMMEDIATE response"""
        ppm = self.ppm_var.get()
        self.ppm_label.config(text=f"{ppm:.3f} ppm")
        self._update_freq_display()
        # Send immediately without debounce
        self._send_cmd_safe(f"ppm {ppm:.4f}")

    def _update_freq_display(self):
        """Update frequency displays (uplink + downlink)"""
        try:
            hz = float(self.freq_hz_var.get())
        except ValueError:
            hz = self.config.freq_hz
        
        # Uplink display
        self.freq_mhz_label.config(text=f"{hz/1_000_000:.4f} MHz ↑")
        
        # QO-100 downlink: uplink 2400.xxx MHz -> downlink 10489.xxx MHz
        # Offset = 10489.5 - 2400.0 = 8089.5 MHz
        downlink_hz = hz + 8089_500_000
        self.downlink_label.config(text=f"{downlink_hz/1_000_000:.4f} MHz ↓")

    def _on_global_scroll(self, event):
        """Global scroll handler - tune frequency if scroll tune is enabled"""
        if not self.scroll_tune_enabled_var.get():
            return  # Let event propagate normally
        
        # Check if we're on the first tab (RF & DSP)
        try:
            current_tab = self.notebook.index(self.notebook.select())
            if current_tab != 0:
                return  # Only tune on RF tab
        except:
            return
        
        # Call the frequency scroll handler
        return self._on_freq_scroll(event)

    def _on_freq_scroll(self, event):
        """Handle mouse scroll for frequency tuning (50 Hz per step)"""
        if not self.scroll_tune_enabled_var.get():
            return  # Scroll tuning disabled
        
        # Determine scroll direction
        if event.num == 4:
            delta = 50  # Linux scroll up
        elif event.num == 5:
            delta = -50  # Linux scroll down
        elif hasattr(event, 'delta'):
            # Windows/macOS: delta is typically ±120
            delta = 50 if event.delta > 0 else -50
        else:
            return
        
        # Get current freq and adjust
        try:
            current_hz = float(self.freq_hz_var.get())
        except ValueError:
            current_hz = self.config.freq_hz
        
        new_hz = current_hz + delta
        new_hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ, new_hz))
        
        # Update all displays
        self.freq_hz_var.set(f"{new_hz:.0f}")
        self.freq_mhz_var.set(new_hz / 1_000_000)
        self._update_freq_display()
        
        # Send immediately
        self._send_cmd_safe(f"freq {new_hz:.1f}")
        
        # Prevent event propagation
        return "break"

    def _on_freq_slider(self, _val):
        """Handle frequency slider - IMMEDIATE response"""
        mhz = self.freq_mhz_var.get()
        hz = int(round(mhz * 1_000_000))
        hz = self._clamp_freq(hz)
        self.freq_hz_var.set(str(hz))
        self._update_freq_display()
        # Send immediately
        self._send_cmd_safe(f"freq {hz}")

    def _clamp_freq(self, hz) -> float:
        """Clamp frequency to valid range (now supports float)"""
        hz = max(self.FREQ_MIN_HZ, min(self.FREQ_MAX_HZ, hz))
        return hz

    def _send_freq(self, hz):
        """Send frequency command (supports sub-Hz precision)"""
        self._send_cmd_safe(f"freq {hz:.1f}")

    def _send_freq_from_entry(self):
        try:
            hz = float(self.freq_hz_var.get().replace(",", "."))
            hz = self._clamp_freq(hz)
            self.freq_hz_var.set(f"{hz:.0f}")
            self.freq_mhz_var.set(hz / 1_000_000)
            self._update_freq_display()
            self._send_cmd_safe(f"freq {hz:.1f}")
        except ValueError:
            messagebox.showerror("Invalid frequency", "Frequency must be a number in Hz")

    def _send_ppm(self):
        """Send PPM from slider (now handled by _on_ppm_slider)"""
        ppm = self.ppm_var.get()
        self._send_cmd_safe(f"ppm {ppm:.4f}")

    def _set_cw_preview(self, char: Optional[str] = None, symbol: Optional[str] = None, state: Optional[str] = None):
        def apply_update():
            if char is not None:
                self.cw_preview_char_var.set(char)
            if symbol is not None:
                self.cw_preview_symbol_var.set(symbol)
            if state is not None:
                self.cw_preview_state_var.set(state)

        if threading.current_thread() is threading.main_thread():
            apply_update()
        else:
            self.master.after(0, apply_update)

    def _start_cw(self):
        self.cw_stop_evt.set()
        self._set_cw_preview(char="Carrier", symbol="ON", state="Continuous CW")
        self._send_cmd_safe("cw")

    def _stop_cw(self):
        self._abort_cw_text()

    def _abort_cw_text(self):
        self.cw_stop_evt.set()
        self._set_cw_preview(symbol="-", state="Aborting...")
        self._send_cmd_safe("stop")
        self._set_cw_preview(state="Idle")

    def _on_escape_stop(self, _event):
        self._abort_cw_text()
        return "break"


    def _on_cw_wpm_slider(self, value):
        try:
            wpm = int(round(float(value)))
        except (TypeError, ValueError):
            return
        wpm = max(5, min(40, wpm))
        if self.cw_wpm_var.get() != wpm:
            self.cw_wpm_var.set(wpm)

    def _start_cw_text(self):
        text = self.cw_text_var.get().strip().upper()
        if not text:
            messagebox.showerror("CW Text", "Please enter text for CW transmission.")
            return

        unsupported = sorted({ch for ch in text if ch != " " and ch not in MORSE_CODE_MAP})
        if unsupported:
            messagebox.showerror(
                "CW Text",
                f"Unsupported characters: {' '.join(unsupported)}"
            )
            return

        if self.cw_thread and self.cw_thread.is_alive():
            messagebox.showinfo("CW Text", "CW text transmission is already running.")
            return

        self.cw_stop_evt.clear()
        wpm = max(5, min(40, int(self.cw_wpm_var.get())))
        self._set_cw_preview(char="-", symbol="-", state=f"Running ({wpm} WPM)")
        self.cw_thread = threading.Thread(target=self._cw_text_worker, args=(text, wpm), daemon=True)
        self.cw_thread.start()
        self._log(f"CW text started ({wpm} WPM): {text}", "info")

    def _cw_text_worker(self, text: str, wpm: int):
        unit_s = 1.2 / float(wpm)

        def sleep_interruptible(duration_s: float):
            end = time.time() + duration_s
            while time.time() < end:
                if self.cw_stop_evt.is_set():
                    return False
                time.sleep(min(0.02, end - time.time()))
            return True

        for i, ch in enumerate(text):
            if self.cw_stop_evt.is_set():
                break

            if ch == " ":
                self._set_cw_preview(char="(space)", symbol=" ", state="Word gap")
                if not sleep_interruptible(unit_s * 7):
                    break
                continue

            self._set_cw_preview(char=ch, symbol="-", state="Character")
            code = MORSE_CODE_MAP[ch]
            for j, symbol in enumerate(code):
                if self.cw_stop_evt.is_set():
                    break

                self._set_cw_preview(symbol=symbol, state="Key down")
                self._send_cmd_safe("cw")
                tone_len = unit_s if symbol == "." else unit_s * 3
                if not sleep_interruptible(tone_len):
                    break

                self._send_cmd_safe("stop")
                self._set_cw_preview(state="Key up")

                if j < len(code) - 1 and not sleep_interruptible(unit_s):
                    break

            if self.cw_stop_evt.is_set():
                break

            if i < len(text) - 1 and text[i + 1] != " ":
                if not sleep_interruptible(unit_s * 3):
                    break

        self._send_cmd_safe("stop")
        if self.cw_stop_evt.is_set():
            self._set_cw_preview(symbol="-", state="Aborted")
            self.master.after(0, lambda: self._log("CW text transmission aborted", "info"))
        else:
            self._set_cw_preview(char="-", symbol="-", state="Finished")
            self.master.after(0, lambda: self._log("CW text transmission finished", "info"))

    def _send_manual_cmd(self):
        cmd = self.manual_cmd_var.get().strip()
        if cmd:
            self._send_cmd_safe(cmd)
            self.manual_cmd_var.set("")

    def _send_all(self):
        """Send all current settings to the device"""
        # RF
        hz = self._clamp_freq(int(self.freq_hz_var.get()))
        self._send_cmd_safe(f"freq {hz}")
        
        try:
            ppm = float(self.ppm_var.get().replace(",", "."))
            if -100 <= ppm <= 100:
                self._send_cmd_safe(f"ppm {ppm}")
        except:
            pass
        
        # TX Power
        self._send_cmd_safe(f"txpwr {int(self.txpwr_var.get())}")
        
        # Enables
        self._send_cmd_safe(f"enable bp {'1' if self.en_bp_var.get() else '0'}")
        self._send_cmd_safe(f"enable eq {'1' if self.en_eq_var.get() else '0'}")
        self._send_cmd_safe(f"enable comp {'1' if self.en_comp_var.get() else '0'}")
        
        # Bandpass
        self._send_cmd_safe(f"set bp_lo {self.bp_lo_var.get():.0f}")
        self._send_cmd_safe(f"set bp_hi {self.bp_hi_var.get():.0f}")
        self._send_cmd_safe(f"set bp_stages {int(self.bp_stages_var.get())}")
        
        # EQ
        self._send_cmd_safe(f"set eq_low_hz {self.eq_low_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_low_db {self.eq_low_db_var.get():.1f}")
        self._send_cmd_safe(f"set eq_high_hz {self.eq_high_hz_var.get():.0f}")
        self._send_cmd_safe(f"set eq_high_db {self.eq_high_db_var.get():.1f}")
        
        # Compressor
        self._send_cmd_safe(f"set comp_thr {self.comp_thr_var.get():.1f}")
        self._send_cmd_safe(f"set comp_ratio {self.comp_ratio_var.get():.1f}")
        self._send_cmd_safe(f"set comp_att {self.comp_att_var.get():.1f}")
        self._send_cmd_safe(f"set comp_rel {self.comp_rel_var.get():.0f}")
        self._send_cmd_safe(f"set comp_makeup {self.comp_makeup_var.get():.1f}")
        self._send_cmd_safe(f"set comp_knee {self.comp_knee_var.get():.1f}")
        self._send_cmd_safe(f"set comp_outlim {self.comp_outlim_var.get():.3f}")
        
        # Power shaping
        self._send_cmd_safe(f"set amp_gain {self.amp_gain_var.get():.3f}")
        self._send_cmd_safe(f"set amp_min_a {self.amp_min_a_var.get()}")
        self._send_cmd_safe(f"set vox {self.vox_level_var.get():.4f}")
        
        self._log("All settings sent", "info")

    # === Logging ===
    
    def _log(self, msg: str, tag: str = "recv"):
        self.log_text.insert("end", msg + "\n", tag)
        self.log_text.see("end")
        
        # Also update info text for certain responses
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

    def _poll_rx(self):
        """Poll for received serial data"""
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._log(line, "recv")
        except queue.Empty:
            pass
        self.master.after(50, self._poll_rx)


# ============================================================
# MAIN ENTRY POINT
# ============================================================

def main():
    root = tk.Tk()
    
    # Try to use a modern theme
    style = ttk.Style(root)
    available_themes = style.theme_names()
    for theme in ["clam", "alt", "default"]:
        if theme in available_themes:
            style.theme_use(theme)
            break
    
    # Custom styles
    style.configure("TLabelframe.Label", font=("TkDefaultFont", 10, "bold"))
    
    app = SX1280ControlApp(root)
    
    # Clean shutdown
    def on_close():
        app.worker.disconnect()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
