"""Headless tests of the actual GUI serial worker and RX callbacks."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("qo100_gui_test", ROOT / "gui.py")
gui = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = gui
spec.loader.exec_module(gui)


class Var:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class FakeThread:
    def __init__(self, target=None, args=(), daemon=True):
        self.target, self.args, self.alive = target, args, False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive


class Port:
    def __init__(self, read=None):
        self.is_open = True
        self.read_action = read

    def read(self, count):
        if self.read_action:
            return self.read_action()
        raise OSError("USB re-enumerated")

    def close(self):
        self.is_open = False


class Worker:
    def __init__(self):
        self.connected, self.lines = True, []

    def is_connected(self):
        return self.connected

    def send_line(self, line):
        self.lines.append(line)


class Master:
    def __init__(self):
        self.timers = []

    def after(self, delay, callback):
        self.timers.append((delay, callback))

    def focus_get(self):
        return None


def make_view():
    view = types.SimpleNamespace(worker=Worker(), master=Master(), rx_queue=queue.Queue(),
                                 _rx_last_on=None, _rx_last_status_at=None,
                                 _rx_next_poll_at=0.0, log=[])
    view.rx_waterfall = types.SimpleNamespace(busy=False, observe=lambda line: False, reset=lambda: None)
    view.rx_playback = types.SimpleNamespace(stop=lambda: None, messages=queue.Queue())
    for field in ["rx_live_var", "rx_state_var", "rx_rssi_var", "rx_snr_var",
                  "rx_fresh_var", "rx_mode_var", "rx_freq_var", "rx_bfo_var",
                  "rx_cal_var", "rx_bw_var", "fw_version_var", "status_var",
                  "log_show_status_var", "rx_pc_audio_var", "rx_pc_audio_status"]:
        setattr(view, field, Var())
    view.rx_live_var.set(True)
    view.log_show_status_var.set(False)
    view.rx_rssi_bar, view.rx_snr_bar = {}, {}
    view.notebook = types.SimpleNamespace(select=lambda: "rx", tab=lambda *args: "RX")
    view._log = lambda line, tag="recv": view.log.append((line, tag))
    view._handle_status_push = lambda line: None
    for name in ["_rx_live_tick", "_rx_reset_status", "_rx_update_freshness",
                 "_handle_rx_status", "_parse_fw_line", "_poll_rx", "_rx_playback_status"]:
        setattr(view, name, types.MethodType(getattr(gui.SX1280ControlApp, name), view))
    return view


def status(on=1, rssi=35, snr=25):
    return (f"!R on={on} mode=usb hz=19360000 cal=0 bw=2200 bfo=0 vol=50 "
            f"mute=0 rssi={rssi} snr={snr} tuner_khz=19360 if_hz=19360000 "
            "actual_hz=19360000 bfo_override=0")


class SerialTests(unittest.TestCase):
    def active_worker(self, port):
        worker = gui.SerialWorker(queue.Queue())
        worker.ser = port
        worker.thread = FakeThread()
        worker.thread.start()
        return worker

    def test_current_reader_error_disconnects_and_closes(self):
        port = Port()
        worker = self.active_worker(port)
        stop = worker.stop_evt
        self.assertTrue(worker.is_connected())
        worker._rx_loop(port, stop)
        self.assertFalse(worker.is_connected())
        self.assertFalse(port.is_open)
        self.assertTrue(stop.is_set())
        self.assertIn("[SERIAL ERROR] USB re-enumerated", worker.rx_queue.get_nowait())

    def test_old_reader_error_does_not_clear_new_connection(self):
        old, new = Port(), Port()
        worker = self.active_worker(new)
        stop = worker.stop_evt
        worker._rx_loop(old, threading.Event())
        self.assertIs(worker.ser, new)
        self.assertIs(worker.stop_evt, stop)
        self.assertTrue(worker.is_connected())
        self.assertTrue(new.is_open)
        self.assertTrue(worker.rx_queue.empty())

    def test_reconnect_uses_distinct_stop_event(self):
        ports = [Port(), Port()]
        worker = gui.SerialWorker(queue.Queue())
        with patch.object(gui, "HAS_SERIAL", True), \
             patch.object(gui, "serial", types.SimpleNamespace(Serial=lambda **kwargs: ports.pop(0))), \
             patch.object(gui.threading, "Thread", FakeThread):
            worker.connect("COM1")
            old_thread, old_stop = worker.thread, worker.stop_evt
            worker.disconnect()
            worker.connect("COM2")
            current = worker.ser
            self.assertIsNot(worker.stop_evt, old_stop)
            self.assertTrue(old_stop.is_set())
            old_thread.target(*old_thread.args)
            self.assertIs(worker.ser, current)
            self.assertTrue(worker.is_connected())
            self.assertTrue(current.is_open)

    def test_old_reader_discards_late_bytes(self):
        new = Port()
        worker = self.active_worker(new)
        old = Port(read=lambda: b"old firmware response\n")
        worker._rx_loop(old, threading.Event())
        self.assertTrue(worker.rx_queue.empty())
        self.assertIs(worker.ser, new)
        self.assertTrue(new.is_open)


class GuiTests(unittest.TestCase):
    def test_version_old_and_git_formats(self):
        cases = [
            ("FW: 2.3.0-rx built Nov 11 2026 22:09:09", "FW 2.3.0-rx"),
            ("  fw=2.3.0-rx  built=Nov 11 2026 22:09:09", "FW 2.3.0-rx"),
            ("SX1280_SDR control ready. FW 2.3.1-rx git=abc123 built Nov 12 2026 08:19:00. Type 'help'.", "git=abc123"),
            ("FW: 2.3.1-rx git=abc123-dirty built Nov 12 2026 08:19:00", "git=abc123-dirty"),
            ("  fw=2.3.1-rx  git=abc123  built=Nov 12 2026 08:19:00", "git=abc123"),
            ("[FW] 2.3.1-rx git=abc123 built Nov 12 2026 08:19:00", "git=abc123"),
        ]
        for line, expected in cases:
            with self.subTest(line=line):
                view = make_view()
                view._parse_fw_line(line)
                self.assertIn(expected, view.fw_version_var.get())
                self.assertIn("Nov 1", view.fw_version_var.get())
                self.assertNotIn("Type", view.fw_version_var.get())

    def test_meters_accept_changed_and_constant_values(self):
        view = make_view()
        with patch.object(gui.time, "monotonic", return_value=100.0):
            view._handle_rx_status(status())
        self.assertEqual(view.rx_rssi_var.get(), "35 dBµV")
        self.assertEqual(view.rx_snr_var.get(), "25 dB")
        self.assertEqual(view.rx_rssi_bar["value"], 35)
        with patch.object(gui.time, "monotonic", return_value=102.0):
            view._handle_rx_status(status(snr=0))
        self.assertEqual(view.rx_snr_bar["value"], 0)
        self.assertEqual(view._rx_last_status_at, 102.0)
        view._rx_update_freshness(106.0)
        self.assertIn("VERALTET", view.rx_fresh_var.get())

    def test_off_state_is_rechecked_and_on_state_resumes(self):
        view = make_view()
        view._rx_last_on = False
        for now in [100.0, 101.0, 105.0]:
            with patch.object(gui.time, "monotonic", return_value=now):
                view._rx_live_tick()
        self.assertEqual(view.worker.lines, ["rx", "rx"])
        with patch.object(gui.time, "monotonic", return_value=106.0):
            view._handle_rx_status(status())
        with patch.object(gui.time, "monotonic", return_value=107.0):
            view._rx_live_tick()
        self.assertEqual(view.worker.lines, ["rx", "rx", "rx"])

    def test_connection_reset_removes_stale_off_state(self):
        view = make_view()
        view._rx_last_on = False
        view._rx_last_status_at = 50.0
        view._rx_next_poll_at = 500.0
        view._rx_reset_status()
        with patch.object(gui.time, "monotonic", return_value=100.0):
            view._rx_live_tick()
        self.assertEqual(view.worker.lines, ["rx"])
        self.assertIsNone(view._rx_last_on)
        self.assertEqual(view.rx_rssi_var.get(), "—")

    def test_command_replies_visible_with_status_pushes_hidden(self):
        view = make_view()
        for line in ["FW: 2.3.1-rx git=abc123 built Nov 12 2026 08:19:00", "Commands: help get", status(), "!S ignored=1"]:
            view.rx_queue.put(line)
        view._poll_rx()
        self.assertEqual([line for line, tag in view.log], [
            "FW: 2.3.1-rx git=abc123 built Nov 12 2026 08:19:00", "Commands: help get", status()])
        self.assertTrue(view.master.timers)
        self.assertEqual(gui.SX1280ControlApp._classify_log(status(), "recv"), "recv")

    def test_reader_failure_updates_connection_status(self):
        view = make_view()
        view.worker.connected = False
        view.status_var.set("🟢 Connected")
        view.rx_queue.put("[SERIAL ERROR] USB re-enumerated")
        view._poll_rx()
        self.assertIn("Verbindung unterbrochen", view.status_var.get())
        self.assertIn("nicht verbunden", view.rx_fresh_var.get())
        self.assertIsNone(view._rx_last_on)

    def test_queued_old_error_does_not_mark_new_connection_failed(self):
        view = make_view()
        view.status_var.set("🟢 Connected")
        view.rx_queue.put("[SERIAL ERROR] old connection failed")
        view._poll_rx()
        self.assertEqual(view.status_var.get(), "🟢 Connected")

    def test_bad_status_does_not_stop_console_receive_timer(self):
        view = make_view()
        def bad(line):
            raise ValueError("bad status")
        view._handle_rx_status = bad
        view.rx_queue.put(status())
        view._poll_rx()
        self.assertIn("[RX PARSE ERROR]", view.log[0][0])
        self.assertTrue(view.master.timers)
        view.rx_queue.put("Commands: help get")
        view._poll_rx()
        self.assertEqual(view.log[-1][0], "Commands: help get")

    def test_bad_push_does_not_stop_console_receive_timer(self):
        view = make_view()
        def bad(line):
            raise ValueError("bad push")
        view._handle_status_push = bad
        view.rx_queue.put("!S invalid=1")
        view._poll_rx()
        self.assertIn("[STATUS PARSE ERROR]", view.log[0][0])
        self.assertTrue(view.master.timers)


class RxAudioTests(unittest.TestCase):
    def test_pc_gain_converts_decibels_to_pcm_scale(self):
        playback = gui.RxUsbPlayback()
        playback.set_gain_db(6)
        self.assertAlmostEqual(playback.gain, 1.9952623, places=6)
        playback.set_gain_db(0)
        self.assertEqual(playback.gain, 1.0)

    @unittest.skipUnless(gui.HAS_AUDIO, "PyAudio and NumPy required")
    def test_pc_highpass_attenuates_rumble_and_preserves_speech_band(self):
        np = gui.np
        playback = gui.RxUsbPlayback()
        self.assertFalse(playback.highpass)
        time = np.arange(4 * playback.BLOCK) / playback.RATE
        for frequency, expected_db in ((60, -19.1), (180, -3.0), (300, -0.53), (700, -0.02)):
            with self.subTest(frequency=frequency):
                tone = np.sin(2 * np.pi * frequency * time)
                state = (0.0, 0.0)
                blocks = []
                for block in np.split(tone, 4):
                    filtered, state = playback._highpass_block(block, state)
                    blocks.append(filtered)
                continuous, _ = playback._highpass_block(tone, (0.0, 0.0))
                np.testing.assert_array_equal(np.concatenate(blocks), continuous)
                settled = slice(2 * playback.BLOCK, None)
                gain_db = 10 * np.log10(np.sum(continuous[settled] ** 2) / np.sum(tone[settled] ** 2))
                self.assertAlmostEqual(gain_db, expected_db, delta=0.4)

    @unittest.skipUnless(gui.HAS_AUDIO, "PyAudio and NumPy required")
    def test_live_spectrum_preserves_the_pcm_sent_to_speakers(self):
        np = gui.np
        playback = gui.RxUsbPlayback()
        playback.output_enabled = True
        frequencies, _ = gui.RxWaterfall.audio_range(3000, 600)
        playback.spectrum_session = frequencies
        time = np.arange(playback.RATE // frequencies.step) / playback.RATE
        samples = (0.3 * 32768 * np.sin(2 * np.pi * 60 * time) +
                   0.1 * 32768 * np.sin(2 * np.pi * 730 * time)).astype(np.int16)
        incoming = iter(block.tobytes() for block in np.split(samples, 5))
        written = []

        def write(data, **kwargs):
            written.append(data)
            if len(written) == 5:
                playback.stop()

        capture = types.SimpleNamespace(read=lambda *args, **kwargs: next(incoming), close=lambda: None)
        speaker = types.SimpleNamespace(write=write, close=lambda: None)
        audio = types.SimpleNamespace(open=lambda **kwargs: capture if kwargs.get("input") else speaker,
                                      terminate=lambda: None)
        out = {"maxOutputChannels": 1, "index": 1, "name": "Speaker"}
        with patch.object(gui.pyaudio, "PyAudio", return_value=audio), \
                patch.object(playback, "devices", return_value=({"index": 0}, out)):
            playback._play()

        self.assertEqual(b"".join(written), samples.tobytes())
        session, spectrum = playback.spectrum_frame
        self.assertIs(session, playback.spectrum_session)
        self.assertEqual(max(zip(spectrum, frequencies)), ((-20, 0), 730))

    @unittest.skipUnless(gui.HAS_AUDIO, "PyAudio and NumPy required")
    def test_live_capture_remains_available_with_the_output_off(self):
        playback = gui.RxUsbPlayback()
        playback.spectrum_session = range(200, 3201, 10)
        read_count = 0

        def read(*args, **kwargs):
            nonlocal read_count
            read_count += 1
            if read_count == 5:
                playback.stop()
            return bytes(playback.BLOCK * 2)

        capture = types.SimpleNamespace(read=read, close=lambda: None)
        opened = []

        def open_stream(**kwargs):
            opened.append(kwargs)
            return capture

        audio = types.SimpleNamespace(open=open_stream, terminate=lambda: None)
        with patch.object(gui.pyaudio, "PyAudio", return_value=audio), \
                patch.object(playback, "devices", return_value=({"index": 0}, None)):
            playback._play()

        self.assertEqual(len(opened), 1)
        self.assertTrue(opened[0]["input"])
        self.assertIs(playback.spectrum_frame[0], playback.spectrum_session)

    @unittest.skipUnless(gui.HAS_AUDIO, "PyAudio and NumPy required")
    def test_zoomed_fft_resolves_two_tones_six_hz_apart(self):
        np = gui.np
        playback = gui.RxUsbPlayback
        bins, _ = gui.RxWaterfall.audio_range(500, 1200)
        window = np.hanning(playback.RATE // bins.step)
        time = np.arange(len(window)) / playback.RATE
        samples = 2500 * (np.sin(2 * np.pi * 1200 * time) + np.sin(2 * np.pi * 1206 * time))
        spectrum = playback._audio_spectrum(samples, window, bins)
        levels = dict(zip(bins, (point[0] for point in spectrum)))
        self.assertGreater(levels[1200], levels[1202] + 3)
        self.assertGreater(levels[1206], levels[1204] + 3)

    def test_selects_radio_input_and_macbook_speakers(self):
        radio = dict(name="QO-100 USB Audio", maxInputChannels=1, maxOutputChannels=2)
        mac = dict(name="MacBook Pro-Lautsprecher", maxInputChannels=0, maxOutputChannels=2)
        audio = types.SimpleNamespace(get_device_count=lambda: 2,
                                      get_device_info_by_index=[radio, mac].__getitem__,
                                      get_default_output_device_info=lambda: radio)
        self.assertEqual(gui.RxUsbPlayback.devices(audio), (radio, mac))

    def test_rejects_playback_back_into_transmitter(self):
        radio = dict(name="QO-100 USB Audio", maxInputChannels=1, maxOutputChannels=2)
        audio = types.SimpleNamespace(get_device_count=lambda: 1,
                                      get_device_info_by_index=lambda _: radio,
                                      get_default_output_device_info=lambda: radio)
        with self.assertRaisesRegex(RuntimeError, "Systemausgabe"):
            gui.RxUsbPlayback.devices(audio)


class WaterfallTests(unittest.TestCase):
    def setUp(self):
        app = make_view()
        app._rx_last_on = True
        app._rx_set_freq = lambda: app.worker.send_line(f"rx freq {app.rx_freq_var.get()}")
        app._send_cmd_safe = app.worker.send_line
        app.rx_playback = gui.RxUsbPlayback()
        app.rx_pc_audio_var = Var(True)
        app._rx_start_capture = lambda: True
        app._rx_stop_capture_if_idle = lambda: None
        app._RX_BW_AM = gui.SX1280ControlApp._RX_BW_AM
        app._RX_BW_SSB = gui.SX1280ControlApp._RX_BW_SSB
        waterfall = object.__new__(gui.RxWaterfall)
        waterfall.app = app
        waterfall.frequencies = range(0)
        waterfall.history = gui.deque(maxlen=90)
        waterfall.points = {}
        waterfall.chunk = None
        waterfall.running = waterfall.audio_view = False
        waterfall.audio_session = waterfall.last_spectrum = None
        waterfall.audio_frequencies, waterfall.audio_center_hz = waterfall.audio_range(3000, 600)
        waterfall.audio_span_var, waterfall.audio_center_var = Var("3000"), Var("600")
        waterfall.filter_var = Var()
        waterfall.tx_idle = False
        waterfall.scan_context = None
        waterfall.nominal_if_hz = None
        waterfall.dial_khz = waterfall.bfo_override = waterfall.restore_bfo = waterfall.pending_tune = None
        waterfall.start_khz, waterfall.stop_khz, waterfall.step_khz = Var("19340"), Var("19346"), Var("1")
        waterfall.status = Var()
        waterfall.low, waterfall.high = Var(), Var()
        waterfall.scale, waterfall.audio_scale = (15, 65), (-95, -65)
        waterfall.live_button = waterfall.scan_button = waterfall.metric_box = waterfall.canvas = waterfall.filter_box = \
            types.SimpleNamespace(config=lambda **kwargs: None)
        waterfall._draw = waterfall._draw_marker = lambda: None
        waterfall.after = app.master.after
        app.rx_waterfall = self.waterfall = waterfall
        self.app = app
        self.lines = app.worker.lines
        waterfall.observe("!S tx=0 tune=0 key=0")
        waterfall.observe(status())

    def feed_chunk(self, first, last, omit=None):
        for line in (f"Scanning {first}..{last} kHz in 1 kHz steps, 8 reads each",
                     "  kHz    rssi avg/peak  snr    level"):
            self.assertTrue(self.waterfall.observe(line))
        for khz in range(first, last + 1):
            if khz == omit:
                self.assertTrue(self.waterfall.observe(f"  {khz}   -- measurement failed: tune"))
            else:
                self.assertTrue(self.waterfall.observe(f"  {khz}    41 / 48   15   #####"))
        self.assertTrue(self.waterfall.observe(f"Highest average: 41 dBuV at {first} kHz"))
        self.assertTrue(self.waterfall.busy)
        self.assertTrue(self.waterfall.observe(status()))

    def test_stitches_chunks_and_keeps_failed_bin_empty(self):
        self.waterfall.start_scan()
        self.assertTrue(self.app.rx_playback.scan_muted)
        self.assertEqual(self.lines, ["rx scan 19340 19345 1"])
        self.assertFalse(self.waterfall.observe("Scanning 19300..19400 kHz in 1 kHz steps, 8 reads each"))
        self.assertFalse(self.waterfall.observe("  19340    53 / 53   11   #####"))
        self.feed_chunk(19340, 19345, omit=19342)
        self.assertFalse(self.waterfall.history)
        with patch.object(gui.time, "monotonic", return_value=self.waterfall.next_at + 1):
            self.waterfall._tick()
        self.assertEqual(self.lines[-1], "rx scan 19346 19346 1")
        self.feed_chunk(19346, 19346)
        self.assertFalse(self.waterfall.running)
        self.assertFalse(self.app.rx_playback.scan_muted)
        self.assertEqual(self.waterfall.history[0], ((41, 15),) * 2 + (None,) + ((41, 15),) * 4)

    def test_click_waits_for_scan_restore_and_tunes_last_visible_bin(self):
        self.waterfall._plot = lambda: (10, 30, 70, 100)
        self.waterfall.start_scan()
        self.waterfall._click(types.SimpleNamespace(x=79, y=50))
        self.assertEqual(self.lines, ["rx scan 19340 19345 1"])
        self.feed_chunk(19340, 19345)
        self.waterfall._tick()
        self.assertEqual(self.lines, ["rx scan 19340 19345 1", "rx freq 19346"])
        self.assertFalse(self.waterfall.running)

    def test_stop_restores_raw_bfo_after_firmware_completes(self):
        self.waterfall.observe(status().replace("bfo=0", "bfo=275").replace("bfo_override=0", "bfo_override=1"))
        self.waterfall.start_scan()
        self.waterfall.pause()
        self.feed_chunk(19340, 19345)
        self.assertEqual(self.lines, ["rx scan 19340 19345 1", "rx bfo 275"])

    def test_guard_error_and_timeout_pause_without_retry(self):
        self.waterfall.observe("!S tx=1 tune=0 key=0")
        self.waterfall.start_scan()
        self.assertFalse(self.lines)
        self.waterfall.observe("!S tx=0 tune=0 key=0")
        self.waterfall.start_scan()
        self.assertFalse(self.waterfall.observe("ERR: receiver is off"))
        self.assertFalse(self.waterfall.busy)
        self.assertFalse(self.waterfall.running)
        self.waterfall.start_scan()
        with patch.object(gui.time, "monotonic", return_value=self.waterfall.chunk.deadline + 1):
            self.waterfall._tick()
        self.assertFalse(self.waterfall.busy)
        self.assertFalse(self.waterfall.running)
        self.assertEqual(len(self.lines), 2)

    def test_live_uses_audio_frames_without_retuning_and_ignores_stale_data(self):
        wave, playback = self.waterfall, self.app.rx_playback
        wave._toggle_live()
        self.assertIs(playback.spectrum_session, wave.audio_session)
        self.assertIsNotNone(wave.audio_session)
        self.assertFalse(playback.scan_muted)
        row = ((-70, 0),) * len(wave.frequencies)
        playback.spectrum_frame = (wave.audio_session, row)
        wave._tick()
        wave._tick()
        self.assertEqual(list(wave.history), [row])
        wave.observe(status().replace("hz=19360000", "hz=19361000"))
        wave._tick()
        self.assertFalse(wave.history)
        wave._toggle_live()
        self.assertIsNone(playback.spectrum_session)
        self.assertFalse(self.lines)

    def test_audio_click_moves_the_selected_offset_to_600_hz(self):
        wave = self.waterfall
        for mode, audio_hz, expected in (("usb", 1440, "19360.945"), ("lsb", 1440, "19359.265"),
                                          ("usb", 400, "19359.905"), ("lsb", 400, "19360.305")):
            with self.subTest(mode=mode, audio_hz=audio_hz):
                wave.observe(status().replace("mode=usb", f"mode={mode}")
                             .replace("actual_hz=19360000", "actual_hz=19360105")
                             .replace("bfo_override=0", "bfo_override=1").replace("bfo=0", "bfo=-105"))
                wave._toggle_live()
                count = len(wave.frequencies)
                wave.history.append(((-75, 0),) * count)
                wave._plot = lambda: (10, 30, count * 2, 100)
                x = 10 + round(wave._bin_fraction(wave.frequencies.index(audio_hz)) * count * 2)
                wave._click(types.SimpleNamespace(x=x, y=50))
                self.assertEqual(self.lines.pop(), f"rx freq {expected}")
                self.assertIsNotNone(wave.audio_session)
                self.assertFalse(self.app.rx_playback.scan_muted)
                wave._toggle_live()

    def test_audio_axis_centers_the_tuning_marker_and_looks_up_the_drawn_bin(self):
        wave = self.waterfall
        wave._toggle_live()
        self.assertEqual(wave._bin_fraction(wave.frequencies.index(600)), 0.5)
        samples = {0.0: 200, 0.25: 400, 0.5: 600, 0.75: 1900, 1.0: 3200}
        for fraction, frequency in samples.items():
            self.assertEqual(wave.frequencies[wave._bin_at_fraction(fraction)], frequency)
        for index in range(len(wave.frequencies)):
            self.assertEqual(wave._bin_at_fraction(wave._bin_fraction(index)), index)
        count = len(wave.frequencies)
        wave.history.append(((-75, 0),) * count)
        wave._plot = lambda: (10, 30, 800, 100)
        wave._click(types.SimpleNamespace(x=410, y=50))
        self.assertFalse(self.lines)

    def test_zoom_changes_audio_resolution_and_center_without_tuning(self):
        wave = self.waterfall
        wave._toggle_live()
        session = wave.audio_session
        wave.history.append(((-75, 0),) * len(wave.frequencies))
        wave.audio_span_var.set("500")
        wave.audio_center_var.set("1200")
        wave._apply_audio_view()
        self.assertEqual(wave.frequencies, range(950, 1451, 2))
        self.assertEqual(wave._bin_fraction(wave.frequencies.index(1200)), 0.5)
        self.assertIsNot(wave.audio_session, session)
        self.assertFalse(wave.history)
        self.assertFalse(self.lines)

    def test_shift_click_selects_nearest_hardware_filter_without_tuning(self):
        wave = self.waterfall
        wave._toggle_live()
        wave._plot = lambda: (10, 30, 1000, 100)
        x = 10 + round(1000 * wave._bin_fraction(wave.frequencies.index(1280)))
        self.assertEqual(wave._click_filter(types.SimpleNamespace(x=x, y=50)), "break")
        self.assertEqual(self.lines, ["rx bw 1200"])
        wave.observe(status().replace("bw=2200", "bw=1200"))
        self.assertEqual(wave.filter_var.get(), "1200")

    def test_audio_click_requires_ssb_and_a_legal_tuning_range(self):
        wave = self.waterfall
        wave._toggle_live()
        count = len(wave.frequencies)
        wave._plot = lambda: (10, 30, count * 2, 100)
        click = types.SimpleNamespace(x=10 + (count - 1) * 2, y=50)
        wave.observe(status().replace("mode=usb", "mode=am"))
        wave._click(click)
        self.assertIn("USB und LSB", wave.status.get())
        wave.observe(status().replace("19360000", "26100000"))
        wave.history.append(((-75, 0),) * count)
        wave._click(click)
        self.assertIn("außerhalb", wave.status.get())
        self.assertFalse(self.lines)

    def test_bounds_match_actual_scan_grid(self):
        self.assertEqual(list(gui.RxWaterfall.sweep_range("19340", "19345", "2")), [19340, 19342, 19344])
        for arguments in (("2300", "3000", "1"), ("2299", "2301", "1"), ("2400", "2402", "3")):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                gui.RxWaterfall.sweep_range(*arguments)

    def test_auto_contrast_keeps_nearby_levels_visible_despite_a_strong_bin(self):
        row = [None, (38, 0)] + [(39, 0)] * 20 + [(41, 0), (82, 22)]
        low, high = gui.RxWaterfall._automatic_scale([row], 0)
        self.assertLess(low, 39)
        self.assertGreaterEqual(high, 41)
        self.assertLessEqual(high - low, 12)

    def test_auto_snr_scale_keeps_zero_as_zero_and_has_a_minimum_span(self):
        self.assertEqual(gui.RxWaterfall._automatic_scale([[(41, 0), None, (42, 0)]], 1), (0, 3))
        self.assertIsNone(gui.RxWaterfall._automatic_scale([[None]], 0))


if __name__ == "__main__":
    unittest.main()
