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
    for field in ["rx_live_var", "rx_state_var", "rx_rssi_var", "rx_snr_var",
                  "rx_fresh_var", "rx_mode_var", "rx_freq_var", "rx_bfo_var",
                  "rx_cal_var", "rx_bw_var", "fw_version_var", "status_var",
                  "log_show_status_var"]:
        setattr(view, field, Var())
    view.rx_live_var.set(True)
    view.log_show_status_var.set(False)
    view.rx_rssi_bar, view.rx_snr_bar = {}, {}
    view.notebook = types.SimpleNamespace(select=lambda: "rx", tab=lambda *args: "RX")
    view._log = lambda line, tag="recv": view.log.append((line, tag))
    view._handle_status_push = lambda line: None
    for name in ["_rx_live_tick", "_rx_reset_status", "_rx_update_freshness",
                 "_handle_rx_status", "_parse_fw_line", "_poll_rx"]:
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


if __name__ == "__main__":
    unittest.main()
