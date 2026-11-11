"""Pi <-> ESP32 protocol, staleness, odometry and shutdown, with a fake serial port.

No wheel encoders: the ESP32 reports ESTIMATED travel per side in metres.
"""

import math
import queue
import threading
import time
import unittest
from vision_detection.esp32_link import Esp32Link, encode_command, parse_telemetry


def t_line(ms, left=0.0, right=None, ranges=("1.000", "2.000", "3.000"), gyro="0.00", heading="0.00"):
    """left/right: estimated travel in metres, as the firmware prints it."""
    right = left if right is None else right
    return f"T,{ms},{left:.4f},{right:.4f},{ranges[0]},{ranges[1]},{ranges[2]},{gyro},{heading}"


class FakeSerial:
    """Byte-stream fake: read() returns queued chunks, write() records bytes."""

    def __init__(self):
        self.incoming = queue.Queue()
        self.written = []
        self.closed = threading.Event()

    def feed(self, data):
        self.incoming.put(data)

    def read(self, _size):
        try:
            return self.incoming.get(timeout=0.01)
        except queue.Empty:
            return b""

    def write(self, data):
        self.written.append(data)

    def close(self):
        self.closed.set()


class FakeClock:
    def __init__(self, now=100.0):
        self.now = now

    def __call__(self):
        return self.now


def wait_until(condition, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.005)
    return False


class ProtocolTests(unittest.TestCase):
    def test_command_format(self):
        self.assertEqual(encode_command(5, 0.15, 0.15), b"C,5,0.150,0.150\n")
        self.assertEqual(encode_command(0x10001, 0, 0), b"C,1,0.000,0.000\n")

    def test_non_finite_command_is_refused(self):
        with self.assertRaises(ValueError):
            encode_command(1, float("nan"), 0.0)

    def test_valid_telemetry(self):
        telemetry = parse_telemetry(t_line(1500, 0.125, 0.13, gyro="-3.25", heading="10.50"), 7.0)
        self.assertEqual(telemetry.esp_ms, 1500)
        self.assertEqual((telemetry.left_travel_m, telemetry.right_travel_m), (0.125, 0.13))
        self.assertEqual(telemetry.ranges_m, (1.0, 2.0, 3.0))
        self.assertEqual(telemetry.gyro_z_dps, -3.25)
        self.assertEqual(telemetry.received_at, 7.0)

    def test_missing_ranges_stay_nan_never_zero(self):
        telemetry = parse_telemetry(t_line(10, ranges=("nan", "inf", "-1.0")), 0.0)
        self.assertTrue(all(math.isnan(value) for value in telemetry.ranges_m))

    def test_malformed_lines_are_rejected(self):
        for line in (
            "",
            "T,1,0,0,1,1,1,0",  # Too few fields.
            "T,1,0,0,1,1,1,0,0,9",  # Too many.
            "X,1,0,0,1,1,1,0,0",
            "T,abc,0,0,1,1,1,0,0",
            "T,-5,0,0,1,1,1,0,0",
            "T,1,0,0,1,one,1,0,0",
            "ets Jun  8 2016 00:22:57",  # ESP32 ROM boot text.
        ):
            self.assertIsNone(parse_telemetry(line, 0.0), line)


class LinkTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.serial = FakeSerial()
        self.link = Esp32Link(
            "fake",
            maximum_range_age_s=0.25,
            speed_window_s=0.2,
            serial_port=self.serial,
            clock=self.clock,
        )

    def tearDown(self):
        self.link.close()

    def feed(self, at, line):
        self.clock.now = at
        self.link.handle_line(line)

    def test_reader_thread_reassembles_split_lines_and_skips_noise(self):
        self.serial.feed(b"\x00garbage\r\n# imu calibrated bias=0.1\n" + t_line(100, 0.04).encode()[:9])
        self.serial.feed(t_line(100, 0.04).encode()[9:] + b"\r\n")
        self.assertTrue(wait_until(lambda: self.link.latest() is not None))
        state = self.link.latest()
        self.assertAlmostEqual(state.left_travel_m, 0.04)
        self.assertEqual(self.link.rejected_lines, 1)
        self.assertEqual(self.link.last_diagnostic, "imu calibrated bias=0.1")

    def test_ranges_become_nan_when_older_than_maximum_age(self):
        self.feed(100.0, t_line(1000))
        self.assertEqual(self.link.latest(now=100.2).ranges_m, (1.0, 2.0, 3.0))
        stale = self.link.latest(now=100.3)
        self.assertTrue(all(math.isnan(r) for r in stale.ranges_m))
        self.assertAlmostEqual(stale.age_s, 0.3)

    def test_speed_needs_history_then_uses_window(self):
        self.feed(100.00, t_line(1000, 0.0))
        self.assertTrue(math.isnan(self.link.latest().speed_mps))  # One sample: unknown.
        self.feed(100.05, t_line(1050, 0.0075))
        self.feed(100.10, t_line(1100, 0.015))
        expected = 0.015 / 0.1
        self.assertAlmostEqual(self.link.latest().left_speed_mps, expected)
        self.assertAlmostEqual(self.link.latest().speed_mps, expected)

    def test_missing_side_estimate_gives_nan_speed(self):
        self.feed(100.00, t_line(1000, 0.0).replace(",0.0000,0.0000,", ",0.0000,nan,"))
        self.feed(100.05, t_line(1050, 0.01).replace(",0.0100,0.0100,", ",0.0100,nan,"))
        self.assertTrue(math.isnan(self.link.latest().speed_mps))

    def test_esp32_reboot_discards_old_travel(self):
        self.feed(100.00, t_line(5000, 3.0))
        self.feed(100.05, t_line(20, 0.0))
        self.assertEqual(len(self.link.history), 1)
        self.assertTrue(math.isnan(self.link.latest().speed_mps))

    def test_odometry_interpolates_and_refuses_gaps(self):
        self.feed(100.00, t_line(1000, 0.0, gyro="2.00"))
        self.feed(100.05, t_line(1050, 0.01, gyro="-20.00"))
        travel, speed, gyro = self.link.odometry_at(100.025)
        self.assertAlmostEqual(travel, 0.005)
        self.assertEqual(gyro, -20.0)  # Larger magnitude: conservative rotation check.
        self.feed(101.00, t_line(2000, 0.02))  # One-second hole.
        self.assertIsNone(self.link.odometry_at(100.5))
        self.assertIsNone(self.link.odometry_at(101.5))  # After the last report.
        self.assertIsNone(self.link.odometry_at(50.0))  # Before history.

    def test_every_command_has_a_new_sequence(self):
        first = self.link.send_speed(0.1, 0.1)
        second = self.link.send_speed(0.0, 0.0)
        self.assertEqual(second, first + 1)
        self.assertEqual(self.serial.written[-1], b"C,2,0.000,0.000\n")

    def test_close_commands_zero_and_releases_port(self):
        self.link.send_speed(0.15, 0.15)
        self.link.close()
        self.assertEqual(self.serial.written[-1].split(b",")[2:], [b"0.000", b"0.000\n"])
        self.assertTrue(self.serial.closed.is_set())
        self.assertFalse(self.link.thread.is_alive())


if __name__ == "__main__":
    unittest.main()
