"""ESP32 SERIAL LINK: send per-side speed commands, receive sensor data.

The ESP32 owns the motors (firmware/esp32_robot). It brakes on its own when no
valid command arrives for 300 ms, so the Pi must call send_speed() every
control tick, including when the target is zero.

Protocol (text lines, 115200 baud):
  Pi -> ESP32  C,<seq>,<left_mps>,<right_mps>
  ESP32 -> Pi  T,<ms>,<estL_m>,<estR_m>,<rangeL_m>,<rangeC_m>,<rangeR_m>,<gyroZ_dps>,<heading_deg>
               estL/estR are ESTIMATED cumulative travel per side in metres.
               The robot has no wheel encoders: the ESP32 integrates the speed
               its open-loop motor model applied. Ranges and gyro are measured.
               "nan" marks missing, out-of-range or stale values.
               Lines starting with "#" are firmware diagnostics.

Unknown is always NaN here, never 0. All host timestamps use time.monotonic(),
the same clock as LatestFrameCamera, so odometry can be matched to frames.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading
import time

NAN = float("nan")


def encode_command(sequence, left_mps, right_mps):
    """Build one command line. Non-finite speeds are refused, not sent."""
    if not (math.isfinite(left_mps) and math.isfinite(right_mps)):
        raise ValueError("Speed commands must be finite")
    return f"C,{int(sequence) & 0xFFFF},{left_mps:.3f},{right_mps:.3f}\n".encode("ascii")


def _number(text, allow_nan):
    """Parse one numeric field; raise ValueError on garbage or disallowed nan/inf."""
    value = float(text)
    if math.isinf(value):
        value = NAN  # Never treat an infinite range as "far away and clear".
    if math.isnan(value) and not allow_nan:
        raise ValueError("unexpected nan")
    return value


@dataclass(frozen=True)
class Telemetry:
    """One parsed T line, stamped with the Pi's receipt time."""

    received_at: float  # Pi monotonic seconds.
    esp_ms: int
    left_travel_m: float  # Estimated from commands, not measured.
    right_travel_m: float
    ranges_m: tuple  # left, centre, right; NaN = unknown.
    gyro_z_dps: float
    heading_deg: float


def parse_telemetry(line, received_at):
    """Return Telemetry for a well-formed T line, otherwise None."""
    fields = line.strip().split(",")
    if len(fields) != 9 or fields[0] != "T":
        return None
    try:
        esp_ms = int(fields[1])
        if esp_ms < 0:
            return None
        left_travel = _number(fields[2], allow_nan=True)
        right_travel = _number(fields[3], allow_nan=True)
        ranges = tuple(_number(value, allow_nan=True) for value in fields[4:7])
        gyro = _number(fields[7], allow_nan=True)
        heading = _number(fields[8], allow_nan=True)
    except ValueError:
        return None
    ranges = tuple(value if value >= 0 else NAN for value in ranges)
    return Telemetry(received_at, esp_ms, left_travel, right_travel, ranges, gyro, heading)


@dataclass(frozen=True)
class LinkState:
    """Latest robot state, with staleness already applied.

    Travel and speed are ESTIMATES from the open-loop motor model (no encoders).
    """

    received_at: float
    age_s: float
    esp_ms: int
    left_travel_m: float
    right_travel_m: float
    left_speed_mps: float  # Estimated; NaN until enough history exists.
    right_speed_mps: float
    ranges_m: tuple  # NaN if unknown or older than maximum_range_age_s.
    gyro_z_dps: float
    heading_deg: float

    @property
    def travel_m(self):
        """Straight-line travel: mean of both sides."""
        return (self.left_travel_m + self.right_travel_m) / 2

    @property
    def speed_mps(self):
        """Estimated forward speed: mean of both sides (NaN if either is unknown)."""
        return (self.left_speed_mps + self.right_speed_mps) / 2


@dataclass(frozen=True)
class _Sample:
    telemetry: Telemetry
    left_travel_m: float
    right_travel_m: float
    left_speed_mps: float
    right_speed_mps: float


def open_serial_port(port, baud):
    """Open a pyserial port. Imported lazily so tests and the laptop app don't need it.

    "sim://<world>" opens the simulated ESP32 instead (sim/fake_esp32.py).
    """
    if port.startswith("sim://"):
        from sim.fake_esp32 import open_sim_port  # Simulation only (sim/ folder).

        return open_sim_port(port)
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "pyserial is required for the robot: pip install -r requirements.txt"
        ) from error
    return serial.Serial(port, baud, timeout=0.1, write_timeout=0.2)


class Esp32Link:
    """Background telemetry reader plus a thread-safe command writer."""

    def __init__(
        self,
        port,
        baud=115200,
        maximum_range_age_s=0.25,
        speed_window_s=0.2,
        history_s=3.0,
        serial_port=None,
        clock=time.monotonic,
    ):
        """Open the port (or use an injected fake) and start the reader thread."""
        self.maximum_range_age_s = maximum_range_age_s
        self.speed_window_ms = speed_window_s * 1000
        self.history_s = history_s
        self.clock = clock
        self.serial = serial_port if serial_port is not None else open_serial_port(port, baud)
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.history = deque()  # _Sample, oldest first.
        self.sequence = 0
        self.rejected_lines = 0
        self.last_diagnostic = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.read_forever, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ reading
    def read_forever(self):
        """Split the byte stream into lines until closed; record port failures."""
        buffer = b""
        try:
            while not self.stop_event.is_set():
                chunk = self.serial.read(256)
                if not chunk:
                    continue
                buffer += chunk
                *lines, buffer = buffer.split(b"\n")
                if len(buffer) > 512:  # No newline in a long time: wrong baud rate.
                    buffer = b""
                    self.rejected_lines += 1
                for raw in lines:
                    self.handle_line(raw.decode("ascii", errors="replace"))
        except Exception as error:  # Unplugged cable, permission loss, ...
            if not self.stop_event.is_set():
                self.error = error

    def handle_line(self, line):
        """Store one telemetry line; keep diagnostics; count anything else."""
        line = line.strip()
        if not line:
            return
        if line.startswith("#"):
            self.last_diagnostic = line[1:].strip()
            return
        telemetry = parse_telemetry(line, self.clock())
        if telemetry is None:
            self.rejected_lines += 1  # Boot noise or corruption: never used.
            return
        with self.lock:
            if self.history and telemetry.esp_ms <= self.history[-1].telemetry.esp_ms:
                # ESP32 rebooted (or repeated a line): travel restarted, so
                # older samples cannot be compared with new ones.
                self.history.clear()
            self.history.append(self._derive(telemetry))
            while (
                self.history
                and telemetry.received_at - self.history[0].telemetry.received_at
                > self.history_s
            ):
                self.history.popleft()

    def _derive(self, telemetry):
        """Derive speed from estimated travel over ~speed_window_s of ESP32 time."""
        left = telemetry.left_travel_m
        right = telemetry.right_travel_m
        left_speed = right_speed = NAN
        reference = None
        for sample in self.history:  # Oldest sample still inside the window.
            if telemetry.esp_ms - sample.telemetry.esp_ms <= self.speed_window_ms:
                reference = sample
                break
        if reference is None and self.history:
            reference = self.history[-1]  # Sparse telemetry: use the previous one.
        if reference is not None:
            seconds = (telemetry.esp_ms - reference.telemetry.esp_ms) / 1000
            if seconds > 0:
                left_speed = (left - reference.left_travel_m) / seconds
                right_speed = (right - reference.right_travel_m) / seconds
        return _Sample(telemetry, left, right, left_speed, right_speed)

    # ------------------------------------------------------------ queries
    def latest(self, now=None):
        """Newest state with stale ranges masked to NaN, or None before any data."""
        with self.lock:
            sample = self.history[-1] if self.history else None
        if sample is None:
            return None
        now = self.clock() if now is None else now
        telemetry = sample.telemetry
        age = now - telemetry.received_at
        ranges = (
            telemetry.ranges_m
            if age <= self.maximum_range_age_s
            else (NAN,) * len(telemetry.ranges_m)
        )
        return LinkState(
            received_at=telemetry.received_at,
            age_s=age,
            esp_ms=telemetry.esp_ms,
            left_travel_m=sample.left_travel_m,
            right_travel_m=sample.right_travel_m,
            left_speed_mps=sample.left_speed_mps,
            right_speed_mps=sample.right_speed_mps,
            ranges_m=ranges,
            gyro_z_dps=telemetry.gyro_z_dps,
            heading_deg=telemetry.heading_deg,
        )

    def odometry_at(self, timestamp, maximum_gap_s=0.25):
        """(travel_m, speed_mps, gyro_z_dps) at a Pi monotonic time, or None.

        Interpolates travel/speed between the two telemetry lines around a
        camera frame. Gyro is the larger magnitude of the two (conservative for
        the rotation check). Returns None instead of extrapolating across gaps.
        """
        with self.lock:
            samples = list(self.history)
        if not samples:
            return None

        def values(sample):
            return (
                (sample.left_travel_m + sample.right_travel_m) / 2,
                (sample.left_speed_mps + sample.right_speed_mps) / 2,
            )

        last = samples[-1]
        if timestamp >= last.telemetry.received_at:
            if timestamp - last.telemetry.received_at > maximum_gap_s:
                return None
            return (*values(last), last.telemetry.gyro_z_dps)
        for before, after in zip(samples, samples[1:]):
            start, end = before.telemetry.received_at, after.telemetry.received_at
            if start <= timestamp <= end:
                if end - start > maximum_gap_s:
                    return None
                weight = 0.0 if end <= start else (timestamp - start) / (end - start)
                (travel0, speed0), (travel1, speed1) = values(before), values(after)
                gyro0, gyro1 = before.telemetry.gyro_z_dps, after.telemetry.gyro_z_dps
                gyro = NAN if math.isnan(gyro0) or math.isnan(gyro1) else max(
                    gyro0, gyro1, key=abs
                )
                return (
                    travel0 + weight * (travel1 - travel0),
                    speed0 + weight * (speed1 - speed0),
                    gyro,
                )
        return None  # Older than the retained history.

    # ------------------------------------------------------------ writing
    def send_speed(self, left_mps, right_mps):
        """Send one command and return its sequence number. Call every tick."""
        with self.write_lock:
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.serial.write(encode_command(self.sequence, left_mps, right_mps))
            return self.sequence

    def close(self):
        """Command zero a few times, stop the reader and release the port."""
        try:
            for _ in range(3):
                self.send_speed(0.0, 0.0)
                time.sleep(0.02)
        except Exception:
            pass  # The firmware watchdog brakes anyway.
        self.stop_event.set()
        self.thread.join(timeout=1)
        try:
            self.serial.close()
        except Exception:
            pass
