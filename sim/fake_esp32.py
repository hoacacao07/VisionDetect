"""FAKE ESP32: the firmware's main loop in Python, plus a simulated robot body.

Speaks exactly the firmware protocol (firmware/esp32_robot/src/main.cpp):
  in   C,<seq>,<left_mps>,<right_mps>
  out  T,<ms>,<estL_m>,<estR_m>,<rangeL_m>,<rangeC_m>,<rangeR_m>,<gyroZ_dps>,<heading_deg>
All firmware numbers come from robot_config.h via load_firmware_constants(),
so the fake cannot drift from the firmware. Each function below mirrors the
C++ function with the same name; keep them in step when editing main.cpp.

The "real world" part (motor response with a left/right gain mismatch, gyro
bias and noise, differential-drive pose, HC-SR04 rays) is NOT in the firmware;
its parameters are in sim/robot_sim.json.

Two ways to run it:
  * FakeEsp32.advance(ms)  - stepped simulation time (tests, sim/runner.py)
  * open_sim_port("sim://corridor") - a real-time serial-port stand-in, used
    by Esp32Link when robot.py is started with --serial sim://<world>
"""

import math
import random
import re
import threading
import time
from urllib.parse import parse_qs, urlparse
from vision_detection.firmware_constants import load_firmware_constants
from .world import load_robot_sim, load_world, start_pose, ultrasonic_true_range

NAN = float("nan")
GYRO_LSB_PER_DPS = 131.0  # MPU-6050 at +/-250 dps, as in main.cpp.


class Side:
    """One motor side, as in main.cpp's struct Side."""

    def __init__(self, invert):
        self.invert = invert
        self.target = 0.0
        self.setpoint = 0.0
        self.applied = 0.0
        self.travel = 0.0
        # What the L298N outputs right now: (braking, direction, duty).
        self.braking, self.direction, self.duty = True, 1, 0.0


class FakeEsp32:
    """Firmware logic + simulated robot, advanced in 1 ms steps."""

    def __init__(self, world, robot=None, constants=None, seed=0, lifted=False):
        self.c = constants or load_firmware_constants()
        self.robot = robot or load_robot_sim()
        self.world = world
        self.lifted = lifted  # Wheels off the ground: they spin, the robot stays put.
        self.random = random.Random(seed)
        self.now_ms = 0
        self.left, self.right = Side(self.c["LEFT_MOTOR_INVERT"]), Side(self.c["RIGHT_MOTOR_INVERT"])
        self.wheel_speed = [0.0, 0.0]  # True left/right ground speed, m/s.
        self.pose = list(start_pose(world))
        self.true_yaw_rate_dps = 0.0
        # Protocol state.
        self.command_valid = False
        self.last_valid_command_ms = 0
        self.line = ""
        self.line_overflow = False
        self.output = []  # Telemetry/diagnostic lines not yet collected.
        # Sensors.
        self.ranges = [NAN] * 3
        self.range_at_ms = [0] * 3
        self.next_sensor = 0
        self.forced_ranges = {}  # Fault injection: {sensor index: metres or nan}.
        self.gyro_z_dps = NAN
        self.heading_deg = 0.0
        self.imu_ok = True
        self.heading_held = False
        self.heading_target_deg = 0.0
        # Timers, as the static variables in loop().
        self.last_control_ms = self.last_imu_ms = self.last_ultrasonic_ms = 0
        self.last_telemetry_ms = -self.c["TELEMETRY_PERIOD_MS"]
        # Logs for tests.
        self.commands = []  # (ms, seq, left, right) for every VALID command.
        self.events = []  # (ms, text) for watchdog / rejected input.
        self.gyro_bias_estimate = self.calibrate_gyro()

    # ------------------------------------------------------------ IMU (imuBegin / updateImu)
    def gyro_raw_dps(self):
        """What a chip-side-up MPU-6050 reports: true rate + bias + noise, in 1/131 dps steps."""
        reading = (
            self.true_yaw_rate_dps
            + self.robot["gyro_bias_dps"]
            + self.random.gauss(0, self.robot["gyro_noise_dps"])
        )
        return round(reading * GYRO_LSB_PER_DPS) / GYRO_LSB_PER_DPS

    def calibrate_gyro(self):
        """imuBegin(): average IMU_CALIBRATION_SAMPLES readings while the robot is still."""
        samples = [self.gyro_raw_dps() for _ in range(self.c["IMU_CALIBRATION_SAMPLES"])]
        return sum(samples) / len(samples)

    def update_imu(self, dt):
        dps = self.gyro_raw_dps() - self.gyro_bias_estimate
        if self.c["IMU_Z_INVERT"]:
            dps = -dps
        self.gyro_z_dps = dps
        self.heading_deg += dps * dt
        while self.heading_deg > 180:
            self.heading_deg -= 360
        while self.heading_deg <= -180:
            self.heading_deg += 360

    # ------------------------------------------------------------ motors
    def brake(self, side):
        side.braking, side.duty, side.applied = True, 0.0, 0.0

    def drive(self, side, speed):
        """Open-loop: requested m/s -> duty. Returns the speed the model expects."""
        if abs(speed) < 1e-3:
            self.brake(side)
            return 0.0
        direction = 1 if speed > 0 else -1
        effort = min(abs(speed) / self.c["SPEED_AT_FULL_DUTY_MPS"], 1.0)
        side.braking, side.direction = False, direction
        side.duty = self.c["MIN_DUTY"] + (1 - self.c["MIN_DUTY"]) * effort
        return direction * effort * self.c["SPEED_AT_FULL_DUTY_MPS"]

    def stop_now(self, side):
        side.target = side.setpoint = 0.0
        self.brake(side)

    def update_side(self, side, dt, correction):
        speeding_up = abs(side.target) > abs(side.setpoint) and side.target * side.setpoint >= 0
        step = (self.c["MAX_ACCEL_MPS2"] if speeding_up else self.c["MAX_DECEL_MPS2"]) * dt
        if side.target > side.setpoint:
            side.setpoint = min(side.target, side.setpoint + step)
        else:
            side.setpoint = max(side.target, side.setpoint - step)
        speed = side.setpoint + correction
        if abs(side.setpoint) < 0.005 or speed * side.setpoint <= 0:
            self.brake(side)
        else:
            side.applied = self.drive(side, speed)
        side.travel += side.applied * dt

    def heading_correction(self, heading_deg, imu_ok):
        straight = self.left.target > 0 and self.left.target == self.right.target
        if not straight or not imu_ok or math.isnan(heading_deg) or self.c["HEADING_HOLD_MPS_PER_DEG"] <= 0:
            self.heading_held = False
            return 0.0
        if not self.heading_held:
            self.heading_held, self.heading_target_deg = True, heading_deg
        error = heading_deg - self.heading_target_deg
        while error > 180:
            error -= 360
        while error <= -180:
            error += 360
        limit = self.c["HEADING_HOLD_MAX_MPS"]
        return max(-limit, min(limit, self.c["HEADING_HOLD_MPS_PER_DEG"] * error))

    # ------------------------------------------------------------ ultrasonic
    def fire_ultrasonic(self):
        """ultrasonicTask(): one HC-SR04 per ULTRASONIC_PERIOD_MS, in turn."""
        i = self.next_sensor
        metres = NAN
        true = ultrasonic_true_range(self.pose, self.robot["ultrasonic_sensors"][i], self.robot, self.world, self.now_ms / 1000)
        if math.isfinite(true):
            measured = true + self.random.gauss(0, self.robot["ultrasonic_noise_m"])
            echo_us = 2 * measured / self.c["SPEED_OF_SOUND_MPS"] * 1e6
            if 0 < echo_us <= self.c["ULTRASONIC_TIMEOUT_US"]:  # Else pulseIn() times out.
                metres = echo_us * 1e-6 * self.c["SPEED_OF_SOUND_MPS"] / 2
                if not self.c["ULTRASONIC_MIN_M"] <= metres <= self.c["ULTRASONIC_MAX_M"]:
                    metres = NAN
        if i in self.forced_ranges:
            metres = self.forced_ranges[i]
        self.ranges[i], self.range_at_ms[i] = metres, self.now_ms
        self.next_sensor = (i + 1) % 3

    def read_range(self, i):
        if self.range_at_ms[i] == 0 or self.now_ms - self.range_at_ms[i] > self.c["RANGE_REPORT_MAX_AGE_MS"]:
            return NAN
        return self.ranges[i]

    # ------------------------------------------------------------ protocol
    def reject_input(self, why):
        self.command_valid = False
        self.stop_now(self.left)
        self.stop_now(self.right)
        self.events.append((self.now_ms, f"rejected: {why}"))

    def handle_line(self, line):
        fields = [field for field in line.split(",") if field][:5]  # Like strtok().
        if len(fields) != 4 or fields[0] != "C":
            return self.reject_input(line)
        if not re.fullmatch(r"\s*[+-]?\d+", fields[1]) or int(fields[1]) < 0:
            return self.reject_input(line)
        try:
            left, right = float(fields[2]), float(fields[3])
        except ValueError:
            return self.reject_input(line)
        if not (math.isfinite(left) and math.isfinite(right)):
            return self.reject_input(line)
        if abs(left) > self.c["COMMAND_SANITY_MPS"] or abs(right) > self.c["COMMAND_SANITY_MPS"]:
            return self.reject_input(line)
        limit = self.c["MAX_SPEED_MPS"]
        self.left.target = max(-limit, min(limit, left))
        self.right.target = max(-limit, min(limit, right))
        self.last_valid_command_ms = self.now_ms
        self.command_valid = True
        self.commands.append((self.now_ms, int(fields[1]), left, right))

    def receive(self, data):
        """readSerial(): bytes from the Pi."""
        for char in data.decode("ascii", errors="replace"):
            if char == "\r":
                continue
            if char == "\n":
                if self.line_overflow:
                    self.reject_input("line too long")
                elif self.line:
                    self.handle_line(self.line)
                self.line, self.line_overflow = "", False
            elif len(self.line) < self.c["MAX_LINE_LENGTH"]:
                self.line += char
            else:
                self.line_overflow = True

    def telemetry_line(self):
        def value(number, decimals):
            return "nan" if not math.isfinite(number) else f"{number:.{decimals}f}"

        fields = [f"T,{self.now_ms}", value(self.left.travel, 4), value(self.right.travel, 4)]
        fields += [value(self.read_range(i), 3) for i in range(3)]
        fields += [value(self.gyro_z_dps if self.imu_ok else NAN, 2), value(self.heading_deg if self.imu_ok else NAN, 2)]
        return ",".join(fields)

    # ------------------------------------------------------------ simulated robot body
    def physics(self, dt):
        """Motor response and differential-drive kinematics (not firmware)."""
        true_full = self.robot.get("true_speed_at_full_duty_mps") or self.c["SPEED_AT_FULL_DUTY_MPS"]
        for index, (side, gain) in enumerate(
            ((self.left, self.robot["motor_gain"]["left"]), (self.right, self.robot["motor_gain"]["right"]))
        ):
            if side.braking:
                goal, tau = 0.0, self.robot["brake_time_constant_s"]
            else:
                # Below MIN_DUTY the motor does not turn; above it, speed is linear in duty.
                fraction = max(0.0, (side.duty - self.c["MIN_DUTY"]) / (1 - self.c["MIN_DUTY"]))
                goal, tau = side.direction * gain * true_full * fraction, self.robot["motor_time_constant_s"]
            self.wheel_speed[index] += (goal - self.wheel_speed[index]) * min(1.0, dt / tau)
        if self.lifted:
            self.true_yaw_rate_dps = 0.0
            return
        v_left, v_right = self.wheel_speed
        speed = (v_left + v_right) / 2
        yaw_rate = (v_right - v_left) / self.robot["track_width_m"]
        self.true_yaw_rate_dps = math.degrees(yaw_rate)
        x, y, heading = self.pose
        self.pose = [
            x + speed * math.cos(heading) * dt,
            y + speed * math.sin(heading) * dt,
            heading + yaw_rate * dt,
        ]

    # ------------------------------------------------------------ main loop
    def tick(self):
        """One millisecond of loop(), then 1 ms of physics."""
        self.now_ms += 1
        now = self.now_ms
        c = self.c
        if self.command_valid and now - self.last_valid_command_ms >= c["COMMAND_TIMEOUT_MS"]:
            self.command_valid = False
            self.stop_now(self.left)
            self.stop_now(self.right)
            self.events.append((now, "watchdog: no command, braking"))
        if not self.command_valid:
            self.left.target = self.right.target = 0.0
        if now - self.last_imu_ms >= c["IMU_PERIOD_MS"]:
            self.update_imu((now - self.last_imu_ms) / 1000)
            self.last_imu_ms = now
        if now - self.last_ultrasonic_ms >= c["ULTRASONIC_PERIOD_MS"]:
            self.fire_ultrasonic()
            self.last_ultrasonic_ms = now
        if now - self.last_control_ms >= c["CONTROL_PERIOD_MS"]:
            dt = (now - self.last_control_ms) / 1000
            self.last_control_ms = now
            if not self.command_valid:
                self.stop_now(self.left)
                self.stop_now(self.right)
                self.heading_correction(NAN, False)
            else:
                correction = self.heading_correction(self.heading_deg, self.imu_ok)
                self.update_side(self.left, dt, correction)
                self.update_side(self.right, dt, -correction)
        if now - self.last_telemetry_ms >= c["TELEMETRY_PERIOD_MS"]:
            self.last_telemetry_ms = now
            self.output.append(self.telemetry_line())
        self.physics(0.001)

    def advance(self, milliseconds):
        """Run the loop for this many milliseconds; return the new output lines."""
        for _ in range(int(milliseconds)):
            self.tick()
        lines, self.output = self.output, []
        return lines

    def braking(self):
        """True when both L298N sides are in brake mode."""
        return self.left.braking and self.right.braking


# ---------------------------------------------------------------- real-time port
ACTIVE_PORT = None  # The sim camera reads the robot pose from here.


class SimSerialPort:
    """pyserial-like object: a FakeEsp32 running in real time on a thread."""

    def __init__(self, fake):
        self.fake = fake
        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.data_ready = threading.Condition(self.lock)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        started = time.monotonic()
        while not self.stop_event.is_set():
            target_ms = int((time.monotonic() - started) * 1000)
            with self.lock:
                lines = self.fake.advance(max(0, target_ms - self.fake.now_ms))
                if lines:
                    self.buffer += "".join(line + "\r\n" for line in lines).encode("ascii")
                    self.data_ready.notify_all()
            time.sleep(0.002)

    def read(self, size):
        with self.data_ready:
            if not self.buffer:
                self.data_ready.wait(timeout=0.1)
            data = bytes(self.buffer[:size])
            del self.buffer[:size]
            return data

    def write(self, data):
        with self.lock:
            self.fake.receive(data)

    def pose_and_time(self):
        """(pose, sim seconds) for the camera renderer."""
        with self.lock:
            return tuple(self.fake.pose), self.fake.now_ms / 1000

    def close(self):
        global ACTIVE_PORT
        self.stop_event.set()
        self.thread.join(timeout=1)
        if ACTIVE_PORT is self:
            ACTIVE_PORT = None


def open_sim_port(url):
    """sim://<world>[?seed=N&lifted=1] -> SimSerialPort (see sim/worlds/)."""
    global ACTIVE_PORT
    parsed = urlparse(url)
    query = parse_qs(parsed.query)
    world = load_world(parsed.netloc + parsed.path or "corridor")
    fake = FakeEsp32(
        world,
        seed=int(query.get("seed", ["0"])[0]),
        lifted=query.get("lifted", ["0"])[0] == "1",
    )
    ACTIVE_PORT = SimSerialPort(fake)
    return ACTIVE_PORT
