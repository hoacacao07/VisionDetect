"""SCENARIO RUNNER: robot.py's control loop on a simulated clock.

Uses the real Pi code (Esp32Link, PerceptionWorker.process, RobotController,
control_tick) against the FakeEsp32 and rendered camera frames. Only the clock
and the threads are replaced: everything advances in fixed simulated steps, so
a 15 s drive runs in about a second and every run is repeatable.

Timeline per step:
  * fake ESP32: 1 ms loop ticks; telemetry lines go straight to Esp32Link
  * camera: one frame every 1/camera_fps s, result published after
    perception_latency_s (a busy Pi can't start a new frame before that)
  * Pi control tick every 1/control.rate_hz s (control_tick from robot_runtime)
"""

import copy
import math
import time
from dataclasses import dataclass, field
from vision_detection.ego_motion import StraightLineTracker
from vision_detection.esp32_link import Esp32Link
from vision_detection.robot_config import load_robot_settings
from vision_detection.robot_control import RobotController
from vision_detection.robot_runtime import (
    PerceptionWorker,
    control_tick,
    make_detector,
    prepare_robot_settings,
)
from vision_detection.settings import PROJECT_DIRECTORY, load_settings
from .fake_esp32 import FakeEsp32
from .render_scene import render_scene
from .world import bumper_clearance, load_robot_sim, load_world

SIM_CAMERA_SETTINGS = PROJECT_DIRECTORY / "config" / "cameras" / "sim.json"
NAN = float("nan")


@dataclass
class Scenario:
    """What to simulate. Times are seconds from the start."""

    name: str
    world: str
    duration_s: float
    motors_enabled: bool = True
    seed: int = 1
    stop_after_travel_m: float | None = None  # End early once the robot has moved this far.
    pi_quiet_at_s: float | None = None  # The Pi stops sending commands (crash / unplug).
    camera_stops_at_s: float | None = None  # No new camera frames after this.
    sensor_nan_at_s: float | None = None  # One HC-SR04 starts returning nan...
    sensor_nan_index: int = 1  # ...this one (0 left, 1 centre, 2 right).
    force_ranges_m: float | None = None  # Every HC-SR04 reports this (camera-only tests).
    firmware_overrides: dict = field(default_factory=dict)  # e.g. disable heading hold.


@dataclass
class SimResult:
    """Everything a test needs to judge a run."""

    scenario: Scenario
    rows: list = field(default_factory=list)  # One dict per Pi control tick.
    frames: list = field(default_factory=list)  # One dict per camera result.
    fake: FakeEsp32 | None = None
    world: dict | None = None
    robot: dict | None = None
    last_command_s: float | None = None  # Last time the Pi sent a command.


class _QuietSerial:
    """Esp32Link's port: writes go to the fake; reads are fed by the runner."""

    def __init__(self, fake):
        self.fake = fake

    def read(self, _size):
        time.sleep(0.05)
        return b""

    def write(self, data):
        self.fake.receive(data)

    def close(self):
        pass


def run_scenario(scenario, camera_settings=None, robot_settings=None, robot_sim=None):
    """Run one scenario and return a SimResult."""
    camera_settings = camera_settings or load_settings(SIM_CAMERA_SETTINGS)
    robot_settings = robot_settings or load_robot_settings()
    robot = robot_sim or load_robot_sim()
    settings = prepare_robot_settings(camera_settings)
    sensors, control = robot_settings["sensors"], robot_settings["control"]
    navigation = copy.deepcopy(robot_settings["navigation"])
    width, height = settings["camera"]["width"], settings["camera"]["height"]

    world = load_world(scenario.world)
    fake = FakeEsp32(world, robot, seed=scenario.seed)
    fake.c = {**fake.c, **scenario.firmware_overrides}
    if scenario.force_ranges_m is not None:
        fake.forced_ranges = {i: scenario.force_ranges_m for i in range(3)}
    clock = {"now": 0.0}
    link = Esp32Link(
        "sim",
        serial_port=_QuietSerial(fake),
        clock=lambda: clock["now"],
        maximum_range_age_s=sensors["maximum_range_age_s"],
        speed_window_s=sensors["speed_window_s"],
    )
    controller = RobotController(navigation, settings["distance"], sensors, control, (width, height))
    worker = PerceptionWorker(
        None,
        settings,
        make_detector(settings),
        StraightLineTracker(settings["motion"], settings["distance"]),
        link,
        sensors,
    )
    result = SimResult(scenario, fake=fake, world=world, robot=robot)
    control_ms = round(1000 / control["rate_hz"])
    frame_ms = round(1000 / robot["camera_fps"])
    latency_s = robot["perception_latency_s"]
    start_x = fake.pose[0]
    perception, pending, busy_until = None, None, 0.0
    previous_tick = 0.0
    try:
        for step_ms in range(1, int(scenario.duration_s * 1000) + 1):
            now = step_ms / 1000
            clock["now"] = now
            for line in fake.advance(1):
                link.handle_line(line)
            if scenario.sensor_nan_at_s is not None and now >= scenario.sensor_nan_at_s:
                fake.forced_ranges[scenario.sensor_nan_index] = NAN
            # Camera: capture, then publish after the processing delay.
            camera_running = scenario.camera_stops_at_s is None or now < scenario.camera_stops_at_s
            if step_ms % frame_ms == 0 and camera_running and now >= busy_until:
                frame = render_scene(
                    world, fake.pose, now, settings["distance"], robot,
                    navigation["front_offset_m"], width, height,
                )
                processed = worker.process(frame, now)
                processed["frame_id"] = len(result.frames) + 1
                pending, busy_until = (now + latency_s, processed), now + latency_s
                result.frames.append(
                    dict(t=now, pose=tuple(fake.pose), detections=processed["detections"],
                         health=processed["health"], rotating=processed["rotating"])
                )
            if pending and now >= pending[0]:
                perception, pending = pending[1], None
            # Pi control tick.
            if step_ms % control_ms == 0:
                pi_alive = scenario.pi_quiet_at_s is None or now < scenario.pi_quiet_at_s
                if not pi_alive:
                    result.rows.append(_row(now, fake, world, robot, None, None))
                    continue
                decision, link_state = control_tick(
                    controller, link, perception, now, 0.0, now - previous_tick, scenario.motors_enabled
                )
                previous_tick, result.last_command_s = now, now
                result.rows.append(_row(now, fake, world, robot, decision, link_state))
                if (
                    scenario.stop_after_travel_m is not None
                    and fake.pose[0] - start_x >= scenario.stop_after_travel_m
                ):
                    break
    finally:
        link.close()
    return result


def _row(now, fake, world, robot, decision, link_state):
    """One log line: true robot state + what the Pi decided."""
    x, y, heading = fake.pose
    row = dict(
        t=now,
        x=x,
        y=y,
        heading_deg=math.degrees(heading),
        true_speed=sum(fake.wheel_speed) / 2,
        clearance_m=bumper_clearance(fake.pose, robot, world, now),
        braking=fake.braking(),
        pi_alive=decision is not None,
    )
    if decision is not None:
        row.update(
            state=decision["state"],
            reason=decision["reason"],
            source=decision["limiting_source"],
            nearest_gap_m=decision["nearest_gap_m"],
            target=decision["target_speed_mps"],
            command=decision["applied_motor_speed_mps"],
            estimated_speed=decision["estimated_speed_mps"],
            ranges=decision["front_ranges_m"],
            vision_age_s=decision["vision_age_s"],
        )
    return row
