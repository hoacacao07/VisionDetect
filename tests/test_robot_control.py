"""Real-robot STOP paths and watchdog expectations, without hardware or models."""

import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from vision_detection.esp32_link import Esp32Link, LinkState
from vision_detection.robot_config import (
    DEFAULT_ROBOT_SETTINGS_PATH,
    ESP32_COMMAND_TIMEOUT_S,
    check_robot_camera_calibration,
    load_robot_settings,
)
from vision_detection.robot_control import RobotController
from vision_detection.settings import PROJECT_DIRECTORY, load_settings

CALIBRATED = dict(
    mode="ground",
    camera_height_m=0.2,
    camera_downward_tilt_degrees=10,
    focal_calibration=dict(focal_px=500, image_width=640),
    maximum_distance_m=8,
)


def link_state(ranges=(3.0, 3.0, 3.0), speed=0.15, gyro=0.0, age=0.05):
    return LinkState(
        received_at=0.0,
        age_s=age,
        esp_ms=1000,
        left_travel_m=1.0,
        right_travel_m=1.0,
        left_speed_mps=speed,
        right_speed_mps=speed,
        ranges_m=tuple(ranges),
        gyro_z_dps=gyro,
        heading_deg=0.0,
    )


def perception(now, age=0.1, health=None, detections=None, odometry_ok=True):
    return dict(
        capture_time=now - age,
        health=health or dict(status="clear", alert=False),
        detections=detections or [],
        width=640,
        height=480,
        odometry_ok=odometry_ok,
        rotating=False,
    )


class RobotControlTests(unittest.TestCase):
    def setUp(self):
        self.robot = load_robot_settings()
        self.controller = RobotController(
            copy.deepcopy(self.robot["navigation"]),
            CALIBRATED,
            self.robot["sensors"],
            self.robot["control"],
            (640, 480),
        )

    def step(self, now, link=None, seen=None, motors=True, dt=0.05):
        link = link_state() if link is None else link
        seen = perception(now) if seen is None else seen
        return self.controller.step(now, 0.0, dt, link, seen, motors)

    def drive_until_moving(self):
        self.step(1.0)
        decision = self.step(2.1)  # clear_hold_s = 1.0 satisfied.
        self.assertEqual(decision["state"], "MOVE")
        for tick in range(20):  # Let the ramp reach cruise speed.
            decision = self.step(2.15 + tick * 0.05)
        self.assertGreater(decision["applied_motor_speed_mps"], 0.1)
        return 2.15 + 20 * 0.05

    def assertStops(self, decision, reason):
        self.assertEqual(decision["state"], "STOP")
        self.assertEqual(decision["target_speed_mps"], 0)
        self.assertEqual(decision["reason"], reason)

    # ---- all inputs fresh ----------------------------------------------
    def test_fresh_inputs_move_after_clear_hold(self):
        self.drive_until_moving()

    def test_dry_run_never_commands_motion(self):
        self.step(1.0, motors=False)
        decision = self.step(2.1, motors=False)
        self.assertEqual(decision["state"], "MOVE")  # Policy would move...
        self.assertEqual(decision["applied_motor_speed_mps"], 0)  # ...but sends 0.

    # ---- STOP paths ------------------------------------------------------
    def test_stale_vision_stops(self):
        now = self.drive_until_moving()
        decision = self.step(now, seen=perception(now, age=1.0))
        self.assertStops(decision, "waiting_for_fresh_vision")

    def test_no_frame_yet_stops(self):
        decision = self.controller.step(1.0, 0.0, 0.05, link_state(), None, True)
        self.assertStops(decision, "waiting_for_first_frame")

    def test_stale_ranges_stop(self):
        now = self.drive_until_moving()
        decision = self.step(now, link=link_state(ranges=(3.0, math.nan, 3.0)))
        self.assertStops(decision, "range_sensor_unavailable")

    def test_out_of_range_echo_is_not_treated_as_clear(self):
        decision = self.step(1.0, link=link_state(ranges=(math.nan,) * 3))
        self.assertStops(decision, "range_sensor_unavailable")

    def test_ranges_masked_by_link_age_stop_end_to_end(self):
        class Clock:
            now = 100.0

            def __call__(self):
                return self.now

        class QuietSerial:
            def read(self, _size):
                return b""

            def write(self, _data):
                pass

            def close(self):
                pass

        clock = Clock()
        link = Esp32Link("fake", serial_port=QuietSerial(), clock=clock, maximum_range_age_s=0.25)
        try:
            link.handle_line("T,1000,0.0,0.0,3.0,3.0,3.0,0.0,0.0")
            clock.now = 100.05
            link.handle_line("T,1050,1.0,1.0,3.0,3.0,3.0,0.0,0.0")
            # 0.2 s later the ranges are still fresh; 0.3 s later they are masked.
            fresh = self.step(100.25, link=link.latest(now=100.25))
            self.assertNotEqual(fresh["reason"], "range_sensor_unavailable")
            stale = self.step(100.36, link=link.latest(now=100.36))
            self.assertStops(stale, "esp32_telemetry_stale")
            self.assertEqual(stale["front_ranges_m"], [None, None, None])
        finally:
            link.close()

    def test_camera_blocked_stops(self):
        now = self.drive_until_moving()
        blocked = perception(now, health=dict(status="blocked_or_too_dark", alert=True))
        self.assertStops(self.step(now, seen=blocked), "camera_blocked")

    def test_stale_or_missing_telemetry_stops(self):
        now = self.drive_until_moving()
        self.assertStops(self.step(now, link=link_state(age=0.5)), "esp32_telemetry_stale")
        decision = self.controller.step(now, 0.0, 0.05, None, perception(now), True)
        self.assertStops(decision, "esp32_telemetry_stale")

    def test_missing_gyro_or_speed_estimate_stops(self):
        now = self.drive_until_moving()
        self.assertStops(self.step(now, link=link_state(gyro=math.nan)), "gyro_unavailable")
        self.assertStops(
            self.step(now + 0.05, link=link_state(speed=math.nan)), "speed_estimate_unavailable"
        )

    def test_stop_distance_uses_speed_estimate_plus_margin(self):
        # No encoders: the real robot may be faster than estimated, so the
        # stopping threshold is computed for (1 + margin) x the estimate.
        def threshold(margin):
            sensors = dict(self.robot["sensors"], speed_estimate_margin=margin)
            controller = RobotController(
                copy.deepcopy(self.robot["navigation"]), CALIBRATED, sensors,
                self.robot["control"], (640, 480),
            )
            decision = controller.step(1.0, 0.0, 0.05, link_state(speed=0.15), perception(1.0), True)
            self.assertEqual(decision["estimated_speed_mps"], 0.15)  # Reported unscaled.
            return decision["stopping_threshold_m"]

        navigation = self.robot["navigation"]
        expected_extra = (
            (0.15 * 1.3) ** 2 - 0.15**2
        ) / (2 * navigation["braking_deceleration_mps2"]) + 0.15 * 0.3 * navigation[
            "reaction_time_s"
        ]
        self.assertAlmostEqual(threshold(0.3) - threshold(0.0), expected_extra, places=2)

    def test_frame_without_odometry_stops(self):
        now = self.drive_until_moving()
        decision = self.step(now, seen=perception(now, odometry_ok=False))
        self.assertStops(decision, "odometry_unavailable_for_frame")

    def test_obstacle_in_range_stops(self):
        now = self.drive_until_moving()
        decision = self.step(now, link=link_state(ranges=(3.0, 0.5, 3.0)))
        self.assertStops(decision, "obstacle_in_stop_zone")

    def test_restart_after_fault_needs_clear_hold_again(self):
        now = self.drive_until_moving()
        self.step(now, seen=perception(now, age=1.0))  # Fault -> STOP.
        self.assertEqual(self.step(now + 0.05)["state"], "STOP")
        self.assertEqual(self.step(now + 0.5)["state"], "STOP")
        self.assertEqual(self.step(now + 1.1)["state"], "MOVE")

    # ---- watchdog expectations (Pi side) ---------------------------------
    def test_stop_ramps_command_to_zero_and_always_returns_a_command(self):
        now = self.drive_until_moving()
        decel = self.robot["control"]["maximum_deceleration_mps2"]
        cruise = self.robot["navigation"]["cruise_speed_mps"]
        ticks = math.ceil(cruise / decel / 0.05) + 1
        commands = [
            self.step(now + i * 0.05, link=link_state(age=1.0))["applied_motor_speed_mps"]
            for i in range(ticks)
        ]
        self.assertTrue(all(math.isfinite(c) and c >= 0 for c in commands))
        self.assertEqual(commands, sorted(commands, reverse=True))
        self.assertEqual(commands[-1], 0)

    def test_shipped_rate_feeds_the_esp32_watchdog(self):
        self.assertLessEqual(1 / self.robot["control"]["rate_hz"], ESP32_COMMAND_TIMEOUT_S / 2)


class RobotConfigTests(unittest.TestCase):
    def write(self, config):
        path = Path(tempfile.mkdtemp()) / "robot_navigation.json"
        path.write_text(json.dumps(config))
        return path

    def load_with(self, group, key, value):
        config = json.loads(DEFAULT_ROBOT_SETTINGS_PATH.read_text())
        config[group][key] = value
        return load_robot_settings(self.write(config))

    def test_slow_control_rate_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "watchdog"):
            self.load_with("control", "rate_hz", 3)

    def test_speed_estimate_margin_must_be_a_fraction(self):
        for bad in (-0.1, 1.5, "0.3"):
            with self.assertRaisesRegex(ValueError, "speed_estimate_margin"):
                self.load_with("sensors", "speed_estimate_margin", bad)

    def test_no_encoder_settings_remain(self):
        self.assertNotIn("drivetrain", load_robot_settings())

    def test_braking_assumption_must_be_achievable(self):
        with self.assertRaisesRegex(ValueError, "braking_deceleration"):
            self.load_with("navigation", "braking_deceleration_mps2", 2.0)

    def test_default_cruise_speed(self):
        self.assertEqual(load_robot_settings()["navigation"]["cruise_speed_mps"], 0.15)

    def test_robot_camera_file_loads_but_refuses_to_drive_until_calibrated(self):
        settings = load_settings(PROJECT_DIRECTORY / "config/cameras/robot.json")
        self.assertEqual(settings["camera"]["driver"], "picamera2")
        # Camera Module 3 Wide (102 deg); 16:9 output keeps the full sensor width.
        self.assertEqual(settings["distance"]["horizontal_fov_degrees"], 102)
        camera = settings["camera"]
        self.assertEqual(camera["width"] * 9, camera["height"] * 16)
        with self.assertRaisesRegex(ValueError, "not calibrated"):
            check_robot_camera_calibration(settings["distance"])
        check_robot_camera_calibration(CALIBRATED)


if __name__ == "__main__":
    unittest.main()
