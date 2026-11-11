"""ROBOT SETTINGS: serial link, sensor freshness, control and stop/go.

Edit config/robot_navigation.json. Camera optics/mounting stay in the camera
settings file (config/cameras/robot.json). The robot has no wheel encoders:
the ESP32 reports travel estimated from its open-loop motor model, calibrated
in firmware/esp32_robot/include/robot_config.h (SPEED_AT_FULL_DUTY_MPS).
"""

import json
import math
from pathlib import Path
from .firmware_constants import load_firmware_constants
from .settings import PROJECT_DIRECTORY, validate_navigation_settings

DEFAULT_ROBOT_SETTINGS_PATH = PROJECT_DIRECTORY / "config" / "robot_navigation.json"
# Read from firmware/esp32_robot/include/robot_config.h, never copied by hand.
FIRMWARE = load_firmware_constants()
# The firmware brakes if no valid command arrives for this long.
ESP32_COMMAND_TIMEOUT_S = FIRMWARE["COMMAND_TIMEOUT_MS"] / 1000


def _positive(group, name, value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{group}.{name} must be positive and finite")


def load_robot_settings(path=DEFAULT_ROBOT_SETTINGS_PATH):
    """Read and validate before any serial port, camera or motor is touched."""
    path = Path(path)
    try:
        settings = json.loads(path.read_text(encoding="utf-8-sig"))
        for group in ("serial", "sensors", "control", "navigation", "output"):
            if not isinstance(settings.get(group), dict):
                raise ValueError(f"Missing settings section: {group}")

        serial = settings["serial"]
        if not isinstance(serial["port"], str) or not serial["port"].strip():
            raise ValueError("serial.port must be a device path such as /dev/ttyUSB0")
        if type(serial["baud"]) is not int or serial["baud"] <= 0:
            raise ValueError("serial.baud must be a positive integer")
        _positive("serial", "startup_timeout_s", serial["startup_timeout_s"])

        sensors = settings["sensors"]
        margin = sensors["speed_estimate_margin"]
        if type(margin) not in (int, float) or not 0 <= margin <= 1:
            # Speed is estimated, not measured: the stop distance is computed
            # for (1 + margin) x the estimate so a faster real robot still stops.
            raise ValueError("sensors.speed_estimate_margin must be from 0 to 1")
        for name in (
            "speed_window_s",
            "maximum_range_age_s",
            "maximum_telemetry_age_s",
            "rotation_gyro_threshold_dps",
        ):
            _positive("sensors", name, sensors[name])

        control = settings["control"]
        for name in (
            "rate_hz",
            "maximum_vision_age_s",
            "maximum_acceleration_mps2",
            "maximum_deceleration_mps2",
            "maximum_session_s",
        ):
            _positive("control", name, control[name])
        if 1 / control["rate_hz"] > ESP32_COMMAND_TIMEOUT_S / 2:
            raise ValueError(
                "control.rate_hz is too low: the ESP32 watchdog needs a command "
                f"at least every {ESP32_COMMAND_TIMEOUT_S / 2:g} s"
            )

        navigation = validate_navigation_settings(settings["navigation"], simulation_fields=())
        if navigation["cruise_speed_mps"] > FIRMWARE["MAX_SPEED_MPS"]:
            raise ValueError(
                f"navigation.cruise_speed_mps above {FIRMWARE['MAX_SPEED_MPS']} m/s "
                "exceeds the firmware's MAX_SPEED_MPS"
            )
        if navigation["braking_deceleration_mps2"] > control["maximum_deceleration_mps2"]:
            # The stop distance assumes this braking; the Pi ramp must deliver it.
            raise ValueError(
                "navigation.braking_deceleration_mps2 must not exceed "
                "control.maximum_deceleration_mps2"
            )

        output = settings["output"]
        if output["api_host"] not in ("127.0.0.1", "0.0.0.0"):
            raise ValueError("output.api_host must be 127.0.0.1 or 0.0.0.0")
        if type(output["api_port"]) is not int or not 1024 <= output["api_port"] <= 65535:
            raise ValueError("output.api_port must be from 1024 to 65535")
        if type(output["write_log"]) is not bool:
            raise ValueError("output.write_log must be true or false")
        return settings
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"Check {path}: {error}") from error


def check_robot_camera_calibration(distance):
    """Refuse to drive with uncalibrated camera geometry.

    StopGoController and StraightLineTracker use ground-plane positions, which
    need a measured focal length, camera height and downward tilt.
    """
    problems = []
    if distance.get("mode") != "ground":
        problems.append('distance.mode must be "ground" on the robot')
    calibration = distance.get("focal_calibration") or {}
    if not calibration.get("focal_px") or not calibration.get("image_width"):
        problems.append(
            "distance.focal_calibration.focal_px is not set: run "
            "tools/calibrate_camera.py --config config/cameras/robot.json ..."
        )
    if not distance.get("camera_height_m"):
        problems.append("distance.camera_height_m is not set (lens height above the floor)")
    if distance.get("camera_downward_tilt_degrees") is None:
        problems.append("distance.camera_downward_tilt_degrees is not set")
    if problems:
        raise ValueError("Camera not calibrated for driving:\n  - " + "\n  - ".join(problems))
