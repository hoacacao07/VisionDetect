"""SETTINGS: paths and grouped configuration. Edit config/settings.json."""

# NAV-CONFIG: add measured mounting transforms, robot footprint, braking limits
# and future controller settings in dedicated groups; do not hide them in UI
# drawing constants or treat the current example front zone as robot geometry.

import json
import math
from pathlib import Path

PROJECT_DIRECTORY = Path(__file__).resolve().parents[2]
DEFAULT_SETTINGS_PATH = PROJECT_DIRECTORY / "config" / "settings.json"


def load_settings(path=DEFAULT_SETTINGS_PATH):
    """Reject common editing mistakes before opening camera hardware."""
    path = Path(path)
    try:
        settings = json.loads(path.read_text(encoding="utf-8-sig"))
        for group in ("camera", "detection", "distance", "output", "motion"):
            if not isinstance(settings.get(group), dict):
                raise ValueError(f"Missing settings section: {group}")
        camera, detection = settings["camera"], settings["detection"]
        distance, output = settings["distance"], settings["output"]
        for name, value in {
            "camera.width": camera["width"],
            "camera.height": camera["height"],
            "detection.image_size": detection["image_size"],
            "detection.cpu_threads": detection["cpu_threads"],
        }.items():
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if type(camera["index"]) is not int or camera["index"] < 0:
            raise ValueError("camera.index must be a non-negative integer")
        if camera.get("driver", "opencv") not in ("opencv", "picamera2"):
            raise ValueError("camera.driver must be opencv or picamera2")
        if camera.get("backend", "auto") not in (
            "auto",
            "dshow",
            "msmf",
            "v4l2",
            "ffmpeg",
            "gstreamer",
        ):
            raise ValueError("camera.backend is not supported")
        source = camera.get("source")
        if source is not None and not (
            (type(source) is int and source >= 0)
            or (isinstance(source, str) and source.strip())
        ):
            raise ValueError(
                "camera.source must be null, a non-negative index, or a non-empty URL/device path"
            )
        if camera.get("driver") == "picamera2" and source is not None:
            raise ValueError("Picamera2 uses camera.index; set camera.source to null")
        for key, default in (("startup_timeout_s", 10), ("maximum_frame_age_s", 2)):
            value = camera.get(key, default)
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"camera.{key} must be positive and finite")
        if (
            type(camera.get("maximum_read_failures", 30)) is not int
            or camera.get("maximum_read_failures", 30) < 1
        ):
            raise ValueError("camera.maximum_read_failures must be a positive integer")
        fps = camera.get("fps")
        if fps is not None and (
            type(fps) not in (int, float) or not math.isfinite(fps) or fps <= 0
        ):
            raise ValueError("camera.fps must be null or positive and finite")
        if not 0 < detection["minimum_confidence"] <= 1:
            raise ValueError("detection.minimum_confidence must be between 0 and 1")
        if not 0 < detection.get("human_minimum_confidence", 0.3) <= 1:
            raise ValueError(
                "detection.human_minimum_confidence must be between 0 and 1"
            )
        if distance["mode"] not in ("size", "ground", "off"):
            raise ValueError("distance.mode must be size, ground, or off")
        if not 10 < distance["horizontal_fov_degrees"] < 150:
            raise ValueError(
                "distance.horizontal_fov_degrees must be between 10 and 150"
            )
        if distance["maximum_distance_m"] <= 0:
            raise ValueError("distance.maximum_distance_m must be positive")
        if (
            type(output["api_port"]) is not int
            or not 1024 <= output["api_port"] <= 65535
        ):
            raise ValueError("output.api_port must be from 1024 to 65535")
        motion = settings["motion"]
        if motion["camera_stationary"] is not True:
            raise ValueError(
                "This tracker requires motion.camera_stationary=true. Use the separate encoder-compensated Webots controller for a moving camera."
            )
        if type(motion["allow_size_based_speed"]) is not bool:
            raise ValueError("motion.allow_size_based_speed must be true or false")
        if type(motion["minimum_samples"]) is not int or motion["minimum_samples"] < 3:
            raise ValueError("motion.minimum_samples must be an integer of at least 3")
        for name in (
            "minimum_observation_s",
            "velocity_window_s",
            "maximum_track_gap_s",
            "maximum_velocity_gap_s",
            "association_distance_fraction",
            "image_motion_threshold_px_s",
            "metric_motion_threshold_mps",
        ):
            if not isinstance(motion[name], (int, float)) or not 0 < motion[
                name
            ] < float("inf"):
                raise ValueError(f"motion.{name} must be positive and finite")
        if motion["velocity_window_s"] < motion["minimum_observation_s"]:
            raise ValueError(
                "Velocity history must cover the minimum observation duration"
            )
        zone = motion["front_zone"]
        if (
            not 0 <= zone["near_m"] < zone["far_m"]
            or zone["width_m"] <= 0
            or zone["prediction_horizon_s"] <= 0
        ):
            raise ValueError(
                "motion.front_zone needs positive width/horizon and 0 <= near_m < far_m"
            )
        return settings
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"Check {path}: {error}") from error


def load_navigation_settings(path=None):
    """Load editable stop/go limits and reject unsafe ordering or invalid units.

    This config controls simulated motors only. The speed-dependent stopping
    allowance is added to stop_gap_m by StopGoController, not baked into a UI.
    """
    path = (
        Path(path)
        if path is not None
        else PROJECT_DIRECTORY / "simulation/navigation_settings.json"
    )
    try:
        config = json.loads(path.read_text(encoding="utf-8-sig"))
        config.setdefault("minimum_slow_speed_fraction", 0.2)
        config.setdefault("unknown_range_center_fraction", 0.2)
        for name in (
            "cruise_speed_mps",
            "stop_gap_m",
            "resume_gap_m",
            "slow_gap_m",
            "braking_deceleration_mps2",
            "corridor_half_width_m",
            "wheel_radius_m",
            "duration_s",
        ):
            value = config[name]
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be positive and finite")
        for name in ("clear_hold_s", "reaction_time_s", "front_offset_m"):
            value = config[name]
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be non-negative and finite")
        if not config["stop_gap_m"] < config["resume_gap_m"] <= config["slow_gap_m"]:
            raise ValueError("Require stop_gap_m < resume_gap_m <= slow_gap_m")
        for name in ("minimum_slow_speed_fraction", "unknown_range_center_fraction"):
            value = config[name]
            if (
                type(value) not in (int, float)
                or not math.isfinite(value)
                or not 0 < value <= 1
            ):
                raise ValueError(f"{name} must be greater than 0 and at most 1")
        return config
    except (ValueError, TypeError, KeyError) as error:
        raise ValueError(f"Check {path}: {error}") from error
