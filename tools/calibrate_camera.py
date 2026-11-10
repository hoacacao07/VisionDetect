"""Calibrate focal length from one fully visible object at a known distance.

Example: python tools/calibrate_camera.py --distance 2 --height 1.7 --pixels 320 --width 640
Measure the object's actual height and box height in the same webcam image.
"""

import argparse
import json
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--distance", type=float, required=True, help="Known optical-axis depth, metres"
    )
    parser.add_argument(
        "--height",
        type=float,
        required=True,
        help="Actual upright object's height, metres",
    )
    parser.add_argument(
        "--pixels",
        type=float,
        required=True,
        help="Full object bounding-box height, pixels",
    )
    parser.add_argument("--width", type=int, required=True, help="Image width, pixels")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config/settings.json",
        help="Camera-specific settings file to update",
    )
    measurements = parser.parse_args()
    if (
        min(
            measurements.distance,
            measurements.height,
            measurements.pixels,
            measurements.width,
        )
        <= 0
    ):
        parser.error("Measurements must be positive")
    config_path = measurements.config
    config = json.loads(config_path.read_text())
    # Rearranged pinhole equation: focal_pixels = depth_m * box_pixels / height_m.
    # Store the calibration image width so distance.py can scale the focal length.
    config["distance"]["focal_calibration"] = {
        "focal_px": measurements.distance * measurements.pixels / measurements.height,
        "image_width": measurements.width,
    }
    # Preserve other user settings. Restart the detector to load this calibration.
    config_path.write_text(json.dumps(config, indent=2) + "\n")
    print(
        "Saved focal calibration. Restart the detector. Recalibrate after changing camera/zoom."
    )
