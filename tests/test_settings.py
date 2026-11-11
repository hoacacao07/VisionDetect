"""Configuration checks protect against confusing errors after manual edits."""

import json
from pathlib import Path
import tempfile
import unittest
from vision_detection.settings import (
    DEFAULT_SETTINGS_PATH,
    load_settings,
    load_navigation_settings,
)


class SettingsTests(unittest.TestCase):
    def test_project_settings_load(self):
        self.assertIn(load_settings()["distance"]["mode"], ("size", "ground", "off"))

    def test_invalid_mode_explains_which_setting_to_fix(self):
        settings = json.loads(DEFAULT_SETTINGS_PATH.read_text())
        settings["distance"]["mode"] = "typo"
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            path.write_text(json.dumps(settings))
            with self.assertRaisesRegex(ValueError, "distance.mode"):
                load_settings(path)

    def test_invalid_navigation_order_is_rejected(self):
        config = load_navigation_settings()
        config["slow_gap_m"] = config["stop_gap_m"]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "navigation.json"
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "stop_gap_m < resume_gap_m"):
                load_navigation_settings(path)

    def test_camera_profiles_load_and_start_without_distance_claims(self):
        # Shipped profiles must not report distances before calibration: either
        # distance is off, or ground mode still lacks the measured mounting.
        from vision_detection.distance import estimate_distance_m

        for path in (DEFAULT_SETTINGS_PATH.parent / "cameras").glob("*.json"):
            settings = load_settings(path)
            if settings["camera"]["driver"] == "sim":
                continue  # Simulated camera: its mounting is known exactly.
            distance, _ = estimate_distance_m(
                [280, 200, 360, 400], "person", 640, 480, settings["distance"]
            )
            self.assertIsNone(distance, path.name)
            self.assertIn(settings["distance"]["mode"], ("off", "ground"), path.name)

    def test_invalid_camera_source_is_rejected(self):
        config = json.loads(DEFAULT_SETTINGS_PATH.read_text())
        config["camera"]["source"] = True
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "settings.json"
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "camera.source"):
                load_settings(path)
