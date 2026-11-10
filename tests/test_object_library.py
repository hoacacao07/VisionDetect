"""Guard mobility semantics and common mistakes when adding new objects."""

import json
from pathlib import Path
import tempfile
import unittest
from vision_detection.object_library import OBJECT_LIBRARY_PATH, load_object_library
from vision_detection.motion import unmeasured_motion


class ObjectLibraryTests(unittest.TestCase):
    def test_vehicle_can_move_but_wall_is_usually_static(self):
        library = load_object_library()
        self.assertEqual(library["car"].motion_group, "potentially_dynamic")
        self.assertEqual(library["wall"].motion_group, "usually_static")

    def test_duplicate_labels_are_rejected_across_categories(self):
        data = json.loads(OBJECT_LIBRARY_PATH.read_text())
        data["motion_groups"]["usually_static"]["categories"]["Duplicate example"] = [
            "car"
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "objects.json"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "Duplicate object label"):
                load_object_library(path)

    def test_new_category_can_be_added_without_python_edits(self):
        data = json.loads(OBJECT_LIBRARY_PATH.read_text())
        data["motion_groups"]["usually_static"]["categories"]["Test additions"] = [
            "garden hose"
        ]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "objects.json"
            path.write_text(json.dumps(data))
            definition = load_object_library(path)["garden hose"]
            self.assertEqual(definition.category, "Test additions")
            self.assertEqual(definition.motion_group, "usually_static")

    def test_unmeasured_motion_does_not_claim_zero_speed_or_arrival(self):
        motion = unmeasured_motion()
        for field in (
            "track_id",
            "object_speed_mps",
            "relative_velocity_mps",
            "closing_speed_mps",
            "time_to_contact_s",
            "front_zone_entry_s",
            "front_zone_exit_s",
        ):
            self.assertIsNone(motion[field])
        self.assertEqual(motion["status"], "unmeasured")
