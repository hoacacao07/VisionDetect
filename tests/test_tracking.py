"""Synthetic timestamped paths test speed/arrival math without a webcam."""

import copy
import unittest
from vision_detection.settings import load_settings
from vision_detection.tracking import StationaryCameraTracker, front_zone_interval


def detection(bounds, label="person"):
    return {"label": label, "bbox_xyxy": bounds, "motion_group": "potentially_dynamic"}


class TrackingTests(unittest.TestCase):
    def setUp(self):
        self.motion = copy.deepcopy(load_settings()["motion"])
        self.distance = {
            "mode": "ground",
            "camera_height_m": 1,
            "camera_downward_tilt_degrees": 0,
            "focal_calibration": {"focal_px": 500, "image_width": 640},
            "maximum_distance_m": 30,
        }

    def test_approaching_object_speed_and_front_zone_arrival(self):
        tracker = StationaryCameraTracker(self.motion, self.distance)
        ids = []
        for index in range(6):
            timestamp = index * 0.2
            forward = 5 - timestamp
            bottom = 240 + 500 / forward
            item = tracker.update(
                [detection([300, bottom - 100, 340, bottom])], timestamp, 640, 480
            )[0]
            ids.append(item["motion"]["track_id"])
        result = item["motion"]
        self.assertEqual(len(set(ids)), 1)
        self.assertAlmostEqual(result["relative_velocity_mps"]["forward"], -1, places=2)
        self.assertAlmostEqual(result["object_speed_mps"], 1, places=2)
        self.assertAlmostEqual(result["closing_speed_mps"], 1, places=2)
        self.assertAlmostEqual(result["front_zone_entry_s"], 2, places=2)
        self.assertIsNone(result["time_to_contact_s"])  # Front zone is not collision.

    def test_image_speed_available_without_metric_scale(self):
        tracker = StationaryCameraTracker(self.motion, {"mode": "off"})
        for index in range(6):
            timestamp = index * 0.2
            item = tracker.update(
                [detection([100 + 10 * timestamp, 150, 140 + 10 * timestamp, 250])],
                timestamp,
                640,
                480,
            )[0]
        self.assertAlmostEqual(item["motion"]["image_speed_px_s"], 10)
        self.assertIsNone(item["motion"]["object_speed_mps"])
        self.assertIsNone(item["motion"]["front_zone_entry_s"])

    def test_no_speed_claim_during_warmup(self):
        item = StationaryCameraTracker(self.motion, self.distance).update(
            [detection([300, 200, 340, 340])], 0, 640, 480
        )[0]
        self.assertEqual(item["motion"]["status"], "collecting_history")
        self.assertIsNone(item["motion"]["object_speed_mps"])

    def test_lost_track_gets_new_identity(self):
        tracker = StationaryCameraTracker(self.motion, self.distance)
        first = tracker.update([detection([300, 200, 340, 340])], 0, 640, 480)[0][
            "motion"
        ]["track_id"]
        second = tracker.update([detection([300, 200, 340, 340])], 2, 640, 480)[0][
            "motion"
        ]["track_id"]
        self.assertNotEqual(first, second)

    def test_stationary_image_does_not_predict_future_approach(self):
        tracker = StationaryCameraTracker(self.motion, self.distance)
        for index in range(6):
            item = tracker.update(
                [detection([300, 200, 340, 340])], index * 0.2, 640, 480
            )[0]
        self.assertAlmostEqual(item["motion"]["object_speed_mps"], 0)
        self.assertEqual(item["motion"]["status"], "approximately_stationary")
        self.assertIsNone(item["motion"]["front_zone_entry_s"])

    def test_cropped_observation_clears_metric_prediction(self):
        tracker = StationaryCameraTracker(self.motion, self.distance)
        for index in range(6):
            tracker.update([detection([300, 200, 340, 340])], index * 0.2, 640, 480)
        item = tracker.update([detection([300, 200, 340, 480])], 1.2, 640, 480)[0]
        self.assertIsNone(item["motion"]["object_speed_mps"])

    def test_crossing_zone_and_missing_zone(self):
        zone = self.motion["front_zone"]
        self.assertEqual(front_zone_interval((1, 2), (0, -1), zone), (1.5, 2.5))
        self.assertEqual(front_zone_interval((3, 2), (-1, 0), zone), (None, None))
        self.assertEqual(front_zone_interval((4, 0), (1, 0), zone), (None, None))

    def test_repeated_capture_timestamps_are_rejected(self):
        tracker = StationaryCameraTracker(self.motion, self.distance)
        tracker.update([], 1, 640, 480)
        with self.assertRaises(ValueError):
            tracker.update([], 1, 640, 480)

    def test_size_based_speed_is_optional(self):
        settings = {
            "mode": "size",
            "horizontal_fov_degrees": 65,
            "assumed_object_heights_m": {"person": 1.7},
        }
        self.motion["allow_size_based_speed"] = False
        tracker = StationaryCameraTracker(self.motion, settings)
        for index in range(6):
            item = tracker.update(
                [detection([300, 150, 340, 350])], index * 0.2, 640, 480
            )[0]
        self.assertIsNone(item["motion"]["object_speed_mps"])
