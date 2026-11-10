"""Check motor decisions at perception failures and stop/resume boundaries."""

import json
import unittest
from vision_detection.longitudinal_control import StopGoController
from vision_detection.settings import PROJECT_DIRECTORY


class StopGoTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(
            (PROJECT_DIRECTORY / "simulation/navigation_settings.json").read_text()
        )
        self.distance = dict(
            mode="ground",
            camera_height_m=1,
            camera_downward_tilt_degrees=0,
            focal_calibration=dict(focal_px=500, image_width=640),
            maximum_distance_m=30,
        )
        self.policy = StopGoController(self.config, self.distance)

    def decide(self, t, ranges=(6, 6, 6), health=None, detections=None):
        return self.policy.decide(
            t, health or {}, detections or [], ranges, 640, 480, 0.35
        )

    def moving(self):
        self.decide(0)
        self.assertGreater(self.decide(1.1)["target_speed_mps"], 0)

    def test_unlabelled_range_obstacle_stops_and_requires_clear_hold_to_resume(self):
        self.moving()
        self.assertEqual(self.decide(2, (1, 6, 6))["state"], "STOP")
        self.assertEqual(self.decide(3, (1.6, 6, 6))["state"], "STOP")
        self.assertEqual(self.decide(4)["state"], "STOP")
        self.assertEqual(self.decide(4.5)["state"], "STOP")
        self.assertEqual(self.decide(5.1)["state"], "MOVE")

    def test_camera_alert_or_missing_range_stops_even_with_no_labels(self):
        self.moving()
        self.assertEqual(
            self.decide(2, health={"alert": True})["reason"], "camera_blocked"
        )
        self.assertEqual(self.decide(3, (float("nan"), 6, 6))["target_speed_mps"], 0)

    def test_cropped_central_detection_is_not_treated_as_clear_space(self):
        self.moving()
        item = dict(label="person", bbox_xyxy=[290, 1, 350, 479])
        self.assertEqual(
            self.decide(2, detections=[item])["reason"], "central_object_range_unknown"
        )

    def test_visibility_debounce_also_stops_before_confirmed_alert(self):
        self.moving()
        self.assertEqual(
            self.decide(2, health={"status": "checking_visibility", "alert": False})[
                "target_speed_mps"
            ],
            0,
        )

    def test_slowing_before_stop_threshold(self):
        self.moving()
        decision = self.decide(2, (2.2, 6, 6))
        self.assertEqual(decision["state"], "SLOW")
        self.assertGreater(decision["target_speed_mps"], 0)
        self.assertLess(decision["target_speed_mps"], self.config["cruise_speed_mps"])

    def test_visible_camera_obstacle_can_stop_without_near_range_return(self):
        self.moving()
        # Flat-ground contact at 1.5m, fully within the camera image.
        bottom = 240 + 500 / 1.5
        # Use a shorter focal length here to keep the nearby contact visible.
        self.distance["focal_calibration"]["focal_px"] = 250
        bottom = 240 + 250 / 1.5
        item = dict(label="person", bbox_xyxy=[280, 100, 360, bottom])
        decision = self.decide(2, detections=[item])
        self.assertEqual(decision["target_speed_mps"], 0)
        self.assertEqual(decision["limiting_source"], "camera_ground_estimate")
