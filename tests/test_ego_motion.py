"""A moving camera must not report its own translation as object motion."""

import copy
import unittest
from vision_detection.ego_motion import StraightLineTracker
from vision_detection.settings import load_settings


class EgoMotionTests(unittest.TestCase):
    def track(self, object_velocity):
        settings = copy.deepcopy(load_settings())
        settings["motion"]["allow_size_based_speed"] = False
        distance = dict(
            mode="ground",
            camera_height_m=1,
            camera_downward_tilt_degrees=0,
            focal_calibration=dict(focal_px=500, image_width=640),
            maximum_distance_m=30,
        )
        tracker = StraightLineTracker(settings["motion"], distance)
        for index in range(6):
            timestamp = index * 0.2
            travel = timestamp * 0.5
            relative_forward = 5 + object_velocity * timestamp - travel
            bottom = 240 + 500 / relative_forward
            item = dict(label="person", bbox_xyxy=[300, bottom - 100, 340, bottom])
            tracker.update([item], timestamp, 640, 480, travel, 0.5)
        return item["motion"]

    def test_stationary_object_stays_stationary_while_robot_approaches(self):
        motion = self.track(0)
        self.assertLess(motion["object_speed_mps"], 0.03)
        self.assertAlmostEqual(
            motion["relative_velocity_mps"]["forward"], -0.5, delta=0.03
        )
        self.assertAlmostEqual(motion["front_zone_entry_s"], 5, delta=0.3)

    def test_approaching_object_speed_and_relative_speed_are_different(self):
        motion = self.track(-0.5)
        self.assertAlmostEqual(motion["object_speed_mps"], 0.5, delta=0.03)
        self.assertAlmostEqual(
            motion["relative_velocity_mps"]["forward"], -1, delta=0.03
        )
        self.assertIsNone(motion["time_to_contact_s"])
