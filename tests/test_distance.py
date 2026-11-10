import math
import unittest
from vision_detection.distance import estimate_distance_m


class DistanceTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "mode": "size",
            "focal_calibration": {"focal_px": 500, "image_width": 640},
            "assumed_object_heights_m": {"person": 1.7},
        }

    def test_known_person_projection(self):
        # A centred 1.7 m object at 5 m projects to 170 pixels at f=500.
        value, _ = estimate_distance_m(
            [290, 155, 350, 325], "person", 640, 480, self.config
        )
        self.assertAlmostEqual(value, 5)

    def test_resolution_scaling(self):
        value, _ = estimate_distance_m(
            [580, 310, 700, 650], "person", 1280, 960, self.config
        )
        self.assertAlmostEqual(value, 5)

    def test_tree_has_no_invented_size(self):
        value, _ = estimate_distance_m(
            [290, 155, 350, 325], "tree", 640, 480, self.config
        )
        self.assertIsNone(value)

    def test_new_obstacles_do_not_receive_guessed_sizes(self):
        for label in ("dog", "bollard", "truck", "rock", "traffic cone"):
            with self.subTest(label=label):
                value, _ = estimate_distance_m(
                    [300, 150, 340, 340], label, 640, 480, self.config
                )
                self.assertIsNone(value)

    def test_ground_geometry_can_be_used_for_any_visible_base(self):
        self.config.update(
            mode="ground", camera_height_m=1, camera_downward_tilt_degrees=0
        )
        for label in ("dog", "bollard", "truck", "rock", "traffic cone"):
            with self.subTest(label=label):
                value, _ = estimate_distance_m(
                    [300, 150, 340, 340], label, 640, 480, self.config
                )
                self.assertAlmostEqual(value, 5)

    def test_cropped_person_has_no_size_estimate(self):
        value, _ = estimate_distance_m(
            [290, 0, 350, 480], "person", 640, 480, self.config
        )
        self.assertIsNone(value)

    def test_ground_intersection(self):
        self.config.update(
            mode="ground", camera_height_m=1, camera_downward_tilt_degrees=0
        )
        # Base 100 px below horizon -> ground range 500/100 = 5 metres.
        value, _ = estimate_distance_m(
            [300, 150, 340, 340], "tree", 640, 480, self.config
        )
        self.assertAlmostEqual(value, 5)

    def test_ground_range_accepts_cropped_head_but_never_cropped_feet(self):
        self.config.update(
            mode="ground", camera_height_m=1, camera_downward_tilt_degrees=0
        )
        self.assertEqual(
            estimate_distance_m([300, 0, 340, 340], "person", 640, 480, self.config)[0],
            5,
        )
        self.assertIsNone(
            estimate_distance_m([300, 0, 340, 480], "person", 640, 480, self.config)[0]
        )

    def test_ground_horizon_and_missing_calibration(self):
        self.config["mode"] = "ground"
        self.assertIsNone(
            estimate_distance_m([300, 100, 340, 240], "tree", 640, 480, self.config)[0]
        )
        self.config.update(camera_height_m=1, camera_downward_tilt_degrees=0)
        self.assertIsNone(
            estimate_distance_m([300, 100, 340, 240], "tree", 640, 480, self.config)[0]
        )


if __name__ == "__main__":
    unittest.main()
