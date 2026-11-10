import unittest
from vision_detection.perception_fusion import merge_people


class FusionTests(unittest.TestCase):
    def test_duplicate_human_is_one_observation_but_neighbour_is_preserved(self):
        def item(label, box, confidence):
            return dict(label=label, bbox_xyxy=box, confidence=confidence)

        detections = [
            item("person", [0, 0, 20, 100], 0.5),
            item("person", [1, 1, 21, 101], 0.8),
            item("person", [30, 0, 50, 100], 0.7),
            item("tree", [0, 0, 20, 100], 0.9),
        ]
        result = merge_people(detections)
        self.assertEqual(len(result), 3)
        self.assertNotIn(detections[0], result)
        self.assertIn(detections[3], result)

    def test_stronger_human_suppresses_near_identical_weak_conflicting_name(self):
        person = dict(label="person", bbox_xyxy=[0, 0, 50, 100], confidence=0.85)
        artifact = dict(label="fire hydrant", bbox_xyxy=[1, 1, 51, 101], confidence=0.4)
        self.assertEqual(merge_people([artifact, person]), [person])
