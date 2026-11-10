import unittest
from vision_detection.detection_continuity import DetectionContinuity


class ContinuityTests(unittest.TestCase):
    def test_fresh_reassociated_box_supersedes_old_prediction(self):
        continuity=DetectionContinuity()
        first=dict(label='person',bbox_xyxy=[10,20,40,100],motion=dict(track_id=1))
        continuity.update([first],0,640,480)
        second=dict(label='person',bbox_xyxy=[11,20,41,100],motion=dict(track_id=2))
        result=continuity.update([second],.1,640,480)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0]['motion']['track_id'],2)
    def test_prediction_is_labelled_not_measured_and_expires(self):
        continuity = DetectionContinuity(0.35)
        observed = dict(
            label="person",
            bbox_xyxy=[10, 20, 40, 100],
            confidence=0.8,
            distance_m=3,
            motion=dict(track_id=1, image_velocity_px_s=dict(right=10, down=0)),
        )
        self.assertEqual(
            continuity.update([observed], 1, 640, 480)[0]["observation_status"],
            "observed",
        )
        predicted = continuity.update([], 1.2, 640, 480)[0]
        self.assertEqual(predicted["observation_status"], "prediction_only")
        self.assertIsNone(predicted["distance_m"])
        self.assertIsNone(predicted["motion"]["object_speed_mps"])
        self.assertAlmostEqual(predicted["bbox_xyxy"][0], 12)
        self.assertEqual(continuity.update([], 1.4, 640, 480), [])

    def test_clear_removes_predictions_when_camera_is_blocked(self):
        continuity = DetectionContinuity()
        continuity.update(
            [
                dict(
                    label="person", bbox_xyxy=[10, 20, 40, 100], motion=dict(track_id=1)
                )
            ],
            0,
            640,
            480,
        )
        continuity.clear()
        self.assertEqual(continuity.update([], 0.1, 640, 480), [])
