import unittest
from vision_detection.sensor_timing import DelayedResult, limited_speed


class TimingTests(unittest.TestCase):
    def test_result_is_not_visible_before_processing_delay(self):
        pipeline = DelayedResult()
        pipeline.submit(10, 0.25, {"object": "person"})
        self.assertIsNone(pipeline.poll(10.2))
        with self.assertRaises(RuntimeError):
            pipeline.submit(10.2, 0.25, {})
        capture, payload = pipeline.poll(10.3)
        self.assertEqual(capture, 10)
        self.assertEqual(payload["object"], "person")
        self.assertIsNone(pipeline.poll(10.4))

    def test_stop_command_brakes_over_time_instead_of_instantly(self):
        speed = limited_speed(0.35, 0, 0.1, 0.3, 0.6)
        self.assertAlmostEqual(speed, 0.29)
        for _ in range(5):
            speed = limited_speed(speed, 0, 0.1, 0.3, 0.6)
        self.assertEqual(speed, 0)

    def test_acceleration_is_limited_and_never_overshoots(self):
        self.assertAlmostEqual(limited_speed(0, 0.35, 0.1, 0.3, 0.6), 0.03)
        self.assertEqual(limited_speed(0.34, 0.35, 0.1, 0.3, 0.6), 0.35)
