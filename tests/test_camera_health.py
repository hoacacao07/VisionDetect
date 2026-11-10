import unittest
import numpy as np
from vision_detection.camera_health import CameraHealthMonitor, draw_camera_alert


class CameraHealthTests(unittest.TestCase):
    def setUp(self):
        self.monitor = CameraHealthMonitor()
        self.black = np.zeros((120, 160, 3), dtype=np.uint8)
        self.textured = np.random.default_rng(7).integers(
            30, 220, (120, 160, 3), dtype=np.uint8
        )

    def test_brief_dark_frame_does_not_alert(self):
        self.assertFalse(self.monitor.update(self.black, 0)["alert"])
        self.assertFalse(self.monitor.update(self.black, 0.5)["alert"])

    def test_sustained_black_frame_alerts(self):
        self.monitor.update(self.black, 0)
        health = self.monitor.update(self.black, 1.1)
        self.assertTrue(health["alert"])
        self.assertIn("ALERT", health["message"])
        self.assertIn("TOO DARK", health["message"])

    def test_texture_without_objects_is_not_blocked(self):
        self.monitor.update(self.textured, 0)
        self.assertFalse(self.monitor.update(self.textured, 2)["alert"])

    def test_featureless_cover_is_qualified_not_proven(self):
        blank = np.full_like(self.black, 180)
        self.monitor.update(blank, 0)
        health = self.monitor.update(blank, 1.1)
        self.assertTrue(health["alert"])
        self.assertIn("MAY BE", health["message"])

    def test_alert_recovers_after_stable_visible_view(self):
        self.monitor.update(self.black, 0)
        self.monitor.update(self.black, 1.1)
        self.assertTrue(self.monitor.update(self.textured, 2)["alert"])
        self.assertFalse(self.monitor.update(self.textured, 2.4)["alert"])

    def test_warning_is_drawn_on_preview(self):
        self.monitor.update(self.black, 0)
        health = self.monitor.update(self.black, 1.1)
        image = draw_camera_alert(self.black.copy(), health)
        self.assertGreater(int(image.max()), 0)
