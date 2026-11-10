"""Exercise camera selection and CSI image format without opening hardware."""

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from vision_detection.camera import OpenCVCameraInput, Picamera2Input


class CameraAdapterTests(unittest.TestCase):
    """Drivers vary; test selection, fallback and image contract independently."""

    def test_windows_auto_releases_failed_backend_and_falls_back(self):
        failed, working = MagicMock(), MagicMock()
        failed.isOpened.return_value = False
        working.isOpened.return_value = True
        with patch("vision_detection.camera.os.name", "nt"), patch(
            "vision_detection.camera.cv2.VideoCapture", side_effect=[failed, working]
        ) as open_camera:
            adapter = OpenCVCameraInput(dict(index=2, width=640, height=480))
            self.assertEqual(open_camera.call_count, 2)
            failed.release.assert_called_once()
            adapter.close()
            working.release.assert_called_once()

    def test_stream_source_does_not_use_windows_usb_backend(self):
        with patch("vision_detection.camera.os.name", "nt"), patch(
            "vision_detection.camera.cv2.VideoCapture"
        ) as open_camera:
            source = "rtsp://example.invalid/video"
            adapter = OpenCVCameraInput(
                dict(index=0, source=source, width=640, height=480)
            )
            self.assertEqual(open_camera.call_args.args, (source, 0))
            adapter.close()

    def test_picamera_uses_opencv_byte_order_and_closes(self):
        camera = MagicMock()
        factory = MagicMock(return_value=camera)
        with patch.dict(sys.modules, {"picamera2": SimpleNamespace(Picamera2=factory)}):
            adapter = Picamera2Input(dict(index=1, width=960, height=540, fps=30))
            factory.assert_called_once_with(camera_num=1)
            main = camera.create_video_configuration.call_args.kwargs["main"]
            self.assertEqual(main, {"size": (960, 540), "format": "RGB888"})
            adapter.read()
            camera.capture_array.assert_called_once_with("main")
            adapter.close()
            camera.stop.assert_called_once()
            camera.close.assert_called_once()
