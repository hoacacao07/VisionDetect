"""Camera adapters: OpenCV devices/streams and optional Raspberry Pi CSI input.

All adapters return unmirrored OpenCV BGR images. The detector always uses the
actual image dimensions. Optics and mounting calibration live in distance settings.
"""

import os
import threading
import time
import cv2


class OpenCVCameraInput:
    """Open a USB/built-in camera or a URL/device path with an explicit backend."""

    def __init__(self, config):
        """Open the configured source and request resolution/rate from its driver."""
        source = config.get("source")
        source = config["index"] if source is None else source
        backends = {
            "auto": cv2.CAP_ANY,
            "dshow": cv2.CAP_DSHOW,
            "msmf": cv2.CAP_MSMF,
            "v4l2": cv2.CAP_V4L2,
            "ffmpeg": cv2.CAP_FFMPEG,
            "gstreamer": cv2.CAP_GSTREAMER,
        }
        backend = config.get("backend", "auto")
        # Windows USB devices often work best with DirectShow. Fall back to
        # OpenCV's default when that backend cannot open this particular device.
        candidates = (
            [cv2.CAP_DSHOW, cv2.CAP_ANY]
            if backend == "auto" and os.name == "nt" and isinstance(source, int)
            else [backends[backend]]
        )
        self.capture = None
        for api in candidates:
            capture = cv2.VideoCapture(source, api)
            if capture.isOpened():
                self.capture = capture
                break
            capture.release()
        if self.capture is None:
            # Do not include a URL in errors: network-camera URLs can contain passwords.
            raise RuntimeError(
                "Cannot open camera. Check camera source/backend, permissions and other camera apps."
            )
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["width"])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["height"])
        if config.get("fps") is not None:
            self.capture.set(cv2.CAP_PROP_FPS, config["fps"])
        # These are requests; cameras/drivers may choose other values.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        """Return success and the next BGR image supplied by the driver."""
        return self.capture.read()

    def close(self):
        """Release this device or stream."""
        self.capture.release()


class Picamera2Input:
    """Optional CSI adapter; import only on a Pi with Picamera2 installed.

    API/color format credited to Raspberry Pi's Picamera2 manual and source:
    https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py
    RGB888 is byte-ordered BGR in capture_array(), matching OpenCV.
    """

    def __init__(self, config):
        """Configure the optional CSI library; release it if startup fails."""
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(
                "Picamera2 is required for camera.driver=picamera2; see docs/RASPBERRY_PI.md."
            ) from error
        self.camera = Picamera2(camera_num=config["index"])
        try:
            controls = {} if config.get("fps") is None else {"FrameRate": config["fps"]}
            configuration = self.camera.create_video_configuration(
                main={"size": (config["width"], config["height"]), "format": "RGB888"},
                controls=controls,
                buffer_count=4,
            )
            self.camera.configure(configuration)
            self.camera.start()
        except Exception:
            self.camera.close()
            raise

    def read(self):
        """Read the main stream as a three-channel BGR array."""
        frame = self.camera.capture_array("main")
        return frame is not None, frame

    def close(self):
        """Stop capture and release the CSI camera."""
        self.camera.stop()
        self.camera.close()


class LatestFrameCamera:
    """Keep only the newest frame; no backlog of old observations.

    Timestamp is host receipt time, not sensor exposure time. Network/driver
    buffering can add unmeasured delay. Future physical control needs measured
    end-to-end latency and synchronized sensor timestamps (NAV-INPUT).
    """

    def __init__(self, config):
        """Choose a hardware adapter and start a background capture thread."""
        adapter = (
            Picamera2Input
            if config.get("driver", "opencv") == "picamera2"
            else OpenCVCameraInput
        )
        self.input = adapter(config)
        self.maximum_read_failures = config.get("maximum_read_failures", 30)
        self.frame = None
        self.frame_sequence = 0
        self.capture_time_monotonic = 0.0
        self.lock = threading.Lock()
        self.stop_capture_event = threading.Event()
        self.capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        self.capture_thread.start()

    def capture_frames(self):
        """Replace the cached frame; signal failure and always release hardware."""
        failures = 0
        try:
            while not self.stop_capture_event.is_set():
                success, frame = self.input.read()
                if success and frame is not None:
                    with self.lock:
                        self.frame = frame
                        self.frame_sequence += 1
                        self.capture_time_monotonic = time.monotonic()
                    failures = 0
                else:
                    failures += 1
                    if failures >= self.maximum_read_failures:
                        self.stop_capture_event.set()
                    self.stop_capture_event.wait(0.02)
        except Exception:
            # Hardware errors must wake the foreground loop, not leave a stale image.
            self.stop_capture_event.set()
        finally:
            self.input.close()

    def get_latest_frame(self):
        """Return an image copy, sequence number and host receipt timestamp."""
        with self.lock:
            return (
                None if self.frame is None else self.frame.copy(),
                self.frame_sequence,
                self.capture_time_monotonic,
            )

    def close(self):
        """Request shutdown; the daemon prevents a blocked driver holding Python open."""
        self.stop_capture_event.set()
        self.capture_thread.join(timeout=3)
