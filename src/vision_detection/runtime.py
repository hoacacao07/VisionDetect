"""COORDINATION: camera -> detection/distance -> display/local output.

Follow the numbered stages below. Implementation details live in their own
modules so camera, model, display, and distance can be edited independently.
Future integration map: docs/NAVIGATION_ROADMAP.md. Search NAV- in the source
for the intended extension points; those comments do not enable robot control.
"""

import argparse
import json
import time
from datetime import datetime, timezone
import cv2
from .camera import LatestFrameCamera
from .detector import ObjectDetector
from .display import annotate_frame, show_preview
from .server import DetectionSnapshot, start_local_server
from .settings import PROJECT_DIRECTORY, DEFAULT_SETTINGS_PATH, load_settings
from .tracking import StationaryCameraTracker
from .camera_health import CameraHealthMonitor, draw_camera_alert
from .detection_continuity import DetectionContinuity


def run_detector(settings, frame_limit=0, show_window=True):
    """Run one session and always release hardware/files/port on exit."""
    # 1. PREPARE — load the pretrained model before activating the webcam.
    detector = ObjectDetector(settings["detection"], settings["distance"])
    tracker = StationaryCameraTracker(settings["motion"], settings["distance"])
    continuity = DetectionContinuity()
    health_monitor = CameraHealthMonitor(settings.get("camera_health"))
    snapshot = DetectionSnapshot()
    camera = server = detection_log = None
    try:
        # 2. CONNECT — camera input and local outputs; no motor commands.
        server = start_local_server(snapshot, settings["output"]["api_port"])
        camera = LatestFrameCamera(settings["camera"])
        if settings["output"]["write_detection_log"]:
            results_directory = PROJECT_DIRECTORY / "results"
            results_directory.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            detection_log = (results_directory / f"detections-{timestamp}.jsonl").open(
                "w", encoding="utf-8"
            )
        print(
            f"Viewer: http://127.0.0.1:{settings['output']['api_port']} | Q stops the camera.",
            flush=True,
        )
        print("Distances are estimates. No wheel commands are sent.", flush=True)
        camera_started_at = time.monotonic()
        processed_frames, previous_sequence = 0, -1
        while True:
            # 3. CAPTURE — keep only fresh observations, with no frame backlog.
            frame, sequence, capture_time = camera.get_latest_frame()
            if camera.stop_capture_event.is_set():
                raise RuntimeError("Camera stopped providing frames.")
            if frame is None and time.monotonic() - camera_started_at > settings[
                "camera"
            ].get("startup_timeout_s", 10):
                raise RuntimeError(
                    "No camera images received before camera.startup_timeout_s."
                )
            if frame is not None and time.monotonic() - capture_time > settings[
                "camera"
            ].get("maximum_frame_age_s", 2):
                raise RuntimeError("Camera frames are stale; stopping detection.")
            if frame is None or sequence == previous_sequence:
                time.sleep(0.01)
                continue
            previous_sequence = sequence
            # 4. ANALYZE — recognition plus explicit distance assumptions.
            started_at = time.perf_counter()
            camera_health = health_monitor.update(frame, capture_time)
            if camera_health["alert"]:
                detections = []
                tracker.tracks.clear()  # Do not retain object motion through an obstruction.
                continuity.clear()
            else:
                detections = detector.detect(frame)
            height, width = frame.shape[:2]
            # NAV-EGO: fuse timestamped robot odometry/IMU and camera mounting
            # transforms here BEFORE estimating world-frame object velocity.
            # Passing only current wheel speed will not account for turning.
            detections = tracker.update(detections, capture_time, width, height)
            detections = continuity.update(detections, capture_time, width, height)
            processing_ms = (time.perf_counter() - started_at) * 1000
            processed_frames += 1
            height, width = frame.shape[:2]
            metadata = {
                "schema_version": 1,
                "status": "running",
                "camera_active": True,
                "camera_health": camera_health,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "frame_id": processed_frames,
                "image_width": width,
                "image_height": height,
                "inference_ms": round(processing_ms, 1),
                "distance_is_estimated": True,
                "model": detector.description,
                "classes": detector.class_names,
                "motion_analysis": "stationary_camera_tracking",
                "motion_assumptions": settings["motion"],
                "capture_age_ms": round((time.monotonic() - capture_time) * 1000, 1),
                "mobility_counts": {
                    group: sum(item["motion_group"] == group for item in detections)
                    for group in ("potentially_dynamic", "usually_static")
                },
                "distance_mode": settings["distance"]["mode"],
                "distance_assumptions": settings["distance"],
                "detections": detections,
            }

            # NAV-CONTROL: a separate future controller consumes this structured
            # metadata, health state and capture age. It should own path/stop/
            # steering decisions; display text must never be its data source.
            # No controller or motor command is currently called here.

            # 5. PRESENT — annotated preview, local JSON, optional metadata log.
            preview = annotate_frame(
                frame, detections, processing_ms, settings["motion"]
            )
            preview = draw_camera_alert(preview, camera_health)
            success, encoded_jpeg = cv2.imencode(
                ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )
            snapshot.publish(metadata, encoded_jpeg.tobytes() if success else None)
            if detection_log:
                detection_log.write(json.dumps(metadata, allow_nan=False) + "\n")
                detection_log.flush()
            if processed_frames % 30 == 1:
                labels = ", ".join(item["label"] for item in detections) or "none"
                print(
                    f"Frame {processed_frames} | {processing_ms:.0f} ms | objects: {labels}",
                    flush=True,
                )
            if show_window and not show_preview(preview):
                break
            if frame_limit and processed_frames >= frame_limit:
                break
    finally:
        # 6. CLEAN UP — also runs after an error or Ctrl+C.
        if camera:
            camera.close()
        if detection_log:
            detection_log.close()
        snapshot.publish(
            {
                "schema_version": 1,
                "status": "stopped",
                "camera_active": False,
                "detections": [],
            },
            None,
        )
        if server:
            server.shutdown()
            server.server_close()
        cv2.destroyAllWindows()


def main():
    """Entry point used by app.py, the launcher, and VS Code tasks."""
    parser = argparse.ArgumentParser(description="Camera object detection (OpenCV or Raspberry Pi CSI)")
    parser.add_argument("--config", default=str(DEFAULT_SETTINGS_PATH))
    parser.add_argument(
        "--frames", type=int, default=0, help="Stop after N frames; 0 runs continuously"
    )
    parser.add_argument("--no-window", action="store_true", help="Use browser/API only")
    arguments = parser.parse_args()
    try:
        run_detector(
            load_settings(arguments.config), arguments.frames, not arguments.no_window
        )
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"Detector stopped: {error}", flush=True)
        raise SystemExit(1) from error
