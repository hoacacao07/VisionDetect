"""Webots camera adapter and independent ground-truth evaluation harness.

Detector input: rendered RGB image ONLY. Simulator positions are used below
for scenario scripting and evaluation, never as detector/tracker measurements.
Simulation timestamps drive tracking, so pause/slow rendering does not fake speed.
"""

from pathlib import Path
import copy
import json
import math
import sys
import time
from datetime import datetime, timezone

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "src"))
from controller import Supervisor
import cv2
import numpy as np
from vision_detection.settings import load_settings
from vision_detection.detector import ObjectDetector
from vision_detection.tracking import StationaryCameraTracker
from vision_detection.display import annotate_frame, show_preview
from vision_detection.camera_health import CameraHealthMonitor, draw_camera_alert
from vision_detection.server import DetectionSnapshot, start_local_server


def scenario(simulation_time):
    """Independent scene script: name, local time, position, velocity, cover."""
    phase = simulation_time % 40
    if phase < 12:
        return "approaching", phase, (6 - 0.3 * phase, 0), (-0.3, 0), False
    if phase < 24:
        local = phase - 12
        return "crossing", local, (3.3, -2.4 + 0.4 * local), (0, 0.4), False
    if phase < 36:
        return "stationary", phase - 24, (4, 0), (0, 0), False
    return "covered_camera", phase - 36, (4, 0), (0, 0), True


def main():
    """Run the fixed-camera recognition scenario and save observations for offline analysis."""
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    camera = robot.getDevice("camera")
    camera.enable(step)
    person = robot.getFromDef("TEST_PERSON").getField("translation")
    cover = robot.getFromDef("CAMERA_COVER").getField("translation")
    settings = copy.deepcopy(load_settings())
    # Known sensor intrinsics/mount are legitimate calibration, not object truth.
    width, height = camera.getWidth(), camera.getHeight()
    settings["distance"].update(
        mode="ground",
        camera_height_m=1.0,
        camera_downward_tilt_degrees=math.degrees(0.10),
        focal_calibration={
            "focal_px": width / (2 * math.tan(camera.getFov() / 2)),
            "image_width": width,
        },
    )
    settings["motion"]["allow_size_based_speed"] = False
    settings["motion"]["front_zone"].update(near_m=1.0, far_m=3.0, width_m=1.5)
    settings["detection"]["image_size"] = 640
    detector = ObjectDetector(settings["detection"], settings["distance"])
    tracker = StationaryCameraTracker(settings["motion"], settings["distance"])
    monitor = CameraHealthMonitor()
    snapshot = DetectionSnapshot()
    server = start_local_server(snapshot, 8880)
    output = PROJECT / "simulation" / "results"
    output.mkdir(exist_ok=True)
    log = (output / "latest_run.jsonl").open("w", encoding="utf-8")
    samples = []
    frame_id = 0
    previous_phase = None
    print(
        "VISION SIMULATION READY: camera images -> real detector; truth is evaluation only.",
        flush=True,
    )
    try:
        while True:
            # Scene control occurs BEFORE stepping, so the image and reference
            # position below correspond to the same scripted scenario instant.
            expected_time = robot.getTime() + step / 1000
            phase, phase_age, position, velocity, covered = scenario(expected_time)
            person.setSFVec3f([position[0], position[1], 1.27])
            cover.setSFVec3f([0.18 if covered else -100, 0, 1])
            if robot.step(step) == -1:
                break
            sim_time = robot.getTime()
            if phase != previous_phase:
                # Separate experiments start with fresh history; do not treat a
                # scene reset/teleport as real object motion.
                tracker = StationaryCameraTracker(
                    settings["motion"], settings["distance"]
                )
                monitor = CameraHealthMonitor()
                previous_phase = phase
                print(f"SCENARIO: {phase}", flush=True)
            raw = camera.getImage()
            if not raw:
                continue
            frame = (
                np.frombuffer(raw, np.uint8).reshape(height, width, 4)[:, :, :3].copy()
            )
            start = time.perf_counter()
            health = monitor.update(frame, sim_time)
            detections = [] if health["alert"] else detector.detect(frame)
            if health["alert"]:
                tracker.tracks.clear()
            tracker.update(detections, sim_time, width, height)
            elapsed_ms = (time.perf_counter() - start) * 1000
            frame_id += 1
            predicted_people = [
                item for item in detections if item["label"] == "person"
            ]
            predicted = max(
                predicted_people, key=lambda item: item["confidence"], default=None
            )
            truth_range = math.hypot(*position)
            truth_speed = math.hypot(*velocity)
            predicted_speed = (
                predicted["motion"]["object_speed_mps"] if predicted else None
            )
            predicted_range = predicted["distance_m"] if predicted else None
            # Only evaluation code reads ground truth. No recognition/position
            # answers are supplied to ObjectDetector or StationaryCameraTracker.
            evaluation = {
                "scenario": phase,
                "phase_age_s": round(phase_age, 3),
                "truth_position_m": {"forward": position[0], "left": position[1]},
                "truth_speed_mps": truth_speed,
                "truth_range_m": truth_range,
                "person_detected": bool(predicted_people),
                "range_error_m": (
                    predicted_range - truth_range
                    if predicted_range is not None
                    else None
                ),
                "speed_error_mps": (
                    predicted_speed - truth_speed
                    if predicted_speed is not None
                    else None
                ),
                "cover_expected": covered,
                "camera_alert": health["alert"],
            }
            metadata = {
                "schema_version": 1,
                "source": "webots_rgb",
                "status": "running",
                "camera_active": True,
                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                "simulation_time_s": sim_time,
                "frame_id": frame_id,
                "image_width": width,
                "image_height": height,
                "inference_ms": round(elapsed_ms, 1),
                "classes": detector.class_names,
                "model": detector.description,
                "distance_mode": "ground",
                "distance_assumptions": settings["distance"],
                "motion_analysis": "stationary_camera_tracking",
                "motion_assumptions": settings["motion"],
                "camera_health": health,
                "detections": detections,
            }
            preview = draw_camera_alert(
                annotate_frame(frame, detections, elapsed_ms, settings["motion"]),
                health,
            )
            cv2.putText(
                preview,
                f"WEBOTS | {phase} | sim {sim_time:.1f}s",
                (10, 74),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                preview,
                "Ground truth is logged separately for evaluation",
                (10, 94),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
            )
            ok, jpeg = cv2.imencode(".jpg", preview)
            snapshot.publish(metadata, jpeg.tobytes() if ok else None)
            record = {"prediction": metadata, "evaluation_only": evaluation}
            log.write(json.dumps(record, allow_nan=False) + "\n")
            log.flush()
            if phase_age >= 2:
                samples.append(evaluation)
            if frame_id % 15 == 1:
                # Unannotated pixels allow fair offline model comparisons.
                raw_dir = output / "raw_frames"
                raw_dir.mkdir(exist_ok=True)
                cv2.imwrite(str(raw_dir / f"{frame_id:04d}_{phase}.jpg"), frame)
                cv2.imwrite(str(output / "latest_preview.jpg"), preview)
                # Keep one inspectable image per scenario, including failures.
                cv2.imwrite(str(output / f"{phase}_preview.jpg"), preview)
                summary = {
                    "frames_logged": frame_id,
                    "simulation_time_s": round(sim_time, 3),
                    "scenarios": {},
                }
                for name in ("approaching", "crossing", "stationary", "covered_camera"):
                    group = [s for s in samples if s["scenario"] == name]
                    ranges = [
                        abs(s["range_error_m"])
                        for s in group
                        if s["range_error_m"] is not None
                    ]
                    speeds = [
                        abs(s["speed_error_mps"])
                        for s in group
                        if s["speed_error_mps"] is not None
                    ]
                    summary["scenarios"][name] = {
                        "samples": len(group),
                        "person_detection_fraction": (
                            sum(s["person_detected"] for s in group) / len(group)
                            if group
                            else None
                        ),
                        "range_mae_m": sum(ranges) / len(ranges) if ranges else None,
                        "range_samples": len(ranges),
                        "speed_mae_mps": sum(speeds) / len(speeds) if speeds else None,
                        "speed_samples": len(speeds),
                        "camera_alert_fraction": (
                            sum(s["camera_alert"] for s in group) / len(group)
                            if group
                            else None
                        ),
                    }
                (output / "latest_summary.json").write_text(
                    json.dumps(summary, indent=2), encoding="utf-8"
                )
            # A bounded run keeps logs small and leaves Webots open for review.
            # Reset/reload the world to start another independent experiment.
            if sim_time >= 39.8:
                robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                print(
                    "TEST COMPLETE: see simulation/results/latest_summary.json",
                    flush=True,
                )
                break
            if not show_preview(preview):
                robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                break
    finally:
        log.close()
        server.shutdown()
        server.server_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
