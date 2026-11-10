"""Drive a Webots robot straight using RGB observations and front range sensors.

Policy inputs: camera image, health, encoder travel, measured front ranges.
Supervisor positions are restricted to scene scripting and evaluation below.
No turning commands, route planner, physical contact-time claim or real motors.
"""

from pathlib import Path
from datetime import datetime, timezone
from collections import deque
import copy
import json
import math
import random
import os
import sys
import time

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT / "src"))
from controller import Supervisor
import cv2
import numpy as np
from vision_detection.settings import load_settings, load_navigation_settings
from vision_detection.detector import ObjectDetector
from vision_detection.ego_motion import StraightLineTracker
from vision_detection.longitudinal_control import StopGoController
from vision_detection.camera_health import CameraHealthMonitor, draw_camera_alert
from vision_detection.display import annotate_frame, show_preview
from vision_detection.server import DetectionSnapshot, start_local_server
from vision_detection.navigation_display import append_navigation_panel
from vision_detection.sensor_timing import DelayedResult, limited_speed
from vision_detection.detection_continuity import DetectionContinuity


def scene_at(timestamp):
    """Continuous kinematic actors; scene descriptions never reach the policy."""
    if timestamp < 10:
        person = (4, 3 - 0.3 * timestamp)
        heading, speed, phase = -math.pi / 2, 0.3, "person_enters_path"
    elif timestamp < 22:
        person = (4, 0)
        heading, speed, phase = -math.pi, 0, "person_waits"
    elif timestamp < 32:
        person = (4, 0.4 * (timestamp - 22))
        heading, speed, phase = math.pi / 2, 0.4, "person_leaves_path"
    else:
        person = (4, 4)
        heading, speed, phase = math.pi / 2, 0, "stationary_box_ahead"
    box = (7, max(0, min(3, 0.75 * (timestamp - 50))))
    if 50 <= timestamp < 55:
        phase = "box_leaves_path"
    covered = 55 <= timestamp < 60
    if covered:
        phase = "camera_covered"
    elif timestamp >= 60:
        phase = "clear_finish"
    return phase, person, box, covered, heading, speed


def animate_person(fields, timestamp, speed):
    """Simple gait, not biomechanics; sinusoidal angles are test assumptions.

    Field-control approach: Cyberbotics pedestrian.py, Apache-2.0;
    https://github.com/cyberbotics/webots/blob/R2025a/projects/humans/pedestrian/controllers/pedestrian/pedestrian.py
    No upstream gait tables are copied here.
    """
    swing = math.sin(2 * math.pi * 0.7 * timestamp) if speed > 0 else 0
    angles = dict(
        leftLegAngle=-0.15 + 0.35 * swing,
        rightLegAngle=-0.15 - 0.35 * swing,
        leftLowerLegAngle=0.2 + 0.3 * max(0, -swing),
        rightLowerLegAngle=0.2 + 0.3 * max(0, swing),
        leftArmAngle=-0.3 * swing,
        rightArmAngle=0.3 * swing,
        leftLowerArmAngle=-0.15,
        rightLowerArmAngle=-0.15,
    )
    for name, value in angles.items():
        if fields.get(name) is not None:
            fields[name].setSFFloat(value)


def main():
    """Wire simulated sensors, perception, policy and motors; keep evaluation separate."""
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    dt = step / 1000
    camera = robot.getDevice("camera")
    camera.enable(64)
    width, height = camera.getWidth(), camera.getHeight()
    settings = copy.deepcopy(load_settings())
    realism = json.loads((PROJECT / "simulation/realism_settings.json").read_text())
    if os.environ.get("VISION_SIM_STRESS") == "1":
        realism.update(
            profile="delay_dropout_stress",
            additional_processing_delay_s=realism[
                "stress_additional_processing_delay_s"
            ],
            camera_dropout_probability=0.04,
            range_dropout_probability=0.08,
            random_seed=73,
        )
    rng = random.Random(realism["random_seed"])
    pipeline = DelayedResult()
    camera_queue = deque(maxlen=8)
    config = load_navigation_settings()
    settings["distance"].update(
        mode="ground",
        camera_height_m=1.0,
        camera_downward_tilt_degrees=math.degrees(0.1),
        focal_calibration=dict(
            focal_px=width / (2 * math.tan(camera.getFov() / 2)), image_width=width
        ),
    )
    settings["motion"]["allow_size_based_speed"] = False
    settings["motion"].update(
        maximum_velocity_gap_s=0.65, maximum_track_gap_s=1.0, velocity_window_s=2.0
    )
    # The webcam settings on disk stay stationary; this controller alone
    # supplies encoder compensation and publishes the moving-camera assumption.
    settings["motion"]["camera_stationary"] = False
    settings["motion"]["front_zone"].update(near_m=1, far_m=3, width_m=1.5)
    # Follow the editable laptop inference size; sensor resolution/FOV remain
    # separate. Smaller model inputs must be rechecked for distant obstacles.
    detector = ObjectDetector(settings["detection"], settings["distance"])
    tracker = StraightLineTracker(settings["motion"], settings["distance"])
    continuity = DetectionContinuity()
    health_monitor = CameraHealthMonitor()
    policy = StopGoController(config, settings["distance"])
    wheel_names = ["front left", "front right", "rear left", "rear right"]
    motors = [robot.getDevice(name + " motor") for name in wheel_names]
    encoders = [robot.getDevice(name + " encoder") for name in wheel_names]
    ranges = [
        robot.getDevice("front " + name + " range")
        for name in ("left", "center", "right")
    ]
    for motor in motors:
        motor.setPosition(float("inf"))
        motor.setVelocity(0)
    for sensor in encoders + ranges:
        sensor.enable(step)
    # Supervisor access below is for placing test obstacles and scoring only.
    body = robot.getFromDef("DRIVE_ROBOT")
    person_node = robot.getFromDef("DRIVE_PERSON")
    person = person_node.getField("translation")
    person_rotation = person_node.getField("rotation")
    joint_fields = {
        name: person_node.getField(name)
        for name in (
            "leftLegAngle",
            "rightLegAngle",
            "leftLowerLegAngle",
            "rightLowerLegAngle",
            "leftArmAngle",
            "rightArmAngle",
            "leftLowerArmAngle",
            "rightLowerArmAngle",
        )
    }
    sun = robot.getFromDef("TEST_SUN").getField("intensity")
    heading = -math.pi / 2
    box = robot.getFromDef("DRIVE_BOX").getField("translation")
    cover = robot.getFromDef("DRIVE_COVER").getField("translation")
    output = PROJECT / "simulation/moving_results"
    if realism["profile"] != "measured_laptop":
        output = output / "stress"
    output.mkdir(exist_ok=True, parents=True)
    snapshot = DetectionSnapshot()
    server = start_local_server(snapshot, 8880)
    log = (output / "latest_run.jsonl").open("w", encoding="utf-8")
    previous_travel = 0.0
    motor_command = 0.0
    last_range_poll = -100.0
    range_values = [None] * 3
    range_times = [-100.0] * 3
    completed = None
    health = dict(status="unavailable", alert=False)
    last_inference_ms = 0.0
    completed_count = 0
    previous_phase = None
    frame_id = 0
    rows = []
    print(
        "MOVING TEST READY: RGB + measured ranges + wheel encoders; no turns.",
        flush=True,
    )
    try:
        while True:
            scene_time = robot.getTime() + dt
            phase, person_xy, box_xy, covered, target_heading, walk_speed = scene_at(
                scene_time
            )
            # No teleport resets: identities/history survive phase boundaries.
            delta = (target_heading - heading + math.pi) % (2 * math.pi) - math.pi
            heading += max(-dt, min(dt, delta))
            person.setSFVec3f([*person_xy, 1.27])
            person_rotation.setSFRotation([0, 0, 1, heading])
            if realism["animate_pedestrian"]:
                animate_person(joint_fields, scene_time, walk_speed)
            box.setSFVec3f([*box_xy, 0.4])
            cover.setSFVec3f([0.18 if covered else -100, 0, 0.75])
            if realism["vary_lighting"]:
                sun.setSFFloat(0.8 + 0.45 * math.sin(2 * math.pi * scene_time / 45))
            if robot.step(step) == -1:
                break
            timestamp = robot.getTime()
            if phase != previous_phase:
                previous_phase = phase
                print("SCENARIO:", phase, flush=True)
            travel = (
                sum(sensor.getValue() for sensor in encoders)
                / len(encoders)
                * config["wheel_radius_m"]
            )
            ego_speed = (travel - previous_travel) / dt
            previous_travel = travel
            # A lost packet holds the previous sample briefly. Stale sensor
            # data forces a stop instead of becoming a clear-path reading.
            if timestamp - last_range_poll >= realism["range_update_period_s"] - 1e-6:
                last_range_poll = timestamp
                for index, sensor in enumerate(ranges):
                    if rng.random() >= realism["range_dropout_probability"]:
                        range_values[index] = max(0, min(6, sensor.getValue()))
                        range_times[index] = timestamp
            measured_ranges = [
                (
                    value
                    if value is not None
                    and timestamp - seen <= realism["maximum_range_age_s"]
                    else float("nan")
                )
                for value, seen in zip(range_values, range_times)
            ]
            raw = camera.getImage()
            if not raw:
                for motor in motors:
                    motor.setVelocity(0)
                continue
            frame = (
                np.frombuffer(raw, np.uint8).reshape(height, width, 4)[:, :, :3].copy()
            )
            new_camera_frame = (
                round(timestamp / dt) % 2 == 0
                and rng.random() >= realism["camera_dropout_probability"]
            )
            if new_camera_frame:
                health = health_monitor.update(frame, timestamp)
                # Acquisition proceeds while inference is busy. Only the
                # newest acquired frame is selected, avoiding a stale backlog.
                camera_queue.append(
                    (timestamp, dict(frame=frame, travel=travel, speed=ego_speed))
                )
            ready = pipeline.poll(timestamp)
            if ready is not None:
                capture_time, payload = ready
                tracker.update(
                    payload["detections"],
                    capture_time,
                    width,
                    height,
                    payload["travel"],
                    payload["speed"],
                )
                payload["detections"] = continuity.update(
                    payload["detections"], capture_time, width, height
                )
                completed = dict(capture_time=capture_time, **payload)
                completed_count += 1
            if health["alert"]:
                tracker.tracks.clear()
                continuity.clear()
                completed = None
                pipeline.pending = None
                camera_queue.clear()
            available = [
                sample
                for sample in camera_queue
                if sample[0] + realism["camera_acquisition_delay_s"] <= timestamp
            ]
            if not health["alert"] and available and pipeline.pending is None:
                capture_time, captured = available[-1]
                while camera_queue and camera_queue[0][0] <= capture_time:
                    camera_queue.popleft()
                start = time.perf_counter()
                observations = detector.detect(captured["frame"])
                # Sparse unannotated evidence for speed/quality comparisons.
                if completed_count % 10 == 0:
                    raw_directory = output / "raw_frames"
                    raw_directory.mkdir(exist_ok=True)
                    cv2.imwrite(
                        str(raw_directory / f"{completed_count:04d}_{phase}.jpg"),
                        captured["frame"],
                    )
                last_inference_ms = (time.perf_counter() - start) * 1000
                # Webots waits for controllers. Withhold results while this
                # measured delay elapses in simulation time, so CPU processing
                # does not incorrectly appear instantaneous to a moving robot.
                delay = (
                    timestamp
                    - capture_time
                    + last_inference_ms / 1000
                    + realism["additional_processing_delay_s"]
                )
                pipeline.submit(
                    capture_time,
                    delay,
                    dict(
                        frame=captured["frame"],
                        detections=observations,
                        travel=captured["travel"],
                        speed=captured["speed"],
                        inference_ms=last_inference_ms,
                    ),
                )
            vision_age = timestamp - completed["capture_time"] if completed else None
            detections = completed["detections"] if completed else []
            decision = policy.decide(
                timestamp, health, detections, measured_ranges, width, height, ego_speed
            )
            if (
                vision_age is None or vision_age > realism["maximum_vision_age_s"]
            ) and not health["alert"]:
                decision.update(
                    state="STOP", target_speed_mps=0, reason="waiting_for_fresh_vision"
                )
                policy.stopped = True
                policy.clear_since = None
            finished = timestamp >= config["duration_s"]
            if timestamp >= config["duration_s"] - 1.5:
                # Allow the physical robot to finish braking BEFORE pausing.
                decision.update(
                    state="STOP", target_speed_mps=0, reason="test_complete_braking"
                )
            if finished:
                decision.update(
                    state="STOP", target_speed_mps=0, reason="test_complete"
                )
            motor_command = limited_speed(
                motor_command,
                decision["target_speed_mps"],
                dt,
                realism["maximum_acceleration_mps2"],
                realism["maximum_deceleration_mps2"],
            )
            decision["applied_motor_speed_mps"] = round(motor_command, 3)
            decision["vision_age_s"] = (
                round(vision_age, 3) if vision_age is not None else None
            )
            # All wheels receive the SAME ramped rate. No steering logic.
            for motor in motors:
                motor.setVelocity(motor_command / config["wheel_radius_m"])
            elapsed_ms = last_inference_ms
            frame_id += 1
            metadata = dict(
                schema_version=1,
                source="webots_moving_rgb_range",
                status="running",
                camera_active=True,
                timestamp_utc=datetime.now(timezone.utc).isoformat(),
                simulation_time_s=timestamp,
                frame_id=frame_id,
                image_width=width,
                image_height=height,
                inference_ms=round(elapsed_ms, 1),
                model=detector.description,
                classes=detector.class_names,
                distance_mode="ground",
                distance_assumptions=settings["distance"],
                motion_analysis="straight_line_encoder_compensated",
                motion_assumptions=settings["motion"],
                camera_health=health,
                detections=detections,
                navigation=decision,
                ego_motion=dict(travel_m=travel, speed_mps=ego_speed),
                front_ranges_m=[
                    value if math.isfinite(value) else None for value in measured_ranges
                ],
                realism_assumptions=realism,
                navigation_assumptions=config,
                scenario_version="continuous_v3_65s",
                perception_capture_time_s=(
                    completed["capture_time"] if completed else None
                ),
                completed_inferences=completed_count,
            )
            # ---------------- EVALUATION ONLY ----------------
            # Do not use these reference positions in policy/tracker inputs.
            true_position = body.getPosition()
            truth_gap = None
            if (
                abs(person_xy[1] - true_position[1]) < 0.65
                and person_xy[0] > true_position[0]
            ):
                truth_gap = person_xy[0] - true_position[0] - 0.35 - 0.15
            if (
                abs(box_xy[1] - true_position[1]) < 0.65
                and box_xy[0] > true_position[0]
            ):
                gap = box_xy[0] - true_position[0] - 0.35 - 0.30
                truth_gap = min(truth_gap, gap) if truth_gap is not None else gap
            evaluation = dict(
                phase=phase,
                robot_position_m=true_position,
                true_forward_speed_mps=body.getVelocity()[0],
                truth_front_gap_m=truth_gap,
                odometry_error_m=travel - true_position[0],
                covered_expected=covered,
                clearance_violation=truth_gap is not None
                and truth_gap < realism["stop_clearance_requirement_m"],
                above_ground_contact=any(
                    contact.point[2] > 0.05 for contact in body.getContactPoints(True)
                ),
                person_detected=any(
                    item["label"] == "person"
                    and item.get("observation_status") == "observed"
                    for item in detections
                ),
            )
            rows.append(
                dict(
                    time_s=timestamp,
                    state=decision["state"],
                    reason=decision["reason"],
                    speed_mps=ego_speed,
                    target_speed_mps=decision["target_speed_mps"],
                    camera_alert=health["alert"],
                    **evaluation,
                )
            )
            log.write(
                json.dumps(
                    dict(prediction=metadata, evaluation_only=evaluation),
                    allow_nan=False,
                )
                + "\n"
            )
            log.flush()
            # ---------------- DISPLAY / OUTPUT ----------------
            preview = draw_camera_alert(
                annotate_frame(
                    completed["frame"] if completed else frame,
                    detections,
                    elapsed_ms,
                    settings["motion"],
                ),
                health,
            )
            preview = append_navigation_panel(
                preview, decision, ego_speed, timestamp, phase
            )
            ok, jpeg = cv2.imencode(".jpg", preview)
            snapshot.publish(metadata, jpeg.tobytes() if ok else None)
            if frame_id % 15 == 1 or finished:
                cv2.imwrite(str(output / "latest_preview.jpg"), preview)
                cv2.imwrite(str(output / f"{phase}_preview.jpg"), preview)
                self_summary = dict(
                    frames=frame_id,
                    profile=realism["profile"],
                    completed_inferences=completed_count,
                    above_ground_contact_frames=sum(
                        r["above_ground_contact"] for r in rows
                    ),
                    assumptions=realism,
                    simulation_time_s=timestamp,
                    clearance_violations=sum(r["clearance_violation"] for r in rows),
                    minimum_front_gap_m=min(
                        (
                            r["truth_front_gap_m"]
                            for r in rows
                            if r["truth_front_gap_m"] is not None
                        ),
                        default=None,
                    ),
                    maximum_odometry_error_m=max(
                        abs(r["odometry_error_m"]) for r in rows
                    ),
                    phases={},
                )
                for name in dict.fromkeys(r["phase"] for r in rows):
                    group = [r for r in rows if r["phase"] == name]
                    self_summary["phases"][name] = dict(
                        frames=len(group),
                        moving_frames=sum(r["target_speed_mps"] > 0 for r in group),
                        stopped_frames=sum(r["target_speed_mps"] == 0 for r in group),
                        person_detection_fraction=sum(
                            r["person_detected"] for r in group
                        )
                        / len(group),
                        camera_alert_frames=sum(r["camera_alert"] for r in group),
                        reasons=sorted(set(r["reason"] for r in group)),
                        maximum_measured_speed_mps=max(r["speed_mps"] for r in group),
                    )
                (output / "latest_summary.json").write_text(
                    json.dumps(self_summary, indent=2), encoding="utf-8"
                )
            if finished or not show_preview(preview):
                robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                print(
                    "MOVING TEST COMPLETE: simulation/moving_results/latest_summary.json",
                    flush=True,
                )
                break
    finally:
        for motor in motors:
            motor.setVelocity(0)
        log.close()
        server.shutdown()
        server.server_close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
