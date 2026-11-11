"""REAL ROBOT: Pi camera -> detection -> stop/go policy -> ESP32 motor commands.

Hardware port of simulation/controllers/straight_robot/straight_robot.py. It
reuses the same detector, StraightLineTracker, StopGoController, limited_speed,
CameraHealthMonitor, vision-age check and dashboard. Webots devices are replaced:

  Webots camera.getImage()         -> LatestFrameCamera (Picamera2Input on the Pi)
  wheel encoders                   -> ESTIMATED travel from the ESP32's open-loop
                                      motor model (this robot has no encoders)
  range sensors                    -> Esp32Link telemetry (measured)
  motor.setVelocity()              -> Esp32Link.send_speed(), EVERY tick, also at 0
  Supervisor / scene script / eval -> removed

Two threads:
  * PerceptionWorker: newest frame -> health -> YOLOE -> tracking (slow on a Pi).
  * Control loop (main thread, control.rate_hz): RobotController.step() and a
    command to the ESP32, so slow inference never starves its 300 ms watchdog.

Equal left/right speeds (straight line only). While |gyro Z| exceeds
sensors.rotation_gyro_threshold_dps, object-speed estimation is disabled because
StraightLineTracker assumes no rotation. Motors stay at 0 unless --drive is given.
"""

import argparse
import copy
import sys
import json
import math
import signal
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
import cv2
from .camera import LatestFrameCamera
from .camera_health import CameraHealthMonitor, draw_camera_alert
from .detection_continuity import DetectionContinuity
from .display import annotate_frame, show_preview
from .ego_motion import StraightLineTracker
from .esp32_link import Esp32Link
from .navigation_display import append_navigation_panel
from .robot_config import (
    DEFAULT_ROBOT_SETTINGS_PATH,
    check_robot_camera_calibration,
    load_robot_settings,
)
from .robot_control import RobotController
from .server import DetectionSnapshot, start_local_server
from .settings import PROJECT_DIRECTORY, load_settings

DEFAULT_CAMERA_SETTINGS_PATH = PROJECT_DIRECTORY / "config" / "cameras" / "robot.json"
# detection.model value that selects the TEST/SIM-ONLY colour-box detector.
SIM_COLOR_DETECTOR = "sim:color_boxes"


def prepare_robot_settings(camera_settings):
    """Calibration check + the moving-camera assumptions, shared with the simulator."""
    check_robot_camera_calibration(camera_settings["distance"])
    settings = copy.deepcopy(camera_settings)
    # Same assumptions as the Webots moving controller, except travel is
    # estimated (no encoders): straight-travel compensation, metric motion only
    # from the calibrated ground plane.
    settings["motion"]["camera_stationary"] = False
    settings["motion"]["allow_size_based_speed"] = False
    return settings


def make_detector(settings):
    """The real YOLOE detector, or the colour-box stand-in for simulation."""
    if settings["detection"]["model"] == SIM_COLOR_DETECTOR:
        sys.path.insert(0, str(PROJECT_DIRECTORY))
        from sim.color_detector import ColorBoxDetector  # TEST/SIM ONLY.

        return ColorBoxDetector(settings["distance"])
    from .detector import ObjectDetector  # Imports torch: only when really needed.

    return ObjectDetector(settings["detection"], settings["distance"])


def control_tick(controller, link, perception, now, started, dt, motors_enabled):
    """SENSE -> DECIDE -> ACT once. The command is sent every tick (heartbeat).

    Used by run_robot() and by the simulator (sim/runner.py), so both run the
    exact same control step.
    """
    link_state = link.latest(now)
    decision = controller.step(now, started, dt, link_state, perception, motors_enabled)
    speed = decision["applied_motor_speed_mps"]
    link.send_speed(speed, speed)  # Equal sides: straight line only.
    return decision, link_state


class PerceptionWorker:
    """Run detection/tracking on the newest frame in a background thread."""

    def __init__(self, camera, settings, detector, tracker, link, sensors):
        """Keep references only; start() launches the thread."""
        self.camera = camera
        self.camera_settings = settings["camera"]
        self.detector = detector
        self.tracker = tracker
        self.link = link
        self.sensors = sensors
        self.continuity = DetectionContinuity()
        self.health_monitor = CameraHealthMonitor(settings.get("camera_health"))
        self.lock = threading.Lock()
        self.result = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        """Begin processing frames."""
        self.thread.start()

    def latest(self):
        """Newest completed result, or None before the first frame."""
        with self.lock:
            return self.result

    def run(self):
        """Process frames until stopped; failures are reported to the control loop."""
        try:
            started_at = time.monotonic()
            previous_sequence, previous_capture, frame_id = -1, None, 0
            while not self.stop_event.is_set():
                frame, sequence, capture_time = self.camera.get_latest_frame()
                if self.camera.stop_capture_event.is_set():
                    raise RuntimeError("Camera stopped providing frames.")
                if frame is None:
                    if time.monotonic() - started_at > self.camera_settings.get(
                        "startup_timeout_s", 10
                    ):
                        raise RuntimeError("No camera images before camera.startup_timeout_s.")
                    time.sleep(0.01)
                    continue
                if time.monotonic() - capture_time > self.camera_settings.get(
                    "maximum_frame_age_s", 2
                ):
                    raise RuntimeError("Camera frames are stale.")
                if sequence == previous_sequence or (
                    previous_capture is not None and capture_time <= previous_capture
                ):
                    time.sleep(0.005)
                    continue
                previous_sequence, previous_capture = sequence, capture_time
                frame_id += 1
                self.publish(self.process(frame, capture_time), frame_id)
        except Exception as error:
            self.error = error

    def process(self, frame, capture_time):
        """Health -> odometry at frame time -> detection -> tracking (if allowed)."""
        processing_started = time.perf_counter()
        height, width = frame.shape[:2]
        health = self.health_monitor.update(frame, capture_time)
        # Robot travel when this frame arrived, so our own approach is not
        # mistaken for object motion. None = no fresh telemetry: STOP later.
        odometry = self.link.odometry_at(
            capture_time, self.sensors["maximum_telemetry_age_s"]
        )
        odometry_ok = odometry is not None and all(math.isfinite(v) for v in odometry)
        detections, rotating = [], False
        if health["alert"] or not odometry_ok:
            self.tracker.tracks.clear()
            self.continuity.clear()
        else:
            travel_m, speed_mps, gyro_z_dps = odometry
            detections = self.detector.detect(frame)
            rotating = abs(gyro_z_dps) > self.sensors["rotation_gyro_threshold_dps"]
            if rotating:
                # StraightLineTracker assumes no rotation; history across a turn
                # would produce false object speeds. Positions are still used
                # by StopGoController, so obstacles still stop the robot.
                self.tracker.tracks.clear()
                self.continuity.clear()
                for detection in detections:
                    detection["motion"]["status"] = "speed_disabled_while_rotating"
                    detection["motion_status"] = "speed_disabled_while_rotating"
            else:
                self.tracker.update(
                    detections, capture_time, width, height, travel_m, speed_mps
                )
                detections = self.continuity.update(detections, capture_time, width, height)
        return dict(
            frame=frame,
            width=width,
            height=height,
            capture_time=capture_time,
            health=health,
            detections=detections,
            odometry_ok=odometry_ok,
            rotating=rotating,
            inference_ms=(time.perf_counter() - processing_started) * 1000,
        )

    def publish(self, result, frame_id):
        """Make one result visible to the control loop atomically."""
        result["frame_id"] = frame_id
        with self.lock:
            self.result = result

    def stop(self):
        """Ask the thread to finish its current frame and exit."""
        self.stop_event.set()
        self.thread.join(timeout=5)


def wait_for_telemetry(link, port, timeout_s):
    """Block until the ESP32 reports (it resets and calibrates the gyro on connect)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if link.error:
            raise RuntimeError(f"ESP32 serial link failed: {link.error}")
        if link.latest() is not None:
            return
        time.sleep(0.05)
    hint = f" Last ESP32 message: {link.last_diagnostic}." if link.last_diagnostic else ""
    raise RuntimeError(
        f"No telemetry from the ESP32 on {port}. Check the port, USB cable and that "
        f"firmware/esp32_robot is flashed ({link.rejected_lines} unreadable lines).{hint}"
    )


def run_robot(camera_settings, robot, show_window=False, motors_enabled=False, stop_event=None):
    """Drive straight with stop/slow/resume until stopped; motors always end at zero.

    stop_event (threading.Event) ends the loop cleanly; used by the sim tests.
    """
    settings = prepare_robot_settings(camera_settings)
    serial_settings = robot["serial"]
    sensors, control, output = robot["sensors"], robot["control"], robot["output"]

    detector = make_detector(settings)
    tracker = StraightLineTracker(settings["motion"], settings["distance"])
    controller = RobotController(
        robot["navigation"],
        settings["distance"],
        sensors,
        control,
        (settings["camera"]["width"], settings["camera"]["height"]),
    )
    snapshot = DetectionSnapshot()
    link = camera = server = worker = log = None
    try:
        server = start_local_server(snapshot, output["api_port"], output["api_host"])
        link = Esp32Link(
            serial_settings["port"],
            serial_settings["baud"],
            maximum_range_age_s=sensors["maximum_range_age_s"],
            speed_window_s=sensors["speed_window_s"],
        )
        wait_for_telemetry(link, serial_settings["port"], serial_settings["startup_timeout_s"])
        camera = LatestFrameCamera(settings["camera"], settings["distance"])
        worker = PerceptionWorker(camera, settings, detector, tracker, link, sensors)
        worker.start()
        if output["write_log"]:
            results = PROJECT_DIRECTORY / "results"
            results.mkdir(exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            log = (results / f"robot-{stamp}.jsonl").open("w", encoding="utf-8")
        mode_text = "MOTORS ENABLED" if motors_enabled else "DRY RUN - motors held at 0"
        print(
            f"ROBOT READY ({mode_text}). Dashboard port {output['api_port']}. Ctrl+C stops.",
            flush=True,
        )

        period = 1 / control["rate_hz"]
        started = previous_tick = next_tick = time.monotonic()
        last_published, preview, last_state = None, None, None
        while stop_event is None or not stop_event.is_set():
            now = time.monotonic()
            dt, previous_tick = now - previous_tick, now
            if worker.error:
                raise RuntimeError(f"Perception stopped: {worker.error}") from worker.error
            if link.error:
                raise RuntimeError(f"ESP32 serial link failed: {link.error}") from link.error

            perception = worker.latest()
            decision, link_state = control_tick(
                controller, link, perception, now, started, dt, motors_enabled
            )
            speed = decision["applied_motor_speed_mps"]

            if decision["state"] != last_state:
                last_state = decision["state"]
                print(
                    f"{now - started:7.1f}s  {decision['state']:4}  {decision['reason']}",
                    flush=True,
                )
            gyro = link_state.gyro_z_dps if link_state else math.nan
            heading = link_state.heading_deg if link_state else math.nan
            if log:
                detections = perception["detections"] if perception else []
                log.write(
                    json.dumps(
                        dict(
                            t=round(now - started, 3),
                            state=decision["state"],
                            reason=decision["reason"],
                            overrides=decision["safety_overrides"],
                            target_mps=decision["target_speed_mps"],
                            applied_mps=speed,
                            estimated_mps=decision["estimated_speed_mps"],
                            nearest_gap_m=decision["nearest_gap_m"],
                            limiting_source=decision["limiting_source"],
                            ranges_m=decision["front_ranges_m"],
                            gyro_z_dps=gyro if math.isfinite(gyro) else None,
                            heading_deg=heading if math.isfinite(heading) else None,
                            telemetry_age_s=round(link_state.age_s, 3) if link_state else None,
                            vision_age_s=decision["vision_age_s"],
                            rotating=perception["rotating"] if perception else None,
                            labels=[d["label"] for d in detections],
                        ),
                        allow_nan=False,
                    )
                    + "\n"
                )

            # PRESENT once per new perception result (dashboard + optional window).
            if perception and perception["frame_id"] != last_published:
                last_published = perception["frame_id"]
                preview = draw_camera_alert(
                    annotate_frame(
                        perception["frame"],
                        perception["detections"],
                        perception["inference_ms"],
                        settings["motion"],
                    ),
                    perception["health"],
                )
                preview = append_navigation_panel(
                    preview,
                    decision,
                    decision["estimated_speed_mps"],
                    now - started,
                    "motors enabled" if motors_enabled else "dry run: motors disabled",
                    title="DELIVERY ROBOT",
                    time_label="Session time",
                    footer="Ctrl+C stops | No steering",
                    sensors_label="Camera + range sensors + gyro",
                    speed_label="Estimated robot speed",
                )
                ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
                snapshot.publish(
                    dict(
                        schema_version=1,
                        source="robot_pi_esp32",
                        status="running",
                        camera_active=True,
                        timestamp_utc=datetime.now(timezone.utc).isoformat(),
                        frame_id=perception["frame_id"],
                        image_width=perception["width"],
                        image_height=perception["height"],
                        inference_ms=round(perception["inference_ms"], 1),
                        model=detector.description,
                        classes=detector.class_names,
                        distance_mode=settings["distance"]["mode"],
                        distance_assumptions=settings["distance"],
                        motion_analysis=(
                            "speed_disabled_while_rotating"
                            if perception["rotating"]
                            else "straight_line_estimated_travel_compensated"
                        ),
                        camera_health=perception["health"],
                        detections=perception["detections"],
                        navigation=decision,
                        ego_motion=dict(
                            travel_m=link_state.travel_m if link_state else None,
                            speed_mps=decision["estimated_speed_mps"],
                            estimated=True,
                            gyro_z_dps=gyro if math.isfinite(gyro) else None,
                            heading_deg=heading if math.isfinite(heading) else None,
                        ),
                        front_ranges_m=decision["front_ranges_m"],
                    ),
                    jpeg.tobytes() if ok else None,
                )
            if show_window and preview is not None and not show_preview(preview):
                break

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()  # Overran; do not try to catch up.
    finally:
        if link:
            link.close()  # Motors first: sends zero, then the watchdog covers the rest.
        if worker:
            worker.stop()
        if camera:
            camera.close()
        if log:
            log.close()
        snapshot.publish(
            dict(schema_version=1, status="stopped", camera_active=False, detections=[]),
            None,
        )
        if server:
            server.shutdown()
            server.server_close()
        cv2.destroyAllWindows()


def main():
    """Entry point used by robot.py."""
    parser = argparse.ArgumentParser(description="Delivery robot: Pi camera + ESP32 stop/go")
    parser.add_argument("--config", default=str(DEFAULT_CAMERA_SETTINGS_PATH))
    parser.add_argument("--robot", default=str(DEFAULT_ROBOT_SETTINGS_PATH))
    parser.add_argument(
        "--serial",
        help="Override serial.port, e.g. /dev/ttyUSB0, or sim://corridor for the "
        "simulated ESP32 (use with --config config/cameras/sim.json)",
    )
    parser.add_argument("--window", action="store_true", help="Show the OpenCV window")
    parser.add_argument(
        "--drive",
        action="store_true",
        help="Actually command the motors. Without it every command is 0 (dry run).",
    )
    arguments = parser.parse_args()
    if not Path(arguments.config).exists():
        raise SystemExit(f"Camera settings not found: {arguments.config}")

    def stop_on_terminate(*_):
        raise KeyboardInterrupt  # systemd stop -> same clean shutdown as Ctrl+C.

    signal.signal(signal.SIGTERM, stop_on_terminate)
    robot = load_robot_settings(arguments.robot)
    if arguments.serial:
        robot["serial"]["port"] = arguments.serial
    try:
        run_robot(
            load_settings(arguments.config),
            robot,
            show_window=arguments.window,
            motors_enabled=arguments.drive,
        )
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"Robot stopped: {error}", flush=True)
        raise SystemExit(1) from error
