"""SYNTHETIC SCENES: rendered images with exact ground truth -> the real vision code.

Images come from sim/render_scene.py (same camera model as distance.py).
Boxes are found by the TEST-ONLY colour detector (sim/color_detector.py);
everything after that is the real code: distance.py, StraightLineTracker,
DetectionContinuity and PerceptionWorker.process.
"""

import copy
import math
import pytest
from sim.color_detector import ColorBoxDetector
from sim.render_scene import render_scene
from sim.world import camera_pose, load_robot_sim
from vision_detection.ego_motion import StraightLineTracker
from vision_detection.robot_config import load_robot_settings
from vision_detection.robot_runtime import PerceptionWorker, prepare_robot_settings
from vision_detection.settings import PROJECT_DIRECTORY, load_settings

SETTINGS = prepare_robot_settings(load_settings(PROJECT_DIRECTORY / "config/cameras/sim.json"))
DISTANCE = SETTINGS["distance"]
ROBOT_SETTINGS = load_robot_settings()
FRONT_OFFSET = ROBOT_SETTINGS["navigation"]["front_offset_m"]
ROBOT = load_robot_sim()
WIDTH, HEIGHT = SETTINGS["camera"]["width"], SETTINGS["camera"]["height"]
START = (0.0, 0.0, 0.0)
CAMERA_X = camera_pose(START, ROBOT, FRONT_OFFSET)[0]
RED = [40, 40, 220]


def box(front_x, centre_y, size=0.3, velocity=(0.0, 0.0), color=RED):
    return dict(
        name="box", x_min=front_x, x_max=front_x + size, y_min=centre_y - size / 2,
        y_max=centre_y + size / 2, height=size, color_bgr=color, velocity_mps=list(velocity),
    )


def world(*obstacles):
    return dict(name="test", robot_start=dict(x=0, y=0, heading_deg=0), obstacles=list(obstacles))


def render(scene, pose=START, t=0.0):
    return render_scene(scene, pose, t, DISTANCE, ROBOT, FRONT_OFFSET, WIDTH, HEIGHT)


def only_detection(frame):
    detections = ColorBoxDetector(DISTANCE).detect(frame)
    assert len(detections) == 1, f"expected one box, found {len(detections)}"
    return detections[0]


# ---------------------------------------------------------------- distance vs truth
# Camera 0.20 m high: at 3 m the box base is ~18 px below the horizon, so one
# pixel of rounding is a few percent. Limits grow with distance accordingly.
DISTANCE_CASES = [
    (0.5, 0.0, 0.03), (1.0, 0.0, 0.03), (1.5, 0.0, 0.03), (2.0, 0.0, 0.04), (3.0, 0.0, 0.06),
    (1.0, 0.5, 0.05), (1.0, -0.5, 0.05), (2.0, 1.0, 0.05), (2.0, -1.0, 0.05),
]


@pytest.mark.parametrize("forward, lateral, limit", DISTANCE_CASES)
def test_rendered_box_distance_matches_ground_truth(forward, lateral, limit):
    # Ground truth: the floor point under the middle of the box's front face.
    # For an off-centre box the lowest image point is the nearest front corner,
    # so the truth is the closest point of the box footprint to the camera.
    front_x = CAMERA_X + forward
    detection = only_detection(render(world(box(front_x, lateral))))
    nearest_y = max(lateral - 0.15, min(0.0, lateral + 0.15))
    truth = math.hypot(front_x - CAMERA_X, nearest_y)
    estimate = detection["distance_m"]
    error = abs(estimate - truth) / truth
    print(f"box at {forward} m, {lateral:+} m left: truth {truth:.3f} m, estimate {estimate} m, error {error * 100:.1f} %")
    assert error < limit


# ---------------------------------------------------------------- tracker + ego-motion
ROBOT_SPEED = 0.15


def track_moving_box(box_velocity, start_y=0.0, front_forward=2.2, seconds=2.5, fps=10):
    """Robot drives at 0.15 m/s; box moves at box_velocity. Returns the final motion dict."""
    tracker = StraightLineTracker(SETTINGS["motion"], DISTANCE)
    scene = world(box(CAMERA_X + front_forward, start_y, velocity=box_velocity))
    motion = None
    for i in range(int(seconds * fps) + 1):
        t = i / fps
        travel = ROBOT_SPEED * t
        detections = ColorBoxDetector(DISTANCE).detect(render(scene, (travel, 0.0, 0.0), t))
        assert len(detections) == 1
        tracker.update(detections, t, WIDTH, HEIGHT, travel, ROBOT_SPEED)
        motion = detections[0]["motion"]
    return motion


@pytest.mark.parametrize(
    "name, velocity, start_y",
    [
        ("stationary", (0.0, 0.0), 0.0),
        ("crossing_left_0.3", (0.0, 0.3), -0.5),
        ("approaching_0.2", (-0.2, 0.0), 0.0),
        ("walking_away_0.1", (0.1, 0.0), 0.0),
    ],
)
def test_tracker_object_speed_with_robot_moving(name, velocity, start_y):
    motion = track_moving_box(velocity, start_y)
    true_speed = math.hypot(*velocity)
    estimate = motion["object_speed_mps"]
    assert estimate is not None, motion["status"]
    print(f"{name}: true object speed {true_speed:.2f} m/s, estimated {estimate:.2f} m/s (robot at {ROBOT_SPEED} m/s)")
    assert abs(estimate - true_speed) < 0.1


def test_tracker_relative_velocity_subtracts_robot_speed():
    motion = track_moving_box((0.0, 0.0))
    # A stationary box approaches at the robot's own speed.
    assert motion["relative_velocity_mps"]["forward"] == pytest.approx(-ROBOT_SPEED, abs=0.1)


# ---------------------------------------------------------------- PerceptionWorker
class FakeLink:
    """odometry_at() stand-in: (travel_m, speed_mps, gyro_z_dps)."""

    def __init__(self, gyro_dps, speed=ROBOT_SPEED):
        self.gyro_dps, self.speed = gyro_dps, speed

    def odometry_at(self, timestamp, _maximum_gap):
        return self.speed * timestamp, self.speed, self.gyro_dps


def worker(gyro_dps):
    settings = copy.deepcopy(SETTINGS)
    return PerceptionWorker(
        None, settings, ColorBoxDetector(DISTANCE),
        StraightLineTracker(settings["motion"], DISTANCE), FakeLink(gyro_dps), ROBOT_SETTINGS["sensors"],
    )


def run_worker(perception, scene, frames=6, fps=10):
    results = []
    for i in range(frames):
        t = (i + 1) / fps
        results.append(perception.process(render(scene, (ROBOT_SPEED * t, 0.0, 0.0), t), t))
    return results


def test_tracker_is_skipped_when_gyro_above_threshold():
    threshold = ROBOT_SETTINGS["sensors"]["rotation_gyro_threshold_dps"]
    perception = worker(gyro_dps=threshold + 5)
    results = run_worker(perception, world(box(2.0, 0.0)))
    assert all(r["rotating"] for r in results)
    assert all(d["motion_status"] == "speed_disabled_while_rotating" for r in results for d in r["detections"])
    assert perception.tracker.tracks == {}
    # Obstacles are still reported, so the controller can still stop for them.
    assert all(len(r["detections"]) == 1 for r in results)


def test_tracker_runs_when_gyro_below_threshold():
    threshold = ROBOT_SETTINGS["sensors"]["rotation_gyro_threshold_dps"]
    perception = worker(gyro_dps=threshold - 5)
    results = run_worker(perception, world(box(2.0, 0.0)))
    assert not any(r["rotating"] for r in results)
    assert len(perception.tracker.tracks) == 1


# ---------------------------------------------------------------- DetectionContinuity
def test_continuity_keeps_one_id_without_jumps():
    results = run_worker(worker(0.0), world(box(2.0, 0.2)), frames=15)
    ids = {r["detections"][0]["motion"]["track_id"] for r in results}
    assert len(ids) == 1
    centres = [(sum(r["detections"][0]["bbox_xyxy"][0::2]) / 2) for r in results]
    jumps = [abs(b - a) for a, b in zip(centres, centres[1:])]
    print(f"largest frame-to-frame box centre jump: {max(jumps):.1f} px")
    assert max(jumps) < 10


def test_continuity_bridges_a_one_frame_gap_with_the_same_id():
    perception = worker(0.0)
    scene = world(box(2.0, 0.0))
    seen = run_worker(perception, scene, frames=5)
    track_id = seen[-1]["detections"][0]["motion"]["track_id"]
    t = 0.6
    gap = perception.process(render(world(), (ROBOT_SPEED * t, 0.0, 0.0), t), t)  # Box missing.
    assert len(gap["detections"]) == 1
    prediction = gap["detections"][0]
    assert prediction["observation_status"] == "prediction_only"
    assert prediction["motion"]["track_id"] == track_id
    assert prediction["distance_m"] is None  # A prediction is never a fresh measurement.
    t = 1.0  # Longer than DetectionContinuity's 0.35 s hold.
    late = perception.process(render(world(), (ROBOT_SPEED * t, 0.0, 0.0), t), t)
    assert late["detections"] == []
