"""VISION INTO THE CONTROLLER: the camera alone must slow/stop the robot.

Every HC-SR04 is forced to report 3.9 m (clear), so only rendered camera
frames -> colour-box detector -> distance.py -> StopGoController can react.
"""

import pytest
from sim.plots import save_plots
from sim.runner import Scenario, run_scenario
from vision_detection.robot_config import load_robot_settings

NAVIGATION = load_robot_settings()["navigation"]
CAMERA = "camera_ground_estimate"  # StopGoController's limiting_source for camera gaps.


def pi_rows(result):
    return [row for row in result.rows if row["pi_alive"]]


@pytest.fixture(scope="module")
def box_in_path():
    result = run_scenario(Scenario("camera_box_in_path", "box_in_corridor", duration_s=30, force_ranges_m=3.9))
    save_plots(result)
    return result


@pytest.fixture(scope="module")
def box_outside_path():
    result = run_scenario(
        Scenario("camera_box_outside_path", "box_outside_corridor", duration_s=30,
                 force_ranges_m=3.9, stop_after_travel_m=2.4)
    )
    save_plots(result)
    return result


def test_box_in_path_slows_because_of_the_camera(box_in_path):
    assert any(row["state"] == "SLOW" and row["source"] == CAMERA for row in pi_rows(box_in_path))


def test_box_in_path_stops_because_of_the_camera(box_in_path):
    final = pi_rows(box_in_path)[-1]
    print(
        f"camera gap {final['nearest_gap_m']:.3f} m, true clearance {final['clearance_m']:.3f} m, "
        f"stop_gap {NAVIGATION['stop_gap_m']} m"
    )
    assert final["state"] == "STOP" and final["source"] == CAMERA
    assert abs(final["true_speed"]) < 0.005


def test_box_in_path_never_touched(box_in_path):
    assert min(row["clearance_m"] for row in box_in_path.rows) >= NAVIGATION["stop_gap_m"]


def test_camera_gap_matches_true_clearance(box_in_path):
    # Whole approach: camera estimate vs true bumper clearance, while moving
    # (includes frame age and pixel rounding).
    rows = [row for row in pi_rows(box_in_path) if row["source"] == CAMERA]
    relative = [abs(row["nearest_gap_m"] - row["clearance_m"]) / row["clearance_m"] for row in rows]
    too_far = max(row["nearest_gap_m"] - row["clearance_m"] for row in rows)
    print(
        f"camera gap error: max {max(relative) * 100:.1f} % of the true clearance; "
        f"largest over-estimate {too_far * 100:+.1f} cm over {len(rows)} ticks"
    )
    assert max(relative) < 0.10
    # Over-estimating the gap is the unsafe direction. The only allowed cause is
    # frame age: the robot keeps moving while a frame is processed (cruise speed
    # x ~0.3 s). navigation.reaction_time_s covers this in the stop distance.
    assert too_far < NAVIGATION["cruise_speed_mps"] * 0.3


def test_box_outside_path_never_slows(box_outside_path):
    rows = pi_rows(box_outside_path)
    assert not any(row["source"] == CAMERA for row in rows)
    moving = [row for row in rows if row["t"] > 2.0]
    assert all(row["state"] == "MOVE" for row in moving)
    assert box_outside_path.rows[-1]["x"] >= 2.4  # Drove past the box's front face.
