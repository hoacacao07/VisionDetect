"""DISTANCE ROUND TRIP: known floor points -> pixels -> distance.py -> same distance.

The projection here is written independently of camera_model.py (camera axes
built with a cross product, numpy matrices), so a mistake in either one shows
up. Pure pinhole maths: no lens distortion, so the limit is 1 %.
"""

import math
import numpy as np
import pytest
from vision_detection.camera_model import pixel_to_ground, project_point
from vision_detection.distance import estimate_distance_m

WIDTH, HEIGHT = 640, 360  # config/cameras/robot.json
CAMERA = dict(
    mode="ground",
    horizontal_fov_degrees=102,  # Camera Module 3 Wide
    focal_calibration={},  # Use the FOV, as before calibration.
    camera_height_m=0.20,
    camera_downward_tilt_degrees=10,
    maximum_distance_m=8,
)
FOCAL = WIDTH / (2 * math.tan(math.radians(102) / 2))  # ~259.1 px


def independent_projection(forward, left, up=0.0, settings=CAMERA):
    """Pinhole projection with a rotation matrix (not camera_model.py)."""
    tilt = math.radians(settings["camera_downward_tilt_degrees"])
    optical = np.array([math.cos(tilt), 0.0, -math.sin(tilt)])  # (forward, left, up)
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(optical, right)
    rotation = np.vstack([right, down, optical])  # world -> camera (x right, y down, z forward)
    x, y, z = rotation @ (np.array([forward, left, up]) - [0, 0, settings["camera_height_m"]])
    return WIDTH / 2 + FOCAL * x / z, HEIGHT / 2 + FOCAL * y / z


def widest_bearing(distance):
    """Largest bearing (deg) that keeps the floor point well inside the image."""
    best = 0
    for bearing in range(0, 60):
        b = math.radians(bearing)
        u, v = independent_projection(distance * math.cos(b), distance * math.sin(b))
        if 25 <= u <= WIDTH - 25 and v <= 0.95 * HEIGHT:
            best = bearing
    return best


CASES = [
    (distance, side)
    for distance in (0.5, 1.0, 2.0, 3.0)
    for side in ("centre", "left_edge", "right_edge")
]


@pytest.mark.parametrize("distance, side", CASES)
def test_floor_point_round_trip_within_1_percent(distance, side):
    bearing = 0 if side == "centre" else widest_bearing(distance)
    if side == "right_edge":
        bearing = -bearing
    b = math.radians(bearing)
    u, v = independent_projection(distance * math.cos(b), distance * math.sin(b))
    box = [u - 10, v - 30, u + 10, v]  # Bottom-centre of the box = the floor point.
    estimate, note = estimate_distance_m(box, "box", WIDTH, HEIGHT, CAMERA)
    print(f"{distance} m {side} (bearing {bearing} deg, pixel {u:.0f},{v:.0f}): {estimate} m")
    assert estimate is not None, note
    assert abs(estimate - distance) / distance < 0.01


def test_edge_points_really_are_near_the_edges():
    # Guards the test itself: the "edge" cases must reach the outer image.
    for distance in (1.0, 2.0, 3.0):
        b = math.radians(widest_bearing(distance))
        u, _ = independent_projection(distance * math.cos(b), distance * math.sin(b))
        assert u < 0.15 * WIDTH


def test_camera_model_projection_matches_independent_projection():
    for forward, left, up in ((0.5, 0.2, 0.0), (2.0, -0.7, 0.3), (3.0, 1.5, 0.1)):
        u, v, _ = project_point(forward, left, up, WIDTH, HEIGHT, CAMERA)
        expected = independent_projection(forward, left, up)
        assert u == pytest.approx(expected[0], abs=1e-6)
        assert v == pytest.approx(expected[1], abs=1e-6)


def test_pixel_to_ground_inverts_projection():
    u, v = independent_projection(1.7, -0.4)
    forward, left = pixel_to_ground(u, v, WIDTH, HEIGHT, CAMERA)
    assert (forward, left) == (pytest.approx(1.7, abs=1e-9), pytest.approx(-0.4, abs=1e-9))


def horizon_row():
    return HEIGHT / 2 - FOCAL * math.tan(math.radians(CAMERA["camera_downward_tilt_degrees"]))


@pytest.mark.parametrize("offset_px", [0.0, -1.0, -50.0])
def test_points_at_or_above_horizon_are_unknown(offset_px):
    v = horizon_row() + offset_px
    estimate, note = estimate_distance_m([300, v - 30, 340, v], "box", WIDTH, HEIGHT, CAMERA)
    # Unknown is None in Python (null in JSON, STOP in the controller); never a huge number.
    assert estimate is None
    assert "horizon" in note


def test_just_below_horizon_is_never_a_huge_number():
    for step in range(1, 120):
        v = horizon_row() + step * 0.25
        estimate, _ = estimate_distance_m([300, v - 30, 340, v], "box", WIDTH, HEIGHT, CAMERA)
        assert estimate is None or estimate <= CAMERA["maximum_distance_m"]
