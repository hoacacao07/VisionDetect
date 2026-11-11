"""CAMERA MODEL: the one pinhole model shared by vision, simulation and tests.

Coordinates (all metres):
  Ground point relative to the camera: forward (along the robot's heading),
  left, and up (height above the floor). The origin is the floor point
  directly below the lens.
  Image pixels: u to the right, v down; (width/2, height/2) is the optical axis.

The camera sits camera_height_m above the floor and is pitched down by
camera_downward_tilt_degrees. No lens distortion is modelled: a wide lens
bends lines near the image edges, so real errors grow there.

Used by distance.py (pixel -> ground), tracking.py and longitudinal_control.py
(focal length) and sim/render_scene.py (3D point -> pixel). Change it here only.
"""

import math

# Rays this close to the horizon (downward component) are rejected: the ground
# intersection would be very far away and extremely sensitive to pixel noise.
MINIMUM_DOWNWARD_RAY = 0.03


def focal_length_px(distance_settings, image_width):
    """Focal length in pixels: calibrated value if present, else from the HFOV.

    A calibrated focal length is scaled to the current image width; this
    assumes the same crop/field of view at both resolutions.
    """
    calibration = distance_settings.get("focal_calibration") or {}
    if calibration.get("focal_px") and calibration.get("image_width"):
        return calibration["focal_px"] * image_width / calibration["image_width"]
    hfov = float(distance_settings.get("horizontal_fov_degrees", 65))
    if not 10 < hfov < 150:
        raise ValueError("invalid camera field of view")
    # Pinhole geometry: f_px = image_width / (2 * tan(HFOV / 2)).
    return image_width / (2 * math.tan(math.radians(hfov) / 2))


def _axes(tilt_degrees):
    """Camera axes in (forward, left, up): optical axis, image-right, image-down."""
    tilt = math.radians(tilt_degrees)
    axis = (math.cos(tilt), 0.0, -math.sin(tilt))
    right = (0.0, -1.0, 0.0)
    down = (-math.sin(tilt), 0.0, -math.cos(tilt))
    return axis, right, down


def point_depth(forward, left, up, distance_settings):
    """Distance of a point along the optical axis (negative = behind the camera)."""
    axis, _, _ = _axes(distance_settings["camera_downward_tilt_degrees"])
    point = (forward, left, up - distance_settings["camera_height_m"])
    return sum(p * a for p, a in zip(point, axis))


def project_point(forward, left, up, image_width, image_height, distance_settings):
    """3D point (camera-relative, see module doc) -> (u, v, depth), or None if behind."""
    focal = focal_length_px(distance_settings, image_width)
    axis, right, down = _axes(distance_settings["camera_downward_tilt_degrees"])
    point = (forward, left, up - distance_settings["camera_height_m"])
    depth = point_depth(forward, left, up, distance_settings)
    if depth <= 1e-6:
        return None
    x = sum(p * r for p, r in zip(point, right))
    y = sum(p * d for p, d in zip(point, down))
    return image_width / 2 + focal * x / depth, image_height / 2 + focal * y / depth, depth


def pixel_to_ground(u, v, image_width, image_height, distance_settings):
    """Pixel -> (forward_m, left_m) on the floor, or None at/above the horizon."""
    focal = focal_length_px(distance_settings, image_width)
    tilt = math.radians(distance_settings["camera_downward_tilt_degrees"])
    ray_right = (u - image_width / 2) / focal
    ray_down = (v - image_height / 2) / focal
    # Rotate the image ray (right, down, 1) by the camera's downward pitch.
    downward = math.sin(tilt) + ray_down * math.cos(tilt)
    forward = math.cos(tilt) - ray_down * math.sin(tilt)
    if downward <= MINIMUM_DOWNWARD_RAY or forward <= 0:
        return None
    # Intersect with the floor: scale the ray until it has dropped camera_height.
    scale = distance_settings["camera_height_m"] / downward
    return scale * forward, -scale * ray_right
