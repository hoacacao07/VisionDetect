"""Transparent monocular estimates: these are NOT sensor measurements."""

# NAV-SCALE: replace/fuse these assumptions with calibrated range/depth before
# relying on them for robot clearance. Preserve null for unsupported estimates.

import math
from .camera_model import focal_length_px, pixel_to_ground


def estimate_distance_m(bounding_box, label, image_width, image_height, config):
    """Return (estimated metres or None, explanation of assumptions/rejection).

    bounding_box contains [left, top, right, bottom] in image pixels.
    Size mode returns approximate range to the object's centre; ground mode
    returns horizontal range to its ground contact. Neither is measured depth.
    A None value must remain null in JSON rather than being converted to zero.
    """
    mode = config.get("mode", "size")
    if mode == "off":
        return None, "disabled"
    box_left_px, box_top_px, box_right_px, box_bottom_px = bounding_box
    # Size mode needs the full height; ground mode needs the contact point.
    # A head outside the top of the image does not invalidate visible feet.
    if box_bottom_px > image_height * 0.98 or (
        mode != "ground" and box_top_px < image_height * 0.02
    ):
        return (
            None,
            "object clipped; full height (size) or base (ground) must be visible",
        )
    calibration = config.get("focal_calibration") or {}
    # Focal length is in PIXELS, not millimetres (camera_model.py).
    try:
        focal_px = focal_length_px(config, image_width)
    except ValueError:
        return None, "invalid camera field of view"
    camera_calibration_note = (
        "calibrated focal length"
        if calibration.get("focal_px") and calibration.get("image_width")
        else "assumed camera field of view"
    )
    if mode == "ground":
        # Ground mode follows a ray through the box's bottom centre. This only
        # works if that pixel represents the actual base on flat, visible ground.
        camera_height = config.get("camera_height_m")
        if (
            camera_height is None
            or config.get("camera_downward_tilt_degrees") is None
            or camera_height <= 0
        ):
            return None, "set measured camera height and downward tilt"
        ground = pixel_to_ground(
            (box_left_px + box_right_px) / 2,
            box_bottom_px,
            image_width,
            image_height,
            config,
        )
        if ground is None:
            return None, "base above/near ground horizon"
        # Horizontal distance on the floor, excluding the camera height.
        estimated_distance_m = math.hypot(*ground)
        if not 0.1 <= estimated_distance_m <= config.get("maximum_distance_m", 30):
            return None, "outside configured estimate range"
        return (
            round(estimated_distance_m, 2),
            "ground-plane estimate; level ground and visible base; "
            + camera_calibration_note,
        )
    # Size mode: Z = f_px * actual_height_m / apparent_height_px.
    # Actual size is usually unknown, so this uses the configured assumption.
    # Trees deliberately have no default size because their heights vary widely.
    assumed_height_m = config.get("assumed_object_heights_m", {}).get(label)
    if not assumed_height_m or assumed_height_m <= 0 or box_bottom_px <= box_top_px:
        return None, "unknown object size; use calibrated ground mode"
    optical_axis_depth_m = focal_px * assumed_height_m / (box_bottom_px - box_top_px)
    ray_horizontal = ((box_left_px + box_right_px) / 2 - image_width / 2) / focal_px
    ray_vertical = ((box_top_px + box_bottom_px) / 2 - image_height / 2) / focal_px
    # Convert forward (optical-axis) depth into approximate centre-ray range.
    estimated_distance_m = optical_axis_depth_m * math.sqrt(
        1 + ray_horizontal * ray_horizontal + ray_vertical * ray_vertical
    )
    if not 0.1 <= estimated_distance_m <= config.get("maximum_distance_m", 30):
        return None, "outside configured estimate range"
    return (
        round(estimated_distance_m, 2),
        f"size estimate; assumed {assumed_height_m:g} m tall; upright full object; {camera_calibration_note}",
    )
