"""Transparent monocular estimates: these are NOT sensor measurements."""

# NAV-SCALE: replace/fuse these assumptions with calibrated range/depth before
# relying on them for robot clearance. Preserve null for unsupported estimates.

import math


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
    calibration = config.get("focal_calibration", {})
    if calibration.get("focal_px") and calibration.get("image_width"):
        # Focal length is in PIXELS, not millimetres. Scale it when resolution
        # changes; this assumes the camera keeps the same crop and field of view.
        focal_length_px = (
            calibration["focal_px"] * image_width / calibration["image_width"]
        )
        camera_calibration_note = "calibrated focal length"
    else:
        horizontal_fov_degrees = float(config.get("horizontal_fov_degrees", 65))
        if not 10 < horizontal_fov_degrees < 150:
            return None, "invalid camera field of view"
        # Pinhole camera geometry: f_px = image_width / (2 * tan(HFOV / 2)).
        focal_length_px = image_width / (
            2 * math.tan(math.radians(horizontal_fov_degrees) / 2)
        )
        camera_calibration_note = "assumed camera field of view"
    if mode == "ground":
        # Ground mode follows a ray through the box's bottom centre. This only
        # works if that pixel represents the actual base on flat, visible ground.
        camera_height = config.get("camera_height_m")
        downward_tilt_radians = config.get("camera_downward_tilt_degrees")
        if camera_height is None or downward_tilt_radians is None or camera_height <= 0:
            return None, "set measured camera height and downward tilt"
        downward_tilt_radians = math.radians(downward_tilt_radians)
        # Normalized image rays: right is +horizontal, down is +vertical,
        # and forward is 1. Rotate the ray by the camera's downward pitch.
        ray_horizontal = (
            (box_left_px + box_right_px) / 2 - image_width / 2
        ) / focal_length_px
        ray_vertical = (box_bottom_px - image_height / 2) / focal_length_px
        ray_downward = math.sin(downward_tilt_radians) + ray_vertical * math.cos(
            downward_tilt_radians
        )
        ray_forward = math.cos(downward_tilt_radians) - ray_vertical * math.sin(
            downward_tilt_radians
        )
        if ray_downward <= 0.03 or ray_forward <= 0:
            return None, "base above/near ground horizon"
        # Intersect with the ground: scale = camera_height / ray_downward.
        # Then take the horizontal distance, excluding the vertical component.
        estimated_distance_m = (
            camera_height * math.hypot(ray_horizontal, ray_forward) / ray_downward
        )
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
    optical_axis_depth_m = (
        focal_length_px * assumed_height_m / (box_bottom_px - box_top_px)
    )
    ray_horizontal = (
        (box_left_px + box_right_px) / 2 - image_width / 2
    ) / focal_length_px
    ray_vertical = (
        (box_top_px + box_bottom_px) / 2 - image_height / 2
    ) / focal_length_px
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
