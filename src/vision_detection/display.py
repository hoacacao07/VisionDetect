"""DISPLAY: annotate a copy of the camera frame without changing detection."""

import cv2

CAMERA_WINDOW_TITLE = "Vision Detection - webcam - Q to quit"
# OpenCV colors are BGR, not RGB.
MOBILITY_COLORS = {
    "potentially_dynamic": (60, 165, 255),  # Orange: could move.
    "usually_static": (255, 185, 70),  # Blue: usually stationary.
}
UNKNOWN_CATEGORY_COLOR = (185, 185, 185)
OBJECT_AREA_TINT_OPACITY = 0.12


def annotate_frame(frame, detections, processing_ms, motion_settings):
    """Render existing observations only; never calculate movement decisions.

    NAV-DISPLAY: future path/zone overlays belong here, while the underlying
    navigation geometry/policy must remain outside this presentation module.
    """
    preview = frame.copy()
    # Tint each detected object's rectangle before drawing any labels, so
    # overlapping rectangles do not paint over text. This is a bounding area,
    # not a pixel-accurate object silhouette or a measured safety buffer.
    area_overlay = frame.copy()
    for detection in detections:
        left, top, right, bottom = map(int, detection["bbox_xyxy"])
        color = MOBILITY_COLORS.get(
            detection.get("motion_group"), UNKNOWN_CATEGORY_COLOR
        )
        cv2.rectangle(area_overlay, (left, top), (right, bottom), color, -1)
    preview = cv2.addWeighted(
        area_overlay, OBJECT_AREA_TINT_OPACITY, preview, 1 - OBJECT_AREA_TINT_OPACITY, 0
    )
    for detection in detections:
        left, top, right, bottom = map(int, detection["bbox_xyxy"])
        distance_m = detection["distance_m"]
        # Color describes the catalog category, never label confidence, danger,
        # distance availability, or proof that this individual is moving.
        color = MOBILITY_COLORS.get(
            detection.get("motion_group"), UNKNOWN_CATEGORY_COLOR
        )
        cv2.rectangle(preview, (left, top), (right, bottom), color, 2)
        distance_label = (
            f"~{distance_m:.1f}m EST" if distance_m is not None else "range unknown"
        )
        label = f"{detection['label']} {detection['confidence']:.0%} | {distance_label}"
        if detection.get("observation_status") == "prediction_only":
            label = f"{detection['label']} | PREDICTED - no fresh detection"
        motion = detection.get("motion", {})
        label = f"#{motion.get('track_id','?')} " + label
        cv2.putText(
            preview,
            label,
            (max(4, left), max(66, top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2,
        )
        if detection.get("observation_status") == "prediction_only":
            speed_text = "Brief recognition gap | speed/range unknown"
        elif motion.get("object_speed_mps") is not None:
            speed_text = f"~{motion['object_speed_mps']:.2f}m/s EST"
        elif motion.get("image_speed_px_s") is not None:
            speed_text = f"{motion['image_speed_px_s']:.0f}px/s | metric speed unknown"
        else:
            speed_text = "Collecting motion history..."
        arrival = motion.get("front_zone_entry_s")
        if arrival is not None:
            arrival_text = (
                "FRONT ZONE: already inside"
                if arrival == 0
                else f"FRONT ZONE: ~{arrival:.1f}s to reach"
            )
        elif motion.get("prediction_status") == "no_front_zone_entry_within_horizon":
            arrival_text = f"FRONT ZONE: no predicted entry in next {motion_settings['front_zone']['prediction_horizon_s']:g}s"
        elif motion.get("status") == "approximately_stationary":
            arrival_text = "FRONT ZONE: not approaching (stationary estimate)"
        elif motion.get("status") == "collecting_history":
            arrival_text = "FRONT ZONE: learning motion..."
        else:
            arrival_text = "FRONT ZONE: time unknown (need usable scale/motion)"
        # Put both lines under the box unless there is no room; then use its
        # inside bottom edge. A dark backing makes labels legible outdoors.
        text_x = max(4, min(left, preview.shape[1] - 300))
        text_y = (
            bottom + 17 if bottom + 43 < preview.shape[0] - 28 else max(65, bottom - 30)
        )
        for index, line in enumerate((speed_text, arrival_text)):
            baseline = text_y + index * 18
            (text_width, text_height), _ = cv2.getTextSize(
                line, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1
            )
            cv2.rectangle(
                preview,
                (text_x - 3, baseline - text_height - 3),
                (min(preview.shape[1], text_x + text_width + 4), baseline + 4),
                (24, 29, 29),
                -1,
            )
            cv2.putText(
                preview,
                line,
                (text_x, baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                color,
                1,
            )
    cv2.rectangle(preview, (0, 0), (preview.shape[1], 51), (28, 25, 22), -1)
    cv2.putText(
        preview,
        f"VISION DETECTION | {processing_ms:.0f}ms | Q to quit",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (235, 240, 245),
        1,
    )
    # A text legend keeps the meaning explicit even without relying on color.
    cv2.putText(
        preview,
        "ORANGE: could move",
        (10, 43),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        MOBILITY_COLORS["potentially_dynamic"],
        1,
    )
    cv2.putText(
        preview,
        "BLUE: usually stationary",
        (190, 43),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        MOBILITY_COLORS["usually_static"],
        1,
    )
    zone = motion_settings["front_zone"]
    footer = f"Arrival = example zone {zone['near_m']:g}-{zone['far_m']:g}m ahead, {zone['width_m']:g}m wide | rough estimates"
    cv2.rectangle(
        preview,
        (0, preview.shape[0] - 22),
        (preview.shape[1], preview.shape[0]),
        (24, 29, 29),
        -1,
    )
    cv2.putText(
        preview,
        footer,
        (8, preview.shape[0] - 7),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.36,
        (225, 230, 235),
        1,
    )
    return preview


def show_preview(preview):
    """Return False when the user presses Q/Escape or closes the native window."""
    cv2.imshow(CAMERA_WINDOW_TITLE, preview)
    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
        return False
    return cv2.getWindowProperty(CAMERA_WINDOW_TITLE, cv2.WND_PROP_VISIBLE) >= 1
