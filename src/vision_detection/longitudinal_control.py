"""Simulation policy: forward movement only, with slow/stop/resume decisions.

No simulator reference positions or object names are accepted here. This is a
reactive prototype, not route planning. A no-return range reading is accepted
only for the configured ideal simulated sensors, not for arbitrary hardware.
"""

import math
from .tracking import metric_position


class StopGoController:
    """Own stop/slow/restart state independently of rendering and device adapters."""

    def __init__(self, settings, distance_settings):
        """Keep policy configuration separate from camera geometry and runtime state."""
        self.settings = settings
        self.distance_settings = distance_settings
        self.clear_since = None
        self.stopped = True

    def decide(self, timestamp, health, detections, ranges_m, width, height, speed_mps):
        """Return a target forward speed and the reason; never command hardware here."""
        config = self.settings
        reason = None
        nearest = (
            min(ranges_m)
            if ranges_m and all(math.isfinite(x) and x >= 0 for x in ranges_m)
            else None
        )
        source = "front_range" if nearest is not None else "unavailable"
        if health.get("alert"):
            reason = "camera_blocked"
        elif health.get("status") == "checking_visibility":
            # Stop during debounce, before the confirmed warning appears.
            reason = "camera_visibility_uncertain"
        if nearest is None:
            reason = reason or "range_sensor_unavailable"
        # A central object with an unknown/clipped base must not be ignored.
        for detection in detections:
            position, _ = metric_position(
                detection, width, height, self.distance_settings, False
            )
            if detection.get("observation_status") == "prediction_only":
                position = None
            box = detection["bbox_xyxy"]
            if position is None:
                # Fraction is the width of the central image band, centred at 50%.
                half_band = config.get("unknown_range_center_fraction", 0.2) / 2
                if box[0] < width * (0.5 + half_band) and box[2] > width * (
                    0.5 - half_band
                ):
                    reason = reason or "central_object_range_unknown"
                continue
            forward, left = position
            focal = self.distance_settings["focal_calibration"]["focal_px"]
            half_width = (box[2] - box[0]) * forward / (2 * focal)
            if (
                forward > 0
                and abs(left) <= config["corridor_half_width_m"] + half_width
            ):
                gap = max(0, forward - config["front_offset_m"])
                if nearest is None or gap < nearest:
                    nearest, source = gap, "camera_ground_estimate"
        stopping_gap = (
            config["stop_gap_m"]
            + max(0, speed_mps) ** 2 / (2 * config["braking_deceleration_mps2"])
            + max(0, speed_mps) * config["reaction_time_s"]
        )
        if nearest is not None and nearest <= stopping_gap:
            reason = reason or "obstacle_in_stop_zone"
        # Hysteresis and a clear-time requirement prevent motor chatter. An
        # obstacle just outside the stop boundary must not trigger a restart.
        if reason:
            self.stopped = True
            self.clear_since = None
        elif self.stopped:
            if nearest <= config["resume_gap_m"]:
                self.clear_since = None
                reason = "waiting_for_clearance"
            else:
                if self.clear_since is None:
                    self.clear_since = timestamp
                if timestamp - self.clear_since < config["clear_hold_s"]:
                    reason = "confirming_clear_path"
                else:
                    self.stopped = False
        target = 0.0 if self.stopped else config["cruise_speed_mps"]
        if target and nearest < config["slow_gap_m"]:
            target *= max(
                config.get("minimum_slow_speed_fraction", 0.2),
                min(
                    1,
                    (nearest - config["stop_gap_m"])
                    / (config["slow_gap_m"] - config["stop_gap_m"]),
                ),
            )
        return dict(
            state=(
                "STOP"
                if target == 0
                else "SLOW" if target < config["cruise_speed_mps"] else "MOVE"
            ),
            target_speed_mps=round(target, 3),
            reason=reason or "path_clear",
            nearest_gap_m=round(nearest, 3) if nearest is not None else None,
            limiting_source=source,
            stopping_threshold_m=round(stopping_gap, 3),
        )
