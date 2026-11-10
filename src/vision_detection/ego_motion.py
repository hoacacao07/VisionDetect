"""Straight-line wheel-odometry compensation; no turning or general SLAM.

The camera must keep a fixed height/pitch and heading on level ground. Wheel
travel is an estimated sensor measurement, not a simulator position lookup.
On a Pi, supply measured encoder travel and speed, not commanded motor speed.
"""

import math
from .tracking import StationaryCameraTracker, linear_velocity, front_zone_interval


class StraightLineTracker(StationaryCameraTracker):
    """Fit object motion after compensating measured straight-line camera travel."""

    def update(self, detections, timestamp, width, height, travel_m, speed_mps):
        """Attach measured encoder travel/speed to this captured image before tracking."""
        self.travel_m = travel_m
        self.speed_mps = speed_mps
        return super().update(detections, timestamp, width, height)

    def metric_sample_position(self, position):
        # Shift observations into the encoder origin BEFORE fitting velocity.
        # A stationary object should then have near-zero object speed even as
        # its camera-relative distance decreases while our robot approaches.
        """Move a camera-relative point into the fixed encoder-origin frame."""
        return position[0] + self.travel_m, position[1]

    def describe(self, track, position):
        """Report object velocity and relative approach under a fixed-heading assumption."""
        result = super().describe(track, position)
        result["camera_assumption"] = "straight_heading_encoder_compensated"
        result["ego_speed_mps"] = round(self.speed_mps, 3)
        result["ego_travel_m"] = round(self.travel_m, 3)
        if position is None or not self.ready(track.metric_samples):
            return result
        object_velocity = linear_velocity(track.metric_samples)
        relative_velocity = object_velocity[0] - self.speed_mps, object_velocity[1]
        result["relative_velocity_mps"] = dict(
            forward=round(relative_velocity[0], 2), left=round(relative_velocity[1], 2)
        )
        result["closing_speed_mps"] = round(
            -sum(p * v for p, v in zip(position, relative_velocity))
            / max(math.hypot(*position), 0.001),
            2,
        )
        entry, exit_time = front_zone_interval(
            position, relative_velocity, self.settings["front_zone"]
        )
        result.update(
            front_zone_entry_s=entry,
            front_zone_exit_s=exit_time,
            prediction_status=(
                "constant_relative_velocity_estimate"
                if entry is not None
                else "no_front_zone_entry_within_horizon"
            ),
        )
        return result
