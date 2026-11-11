"""ROBOT DECISION STEP: sensor freshness checks + StopGoController + speed ramp.

Pure Python with no camera, serial or model imports, so every STOP path can be
unit-tested. robot_runtime.py calls step() once per control tick and sends the
returned applied_motor_speed_mps to the ESP32 every tick, including zero.

Rule: any missing or stale input forces STOP. NaN means unknown, never clear.

No wheel encoders: robot speed is ESTIMATED by the ESP32 from what it commanded.
The stop/slow distance uses that estimate raised by sensors.speed_estimate_margin,
so a robot that is really faster than its estimate still stops in time.
"""

import math
from .longitudinal_control import StopGoController
from .sensor_timing import limited_speed


def _finite(value):
    return value is not None and math.isfinite(value)


def safety_overrides(now, started, link_state, perception, sensors, control):
    """Reasons that force STOP regardless of the policy, most important first."""
    reasons = []
    if link_state is None or link_state.age_s > sensors["maximum_telemetry_age_s"]:
        reasons.append("esp32_telemetry_stale")
    else:
        if not _finite(link_state.speed_mps):
            reasons.append("speed_estimate_unavailable")
        if not _finite(link_state.gyro_z_dps):
            reasons.append("gyro_unavailable")
    if perception is None:
        reasons.append("waiting_for_first_frame")
    else:
        if now - perception["capture_time"] > control["maximum_vision_age_s"]:
            reasons.append("waiting_for_fresh_vision")
        if not perception["odometry_ok"]:
            reasons.append("odometry_unavailable_for_frame")
    if now - started > control["maximum_session_s"]:
        reasons.append("session_time_limit")
    return reasons


class RobotController:
    """Owns policy state and the ramped motor command between ticks."""

    def __init__(self, navigation, distance_settings, sensors, control, default_size):
        """default_size is (width, height) used before the first camera frame."""
        self.policy = StopGoController(navigation, distance_settings)
        self.sensors = sensors
        self.control = control
        self.default_size = default_size
        self.motor_command = 0.0

    def step(self, now, started, dt, link_state, perception, motors_enabled):
        """Return the decision dict; applied_motor_speed_mps is what to send."""
        telemetry_fresh = (
            link_state is not None
            and link_state.age_s <= self.sensors["maximum_telemetry_age_s"]
        )
        # Ranges arrive already NaN-masked when older than maximum_range_age_s.
        ranges = list(link_state.ranges_m) if telemetry_fresh else [math.nan] * 3
        ego_speed = (
            link_state.speed_mps
            if telemetry_fresh and _finite(link_state.speed_mps)
            else 0.0
        )
        if perception is not None:
            health, detections = perception["health"], perception["detections"]
            width, height = perception["width"], perception["height"]
        else:
            health, detections = dict(status="starting", alert=False), []
            width, height = self.default_size

        # Conservative: the real speed may exceed the open-loop estimate.
        braking_speed = ego_speed * (1 + self.sensors["speed_estimate_margin"])
        decision = self.policy.decide(
            now, health, detections, ranges, width, height, braking_speed
        )
        overrides = safety_overrides(
            now, started, link_state, perception, self.sensors, self.control
        )
        if overrides:
            decision.update(state="STOP", target_speed_mps=0.0, reason=overrides[0])
            # A restart must again satisfy the policy's clear-hold time.
            self.policy.stopped, self.policy.clear_since = True, None

        target = decision["target_speed_mps"] if motors_enabled else 0.0
        self.motor_command = limited_speed(
            self.motor_command,
            target,
            max(0.0, dt),
            self.control["maximum_acceleration_mps2"],
            self.control["maximum_deceleration_mps2"],
        )
        vision_age = now - perception["capture_time"] if perception else None
        decision.update(
            safety_overrides=overrides,
            applied_motor_speed_mps=round(self.motor_command, 3),
            motors_enabled=motors_enabled,
            vision_age_s=round(vision_age, 3) if vision_age is not None else None,
            front_ranges_m=[r if math.isfinite(r) else None for r in ranges],
            estimated_speed_mps=round(ego_speed, 3),
        )
        return decision
