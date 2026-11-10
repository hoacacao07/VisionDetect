"""Motion record defaults. Tracking fills fields only when it has enough data.

NAV-SCHEMA: preserve units/reference-frame meaning when extending these fields.
Physical time_to_contact_s must stay null until a collision model supplies it.
The contract and future changes are documented in docs/NAVIGATION_ROADMAP.md.
"""


def unmeasured_motion():
    """Return a fresh record: null means unavailable, not zero/stationary/safe."""
    return {
        "status": "unmeasured",
        "track_id": None,
        "object_speed_mps": None,
        "relative_velocity_mps": None,
        "closing_speed_mps": None,
        "time_to_contact_s": None,
        "front_zone_entry_s": None,
        "front_zone_exit_s": None,
        "prediction_status": "tracking_and_metric_position_required",
    }
