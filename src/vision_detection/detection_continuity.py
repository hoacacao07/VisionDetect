"""Bridge brief recognition gaps, explicitly marked as predictions.

This cannot turn a missed/never-seen object into a detection. Predicted boxes
expire quickly, carry no new range/speed measurement, and must not inflate
recognition scores in evaluation. Their last known label is only a hypothesis.
"""

import copy
from .motion import unmeasured_motion
from .perception_fusion import intersection_over_union


class DetectionContinuity:
    """Maintain short-lived display predictions separately from fresh observations."""

    def __init__(self, maximum_hold_s=0.35):
        """Set the maximum prediction age in seconds and start with no remembered tracks."""
        self.maximum_hold_s = maximum_hold_s
        self.last_seen = {}

    def clear(self):
        """Discard remembered tracks after obstruction or session reset."""
        self.last_seen.clear()

    def update(self, detections, capture_time, width, height):
        """Return fresh observations plus unexpired predictions with unknown metric values."""
        current = set()
        result = []
        for detection in detections:
            item = copy.deepcopy(detection)
            identity = item["motion"]["track_id"]
            current.add(identity)
            item.update(observation_status="observed", observation_age_s=0)
            self.last_seen[identity] = (capture_time, item)
            result.append(item)
        for identity, (seen, item) in list(self.last_seen.items()):
            age = capture_time - seen
            if age > self.maximum_hold_s:
                del self.last_seen[identity]
                continue
            if identity in current:
                continue
            # Reassociation or a label change can give the same physical box a
            # new ID. A fresh overlapping observation supersedes the old track;
            # do not add a second prediction or an artificial unknown-range stop.
            if any(
                intersection_over_union(item["bbox_xyxy"], fresh["bbox_xyxy"]) > 0.35
                for fresh in detections
            ):
                continue
            prediction = copy.deepcopy(item)
            velocity = item["motion"].get("image_velocity_px_s") or {}
            dx, dy = velocity.get("right", 0) * age, velocity.get("down", 0) * age
            left, top, right, bottom = item["bbox_xyxy"]
            bounds = [
                max(0, left + dx),
                max(0, top + dy),
                min(width, right + dx),
                min(height, bottom + dy),
            ]
            if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
                continue
            motion = unmeasured_motion()
            motion.update(
                track_id=identity,
                status="prediction_only",
                prediction_status="brief_gap_prediction",
            )
            prediction.update(
                bbox_xyxy=bounds,
                motion=motion,
                motion_status="prediction_only",
                observation_status="prediction_only",
                observation_age_s=round(age, 3),
                recognition_source="temporal_prediction",
                distance_m=None,
                distance_status="unavailable",
                distance_note="Predicted track during a brief recognition gap; no fresh range measurement",
            )
            result.append(prediction)
        return result
