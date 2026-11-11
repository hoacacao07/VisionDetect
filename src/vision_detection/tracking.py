"""STATIONARY-CAMERA TRACKING: identities, image velocity and rough metric motion.

Original project code; standard-library math and constant-velocity geometry.
Not a trained tracker. Identity can swap in crowded/occluded scenes. Metric
predictions inherit all distance assumptions. No ego-motion correction yet.
"""

from collections import deque
from dataclasses import dataclass, field
import math
from .camera_model import focal_length_px
from .distance import estimate_distance_m
from .motion import unmeasured_motion


def linear_velocity(samples):
    """Fit velocity to recent (capture_seconds, coordinate_1, coordinate_2).

    Image coordinates produce pixels/second; metric coordinates produce
    metres/second. Fitting a window reduces jitter but adds response lag.
    This estimates constant velocity, not acceleration or prediction confidence.
    """
    mean_time = sum(s[0] for s in samples) / len(samples)
    denominator = sum((s[0] - mean_time) ** 2 for s in samples)
    if denominator < 1e-9:
        return None
    return tuple(
        sum((s[0] - mean_time) * s[axis] for s in samples) / denominator
        for axis in (1, 2)
    )


def front_zone_interval(position, velocity, zone):
    """Constant-velocity point/rectangle intersection in (forward,left) metres.

    Returns entry/exit seconds from now, or (None,None) for no future crossing.
    This predicts a point reaching a zone, NOT physical footprint collision.
    """
    entry, exit_time = 0.0, float("inf")
    bounds = (
        (zone["near_m"], zone["far_m"]),
        (-zone["width_m"] / 2, zone["width_m"] / 2),
    )
    # Each axis supplies the time interval inside its rectangle bounds. The
    # intersection of both intervals is when the point occupies the front zone.
    # NAV-PREDICT: replace this point/constant-velocity model with object and
    # robot footprint trajectories before claiming physical collision time.
    for coordinate, speed, (lower, upper) in zip(position, velocity, bounds):
        if abs(speed) < 1e-6:
            if not lower <= coordinate <= upper:
                return None, None
            continue
        first, second = sorted(
            ((lower - coordinate) / speed, (upper - coordinate) / speed)
        )
        entry, exit_time = max(entry, first), min(exit_time, second)
        if exit_time < entry:
            return None, None
    if exit_time < 0 or entry > zone["prediction_horizon_s"]:
        return None, None
    # An unknown/indefinite exit remains null, not an infinite JSON value.
    return round(entry, 2), round(exit_time, 2) if math.isfinite(exit_time) else None


def metric_position(detection, width, height, settings, allow_size):
    """Return ((forward_m, left_m), scale_basis), or (None, None).

    Ground mode refers to a visible ground-contact point; size mode approximates
    the object's centre. These points and uncertainty differ, so they must not
    be treated as equivalent physical obstacle footprints by future navigation.
    """
    # NAV-SCALE: use calibrated depth/range fusion and a consistent robot-frame
    # reference point here. Do not assign body-centre distance to arms or feet.
    mode = settings.get("mode")
    if mode == "off" or (mode == "size" and not allow_size):
        return None, None
    if mode == "ground" and not settings.get("focal_calibration", {}).get("focal_px"):
        return None, None
    distance, _ = estimate_distance_m(
        detection["bbox_xyxy"], detection["label"], width, height, settings
    )
    if distance is None:
        return None, None
    focal = focal_length_px(settings, width)
    left, top, right, bottom = detection["bbox_xyxy"]
    ray_right = ((left + right) / 2 - width / 2) / focal
    if mode == "ground":
        pitch = math.radians(settings["camera_downward_tilt_degrees"])
        ray_down = (bottom - height / 2) / focal
        ray_forward = math.cos(pitch) - ray_down * math.sin(pitch)
        scale = distance / math.hypot(ray_forward, ray_right)
        return (scale * ray_forward, -scale * ray_right), "calibrated_ground_plane"
    ray_down = ((top + bottom) / 2 - height / 2) / focal
    forward = distance / math.sqrt(1 + ray_right**2 + ray_down**2)
    return (forward, -forward * ray_right), "assumed_object_size"


@dataclass
class Track:
    """Recent timestamped observations belonging to one tentative identity."""

    track_id: int
    label: str
    bounds: list  # Latest [left, top, right, bottom] image pixels.
    last_seen: float  # Monotonic capture time; not wall-clock/publication time.
    image_samples: deque = field(default_factory=deque)  # (time, right_px, down_px).
    metric_samples: deque = field(default_factory=deque)  # (time, forward_m, left_m).
    metric_basis: str | None = None


class StationaryCameraTracker:
    """Associate observations, retain history and estimate supported motion.

    IDs last only for the current session and can change after occlusion/loss.
    Class-based mobility is metadata; it never forces a measured speed to zero.
    """

    def __init__(self, motion_settings, distance_settings):
        """Create an empty session; settings define matching and history limits."""
        self.settings = motion_settings
        self.distance_settings = distance_settings
        self.tracks = {}
        self.next_id = 1
        self.last_timestamp = None

    def update(self, detections, timestamp, width, height):
        """Enrich current detections in place with a fresh motion record.

        timestamp must increase and use the same clock throughout the session.
        Missing detections are retained internally briefly for association, but
        are not emitted as though currently visible. Returns the input list.
        """
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            raise ValueError(
                "Tracking requires strictly increasing capture timestamps."
            )
        self.last_timestamp = timestamp
        self.tracks = {
            key: track
            for key, track in self.tracks.items()
            if timestamp - track.last_seen <= self.settings["maximum_track_gap_s"]
        }
        # Match same-label boxes by spatial distance plus overlap. Greedy matching
        # is deliberately simple and can fail when similar objects cross.
        # NAV-TRACK: improve association/re-identification and track uncertainty
        # before making movement decisions around crowds or long occlusions.
        candidates = []
        diagonal = math.hypot(width, height)
        for index, detection in enumerate(detections):
            box = detection["bbox_xyxy"]
            centre = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
            for key, track in self.tracks.items():
                if track.label != detection["label"]:
                    continue
                old = track.bounds
                old_centre = ((old[0] + old[2]) / 2, (old[1] + old[3]) / 2)
                displacement = math.dist(centre, old_centre) / diagonal
                if displacement > self.settings["association_distance_fraction"]:
                    continue
                intersection = max(0, min(box[2], old[2]) - max(box[0], old[0])) * max(
                    0, min(box[3], old[3]) - max(box[1], old[1])
                )
                union = (
                    (box[2] - box[0]) * (box[3] - box[1])
                    + (old[2] - old[0]) * (old[3] - old[1])
                    - intersection
                )
                overlap = intersection / max(union, 1)
                candidates.append((displacement + 0.1 * (1 - overlap), index, key))
        assignments, used_tracks = {}, set()
        for _, index, key in sorted(candidates):
            if index not in assignments and key not in used_tracks:
                assignments[index] = key
                used_tracks.add(key)
        for index, detection in enumerate(detections):
            if index not in assignments:
                key = self.next_id
                self.next_id += 1
                self.tracks[key] = Track(
                    key, detection["label"], detection["bbox_xyxy"], timestamp
                )
                assignments[index] = key
            track = self.tracks[assignments[index]]
            gap = timestamp - track.last_seen
            if gap > self.settings["maximum_velocity_gap_s"]:
                # Do not fit one velocity across a period with no observations.
                track.image_samples.clear()
                track.metric_samples.clear()
            track.bounds, track.last_seen = detection["bbox_xyxy"], timestamp
            left, top, right, bottom = track.bounds
            track.image_samples.append(
                (timestamp, (left + right) / 2, (top + bottom) / 2)
            )
            position, basis = metric_position(
                detection,
                width,
                height,
                self.distance_settings,
                self.settings["allow_size_based_speed"],
            )
            if position is None or basis != track.metric_basis:
                # A scale loss/change invalidates the old metric velocity; image
                # history may still support a pixels/second result independently.
                track.metric_samples.clear()
            track.metric_basis = basis
            if position is not None:
                track.metric_samples.append(
                    (timestamp, *self.metric_sample_position(position))
                )
            for samples in (track.image_samples, track.metric_samples):
                while (
                    samples
                    and timestamp - samples[0][0] > self.settings["velocity_window_s"]
                ):
                    samples.popleft()
            detection["motion"] = self.describe(track, position)
            detection["motion_status"] = detection["motion"]["status"]
        return detections

    def metric_sample_position(self, position):
        """Stationary camera: local metric positions share a fixed origin."""
        return position

    def ready(self, samples):
        """Require both sample count and elapsed time, not frame count alone."""
        return (
            len(samples) >= self.settings["minimum_samples"]
            and samples[-1][0] - samples[0][0] >= self.settings["minimum_observation_s"]
        )

    def describe(self, track, position):
        """Build one observation's motion output, keeping unknown fields null.

        Progresses from collecting history -> image velocity -> metric velocity
        -> optional zone forecast. Each stage depends on the previous evidence.
        """
        result = unmeasured_motion()
        result.update(
            track_id=track.track_id,
            status="collecting_history",
            camera_assumption="stationary",
            metric_basis=track.metric_basis,
            speed_is_estimated=True,
            image_velocity_px_s=None,
            image_speed_px_s=None,
            position_m=None,
        )
        if not self.ready(track.image_samples):
            result["prediction_status"] = "collecting_history"
            return result
        image_velocity = linear_velocity(track.image_samples)
        image_speed = math.hypot(*image_velocity)
        result.update(
            image_velocity_px_s={
                "right": round(image_velocity[0], 1),
                "down": round(image_velocity[1], 1),
            },
            image_speed_px_s=round(image_speed, 1),
            status=(
                "moving_in_image"
                if image_speed >= self.settings["image_motion_threshold_px_s"]
                else "little_image_motion"
            ),
            prediction_status="metric_position_unavailable",
        )
        if position is None or not self.ready(track.metric_samples):
            return result
        velocity = linear_velocity(track.metric_samples)
        speed = math.hypot(*velocity)
        separation = math.hypot(*position)
        closing_speed = -sum(p * v for p, v in zip(position, velocity)) / max(
            separation, 0.001
        )
        # Negative dot(position, velocity) means decreasing separation. A large
        # total speed alone does not mean the object is approaching the camera.
        # NAV-EGO: object_speed currently equals relative planar speed ONLY
        # because this implementation assumes the camera is stationary.
        result.update(
            position_m={
                "forward": round(position[0], 2),
                "left": round(position[1], 2),
            },
            relative_velocity_mps={
                "forward": round(velocity[0], 2),
                "left": round(velocity[1], 2),
            },
            object_speed_mps=round(speed, 2),
            closing_speed_mps=round(closing_speed, 2),
            status=(
                "moving_estimate"
                if speed >= self.settings["metric_motion_threshold_mps"]
                else "approximately_stationary"
            ),
        )
        if speed < self.settings["metric_motion_threshold_mps"]:
            # Suppress unstable far-future forecasts driven by tiny box jitter.
            # This threshold is not a statistical motion confidence estimate.
            zone = self.settings["front_zone"]
            if (
                zone["near_m"] <= position[0] <= zone["far_m"]
                and abs(position[1]) <= zone["width_m"] / 2
            ):
                result.update(
                    front_zone_entry_s=0.0,
                    prediction_status="already_inside_front_zone",
                )
                return result
            result.update(prediction_status="below_motion_threshold")
            return result
        entry, exit_time = front_zone_interval(
            position, velocity, self.settings["front_zone"]
        )
        result.update(
            front_zone_entry_s=entry,
            front_zone_exit_s=exit_time,
            prediction_status=(
                "constant_velocity_estimate"
                if entry is not None
                else "no_front_zone_entry_within_horizon"
            ),
        )
        # Physical time to contact stays null: no object footprint/robot trajectory
        # model exists yet. Front-zone arrival is not automatically collision time.
        return result
