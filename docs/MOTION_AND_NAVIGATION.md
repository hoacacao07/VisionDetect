# Stationary-camera tracking & on-screen predictions

[← Project home](../README.md) · [Object library](OUTDOOR_OBJECTS.md) · [Distance calibration](DISTANCE_GUIDE.md)

## What you see beside each box

1. **Track ID + label + estimated distance.** An ID associates an object across frames; it is not a permanent identity.
2. **Speed:** rough metres/second if usable metric positions exist, otherwise pixels/second. A partial person often has no usable full-height estimate.
3. **Time to the front zone:** estimated entry under a constant-velocity assumption. Unknown or insufficient data is explicitly labeled.

The example front zone is **1 m wide, from 0.5 to 2 m forward of the camera**. These are configurable demonstration dimensions, not a measured robot footprint. Time to this zone is not physical collision time. `time_to_contact_s` remains null until a footprint/collision model exists.

## Test it on the laptop

1. Keep the camera and laptop still; opening/tilting the lid changes the geometry.
2. Run **Start Webcam.cmd**. Open the physical webcam shutter if present.
3. Stand far enough away for your full body to fit in the image. Walk across the view, then slowly toward it.
4. Wait about **0.6 seconds** for each new track. An image-speed result can appear even when metre-based speed is unavailable.
5. Compare with known distances/timing before judging accuracy. A close-up face, a cropped body, or a changing pose cannot provide a reliable full-height distance.

The default size mode assumes a 1.7 m person or 1.5 m car and an unverified camera field of view. Metric speeds and arrival times derived from it are **rough scale-dependent estimates**, not calibrated measurements. No height or angle was guessed from the ThinkPad brand. Set `motion.allow_size_based_speed` to false to require calibrated ground-mode scale for metric speed. Ground mode needs measured camera height/angle and focal calibration; it supports other visible object bases too.

## How tracking works

`tracking.py` associates nearby same-label boxes, retains their capture timestamps, and fits velocity over a recent observation window. It reports horizontal image motion separately from estimated forward/left metric motion. Every object is tracked, including usually-static classes: the class group does not determine actual movement.

Identity can swap when similar objects overlap or cross. Confidence jitter, size/pose changes, occlusion, missed detections and wrong scale can corrupt velocity. A detection gap resets velocity history; expired tracks get new IDs. This is a simple prototype, not a validated navigation tracker.

## Output fields

| Field in `motion` | Meaning |
| --- | --- |
| `track_id` | Current track identity; may change after loss/occlusion. |
| `status` | Collecting history, image motion, estimated metric motion, or approximately stationary. |
| `image_velocity_px_s` | Right/down image velocities, pixels per second. |
| `image_speed_px_s` | Image-plane speed magnitude; not physical object speed. |
| `position_m` | Estimated `{forward,left}` position relative to the fixed camera. |
| `object_speed_mps` | Estimated planar speed for the stationary-camera assumption. |
| `relative_velocity_mps` | Estimated forward/left velocity relative to this stationary camera. |
| `closing_speed_mps` | Positive means separation decreases; negative means separation increases. |
| `metric_basis` | `assumed_object_size`, `calibrated_ground_plane`, or null. |
| `speed_is_estimated` | True; metric speeds inherit depth/scale assumptions. |
| `front_zone_entry_s`, `front_zone_exit_s` | Constant-velocity point entry/exit predictions in seconds from the current observation. |
| `time_to_contact_s` | Null: physical contact is not modeled. |
| `prediction_status` | Why a forecast is available or unavailable. |

Unknown values are null, never assumed zero. No predicted zone entry within the configured horizon is not a guarantee of a clear path. Capture timestamps, rather than processing durations, determine velocity. The JSON publication timestamp and capture age are reported separately.

## Motion settings

In `config/settings.json`, the `motion` group contains:

- `camera_stationary`: must be true. The program rejects false until ego-motion compensation exists.
- `allow_size_based_speed`: allow rough scale estimates from configured heights; default true.
- `minimum_samples`, `minimum_observation_s`: wait before calculating velocity.
- `velocity_window_s`: recent history used for velocity fitting.
- `maximum_track_gap_s`: expire a missing track; `maximum_velocity_gap_s`: clear its velocity history after a shorter gap.
- `association_distance_fraction`: matching displacement limit relative to image diagonal.
- `image_motion_threshold_px_s`, `metric_motion_threshold_mps`: state/noise thresholds; small metric motion does not trigger distant forecasts.
- `front_zone`: `near_m`, `far_m`, `width_m` and `prediction_horizon_s`.

## Later: a robot with changing speed

A moving camera changes all relative measurements. Add timestamped wheel odometry/IMU or camera-motion estimates, rotate/translate observations into a consistent frame, estimate object world motion, and predict the robot's own future trajectory. Then perform footprint/trajectory intersection for actual time-to-contact and navigation decisions. Merely subtracting one robot speed number does not compensate for turning, lateral motion or acceleration.

The webcam viewer sends no wheel commands. The separate [Webots test](../simulation/MOVING_ROBOT.md) now uses measured encoder travel for straight-line compensation and a slow/stop/resume policy on simulated motors. Turning, full camera-pose estimation and physical collision-trajectory prediction remain future work.
