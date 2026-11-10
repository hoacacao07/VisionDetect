# Navigation & avoidance extension map

[← Project home](../README.md) · [Code guide](EDITING_GUIDE.md) · [Current motion behavior](MOTION_AND_NAVIGATION.md)

This maps future work to specific files. The webcam program remains a **stationary-camera perception prototype**. The separate [Webots moving test](../simulation/MOVING_ROBOT.md) adds straight-line encoder compensation and slow/stop/resume control of simulated wheels. Search **`NAV-`** in VS Code to find extension comments.

## Current data flow

```text
camera.py          → image + capture time
camera_health.py   → visibility warning
detector.py        → labels + boxes + catalog meaning
distance.py        → approximate distance or unknown
tracking.py        → IDs → velocity estimates → example-zone arrival
runtime.py         → timestamped observations + camera health
                    ├── display.py / web/  presentation
                    ├── server.py         local JSON interface
                    └── results/          optional metadata logs
```

`object_library.py` owns editable category/mobility definitions. `motion.py` supplies unknown/default motion fields. `settings.py` owns configuration. Neither the object catalog nor the display decides whether the robot should move.

## Where future changes belong

| Search marker | File | Future change | Verification needed |
| --- | --- | --- | --- |
| `NAV-INPUT` | `camera.py` | Pi/CSI/depth input with synchronized capture times and calibration metadata. | Resolution, timestamp ordering, source loss and recovery. |
| `NAV-SCALE` | `distance.py`, `tracking.py` | Calibrated depth/range fusion, consistent obstacle reference points and uncertainty. | Compare with real measured distances, including partial/occluded objects. |
| `NAV-PERCEPTION` | `detector.py` | Keep labels/boxes distinct from spatial occupancy and path-obstruction decisions. | Confirm the controller uses geometry, health and uncertainty rather than label alone. |
| `NAV-TRACK` | `tracking.py` | Robust identity association/re-identification and track quality. | Crossing similar objects, occlusions, missed detections and identity swaps. |
| `NAV-EGO` | `runtime.py`, `tracking.py` | Fuse timestamped robot odometry/IMU; transform observations before estimating world motion. | Fixed objects should retain near-zero world speed as the robot moves and turns. |
| `NAV-PREDICT` | `tracking.py` | Replace point/constant-velocity prediction with object/robot footprints, planned trajectories and uncertainty. | Approaching, crossing, separating, overlapping and no-collision paths. |
| `NAV-CONTROL` | `runtime.py` | Connect a separate controller module/process for slow/stop/steer policy. | Stale data, camera obstruction, unknown distances and controller timeouts. |
| `NAV-HEALTH` | `camera_health.py`, `runtime.py` | Unified visibility, missing/stale/frozen stream and recovery handling. | Input failures independently of whether objects are detected. |
| `NAV-OUTPUT` | `server.py` | Bounded-age observation transport and a controller heartbeat/watchdog. | Reject old snapshots and handle detector disappearance. |
| `NAV-CONFIG` | `settings.py` | Explicit mounting, robot dimensions, clearance and braking parameters. | Validate units/limits; do not reuse the example zone as robot geometry. |
| `NAV-SCHEMA` | `motion.py` | Extend output without changing unit/reference-frame meaning. | Consumer compatibility and unknown-versus-zero semantics. |
| `NAV-DISPLAY` | `display.py` | Visualize future paths and footprints produced by other modules. | UI changes must not change control decisions. |
| `NAV-DEPLOY` | `tools/export_for_pi.py` | Benchmark the complete Pi pipeline under load. | End-to-end latency and stale-data behavior, not just model inference. |

## Assumptions future navigation must address

- **Coordinates:** image right/down in pixels; metric estimates use camera-relative forward/left. Size-mode centre points and ground-mode base points are different approximations.
- **Motion:** object speed currently equals relative planar speed only because the camera is stationary. Subtracting one wheel-speed number does not account for turning, lateral motion, sensor mounting or acceleration.
- **Timing:** tracking uses monotonic capture time. `timestamp_utc` is publication time and `capture_age_ms` is age at publication. A consumer must also reject results that have become stale afterward.
- **Geometry:** the front zone is a camera-relative example, not a robot footprint. Its entry time is not physical contact time. `time_to_contact_s` stays null until a collision model supplies it.
- **Unknown results:** null is not zero; an empty detection list during an obstructed/failed view is not permission to drive.
- **Class defaults:** could-move/usually-stationary categories are not measured motion or avoidance policy.

## Suggested order

1. Calibrate actual camera/range input and mounting transforms.
2. Validate tracking, metric positions and uncertainty in intended scenes.
3. Integrate synchronized robot motion and distinguish world/relative velocity.
4. Add footprint/trajectory occupancy and collision prediction.
5. Build the separate navigation policy with observation freshness checks.
6. Integrate wheel control after exercising the prediction and failure cases.

General navigation, steering, physical motor control and collision-trajectory prediction remain extension work. The limited Webots policy is in `longitudinal_control.py`; straight-line compensation is in `ego_motion.py`, and simulated sensing/timing is in the Webots controller and `sensor_timing.py`. Original-source credits remain in [SOURCE_CREDITS.md](SOURCE_CREDITS.md).
