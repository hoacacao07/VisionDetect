# Code & editing guide

[← Project home](../README.md) · [Settings](SETTINGS.md) · [Distance guide](DISTANCE_GUIDE.md)

For the common edits and exact stopping/slowing units, begin with [START_HERE.md](START_HERE.md). Camera profiles are independent full settings files; select one in the VS Code task. Navigation limits have hover descriptions and startup validation.

## Read the project in this order

1. **`app.py`** — a small launcher that finds the source folder.
2. **`src/vision_detection/runtime.py`** — the complete six-stage flow, from startup to cleanup.
3. Open the module for the part you want to change. Each has a short responsibility statement at the top.

## One responsibility per module

| Module | Owns | Does not own |
| --- | --- | --- |
| `camera.py` | Camera connection, background capture, newest frame | Recognition, distance, motor decisions |
| `camera_health.py` | Debounced visibility warning and alert banner | Proof of a physical cover or navigation permission |
| `detector.py` | Model loading, label/box prediction, observation records | Drawing, camera hardware, logging |
| `perception_fusion.py` | Duplicate people and strongly conflicting overlapping labels | Training or movement decisions |
| `prompt_library.py` | Visual description aliases mapped to stable categories | Training or simulator truth |
| `detection_continuity.py` | Brief explicitly labelled track predictions | New detections or fabricated measurements |
| `sensor_timing.py` | Delayed-result availability and command ramps | Recognition or scene truth |
| `ego_motion.py` | Straight-heading wheel-encoder compensation | Turning, SLAM or simulator truth |
| `longitudinal_control.py` | Simulation slow/stop/resume policy | Steering or physical motor drivers |
| `navigation_display.py` | Simulation movement-status side panel | Control decisions |
| `distance.py` | Size and ground-plane formulas, rejection reasons | Model loading or physical sensor measurements |
| `object_library.py` | Validated labels, categories and mobility defaults | Measured movement or hazard decisions |
| `motion.py` | Shared unknown/default motion record | Tracking or motor commands |
| `tracking.py` | Session IDs, velocity fitting and example-zone prediction | Moving-robot compensation or physical contact prediction |
| `display.py` | Native camera overlay and window controls | Detection or distance calculations |
| `server.py` | Thread-safe snapshots, dashboard files, read-only local API | Arbitrary project files or remote control |
| `settings.py` | Project paths, grouped settings, common input validation | Live detection |
| `runtime.py` | Connects the stages, metadata logs and cleanup | Detailed mathematics or page layout |

## Change one thing at a time

For the moving simulation, start with [the moving-robot guide](../simulation/MOVING_ROBOT.md). It identifies the scene, robot dimensions, control thresholds and encoder interface separately. The webcam application remains a stationary-camera viewer.

| Your change | Edit | Then |
| --- | --- | --- |
| Camera adapter / source / resolution | `config/settings.json` → `camera` | Restart; recalibrate if optics/crop changed |
| Detection threshold / speed | settings → `detection` | Restart and compare missed/false detections |
| Distance assumptions | settings → `distance` | Restart and validate at measured distances |
| Target object categories | `config/outdoor_objects.json` → `motion_groups` → `categories` | Rerun preparation, then restart |
| Distance formulas | `src/vision_detection/distance.py` | Run tests, then validate measurements |
| Matching / speed / arrival logic | `src/vision_detection/tracking.py` | Test synthetic paths and real camera observations |
| Covered / dark camera warning | `src/vision_detection/camera_health.py` | Check alert delay and visibility recovery |
| Camera overlay | `src/vision_detection/display.py` | Restart detector |
| Browser appearance | `web/styles.css` | Refresh browser |
| Browser structure / headings | `web/index.html` | Refresh browser |
| Cards / browser refresh logic | `web/viewer.js` | Refresh browser |
| Pi model export | `tools/export_for_pi.py` | Export, then benchmark on the Pi |

## Helper tools

Run from the project root with this project's interpreter:

```powershell
# Configure the pretrained model's target classes (internet may be needed).
.\.venv\Scripts\python.exe tools\prepare_model.py

# Calculate focal length from your own measurements; see the distance guide.
.\.venv\Scripts\python.exe tools\calibrate_camera.py --help

# Optional Pi-oriented model export.
.\.venv\Scripts\python.exe tools\export_for_pi.py
```

Do not edit `.venv/`, downloaded model binaries, or generated results to change behavior.

## Conventions to preserve

- `_px` = pixels; `_m` = metres; `_ms` = milliseconds. Angles say degrees or radians.
- A box is `[left, top, right, bottom]`. Images are not mirrored. Horizontal offset is −1 at the left through +1 at the right.
- OpenCV colors are BGR, unlike browser CSS colors.
- Distance `None` in Python becomes `null` in JSON. Unknown is never converted to zero.
- Capture freshness uses a monotonic clock; JSON `timestamp_utc` records publication time.
- Label confidence is not distance confidence.
- Camera capture keeps only the newest frame. A lock protects snapshots shared with browser threads.
- `finally` releases the camera, metadata log and HTTP port even after errors.

## Future wheel-controller interface

Start with the [navigation extension map](NAVIGATION_ROADMAP.md). Search **`NAV-`** in source for specific integration points, assumptions and verification criteria.

Read `/api/detections` or the JSONL records. Preserve the existing field names when extending the program so a future Pi consumer stays compatible. Check timestamps and handle missing/unknown observations. Motor policy belongs in a separate controller; this detector only reports observations.

## Verify an edit

Stop the old camera window with **Q**, run the VS Code check task, then run the detector again. Do not launch two detector instances against the same camera/port.


## Object mobility and future navigation

`object_library.py` loads and validates the editable label/category/mobility catalog. `motion.py` defines unavailable defaults; `tracking.py` fills track IDs, image speeds and supported metric/zone estimates for the stationary camera. See [Motion & navigation](MOTION_AND_NAVIGATION.md) before changing tracking or prediction. Adding labels requires model preparation; regrouping existing labels only needs a restart. No Python changes are needed for a new category.
