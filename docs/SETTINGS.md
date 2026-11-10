# Settings reference

[← Project home](../README.md) · [Code guide](EDITING_GUIDE.md)

Edit **`config/settings.json`**. It is divided into five groups. Save, stop the camera with **Q**, then restart to apply changes. JSON does not allow comments; this reference and editor hover descriptions explain the fields. Simulation movement thresholds are separate in `simulation/navigation_settings.json`; see [the moving-robot guide](../simulation/MOVING_ROBOT.md).

## Camera · where images come from

| Field | Meaning |
| --- | --- |
| `camera.index` | Integer camera number: 0 is usually built-in; 1 may be a USB camera. |
| `camera.width`, `camera.height` | Requested image dimensions in pixels. Hardware can select a different supported resolution; output shows the actual dimensions. |

## Detection · what the model does

| Field | Meaning |
| --- | --- |
| `detection.model` | Prepared weights or exported model directory, relative to the project root. Currently `models/outdoor_detector_small_openvino_model`. |
| `detection.image_size` | Model input size: currently 640 for consistency in the wide view. Lower sizes can be faster but must be checked for missed small objects. |
| `detection.minimum_confidence` | Minimum label confidence, 0–1. Higher suppresses weak detections; lower includes more tentative guesses. |
| `detection.cpu_threads` | Maximum inference CPU threads, bounded by available logical CPUs. Default 4. |
| `detection.human_model` | Optional people specialist checkpoint or exported directory. Currently empty: the selected model handles all categories. Prepare the optional specialist with `tools/prepare_human_model.py`. |
| `detection.human_minimum_confidence` | Minimum people-specialist confidence, currently 0.30. Lower values can increase false positives. Duplicate people are merged with the outdoor detections. |

## Distance · assumptions, calibration and limits

| Field | Meaning |
| --- | --- |
| `distance.mode` | `size`, `ground`, or `off`. See the [distance guide](DISTANCE_GUIDE.md). |
| `distance.horizontal_fov_degrees` | Assumed horizontal field of view when calibrated focal length is absent. Default 65 is unverified. |
| `distance.assumed_object_heights_m` | Label → assumed full upright height. Defaults: person 1.7 m, car 1.5 m. No default tree size. |
| `distance.camera_height_m` | Lens height above level ground; required for ground mode. `null` = not set. |
| `distance.camera_downward_tilt_degrees` | Ground-mode angle below horizontal: 0 is level, positive points down. `null` = not set. |
| `distance.focal_calibration` | Empty until calibrated; stores `focal_px` and the calibration `image_width`. |
| `distance.maximum_distance_m` | Reporting limit: default 30 m. Not demonstrated accuracy or usable sensor range. |

## Output · where results appear

| Field | Meaning |
| --- | --- |
| `output.api_port` | Local viewer/API port, currently 8879. Change the browser URL if you change it. |
| `output.write_detection_log` | Save metadata JSONL in results when true. Neither setting records video. |

Model classes are configured separately in **`config/outdoor_objects.json`**. Rerun `tools/prepare_model.py` after changing those groups.

Alternative visual descriptions are in **`config/detection_prompts.json`**. They map to the existing canonical labels. For the selected model, run `tools/prepare_model.py --size small`, then `tools/optimize_for_laptop.py`, after changing categories or descriptions. Preparation requires the original checkpoint/text encoder and configures existing weights rather than training them.

## Motion · stationary-camera demo

The `motion` group controls track history, matching, optional rough size-based metric speeds, movement thresholds and the example front zone. See [Motion settings and on-screen predictions](MOTION_AND_NAVIGATION.md#motion-settings) for every field. Keep `camera_stationary` true; moving-robot compensation is future work.

## Multiple cameras and movement limits

See [START_HERE.md](START_HERE.md) for camera drivers, stream sources, backend selection, requested FPS, freshness timeouts and per-camera calibration. Full example configurations are in `config/cameras/`; select a file with `--config`.

Movement thresholds remain in `simulation/navigation_settings.json`, with per-field hover descriptions supplied by `navigation_settings.schema.json`. Startup validates positive/finite values and `stop_gap_m < resume_gap_m <= slow_gap_m`. Motor ramps and sensor timing are in `simulation/realism_settings.json`. Restart after edits.
