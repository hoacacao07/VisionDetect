# Webots camera test

For a robot that moves straight and responds to obstacles, use **Start Moving Robot Simulation.cmd** and read [the moving-robot guide](MOVING_ROBOT.md). The launcher described below retains the fixed-camera accuracy experiment.

Run **Start Webots Simulation.cmd** in the project folder. Requires Webots R2025a and the existing project Python environment. The first load may download Cyberbotics assets. Close this test's Webots window before launching another copy. The laptop webcam is not used.

The separate world uses a fixed RGB camera, the same pretrained detector, tracking, distance formulas, category colors and camera-blocked alert as the webcam application. It does not change the original e-puck world or drive wheels.

## What the test does

| Simulated time | Experiment |
| --- | --- |
| 0–12 seconds | Person approaches at 0.30 m/s |
| 12–24 seconds | Person crosses at 0.40 m/s |
| 24–36 seconds | Person stands still |
| 36–40 seconds | An opaque panel covers the camera |

A parked car and tree provide additional scene objects. These are not independently scored in the current report. The pedestrian slides along scripted paths; this is not a gait or collision-avoidance simulation.

The preview displays predictions. While running, the dashboard is at http://127.0.0.1:8880. Press Q in the preview to stop early. The test automatically pauses and closes the preview/server after one cycle; Webots stays open. Reset/reload the world to rerun.

## Read the results

- `results/latest_summary.json`: person detection rate, distance/speed absolute errors and camera alert rate per experiment.
- `results/latest_run.jsonl`: each prediction alongside a separate `evaluation_only` reference record.
- `results/*_preview.jpg`: saved annotated images for reviewing successes and misses.

**Missed detections count in detection rate.** Distance and speed errors include only available predictions; low error alone does not mean the detector passed. Null means no estimate. The first two seconds of each experiment are excluded from aggregate measurements to allow tracking and alert warm-up. Speed uses simulation timestamps, so a slow laptop does not artificially change the scripted speed.

Only camera pixels enter recognition. Known camera height, pitch and field of view calibrate ground-plane distance. Scripted object positions and velocities are used separately for evaluation, never supplied as detector answers. Calibration assumes upright objects touching flat ground; boxes and synthetic appearance still introduce error. Front-zone arrival is displayed by the existing tracker but is not yet independently scored.

This is a diagnostic test, not proof of safe navigation. Initial testing found missed person detections in the rendered scene, including the approaching and stationary phases. The larger people specialist now improves coverage; see `results/detection_comparison.json` for a same-image comparison. The initial report is historical. The moving test adds straight-line wheel-encoder compensation, not general camera-motion estimation or turning.

## Edit or extend

- `worlds/vision_test.wbt`: camera and scene assets. If camera pose changes, update the matching calibration in the controller.
- `controllers/vision_test/vision_test.py`: scenario paths, calibration, image adapter and evaluation. Comments identify the boundary between predictions and reference data.
- `tools/start_webots_simulation.py` in the main project: locates Webots and sets the controller's local Python path.

## Credits and licenses

Simulation engine and scene assets: [Cyberbotics Webots R2025a](https://github.com/cyberbotics/webots/tree/R2025a). The world references upstream [Pedestrian](https://github.com/cyberbotics/webots/blob/R2025a/projects/humans/pedestrian/protos/Pedestrian.proto), [ToyotaPriusSimple](https://github.com/cyberbotics/webots/blob/R2025a/projects/vehicles/protos/toyota/ToyotaPriusSimple.proto) and [SimpleTree](https://github.com/cyberbotics/webots/blob/R2025a/projects/objects/trees/protos/SimpleTree.proto). The moving world also uses [TexturedBackground](https://github.com/cyberbotics/webots/blob/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto) and [Asphalt](https://github.com/cyberbotics/webots/blob/R2025a/projects/appearances/protos/Asphalt.proto). Assets retain their upstream notices and [Webots assets license](https://cyberbotics.com/webots_assets_license); Asphalt's PROTO declares Apache-2.0. These external assets are not relicensed under the project's AGPL license.

The walking-animation field-control approach follows [Cyberbotics' pedestrian controller](https://github.com/cyberbotics/webots/blob/R2025a/projects/humans/pedestrian/controllers/pedestrian/pedestrian.py), Apache-2.0. The local sinusoidal test gait is not a copied gait table or biomechanical model.

Recognition uses the existing [Ultralytics YOLOE foundation and supporting libraries](../docs/SOURCE_CREDITS.md). The camera adapter, scenarios and evaluation harness were added for this project; no detector was trained on this scene.
