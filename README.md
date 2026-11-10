# Vision Lab

> **Built on [Ultralytics YOLOE](https://docs.ultralytics.com/models/yoloe/)** — [original open-source repository](https://github.com/ultralytics/ultralytics).
> The pretrained object detector and inference library come from Ultralytics. The webcam pipeline, distance-estimation formulas, dashboard and project organization were added for this project. **The detection model was not trained from scratch here.** See [source credits](docs/SOURCE_CREDITS.md) for the exact model and license.

**Editing and setup:** [Start here: cameras, slowing and stopping](docs/START_HERE.md). Open **VisionDetection.code-workspace** in VS Code.

**A local webcam prototype for recognizing common outdoor obstacles.**

Start here. Everyday settings are in `config/`; application code is in `src/`; deeper explanations are in `docs/`.

## 1 · Run it

Windows terminal: `.\"Start Webcam.cmd"` from this project folder. No environment activation is required. See [terminal startup help](docs/START_HERE.md#start-the-webcam-from-the-windows-terminal).

Double-click **Start Webcam.cmd**, or use **Terminal → Run Task → Start laptop camera** in VS Code.

Open the [live dashboard](http://127.0.0.1:8879) while the program is running. Press **Q** in the camera window to stop and release the webcam. Run one instance at a time.

The native camera view now displays **track ID, label, distance, speed and time to the example front zone** next to each box. Keep the laptop camera stationary. Speed starts after roughly 0.6 seconds of history. Pixels/second is available without scale; rough metres/second requires a usable configured object size or calibrated ground mode.

**Webots tests:** double-click **Start Moving Robot Simulation.cmd** to watch a robot move straight, slow down, stop for obstacles and resume after clearance. See the [moving-robot guide](simulation/MOVING_ROBOT.md). **Start Webots Simulation.cmd** runs the separate fixed-camera accuracy test. Run one simulation at a time.

Recognition now uses the larger **YOLOE small** outdoor model, optimized with OpenVINO for the laptop CPU. It reports 48 categories using 57 visual descriptions, with stable output names. Optional people-specialist support remains available but is disabled in the selected configuration. Model paths and thresholds are in `config/settings.json` → `detection`; alternative descriptions are in `config/detection_prompts.json`. See [credits](docs/SOURCE_CREDITS.md).

Short recognition gaps can display a box marked **PREDICTED** for up to 0.35 seconds. It has no fresh distance or speed measurement and does not count as a successful detection in simulation evaluation. A visible object can still be missed or misclassified.

## 2 · Understand the flow

```text
 CAMERA INPUT        RECOGNITION          DISTANCE            LOCAL OUTPUT
 Webcam image   →    Pretrained YOLOE  →  Estimate + status →  Preview + JSON
 camera.py           detector.py          distance.py         display.py / server.py
```

The camera sees pixels. The pretrained model proposes labels and boxes. Distance formulas add an estimate when their assumptions allow it. The viewer presents observations; it does not control wheels.

See the [outdoor object library](docs/OUTDOOR_OBJECTS.md) for the full 48-category list, distance behavior, and coverage limits.

Objects are also organized by **could move / usually stationary**. [Motion & navigation](docs/MOTION_AND_NAVIGATION.md) explains stationary-camera tracking, estimated speeds and front-zone arrival predictions.

## 3 · Find the right part

**For future navigation work:** open the [navigation extension map](docs/NAVIGATION_ROADMAP.md), then search **`NAV-`** in the source to find the exact places to extend.

| What you want to do | Where to go |
| :--- | :--- |
| Start the project | `app.py` or **Start Webcam.cmd** |
| Change camera, sensitivity or distance mode | [config/settings.json](config/settings.json) |
| Understand every setting | [Settings reference](docs/SETTINGS.md) |
| Change how a processing stage works | [Code & editing guide](docs/EDITING_GUIDE.md) |
| Set up webcam distance estimation | [Distance & calibration](docs/DISTANCE_GUIDE.md) |
| Plan the Raspberry Pi transfer | [Pi deployment notes](docs/RASPBERRY_PI.md) |
| Inspect current observations | [Live JSON](http://127.0.0.1:8879/api/detections) |

## 4 · Project layout

```text
VisionDetection/
├── app.py                  ← small starting point
├── Start Webcam.cmd        ← Windows launcher
├── config/                 ← grouped, editable settings
├── src/vision_detection/   ← one module for each responsibility
├── web/                    ← dashboard: HTML, CSS and JavaScript
├── tools/                  ← model preparation, calibration, Pi export
├── tests/                  ← geometry and configuration checks
├── docs/                   ← focused guides and references
├── models/                 ← downloaded/prepared weights
└── results/                ← generated detection metadata
```

Installed dependencies and caches are hidden in the VS Code Explorer so you can focus on your own files. They are still present on disk.

## What is working / what remains approximate

- **Recognition:** pretrained YOLOE-26n, configured with 46 outdoor-object prompts. No new training was performed. These predictions need validation in your intended environment.
- **Distance:** webcam estimates, not sensor measurements. Size mode initially assumes a 1.7 m person, 1.5 m car and unverified 65° camera field of view. Trees need ground mode and calibration. A distance of `null` means unknown.
- **Output:** local preview, local API, optional metadata logs. No video recording, remote transmission or wheel commands.
- **Pi:** the code is structured for later transfer; performance on your Pi has not yet been measured.

## Check changes

Use **Terminal → Run Task → Check distance calculations**. This runs geometry and settings tests. Then test the changed behavior on the camera. Mathematical tests do not establish real-world detection accuracy.

## Open-source foundation

[Ultralytics YOLOE](https://docs.ultralytics.com/models/yoloe/) supplies the pretrained detector. The surrounding webcam, distance and display program is this project's code. Upstream and project source are supplied under AGPL-3.0; see [LICENSE](LICENSE) and [upstream license information](https://www.ultralytics.com/license).


## Blocked / dark camera alert

The camera window shows a red **ALERT: CAMERA BLOCKED / TOO DARK** banner after about one second of nearly black frames. A sustained featureless image shows **ALERT: CAMERA VIEW MAY BE BLOCKED**. The warning clears after a usable view returns. During the alert, object detections are withheld and tracking history is cleared rather than presenting stale observations.

This is an image-quality heuristic: darkness can resemble a lens cover, and a blank wall can resemble an opaque obstruction. It does not detect every partial obstruction. A disconnected camera is reported separately as an input failure.

Implementation: `src/vision_detection/camera_health.py`. No additional model or dependency is used.

## Category colors

The camera preview uses **orange boxes/tinted areas for potentially moving categories** and **blue for usually-stationary categories**, with an on-screen legend. Gray is an unknown category. Browser cards use matching side borders. The tint is the detector's rectangular box, not an object silhouette, physical clearance zone or proof of movement. Actual estimated motion remains in the text/JSON. Colors can be changed in `display.py` → `MOBILITY_COLORS`; browser colors are in `web/styles.css`.
