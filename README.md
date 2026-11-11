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
| Drive the real robot (Pi + ESP32) | [Delivery robot section](#delivery-robot-pi--esp32) · `robot.py` · `firmware/esp32_robot/` |
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

## Delivery robot (Pi + ESP32)

`robot.py` drives the real robot. It is the hardware version of the Webots moving test: it uses the same detector, `StraightLineTracker`, `StopGoController`, speed ramp, camera-health check and dashboard.

**What it can do:**
- drive forward in a straight line
- slow down, stop and resume for obstacles

**What it can't do yet:** steer, or follow a route.

**No wheel encoders.** Motor speed is open-loop: the ESP32 turns the requested speed into a PWM duty using a calibrated constant, and reports the travel it *expects* from that. Robot speed and travel are therefore **estimates**. The gyro keeps the robot straight, and the stop distance is computed for a speed `sensors.speed_estimate_margin` (default 30 %) higher than estimated. A stalled or slipping wheel is not detected.

```text
 Raspberry Pi 5                                        ESP32 DevKit V1
 ─────────────────────────────────────────             ────────────────────────────────
 IMX708 ─▶ PerceptionWorker thread                     open-loop speed (no encoders)
           health → YOLOE (NCNN) → tracker ◀── est. travel ── duty + gyro heading hold ─▶ 2 × L298N ─▶ 4 TT motors
 control loop 20 Hz: freshness checks +                3 × HC-SR04 (core 0, 30 ms apart)
   StopGoController → ramp ──── C,seq,L,R ──USB──▶     MPU-6050 gyro Z + heading
                      ◀──── T,ms,estL_m,estR_m,ranges,gyro,heading (every 50 ms)
                                                       WATCHDOG: no valid C for 300 ms → brake
```

**Who does what:**
- The Pi chooses a target speed.
- The ESP32 drives the wheels.
- If the Pi crashes, the USB cable is unplugged, or a malformed line arrives, the ESP32 brakes on its own.

**What forces a STOP:** any missing or stale input. That covers:
- no fresh vision
- a range reading that is missing, stale or out of range
- stale ESP32 telemetry
- no gyro or speed estimate
- a blocked camera
- no odometry for the current frame

**Files:**

| File | Role |
| --- | --- |
| `robot.py` | Entry point |
| `src/vision_detection/robot_runtime.py` | Perception thread, control loop, dashboard |
| `src/vision_detection/robot_control.py` | Freshness checks, policy and ramp (unit-tested) |
| `src/vision_detection/esp32_link.py` | Serial protocol, telemetry, odometry |
| `config/robot_navigation.json` | Serial port, sensor ages, speed-estimate margin, speeds and gaps |
| `config/cameras/robot.json` | Pi camera, model and calibration |
| `firmware/esp32_robot/` | PlatformIO firmware. Pins are in `include/pins.h`; constants in `include/robot_config.h` |
| `tools/esp32_console.py` | Bench test of the ESP32 without the camera |

### Pin map and wiring

**Full wiring, power and safety notes: [WIRING.md](WIRING.md).** Read it before connecting anything. In short:

- **Echo lines need dividers.** The HC-SR04 runs on 5 V and its Echo outputs 5 V, so every Echo line needs a **1 kΩ / 2 kΩ divider** before the ESP32.
- **Motors get their own battery** (2S Li-ion or 6 × AA) through the L298N VS terminals, with the 5V-EN jumper **on**.
- **One common ground** for everything.

| Function | GPIO (30-pin ESP32-WROOM-32 DevKit) | Connects to |
| --- | --- | --- |
| Left PWM | 25 | L298N #1 **ENA and ENB** (both) |
| Left IN1 / IN2 | 26 / 27 | L298N #1 IN1+IN3 / IN2+IN4 |
| Right PWM | 13 | L298N #2 **ENA and ENB** (both) |
| Right IN1 / IN2 | 32 / 33 | L298N #2 IN1+IN3 / IN2+IN4 |
| HC-SR04 Trig left / centre / right | 16 / 17 / 4 | HC-SR04 Trig (no divider needed) |
| HC-SR04 Echo left / centre / right | 34 / 35 / 36 (input-only) | HC-SR04 Echo **through the 1k/2k divider** |
| I2C SDA / SCL | 21 / 22 | MPU-6050 (GY-521, powered from 3V3) |
| Serial to the Pi | USB (GPIO 1/3) | Pi USB port → `/dev/ttyUSB0` (CP2102) |

**Pins avoided:** 6–11 (flash), 0, 2, 5, 12 and 15 (strapping), and 1, 3 (USB serial). **Spare:** 39, 14, 18, 19, 23. `tests/test_hardware_consistency.py` checks these rules.

**HC-SR04 timing:** a 10 µs trigger, one sensor every 30 ms, and a 25 ms echo timeout. Readings outside 0.02–4.0 m are reported as `nan`, and `nan` means STOP.

**MPU-6050:** keep the robot **still for about 1 s after the ESP32 resets**, because that's when it measures the gyro bias. The Pi resets the ESP32 when it opens the port.

### Flash the ESP32 (PlatformIO)

```bash
cd firmware/esp32_robot
pio run -t upload          # builds for esp32doit-devkit-v1
pio device monitor         # T lines at 20 Hz; '#' lines are diagnostics
```

### Speed calibration (no encoders)

The robot only knows its speed through `SPEED_AT_FULL_DUTY_MPS` and `MIN_DUTY` in `include/robot_config.h`. The defaults (`MIN_DUTY 0.40`, `SPEED_AT_FULL_DUTY_MPS 0.50`) are worked out from the TT motor data sheet in that file's comments. **They are estimates and must be calibrated.** Calibrate on the floor, with the real load and a charged battery:

1. Find `MIN_DUTY`: lower it until the wheels just fail to turn at `--speed 0.05`, then raise it slightly until they turn reliably.
2. Run `tools/esp32_console.py --speed 0.15 --seconds 5` on the floor. It prints the **estimated travel** at the end.
3. Measure the real distance with a tape measure. Set `SPEED_AT_FULL_DUTY_MPS = current value × real ÷ estimated`, reflash, and repeat until they agree within about 5 %.
4. Repeat at `--speed 0.1` and `0.25`. If they disagree much more than at 0.15, raise `sensors.speed_estimate_margin`.

Recalibrate when the load, battery, wheels or floor change.

### Pi setup

```bash
sudo apt update && sudo apt install -y python3-picamera2 python3-venv
python3 -m venv --system-site-packages .venv      # exposes the system picamera2 package
.venv/bin/pip install -r requirements.txt ncnn
sudo usermod -aG dialout $USER                     # serial port access; then log out and back in

```

**Camera bring-up checklist.** Do these in order, and don't go further until each one passes.

1. Power off, connect the 22-pin to 15-pin FPC cable (see [WIRING.md](WIRING.md#camera)), then power on.
2. Run `rpicam-hello --list-cameras`. It must list **`imx708_wide`**. If it lists nothing, reseat the cable. Plain `imx708` means you have the normal lens, not the Wide one, and the 102° settings will be wrong.
3. Run `rpicam-hello -t 5000` and check the preview is sharp and the right way up.
4. Check that Picamera2 and OpenCV can read 640×360 BGR frames, which is exactly what `robot.py` uses:

   ```bash
   .venv/bin/python - <<'PY'
   from picamera2 import Picamera2
   import cv2
   cam = Picamera2()
   cam.configure(cam.create_video_configuration(main={"size": (640, 360), "format": "RGB888"}))  # Picamera2 "RGB888" = BGR bytes, same as camera.py
   cam.start()
   frame = cam.capture_array()
   cam.stop()
   print(frame.shape, frame.dtype)        # expect (360, 640, 3) uint8
   cv2.imwrite("check.jpg", frame)        # open it: colours must look right
   PY
   ```

5. Open `check.jpg`. If the colours look swapped (red objects look blue), the channel order is wrong. Fix that before going on: the detector expects BGR.

**If the camera isn't detected:** edit `/boot/firmware/config.txt`, set `camera_auto_detect=0`, add `dtoverlay=imx708` (use `dtoverlay=imx708,cam0` if it's on the CAM0 connector), then reboot.

**Lens type:** the robot uses the **wide** lens (`imx708_wide`, 102° horizontal). `--list-cameras` should report `imx708_wide`. `config/cameras/robot.json` sets `horizontal_fov_degrees: 102`.

**Resolution 640×360 (16:9):** the sensor is 16:9, so a 4:3 size like 640×480 would crop the sides and cut the view to about 86°. 640×360 keeps the full 102°. The 102° value is only a fallback; focal calibration measures the real view, which is one reason it's required. Wide lenses bend straight lines near the image edges, so distances are most accurate near the centre. Calibrate with the object near the middle of the image.

**Model:** build the NCNN model with `tools/export_for_pi.py` (320 px, the default). For the 640 px comparison run `tools/export_for_pi.py --imgsz 640`, which writes `models/outdoor_detector_small_640_ncnn_model/`. It needs `models/outdoor_detector_small.pt`, which isn't in git, so copy it over. You can export on the Pi, or export on the laptop and copy `models/outdoor_detector_small_ncnn_model/` to the Pi.

**Calibrate the mounted camera.** `robot.py` refuses to start until this is done.
1. Set `distance.camera_height_m` to the lens height above the floor, in metres.
2. Set `distance.camera_downward_tilt_degrees` to the downward tilt.
3. Run `.venv/bin/python tools/calibrate_camera.py --config config/cameras/robot.json --distance <m> --height <object m> --pixels <box px> --width 640` (measure `<box px>` in a 640×360 frame).
4. Check the estimated distances against a tape measure at 0.5, 1 and 2 m.

### Run

```bash
.venv/bin/python tools/esp32_console.py              # ESP32 telemetry only
.venv/bin/python robot.py                            # DRY RUN: full pipeline, motors held at 0
.venv/bin/python robot.py --drive                    # motors enabled
```

To watch from a laptop, set `output.api_host` to `"0.0.0.0"` in `config/robot_navigation.json` and open `http://<pi-ip>:8879`. Each control tick is logged to `results/robot-<time>.jsonl`.

### Bring-up order (do not skip steps)

1. **ESP32 alone, wheels off the ground.**
   - Run the straight-line bench test (wheels lifted, 0.15 m/s for 3 s, prints PASS/FAIL, logs to `results/bench/`):
     ```bash
     .venv/bin/python scripts/bench_straight.py
     ```
   - Run `tools/esp32_console.py --speed 0.1 --seconds 30`. Both sides should turn forward. If a side runs backwards, flip `*_MOTOR_INVERT`. The estimated travel counts up whenever the motors are commanded, even if a wheel is blocked: there are no encoders.
   - Rotate the robot by hand to the **left** (counter-clockwise from above): the heading must **increase**. If it decreases, set `IMU_Z_INVERT = true`, or the heading hold will steer the wrong way.
   - Block each HC-SR04 with your hand and check its range column changes. Point one at open space: it must show `nan`, not 0 or a big number.
   - **Unplug the USB cable while the wheels are turning: they must brake within 0.3 s.**
2. **Pi + ESP32 with motor power off** (e-stop pressed or motor battery disconnected).
   - Run `robot.py` and walk in front of the camera.
   - The terminal should print `STOP obstacle_in_stop_zone`, then `MOVE path_clear`.
   - Cover the lens: you should see `camera_blocked`.
   - Unplug the ESP32: the program should exit.
3. **Speed calibration** (see above), then check the robot drives straight over 3 m. If it still curves, raise `HEADING_HOLD_MPS_PER_DEG`; if it wobbles, lower it.
4. **Wheels on the ground at 0.15 m/s** (the default `cruise_speed_mps`).
   - Run `robot.py --drive` with a soft obstacle such as a cardboard box or a cushion.
   - Keep a hand on the power switch or e-stop.
5. **Raise the speed only after your measured stopping distance beats `stop_gap_m`.** Without encoders this tape-measure check is the only proof the robot really stops in time.
   - Measure the gap left between the bumper and the obstacle at the current speed.
   - Raise `cruise_speed_mps` in small steps, re-measuring each time.

### Why it stopped

| Reason | Meaning |
| --- | --- |
| `esp32_telemetry_stale` | No ESP32 report in the last `sensors.maximum_telemetry_age_s` |
| `speed_estimate_unavailable` / `gyro_unavailable` | The speed estimate or MPU-6050 data is missing (shown as nan) |
| `range_sensor_unavailable` | An HC-SR04 reading is stale, missing, or **out of range (nothing within about 4 m)**. Out-of-range counts as unknown, so the robot also stops when one sensor faces open space, such as down a long hallway. Test where walls or objects are within 4 m of all three sensors |
| `waiting_for_first_frame` / `waiting_for_fresh_vision` | No camera result yet, or the last one is older than `control.maximum_vision_age_s` |
| `odometry_unavailable_for_frame` | No telemetry at the moment the frame was captured |
| `camera_blocked` / `camera_visibility_uncertain` | Dark or covered lens |
| `central_object_range_unknown` | Something is in the middle of the image but its distance can't be estimated |
| `obstacle_in_stop_zone` / `waiting_for_clearance` / `confirming_clear_path` | Normal stop and resume |
| `session_time_limit` | `control.maximum_session_s` has elapsed |

While the gyro shows more than `sensors.rotation_gyro_threshold_dps` of rotation, the tracker can't estimate object speeds. Detected obstacles still stop the robot.

### Simulation and tests (no hardware needed)

`sim/` holds a fake ESP32 that speaks the same serial protocol and reads its constants from `firmware/esp32_robot/include/robot_config.h`. It models the 50 Hz loop, ramp, `MIN_DUTY`, heading hold, motor mismatch, gyro bias and noise, HC-SR04 rays and the 300 ms watchdog. A simulated camera renders the same world. Worlds are JSON files in `sim/worlds/`.

```bash
python -m pytest tests                       # everything (about 1-2 min)
python -m pytest tests/sim -s                # robot scenarios; path/speed PNGs in results/sim/
python robot.py --serial sim://wall_1_5m --config config/cameras/sim.json --drive   # full robot.py, simulated
python scripts/bench_straight.py --serial "sim://corridor?lifted=1"                 # bench script on the fake
```

The sim camera uses a **test-only colour-box detector** (`sim/color_detector.py`, model name `sim:color_boxes`), not YOLOE. It checks the geometry and control, not recognition.

### Vision checks on the real camera

1. **Accuracy.** Record images with the mounted camera, measure true distances with a tape, and compare at 320 vs 640 px. Errors over 10 % near the centre at 0.5–3 m are flagged. The template is `scripts/vision_ground_truth_template.csv`, and the steps are at the top of the script.
   ```bash
   .venv/bin/python scripts/eval_vision.py --video clip.h264 --every 10      # split a recording
   .venv/bin/python scripts/eval_vision.py --images results/vision/frames --truth my_truth.csv
   ```
2. **Speed on the Pi.** Reports fps, p95 latency, CPU temperature, and how often the control loop would have stopped for stale vision, at 320 and 640:
   ```bash
   .venv/bin/python scripts/bench_vision_pi.py --seconds 60
   ```
