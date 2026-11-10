# Moving robot: straight ahead, slow, stop, resume

Double-click **Start Moving Robot Simulation.cmd** in the project folder. **Start Stress Simulation.cmd** adds 150 ms of processing delay and more lost sensor samples, with seed 73 instead of 42. Run one world at a time. Each runs for 65 simulated seconds, finishes braking and pauses. Q pauses early. Runtime may be slower than real time because the CPU runs recognition while Webots renders the scene.

The camera image shows labeled boxes, distance, object speed and example front-zone arrival. The side panel shows measured robot speed, forward command, nearest path clearance, the limiting sensor and the movement decision. Objects may retain unknown distance/speed when the evidence is insufficient. The dashboard is at http://127.0.0.1:8880 while running.

The selected detector is **one YOLOE small model**, with 48 canonical categories and 57 visual prompts, exported through OpenVINO. This replaces the earlier nano-outdoor + small-people combination. Broader descriptions improved recognition of the plain colored box; canonical labels stay stable. The model was not retrained. Brief prediction-only boxes are explicitly labelled, have no fresh metric measurements, and are excluded from recognition scoring. Recognition can still miss or misclassify visible objects.

## Camera and hardware assumptions

The camera target is **Raspberry Pi Camera Module 3 Wide**. Its [published specifications](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf) are 120° diagonal, 102° horizontal and 67° vertical. The simulation uses a 102° horizontal pinhole camera at 960×540 (about 69.6° vertically), sampled every 64 ms. This approximates the intended camera; actual lens distortion, autofocus, rolling shutter and sensor cropping are not calibrated or fully simulated.

The physical robot is not specified yet. Assumed body: 0.70×0.50 m, four driven 0.15 m radius wheels, 8 kg body plus wheel masses, camera lens 1 m above the ground, pitch 0.1 rad downward, cruise 0.35 m/s. These must be replaced with actual dimensions, mount geometry and measured braking before claiming hardware agreement.

## What drives movement

```text
RGB image -> optimized YOLOE model ---> boxes and calibrated ground estimates
Front range sensors -----------------> measured clearance
Wheel encoders ----------------------> measured travel and robot speed
Camera visibility -------------------> stop if view is suspect/blocked
                                      |
                         slow / stop / resume policy
                                      |
                           equal speed to all wheels
```

There is no steering: all four wheel motors receive the same command. The robot starts after one second of clear readings, cruises at 0.35 m/s, slows below 2.8 m clearance and stops around the configured 1.2 m gap plus a speed-dependent braking/reaction allowance. It requires more than 1.8 m clearance for one second to restart. A central object with unknown ground range also stops movement. The camera's visibility-checking state stops the robot before the one-second confirmed blocked-camera alert appears.

Three **Webots sonar sensors** have seven rays each, a 0.30 rad cone, six-metre maximum, 1% lookup-table noise and 1 cm resolution. Two are at 0.70 m height; the centre is at 1.25 m. Readings reach the controller every 128 ms, with sample loss and an age limit. Webots models incidence-angle reflection losses; this is still a simplified sonar model, not a characterized hardware sensor. Coverage between/below the cones is incomplete.

Camera Gaussian noise and motion blur, textured asphalt, HDR surroundings, shadows and varying directional lighting make the images less ideal. The walking person's translation, heading and joint poses change continuously; a parked car remains fixed. Wheel-ground contacts use an assumed friction coefficient of 0.8. The floor is flat and the pedestrian/crate are kinematic actors, not biomechanical or force-driven models.

`sensor_timing.py` withholds results until measured inference duration has elapsed in simulation time. Acquisition proceeds concurrently and only the newest available image is processed; frames do not build a stale backlog. An assumed 50 ms acquisition delay applies. The stress profile adds 150 ms compute delay; that is a stress assumption, **not measured Pi performance**. A stale camera or range feed stops movement. Commands ramp at 0.3 m/s² acceleration and 0.6 m/s² deceleration rather than stopping instantaneously.

## Test sequence

| Time | Scene event |
| --- | --- |
| 0–10 s | Person walks continuously into the path at 0.30 m/s |
| 10–22 s | Person waits in the path |
| 22–32 s | Person walks sideways away at 0.40 m/s |
| 32–50 s | Robot approaches the stationary box farther ahead |
| 50–54 s | Box moves continuously sideways out of the path |
| 55–60 s | Camera covered; robot should stop |
| 60–65 s | View restored; robot should resume, then finish braking before test completion |

Tracks survive phase boundaries because the actor paths are continuous. Camera covering is a deliberate visibility-failure event. The scene is a repeatable diagnostic scenario, not a full traffic/pedestrian simulator.

## Moving-camera speed

`StraightLineTracker` adds measured wheel travel to camera-relative observations before estimating object speed. It subtracts current measured robot speed to recover relative approach speed and front-zone arrival. A stationary obstacle therefore should not be labeled as moving just because the camera moves. This assumes fixed heading, fixed camera height/pitch, level ground and little wheel slip. It does not handle turns or camera rotation. Physical time-to-contact remains unknown; the displayed arrival predicts a point entering a front rectangle.

## Results and honest limits

`moving_results/latest_run.jsonl` separates `prediction` from `evaluation_only`; stress outputs are under `moving_results/stress/`. `latest_summary.json` counts control steps, completed inference updates, stop/move commands, fresh person observations, reference clearance violations and non-ground contact frames. Phase previews and sparse raw images are saved alongside it. Repeated control steps are not extra camera detections. A zero target command is not necessarily physically stationary during braking; measured speed is logged separately.

The clearance check uses approximate footprints and a 0.60 m minimum test requirement. Physics contacts above 5 cm are also counted as a separate diagnostic. Neither is a full safety certification. Coverage ratios by phase include times when the person leaves the view, so they are not universal visibility-normalized recall. Reports must retain misses; predicted boxes never count as new recognition.

Simulator truth is used only to script events and evaluate results. Camera calibration uses known mount geometry. Wheel travel comes from encoder devices, not Supervisor position. Neither truth labels nor positions are supplied to perception, tracking or the movement policy.

This scene is diagnostic and has been used while tuning the model choice. It is not a held-out test, a broad false-positive assessment or proof of safe navigation. The stock human/trees are stylized and the background is a distant image. Real videos, actual-camera calibration and robot trials remain necessary. General uneven terrain, wheel slip/bias, rain, transparent hazards, realistic exposure control and rolling-shutter distortion are not established by this test.

## Edit the right part

| Change | File |
| --- | --- |
| Cruise, gaps, braking allowance, restart delay | `simulation/navigation_settings.json` |
| Noise/dropout timing, stress allowance, motion ramps | `simulation/realism_settings.json` and camera/sensor fields in the world |
| Model categories / alternative visual descriptions | `config/outdoor_objects.json`, `config/detection_prompts.json` |
| Robot geometry, wheels, camera and range-device mounts | `simulation/worlds/straight_robot.wbt` |
| Obstacle paths, scene timing, Webots device wiring and evaluation | `simulation/controllers/straight_robot/straight_robot.py` |
| Stop/go decisions independent of Webots | `src/vision_detection/longitudinal_control.py` |
| Straight-line encoder compensation | `src/vision_detection/ego_motion.py` |
| People duplicate/conflicting-label handling | `src/vision_detection/perception_fusion.py` |
| Movement panel appearance | `src/vision_detection/navigation_display.py` |

`stop_gap_m` is clearance from the robot's front, not its camera. Keep `front_offset_m`, `wheel_radius_m`, camera height/pitch/focal calibration and world geometry consistent when editing mounts or dimensions. Actual braking and sensor freshness must be measured before adapting the policy to physical motors. For a Pi, replace device adapters while keeping perception/control/evaluation separate.

## Sources

The robot body, wheel arrangement, policy and harness were added for this project using Webots APIs. The scene uses Cyberbotics' Pedestrian, SimpleTree, ToyotaPriusSimple, Asphalt and TexturedBackground, retaining their separate [upstream credits/licenses](README.md#credits-and-licenses). Gait field-control follows the approach in Cyberbotics' Apache-2.0 pedestrian controller, with project-defined sinusoidal angles rather than copied tables. Detection uses [Ultralytics YOLOE and OpenVINO](../docs/SOURCE_CREDITS.md); no model was trained from scratch.
