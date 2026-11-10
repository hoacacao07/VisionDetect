# Moving-robot validation — 2026-10-24

Both 65-second continuous_v3_65s runs passed all ten physical-response checks. The robot drove straight, slowed, stopped for the person and box, resumed when clear, stopped with a covered camera, and finished stationary. Evaluation ground truth is separate from control inputs.

| Measurement | Normal laptop | Delay/dropout stress |
| --- | ---: | ---: |
| Minimum approximate front clearance (m) | 1.233 | 1.237 |
| Camera-cover physical stopping delay (s) | 0.744 | 0.680 |
| Fresh inference updates per real second, whole run | 7.98 | 3.88 |
| Completed inference updates | 837 | 268 |
| Non-ground contact frames | 0 | 0 |

## Fresh detection consistency

Predicted boxes are excluded. These are selected scripted visibility windows, not general real-world recall.

| Window / category | Normal | Stress |
| --- | ---: | ---: |
| person waiting / person | 147/147 | 49/49 |
| person waiting / tree | 133/147 | 46/49 |
| person waiting / car | 147/147 | 49/49 |
| stationary box / box | 131/131 | 44/44 |
| stationary box / tree | 131/131 | 44/44 |
| stationary box / car | 131/131 | 44/44 |

The tree still has missed fresh detections. Brief continuity boxes are explicitly predictions and have no fresh distance/speed measurement. The system cannot guarantee every visible object is recognized.

## What this establishes

The selected pretrained YOLOE small detector uses 48 canonical categories and 57 visual prompts, with OpenVINO laptop execution. No model was trained from scratch. The test includes acquisition/inference delay, occasional sensor loss, camera noise/blur, lighting changes, wheel physics and acceleration limits. The viewer no longer limits image refresh to 2.5 FPS. Full simulation update rates above include rendering and control; inference-only benchmarks are separate.

This is a tuned diagnostic with stylized assets, approximate clearance footprints and a flat floor. The robot dimensions, camera mounting and braking are editable assumptions. The camera approximates Camera Module 3 Wide with 102-degree horizontal FOV, without measured lens calibration. Physical Raspberry Pi performance, motor braking, lens distortion and real outdoor recognition remain unverified. Conservative unknown-range stops can stop the robot early.

## Reproduce and edit

Run `Start Moving Robot Simulation.cmd` or `Start Stress Simulation.cmd`; each pauses after 65 simulated seconds. Run `tools/summarize_simulation.py` after both to regenerate `validation_results.json`. Timing windows require scenario version v3. See [simulation configuration](MOVING_ROBOT.md) and [upstream credits](../docs/SOURCE_CREDITS.md). All 47 unit tests and the browser JavaScript syntax check passed.
