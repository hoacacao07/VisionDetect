# Webcam distance & calibration

[← Project home](../README.md) · [Settings reference](SETTINGS.md)

## What a webcam can tell you

A single uncalibrated image does not uniquely determine real-world scale. These formulas produce **estimates**, not depth measurements. Label confidence does not quantify range accuracy. Validate against several tape-measured distances before interpreting the numbers.

## Option A · Size mode — immediate rough estimates

In `config/settings.json`, `distance.mode` is `size` by default.

The pinhole approximation uses **focal length × assumed full height ÷ box height**. It then approximates range along the centre ray. Defaults assume a 1.7 m person, a 1.5 m car and unverified 65° horizontal field of view.

Use an upright, fully visible object near the view direction and a nearly level camera. Seated people, partial bodies, unusual cars and strongly tilted cameras violate these assumptions. Cropped objects return unknown. Trees have no default height because their sizes vary too widely.

## Calibrate the focal length

1. Place a fully visible upright object of **measured height** near the image centre, with the camera level.
2. Measure its **optical-axis distance** from the camera.
3. Read its full bounding-box pixel height and image width from the JSON.
4. Run the helper using your measurements. These are examples, not calibration values for your camera:

```powershell
.\.venv\Scripts\python.exe tools\calibrate_camera.py --distance 2 --height 1.7 --pixels 320 --width 640
```

The helper stores focal length in pixels under `distance.focal_calibration`. Restart the detector. Recalibrate after changing the camera, zoom or crop. Resolution scaling assumes the same field of view.

## Option B · Ground mode — including visible tree bases

Set these values in the `distance` settings group:

- `mode`: `ground`
- `camera_height_m`: measured lens height above level ground
- `camera_downward_tilt_degrees`: measured angle below horizontal (0 = level)

Calibrate focal length first. The formula intersects a ray through the box's bottom centre with the ground plane. It returns **horizontal ground distance**, not the centre-ray range used by size mode.

It only makes sense if that pixel is the real ground contact: feet, tree base, or car ground contact. Slopes, occlusion, an inaccurate box, or a canopy box with no visible trunk can invalidate it. The program cannot verify these assumptions automatically. Missing camera measurements and bases near/above the horizon produce unknown distance.

## Unknown is useful information

`distance_m: null` is explicit: the current method cannot provide a suitable estimate. It never means zero metres, no obstacle, or a clear path. Use physical ranging for reliable collision avoidance in a later robot.
