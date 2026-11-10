# Start here: cameras and robot settings

Open **VisionDetection.code-workspace** in VS Code. The Explorer contains just this project. Use **Terminal → Run Task** for laptop camera, a selected camera settings file, moving simulation, stress simulation or project checks. The two detector debug options are also available under Run and Debug.

## Start the webcam from the Windows terminal

Open a terminal in the VisionDetection project folder and run:

```powershell
.\"Start Webcam.cmd"
```

Or run the same application directly, without activating an environment:

```powershell
& ".\.venv\Scripts\python.exe" ".\app.py"
```

Press **Q** in the camera window or **Ctrl+C** in the terminal to stop. In VS Code, **Terminal → Run Task → Start laptop camera** runs the same project Python. The task no longer depends on the selected Python interpreter or the Python extension command. A terminal at the parent folder must first enter `outputs\VisionDetection`; a terminal opened through this workspace starts in the correct project folder.

Avoid bare `python` on this laptop: it currently resolves to a Windows Store shortcut rather than this project's environment. In other examples below, replace `python` with `& ".\.venv\Scripts\python.exe"` on Windows. Linux/Pi uses `.venv/bin/python` after setting up its own environment. Do not run the Pi CSI example on Windows.

## Change movement behaviour

Edit **simulation/navigation_settings.json**. Hover a name in VS Code to see its explanation and units. Restart the simulation after editing.

| Setting | Current value | Meaning |
| --- | ---: | --- |
| `cruise_speed_mps` | 0.35 | Normal forward speed, metres per second |
| `slow_gap_m` | 2.8 | Begin slowing below this front clearance in metres |
| `stop_gap_m` | 1.2 | Base stopping clearance, metres |
| `resume_gap_m` | 1.8 | Clearance required before restarting |
| `clear_hold_s` | 1.0 | Clear-path wait before restarting, seconds |
| `minimum_slow_speed_fraction` | 0.2 | Minimum crawl speed as a fraction of cruise |
| `reaction_time_s` | 0.3 | Reaction allowance, seconds |
| `braking_deceleration_mps2` | 0.8 | Assumed braking ability used by the policy |

The actual stop threshold is `stop_gap_m + speed² / (2 × braking_deceleration_mps2) + speed × reaction_time_s`. It grows with measured speed. Use `stop_gap_m < resume_gap_m <= slow_gap_m`; the loader rejects invalid ordering. Motor acceleration/deceleration, delayed/dropout sensor timing and age limits are separately editable in **simulation/realism_settings.json**. Policy braking assumptions must be consistent with achievable motor braking. Tests are diagnostics, not permission to reduce clearances on physical hardware.

These movement values operate simulated motors. The webcam viewer does not command wheels. **motion.front_zone** describes an estimated arrival region; it is separate from stopping clearance.

## Choose a camera

The detector accepts ordinary BGR images, so recognition is independent of camera brand. Supported input paths are:

| Camera/input | Settings |
| --- | --- |
| Built-in / USB webcam | `camera.driver: "opencv"`, `source: null`, choose `index` |
| Linux video-device path | OpenCV driver, `source: "/dev/video0"` |
| Network camera / phone video stream | OpenCV driver, `source` set to the stream URL; codec/backend support is required |
| Raspberry Pi CSI camera | `camera.driver: "picamera2"`, `source: null`, choose `index`; Picamera2 must be installed on the Pi |

Start with **config/settings.json** for this laptop. Copy **config/cameras/usb.example.json** or **config/cameras/pi-csi.example.json** to your own settings filename for a different camera. Select it using the VS Code task or `python app.py --config config/cameras/my-camera.json`. Full settings files keep camera geometry and processing assumptions together. Do not store camera passwords in shared settings.

Requested `width`, `height` and `fps` are driver requests; actual images may differ. All detection/overlay geometry uses actual image dimensions. OpenCV `backend` is `auto` by default; explicit options are shown on hover. Windows automatic capture tries DirectShow and then the default backend. Unsupported cameras/codecs need their vendor SDK or another adapter; universal plug-and-play cannot be guaranteed.

The CSI adapter follows the [official Picamera2 manual](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf) and [source colour-format mapping](https://github.com/raspberrypi/picamera2/blob/main/picamera2/request.py). It requests `RGB888`, whose capture-array byte order is BGR for OpenCV. The Pi adapter is implemented and mock-tested, but has not been exercised on your Pi hardware.

## Recalibrate when optics or mounting change

A new camera can recognize objects without metric calibration; trustworthy distances require calibration. The example profiles deliberately use **distance.mode: "off"** until configured. The laptop's existing size estimates remain enabled and approximate.

1. Choose `size` for rough estimates using known/assumed object heights, or `ground` for objects resting on flat visible ground.
2. Set the camera's actual horizontal field of view, or calibrate focal length using a fully visible measured object at a known depth. The helper accepts `--config` so it updates the selected camera file, not always the laptop file.
3. Ground mode also requires measured `camera_height_m` and `camera_downward_tilt_degrees`.
4. Validate at several measured distances and image positions. Repeat after changing lens, zoom, crop, aspect ratio or mounting. Focal scaling by image width assumes the same optics/crop. Significant wide-angle distortion needs additional lens calibration/undistortion before accurate geometry is possible.

Example: `python tools/calibrate_camera.py --config config/cameras/my-camera.json --distance 2 --height 1.7 --pixels 320 --width 640`. Values are examples, not measurements of your camera. The helper writes focal calibration only; select the appropriate distance mode yourself.

Frame timestamps currently describe host receipt, not sensor exposure. Stream/driver buffering can hide extra latency. The app exits on no initial frames, failed reads or stale received frames according to editable camera timeouts. Future physical navigation must use measured latency, range sensing, odometry and a motor-controller watchdog. The laptop tracker assumes a stationary camera; the Webots tracker supports measured straight-line travel only.

## Edit one responsibility at a time

Use [EDITING_GUIDE.md](EDITING_GUIDE.md) for the module map. Every application Python module, class and function has a responsibility docstring; additional comments explain geometry, uncertainty, thread ownership and future `NAV-` integration points. Downloaded libraries, weights and generated logs are dependencies/results, not editable application code. Source credits are in [SOURCE_CREDITS.md](SOURCE_CREDITS.md).
