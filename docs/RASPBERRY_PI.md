# Moving to Raspberry Pi 5 · 4 GB

[← Project home](../README.md) · [Code guide](EDITING_GUIDE.md)

## What transfers

Copy `app.py`, `src/`, `config/`, `web/`, `requirements.txt`, and `models/outdoor_detector_small.pt`. Keep the license and guides. **Do not copy the Windows .venv.** Create a Linux virtual environment on 64-bit Raspberry Pi OS and install dependencies for that platform. Before running, set `detection.model` to `models/outdoor_detector_small.pt`; the default OpenVINO directory is optimized/tested on this laptop, not on the Pi.

The selected configuration uses one model; `detection.human_model` is empty. If enabling the optional specialist, also copy its checkpoint and benchmark the extra time/memory. The current Pi export helper does not optimize the specialist automatically.

The prepared checkpoint already contains the configured outdoor-object prompts. Normal inference does not require the large text encoder downloaded during model preparation.

## Camera adapter

A USB webcam uses the OpenCV adapter. The CSI Picamera2 adapter is now implemented in `camera.py`. Copy `config/cameras/pi-csi.example.json` to your own camera settings and select it with `--config`. It selects the portable PyTorch checkpoint and leaves distance estimates disabled pending calibration. Install Raspberry Pi OS’s `python3-picamera2` package; create the Pi virtual environment with `--system-site-packages` to expose that system camera library. The CSI path has mock tests but still needs verification on the actual Pi. See [camera setup](START_HERE.md).

The intended camera is the **Camera Module 3 Wide**: [published specifications](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf) give 120° diagonal, 102° horizontal and 67° vertical view. The Webots camera uses a 102° pinhole approximation at 960×540, which yields about 69.6° vertically. Real lens distortion, the selected sensor crop, autofocus and actual mounting must be calibrated before trusting metric distances.

## Performance

Try detection image size 320 first. Measure memory use, end-to-end latency and detection quality on the actual Pi; laptop timing does not establish Pi performance. The small model is a candidate, not a verified deployment on your device.

Ultralytics recommends NCNN for Pi inference. To prepare that optional model on the laptop:

```powershell
.\.venv\Scripts\python.exe tools\export_for_pi.py
```

Copy the exported model directory, update `detection.model`, and test on the Pi. Export can download additional tools. No Pi export or hardware benchmark is claimed complete yet.

## Keep wheel decisions separate

The detector supplies observations with timestamps, labels, positions and estimated/unknown distance. A wheel controller should decide movement separately, reject stale data, and account for missed detections. It should use a real range sensor for reliable collision avoidance.

The separate Webots world now has a straight-line slow/stop/resume controller and wheel-encoder compensation. It drives simulated motors only. Physical sensor integration, motor-driver control and transmission to a Pi are still future work. The local `/api/detections` schema is the integration point. Replace simulated range devices with calibrated physical readings and feed measured wheel travel/speed into `StraightLineTracker`; do not substitute commanded speed for measurements.

References: [YOLOE](https://docs.ultralytics.com/models/yoloe/) · [Pi deployment](https://docs.ultralytics.com/guides/raspberry-pi/)
