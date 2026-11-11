"""PI VISION BENCHMARK: camera capture + detection, like robot.py, for 60 s per size.

On the Pi (camera connected, nothing else running):
  .venv/bin/python scripts/bench_vision_pi.py
Prints, for image_size 320 and 640:
  * average results per second (fps) and p95 latency per frame
    (inference time, and frame age = capture -> result ready)
  * CPU temperature (start / max) - the Pi 5 slows down when it gets hot
  * how often the 20 Hz control loop saw a result older than
    control.maximum_vision_age_s (0.8 s) - every one of those is a STOP
NCNN models are exported for ONE image size. Build the 640 one first:
  .venv/bin/python tools/export_for_pi.py --imgsz 640
Try it on a laptop with the simulated camera and colour detector:
  python scripts/bench_vision_pi.py --config config/cameras/sim.json --seconds 5
"""

import argparse
import copy
import statistics
import sys
import threading
import time
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PROJECT / "src"), str(PROJECT)]
from vision_detection.camera import LatestFrameCamera  # noqa: E402
from vision_detection.robot_config import load_robot_settings  # noqa: E402
from vision_detection.robot_runtime import make_detector  # noqa: E402
from vision_detection.settings import load_settings  # noqa: E402

THERMAL = Path("/sys/class/thermal/thermal_zone0/temp")
DEFAULT_MODEL_640 = "models/outdoor_detector_small_640_ncnn_model"


def cpu_temperature_c():
    """Pi CPU temperature in deg C, or None when not on a Pi."""
    try:
        return int(THERMAL.read_text()) / 1000
    except (OSError, ValueError):
        return None


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))] if ordered else float("nan")


def bench(settings, seconds, maximum_vision_age_s, control_rate_hz=20):
    """Run capture + detection; a 20 Hz thread samples result age like the control loop."""
    detector = make_detector(settings)
    camera = LatestFrameCamera(settings["camera"], settings["distance"])
    latest = {"capture": None}
    control = {"ticks": 0, "stale": 0}
    stop = threading.Event()

    def control_loop():
        while not stop.wait(1 / control_rate_hz):
            capture = latest["capture"]
            if capture is None:
                continue  # Count from the first result (model warm-up isn't driving).
            control["ticks"] += 1
            if time.monotonic() - capture > maximum_vision_age_s:
                control["stale"] += 1

    inference_ms, frame_age_ms, temperatures = [], [], []
    try:
        deadline = time.monotonic() + settings["camera"].get("startup_timeout_s", 10)
        while camera.get_latest_frame()[0] is None:
            if time.monotonic() > deadline or camera.stop_capture_event.is_set():
                raise SystemExit("No camera frames. Run the camera bring-up checklist in README.md.")
            time.sleep(0.02)
        thread = threading.Thread(target=control_loop, daemon=True)
        thread.start()
        started, last_sequence, next_temperature = time.monotonic(), -1, 0.0
        while time.monotonic() - started < seconds:
            frame, sequence, capture_time = camera.get_latest_frame()
            if sequence == last_sequence:
                time.sleep(0.002)
                continue
            last_sequence = sequence
            t0 = time.perf_counter()
            detector.detect(frame)
            inference_ms.append((time.perf_counter() - t0) * 1000)
            frame_age_ms.append((time.monotonic() - capture_time) * 1000)
            latest["capture"] = capture_time
            if time.monotonic() >= next_temperature:
                temperature = cpu_temperature_c()
                if temperature is not None:
                    temperatures.append(temperature)
                next_temperature = time.monotonic() + 1
        elapsed = time.monotonic() - started
    finally:
        stop.set()
        camera.close()
    return dict(
        fps=len(inference_ms) / elapsed,
        inference_mean_ms=statistics.mean(inference_ms),
        inference_p95_ms=percentile(inference_ms, 0.95),
        frame_age_p95_ms=percentile(frame_age_ms, 0.95),
        stale_fraction=control["stale"] / max(control["ticks"], 1),
        control_ticks=control["ticks"],
        temperature_start_c=temperatures[0] if temperatures else None,
        temperature_max_c=max(temperatures) if temperatures else None,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(PROJECT / "config/cameras/robot.json"))
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--sizes", type=int, nargs="+", default=[320, 640])
    parser.add_argument("--model320", help="Model for 320 (default: detection.model in --config)")
    parser.add_argument("--model640", help="Model for 640 (default: models/outdoor_detector_small_640_ncnn_model; "
                        "an NCNN model must be exported at 640)")
    arguments = parser.parse_args()
    base = load_settings(arguments.config)
    maximum_age = load_robot_settings()["control"]["maximum_vision_age_s"]
    for size in arguments.sizes:
        settings = copy.deepcopy(base)
        settings["detection"]["image_size"] = size
        model = arguments.model320 if size == 320 else arguments.model640 if size == 640 else None
        if size == 640 and model is None and not settings["detection"]["model"].startswith("sim:"):
            model = DEFAULT_MODEL_640
        if model:
            settings["detection"]["model"] = model
        model_name = settings["detection"]["model"]
        if not model_name.startswith("sim:") and not (PROJECT / model_name).exists():
            print(f"\nimage_size {size}: SKIPPED, model not found: {model_name}")
            continue
        print(f"\nimage_size {size}: {model_name}, {arguments.seconds:g} s ...", flush=True)
        r = bench(settings, arguments.seconds, maximum_age)
        temperature = (
            f"{r['temperature_start_c']:.1f} -> max {r['temperature_max_c']:.1f} C"
            if r["temperature_max_c"] is not None else "n/a (not a Pi)"
        )
        print(f"  fps                    {r['fps']:.1f}")
        print(f"  inference              mean {r['inference_mean_ms']:.0f} ms, p95 {r['inference_p95_ms']:.0f} ms")
        print(f"  frame age at result    p95 {r['frame_age_p95_ms']:.0f} ms")
        print(f"  control ticks stale    {r['stale_fraction'] * 100:.1f} % of {r['control_ticks']} "
              f"(result older than {maximum_age} s -> STOP)")
        print(f"  CPU temperature        {temperature}")


if __name__ == "__main__":
    main()
