"""Compare the same selected weights under PyTorch and OpenVINO.

Uses saved raw fixed-camera images. Reports inference-only timing and image-
level person observations, not camera/render FPS or universal detection recall.
Pause Webots and close other detector instances for a useful timing comparison.
"""

from pathlib import Path
import copy
import json
import sys
import time
import cv2

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from vision_detection.settings import load_settings
from vision_detection.detector import ObjectDetector

if __name__ == "__main__":
    settings = load_settings()
    frames = sorted((PROJECT / "simulation/results/raw_frames").glob("*.jpg"))
    if not frames:
        raise SystemExit("Run the fixed-camera simulation first to save raw frames.")
    results = {}
    for name, model in (
        ("pytorch_small", "models/outdoor_detector_small.pt"),
        ("openvino_small", "models/outdoor_detector_small_openvino_model"),
    ):
        config = copy.deepcopy(settings["detection"])
        config.update(model=model, human_model="", image_size=640)
        detector = ObjectDetector(config, settings["distance"])
        times = []
        phases = {}
        for path in frames:
            frame = cv2.imread(str(path))
            start = time.perf_counter()
            observations = detector.detect(frame)
            times.append((time.perf_counter() - start) * 1000)
            phase = path.stem.split("_", 1)[1]
            group = phases.setdefault(phase, dict(images=0, person_observed_images=0))
            group["images"] += 1
            group["person_observed_images"] += any(
                x["label"] == "person" for x in observations
            )
        average = sum(times) / len(times)
        results[name] = dict(
            mean_processing_ms=average,
            inference_only_fps=1000 / average,
            image_count=len(frames),
            person_presence=phases,
        )
        print(name, results[name], flush=True)
    (PROJECT / "simulation/results/selected_model_performance.json").write_text(
        json.dumps(results, indent=2)
    )
