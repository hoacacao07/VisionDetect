"""Compare baseline/hybrid recognition on the SAME saved unannotated images.

Image-level person presence is scored, not bounding-box accuracy. The fixed
world contains a person in three phases; the covered-camera phase is negative.
This small, tuned diagnostic scene is not a held-out real-world benchmark.
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


def main():
    """Compare prepared detector candidates on saved images; never use simulator truth as input."""
    frames = sorted((PROJECT / "simulation/results/raw_frames").glob("*.jpg"))
    if not frames:
        raise SystemExit("Run Start Webots Simulation.cmd first to capture raw images.")
    settings = load_settings()
    report = dict(
        image_count=len(frames), evaluation="image-level person presence", models={}
    )
    for name, use_specialist in (("nano_baseline", False), ("selected_model", True)):
        detection_settings = copy.deepcopy(settings["detection"])
        detection_settings["image_size"] = 640
        if not use_specialist:
            detection_settings["human_model"] = ""
            detection_settings["model"] = "models/outdoor_detector_openvino_model"
        detector = ObjectDetector(detection_settings, settings["distance"])
        groups = {}
        for path in frames:
            frame = cv2.imread(str(path))
            if frame is None:
                raise RuntimeError(f"Cannot read {path}")
            phase = path.stem.split("_", 1)[1]
            start = time.perf_counter()
            observations = detector.detect(frame)
            elapsed = (time.perf_counter() - start) * 1000
            group = groups.setdefault(
                phase, dict(frames=0, person_detected_frames=0, inference_ms=[])
            )
            group["frames"] += 1
            group["person_detected_frames"] += any(
                item["label"] == "person" for item in observations
            )
            group["inference_ms"].append(elapsed)
        for group in groups.values():
            group["mean_processing_ms"] = round(
                sum(group.pop("inference_ms")) / group["frames"], 1
            )
        report["models"][name] = groups
    output = PROJECT / "simulation/results/detection_comparison.json"
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
