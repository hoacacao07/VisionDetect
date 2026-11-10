"""Export the two configured pretrained models for faster CPU inference.

Ultralytics exporter: https://docs.ultralytics.com/integrations/openvino/
OpenVINO runtime: https://github.com/openvinotoolkit/openvino (Apache-2.0).
This converts inference graphs; it does not retrain models or establish Pi FPS.
Run the benchmark before selecting the exported paths in settings.
"""

from pathlib import Path
import os

PROJECT = Path(__file__).resolve().parents[1]
os.environ.setdefault("YOLO_CONFIG_DIR", str(PROJECT / ".ultralytics"))
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
from ultralytics import YOLOE

if __name__ == "__main__":
    for name in (
        "outdoor_detector.pt",
        "outdoor_detector_small.pt",
        "human_specialist_small.pt",
    ):
        model = YOLOE(str(PROJECT / "models" / name))
        model.export(format="openvino", imgsz=640, quantize=16, dynamic=True)
