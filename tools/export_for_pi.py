"""Optional: bake configured class embeddings into a Pi-oriented NCNN model."""

# NAV-DEPLOY: export alone does not establish control-loop timing on the Pi.
# Benchmark full camera -> inference -> tracking -> output latency and verify
# failure handling before connecting a navigation controller to real motors.

from pathlib import Path
import os
from ultralytics import YOLOE

if __name__ == "__main__":
    project_directory = Path(__file__).resolve().parents[1]
    os.chdir(project_directory)
    prepared_model_path = project_directory / "models" / "outdoor_detector_small.pt"
    model = YOLOE(str(prepared_model_path))
    # Smaller input reduces CPU cost on the Pi at the expense of small-object
    # accuracy. Exported classes are fixed; re-prepare/re-export to change them.
    pi_inference_size = 320
    model.export(format="ncnn", imgsz=pi_inference_size)
