"""Optional: bake configured class embeddings into a Pi-oriented NCNN model."""

# NAV-DEPLOY: export alone does not establish control-loop timing on the Pi.
# Benchmark full camera -> inference -> tracking -> output latency and verify
# failure handling before connecting a navigation controller to real motors.

from pathlib import Path
import argparse
import os
import shutil
from ultralytics import YOLOE

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--imgsz", type=int, default=320,
        help="Inference size baked into the NCNN model (320 for robot.py; 640 for "
        "scripts/bench_vision_pi.py comparisons -> outdoor_detector_small_640_ncnn_model)",
    )
    arguments = parser.parse_args()
    project_directory = Path(__file__).resolve().parents[1]
    os.chdir(project_directory)
    prepared_model_path = project_directory / "models" / "outdoor_detector_small.pt"
    source = prepared_model_path
    if arguments.imgsz != 320:
        # Export a renamed copy so the output folder is
        # outdoor_detector_small_<size>_ncnn_model and the 320 model robot.json
        # uses is never overwritten.
        source = prepared_model_path.with_name(f"outdoor_detector_small_{arguments.imgsz}.pt")
        shutil.copyfile(prepared_model_path, source)
    # Smaller input reduces CPU cost on the Pi at the expense of small-object
    # accuracy. Exported classes are fixed; re-prepare/re-export to change them.
    try:
        exported = YOLOE(str(source)).export(format="ncnn", imgsz=arguments.imgsz)
    finally:
        if source != prepared_model_path:
            source.unlink(missing_ok=True)
    print(f"Saved {exported}")
