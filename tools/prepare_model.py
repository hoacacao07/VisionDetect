"""Configure an upstream pretrained detector with our outdoor-object prompts.

Foundation: Ultralytics YOLOE — https://docs.ultralytics.com/models/yoloe/
Credits: docs/SOURCE_CREDITS.md. This is prompt configuration, not model training.
"""

import os
from pathlib import Path
import sys
import argparse

PROJECT_DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIRECTORY / "src"))
from vision_detection.object_library import load_object_library
from vision_detection.prompt_library import load_detection_prompts

os.environ.setdefault("YOLO_CONFIG_DIR", str(PROJECT_DIRECTORY / ".ultralytics"))
from ultralytics import YOLOE

# EDIT config/outdoor_objects.json to change categories. Names are baked into
# the saved checkpoint, so rerun this tool after editing the object library.
PRETRAINED_CHECKPOINT_NAME = "yoloe-26n-seg.pt"
PREPARED_CHECKPOINT_NAME = "outdoor_detector.pt"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--size",
        choices=["nano", "small"],
        default="nano",
        help="Pretrained backbone size; no new training.",
    )
    args = parser.parse_args()
    if args.size == "small":
        PRETRAINED_CHECKPOINT_NAME = "yoloe-26s-seg.pt"
        PREPARED_CHECKPOINT_NAME = "outdoor_detector_small.pt"
    # The same loader is used by inference, keeping label/mobility definitions
    # consistent as the library grows. The dict preserves the configured order.
    object_classes = list(load_detection_prompts(load_object_library()))
    # Upstream downloads the detector/text encoder here on first preparation.
    # This configures a pretrained model; it does not train it on new images.
    (PROJECT_DIRECTORY / "models").mkdir(exist_ok=True)
    os.chdir(PROJECT_DIRECTORY / "models")
    model = YOLOE(PRETRAINED_CHECKPOINT_NAME)
    model.set_classes(object_classes)
    # Save completely before replacing the working model; keep a failed
    # preparation from leaving a partially written checkpoint behind.
    temporary_checkpoint = Path(PREPARED_CHECKPOINT_NAME).with_suffix(".pending.pt")
    model.save(temporary_checkpoint)
    temporary_checkpoint.replace(PREPARED_CHECKPOINT_NAME)
    print("Prepared pretrained model:", model.names)
