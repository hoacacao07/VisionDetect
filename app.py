"""VISION LAB — built on the pretrained Ultralytics YOLOE detector.

Original project: https://github.com/ultralytics/ultralytics
Model documentation: https://docs.ultralytics.com/models/yoloe/
Credits and licensing: docs/SOURCE_CREDITS.md and LICENSE.

The model/inference library are upstream; the webcam pipeline, distance
estimates and dashboard are additions for this project. No new model training.

START HERE. Settings: config/settings.json. Project map: README.md.
"""

from pathlib import Path
import sys

# Run local source directly; there is no package installation step.
sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from vision_detection.runtime import main

if __name__ == "__main__":
    main()
