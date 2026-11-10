"""Configure a people-focused YOLOE prompt set; this does not train new weights.

Ultralytics YOLOE: https://docs.ultralytics.com/models/yoloe/ (AGPL-3.0).
Uses the larger small checkpoint, compared with the nano outdoor model. Related prompts help
with appearance variation; all outputs are mapped to the existing person label.
"""

from pathlib import Path
import os

PROJECT = Path(__file__).resolve().parents[1]
os.environ.setdefault("YOLO_CONFIG_DIR", str(PROJECT / ".ultralytics"))
from ultralytics import YOLOE

if __name__ == "__main__":
    os.chdir(PROJECT / "models")
    model = YOLOE("yoloe-26s-seg.pt")
    model.set_classes(["person", "man", "woman", "human", "pedestrian"])
    model.save("human_specialist_small.pending.pt")
    Path("human_specialist_small.pending.pt").replace("human_specialist_small.pt")
    print("Prepared human_specialist_small.pt from pretrained YOLOE; no new training.")
