"""DETECTION: turn pixels into labeled objects, with distance estimates.

PRETRAINED MODEL / INFERENCE LIBRARY: Ultralytics YOLOE, AGPL-3.0.
Original source: https://github.com/ultralytics/ultralytics
Model reference: https://docs.ultralytics.com/models/yoloe/
This module integrates the upstream detector; it does not train a new model.

No camera access, drawing, logging or wheel control belongs in this module.
"""

import os
from .settings import PROJECT_DIRECTORY

os.environ.setdefault("YOLO_AUTOINSTALL", "false")
os.environ.setdefault("YOLO_CONFIG_DIR", str(PROJECT_DIRECTORY / ".ultralytics"))
import numpy as np
import torch
from ultralytics import YOLOE
from .distance import estimate_distance_m
from .object_library import load_object_library
from .motion import unmeasured_motion
from .perception_fusion import merge_people
from .prompt_library import load_detection_prompts


class ObjectDetector:
    """Load the prepared model once; reuse it for every camera frame."""

    def __init__(self, detection_settings, distance_settings):
        """Load weights/catalog once; reject class-library mismatches at startup."""
        self.settings = detection_settings
        self.distance_settings = distance_settings
        torch.set_num_threads(
            min(detection_settings["cpu_threads"], os.cpu_count() or 1)
        )
        model_path = PROJECT_DIRECTORY / detection_settings["model"]
        if not model_path.exists():
            raise FileNotFoundError("Model missing. Run tools/prepare_model.py first.")
        self.model = YOLOE(str(model_path))
        self.object_library = load_object_library()
        self.prompt_to_label = load_detection_prompts(self.object_library)
        self.class_names = list(self.object_library)
        if set(self.model.names.values()) != set(self.prompt_to_label):
            raise ValueError(
                "Object labels changed. Run tools/prepare_model.py, then restart."
            )
        self.description = f"{model_path.name}: {len(self.class_names)} categories, {len(self.prompt_to_label)} visual prompts"
        self.human_model = None
        if detection_settings.get("human_model"):
            human_path = PROJECT_DIRECTORY / detection_settings["human_model"]
            if not human_path.exists():
                raise FileNotFoundError(
                    "Human specialist missing. Run tools/prepare_human_model.py."
                )
            self.human_model = YOLOE(str(human_path))
            if not set(self.human_model.names.values()) <= {
                "person",
                "man",
                "woman",
                "human",
                "pedestrian",
            }:
                raise ValueError(
                    "Human specialist contains a non-human prompt; rebuild it with tools/prepare_human_model.py."
                )
            self.description += f"; people specialist: {human_path.name}"
        # Warm-up prevents model initialization delaying the first live frame.
        self.detect(np.zeros((480, 640, 3), dtype=np.uint8))

    def detect(self, frame):
        """Return JSON-compatible objects. Unknown distance stays None/null."""
        result = self.model.predict(
            frame,
            imgsz=self.settings["image_size"],
            conf=self.settings["minimum_confidence"],
            device="cpu",
            verbose=False,
        )[0]
        results = [(result, False)]
        if self.human_model is not None:
            specialist = self.human_model.predict(
                frame,
                imgsz=self.settings["image_size"],
                conf=self.settings.get("human_minimum_confidence", 0.30),
                device="cpu",
                verbose=False,
            )[0]
            results.append((specialist, True))
        height, width = frame.shape[:2]
        detections = []
        for result, human_only in results:
            for box in result.boxes:
                label = (
                    "person"
                    if human_only
                    else self.prompt_to_label[result.names[int(box.cls.item())]]
                )
                definition = self.object_library[label]
                # NAV-PERCEPTION: a class label/box is an observation, not a decision
                # that the object blocks the robot's path. Keep classification,
                # metric geometry and control policy separate when extending this.
                # Pixel order: left, top, right, bottom. Images are not mirrored.
                bounds = [float(value) for value in box.xyxy[0].tolist()]
                distance_m, explanation = estimate_distance_m(
                    bounds, label, width, height, self.distance_settings
                )
                centre_x = (bounds[0] + bounds[2]) / 2
                position = (
                    "left"
                    if centre_x < width / 3
                    else "right" if centre_x > 2 * width / 3 else "center"
                )
                detections.append(
                    {
                        "label": label,
                        "recognition_source": (
                            "human_specialist" if human_only else "outdoor_detector"
                        ),
                        "object_category": definition.category,
                        "motion_group": definition.motion_group,
                        # Do not equate a class-based default with measured movement.
                        "motion_status": "unmeasured",
                        "motion": unmeasured_motion(),
                        "confidence": round(float(box.conf.item()), 3),
                        "bbox_xyxy": [round(value, 1) for value in bounds],
                        "horizontal_position": position,
                        "horizontal_offset_normalized": round(
                            2 * centre_x / width - 1, 3
                        ),
                        "distance_m": distance_m,
                        "distance_status": (
                            "estimated" if distance_m is not None else "unavailable"
                        ),
                        "distance_method": self.distance_settings["mode"],
                        "distance_note": explanation,
                    }
                )
        return merge_people(detections)
