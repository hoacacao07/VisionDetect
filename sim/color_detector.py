"""TEST/SIM-ONLY DETECTOR: find brightly coloured boxes with plain OpenCV.

NOT the real detector. The real robot uses vision_detection.detector
(YOLOE), which is trained on real objects and does not reliably recognise flat
synthetic boxes. This stand-in lets the simulation and tests exercise the
geometry, tracking and control code with known ground truth.

It returns the same dict layout as ObjectDetector.detect(), so everything
downstream (distance, tracker, continuity, controller, display) is the real code.
Selected with detection.model = "sim:color_boxes" (config/cameras/sim.json).
"""

import cv2
import numpy as np
from vision_detection.distance import estimate_distance_m
from vision_detection.motion import unmeasured_motion

SIM_COLOR_DETECTOR = "sim:color_boxes"
MINIMUM_SATURATION = 100  # Walls and floor are grey/beige; boxes are saturated.
MINIMUM_AREA_PX = 30


class ColorBoxDetector:
    """Colour threshold + contours -> bounding boxes labelled "box"."""

    description = "TEST-ONLY colour-box detector (not YOLOE)"
    class_names = ["box"]

    def __init__(self, distance_settings):
        """Keep the distance settings so each box gets the same distance estimate as the real detector."""
        self.distance_settings = distance_settings

    def detect(self, frame):
        """Return detections in the same format as ObjectDetector.detect()."""
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = ((hsv[:, :, 1] >= MINIMUM_SATURATION) & (hsv[:, :, 2] >= 50)).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h < MINIMUM_AREA_PX:
                continue
            # Pixel edges in continuous coordinates (bottom = y + h).
            bounds = [float(x), float(y), float(x + w), float(y + h)]
            distance_m, note = estimate_distance_m(bounds, "box", width, height, self.distance_settings)
            centre_x = (bounds[0] + bounds[2]) / 2
            detections.append(
                {
                    "label": "box",
                    "recognition_source": "sim_color_detector",
                    "object_category": "obstacle",
                    "motion_group": "usually_static",
                    "motion_status": "unmeasured",
                    "motion": unmeasured_motion(),
                    "confidence": 1.0,
                    "bbox_xyxy": bounds,
                    "horizontal_position": (
                        "left" if centre_x < width / 3 else "right" if centre_x > 2 * width / 3 else "center"
                    ),
                    "horizontal_offset_normalized": round(2 * centre_x / width - 1, 3),
                    "distance_m": distance_m,
                    "distance_status": "estimated" if distance_m is not None else "unavailable",
                    "distance_method": self.distance_settings["mode"],
                    "distance_note": note,
                }
            )
        return detections
