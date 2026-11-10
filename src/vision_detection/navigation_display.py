"""Presentation-only side panel for the straight-line simulation."""

import cv2
import numpy as np


def append_navigation_panel(preview, decision, ego_speed, timestamp, phase):
    """Keep movement status beside the image so it never hides object labels."""
    height, width = preview.shape[:2]
    panel = np.full((height, 320, 3), (239, 244, 243), dtype=np.uint8)
    ink = (48, 64, 60)
    colors = {"STOP": (65, 65, 185), "SLOW": (20, 125, 170), "MOVE": (92, 135, 26)}

    def text(label, y, scale=0.49, color=ink, weight=1):
        """Draw one panel label using the shared position, font and colour defaults."""
        cv2.putText(
            panel, label, (22, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, weight
        )

    text("STRAIGHT-LINE ROBOT", 35, 0.57, weight=2)
    text("Camera + range sensors + encoders", 59, 0.40)
    cv2.rectangle(panel, (20, 82), (300, 143), colors[decision["state"]], -1)
    text(decision["state"], 124, 1.1, (255, 255, 255), 2)
    text("Measured robot speed", 179, 0.42)
    text(f"{ego_speed:.2f} m/s", 205, 0.74, weight=2)
    text("Forward command", 239, 0.42)
    text(f"{decision['target_speed_mps']:.2f} m/s", 264, 0.65)
    gap = decision["nearest_gap_m"]
    text("Nearest path clearance", 299, 0.42)
    text(f"{gap:.2f} m" if gap is not None else "Unknown", 324, 0.65)
    text("Using: " + decision["limiting_source"].replace("_", " "), 347, 0.38)
    text(decision["reason"].replace("_", " "), 378, 0.42)
    cv2.line(panel, (20, 396), (300, 396), (200, 212, 208), 1)
    text(phase.replace("_", " "), 420, 0.43)
    text(f"Simulated time: {timestamp:.1f} s", 444, 0.43)
    age = decision.get("vision_age_s")
    if age is not None:
        text(f"Vision age: {age:.2f} s", 466, 0.39)
    text("Q stops the test | No steering", height - 10, 0.39)
    return np.hstack((preview, panel))
