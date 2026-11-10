"""Image-quality alert for a covered/dark camera, not proof of obstruction.

A dark room and a lens cover can look identical. A featureless wall can look
like an opaque cover. Messages deliberately describe that ambiguity.
NAV-HEALTH: this is one visibility signal, not a complete sensor-health system.
A future controller must also handle stale/missing frames and connection loss.
"""

import cv2
import numpy as np


class CameraHealthMonitor:
    """Debounce image visibility symptoms; a warning is not proof of a physical cover."""

    def __init__(self, settings=None):
        """Initialize debounce state; thresholds are image-quality heuristics."""
        self.settings = settings or {}
        self.suspect_since = None
        self.clear_since = None
        self.active_alert = None

    def update(self, frame, timestamp):
        """Debounce persistent poor visibility and clear after a stable recovery."""
        gray = cv2.cvtColor(cv2.resize(frame, (160, 120)), cv2.COLOR_BGR2GRAY)
        mean, std = float(gray.mean()), float(gray.std())
        dark_fraction = float(np.mean(gray < 20))
        dark = (
            mean < self.settings.get("dark_mean_threshold", 15) or dark_fraction > 0.98
        )
        flat = std < self.settings.get("flat_contrast_threshold", 5)
        suspect = dark or flat
        if suspect:
            # A brief exposure change should not immediately become a warning.
            self.clear_since = None
            if self.suspect_since is None:
                self.suspect_since = timestamp
            if timestamp - self.suspect_since >= self.settings.get(
                "alert_after_s", 1.0
            ):
                self.active_alert = (
                    "blocked_or_too_dark" if dark else "possibly_obscured"
                )
        else:
            # Recovery is debounced too, so one bright frame does not clear a
            # sustained obstruction while the view is still flickering.
            self.suspect_since = None
            if self.clear_since is None:
                self.clear_since = timestamp
            if timestamp - self.clear_since >= self.settings.get("clear_after_s", 0.3):
                self.active_alert = None
        message = None
        if self.active_alert == "blocked_or_too_dark":
            message = "ALERT: CAMERA BLOCKED / TOO DARK"
        elif self.active_alert:
            message = "ALERT: CAMERA VIEW MAY BE BLOCKED"
        return {
            "status": self.active_alert
            or ("checking_visibility" if suspect else "clear"),
            "alert": self.active_alert is not None,
            "message": message,
            "mean_brightness": round(mean, 1),
            "image_contrast": round(std, 1),
        }


def draw_camera_alert(preview, health):
    """Draw the warning last so boxes cannot hide it."""
    if not health["alert"]:
        return preview
    height, width = preview.shape[:2]
    top = max(32, height // 2 - 40)
    cv2.rectangle(preview, (0, top), (width, top + 80), (25, 30, 165), -1)
    scale = min(
        0.62,
        (width - 24)
        / max(
            1, cv2.getTextSize(health["message"], cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]
        ),
    )
    cv2.putText(
        preview,
        health["message"],
        (12, top + 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        preview,
        "Uncover the lens / open the shutter / improve lighting",
        (12, top + 57),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 255, 255),
        1,
    )
    return preview
