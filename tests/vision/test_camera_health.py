"""CAMERA HEALTH: black, covered-lens and very dark frames alert; a normal frame does not.

The monitor only alerts after a symptom lasts alert_after_s (1 s), so each
frame type is fed for 1.5 s.
"""

import json
import numpy as np
import pytest
from sim.render_scene import render_scene
from sim.world import load_robot_sim, load_world, start_pose
from vision_detection.camera_health import CameraHealthMonitor
from vision_detection.settings import PROJECT_DIRECTORY, load_settings

DISTANCE = load_settings(PROJECT_DIRECTORY / "config/cameras/sim.json")["distance"]
FRONT_OFFSET = json.loads((PROJECT_DIRECTORY / "config/robot_navigation.json").read_text())["navigation"]["front_offset_m"]


def normal_frame():
    corridor = load_world("corridor")
    return render_scene(corridor, start_pose(corridor), 0.0, DISTANCE, load_robot_sim(), FRONT_OFFSET)


def frames():
    rng = np.random.default_rng(3)
    normal = normal_frame()
    covered = np.clip(128 + rng.normal(0, 1.0, normal.shape), 0, 255).astype(np.uint8)
    very_dark = (normal.astype(float) * 0.06).astype(np.uint8)  # Texture remains, mean ~7.
    return {
        "black": (np.zeros_like(normal), "blocked_or_too_dark"),
        "covered_lens": (covered, "possibly_obscured"),
        "very_dark": (very_dark, "blocked_or_too_dark"),
        "normal": (normal, None),
    }


def health_after(frame, seconds=1.5, fps=10):
    monitor = CameraHealthMonitor()
    health = None
    for i in range(int(seconds * fps) + 1):
        health = monitor.update(frame, i / fps)
    return health


@pytest.mark.parametrize("name", ["black", "covered_lens", "very_dark", "normal"])
def test_camera_health(name):
    frame, expected = frames()[name]
    health = health_after(frame)
    print(f"{name}: mean {health['mean_brightness']}, contrast {health['image_contrast']} -> {health['status']}")
    if expected is None:
        assert not health["alert"]
        assert health["status"] == "clear"
    else:
        assert health["alert"]
        assert health["status"] == expected


def test_very_dark_frame_still_has_texture():
    # Guards the test: "very dark" must be dark, not just a black frame again.
    frame, _ = frames()["very_dark"]
    assert frame.mean() < 15 and frame.std() > 0.5
