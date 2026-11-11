"""SCENE RENDERER: draw what the robot's camera sees, using the shared camera model.

Every pixel comes from vision_detection.camera_model, the same model that
distance.py inverts. The floor is a grey checkerboard (so the camera-health
check sees texture), and obstacles are flat-shaded boxes. No lens distortion,
blur or lighting: the images test GEOMETRY, not detector robustness.
"""

import math
import cv2
import numpy as np
from vision_detection.camera_model import (
    focal_length_px,
    point_depth,
    project_point,
)
from .world import camera_pose, obstacle_rect

SKY_BGR = (205, 195, 185)
FLOOR_DARK, FLOOR_LIGHT = 95, 125
TILE_M = 0.25
NEAR_M = 0.02
SUBPIXEL_BITS = 4  # cv2.fillPoly fixed-point precision (1/16 pixel).


def _to_camera(point, camera):
    """World (x, y, z) -> camera-relative (forward, left, up)."""
    cx, cy, heading = camera
    dx, dy = point[0] - cx, point[1] - cy
    return (
        dx * math.cos(heading) + dy * math.sin(heading),
        -dx * math.sin(heading) + dy * math.cos(heading),
        point[2],
    )


def _floor(width, height, camera, distance_settings):
    """Checkerboard floor and plain sky, computed for every pixel at once."""
    focal = focal_length_px(distance_settings, width)
    tilt = math.radians(distance_settings["camera_downward_tilt_degrees"])
    # float32 keeps memory small (the whole image is computed at once).
    u = ((np.arange(width) + 0.5 - width / 2) / focal).astype(np.float32)
    v = ((np.arange(height) + 0.5 - height / 2) / focal).astype(np.float32)
    ray_right, ray_down = np.meshgrid(u, v)
    downward = math.sin(tilt) + ray_down * math.cos(tilt)
    forward = math.cos(tilt) - ray_down * math.sin(tilt)
    ground = downward > 1e-3
    scale = np.where(ground, distance_settings["camera_height_m"] / np.maximum(downward, 1e-3), 0)
    f, left = scale * forward, -scale * ray_right
    cx, cy, heading = camera
    x = cx + f * math.cos(heading) - left * math.sin(heading)
    y = cy + f * math.sin(heading) + left * math.cos(heading)
    checker = (np.floor(x / TILE_M) + np.floor(y / TILE_M)) % 2
    shade = np.where(checker == 0, FLOOR_DARK, FLOOR_LIGHT).astype(np.uint8)
    image = np.empty((height, width, 3), np.uint8)
    image[:] = SKY_BGR
    image[ground] = np.stack([shade[ground]] * 3, axis=-1)
    return image


def _box_faces(rect, height):
    """Four sides and the top of a box, as lists of world points."""
    x0, x1, y0, y1 = rect
    bottom = [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)]
    top = [(x, y, height) for x, y, _ in bottom]
    faces = [(top, 1.15)]
    for i in range(4):
        j = (i + 1) % 4
        brightness = 1.0 if i % 2 == 1 else 0.8  # x-facing vs y-facing sides.
        faces.append(([bottom[i], bottom[j], top[j], top[i]], brightness))
    return faces


def _clip_near(polygon, distance_settings):
    """Sutherland-Hodgman clip of a camera-relative polygon to depth >= NEAR_M."""
    result = []
    depths = [point_depth(*p, distance_settings) - NEAR_M for p in polygon]
    for i, point in enumerate(polygon):
        nxt, d0, d1 = polygon[(i + 1) % len(polygon)], depths[i], depths[(i + 1) % len(polygon)]
        if d0 >= 0:
            result.append(point)
        if (d0 >= 0) != (d1 >= 0):
            k = d0 / (d0 - d1)
            result.append(tuple(a + k * (b - a) for a, b in zip(point, nxt)))
    return result


def render_scene(world, pose, t, distance_settings, robot, front_offset_m, width=640, height=360):
    """Return a BGR image of the world from the robot's camera at time t."""
    camera = camera_pose(pose, robot, front_offset_m)
    image = _floor(width, height, camera, distance_settings)
    faces = []
    for obstacle in world["obstacles"]:
        color = np.array(obstacle["color_bgr"], float)
        for polygon, brightness in _box_faces(obstacle_rect(obstacle, t), obstacle["height"]):
            local = _clip_near([_to_camera(p, camera) for p in polygon], distance_settings)
            if len(local) < 3:
                continue
            pixels = [project_point(*p, width, height, distance_settings) for p in local]
            depth = sum(p[2] for p in pixels) / len(pixels)
            shade = tuple(int(c) for c in np.clip(color * brightness, 0, 255))
            faces.append((depth, [(p[0], p[1]) for p in pixels], shade))
    # Painter's algorithm: far faces first, near faces paint over them.
    for _, points, shade in sorted(faces, key=lambda face: -face[0]):
        # camera_model puts pixel i's centre at i + 0.5; OpenCV puts it at i.
        fixed = np.round((np.array(points) - 0.5) * (1 << SUBPIXEL_BITS)).astype(np.int32)
        cv2.fillPoly(image, [fixed], shade, lineType=cv2.LINE_8, shift=SUBPIXEL_BITS)
    return image
