"""SIM WORLD: obstacles on a flat floor, robot geometry and ultrasonic ray casting.

World coordinates: x forward from the start, y to the left, z up (metres).
Robot pose: (x, y, heading_rad) of the midpoint between the wheels.
Obstacles are axis-aligned boxes; an optional "velocity_mps": [vx, vy] moves them.
The camera renderer (render_scene.py) and the HC-SR04 model both use this
module, so the camera sees exactly what the ultrasonic rays hit.
"""

import json
import math
from pathlib import Path

SIM_DIRECTORY = Path(__file__).resolve().parent
PROJECT_DIRECTORY = SIM_DIRECTORY.parent
WORLD_DIRECTORY = SIM_DIRECTORY / "worlds"


def load_world(name_or_path):
    """Load sim/worlds/<name>.json, or a JSON file path."""
    path = Path(name_or_path)
    if not path.suffix:
        path = WORLD_DIRECTORY / f"{name_or_path}.json"
    world = json.loads(path.read_text(encoding="utf-8"))
    world.setdefault("name", path.stem)
    for obstacle in world["obstacles"]:
        obstacle.setdefault("velocity_mps", [0.0, 0.0])
    return world


def load_robot_sim(path=SIM_DIRECTORY / "robot_sim.json"):
    """Physical parameters that only the simulator needs (see the file's comment)."""
    return json.loads(Path(path).read_text(encoding="utf-8"))


def start_pose(world):
    """Robot start pose (x, y, heading_rad)."""
    start = world["robot_start"]
    return start["x"], start["y"], math.radians(start["heading_deg"])


def obstacle_rect(obstacle, t):
    """(x_min, x_max, y_min, y_max) of an obstacle at time t seconds."""
    vx, vy = obstacle["velocity_mps"]
    return (
        obstacle["x_min"] + vx * t,
        obstacle["x_max"] + vx * t,
        obstacle["y_min"] + vy * t,
        obstacle["y_max"] + vy * t,
    )


def body_point(pose, forward, left):
    """World (x, y) of a point given in the robot frame."""
    x, y, heading = pose
    return (
        x + forward * math.cos(heading) - left * math.sin(heading),
        y + forward * math.sin(heading) + left * math.cos(heading),
    )


def camera_pose(pose, robot, front_offset_m):
    """Camera (x, y, heading): on the centre line, front_offset_m behind the bumper."""
    x, y = body_point(pose, robot["axle_to_bumper_m"] - front_offset_m, 0.0)
    return x, y, pose[2]


def ray_distance(origin, angle, world, t, max_range=10.0):
    """Distance from origin along angle to the nearest obstacle edge, or inf."""
    ox, oy = origin
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for obstacle in world["obstacles"]:
        x_min, x_max, y_min, y_max = obstacle_rect(obstacle, t)
        # Slab method for a ray against an axis-aligned rectangle.
        t_near, t_far = -math.inf, math.inf
        hit = True
        for o, d, low, high in ((ox, dx, x_min, x_max), (oy, dy, y_min, y_max)):
            if abs(d) < 1e-12:
                if not low <= o <= high:
                    hit = False
                    break
                continue
            a, b = sorted(((low - o) / d, (high - o) / d))
            t_near, t_far = max(t_near, a), min(t_far, b)
        if hit and t_near <= t_far and t_far >= 0:
            distance = max(t_near, 0.0)
            if distance < best:
                best = distance
    return best if best <= max_range else math.inf


def ultrasonic_true_range(pose, sensor, robot, world, t):
    """Noise-free HC-SR04 range: nearest hit over 3 rays across the beam."""
    origin = body_point(pose, robot["axle_to_bumper_m"], sensor["left_m"])
    centre = pose[2] + math.radians(sensor["angle_deg"])
    half = math.radians(robot["ultrasonic_beam_half_angle_deg"])
    return min(ray_distance(origin, centre + k * half, world, t) for k in (-1, 0, 1))


def bumper_clearance(pose, robot, world, t):
    """True gap straight ahead of the bumper (across the body width), or inf."""
    half = robot["body_half_width_m"]
    return min(
        ray_distance(body_point(pose, robot["axle_to_bumper_m"], left), pose[2], world, t)
        for left in (-half, 0.0, half)
    )
