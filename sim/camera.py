"""SIM CAMERA: a camera adapter that renders the simulated world (driver "sim").

Selected by camera.driver = "sim" (config/cameras/sim.json). Uses the robot
pose from the running fake ESP32 (robot.py --serial sim://<world>), so the
camera sees the same obstacles the simulated HC-SR04 rays hit. Without a
running fake ESP32 it shows the corridor world from the start pose.
"""

import time
from . import fake_esp32
from .render_scene import render_scene
from .world import PROJECT_DIRECTORY, load_robot_sim, load_world, start_pose


class SimCameraInput:
    """Same read()/close() interface as OpenCVCameraInput and Picamera2Input."""

    def __init__(self, config, distance_settings):
        import json

        if distance_settings is None:
            raise ValueError("The sim camera needs the distance settings (camera height/tilt)")
        self.config = config
        self.distance = distance_settings
        self.robot = load_robot_sim()
        navigation = json.loads((PROJECT_DIRECTORY / "config/robot_navigation.json").read_text())
        self.front_offset_m = navigation["navigation"]["front_offset_m"]
        self.period = 1 / (config.get("fps") or 10)
        self.next_frame = time.monotonic()
        self.static_world = load_world("corridor")

    def read(self):
        """Wait for the next frame time, then render the current pose."""
        delay = self.next_frame - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.next_frame = max(self.next_frame + self.period, time.monotonic())
        port = fake_esp32.ACTIVE_PORT
        if port is not None:
            pose, t = port.pose_and_time()
            world = port.fake.world
        else:
            world, t = self.static_world, 0.0
            pose = start_pose(world)
        frame = render_scene(
            world, pose, t, self.distance, self.robot, self.front_offset_m,
            self.config["width"], self.config["height"],
        )
        return True, frame

    def close(self):
        pass
