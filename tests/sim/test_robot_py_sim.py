"""END TO END: the real run_robot() (robot.py) against the fake ESP32, in REAL time.

Same as:  python robot.py --serial sim://wall_1_5m --config config/cameras/sim.json --drive
Threads, serial parsing, the camera thread and the dashboard all run for real,
so these tests take about 25 s.
"""

import threading
import time
import pytest
from sim import fake_esp32
from sim.world import bumper_clearance
from vision_detection.robot_config import load_robot_settings
from vision_detection.robot_runtime import run_robot
from vision_detection.settings import PROJECT_DIRECTORY, load_settings

SIM_CAMERA = PROJECT_DIRECTORY / "config/cameras/sim.json"


def start_robot(world, drive):
    robot = load_robot_settings()
    robot["serial"]["port"] = f"sim://{world}"
    robot["output"].update(api_port=18879, write_log=False)  # Don't clash with a running robot.py.
    stop = threading.Event()
    errors = []

    def target():
        try:
            run_robot(load_settings(SIM_CAMERA), robot, motors_enabled=drive, stop_event=stop)
        except Exception as error:  # Reported by the test, not lost in the thread.
            errors.append(error)

    thread = threading.Thread(target=target, daemon=True)
    thread.start()
    deadline = time.monotonic() + 15
    while fake_esp32.ACTIVE_PORT is None and time.monotonic() < deadline:
        time.sleep(0.05)
    assert fake_esp32.ACTIVE_PORT is not None, errors
    return fake_esp32.ACTIVE_PORT, stop, thread, errors


def finish(stop, thread, errors):
    stop.set()
    thread.join(timeout=10)
    assert not thread.is_alive()
    assert not errors, errors


def test_robot_py_drives_and_stops_before_the_wall():
    port, stop, thread, errors = start_robot("wall_1_5m", drive=True)
    fake = port.fake
    deadline, moved, stopped_since = time.monotonic() + 40, False, None
    while time.monotonic() < deadline:
        time.sleep(0.2)
        with port.lock:
            speed, x = sum(fake.wheel_speed) / 2, fake.pose[0]
        moved = moved or x > 0.3
        if moved and abs(speed) < 0.001:
            stopped_since = stopped_since or time.monotonic()
            if time.monotonic() - stopped_since > 1.5:
                break
        else:
            stopped_since = None
    with port.lock:
        clearance = bumper_clearance(fake.pose, fake.robot, fake.world, fake.now_ms / 1000)
        commands = list(fake.commands)
    finish(stop, thread, errors)
    stop_gap = load_robot_settings()["navigation"]["stop_gap_m"]
    print(f"robot.py stopped with {clearance:.3f} m true clearance (stop_gap {stop_gap} m)")
    assert moved and stopped_since is not None, "robot.py did not drive and then stop"
    assert clearance >= stop_gap
    assert max(left for _, _, left, _ in commands) > 0


def test_robot_py_without_drive_flag_sends_only_zero():
    port, stop, thread, errors = start_robot("corridor", drive=False)
    time.sleep(5)
    with port.lock:
        commands = list(port.fake.commands)
        x = port.fake.pose[0]
    finish(stop, thread, errors)
    assert len(commands) > 50  # ~20 Hz heartbeat.
    assert all(left == 0 and right == 0 for _, _, left, right in commands)
    assert abs(x) < 0.001
