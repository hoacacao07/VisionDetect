"""The hardware scripts, run against the simulator so they're known to work before the Pi."""

import copy
import csv
import importlib.util
import math
import pytest
from sim import fake_esp32
from sim.render_scene import render_scene
from sim.world import camera_pose, load_robot_sim
from vision_detection.esp32_link import Esp32Link
from vision_detection.robot_config import load_robot_settings
from vision_detection.settings import PROJECT_DIRECTORY, load_settings

SIM_SETTINGS = load_settings(PROJECT_DIRECTORY / "config/cameras/sim.json")


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, PROJECT_DIRECTORY / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bench_straight = load_script("bench_straight")
eval_vision = load_script("eval_vision")
bench_vision_pi = load_script("bench_vision_pi")


# ---------------------------------------------------------------- bench_straight
def test_bench_straight_passes_on_lifted_fake_robot():
    robot = load_robot_settings()
    link = Esp32Link("sim://corridor?lifted=1", maximum_range_age_s=robot["sensors"]["maximum_range_age_s"])
    try:
        while link.latest() is None:
            pass
        samples, commands = bench_straight.run_bench(link, 0.15, drive_s=1.5, still_s=0.5, brake_s=0.5)
    finally:
        link.close()
    results = bench_straight.evaluate(samples, commands)
    for check, passed, detail in results:
        print(f"{'PASS' if passed else 'FAIL'} {check}: {detail}")
    assert all(passed for _, passed, _ in results)
    assert fake_esp32.ACTIVE_PORT is None  # Port closed cleanly.


def good_samples():
    rows = []
    for i in range(60):
        phase = "still" if i < 10 else "drive" if i < 50 else "brake"
        travel = max(0, i - 10) * 0.0075
        rows.append(dict(
            phase=phase, esp_ms=i * 50, est_left_m=travel, est_right_m=travel,
            range_left_m=2.5, range_centre_m=3.7, range_right_m=2.5, gyro_z_dps=0.02,
        ))
    return rows


COMMANDS = [(0, "drive", 0.15, 0.15)]


def failing(samples):
    return {check for check, passed, _ in bench_straight.evaluate(samples, COMMANDS) if not passed}


def test_bench_straight_checks_pass_on_good_data():
    assert failing(good_samples()) == set()


def test_bench_straight_flags_a_spinning_gyro():
    samples = good_samples()
    for row in samples:
        row["gyro_z_dps"] = 3.0
    assert failing(samples) == {"gyro Z near zero"}


def test_bench_straight_flags_unequal_sides():
    samples = good_samples()
    for row in samples:
        row["est_right_m"] *= 0.9
    assert failing(samples) == {"both sides same command and applied speed"}


def test_bench_straight_flags_a_dead_ultrasonic():
    samples = good_samples()
    for row in samples:
        row["range_centre_m"] = math.nan
    assert failing(samples) == {"HC-SR04 centre sane"}


def test_bench_straight_flags_different_commands():
    assert "both sides same command and applied speed" in {
        check for check, passed, _ in bench_straight.evaluate(good_samples(), [(0, "drive", 0.15, 0.10)])
        if not passed
    }


# ---------------------------------------------------------------- eval_vision
def test_eval_vision_on_rendered_images(tmp_path, monkeypatch):
    # Same code path as with real images, using the TEST-ONLY colour detector.
    monkeypatch.setattr(eval_vision, "OUTPUT", tmp_path / "out")
    (tmp_path / "out").mkdir()
    distance = SIM_SETTINGS["distance"]
    robot = load_robot_sim()
    front_offset = load_robot_settings()["navigation"]["front_offset_m"]
    camera_x = camera_pose((0, 0, 0), robot, front_offset)[0]
    rows = []
    for i, d in enumerate((0.5, 1.0, 2.0, 3.0)):
        front = camera_x + d
        world = dict(obstacles=[dict(x_min=front, x_max=front + 0.3, y_min=-0.15, y_max=0.15,
                                     height=0.3, color_bgr=[40, 40, 220], velocity_mps=[0, 0])])
        image = render_scene(world, (0, 0, 0), 0, distance, robot, front_offset)
        import cv2

        cv2.imwrite(str(tmp_path / f"img_{i}.png"), image)
        rows.append([f"img_{i}.png", "box", d])
    truth = tmp_path / "truth.csv"
    with truth.open("w", newline="") as file:
        csv.writer(file).writerows([["file", "class", "true_distance_m"], ["# comment row", "", ""], *rows])
    results, times = eval_vision.evaluate_size(
        copy.deepcopy(SIM_SETTINGS), 320, None, tmp_path, eval_vision.read_truth(truth), SIM_SETTINGS["motion"]
    )
    eval_vision.print_table(320, results, times)
    assert [r["detected"] for r in results] == [True] * 4
    assert all(r["error"] < 0.10 and not r["flag"] for r in results)
    assert len(list((tmp_path / "out" / "imgsz320").iterdir())) == 4


def test_eval_vision_flags_a_bad_estimate():
    detection = dict(label="person", bbox_xyxy=[300, 100, 340, 300], distance_m=1.5, confidence=0.9)
    assert eval_vision.best_match([detection], "person", 640) is detection
    assert eval_vision.best_match([detection], "car", 640) is None


# ---------------------------------------------------------------- bench_vision_pi
def test_bench_vision_runs_with_sim_camera():
    result = bench_vision_pi.bench(copy.deepcopy(SIM_SETTINGS), seconds=2, maximum_vision_age_s=0.8)
    print(result)
    assert result["fps"] > 3
    assert result["stale_fraction"] == 0
    assert result["inference_p95_ms"] >= result["inference_mean_ms"] * 0.5


def test_percentile():
    assert bench_vision_pi.percentile(list(range(101)), 0.95) == 95
    assert math.isnan(bench_vision_pi.percentile([], 0.95))
