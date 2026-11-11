"""BENCH TEST: straight-line drive with the WHEELS LIFTED OFF THE GROUND.

Put the robot on a box so no wheel touches anything, then on the Pi:
  .venv/bin/python scripts/bench_straight.py
It keeps the robot still for 1 s (gyro baseline), drives both sides at
0.15 m/s for 3 s, brakes for 1 s, logs every telemetry line to
results/bench/straight-<time>.csv and prints PASS/FAIL for:
  * gyro Z stays near zero (the robot is not moving, so it should read ~0)
  * both sides got the same command, and the ESP32 applied the same speed
  * every HC-SR04 reading is sane (0.02-4.0 m or nan, mostly valid, steady)
  * telemetry arrives at ~20 Hz
Try it without hardware:  python scripts/bench_straight.py --serial "sim://corridor?lifted=1"
"""

import argparse
import csv
import math
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PROJECT / "src"), str(PROJECT)]
from vision_detection.esp32_link import Esp32Link  # noqa: E402
from vision_detection.firmware_constants import load_firmware_constants  # noqa: E402
from vision_detection.robot_config import load_robot_settings  # noqa: E402

GYRO_MEAN_LIMIT_DPS = 1.0  # Average |gyro Z| while driving.
GYRO_PEAK_LIMIT_DPS = 5.0  # Largest single |gyro Z| (motor vibration).
SIDE_MISMATCH_LIMIT = 0.02  # Estimated travel left vs right, fraction.
RANGE_VALID_FRACTION = 0.8  # At least 80 % of readings must be numbers, not nan.
RANGE_STEADY_STD_M = 0.05  # The robot isn't moving, so readings must be steady.


def run_bench(link, speed=0.15, drive_s=3.0, still_s=1.0, brake_s=1.0, rate_hz=20):
    """Drive the sequence and return (samples, commands). Commands every tick."""
    samples, commands = [], []
    started = time.monotonic()
    last_ms = None
    for phase, duration, command in (("still", still_s, 0.0), ("drive", drive_s, speed), ("brake", brake_s, 0.0)):
        phase_end = time.monotonic() + duration
        while time.monotonic() < phase_end:
            link.send_speed(command, command)
            commands.append((time.monotonic() - started, phase, command, command))
            state = link.latest()
            if state is not None and state.esp_ms != last_ms:
                last_ms = state.esp_ms
                samples.append(dict(
                    t=round(state.received_at - started, 3), phase=phase, esp_ms=state.esp_ms,
                    command_left=command, command_right=command,
                    est_left_m=state.left_travel_m, est_right_m=state.right_travel_m,
                    range_left_m=state.ranges_m[0], range_centre_m=state.ranges_m[1], range_right_m=state.ranges_m[2],
                    gyro_z_dps=state.gyro_z_dps, heading_deg=state.heading_deg,
                ))
            if link.error:
                raise RuntimeError(f"Serial link failed: {link.error}")
            time.sleep(1 / rate_hz)
    return samples, commands


def evaluate(samples, commands, limits=None):
    """Return a list of (check, passed, detail). Pure function: unit-tested."""
    c = limits or load_firmware_constants()
    results = []
    drive = [s for s in samples if s["phase"] == "drive"]
    gyro = [abs(s["gyro_z_dps"]) for s in drive if math.isfinite(s["gyro_z_dps"])]
    if len(gyro) < max(1, len(drive) // 2):
        results.append(("gyro Z near zero", False, "gyro missing (nan) - check the MPU-6050"))
    else:
        mean, peak = statistics.mean(gyro), max(gyro)
        results.append((
            "gyro Z near zero",
            mean < GYRO_MEAN_LIMIT_DPS and peak < GYRO_PEAK_LIMIT_DPS,
            f"mean |gyro| {mean:.2f} dps (limit {GYRO_MEAN_LIMIT_DPS}), peak {peak:.2f} dps (limit {GYRO_PEAK_LIMIT_DPS})",
        ))
    same = all(left == right for _, _, left, right in commands)
    if drive:
        left = drive[-1]["est_left_m"] - drive[0]["est_left_m"]
        right = drive[-1]["est_right_m"] - drive[0]["est_right_m"]
        mismatch = abs(left - right) / max(abs(left), abs(right), 1e-6)
        moved = left > 0.1 and right > 0.1
        results.append((
            "both sides same command and applied speed",
            same and moved and mismatch < SIDE_MISMATCH_LIMIT,
            f"commands identical: {same}; ESP32 estimated travel L {left:.3f} m, R {right:.3f} m "
            f"(mismatch {mismatch * 100:.1f} %, limit {SIDE_MISMATCH_LIMIT * 100:.0f} %)",
        ))
    else:
        results.append(("both sides same command and applied speed", False, "no telemetry while driving"))
    for index, name in enumerate(("left", "centre", "right")):
        values = [s[f"range_{name}_m"] for s in samples]
        valid = [v for v in values if math.isfinite(v)]
        in_range = all(c["ULTRASONIC_MIN_M"] <= v <= c["ULTRASONIC_MAX_M"] for v in valid)
        fraction = len(valid) / max(len(values), 1)
        spread = statistics.pstdev(valid) if len(valid) > 1 else 0.0
        detail = (
            f"{fraction * 100:.0f} % valid, "
            + (f"{min(valid):.3f}-{max(valid):.3f} m, std {spread * 100:.1f} cm" if valid else "no readings")
        )
        results.append((
            f"HC-SR04 {name} sane",
            in_range and fraction >= RANGE_VALID_FRACTION and spread < RANGE_STEADY_STD_M,
            detail,
        ))
    gaps = [(b["esp_ms"] - a["esp_ms"]) for a, b in zip(samples, samples[1:])]
    worst = max(gaps) if gaps else math.inf
    results.append(("telemetry ~20 Hz", worst <= 3 * c["TELEMETRY_PERIOD_MS"], f"largest gap {worst} ms"))
    return results


def save_log(samples, directory=PROJECT / "results" / "bench"):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"straight-{datetime.now():%Y%m%d-%H%M%S}.csv"
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(samples[0]) if samples else ["t"])
        writer.writeheader()
        writer.writerows(samples)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--serial", help="Serial port (default: serial.port in config/robot_navigation.json)")
    parser.add_argument("--speed", type=float, default=0.15)
    parser.add_argument("--seconds", type=float, default=3.0)
    arguments = parser.parse_args()
    if not 0 < arguments.speed <= 0.3:
        parser.error("Bench speed must be between 0 and 0.3 m/s")
    robot = load_robot_settings()
    port = arguments.serial or robot["serial"]["port"]
    print("WHEELS MUST BE OFF THE GROUND. Starting in 3 s (Ctrl+C to abort)...", flush=True)
    time.sleep(3)
    link = Esp32Link(
        port, robot["serial"]["baud"],
        maximum_range_age_s=robot["sensors"]["maximum_range_age_s"],
        speed_window_s=robot["sensors"]["speed_window_s"],
    )
    try:
        deadline = time.monotonic() + robot["serial"]["startup_timeout_s"]
        while link.latest() is None:
            if time.monotonic() > deadline:
                raise SystemExit(f"No telemetry from {port}. Last ESP32 message: {link.last_diagnostic}")
            time.sleep(0.05)
        time.sleep(1.5)  # The ESP32 resets on connect and calibrates its gyro (~1 s).
        samples, commands = run_bench(link, arguments.speed, arguments.seconds)
    finally:
        link.close()
    print(f"Log: {save_log(samples)}")
    results = evaluate(samples, commands)
    for check, passed, detail in results:
        print(f"{'PASS' if passed else 'FAIL'}  {check}: {detail}")
    raise SystemExit(0 if all(passed for _, passed, _ in results) else 1)


if __name__ == "__main__":
    main()
