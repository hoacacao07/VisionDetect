"""Bench-test the ESP32 without the camera. PUT THE ROBOT ON A STAND FIRST.

Watch telemetry only (commands 0, keeps the watchdog fed):
  .venv/bin/python tools/esp32_console.py
Spin both sides at 0.1 m/s for 3 s, then stop:
  .venv/bin/python tools/esp32_console.py --speed 0.1 --seconds 3
Watchdog test: run with --speed 0.1 --seconds 30, then unplug the USB cable.
The wheels must brake within 0.3 s.

No wheel encoders: "travel" and "speed" are ESTIMATES from the ESP32's motor
model. Speed calibration (on the floor, with the real load):
  .venv/bin/python tools/esp32_console.py --speed 0.15 --seconds 5
then compare the printed estimated travel with a tape measure.
"""

import argparse
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from vision_detection.esp32_link import Esp32Link
from vision_detection.robot_config import DEFAULT_ROBOT_SETTINGS_PATH, load_robot_settings


def show(value, unit="", width=6, decimals=2):
    return f"{'nan':>{width}}" if not math.isfinite(value) else f"{value:{width}.{decimals}f}{unit}"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default=str(DEFAULT_ROBOT_SETTINGS_PATH))
    parser.add_argument("--speed", type=float, default=0.0, help="m/s, both sides")
    parser.add_argument("--seconds", type=float, default=10.0)
    arguments = parser.parse_args()
    if not 0 <= arguments.speed <= 0.3:
        parser.error("Bench speed must be between 0 and 0.3 m/s")
    robot = load_robot_settings(arguments.robot)
    sensors = robot["sensors"]
    link = Esp32Link(
        robot["serial"]["port"],
        robot["serial"]["baud"],
        maximum_range_age_s=sensors["maximum_range_age_s"],
        speed_window_s=sensors["speed_window_s"],
    )
    start_travel = None
    try:
        started = time.monotonic()
        last_print = 0.0
        while time.monotonic() - started < arguments.seconds:
            link.send_speed(arguments.speed, arguments.speed)  # 20 Hz heartbeat.
            if link.error:
                raise SystemExit(f"Serial error: {link.error}")
            state = link.latest()
            if time.monotonic() - last_print >= 0.2:
                last_print = time.monotonic()
                if state is None:
                    print(
                        f"waiting for telemetry... ({link.rejected_lines} unreadable lines; "
                        f"last message: {link.last_diagnostic})"
                    )
                else:
                    if start_travel is None:
                        start_travel = state.travel_m
                    ranges = " ".join(show(r, "m", 5) for r in state.ranges_m)
                    print(
                        f"est. travel L {show(state.left_travel_m, 'm', 7, 3)} R {show(state.right_travel_m, 'm', 7, 3)} | "
                        f"est. speed L {show(state.left_speed_mps)} R {show(state.right_speed_mps)} | "
                        f"ranges L/C/R {ranges} | gyro {show(state.gyro_z_dps, 'dps')} "
                        f"heading {show(state.heading_deg, 'deg', 7, 1)} | age {state.age_s * 1000:3.0f} ms",
                        flush=True,
                    )
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        state = link.latest()
        link.close()
        print("Stopped.")
        if arguments.speed > 0 and state is not None and start_travel is not None:
            estimated = state.travel_m - start_travel
            print(
                f"Estimated travel: {estimated:.3f} m. Measure the real distance, then set
"
                "  SPEED_AT_FULL_DUTY_MPS = current value x real / estimated
"
                "in firmware/esp32_robot/include/robot_config.h, reflash and repeat "
                "until they agree within ~5 %."
            )
