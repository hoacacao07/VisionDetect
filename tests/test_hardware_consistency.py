"""HARDWARE CONSISTENCY: pins, constants, configs and protocol must agree everywhere.

These catch "edited one file, forgot the other" mistakes between the firmware,
the fake ESP32, the Pi code and the camera configs.
"""

import json
import math
import re
from pathlib import Path
from sim.fake_esp32 import FakeEsp32
from sim.world import load_world
from vision_detection.camera_model import focal_length_px
from vision_detection.esp32_link import encode_command, parse_telemetry
from vision_detection.firmware_constants import load_firmware_constants
from vision_detection.settings import PROJECT_DIRECTORY

PINS_H = PROJECT_DIRECTORY / "firmware/esp32_robot/include/pins.h"
CAMERAS = PROJECT_DIRECTORY / "config/cameras"

# GPIOs broken out on the 30-pin ESP32-WROOM-32 DevKit.
DEVKIT_30_PIN = {0, 1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39}
FLASH = set(range(6, 12))
STRAPPING = {0, 2, 5, 12, 15}
USB_SERIAL = {1, 3}
INPUT_ONLY = {34, 35, 36, 39}


def pins():
    """{NAME: [gpio, ...]} from pins.h (scalars and {a, b, c} arrays)."""
    found = {}
    for name, value in re.findall(r"constexpr\s+\w+\s+(\w+)(?:\[\d+\])?\s*=\s*([^;]+);", PINS_H.read_text()):
        found[name] = [int(v) for v in re.findall(r"\d+", value)]
    return found


def test_every_pin_exists_on_the_30_pin_devkit():
    for name, gpios in pins().items():
        assert set(gpios) <= DEVKIT_30_PIN, name


def test_no_pin_is_used_twice():
    used = [gpio for gpios in pins().values() for gpio in gpios]
    assert len(used) == len(set(used))


def test_outputs_avoid_flash_strapping_usb_and_input_only_pins():
    outputs = [g for name, gpios in pins().items() if "ECHO" not in name for g in gpios]
    assert not set(outputs) & (FLASH | STRAPPING | USB_SERIAL | INPUT_ONLY)


def test_hc_sr04_echo_uses_input_only_pins():
    assert set(pins()["PIN_US_ECHO"]) <= INPUT_ONLY
    assert len(pins()["PIN_US_ECHO"]) == len(pins()["PIN_US_TRIG"]) == 3


def test_mpu6050_on_gpio_21_22():
    assert pins()["PIN_I2C_SDA"] == [21] and pins()["PIN_I2C_SCL"] == [22]


def test_hc_sr04_timing_in_firmware_header():
    c = load_firmware_constants()
    assert c["ULTRASONIC_TRIGGER_US"] == 10
    assert c["ULTRASONIC_PERIOD_MS"] == 30
    assert (c["ULTRASONIC_MIN_M"], c["ULTRASONIC_MAX_M"]) == (0.02, 4.0)
    # The echo timeout must cover the maximum range (round trip) - about 25 ms.
    assert c["ULTRASONIC_TIMEOUT_US"] >= 2 * c["ULTRASONIC_MAX_M"] / c["SPEED_OF_SOUND_MPS"] * 1e6


def test_no_us100_left_anywhere():
    for folder in ("firmware", "src", "sim", "config", "tests", "scripts", "tools"):
        for path in (PROJECT_DIRECTORY / folder).rglob("*"):
            if path.suffix in {".py", ".h", ".cpp", ".json", ".md", ".ini"} and ".pio" not in path.parts:
                if path.resolve() == Path(__file__).resolve():
                    continue
                assert "US-100" not in path.read_text(encoding="utf-8", errors="ignore"), path
    for doc in ("README.md", "WIRING.md"):
        assert "US-100" not in (PROJECT_DIRECTORY / doc).read_text(encoding="utf-8")


def test_fake_esp32_uses_only_real_firmware_constants():
    source = (PROJECT_DIRECTORY / "sim/fake_esp32.py").read_text()
    used = set(re.findall(r'c\["([A-Z0-9_]+)"\]', source))
    assert used, "the fake ESP32 should read firmware constants"
    assert used <= set(load_firmware_constants())
    assert FakeEsp32(load_world("corridor")).c == load_firmware_constants()


def test_protocol_round_trip_between_pi_and_fake():
    fake = FakeEsp32(load_world("corridor"))
    fake.receive(encode_command(7, 0.12, 0.12))
    assert fake.commands[-1][1:] == (7, 0.12, 0.12)
    lines = [line for line in fake.advance(100) if line.startswith("T,")]
    telemetry = parse_telemetry(lines[-1], 0.0)
    # Telemetry is sent from the loop, so the last one is within one period of "now".
    assert telemetry is not None
    assert 100 - fake.c["TELEMETRY_PERIOD_MS"] <= telemetry.esp_ms <= 100
    assert telemetry.left_travel_m > 0 and telemetry.right_travel_m > 0


def test_robot_camera_is_640x360_102_degrees():
    robot = json.loads((CAMERAS / "robot.json").read_text())
    assert (robot["camera"]["width"], robot["camera"]["height"]) == (640, 360)
    assert robot["distance"]["horizontal_fov_degrees"] == 102


def test_sim_camera_matches_robot_camera_except_sim_only_fields():
    robot = json.loads((CAMERAS / "robot.json").read_text())
    sim = json.loads((CAMERAS / "sim.json").read_text())
    sim_only = {
        ("camera", "driver"),
        ("detection", "model"),
        ("distance", "focal_calibration"),
        ("distance", "camera_height_m"),
        ("distance", "camera_downward_tilt_degrees"),
        ("output", "write_detection_log"),
    }
    for group, values in robot.items():
        if not isinstance(values, dict):
            continue
        for key, value in values.items():
            if (group, key) not in sim_only:
                assert sim[group][key] == value, f"{group}.{key} differs between robot.json and sim.json"


def test_sim_focal_length_is_the_102_degree_lens():
    sim = json.loads((CAMERAS / "sim.json").read_text())["distance"]
    from_fov = focal_length_px(dict(sim, focal_calibration={}), 640)
    assert math.isclose(sim["focal_calibration"]["focal_px"], from_fov, abs_tol=0.01)
