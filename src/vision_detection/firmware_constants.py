"""FIRMWARE CONSTANTS: read firmware/esp32_robot/include/robot_config.h from Python.

robot_config.h is the single source of truth for the ESP32 motor model, ramp,
heading hold, ultrasonic timing and watchdog. The fake ESP32 (sim/) and the Pi
settings check read the header itself, so they cannot drift from the firmware.

Only simple lines are understood:  constexpr <type> NAME = <literal>;
where <literal> is a number (0.35f, 1000, 0x68) or true/false.
"""

import re
from .settings import PROJECT_DIRECTORY

ROBOT_CONFIG_HEADER = PROJECT_DIRECTORY / "firmware" / "esp32_robot" / "include" / "robot_config.h"

_LINE = re.compile(r"^\s*constexpr\s+\w+\s+(\w+)\s*=\s*([^;]+);")


def _literal(text):
    """Convert one C++ literal to a Python bool/int/float."""
    text = text.strip()
    if text in ("true", "false"):
        return text == "true"
    if text.lower().startswith("0x"):
        return int(text, 16)
    text = text.rstrip("fFuUlL")
    return float(text) if any(c in text for c in ".eE") else int(text)


def load_firmware_constants(path=ROBOT_CONFIG_HEADER):
    """Return {NAME: value} for every constexpr in the header."""
    constants = {}
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        match = _LINE.match(line.split("//")[0])
        if not match:
            continue
        name, value = match.groups()
        try:
            constants[name] = _literal(value)
        except ValueError as error:
            raise ValueError(
                f"{path.name}:{number}: {name} must be a plain literal, not {value!r}"
            ) from error
    return constants
