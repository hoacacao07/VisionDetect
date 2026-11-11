"""DELIVERY ROBOT entry point: Pi camera + YOLOE + ESP32 motor controller.

Dry run (motors held at 0):  .venv/bin/python robot.py
Drive:                       .venv/bin/python robot.py --drive
Camera settings: config/cameras/robot.json. Robot limits: config/robot_navigation.json.
Wiring and bring-up: README.md -> "Delivery robot (Pi + ESP32)".
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))
from vision_detection.robot_runtime import main

if __name__ == "__main__":
    main()
