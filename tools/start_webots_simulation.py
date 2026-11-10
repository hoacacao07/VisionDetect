"""Launch our separate Webots world with this project's Python environment."""

import os
from pathlib import Path
import subprocess
import sys

PROJECT = Path(__file__).resolve().parents[1]
simulation = PROJECT / "simulation"
moving = "--moving" in sys.argv
controller_name = "straight_robot" if moving else "vision_test"
world_name = "straight_robot.wbt" if moving else "vision_test.wbt"
webots_home = Path(os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots"))
webots = webots_home / "msys64" / "mingw64" / "bin" / "webots.exe"
if not webots.is_file():
    raise SystemExit("Webots was not found. Install Webots R2025a or set WEBOTS_HOME.")

# Webots normally selects a global Python. Use the project environment instead;
# regenerate this local path on every launch so moving the folder still works.
(simulation / "controllers" / controller_name / "runtime.ini").write_text(
    "[python]\nCOMMAND = " + Path(sys.executable).as_posix() + "\n",
    encoding="utf-8",
)
log_directory = simulation / ("moving_results" if moving else "results")
log_directory.mkdir(exist_ok=True)
with (log_directory / "webots_console.log").open("w", encoding="utf-8") as console:
    subprocess.Popen(
        [
            str(webots),
            "--mode=realtime",
            "--stdout",
            "--stderr",
            str(simulation / "worlds" / world_name),
        ],
        cwd=simulation,
        stdout=console,
        stderr=subprocess.STDOUT,
        env={**os.environ, "VISION_SIM_STRESS": "1" if "--stress" in sys.argv else "0"},
    )
print(
    "Webots is opening. The test pauses after "
    + ("65" if moving else "40")
    + " simulated seconds."
)
