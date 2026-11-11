"""SIM PLOTS: top-down path and speed/command-vs-time PNGs for one scenario run."""

from pathlib import Path
import matplotlib

matplotlib.use("Agg")  # No window; just write PNG files.
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from .world import PROJECT_DIRECTORY, obstacle_rect

RESULTS_DIRECTORY = PROJECT_DIRECTORY / "results" / "sim"


def save_plots(result, directory=RESULTS_DIRECTORY):
    """Write <name>_path.png and <name>_speed.png; return both paths."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    name, rows = result.scenario.name, result.rows
    t = [r["t"] for r in rows]

    # Top-down path, with the obstacles at their final positions.
    figure, axes = plt.subplots(figsize=(8, 4))
    for obstacle in result.world["obstacles"]:
        x0, x1, y0, y1 = obstacle_rect(obstacle, t[-1] if t else 0)
        b, g, r = obstacle["color_bgr"]
        axes.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=(r / 255, g / 255, b / 255)))
    axes.plot([r["x"] for r in rows], [r["y"] for r in rows], "k-", label="robot (wheel axle centre)")
    if rows:
        axes.plot(rows[0]["x"], rows[0]["y"], "go", label="start")
        axes.plot(rows[-1]["x"], rows[-1]["y"], "rs", label="end")
    axes.set_aspect("equal")
    axes.set_xlabel("x forward (m)")
    axes.set_ylabel("y left (m)")
    axes.set_title(f"{name}: top-down path")
    axes.legend(loc="upper left", fontsize=8)
    axes.grid(True, alpha=0.3)
    path_png = directory / f"{name}_path.png"
    figure.tight_layout()
    figure.savefig(path_png, dpi=110)
    plt.close(figure)

    # Speeds and commands over time.
    figure, axes = plt.subplots(figsize=(8, 4))
    axes.plot(t, [r["true_speed"] for r in rows], label="true speed (sim)")
    pi = [r for r in rows if r["pi_alive"]]
    axes.plot([r["t"] for r in pi], [r["command"] for r in pi], "--", label="Pi command (applied)")
    axes.plot([r["t"] for r in pi], [r["target"] for r in pi], ":", label="policy target")
    axes.plot([r["t"] for r in pi], [r["estimated_speed"] for r in pi], "-.", label="ESP32 estimated speed")
    stops = [r["t"] for r in pi if r["state"] == "STOP"]
    if stops:
        axes.plot(stops, [0] * len(stops), "r|", markersize=8, label="decision STOP")
    if result.scenario.pi_quiet_at_s is not None:
        axes.axvline(result.scenario.pi_quiet_at_s, color="grey", label="Pi goes quiet")
    if result.scenario.camera_stops_at_s is not None:
        axes.axvline(result.scenario.camera_stops_at_s, color="grey", label="camera stops")
    if result.scenario.sensor_nan_at_s is not None:
        axes.axvline(result.scenario.sensor_nan_at_s, color="grey", label="ultrasonic -> nan")
    axes.set_xlabel("time (s)")
    axes.set_ylabel("m/s")
    axes.set_title(f"{name}: speed and command")
    axes.legend(loc="upper right", fontsize=8)
    axes.grid(True, alpha=0.3)
    speed_png = directory / f"{name}_speed.png"
    figure.tight_layout()
    figure.savefig(speed_png, dpi=110)
    plt.close(figure)
    return path_png, speed_png
