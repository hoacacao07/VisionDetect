"""ROBOT SCENARIOS: the real Pi control code against the fake ESP32 (simulated time).

Each scenario runs once (module-scoped fixture) and saves two plots to
results/sim/. Pass/fail limits are written next to each check.
Run just these:  python -m pytest tests/sim -v
"""

import math
import pytest
from sim.fake_esp32 import FakeEsp32
from sim.plots import save_plots
from sim.runner import Scenario, run_scenario
from sim.world import load_world
from vision_detection.firmware_constants import load_firmware_constants
from vision_detection.robot_config import load_robot_settings

FIRMWARE = load_firmware_constants()
ROBOT = load_robot_settings()
NAVIGATION, CONTROL = ROBOT["navigation"], ROBOT["control"]
CONTROL_PERIOD_S = 1 / CONTROL["rate_hz"]


def run(scenario):
    result = run_scenario(scenario)
    save_plots(result)
    return result


def pi_rows(result):
    return [row for row in result.rows if row["pi_alive"]]


# ---------------------------------------------------------------- straight_line
@pytest.fixture(scope="module")
def straight():
    return run(Scenario("straight_line", "corridor", duration_s=30, stop_after_travel_m=2.0))


def test_straight_line_drives_two_metres(straight):
    assert straight.rows[-1]["x"] >= 2.0


def test_straight_line_lateral_drift_under_5cm(straight):
    drift = max(abs(row["y"]) for row in straight.rows)
    print(f"max lateral drift over 2 m: {drift * 100:.2f} cm")
    assert drift < 0.05


def test_straight_line_heading_error_under_3deg(straight):
    heading = abs(straight.rows[-1]["heading_deg"])
    print(f"final heading error: {heading:.2f} deg")
    assert heading < 3.0


def test_straight_line_ran_at_cruise_speed(straight):
    # Proves the corridor did not just make the robot crawl in SLOW.
    assert any(row.get("state") == "MOVE" for row in straight.rows)


def test_motor_mismatch_is_real_without_heading_hold():
    # Same run with heading hold disabled must FAIL the drift limit; otherwise
    # the straight_line test would not be testing the heading hold at all.
    result = run(
        Scenario(
            "straight_line_no_heading_hold", "corridor", duration_s=30,
            stop_after_travel_m=1.0, firmware_overrides={"HEADING_HOLD_MPS_PER_DEG": 0.0},
        )
    )
    drift = max(abs(row["y"]) for row in result.rows)
    print(f"drift without heading hold after <= 1 m: {drift * 100:.1f} cm")
    assert drift > 0.05


# ---------------------------------------------------------------- stop_before_wall
@pytest.fixture(scope="module")
def wall():
    return run(Scenario("stop_before_wall", "wall_1_5m", duration_s=30))


def test_stop_before_wall_keeps_stop_gap(wall):
    final = wall.rows[-1]
    print(f"final true clearance {final['clearance_m']:.3f} m, stop_gap {NAVIGATION['stop_gap_m']} m")
    assert final["state"] == "STOP"
    assert abs(final["true_speed"]) < 0.005
    assert final["clearance_m"] >= NAVIGATION["stop_gap_m"]


def test_stop_before_wall_never_touches(wall):
    assert min(row["clearance_m"] for row in wall.rows) > 0


def test_stop_before_wall_actually_drove(wall):
    assert wall.rows[-1]["x"] - wall.rows[0]["x"] > 0.5


# ---------------------------------------------------------------- pi_goes_quiet
@pytest.fixture(scope="module")
def quiet():
    return run(Scenario("pi_goes_quiet", "corridor", duration_s=7, pi_quiet_at_s=5.0))


def test_pi_goes_quiet_esp32_brakes_within_300ms(quiet):
    # Ignore the zero commands Esp32Link.close() sends when the run ends.
    last_command_ms = max(ms for ms, *_ in quiet.fake.commands if ms < 5000)
    watchdog = [ms for ms, text in quiet.fake.events if text.startswith("watchdog")]
    assert watchdog, "the fake ESP32 never braked"
    delay = watchdog[0] - last_command_ms
    print(f"brake {delay} ms after the last command (limit {FIRMWARE['COMMAND_TIMEOUT_MS']} ms)")
    assert delay <= FIRMWARE["COMMAND_TIMEOUT_MS"]


def test_pi_goes_quiet_robot_was_moving_then_stops(quiet):
    before = [row for row in quiet.rows if row["t"] < 5.0]
    assert before[-1]["true_speed"] > 0.1
    assert quiet.rows[-1]["braking"] and abs(quiet.rows[-1]["true_speed"]) < 0.005


def test_bad_line_brakes_immediately():
    fake = FakeEsp32(load_world("corridor"))
    for seq in range(40):
        fake.receive(f"C,{seq},0.150,0.150\n".encode())
        fake.advance(50)
    assert not fake.braking()
    fake.receive(b"C,41,abc,0.1\n")
    assert fake.braking()  # Same call: no control tick needed.
    assert fake.events[-1][1].startswith("rejected")


# ---------------------------------------------------------------- sensor_dropout
@pytest.fixture(scope="module")
def dropout():
    return run(Scenario("sensor_dropout", "corridor", duration_s=8, sensor_nan_at_s=5.0))


def test_sensor_dropout_commands_stop(dropout):
    after = [row for row in pi_rows(dropout) if row["t"] >= 5.0]
    first_stop = next(row for row in after if row["state"] == "STOP")
    delay = first_stop["t"] - 5.0
    print(f"STOP {delay * 1000:.0f} ms after the centre HC-SR04 went nan ({first_stop['reason']})")
    assert first_stop["reason"] == "range_sensor_unavailable"
    # The sensor is re-read every 90 ms, plus one control tick.
    assert delay <= 3 * FIRMWARE["ULTRASONIC_PERIOD_MS"] / 1000 + CONTROL_PERIOD_S
    later = [row for row in after if row["t"] >= first_stop["t"]]
    assert all(row["state"] == "STOP" for row in later)
    assert later[-1]["command"] == 0


# ---------------------------------------------------------------- stale_vision
@pytest.fixture(scope="module")
def stale():
    return run(Scenario("stale_vision", "corridor", duration_s=8, camera_stops_at_s=5.0))


def test_stale_vision_stops_within_limit(stale):
    last_capture = max(frame["t"] for frame in stale.frames)
    first_stop = next(row for row in pi_rows(stale) if row["t"] > 5.0 and row["state"] == "STOP")
    print(
        f"STOP {(first_stop['t'] - 5.0) * 1000:.0f} ms after frames stopped, "
        f"{(first_stop['t'] - last_capture) * 1000:.0f} ms after the last frame was captured "
        f"({first_stop['reason']})"
    )
    assert first_stop["reason"] == "waiting_for_fresh_vision"
    # Frames stop at 5.0 s: STOP must come within maximum_vision_age_s (0.8 s).
    assert first_stop["t"] - 5.0 <= CONTROL["maximum_vision_age_s"]
    # The rule is "last frame older than 0.8 s", checked every control tick.
    assert first_stop["t"] - last_capture <= CONTROL["maximum_vision_age_s"] + CONTROL_PERIOD_S
    assert pi_rows(stale)[-1]["command"] == 0


# ---------------------------------------------------------------- no_drive_flag
@pytest.fixture(scope="module")
def no_drive():
    return run(Scenario("no_drive_flag", "corridor", duration_s=6, motors_enabled=False))


def test_no_drive_flag_every_command_is_zero(no_drive):
    assert len(no_drive.fake.commands) > 100
    assert all(left == 0 and right == 0 for _, _, left, right in no_drive.fake.commands)
    assert abs(no_drive.rows[-1]["x"] - no_drive.rows[0]["x"]) < 0.001


def test_no_drive_flag_policy_would_have_moved(no_drive):
    # Proves the zeros come from the missing --drive flag, not from a STOP.
    assert any(row["state"] == "MOVE" for row in pi_rows(no_drive))


def test_unknown_values_never_become_zero_in_telemetry():
    fake = FakeEsp32(load_world("empty"))  # Nothing within 4 m: every echo times out.
    line = fake.advance(200)[-1].split(",")
    assert line[4:7] == ["nan", "nan", "nan"]
    assert math.isfinite(float(line[7]))


def test_empty_world_refuses_to_drive():
    # No echo anywhere = unknown = STOP, so an open space is NOT treated as clear.
    result = run(Scenario("empty_world", "empty", duration_s=4))
    assert all(row["state"] == "STOP" for row in pi_rows(result))
    assert {row["reason"] for row in pi_rows(result)[5:]} == {"range_sensor_unavailable"}
    assert abs(result.rows[-1]["x"]) < 0.001
