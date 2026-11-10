"""Score physical response and fresh observations from saved Webots logs.

Scene-specific checks are explicit; they do not prove general navigation safety.
Predicted/coasted boxes never count toward recognition. Ground truth remains
in evaluation_only; this offline tool cannot control a robot or alter its log.
"""

from pathlib import Path
from datetime import datetime
import json

PROJECT = Path(__file__).resolve().parents[1]


def analyze(folder):
    """Score one completed scenario log using explicit timing and physical-response checks."""
    path = folder / "latest_run.jsonl"
    records = [
        json.loads(line) for line in path.read_text().splitlines() if line.strip()
    ]
    if not records:
        raise ValueError(f"No recorded observations in {path}")

    def choose(start, end):
        return [
            r for r in records if start <= r["prediction"]["simulation_time_s"] < end
        ]

    def speed(record):
        return abs(record["evaluation_only"]["true_forward_speed_mps"])

    def stopped_near(start, end):
        return any(
            speed(r) < 0.02
            and r["evaluation_only"]["truth_front_gap_m"] is not None
            and r["evaluation_only"]["truth_front_gap_m"] < 2
            for r in choose(start, end)
        )

    def resumed(start, end):
        return any(
            speed(r) > 0.1 and r["prediction"]["navigation"]["target_speed_mps"] > 0
            for r in choose(start, end)
        )

    final = records[-1]["prediction"]
    if final.get("scenario_version") != "continuous_v3_65s":
        raise ValueError(
            "Rerun the continuous_v3_65s scenario before scoring its timing windows."
        )
    gaps = [
        r["evaluation_only"]["truth_front_gap_m"]
        for r in records
        if r["evaluation_only"]["truth_front_gap_m"] is not None
    ]
    minimum_gap = min(gaps) if gaps else None
    contacts = sum(
        r["evaluation_only"].get("above_ground_contact", False) for r in records
    )
    cover_stop = next(
        (
            r["prediction"]["simulation_time_s"] - 55
            for r in choose(55, 60)
            if speed(r) < 0.02
        ),
        None,
    )
    requirement = final["realism_assumptions"]["stop_clearance_requirement_m"]
    checks = dict(
        completed=final["simulation_time_s"] >= 65,
        minimum_clearance_passed=minimum_gap is not None and minimum_gap >= requirement,
        no_non_ground_contacts=contacts == 0,
        physically_stopped_near_person=stopped_near(10, 22),
        resumed_after_person=resumed(24, 32),
        physically_stopped_near_box=stopped_near(32, 50),
        resumed_after_box=resumed(52, 55),
        stopped_during_camera_cover=cover_stop is not None,
        resumed_after_uncover=resumed(61, 63.5),
        stationary_at_completion=speed(records[-1]) < 0.02,
    )
    unique = {}
    for record in records:
        capture = record["prediction"].get("perception_capture_time_s")
        if capture is not None and capture not in unique:
            unique[capture] = record
    windows = {}
    for name, start, end, labels in (
        ("person_waiting", 10.5, 21.5, ("person", "tree", "car")),
        ("stationary_box", 36, 46, ("box", "tree", "car")),
    ):
        samples = [r for t, r in unique.items() if start <= t < end]
        presence = [
            set(
                x["label"]
                for x in r["prediction"]["detections"]
                if x.get("observation_status") != "prediction_only"
            )
            for r in samples
        ]
        windows[name] = dict(
            images=len(samples),
            expected_visible_categories=list(labels),
            observed_frames={
                label: sum(label in items for items in presence) for label in labels
            },
            all_expected_observed_frames=sum(
                set(labels) <= items for items in presence
            ),
        )
    elapsed = (
        datetime.fromisoformat(final["timestamp_utc"])
        - datetime.fromisoformat(records[0]["prediction"]["timestamp_utc"])
    ).total_seconds()
    return dict(
        profile=final["realism_assumptions"]["profile"],
        scenario_version=final.get("scenario_version"),
        model=final["model"],
        checks=checks,
        all_response_checks_passed=all(checks.values()),
        minimum_reference_gap_m=minimum_gap,
        non_ground_contact_frames=contacts,
        covered_camera_physical_stop_latency_s=cover_stop,
        completed_inferences=final["completed_inferences"],
        control_steps=len(records),
        elapsed_wall_s=elapsed,
        mean_inference_updates_per_wall_s=final["completed_inferences"] / elapsed,
        recognition_windows=windows,
        limitations="Scene-specific, tuned diagnostic. Approximate reference footprints; stylized assets. Not real-world recall or safety certification.",
    )


if __name__ == "__main__":
    results = {}
    for key, folder in (
        ("measured_laptop", PROJECT / "simulation/moving_results"),
        ("stress", PROJECT / "simulation/moving_results/stress"),
    ):
        if (folder / "latest_run.jsonl").is_file():
            results[key] = analyze(folder)
    output = PROJECT / "simulation/validation_results.json"
    output.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))
