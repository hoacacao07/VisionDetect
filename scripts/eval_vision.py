"""EVALUATE THE REAL DETECTOR + distance.py on images you recorded with the robot camera.

1. Record (on the Pi, camera mounted and calibrated, 640x360):
     rpicam-vid -t 20000 --width 640 --height 360 --framerate 10 -o clip.h264
   or still images:  rpicam-still --width 640 --height 360 -o img_0001.jpg
2. If you recorded a video, split it into frames:
     python scripts/eval_vision.py --video clip.h264 --every 10
   (writes results/vision/frames/ and a truth CSV to fill in)
3. Fill in the truth CSV (template: scripts/vision_ground_truth_template.csv):
     file, class, true_distance_m
   true_distance_m = floor distance from the point under the lens to where the
   object touches the floor, measured with a tape measure.
4. Run:
     python scripts/eval_vision.py --images results/vision/frames --truth my_truth.csv
   It runs ObjectDetector + distance.py at image_size 320 and 640, prints
   detected yes/no, estimated vs true distance and % error per object, flags
   anything worse than 10 % near the image centre at 0.5-3 m, and saves
   annotated images to results/vision/imgsz<size>/.
"""

import argparse
import copy
import csv
import math
import statistics
import sys
import time
from pathlib import Path
import cv2

PROJECT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(PROJECT / "src"), str(PROJECT)]
from vision_detection.robot_config import check_robot_camera_calibration  # noqa: E402
from vision_detection.robot_runtime import make_detector  # noqa: E402
from vision_detection.settings import load_settings  # noqa: E402

OUTPUT = PROJECT / "results" / "vision"
ERROR_LIMIT = 0.10
CENTRE_FRACTION = 1 / 3  # "Near the centre": middle third of the image width.


def extract_frames(video, every, directory=OUTPUT / "frames"):
    """Save every Nth video frame as frame_0001.jpg ... and a truth CSV to fill in."""
    directory.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video))
    saved, index = [], 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % every == 0:
            name = f"frame_{len(saved) + 1:04d}.jpg"
            cv2.imwrite(str(directory / name), frame)
            saved.append(name)
        index += 1
    capture.release()
    truth = directory / "truth_to_fill_in.csv"
    with truth.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["file", "class", "true_distance_m"])
        writer.writerows([name, "", ""] for name in saved)
    return saved, truth


def read_truth(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as file:
        rows = [row for row in csv.DictReader(file) if row.get("file") and not row["file"].startswith("#")]
    for row in rows:
        text = (row.get("true_distance_m") or "").strip()
        row["true_distance_m"] = float(text) if text else None
    return rows


def best_match(detections, label, width):
    """The detection of this class closest to the image centre, or None."""
    candidates = [d for d in detections if d["label"] == label]
    return min(
        candidates,
        key=lambda d: abs((d["bbox_xyxy"][0] + d["bbox_xyxy"][2]) / 2 - width / 2),
        default=None,
    )


def evaluate_size(settings, size, model, images, truth, motion_settings):
    """Run one image size; return result rows and the inference times."""
    settings = copy.deepcopy(settings)
    settings["detection"]["image_size"] = size
    if model:
        settings["detection"]["model"] = model
    detector = make_detector(settings)
    known = set(getattr(detector, "class_names", []))
    out_dir = OUTPUT / f"imgsz{size}"
    out_dir.mkdir(parents=True, exist_ok=True)
    results, times = [], []
    cache = {}
    for row in truth:
        label = row["class"].strip()
        if known and label not in known:
            raise SystemExit(f"{row['file']}: class '{label}' is not a detector class: {sorted(known)}")
        if row["file"] not in cache:
            image = cv2.imread(str(Path(images) / row["file"]))
            if image is None:
                raise SystemExit(f"Cannot read image {Path(images) / row['file']}")
            if image.shape[1] != 640 or image.shape[0] != 360:
                print(f"WARNING {row['file']}: {image.shape[1]}x{image.shape[0]}, expected 640x360")
            started = time.perf_counter()
            detections = detector.detect(image)
            times.append((time.perf_counter() - started) * 1000)
            cache[row["file"]] = (image, detections)
        image, detections = cache[row["file"]]
        height, width = image.shape[:2]
        match = best_match(detections, label, width)
        estimate = match["distance_m"] if match else None
        true = row["true_distance_m"]
        error = abs(estimate - true) / true if estimate is not None and true else None
        centre_x = (match["bbox_xyxy"][0] + match["bbox_xyxy"][2]) / 2 if match else width / 2
        near_centre = abs(centre_x - width / 2) <= width * CENTRE_FRACTION / 2
        in_band = true is not None and 0.5 <= true <= 3.0
        flag = ""
        if match is None:
            flag = "NOT DETECTED"
        elif estimate is None:
            flag = "NO DISTANCE"
        elif error is not None and near_centre and in_band and error > ERROR_LIMIT:
            flag = f"ERROR > {ERROR_LIMIT:.0%}"
        results.append(dict(
            file=row["file"], label=label, detected=match is not None,
            confidence=match["confidence"] if match else None,
            estimate_m=estimate, true_m=true, error=error, near_centre=near_centre, flag=flag,
        ))
        annotated = image.copy()
        for detection in detections:
            x0, y0, x1, y1 = map(int, detection["bbox_xyxy"])
            color = (0, 200, 0) if detection is match else (160, 160, 160)
            cv2.rectangle(annotated, (x0, y0), (x1, y1), color, 2)
            text = f"{detection['label']} {detection['confidence']:.2f} " + (
                f"{detection['distance_m']:.2f}m" if detection["distance_m"] is not None else "range ?"
            )
            cv2.putText(annotated, text, (x0, max(14, y0 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        truth_text = f"TRUE {label} {true:.2f} m" if true else f"TRUE {label} (no distance)"
        cv2.putText(annotated, truth_text, (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imwrite(str(out_dir / Path(row["file"]).name), annotated)
    with (OUTPUT / f"eval-imgsz{size}.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    return results, times


def print_table(size, results, times):
    print(f"\n=== image_size {size} ===")
    print(f"{'file':<32}{'class':<12}{'found':<7}{'estimate':>9}{'true':>7}{'error':>8}  flag")
    for r in results:
        est = f"{r['estimate_m']:.2f}" if r["estimate_m"] is not None else "-"
        true = f"{r['true_m']:.2f}" if r["true_m"] is not None else "-"
        err = f"{r['error'] * 100:.1f}%" if r["error"] is not None else "-"
        print(f"{r['file']:<32}{r['label']:<12}{'yes' if r['detected'] else 'NO':<7}{est:>9}{true:>7}{err:>8}  {r['flag']}")
    found = sum(r["detected"] for r in results)
    errors = [r["error"] for r in results if r["error"] is not None]
    summary = f"detected {found}/{len(results)}"
    if errors:
        summary += f", distance error median {statistics.median(errors) * 100:.1f}% max {max(errors) * 100:.1f}%"
    if times:
        summary += f", inference mean {statistics.mean(times):.0f} ms"
    flagged = sum(bool(r["flag"]) for r in results)
    print(summary + f", flagged {flagged}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", help="Split this video into frames first, then stop")
    parser.add_argument("--every", type=int, default=10, help="Keep every Nth video frame")
    parser.add_argument("--images", help="Folder with the images named in the truth CSV")
    parser.add_argument("--truth", help="CSV: file,class,true_distance_m")
    parser.add_argument("--config", default=str(PROJECT / "config/cameras/robot.json"))
    parser.add_argument("--sizes", type=int, nargs="+", default=[320, 640])
    parser.add_argument("--model", help="Model for every size (default: detection.model in --config). "
                        "On a laptop use models/outdoor_detector_small.pt.")
    arguments = parser.parse_args()
    if arguments.video:
        saved, truth = extract_frames(arguments.video, arguments.every)
        print(f"Saved {len(saved)} frames to {truth.parent}. Fill in {truth}, then run with --images/--truth.")
        return
    if not (arguments.images and arguments.truth):
        parser.error("Give --images and --truth (or --video to split a recording)")
    settings = load_settings(arguments.config)
    try:
        check_robot_camera_calibration(settings["distance"])
    except ValueError as error:
        print(f"WARNING: distances will be missing or wrong until calibrated.\n{error}")
    truth = read_truth(arguments.truth)
    for size in arguments.sizes:
        results, times = evaluate_size(settings, size, arguments.model, arguments.images, truth, settings["motion"])
        print_table(size, results, times)
    print(f"\nAnnotated images and CSVs: {OUTPUT}")


if __name__ == "__main__":
    main()
