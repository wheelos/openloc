#!/usr/bin/env python3
"""Run a bounded KITTI LiDAR-only odometry diagnostic and render its evidence."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def pose(row, prefix):
    values = [float(row[f"{prefix}_{r}{c}"])
              for r in range(3) for c in range(4)]
    return values


def pose_errors(row):
    estimate = pose(row, "estimate")
    reference = pose(row, "reference")
    translation = math.sqrt(sum(
        (estimate[index * 4 + 3] - reference[index * 4 + 3]) ** 2
        for index in range(3)
    ))
    rotation = [[estimate[r * 4 + c] for c in range(3)] for r in range(3)]
    truth = [[reference[r * 4 + c] for c in range(3)] for r in range(3)]
    trace = sum(rotation[r][c] * truth[r][c] for r in range(3) for c in range(3))
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return translation, math.acos(cosine)


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize(rows):
    completed = [row for row in rows
                 if row["status"] in ("ANCHOR", "ICP_CONVERGED")]
    failed = next((row for row in rows if row["status"] == "ICP_REJECTED"), None)
    errors = [pose_errors(row) for row in completed]
    translation = [item[0] for item in errors]
    rotation = [item[1] for item in errors]
    path_estimate = 0.0
    path_reference = 0.0
    for previous, current in zip(completed, completed[1:]):
        for prefix, key in (("estimate", "estimate"), ("reference", "reference")):
            first, second = pose(previous, prefix), pose(current, prefix)
            distance = math.sqrt(sum(
                (second[index * 4 + 3] - first[index * 4 + 3]) ** 2
                for index in range(3)
            ))
            if key == "estimate":
                path_estimate += distance
            else:
                path_reference += distance
    return {
        "status": "failed_registration" if failed else "complete",
        "requested_scans": len(rows),
        "completed_scans": len(completed),
        "converged_icp_scans": sum(row["status"] == "ICP_CONVERGED" for row in completed),
        "first_failed_frame": int(failed["frame"]) if failed else None,
        "translation_error_m": {
            "rmse": math.sqrt(sum(value * value for value in translation) / len(translation))
            if translation else None,
            "median": statistics.median(translation) if translation else None,
            "p95": percentile(translation, 0.95),
            "max": max(translation) if translation else None,
        },
        "rotation_error_rad": {
            "median": statistics.median(rotation) if rotation else None,
            "p95": percentile(rotation, 0.95),
            "max": max(rotation) if rotation else None,
        },
        "estimated_path_length_m": path_estimate,
        "reference_path_length_m": path_reference,
        "path_length_ratio": path_estimate / path_reference if path_reference else None,
        "coverage": {
            "raw_imu": "unavailable",
            "gnss_observations": "unavailable",
            "loop_closure": "not_implemented",
            "independent_fusion_accuracy": "unavailable",
        },
    }, completed


def svg_trajectory(completed):
    width, height = 960, 720
    left, top, plot_width, plot_height = 86, 65, 820, 560
    points = {}
    for prefix in ("estimate", "reference"):
        points[prefix] = [
            (float(row[f"{prefix}_03"]), float(row[f"{prefix}_13"]))
            for row in completed
        ]
    all_points = points["estimate"] + points["reference"]
    if not all_points:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="960" height="720"/>'
    xs, ys = [point[0] for point in all_points], [point[1] for point in all_points]
    span_x, span_y = max(max(xs) - min(xs), 1.0), max(max(ys) - min(ys), 1.0)
    scale = min(plot_width / span_x, plot_height / span_y)
    center_x, center_y = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

    def screen(point):
        return (left + plot_width / 2 + (point[0] - center_x) * scale,
                top + plot_height / 2 - (point[1] - center_y) * scale)

    def polyline(values, color):
        coords = " ".join(f"{x:.2f},{y:.2f}" for x, y in map(screen, values))
        return f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2.5"/>'

    markup = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="86" y="34" font-family="sans-serif" font-size="20">'
        'KITTI sequence 00 — LiDAR-only scan-to-scan ICP</text>',
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" '
        'fill="none" stroke="#999"/>',
        polyline(points["reference"], "#207a3c"),
        polyline(points["estimate"], "#2458a6"),
        '<text x="86" y="670" font-family="sans-serif" font-size="15">'
        'Green: reference (fixed first-scan anchor)  Blue: chained ICP estimate  '
        'Equal XY scale</text>',
        f'<text x="20" y="360" transform="rotate(-90 20 360)" '
        'font-family="sans-serif" font-size="14">Y (m)</text>',
        '<text x="440" y="700" font-family="sans-serif" font-size="14">X (m)</text>',
        '</svg>',
    ]
    return "\n".join(markup)


def run(args):
    sequence = args.sequence.resolve(strict=True)
    binary = args.binary.resolve(strict=True)
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite output: {output}")
    if not (sequence / "velodyne").is_dir():
        raise ValueError("sequence must contain velodyne/")
    if args.first_frame < 0 or args.frame_count < 2:
        raise ValueError("require first-frame >= 0 and frame-count >= 2")
    output.mkdir(parents=True)

    stop = args.first_frame + args.frame_count
    inputs = [sequence / "calib.txt", sequence / "poses.txt", sequence / "times.txt"]
    inputs.extend(sequence / "velodyne" / f"{frame:06d}.bin"
                  for frame in range(args.first_frame, stop))
    missing = [str(path) for path in inputs if not path.is_file()]
    if missing:
        raise FileNotFoundError("missing dataset inputs: " + ", ".join(missing[:5]))
    source_hashes = {str(path): sha256(path) for path in inputs}
    manifest = {
        "schema": "openloc.kitti-lidar-odometry-diagnostic.v1",
        "experiment_type": "lidar_odometry_diagnostic",
        "sequence": str(sequence),
        "first_frame": args.first_frame,
        "frame_count": args.frame_count,
        "source_sha256": source_hashes,
        "algorithm": {
            "name": "PCL point-to-point ICP",
            "initial_guess": "identity for every consecutive scan pair",
            "voxel_leaf_m": 0.5,
            "maximum_correspondence_distance_m": 2.0,
            "maximum_iterations": 40,
            "reference_used_by_estimator": False,
        },
        "implementation_sha256": sha256(Path(__file__)),
        "binary_sha256": sha256(binary),
        "limitations": [
            "KITTI provides no raw IMU or GNSS observations in this package.",
            "The scans are not deskewed; per-point times are unavailable.",
            "This does not test OpenLoc native LIO, GNSS fusion, loop closure, or mapping.",
            "The reference is used only for fixed-first-frame scoring.",
            "Short-window results are not the official 100-800 m KITTI segment score.",
        ],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    result_path = output / "trajectory.csv"
    command = [
        str(binary), str(sequence), str(args.first_frame), str(args.frame_count),
        str(result_path),
    ]
    completed_run = subprocess.run(command, check=False, capture_output=True, text=True)
    (output / "run.log").write_text(
        "$ " + " ".join(command) + "\n" + completed_run.stdout + completed_run.stderr
    )
    if completed_run.returncode != 0:
        raise RuntimeError(f"odometry executable failed with {completed_run.returncode}")
    source_hashes_after = {str(path): sha256(path) for path in inputs}
    if source_hashes_after != source_hashes:
        raise ValueError("dataset source hashes changed during the diagnostic run")

    rows = read_rows(result_path)
    if len(rows) != args.frame_count:
        raise ValueError(f"expected {args.frame_count} output rows, got {len(rows)}")
    if any(row["status"] not in ("ANCHOR", "ICP_CONVERGED", "ICP_REJECTED")
           for row in rows):
        raise ValueError("odometry output contains an unknown status")
    summary, valid_rows = summarize(rows)
    summary.update({
        "schema": "openloc.kitti-lidar-odometry-report.v1",
        "experiment_type": "lidar_odometry_diagnostic",
        "trajectory_sha256": sha256(result_path),
        "manifest_sha256": sha256(output / "manifest.json"),
        "reference_alignment": "fixed first reference pose; no fitted alignment",
    })
    (output / "report.json").write_text(json.dumps(summary, indent=2) + "\n")
    visualization = output / "trajectory.svg"
    visualization.write_text(svg_trajectory(valid_rows))
    artifacts = ("manifest.json", "run.log", "trajectory.csv", "report.json",
                 "trajectory.svg")
    verification = {
        "schema": "openloc.kitti-lidar-odometry-verification.v1",
        "source_unchanged": True,
        "source_sha256_before": source_hashes,
        "source_sha256_after": source_hashes_after,
        "artifacts_sha256": {name: sha256(output / name) for name in artifacts},
        "implementation_sha256": manifest["implementation_sha256"],
        "binary_sha256": manifest["binary_sha256"],
    }
    (output / "verification.json").write_text(
        json.dumps(verification, indent=2) + "\n"
    )
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--first-frame", type=int, default=0)
    parser.add_argument("--frame-count", type=int, default=100)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
