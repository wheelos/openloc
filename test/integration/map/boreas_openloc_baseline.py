#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0

"""Prepare and score the diagnostic Boreas scan-to-map baseline."""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np


MAP_SEQUENCE = "boreas-2020-11-26-13-58"
QUERY_SEQUENCE = "boreas-2021-01-15-12-17"
MAP_INDEX_RANGE = range(5434, 5534, 2)
QUERY_INDEX_RANGE = range(5306, 5405, 2)
MISSING_QUERY_TIMESTAMPS_US = {"1610731622464417"}
BIAS_TRANSLATION_M = (1.0, 0.0, 0.0)
BIAS_YAW_RAD = math.radians(2.0)
MAP_VOXEL_SIZE_M = 0.5
POSE_COLUMNS = (
    "GPSTime",
    "easting",
    "northing",
    "altitude",
    "roll",
    "pitch",
    "heading",
)


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_poses(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if not set(POSE_COLUMNS).issubset(reader.fieldnames or []):
            raise ValueError(f"unexpected Boreas pose columns: {path}")
        for row in reader:
            rows.append(
                (
                    int(row["GPSTime"]),
                    float(row["easting"]),
                    float(row["northing"]),
                    float(row["altitude"]),
                    float(row["roll"]),
                    float(row["pitch"]),
                    float(row["heading"]),
                )
            )
    if len(rows) < 2:
        raise ValueError(f"pose table has fewer than two rows: {path}")
    poses = np.asarray(rows, dtype=np.float64)
    if not np.isfinite(poses).all() or not np.all(np.diff(poses[:, 0]) > 0):
        raise ValueError(f"pose table contains invalid or unordered values: {path}")
    return poses


def quaternion_from_rpy(roll, pitch, yaw):
    """Return xyzw quaternion for Boreas' Rx(roll) Ry(pitch) Rz(yaw) order."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr, sr = np.cos(half_roll), np.sin(half_roll)
    cp, sp = np.cos(half_pitch), np.sin(half_pitch)
    cy, sy = np.cos(half_yaw), np.sin(half_yaw)
    return np.stack(
        (
            sr * cp * cy + cr * sp * sy,
            cr * sp * cy - sr * cp * sy,
            cr * cp * sy + sr * sp * cy,
            cr * cp * cy - sr * sp * sy,
        ),
        axis=-1,
    )


def quaternion_slerp(first, second, fraction):
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    fraction = np.asarray(fraction, dtype=np.float64)
    dot = np.sum(first * second, axis=-1)
    second = np.where((dot < 0.0)[..., None], -second, second)
    dot = np.abs(dot)
    near = dot > 0.9995
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    sine = np.sin(angle)
    safe_sine = np.where(near, 1.0, sine)
    weight_first = np.sin((1.0 - fraction) * angle) / safe_sine
    weight_second = np.sin(fraction * angle) / safe_sine
    spherical = (
        weight_first[..., None] * first + weight_second[..., None] * second
    )
    linear = (1.0 - fraction)[..., None] * first + fraction[..., None] * second
    result = np.where(near[..., None], linear, spherical)
    return result / np.linalg.norm(result, axis=-1, keepdims=True)


def rotate_points(quaternions, points):
    vector = quaternions[..., :3]
    twice_cross = 2.0 * np.cross(vector, points)
    return (
        points
        + quaternions[..., 3, None] * twice_cross
        + np.cross(vector, twice_cross)
    )


def pose_at(poses, timestamps_us):
    timestamps_us = np.asarray(timestamps_us, dtype=np.float64)
    pose_times = poses[:, 0]
    if (
        np.any(timestamps_us < pose_times[0])
        or np.any(timestamps_us > pose_times[-1])
    ):
        raise ValueError("a LiDAR point timestamp lies outside the pose table")
    lower = np.searchsorted(pose_times, timestamps_us, side="right") - 1
    lower = np.clip(lower, 0, len(poses) - 2)
    upper = lower + 1
    fraction = (timestamps_us - pose_times[lower]) / (
        pose_times[upper] - pose_times[lower]
    )
    translation = (
        (1.0 - fraction)[..., None] * poses[lower, 1:4]
        + fraction[..., None] * poses[upper, 1:4]
    )
    q_first = quaternion_from_rpy(
        poses[lower, 4], poses[lower, 5], poses[lower, 6]
    )
    q_second = quaternion_from_rpy(
        poses[upper, 4], poses[upper, 5], poses[upper, 6]
    )
    return translation, quaternion_slerp(q_first, q_second, fraction)


def load_extrinsic(path):
    transform = np.loadtxt(path, dtype=np.float64)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"invalid T_applanix_lidar matrix: {path}")
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ValueError(f"non-rigid homogeneous row in extrinsic: {path}")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6):
        raise ValueError(f"extrinsic rotation is not orthonormal: {path}")
    return rotation, transform[:3, 3]


def selected_scans(data_root, sequence, indices):
    sequence_root = Path(data_root) / sequence
    pose_path = sequence_root / "applanix" / "lidar_poses.csv"
    poses = read_poses(pose_path)
    selected = []
    for index in indices:
        if index >= len(poses):
            raise ValueError(f"pose index {index} is outside {pose_path}")
        timestamp = str(int(poses[index, 0]))
        selected.append((timestamp, poses[index]))
    expected = {timestamp for timestamp, _ in selected}
    available = {
        path.stem for path in (sequence_root / "lidar").glob("*.bin")
    }
    missing = expected - available
    unexpected = available - expected
    allowed_missing = (
        MISSING_QUERY_TIMESTAMPS_US if sequence == QUERY_SEQUENCE else set()
    )
    if missing != allowed_missing or unexpected:
        raise ValueError(
            f"unexpected scan inventory for {sequence}: "
            f"missing={sorted(missing)}, unexpected={sorted(unexpected)}"
        )
    return sequence_root, poses, selected


def read_scan(path):
    data = Path(path).read_bytes()
    if len(data) % 24 != 0:
        raise ValueError(f"Boreas scan does not contain 6 float32 fields: {path}")
    points = np.frombuffer(data, dtype="<f4").reshape((-1, 6))
    if points.size == 0 or not np.isfinite(points).all():
        raise ValueError(f"empty or non-finite Boreas scan: {path}")
    return data, points


def transform_scan(
    points,
    timestamp_us,
    poses,
    rotation_body_lidar,
    translation_body_lidar,
    center_pose=None,
):
    point_times_us = timestamp_us + points[:, 5].astype(np.float64) * 1e6
    world_translation, world_rotation = pose_at(poses, point_times_us)
    body_points = (
        points[:, :3].astype(np.float64) @ rotation_body_lidar.T
        + translation_body_lidar
    )
    world_points = (
        rotate_points(world_rotation, body_points) + world_translation
    )
    if center_pose is None:
        return world_points
    center_translation, center_rotation = center_pose
    inverse_center_rotation = np.array(
        (
            -center_rotation[0],
            -center_rotation[1],
            -center_rotation[2],
            center_rotation[3],
        )
    )
    return rotate_points(
        np.broadcast_to(inverse_center_rotation, (len(world_points), 4)),
        world_points - center_translation,
    )


def write_pcd(path, points):
    points = np.asarray(points, dtype="<f4")
    if points.ndim != 2 or points.shape[1] != 3 or not len(points):
        raise ValueError(f"invalid point cloud for PCD output: {path}")
    with Path(path).open("wb") as output:
        output.write(
            (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION .7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                f"WIDTH {len(points)}\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                f"POINTS {len(points)}\n"
                "DATA binary\n"
            ).encode("ascii")
        )
        points.tofile(output)


def voxel_centroids(points, voxel_size_m):
    points = np.asarray(points, dtype=np.float64)
    if (
        points.ndim != 2
        or points.shape[1] != 3
        or not len(points)
        or not np.isfinite(points).all()
        or not math.isfinite(voxel_size_m)
        or voxel_size_m <= 0.0
    ):
        raise ValueError("voxel input must be finite nonempty XYZ and positive")
    voxel_indices = np.floor(points / voxel_size_m).astype(np.int64)
    _, inverse = np.unique(voxel_indices, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centroids = np.column_stack(
        [
            np.bincount(inverse, weights=points[:, axis]) / counts
            for axis in range(3)
        ]
    )
    return centroids


def pose_quaternion(row):
    return quaternion_from_rpy(row[4], row[5], row[6])


def apply_map_bias(translation, quaternion):
    half_yaw = BIAS_YAW_RAD * 0.5
    bias_q = np.array((0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)))
    biased_q = np.array(
        (
            bias_q[3] * quaternion[0] + bias_q[0] * quaternion[3]
            + bias_q[1] * quaternion[2] - bias_q[2] * quaternion[1],
            bias_q[3] * quaternion[1] - bias_q[0] * quaternion[2]
            + bias_q[1] * quaternion[3] + bias_q[2] * quaternion[0],
            bias_q[3] * quaternion[2] + bias_q[0] * quaternion[1]
            - bias_q[1] * quaternion[0] + bias_q[2] * quaternion[3],
            bias_q[3] * quaternion[3] - bias_q[0] * quaternion[0]
            - bias_q[1] * quaternion[1] - bias_q[2] * quaternion[2],
        ),
        dtype=np.float64,
    )
    return (
        np.asarray(translation, dtype=np.float64)
        + np.asarray(BIAS_TRANSLATION_M),
        biased_q,
    )


def write_csv(path, header, rows):
    with Path(path).open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(header)
        writer.writerows(rows)


def prepare(args):
    data_root = Path(args.data_root).resolve(strict=True)
    output_dir = Path(args.output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    map_root, map_poses, map_samples = selected_scans(
        data_root, MAP_SEQUENCE, MAP_INDEX_RANGE
    )
    query_root, query_poses, query_samples = selected_scans(
        data_root, QUERY_SEQUENCE, QUERY_INDEX_RANGE
    )
    map_rotation, map_translation = load_extrinsic(
        map_root / "calib" / "T_applanix_lidar.txt"
    )
    query_rotation, query_translation = load_extrinsic(
        query_root / "calib" / "T_applanix_lidar.txt"
    )

    map_origin = map_samples[0][1][1:4].copy()
    map_blocks = []
    source_hashes = {}
    for timestamp, _ in map_samples:
        path = map_root / "lidar" / f"{timestamp}.bin"
        data, points = read_scan(path)
        source_hashes[str(path.relative_to(data_root))] = sha256_bytes(data)
        world_points = transform_scan(
            points,
            int(timestamp),
            map_poses,
            map_rotation,
            map_translation,
        )
        map_blocks.append(world_points - map_origin)
    map_cloud = np.concatenate(map_blocks, axis=0)
    filtered_map_cloud = voxel_centroids(map_cloud, MAP_VOXEL_SIZE_M)
    raw_map_pcd = output_dir / "map_raw.pcd"
    map_pcd = output_dir / "map.pcd"
    write_pcd(raw_map_pcd, map_cloud)
    write_pcd(map_pcd, filtered_map_cloud)

    scan_dir = output_dir / "queries"
    scan_dir.mkdir()
    request_rows = []
    reference_rows = []
    for timestamp, row in query_samples:
        scan_path = query_root / "lidar" / f"{timestamp}.bin"
        if not scan_path.exists():
            continue
        data, points = read_scan(scan_path)
        source_hashes[str(scan_path.relative_to(data_root))] = sha256_bytes(data)
        center_translation = row[1:4] - map_origin
        center_rotation = pose_quaternion(row)
        deskewed = transform_scan(
            points,
            int(timestamp),
            query_poses,
            query_rotation,
            query_translation,
            (row[1:4], center_rotation),
        )
        pcd_path = scan_dir / f"{timestamp}.pcd"
        write_pcd(pcd_path, deskewed)
        biased_translation, biased_quaternion = apply_map_bias(
            center_translation, center_rotation
        )
        timestamp_ns = int(timestamp) * 1000
        request_rows.append(
            (
                timestamp_ns,
                str(pcd_path.resolve()),
                *biased_translation,
                *biased_quaternion,
            )
        )
        reference_rows.append(
            (timestamp_ns, *center_translation, *center_rotation)
        )

    write_csv(
        output_dir / "requests.csv",
        ("timestamp_ns", "scan_path", "x", "y", "z", "qx", "qy", "qz", "qw"),
        request_rows,
    )
    write_csv(
        output_dir / "reference.csv",
        ("timestamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"),
        reference_rows,
    )
    if len(request_rows) != 49:
        raise ValueError(f"expected 49 query scans, prepared {len(request_rows)}")

    source_inputs = {}
    for sequence_root in (map_root, query_root):
        for relative in (
            "applanix/lidar_poses.csv",
            "calib/T_applanix_lidar.txt",
            "route.html",
        ):
            path = sequence_root / relative
            source_inputs[str(path.relative_to(data_root))] = sha256_file(path)
    generated_files = [
        raw_map_pcd,
        output_dir / "map.pcd",
        output_dir / "requests.csv",
        output_dir / "reference.csv",
        *sorted(scan_dir.glob("*.pcd")),
    ]
    manifest = {
        "schema": "openloc.boreas.baseline.v1",
        "evidence_scope": "diagnostic_local_scan_to_map_component_only",
        "dataset": {
            "name": "Boreas",
            "source": "https://boreas.utias.utoronto.ca/",
            "license": "CC BY 4.0",
            "map_sequence": MAP_SEQUENCE,
            "query_sequence": QUERY_SEQUENCE,
            "map_pose_indices": [MAP_INDEX_RANGE.start, MAP_INDEX_RANGE.stop, 2],
            "query_pose_indices": [
                QUERY_INDEX_RANGE.start,
                QUERY_INDEX_RANGE.stop,
                2,
            ],
            "map_scan_count": len(map_samples),
            "query_expected_scan_count": len(query_samples),
            "query_prepared_scan_count": len(request_rows),
            "missing_query_timestamps_us": sorted(
                MISSING_QUERY_TIMESTAMPS_US
            ),
            "pose_timestamp_unit": "microseconds",
            "point_time_unit": "seconds_relative_to_scan_center",
        },
        "frames": {
            "map_frame": "local_enu",
            "base_frame": "Applanix",
            "cloud_frame": "Applanix",
            "extrinsic": "T_applanix_lidar",
            "map_origin_enu_m": map_origin.tolist(),
            "pose_rotation_order": "Rx(roll) * Ry(pitch) * Rz(heading)",
            "map_construction": (
                "per-point pose interpolation; LiDAR points transformed to "
                "local ENU and map-origin translation subtracted"
            ),
            "query_deskew": (
                "per-point pose interpolation into Applanix body at scan center"
            ),
        },
        "initial_condition": {
            "source": "Boreas post-processed query reference trajectory",
            "map_frame_translation_bias_m": list(BIAS_TRANSLATION_M),
            "map_frame_yaw_bias_rad": BIAS_YAW_RAD,
            "limitation": (
                "reference-derived seed and motion are not independent ODOM; "
                "this tests controlled local convergence only"
            ),
        },
        "map": {
            "path": "map.pcd",
            "sha256": sha256_file(map_pcd),
            "point_count": int(len(filtered_map_cloud)),
            "voxel_filter": "XYZ centroid by floor(point / leaf_size)",
            "voxel_size_m": MAP_VOXEL_SIZE_M,
            "raw_path": "map_raw.pcd",
            "raw_sha256": sha256_file(raw_map_pcd),
            "raw_point_count": int(len(map_cloud)),
            "map_id": MAP_SEQUENCE,
            "version": "boreas-pilot-v1",
            "content_id": sha256_file(map_pcd),
        },
        "source_hashes": {**source_inputs, **source_hashes},
        "generated_hashes": {
            str(path.relative_to(output_dir)): sha256_file(path)
            for path in generated_files
        },
        "preparation_script_sha256": sha256_file(Path(__file__)),
        "preparation_environment": {
            "python": sys.version,
            "numpy": np.__version__,
            "platform": platform.platform(),
        },
    }
    with (output_dir / "manifest.json").open("w", encoding="utf-8") as output:
        json.dump(manifest, output, indent=2, sort_keys=True)
        output.write("\n")


def normalize_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("zero quaternion in trajectory output")
    return quaternion / norm


def pose_errors(estimate, reference):
    translation = np.linalg.norm(
        np.asarray(estimate[:3], dtype=np.float64)
        - np.asarray(reference[:3], dtype=np.float64)
    )
    q_estimate = normalize_quaternion(estimate[3:7])
    q_reference = normalize_quaternion(reference[3:7])
    dot = abs(float(np.dot(q_estimate, q_reference)))
    rotation = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    return translation, rotation


def percentile_summary(values):
    if not values:
        return None
    return {
        "count": len(values),
        "median": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(max(values)),
    }


def run(args):
    prepared_dir = Path(args.prepared_dir).resolve(strict=True)
    output_dir = Path(args.output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"output directory is not empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    with (prepared_dir / "manifest.json").open(encoding="utf-8") as source:
        manifest = json.load(source)
    if manifest.get("schema") != "openloc.boreas.baseline.v1":
        raise ValueError("unsupported Boreas baseline manifest")
    for relative, expected_hash in manifest["generated_hashes"].items():
        if sha256_file(prepared_dir / relative) != expected_hash:
            raise ValueError(f"prepared input hash mismatch: {relative}")
    runner = Path(args.runner).resolve(strict=True)
    map_info = manifest["map"]
    results_path = output_dir / "results.csv"
    command = [
        str(runner),
        str(prepared_dir / map_info["path"]),
        str(prepared_dir / "requests.csv"),
        str(results_path),
        map_info["map_id"],
        map_info["version"],
        map_info["content_id"],
        "--matcher",
        args.matcher,
        "--map-voxel-m",
        str(args.map_voxel_m),
    ]
    if args.matcher == "point-to-plane-icp":
        command.extend(
            (
                "--icp-max-points",
                str(args.icp_max_points),
                "--icp-iterations",
                str(args.icp_iterations),
                "--profile-icp-timing",
                "true",
            )
        )
    subprocess.run(command, check=True)

    with (prepared_dir / "reference.csv").open(
        newline="", encoding="utf-8"
    ) as source:
        references = {
            int(row["timestamp_ns"]): row for row in csv.DictReader(source)
        }
    with (prepared_dir / "requests.csv").open(
        newline="", encoding="utf-8"
    ) as source:
        requests = {
            int(row["timestamp_ns"]): row for row in csv.DictReader(source)
        }
    if set(requests) != set(references):
        raise ValueError("prepared requests and references have different timestamps")
    with results_path.open(newline="", encoding="utf-8") as source:
        results = list(csv.DictReader(source))
    if len(results) != manifest["dataset"]["query_prepared_scan_count"]:
        raise ValueError("matcher output row count differs from prepared requests")
    if len({int(row["timestamp_ns"]) for row in results}) != len(results):
        raise ValueError("matcher output contains duplicate timestamps")
    if {int(row["timestamp_ns"]) for row in results} != set(references):
        raise ValueError("matcher output timestamps differ from reference set")

    evaluation_rows = []
    statuses = {}
    prior_translation_errors = []
    prior_rotation_errors = []
    matched_translation_errors = []
    matched_rotation_errors = []
    for result in results:
        timestamp = int(result["timestamp_ns"])
        reference = references[timestamp]
        ref_pose = [
            float(reference[name])
            for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
        ]
        request = requests.get(timestamp)
        if request is None:
            raise ValueError(f"no request found for output timestamp {timestamp}")
        initial_pose = [
            float(request[name])
            for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
        ]
        prior_translation, prior_rotation = pose_errors(initial_pose, ref_pose)
        prior_translation_errors.append(prior_translation)
        prior_rotation_errors.append(prior_rotation)
        valid = result["valid"] == "1"
        if valid:
            matched_pose = [
                float(result[name])
                for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
            ]
            matched_translation, matched_rotation = pose_errors(
                matched_pose, ref_pose
            )
            matched_translation_errors.append(matched_translation)
            matched_rotation_errors.append(matched_rotation)
        else:
            matched_translation = ""
            matched_rotation = ""
        statuses[result["status"]] = statuses.get(result["status"], 0) + 1
        evaluation_rows.append(
            (
                timestamp,
                result["status"],
                result["reason"],
                result["valid"],
                prior_translation,
                prior_rotation,
                matched_translation,
                matched_rotation,
                result["covariance_calibrated"],
                result["calibration_state"],
            )
        )
    write_csv(
        output_dir / "evaluation.csv",
        (
            "timestamp_ns",
            "status",
            "reason",
            "valid",
            "prior_translation_error_m",
            "prior_rotation_error_rad",
            "matched_translation_error_m",
            "matched_rotation_error_rad",
            "covariance_calibrated",
            "calibration_state",
        ),
        evaluation_rows,
    )
    summary = {
        "schema": "openloc.boreas.baseline.results.v1",
        "evidence_scope": manifest["evidence_scope"],
        "processed_scans": len(results),
        "status_counts": statuses,
        "matcher_configuration": {
            "algorithm": args.matcher,
            "map_voxel_size_m": args.map_voxel_m,
            "icp_max_points": (
                args.icp_max_points
                if args.matcher == "point-to-plane-icp"
                else None
            ),
            "icp_iterations": (
                args.icp_iterations
                if args.matcher == "point-to-plane-icp"
                else None
            ),
            "icp_timing_profiled": args.matcher == "point-to-plane-icp",
        },
        "initial_error": {
            "translation_m": percentile_summary(prior_translation_errors),
            "rotation_rad": percentile_summary(prior_rotation_errors),
        },
        "valid_match_error_to_reference": {
            "translation_m": percentile_summary(matched_translation_errors),
            "rotation_rad": percentile_summary(matched_rotation_errors),
        },
        "accuracy_threshold_applied": False,
        "interpretation": (
            "Descriptive local scan-to-map component baseline only. The "
            "post-processed reference seeds each initial pose and is also the "
            "scoring reference; this is not independent ODOM, relocalization, "
            "calibrated uncertainty, or production evidence."
        ),
        "command": command,
        "runner_sha256": sha256_file(runner),
        "preparation_script_sha256": manifest["preparation_script_sha256"],
        "prepared_manifest_sha256": sha256_file(prepared_dir / "manifest.json"),
        "results_sha256": sha256_file(results_path),
        "evaluation_sha256": sha256_file(output_dir / "evaluation.csv"),
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "cwd": os.getcwd(),
        },
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as output:
        json.dump(summary, output, indent=2, sort_keys=True)
        output.write("\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    prepare_parser = subparsers.add_parser("prepare")
    prepare_parser.add_argument("--data-root", required=True)
    prepare_parser.add_argument("--output-dir", required=True)
    prepare_parser.set_defaults(function=prepare)
    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("--prepared-dir", required=True)
    run_parser.add_argument("--runner", required=True)
    run_parser.add_argument("--output-dir", required=True)
    run_parser.add_argument(
        "--matcher",
        choices=("ndt", "point-to-plane-icp"),
        default="ndt",
    )
    run_parser.add_argument(
        "--map-voxel-m", type=float, default=MAP_VOXEL_SIZE_M
    )
    run_parser.add_argument("--icp-max-points", type=int, default=12000)
    run_parser.add_argument("--icp-iterations", type=int, default=40)
    run_parser.set_defaults(function=run)
    args = parser.parse_args()
    args.function(args)


if __name__ == "__main__":
    main()
