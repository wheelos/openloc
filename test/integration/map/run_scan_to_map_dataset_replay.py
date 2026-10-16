#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0

"""Verify the selected map artifacts and run a diagnostic keyframe replay."""

import argparse
import bisect
import csv
from decimal import Decimal
import hashlib
import json
import math
from pathlib import Path
import platform
import statistics
import subprocess
import sys


MAP_VERSION = "66428fe8bf1ef4dbbf07-georef-v2"
MAP_CONTENT_ID = "bf0de95c1702ff87bb5504de137e3395e76e0aebef077a964056d04b8b20b834"
GLOBAL_PCD_SHA256 = "f8d93d8bdfe1691f004c356d7ea086fc61d7dc636f4c069ac51f0ccaf5deefd7"
ODOM_WORLD_FRAME = "camera_init"
CLOUD_FRAME = "body"
GNSS_MAX_BRACKET_GAP_S = 1.1


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path):
    with Path(path).open(encoding="utf-8") as source:
        return json.load(source)


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as source:
        return list(csv.DictReader(source))


def timestamp_ns(text):
    nanoseconds = Decimal(text) * Decimal(1_000_000_000)
    if nanoseconds != nanoseconds.to_integral_value():
        raise ValueError(f"timestamp is not an integer nanosecond value: {text}")
    return int(nanoseconds)


def assert_under_root(root, relative_path):
    resolved_root = Path(root).resolve(strict=True)
    resolved = (resolved_root / relative_path).resolve(strict=True)
    if not resolved.is_relative_to(resolved_root):
        raise ValueError(f"artifact path escapes map root: {relative_path}")
    return resolved


def verify_map_artifact(map_root):
    root = Path(map_root).resolve(strict=True)
    metadata_path = root / "metadata.json"
    metadata = read_json(metadata_path)
    if metadata.get("schema") != "wheelos.map.v1":
        raise ValueError("unsupported map metadata schema")
    if metadata.get("version") != MAP_VERSION:
        raise ValueError("map version does not match the selected artifact")
    if metadata.get("content_id") != MAP_CONTENT_ID:
        raise ValueError("map content ID does not match the selected artifact")
    if metadata.get("global_cloud", {}).get("sha256") != GLOBAL_PCD_SHA256:
        raise ValueError("global PCD digest in metadata does not match the pin")

    verified = 0
    for relative_path, expected_digest in metadata.get("files", {}).items():
        path = assert_under_root(root, relative_path)
        if sha256(path) != expected_digest:
            raise ValueError(f"map artifact digest mismatch: {relative_path}")
        verified += 1
    if verified == 0:
        raise ValueError("map metadata contains no file integrity inventory")

    frames_path = root / "frames.csv"
    if sha256(frames_path) != metadata.get("frames_sha256"):
        raise ValueError("map frames.csv digest does not match metadata")
    global_pcd = assert_under_root(root, metadata["global_cloud"]["path"])
    if sha256(global_pcd) != GLOBAL_PCD_SHA256:
        raise ValueError("global PCD digest does not match the pinned artifact")

    provenance = read_json(root / "dataset_provenance.json")
    if (
        provenance.get("world_frame") != ODOM_WORLD_FRAME
        or provenance.get("cloud_frame") != CLOUD_FRAME
        or provenance.get("pose_convention") != "T_world_cloud"
        or provenance.get("deskew_owner") != "frontend"
    ):
        raise ValueError("map input frame/deskew provenance is incompatible")
    if metadata.get("cloud_frame") != CLOUD_FRAME:
        raise ValueError("map cloud frame is not the declared body frame")
    reference = metadata.get("coordinate_reference", {})
    if (
        reference.get("T_enu_map_status") != "unavailable"
        or metadata.get("qualification", {}).get("status") != "draft"
    ):
        raise ValueError("map qualification/reference state differs from scope")

    return {
        "root": root,
        "metadata": metadata,
        "metadata_sha256": sha256(metadata_path),
        "provenance": provenance,
        "global_pcd": global_pcd,
        "verified_file_count": verified,
    }


def verify_frontend(odom_csv, frontend_manifest, scan_association, provenance):
    odom_csv = Path(odom_csv).resolve(strict=True)
    manifest_path = Path(frontend_manifest).resolve(strict=True)
    manifest = read_json(manifest_path)
    expected_manifest_sha = provenance["frontend"]["manifest_sha256"]
    if sha256(manifest_path) != expected_manifest_sha:
        raise ValueError("FAST-LIO manifest digest differs from map provenance")
    if manifest != provenance["frontend"]["manifest"]:
        raise ValueError("FAST-LIO manifest contents differ from map provenance")
    settings = manifest.get("settings", {})
    if settings.get("gnss_mode") != "off" or settings.get("gps_input_loaded") != "false":
        raise ValueError("FAST-LIO source trajectory was not generated GNSS-off")

    association = read_json(scan_association)
    if (
        association.get("scan_topic") != "/cloud_registered_body"
        or association.get("pose_topic") != "/Odometry"
        or association.get("scan_count") != association.get("pose_count")
        or association.get("exact_timestamp_match_count")
        != association.get("scan_count")
    ):
        raise ValueError("frontend scan/ODOM association is not exact and complete")

    rows = read_csv(odom_csv)
    if len(rows) != association["pose_count"]:
        raise ValueError("ODOM CSV row count differs from its association report")
    poses = {}
    last_time = None
    for row in rows:
        time_ns = timestamp_ns(row["timestamp"])
        if last_time is not None and time_ns <= last_time:
            raise ValueError("ODOM timestamps are not strictly increasing")
        last_time = time_ns
        if time_ns in poses:
            raise ValueError("duplicate ODOM timestamp")
        poses[time_ns] = row
    return {
        "rows": rows,
        "poses": poses,
        "sha256": sha256(odom_csv),
        "manifest_sha256": sha256(manifest_path),
        "scan_association_sha256": sha256(scan_association),
    }


def verify_gnss(gnss_csv, gnss_source_csv, gnss_contract, raw_gga):
    gnss_csv = Path(gnss_csv).resolve(strict=True)
    source_csv = Path(gnss_source_csv).resolve(strict=True)
    contract_path = Path(gnss_contract).resolve(strict=True)
    contract = read_json(contract_path)
    if sha256(gnss_csv) != contract.get("csv_sha256"):
        raise ValueError("projected RTK CSV digest differs from its contract")
    if sha256(source_csv) != contract.get("source_csv_sha256"):
        raise ValueError("GNSS source CSV digest differs from its contract")
    expected_raw_hash = (
        contract.get("raw_gga_stream", {}).get("sha256")
        or contract.get("raw_gga_stream_sha256")
    )
    if expected_raw_hash and sha256(raw_gga) != expected_raw_hash:
        raise ValueError("raw GGA digest differs from the GNSS contract")
    if contract.get("independent_reference") is not False:
        raise ValueError("GNSS contract unexpectedly claims independent truth")
    if contract.get("uncertainty_calibrated") is not False:
        raise ValueError("GNSS contract unexpectedly claims calibrated uncertainty")

    rows = read_csv(gnss_csv)
    if not rows:
        raise ValueError("projected RTK CSV is empty")
    fixes = []
    last_time = None
    for row in rows:
        time_ns = timestamp_ns(row["timestamp"])
        if last_time is not None and time_ns <= last_time:
            raise ValueError("RTK measurement times are not strictly increasing")
        last_time = time_ns
        if int(row["quality"]) != 4:
            continue
        fixes.append(
            {
                "timestamp_ns": time_ns,
                "east_m": float(row["east_m"]),
                "north_m": float(row["north_m"]),
                "correction_age_s": float(row["correction_age_s"]),
                "satellites_used": int(row["satellites_used"]),
                "hdop": float(row["hdop"]),
                "station_id": row["station_id"],
                "quality": int(row["quality"]),
            }
        )
    if not fixes:
        raise ValueError("projected RTK CSV contains no quality-4 epochs")
    return {
        "fixes": fixes,
        "sha256": sha256(gnss_csv),
        "source_csv_sha256": sha256(source_csv),
        "contract_sha256": sha256(contract_path),
        "raw_gga_sha256": sha256(raw_gga),
        "contract": contract,
        "row_count": len(rows),
    }


def quaternion_from_matrix(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = (
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
            0.25 * scale,
        )
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        q = (
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[2][1] - matrix[1][2]) / scale,
        )
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        q = (
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
        )
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        q = (
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    return normalize_quaternion(q)


def normalize_quaternion(q):
    length = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("invalid zero or nonfinite quaternion")
    return tuple(value / length for value in q)


def rotation_matrix(q):
    x, y, z, w = normalize_quaternion(q)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def pose_matrix(row):
    matrix = [[0.0] * 4 for _ in range(4)]
    rotation = rotation_matrix(tuple(float(row[key]) for key in ("qx", "qy", "qz", "qw")))
    for index in range(3):
        matrix[index][:3] = rotation[index]
        matrix[index][3] = float(row[("x", "y", "z")[index]])
    matrix[3][3] = 1.0
    return matrix


def matrix_multiply(lhs, rhs):
    return [
        [
            sum(lhs[row][inner] * rhs[inner][column] for inner in range(4))
            for column in range(4)
        ]
        for row in range(4)
    ]


def pose_fields(matrix):
    rotation = [row[:3] for row in matrix[:3]]
    quaternion = quaternion_from_matrix(rotation)
    return {
        "x": matrix[0][3],
        "y": matrix[1][3],
        "z": matrix[2][3],
        "qx": quaternion[0],
        "qy": quaternion[1],
        "qz": quaternion[2],
        "qw": quaternion[3],
    }


def rotate(q, vector):
    matrix = rotation_matrix(q)
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quaternion_angle(q):
    return 2.0 * math.acos(min(1.0, abs(normalize_quaternion(q)[3])))


def rigid_transform_status(matrix):
    rotation = [row[:3] for row in matrix[:3]]
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    error = max(
        abs(
            sum(rotation[k][row] * rotation[k][column] for k in range(3))
            - (1.0 if row == column else 0.0)
        )
        for row in range(3)
        for column in range(3)
    )
    return error, determinant


def linear_percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def metric_summary(values):
    if not values:
        return {"count": 0, "median": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "p95": linear_percentile(values, 95.0),
        "max": max(values),
    }


def interpolate_fix(fixes, times, query_ns, max_gap_ns):
    index = bisect.bisect_left(times, query_ns)
    if index < len(times) and times[index] == query_ns:
        left = right = fixes[index]
        gap_ns = 0
        fraction = 0.0
    elif index == 0:
        return None, "before_first_fix"
    elif index == len(times):
        return None, "after_last_fix"
    else:
        left = fixes[index - 1]
        right = fixes[index]
        gap_ns = right["timestamp_ns"] - left["timestamp_ns"]
        if gap_ns > max_gap_ns:
            return None, "bracket_gap_exceeds_limit"
        fraction = (query_ns - left["timestamp_ns"]) / gap_ns
    if gap_ns > max_gap_ns:
        return None, "bracket_gap_exceeds_limit"
    interpolated = {}
    for key in ("east_m", "north_m", "correction_age_s"):
        interpolated[key] = left[key] + fraction * (right[key] - left[key])
    interpolated["quality"] = 4
    interpolated["station_id"] = (
        left["station_id"] if left["station_id"] == right["station_id"] else ""
    )
    interpolated["bracket_gap_s"] = gap_ns / 1_000_000_000.0
    return interpolated, ""


def point_with_lever_arm(pose, lever_arm):
    quaternion = tuple(pose[key] for key in ("qx", "qy", "qz", "qw"))
    offset = rotate(quaternion, lever_arm)
    return [pose[key] + offset[index] for index, key in enumerate(("x", "y", "z"))]


def percentile_summary(samples):
    return {
        "count": len(samples),
        "median_m": linear_percentile(samples, 50.0),
        "p95_m": linear_percentile(samples, 95.0),
        "max_m": max(samples) if samples else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-root", required=True)
    parser.add_argument("--odom-csv", required=True)
    parser.add_argument("--frontend-manifest", required=True)
    parser.add_argument("--scan-association", required=True)
    parser.add_argument("--gnss-csv", required=True)
    parser.add_argument("--gnss-source-csv", required=True)
    parser.add_argument("--gnss-contract", required=True)
    parser.add_argument("--raw-gga", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--matcher",
        choices=("ndt", "point-to-plane-icp"),
        default="ndt",
    )
    parser.add_argument("--seed-source", choices=("odom", "optimized"), default="odom")
    parser.add_argument("--scan-voxel-m", type=float, default=0.5)
    parser.add_argument("--map-voxel-m", type=float, default=0.0)
    parser.add_argument("--ndt-resolution-m", type=float, default=1.0)
    parser.add_argument("--ndt-step-size-m", type=float, default=0.1)
    parser.add_argument("--ndt-transformation-epsilon-m", type=float, default=0.001)
    parser.add_argument("--ndt-max-iterations", type=int, default=40)
    parser.add_argument("--icp-max-points", type=int, default=12000)
    parser.add_argument("--icp-iterations", type=int, default=40)
    parser.add_argument("--icp-normal-neighbors", type=int, default=15)
    parser.add_argument("--icp-normal-planarity-ratio", type=float, default=0.1)
    parser.add_argument("--icp-distance-m", type=float, default=1.5)
    parser.add_argument("--icp-min-pairs", type=int, default=100)
    parser.add_argument("--icp-min-source-overlap", type=float, default=0.4)
    parser.add_argument("--icp-min-target-overlap", type=float, default=0.4)
    parser.add_argument("--prewarm-local-windows", action="store_true")
    parser.add_argument("--profile-icp-timing", action="store_true")
    parser.add_argument("--icp-max-rmse-m", type=float, default=0.2)
    parser.add_argument("--icp-condition-ratio", type=float, default=1e-6)
    parser.add_argument("--icp-translation-tolerance-m", type=float, default=0.001)
    parser.add_argument("--icp-rotation-tolerance-rad", type=float, default=0.0001)
    parser.add_argument(
        "--icp-max-correction-translation-m",
        type=float,
        default=15.0,
    )
    parser.add_argument(
        "--icp-max-correction-rotation-rad",
        type=float,
        default=1.0471975511965976,
    )
    args = parser.parse_args()
    for name in (
        "scan_voxel_m",
        "ndt_resolution_m",
        "ndt_step_size_m",
        "ndt_transformation_epsilon_m",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if not math.isfinite(args.map_voxel_m) or args.map_voxel_m < 0.0:
        parser.error("--map-voxel-m must be finite and nonnegative")
    if args.ndt_max_iterations <= 0:
        parser.error("--ndt-max-iterations must be positive")
    for name in (
        "icp_normal_planarity_ratio",
        "icp_distance_m",
        "icp_min_source_overlap",
        "icp_min_target_overlap",
        "icp_max_rmse_m",
        "icp_condition_ratio",
        "icp_translation_tolerance_m",
        "icp_rotation_tolerance_rad",
        "icp_max_correction_translation_m",
        "icp_max_correction_rotation_rad",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if (
        args.icp_max_points <= 0
        or args.icp_iterations <= 0
        or args.icp_normal_neighbors < 4
        or args.icp_min_pairs < 10
        or args.icp_max_points < args.icp_min_pairs
        or args.icp_min_source_overlap > 1.0
        or args.icp_min_target_overlap > 1.0
        or args.icp_normal_planarity_ratio > 1.0
        or args.icp_condition_ratio > 1.0
    ):
        parser.error("invalid point-to-plane ICP point/overlap/observability budget")

    map_data = verify_map_artifact(args.map_root)
    frontend = verify_frontend(
        args.odom_csv,
        args.frontend_manifest,
        args.scan_association,
        map_data["provenance"],
    )
    gnss = verify_gnss(
        args.gnss_csv,
        args.gnss_source_csv,
        args.gnss_contract,
        args.raw_gga,
    )
    metadata = map_data["metadata"]
    gnss_map_contract = metadata["lineage"]["gnss_contract"]
    map_from_world = gnss_map_contract["T_map_world"]
    orthogonality_error, determinant = rigid_transform_status(map_from_world)
    if orthogonality_error > 1e-6 or abs(determinant - 1.0) > 1e-6:
        raise ValueError("map-from-world seed is not a rigid transform")
    map_from_world_pose = pose_fields(map_from_world)

    odom_by_time = frontend["poses"]
    frames = read_csv(map_data["root"] / "frames.csv")
    optimized = read_csv(map_data["root"] / "pose_graph" / "optimized_frames.csv")
    if len(frames) != len(optimized) or not frames:
        raise ValueError("map frame manifest and optimized pose counts differ")
    requests_path = Path(args.output_dir) / "scan_requests.csv"
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    request_rows = []
    alignment_position_errors = []
    alignment_rotation_errors = []
    last_timestamp = None
    for frame, map_pose_row in zip(frames, optimized):
        time_ns = int(frame["timestamp_ns"])
        if int(map_pose_row["timestamp_ns"]) != time_ns:
            raise ValueError("map frame and optimized pose timestamps differ")
        if last_timestamp is not None and time_ns <= last_timestamp:
            raise ValueError("map scan timestamps are not strictly increasing")
        last_timestamp = time_ns
        if frame["cloud"] != map_pose_row["cloud"]:
            raise ValueError("map frame cloud path differs from optimized pose row")
        if time_ns not in odom_by_time:
            raise ValueError(f"no exact sensor-time ODOM pose for {time_ns}")
        odom_matrix = pose_matrix(odom_by_time[time_ns])
        aligned_odom = matrix_multiply(map_from_world, odom_matrix)
        aligned_pose = pose_fields(aligned_odom)
        optimized_pose = pose_fields(pose_matrix(map_pose_row))
        initial_pose = optimized_pose if args.seed_source == "optimized" else aligned_pose
        alignment_position_errors.append(
            math.dist(
                [aligned_pose[key] for key in ("x", "y", "z")],
                [optimized_pose[key] for key in ("x", "y", "z")],
            )
        )
        q_aligned = tuple(aligned_pose[key] for key in ("qx", "qy", "qz", "qw"))
        q_optimized = tuple(optimized_pose[key] for key in ("qx", "qy", "qz", "qw"))
        relative = quaternion_multiply(quaternion_conjugate(q_aligned), q_optimized)
        alignment_rotation_errors.append(math.degrees(quaternion_angle(relative)))
        scan_path = assert_under_root(map_data["root"], frame["cloud"])
        request_rows.append(
            {
                "timestamp_ns": time_ns,
                "scan_path": str(scan_path),
                **initial_pose,
            }
        )

    with requests_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["timestamp_ns", "scan_path", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for row in request_rows:
            writer.writerow(
                [
                    row["timestamp_ns"],
                    row["scan_path"],
                    *(format(row[key], ".17g") for key in ("x", "y", "z", "qx", "qy", "qz", "qw")),
                ]
            )

    result_csv = output_dir / "scan_matches.csv"
    runner = Path(args.runner).resolve(strict=True)
    command = [
        str(runner),
        str(map_data["global_pcd"]),
        str(requests_path),
        str(result_csv),
        MAP_VERSION,
        MAP_VERSION,
        MAP_CONTENT_ID,
        "--matcher",
        args.matcher,
        "--scan-voxel-m",
        str(args.scan_voxel_m),
        "--map-voxel-m",
        str(args.map_voxel_m),
        "--ndt-resolution-m",
        str(args.ndt_resolution_m),
        "--ndt-step-size-m",
        str(args.ndt_step_size_m),
        "--ndt-transformation-epsilon-m",
        str(args.ndt_transformation_epsilon_m),
        "--ndt-max-iterations",
        str(args.ndt_max_iterations),
        "--icp-max-points",
        str(args.icp_max_points),
        "--icp-iterations",
        str(args.icp_iterations),
        "--icp-normal-neighbors",
        str(args.icp_normal_neighbors),
        "--icp-normal-planarity-ratio",
        str(args.icp_normal_planarity_ratio),
        "--icp-distance-m",
        str(args.icp_distance_m),
        "--icp-min-pairs",
        str(args.icp_min_pairs),
        "--icp-min-source-overlap",
        str(args.icp_min_source_overlap),
        "--icp-min-target-overlap",
        str(args.icp_min_target_overlap),
        "--icp-max-rmse-m",
        str(args.icp_max_rmse_m),
        "--icp-condition-ratio",
        str(args.icp_condition_ratio),
        "--icp-translation-tolerance-m",
        str(args.icp_translation_tolerance_m),
        "--icp-rotation-tolerance-rad",
        str(args.icp_rotation_tolerance_rad),
        "--icp-max-correction-translation-m",
        str(args.icp_max_correction_translation_m),
        "--icp-max-correction-rotation-rad",
        str(args.icp_max_correction_rotation_rad),
        "--prewarm-local-windows",
        str(args.prewarm_local_windows).lower(),
        "--profile-icp-timing",
        str(args.profile_icp_timing).lower(),
    ]
    process = subprocess.run(command, check=False, text=True, capture_output=True)
    if process.returncode != 0:
        raise RuntimeError(
            "scan-to-map replay failed "
            f"(exit={process.returncode}): {process.stderr[-4000:]}"
        )
    matches = read_csv(result_csv)
    if len(matches) != len(request_rows):
        raise ValueError("matcher result count differs from replay scan count")
    runner_summary = {}
    for output_line in process.stdout.splitlines():
        fields = output_line.split()
        if fields and fields[0].startswith("processed_scans="):
            runner_summary = dict(field.split("=", 1) for field in fields)
    if not runner_summary:
        raise ValueError("matcher runner did not emit its replay summary")
    if int(runner_summary["processed_scans"]) != len(request_rows):
        raise ValueError("matcher runner summary scan count differs from requests")
    prewarmed_window_count = int(runner_summary["prewarmed_local_windows"])
    cached_window_count = int(runner_summary["cached_windows"])
    if args.prewarm_local_windows and (
        prewarmed_window_count == 0 or prewarmed_window_count != cached_window_count
    ):
        raise ValueError("local-window preparation did not cache every replay window")
    if not args.prewarm_local_windows and prewarmed_window_count != 0:
        raise ValueError("runner reported prepared windows when prewarming is disabled")
    for row, match in zip(request_rows, matches):
        if int(match["timestamp_ns"]) != row["timestamp_ns"]:
            raise ValueError("matcher output timestamps differ from input scans")

    jacobian = metadata["coordinate_reference"][
        "map_xy_to_local_enu_xy_jacobian"
    ]["matrix_m_per_m"]
    determinant_j = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]
    if abs(determinant_j) < 1e-9:
        raise ValueError("map-to-local-horizontal Jacobian is singular")
    inv_j = [
        [jacobian[1][1] / determinant_j, -jacobian[0][1] / determinant_j],
        [-jacobian[1][0] / determinant_j, jacobian[0][0] / determinant_j],
    ]
    lever_arm = gnss_map_contract.get("antenna_in_cloud_m")
    if not lever_arm or len(lever_arm) != 3:
        raise ValueError("conditional antenna lever arm is missing")
    fixes = gnss["fixes"]
    fix_times = [fix["timestamp_ns"] for fix in fixes]
    max_gap_ns = int(GNSS_MAX_BRACKET_GAP_S * 1_000_000_000)
    scored_rows = []
    odom_errors = []
    match_errors = []
    common_odom_errors = []
    common_match_errors = []
    match_status_counts = {}
    rejection_reasons = {}
    match_latencies = []
    match_iterations = []
    source_overlaps = []
    target_overlaps = []
    converged_matches = 0
    nonconverged_matches = 0
    matched_poses = []
    matched_odom_poses = []
    for request, match in zip(request_rows, matches):
        match_status_counts[match["status"]] = (
            match_status_counts.get(match["status"], 0) + 1
        )
        if match["status"] != "MATCHED":
            rejection_reasons[match["reason"]] = (
                rejection_reasons.get(match["reason"], 0) + 1
            )
        match_latencies.append(float(match["processing_time_ms"]))
        match_iterations.append(int(match["iterations"]))
        source_overlaps.append(float(match["source_overlap"]))
        target_overlaps.append(float(match["target_overlap"]))
        if int(match["converged"]):
            converged_matches += 1
        else:
            nonconverged_matches += 1
        interpolated, skip_reason = interpolate_fix(
            fixes, fix_times, request["timestamp_ns"], max_gap_ns
        )
        row_out = {
            "timestamp_ns": request["timestamp_ns"],
            "gnss_status": "INTERPOLATED" if interpolated else "UNMATCHED",
            "gnss_skip_reason": skip_reason,
            "gnss_bracket_gap_s": (
                interpolated["bracket_gap_s"] if interpolated else ""
            ),
            "gnss_correction_age_s": (
                interpolated["correction_age_s"] if interpolated else ""
            ),
            "gnss_quality": interpolated["quality"] if interpolated else "",
            "match_status": match["status"],
            "match_reason": match["reason"],
            "match_score": match["score"],
            "match_overlap": match["overlap"],
            "match_source_overlap": match["source_overlap"],
            "match_target_overlap": match["target_overlap"],
            "match_converged": match["converged"],
            "match_iterations": match["iterations"],
            "match_latency_ms": match["processing_time_ms"],
            "map_window_build_ms": match["map_window_build_ms"],
            "target_voxelization_ms": match["target_voxelization_ms"],
            "target_kdtree_build_ms": match["target_kdtree_build_ms"],
            "target_normal_estimation_ms": match["target_normal_estimation_ms"],
            "target_normal_neighbor_search_ms": match[
                "target_normal_neighbor_search_ms"
            ],
            "target_normal_fit_ms": match["target_normal_fit_ms"],
            "icp_nearest_search_ms": match["icp_nearest_search_ms"],
            "icp_linearization_ms": match["icp_linearization_ms"],
            "icp_solver_ms": match["icp_solver_ms"],
            "match_hessian_min_eigenvalue": match["hessian_min_eigenvalue"],
            "match_hessian_condition_number": match["hessian_condition_number"],
            "match_geometry_full_rank": match["geometry_full_rank"],
            "match_observability_available": match["observability_available"],
            "match_covariance_calibrated": match["covariance_calibrated"],
            "match_calibration_state": match["calibration_state"],
            "odom_body_x_m": request["x"],
            "odom_body_y_m": request["y"],
            "match_body_x_m": match["x"],
            "match_body_y_m": match["y"],
        }
        odom_antenna = point_with_lever_arm(request, lever_arm)
        if interpolated:
            map_rtk_xy = [
                inv_j[0][0] * interpolated["east_m"]
                + inv_j[0][1] * interpolated["north_m"],
                inv_j[1][0] * interpolated["east_m"]
                + inv_j[1][1] * interpolated["north_m"],
            ]
            odom_error = [
                odom_antenna[0] - map_rtk_xy[0],
                odom_antenna[1] - map_rtk_xy[1],
            ]
            odom_distance = math.hypot(*odom_error)
            odom_errors.append(odom_distance)
            row_out.update(
                {
                    "rtk_map_east_m": map_rtk_xy[0],
                    "rtk_map_north_m": map_rtk_xy[1],
                    "odom_antenna_east_m": odom_antenna[0],
                    "odom_antenna_north_m": odom_antenna[1],
                    "odom_error_east_m": odom_error[0],
                    "odom_error_north_m": odom_error[1],
                    "odom_horizontal_error_m": odom_distance,
                }
            )
            if match["status"] == "MATCHED" and match["valid"] == "1":
                matched_pose = {
                    "timestamp_ns": request["timestamp_ns"],
                    **{
                        key: float(match[key])
                        for key in ("x", "y", "z", "qx", "qy", "qz", "qw")
                    },
                }
                matched_antenna = point_with_lever_arm(matched_pose, lever_arm)
                match_error = [
                    matched_antenna[0] - map_rtk_xy[0],
                    matched_antenna[1] - map_rtk_xy[1],
                ]
                match_distance = math.hypot(*match_error)
                match_errors.append(match_distance)
                common_odom_errors.append(odom_distance)
                common_match_errors.append(match_distance)
                matched_poses.append(matched_pose)
                matched_odom_poses.append(request)
                row_out.update(
                    {
                        "match_antenna_east_m": matched_antenna[0],
                        "match_antenna_north_m": matched_antenna[1],
                        "match_error_east_m": match_error[0],
                        "match_error_north_m": match_error[1],
                        "match_horizontal_error_m": match_distance,
                        "match_minus_odom_east_m": matched_pose["x"] - request["x"],
                        "match_minus_odom_north_m": matched_pose["y"] - request["y"],
                    }
                )
        scored_rows.append(row_out)

    comparison_csv = output_dir / "trajectory_comparison.csv"
    fieldnames = sorted({key for row in scored_rows for key in row})
    with comparison_csv.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(scored_rows)

    increment_translation_deltas = []
    increment_rotation_deltas_deg = []
    for index in range(1, len(matched_poses)):
        previous_match = matched_poses[index - 1]
        current_match = matched_poses[index]
        previous_odom = matched_odom_poses[index - 1]
        current_odom = matched_odom_poses[index]
        if (
            current_match["timestamp_ns"] - previous_match["timestamp_ns"]
            != current_odom["timestamp_ns"] - previous_odom["timestamp_ns"]
        ):
            continue
        match_delta = [
            current_match[key] - previous_match[key] for key in ("x", "y", "z")
        ]
        odom_delta = [
            current_odom[key] - previous_odom[key] for key in ("x", "y", "z")
        ]
        increment_translation_deltas.append(
            math.dist(match_delta[:2], odom_delta[:2])
        )
        q_match_prev = tuple(previous_match[key] for key in ("qx", "qy", "qz", "qw"))
        q_match_curr = tuple(current_match[key] for key in ("qx", "qy", "qz", "qw"))
        q_odom_prev = tuple(previous_odom[key] for key in ("qx", "qy", "qz", "qw"))
        q_odom_curr = tuple(current_odom[key] for key in ("qx", "qy", "qz", "qw"))
        match_relative = quaternion_multiply(
            quaternion_conjugate(q_match_prev), q_match_curr
        )
        odom_relative = quaternion_multiply(
            quaternion_conjugate(q_odom_prev), q_odom_curr
        )
        relative_difference = quaternion_multiply(
            quaternion_conjugate(odom_relative), match_relative
        )
        increment_rotation_deltas_deg.append(
            math.degrees(quaternion_angle(relative_difference))
        )

    correction_ages = [fix["correction_age_s"] for fix in fixes]
    summary = {
        "schema_version": 1,
        "result_classification": "same_record_map_consistency_diagnostic_only",
        "run_mode": "GNSS-off ODOM prediction plus scan-to-map; RTK used only offline",
        "data_source_scope": (
            "196 frontend-deskewed body-frame keyframe PCDs and the matched "
            "GNSS-off FAST-LIO2 ODOM export; this is not raw-LiDAR bag replay"
        ),
        "map": {
            "version": MAP_VERSION,
            "content_id": MAP_CONTENT_ID,
            "global_pcd_sha256": sha256(map_data["global_pcd"]),
            "metadata_sha256": map_data["metadata_sha256"],
            "verified_inventory_files": map_data["verified_file_count"],
            "points": metadata["global_cloud"]["points"],
            "physical_up_qualified": False,
            "independent_reference": False,
        },
        "frontend": {
            "manifest_sha256": frontend["manifest_sha256"],
            "gnss_mode": "off",
            "odom_csv_sha256": frontend["sha256"],
            "odom_pose_count": len(frontend["rows"]),
            "exact_scan_odom_time_matches": len(request_rows),
            "scan_association_sha256": frontend["scan_association_sha256"],
        },
        "initialization": {
            "source": (
                "optimized map keyframe pose"
                if args.seed_source == "optimized"
                else "map artifact diagnostic T_map_world metadata"
            ),
            "seed_source": args.seed_source,
            "T_map_world": map_from_world,
            "uses_gnss_as_localizer_input": False,
            "map_transform_is_derived_from_same_record_gnss": True,
            "map_to_odom_vs_optimized_pose_translation_m": metric_summary(
                alignment_position_errors
            ),
            "map_to_odom_vs_optimized_pose_rotation_deg": metric_summary(
                alignment_rotation_errors
            ),
        },
        "matcher": {
            "input_scans": len(request_rows),
            "status_counts": match_status_counts,
            "rejection_reasons": rejection_reasons,
            "convergence_counts": {
                "converged": converged_matches,
                "not_converged": nonconverged_matches,
            },
            "iterations": metric_summary(match_iterations),
            "source_overlap": metric_summary(source_overlaps),
            "target_overlap": metric_summary(target_overlaps),
            "latency_ms": metric_summary(match_latencies),
            "score_type": (
                "point_to_plane_rmse_m"
                if args.matcher == "point-to-plane-icp"
                else "pcl_ndt_fitness"
            ),
            "config": {
                "algorithm": args.matcher,
                "scan_voxel_size_m": args.scan_voxel_m,
                "map_voxel_size_m": (
                    args.map_voxel_m
                    if args.map_voxel_m > 0.0
                    else args.scan_voxel_m
                ),
                "ndt_resolution_m": args.ndt_resolution_m,
                "ndt_step_size_m": args.ndt_step_size_m,
                "ndt_transformation_epsilon_m": args.ndt_transformation_epsilon_m,
                "ndt_maximum_iterations": args.ndt_max_iterations,
                "icp_maximum_points": args.icp_max_points,
                "icp_maximum_iterations": args.icp_iterations,
                "icp_normal_neighbors": args.icp_normal_neighbors,
                "icp_normal_planarity_ratio": args.icp_normal_planarity_ratio,
                "icp_correspondence_distance_m": args.icp_distance_m,
                "icp_minimum_pairs": args.icp_min_pairs,
                "icp_minimum_source_overlap": args.icp_min_source_overlap,
                "icp_minimum_target_overlap": args.icp_min_target_overlap,
                "icp_maximum_point_plane_rmse_m": args.icp_max_rmse_m,
                "icp_condition_ratio": args.icp_condition_ratio,
                "icp_translation_tolerance_m": args.icp_translation_tolerance_m,
                "icp_rotation_tolerance_rad": args.icp_rotation_tolerance_rad,
                "icp_maximum_correction_translation_m": (
                    args.icp_max_correction_translation_m
                ),
                "icp_maximum_correction_rotation_rad": (
                    args.icp_max_correction_rotation_rad
                ),
                "detailed_icp_timing_enabled": args.profile_icp_timing,
                "solver": (
                    "OpenLoc custom point-to-plane ICP"
                    if args.matcher == "point-to-plane-icp"
                    else "PCL NormalDistributionsTransform"
                ),
                "maximum_fitness": 0.5,
                "correspondence_distance_m": 1.0,
                "minimum_overlap": 0.5,
                "minimum_points": 100,
                "local_window_radius_m": 40.0,
                "window_cell_size_m": 20.0,
                "maximum_cached_windows": 8,
                "threshold_provenance": "initial diagnostic defaults; uncalibrated",
            },
            "local_window_preparation": {
                "enabled": args.prewarm_local_windows,
                "prepared_window_count": int(
                    runner_summary["prewarmed_local_windows"]
                ),
                "preparation_time_ms": float(
                    runner_summary["window_preparation_ms"]
                ),
                "cached_window_count_after_replay": int(
                    runner_summary["cached_windows"]
                ),
                "cost_is_included_in_per_frame_latency": False,
            },
            "covariance_calibrated": False,
            "calibration_state": "uncalibrated",
        },
        "gnss": {
            "contract_sha256": gnss["contract_sha256"],
            "projected_track_sha256": gnss["sha256"],
            "source_csv_sha256": gnss["source_csv_sha256"],
            "raw_gga_sha256": gnss["raw_gga_sha256"],
            "input_epochs": gnss["row_count"],
            "quality_4_epochs": len(fixes),
            "quality_4_age_s": {
                "median": statistics.median(correction_ages),
                "p95": linear_percentile(correction_ages, 95.0),
                "max": max(correction_ages),
                "over_5_s": sum(age > 5.0 for age in correction_ages),
                "over_10_s": sum(age > 10.0 for age in correction_ages),
            },
            "independent_reference": False,
            "uncertainty_calibrated": False,
            "vertical_used": False,
            "interpolation_limit_s": GNSS_MAX_BRACKET_GAP_S,
            "antenna_lever_arm_body_m": lever_arm,
            "antenna_lever_arm_verified": False,
        },
        "comparison": {
            "common_keyframe_times_with_bracketed_rtk": sum(
                row["gnss_status"] == "INTERPOLATED" for row in scored_rows
            ),
            "unmatched_gnss_reasons": {
                reason: sum(row["gnss_skip_reason"] == reason for row in scored_rows)
                for reason in sorted(
                    {row["gnss_skip_reason"] for row in scored_rows if row["gnss_skip_reason"]}
                )
            },
            "odom_vs_rtk_horizontal_error_m": percentile_summary(odom_errors),
            "scan_to_map_vs_rtk_horizontal_error_m": percentile_summary(match_errors),
            "same_matches_odom_vs_rtk_horizontal_error_m": percentile_summary(
                common_odom_errors
            ),
            "same_matches_scan_to_map_vs_rtk_horizontal_error_m": percentile_summary(
                common_match_errors
            ),
            "increment_translation_difference_m": metric_summary(
                increment_translation_deltas
            ),
            "increment_rotation_difference_deg": metric_summary(
                increment_rotation_deltas_deg
            ),
            "evidence_note": (
                "RTK is receiver-reported quality-4 data and was used in "
                "georeferencing this same map. These are in-sample horizontal "
                "consistency residuals, not independent APE or accuracy."
            ),
        },
        "outputs": {
            "scan_requests_csv": str(requests_path),
            "scan_matches_csv": str(result_csv),
            "trajectory_comparison_csv": str(comparison_csv),
        },
        "provenance": {
            "runner_sha256": sha256(runner),
            "script_sha256": sha256(Path(__file__)),
            "python": platform.python_version(),
            "command": command,
            "runner_stdout": process.stdout.strip(),
        },
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary["comparison"], indent=2, sort_keys=True))
    print(f"summary_json={summary_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, RuntimeError, csv.Error) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2) from error
