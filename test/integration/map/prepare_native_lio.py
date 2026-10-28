#!/usr/bin/env python3
"""Prepare bounded raw Livox/IMU inputs, without external odometry or seeds."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def timestamp(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def imu_values(message, receive_ns, scale):
    values = [
        getattr(message.linear_acceleration, axis) * scale for axis in "xyz"
    ] + [getattr(message.angular_velocity, axis) for axis in "xyz"]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite raw IMU")
    return [timestamp(message.header), receive_ns, *values]


def scan_values(message, max_points):
    if message.point_num != len(message.points) or not message.points:
        raise ValueError("raw Livox point count mismatch or empty scan")
    if int(message.timebase) <= 0 or max_points <= 0:
        raise ValueError("invalid raw timebase or point capacity")
    if abs(timestamp(message.header) - int(message.timebase)) > 1_000_000:
        raise ValueError("Livox timebase/header differ by more than 1 ms")
    offsets = [int(point.offset_time) for point in message.points]
    if any(b < a for a, b in zip(offsets, offsets[1:])):
        raise ValueError("raw point times regressed")
    valid = []
    for point in message.points:
        xyz = [float(getattr(point, axis)) for axis in "xyz"]
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("nonfinite raw LiDAR point")
        if xyz == [0.0, 0.0, 0.0]:
            continue
        valid.append([int(message.timebase) + int(point.offset_time), *xyz])
    stride = max(1, math.ceil(len(valid) / max_points))
    return int(message.timebase) + max(offsets), valid[::stride]


def scan_decision(message, receive_ns, sequence, end_ns, points):
    rejected = sum(p.x == 0 and p.y == 0 and p.z == 0 for p in message.points)
    return [sequence, end_ns, receive_ns, len(message.points), rejected,
            len(points), "selected" if points else "no_valid_returns"]


def prepare(args):
    import yaml
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    profile = yaml.safe_load(args.profile.read_text())
    if (
        profile["device"]["imu"]["acceleration"]["source_unit"] != "g"
        or profile["device"]["imu"]["angular_velocity"]["unit"] != "rad/s"
        or profile["device"]["imu"]["orientation_field"]["use_as_measurement"]
    ):
        raise ValueError("unsupported IMU contract")
    transform = profile["extrinsics"]["T_I_L"]
    if transform["rotation_matrix_row_major"] != [1, 0, 0, 0, 1, 0, 0, 0, 1]:
        raise ValueError("this adapter requires the declared parallel sensor axes")
    config = json.loads(args.config.read_text())
    capacity = config["max_points"]
    if not isinstance(capacity, int) or capacity <= 0:
        raise ValueError("max_points must be a positive integer")
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    (args.output / "points").mkdir()
    store = get_typestore(Stores.ROS1_NOETIC)
    counts = {"imu": 0, "scans": 0, "raw_points": 0, "selected_points": 0,
              "zero_xyz_no_returns": 0, "empty_valid_scans": 0}
    with (
        Reader(args.bag) as reader,
        (args.output / "imu.csv").open("w", newline="") as imu_file,
        (args.output / "scans.csv").open("w", newline="") as scans_file,
        (args.output / "adapter_decisions.csv").open("w", newline="") as decisions_file,
    ):
        connections = [
            connection for connection in reader.connections
            if connection.topic in ("/livox/imu", "/livox/lidar")
        ]
        if {connection.topic for connection in connections} != {
            "/livox/imu", "/livox/lidar"
        }:
            raise ValueError("required raw Livox topics missing")
        for connection in connections:
            store.register(get_types_from_msg(
                connection.msgdef.data, connection.msgtype
            ))
        imu_writer = csv.writer(imu_file)
        imu_writer.writerow("timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz".split(","))
        scan_writer = csv.writer(scans_file)
        scan_writer.writerow(["end_ns", "receive_ns", "points"])
        decision_writer = csv.writer(decisions_file)
        decision_writer.writerow("source_sequence,raw_end_ns,receive_ns,raw_count,"
                                 "no_return_count,selected_count,reason".split(","))
        source_sequence = 0
        start = reader.start_time + int(args.start_offset * 1e9)
        stop = start + int(args.duration * 1e9)
        for connection, receive, raw in reader.messages(
            connections=connections, start=start, stop=stop
        ):
            message = store.deserialize_ros1(raw, connection.msgtype)
            if message.header.frame_id != "livox_frame":
                raise ValueError("raw sensor frame differs from profile")
            if connection.topic == "/livox/imu":
                imu_writer.writerow(imu_values(
                    message, receive,
                    profile["device"]["imu"]["acceleration"]["adapter_scale"],
                ))
                counts["imu"] += 1
            else:
                end, points = scan_values(message, capacity)
                source_sequence += 1
                decision_writer.writerow(scan_decision(
                    message, receive, source_sequence, end, points))
                counts["zero_xyz_no_returns"] += sum(
                    p.x == 0 and p.y == 0 and p.z == 0 for p in message.points
                )
                counts["raw_points"] += len(message.points)
                if not points:
                    counts["empty_valid_scans"] += 1
                    continue
                filename = f"points/{counts['scans']:06d}.csv"
                with (args.output / filename).open("w", newline="") as point_file:
                    writer = csv.writer(point_file)
                    writer.writerow(["timestamp_ns", "x", "y", "z"])
                    writer.writerows(points)
                scan_writer.writerow([end, receive, filename])
                counts["scans"] += 1
                counts["selected_points"] += len(points)
    if not counts["imu"] or not counts["scans"]:
        raise ValueError("bounded interval lacks raw IMU or LiDAR")
    with (args.output / "config.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["name", "value"])
        writer.writerows(config.items())
    with (args.output / "extrinsic.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["tx", "ty", "tz"])
        writer.writerow(transform["translation_m"])
    manifest = {
        "schema": "openloc.native-raw-lio-input.v1",
        "diagnostic_only": True,
        "bag": str(args.bag.resolve()),
        "bag_sha256": digest(args.bag),
        "profile": str(args.profile.resolve()),
        "profile_sha256": digest(args.profile),
        "config_sha256": digest(args.config),
        "start_offset_s": args.start_offset,
        "duration_s": args.duration,
        "counts": counts,
        "body_reference": "integrated IMU origin; LiDAR points use T_I_L",
        "point_time": "timebase + original offset_time, integer nanoseconds",
        "point_admission": "exact zero XYZ is the schema-documented no-return "
                           "sentinel; filter before deterministic stride; no "
                           "range/tag/reflectivity filter; original raw scan end "
                           "retained; all-no-return scans counted, not geometry",
        "scan_admission_journal": "adapter_decisions.csv retains every raw scan "
                                  "arrival/end/source sequence, including empty "
                                  "valid geometry omitted from scans.csv",
        "arrival": "original bag record timestamp, not rewritten to measurement",
        "scan_reference": "first processed IMU frontier after arrival and raw end",
        "external_odometry_used": False,
        "external_initial_pose_used": False,
        "limitations": [
            "Native private LocalEstimator candidate, not FAST-LIO2.",
            "IMU-only quiet-window initialization does not prove zero velocity.",
            "Deterministic point stride bounds work; no external deskew is used.",
            "Noise and LIO configuration are diagnostic and uncalibrated.",
            "Bag timestamps are delivery evidence, not certified hardware arrival.",
            "Nominal manual extrinsic is not independently calibrated.",
            "Native LIO currently revokes joint covariance model eligibility.",
            "No map/global/host or accuracy qualification follows from this replay.",
        ],
        "files": {
            str(path.relative_to(args.output)): digest(path)
            for path in sorted(args.output.rglob("*.csv"))
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(counts))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, required=True)
    args = parser.parse_args()
    if (
        not math.isfinite(args.duration) or not 0 < args.duration <= 180
        or not math.isfinite(args.start_offset) or args.start_offset < 0
    ):
        parser.error("require 0 < duration <= 180 s and finite start-offset >= 0")
    prepare(args)


if __name__ == "__main__":
    main()
