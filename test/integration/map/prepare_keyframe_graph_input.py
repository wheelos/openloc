#!/usr/bin/env python3
"""Prepare frozen native outputs for the explicit approximate keyframe model."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import shutil

from prepare_native_lio import digest
from public_xy_validation import interpolate


POSE = ("px", "py", "pz", "qx", "qy", "qz", "qw")
MEASUREMENTS = (
    "kind,timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw,"
    "projection,covariance,lever_x,lever_y,lever_z,ambiguous"
)


def read(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if any(None in row or None in row.values() for row in rows):
        raise ValueError(f"Malformed CSV: {path}")
    return rows


def write(path, fields, records):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields.split(","))
        writer.writerows(records)


def prepare(replay, model, output, maximum_interpolation_gap_ns):
    if output.exists():
        raise ValueError(f"Output must be new: {output}")
    if maximum_interpolation_gap_ns <= 0:
        raise ValueError("An explicit positive interpolation gap is required")
    paths = [replay / name for name in
             ("odometry.csv", "lidar.csv", "graph_measurements.csv")] + [model]
    pins = {str(path.resolve()): digest(path) for path in paths}
    original = read(paths[0])
    times = [int(row["timestamp_ns"]) for row in original]
    if not times or any(a >= b for a, b in zip(times, times[1:])):
        raise ValueError("Native measurement times must strictly increase")
    odom = []
    started = False
    for row in original:
        if row["valid"] not in ("0", "1"):
            raise ValueError("Invalid native validity flag")
        if row["valid"] == "0":
            if started:
                raise ValueError("A single uninterrupted valid native segment is required")
            continue
        started = True
        pose = [float(row[key]) for key in POSE]
        if (not all(math.isfinite(value) for value in pose)
                or abs(math.sqrt(sum(v * v for v in pose[3:])) - 1) > 1e-9
                or int(row["receive_ns"]) < int(row["timestamp_ns"])):
            raise ValueError("Invalid native pose or reception time")
        odom.append(row)
    if not odom:
        raise ValueError("No valid native states")
    times = [int(row["timestamp_ns"]) for row in odom]
    receives = [int(row["receive_ns"]) for row in odom]
    if any(a > b for a, b in zip(receives, receives[1:])):
        raise ValueError("Native reception times regress")
    measurements, excluded = [], []
    for row in read(paths[2]):
        if set(row) != set(MEASUREMENTS.split(",")):
            raise ValueError("Unexpected graph measurement schema")
        stamp = int(row["timestamp_ns"])
        if row["ambiguous"] not in ("0", "1"):
            raise ValueError("Invalid measurement ambiguity flag")
        reason = ("ambiguous" if row["ambiguous"] == "1" else
                  "outside_valid_native_support"
                  if stamp < times[0] or stamp > times[-1] else None)
        if reason:
            excluded.append({
                "kind": row["kind"], "sequence": row["sequence"],
                "timestamp_ns": row["timestamp_ns"], "reason": reason,
            })
        else:
            measurements.append(row)
    scans = read(paths[1])
    stamps = sorted({int(row["timestamp_ns"]) for row in measurements}
                    | {int(row["reference_ns"]) for row in scans
                       if row["bootstrap_stage"] == "0"
                       and times[0] <= int(row["reference_ns"]) <= times[-1]})
    anchors = [int(row["timestamp_ns"]) for row in measurements
               if row["kind"] == "map"]
    if not anchors:
        raise ValueError("An explicit common full-pose map anchor is required")
    nodes, associations = [], []
    for stamp in stamps:
        right = bisect.bisect_left(times, stamp)
        left = right if times[right] == stamp else right - 1
        if times[right] - times[left] > maximum_interpolation_gap_ns:
            raise ValueError(f"Native interpolation gap exceeds declared limit at {stamp}")
        a, b = odom[left], odom[right]
        if left == right:
            pose = [a[key] for key in POSE]
        else:
            position, rotation = interpolate(
                [times[left], *[float(a[key]) for key in POSE]],
                [times[right], *[float(b[key]) for key in POSE]], stamp)
            pose = [*position, *rotation]
        # A node cannot be available before the right IMU bracket arrives.
        receive = max(int(a["receive_ns"]), int(b["receive_ns"]))
        nodes.append([stamp, *pose, receive])
        associations.append([stamp, times[left], times[right], receive])
    measurements.sort(key=lambda row: int(row["receive_ns"]))
    if any(digest(Path(path)) != pin for path, pin in pins.items()):
        raise ValueError("Input changed during preparation")
    output.mkdir()
    write(output / "nodes.csv",
          "timestamp_ns,px,py,pz,qx,qy,qz,qw,receive_ns", nodes)
    write(output / "measurements.csv", MEASUREMENTS,
          [[row[key] for key in MEASUREMENTS.split(",")] for row in measurements])
    write(output / "associations.csv",
          "node_ns,left_ns,right_ns,native_available_ns", associations)
    shutil.copyfile(model, output / "model.csv")
    protocol = {
        "scope": "frozen native nominal/frontend outputs; not raw sensor replay",
        "independence_approximation": True,
        "native_model": False,
        "trusted": False,
        "future_factors": False,
        "common_full_pose_map_anchor": min(anchors),
        "interpolation": "linear position; shortest-arc quaternion interpolation",
        "interpolation_available_after_right_IMU": True,
        "maximum_interpolation_gap_ns": maximum_interpolation_gap_ns,
        "nodes": len(nodes), "measurements": len(measurements),
        "excluded_measurements": excluded,
        "source_sha256": pins,
        "preparer_sha256": digest(Path(__file__)),
        "input_sha256": {p.name: digest(p) for p in output.iterdir()},
    }
    (output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    return protocol


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-interpolation-gap-ns", type=int, required=True)
    args = parser.parse_args()
    result = prepare(args.replay, args.model, args.output,
                     args.maximum_interpolation_gap_ns)
    print(json.dumps({key: result[key] for key in ("nodes", "measurements",
                                                  "native_model", "trusted")}))


if __name__ == "__main__":
    main()
