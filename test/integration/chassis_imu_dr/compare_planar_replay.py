#!/usr/bin/env python3
"""Frozen Apollo parity and Zhongji planar/ESKF same-input diagnostics."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

from record_dr import interpolate_rows, local_enu, quat_yaw
from wheel_odometry import Series, eligible_intervals, load, replay, statistics, wrap


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def write_csv(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def export_inputs(path, intervals):
    rows = []
    for segment, seed_yaw, grid, inputs in intervals:
        start = not rows or rows[-1]["segment"] != segment
        for index in range(0 if start else 1, len(grid)):
            rows.append({
                "segment": segment, "time": grid[index],
                "vx": inputs[index][0][0], "vy": 0,
                "gz": inputs[index][1][0],
                "seed_yaw": seed_yaw if start and index == 0 else "",
                "score": int(index == len(grid) - 1 or (start and index == 0)),
            })
    require(rows, "No eligible planar inputs")
    write_csv(path, list(rows[0]), rows)


def quiet_wheel_gate(chassis):
    windows = []
    current = None
    for row in chassis:
        time = float(row["header_time"])
        if abs(float(row["speed"])) > 0.02:
            current = None
            continue
        if current is None or time - current["end"] > 0.05:
            current = {"start": time, "end": time, "samples": 0}
            windows.append(current)
        current["end"] = time
        current["samples"] += 1
    longest = max(windows, key=lambda w: w["end"] - w["start"]) if windows else None
    first = float(chassis[0]["header_time"])
    qualifying = [w for w in windows if w["end"] - w["start"] >= 2 and w["samples"] >= 50]
    return {
        "longest_speed_only_quiet_s": longest["end"] - longest["start"] if longest else 0,
        "earliest_possible_init_elapsed_s": qualifying[0]["start"] + 2 - first if qualifying else None,
        "full_gyro_available": False,
        "status": "blocked: gx/gy absent; do not synthesize zero axes",
        "limits": "Wheel-only necessary gate; does not qualify IMU, frame, gear or production initialization.",
    }


def run_apollo(root, output, run):
    audit = json.loads((root / "audit.json").read_text())
    for name, expected in audit["artifact_sha256"].items():
        require(digest(root / name) == expected, f"Frozen Apollo hash mismatch: {name}")
    reports = {}
    for name in ("sensor_rgb", "demo_3.5"):
        destination = output / name
        destination.mkdir()
        chassis = load(root / f"{name}_chassis.csv")
        imu = load(root / f"{name}_imu.csv")
        speed = Series(
            [float(r["header_time"]) for r in chassis],
            [[float(r["speed"])] for r in chassis], 0.05,
        )
        gyro = Series(
            [float(r["measurement_time"]) + 315964782 for r in imu],
            [[float(r["gz"])] for r in imu], 0.05,
        )
        baseline_dir = root / "wheel_odometry_distance_20261014" / name
        baseline_summary = json.loads((baseline_dir / "summary.json").read_text())
        baseline_path = baseline_dir / "wheel_raw_gyro.csv"
        require(
            digest(baseline_path) == baseline_summary["output_sha256"][baseline_path.name],
            "Frozen Python trajectory hash mismatch",
        )
        control = destination / "python_control"
        replay(
            root / f"{name}_chassis.csv", root / f"{name}_imu.csv",
            root / f"{name}_pose.csv", baseline_summary["protocol"]["steering_coefficient"],
            control,
        )
        require(
            digest(control / baseline_path.name) == digest(baseline_path),
            "Shared-grid refactor changed frozen Python trajectory/scoring",
        )
        baseline = load(baseline_path)
        segments = baseline_summary["wheel_raw_gyro"]["segments"]
        intervals = []
        for begin, end, grid, inputs in eligible_intervals(speed, gyro, 0.05):
            if not inputs:
                continue
            segment = next((s for s in segments if s["start"] <= begin and end <= s["end"]), None)
            require(segment is not None, "Unexpected eligible Apollo interval")
            intervals.append((segment["segment"], segment["origin"][2], grid, inputs))
        input_path = destination / "inputs.csv"
        export_inputs(input_path, intervals)
        trajectory_path = destination / "trajectory.csv"
        run(input_path, trajectory_path, 0)
        candidate = [
            row for row in load(trajectory_path)
            if float(row["time"]) != segments[int(row["segment"])]["start"]
        ]
        require(len(candidate) == len(baseline), "Apollo parity sample-count mismatch")
        differences = []
        scored = []
        for actual, previous in zip(candidate, baseline):
            require(
                float(actual["time"]) == float(previous["time"]) and actual["segment"] == previous["segment"],
                "Apollo parity time/segment mismatch",
            )
            delta_position = math.hypot(
                float(actual["local_x"]) - float(previous["local_x"]),
                float(actual["local_y"]) - float(previous["local_y"]),
            )
            delta_yaw = abs(wrap(float(actual["yaw"]) - float(previous["yaw"])))
            delta_distance = abs(float(actual["wheel_distance_m"]) - float(previous["wheel_distance_m"]))
            differences.append((delta_position, delta_yaw, delta_distance))
            require(max(delta_position, delta_yaw, delta_distance) < 1e-8, "C++/Python parity exceeded 1e-8")
            scored.append(actual | {
                "horizontal_error_m": math.hypot(
                    float(actual["local_x"]) - float(previous["reference_x"]),
                    float(actual["local_y"]) - float(previous["reference_y"]),
                ),
            })
        write_csv(destination / "scored.csv", list(scored[0]), scored)
        reports[name] = {
            "paired_samples": len(candidate),
            "max_cpp_python_position_difference_m": max(d[0] for d in differences),
            "max_cpp_python_heading_difference_rad": max(d[1] for d in differences),
            "max_cpp_python_distance_difference_m": max(d[2] for d in differences),
            "segments": segments,
            "eskf_gate": quiet_wheel_gate(chassis),
        }
    return reports


def run_zhongji(root, output, planar, eskf, commands):
    frozen = json.loads((root / "bias_feedback_20261014" / "diagnostic_manifest.json").read_text())
    for name in ("events", "reference"):
        require(digest(root / f"{name}.csv") == frozen[f"{name}_sha256"], f"Frozen {name} mismatch")
    destination = output / "zhongji"
    destination.mkdir()
    trajectory_path = destination / "eskf.csv"
    command = [
        str(eskf), "--events", str(root / "events.csv"), "--trajectory", str(trajectory_path),
        "--wheel-variance", "0.01", "--use-nhc", "false",
    ]
    commands.append(command)
    (output / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
    subprocess.run(command, check=True)
    require(
        digest(trajectory_path) == frozen["results"]["all"]["hashes"]["trajectory.csv"],
        "Original ESKF failed byte-identical frozen baseline reproduction",
    )
    states = [r for r in load(trajectory_path) if r["valid"] == "1"]
    events = load(root / "events.csv")
    wheels = [r for r in events if r["kind"] == "W"]
    imus = [r for r in events if r["kind"] == "I"]
    speed = Series(
        [float(r["measurement_time"]) for r in wheels],
        [[float(r["speed_mps"])] for r in wheels], 0.05,
    )
    gyro = Series(
        [float(r["measurement_time"]) for r in imus],
        [[float(r["gz"])] for r in imus], 0.05,
    )
    start, finish = float(states[0]["time"]), float(states[-1]["time"])
    times = [t for t in gyro.times if start <= t <= finish]
    endpoint_series = Series(times, [[0] for _ in times], 0.05)
    intervals = []
    for begin, end, grid, _ in eligible_intervals(endpoint_series, gyro, 0.05):
        inputs = [(speed.at(t), gyro.at(t)) for t in grid]
        require(all(a is not None and b is not None for a, b in inputs), "Unbracketed Zhongji interval")
        intervals.append((0, quat_yaw(states[0]), grid, inputs))
    input_path = destination / "inputs.csv"
    export_inputs(input_path, intervals)
    trajectories = {"eskf": states}
    for name, bias in (("planar_raw", 0), ("planar_fixed_initial_bias", float(states[0]["gyro_bias_z"]))):
        path = destination / f"{name}.csv"
        planar(input_path, path, bias)
        trajectories[name] = load(path)
    require(
        all([float(r["time"]) for r in rows] == [float(r["time"]) for r in states] for rows in trajectories.values()),
        "Zhongji comparison must pair every initialized ESKF state",
    )
    refs = [
        {k: float(v) for k, v in row.items()}
        for row in load(root / "reference.csv")
    ]
    ref_times = [r["time"] for r in refs]
    first_ref = interpolate_rows(refs, ref_times, start, 0.05)
    require(first_ref is not None, "No reference at common seed")
    origin = (first_ref["latitude"], first_ref["longitude"], first_ref["height_msl"] + first_ref["undulation"])
    for ref in refs:
        ref["east"], ref["north"] = local_enu(origin, ref)
    first_ref = interpolate_rows(refs, ref_times, start, 0.05)
    offset = math.radians(90 - first_ref["heading_deg"]) - quat_yaw(states[0])
    c, s = math.cos(offset), math.sin(offset)
    scored, unscored = [], []
    for index, state in enumerate(states):
        time = float(state["time"])
        ref = interpolate_rows(refs, ref_times, time, 0.05)
        if ref is None:
            unscored.append({"time": time, "reason": "reference_gap_or_unbracketed"})
            continue
        require(ref["status"] == 0 and ref["type"] == 56 and ref["heading_status"] == 0 and ref["heading_type"] == 56,
                "Common reference status invalid")
        require(max(ref["latitude_std_dev"], ref["longitude_std_dev"]) <= 0.10, "Common reference covariance gate failed")
        row = {"time": time, "elapsed_s": time - start,
               "wheel_distance_m": float(trajectories["planar_raw"][index]["wheel_distance_m"])}
        for name, rows in trajectories.items():
            actual = rows[index]
            x, y = (float(actual[k]) for k in (("px", "py") if name == "eskf" else ("local_x", "local_y")))
            yaw = quat_yaw(actual) if name == "eskf" else float(actual["yaw"])
            row[f"{name}_error_m"] = math.hypot(
                c * x - s * y + first_ref["east"] - ref["east"],
                s * x + c * y + first_ref["north"] - ref["north"],
            )
            row[f"{name}_heading_error_deg"] = math.degrees(wrap(yaw + offset - math.radians(90 - ref["heading_deg"])))
        scored.append(row)
    require(scored, "No common scored states")
    write_csv(destination / "paired_scores.csv", list(scored[0]), scored)
    return {
        "paired_samples": len(scored), "integrated_samples": len(states),
        "unscored_states": unscored,
        "duration_s": scored[-1]["time"] - start,
        "integrated_duration_s": finish - start,
        "wheel_distance_m": scored[-1]["wheel_distance_m"],
        "seed_time": start, "initial_gyro_bias_z_radps": float(states[0]["gyro_bias_z"]),
        "eskf_byte_identical_to_frozen_baseline": True,
        "results": {
            name: {
                "horizontal_error_m": statistics([r[f"{name}_error_m"] for r in scored]),
                "endpoint_error_m": scored[-1][f"{name}_error_m"],
                "endpoint_heading_error_deg": scored[-1][f"{name}_heading_error_deg"],
                "distance_checkpoints": [
                    next(r for r in scored if r["wheel_distance_m"] >= threshold)
                    for threshold in (50, 100, 200) if scored[-1]["wheel_distance_m"] >= threshold
                ],
            } for name in trajectories
        },
        "limits": [
            "Scalar forward-only model at the previously declared reference; not a qualified rear-steer twist.",
            "Both candidates share the first valid ESKF local yaw/time; reference used only for scoring alignment.",
            "Fixed bias comes only from the ESKF startup quiet window, not a target-trajectory fit.",
            "Planar input uses bounded offline interpolation; this is not online latency validation.",
            "Planar omits acceleration, tilt and bias feedback together; not a single-path causal ablation.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("apollo-root", "zhongji-root", "output-dir", "planar-runner", "eskf-runner"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    commands = []

    def planar(input_path, output_path, bias):
        command = [
            str(args.planar_runner), "--input", str(input_path), "--output", str(output_path),
            "--gyro-bias", str(bias),
        ]
        commands.append(command)
        (args.output_dir / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        subprocess.run(command, check=True)

    report = {
        "classification": "Same-record consistency; not independent accuracy or production qualification.",
        "commands": commands, "invocation": sys.argv,
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "source_sha256": {
            str(path): digest(path) for path in [
                Path(__file__), Path(__file__).with_name("wheel_odometry.py"),
                Path(__file__).with_name("record_dr.py"),
                Path(__file__).with_name("planar_wheel_replay.cc"),
                Path("src/localization/candidates/local/planar_wheel_dr.cc"),
                Path("src/localization/candidates/local/planar_wheel_dr.h"),
                Path("src/localization/candidates/local/local_estimator.cc"),
                Path("src/localization/candidates/local/local_estimator.h"),
                Path(__file__).with_name("local_estimator_replay.cc"), Path("BUILD"),
            ]
        },
        "binary_sha256": {str(p): digest(p) for p in (args.planar_runner, args.eskf_runner)},
        "frozen_input_sha256": {
            str(p): digest(p) for p in [
                args.apollo_root / name for name in json.loads((args.apollo_root / "audit.json").read_text())["artifact_sha256"]
            ] + [args.zhongji_root / "events.csv", args.zhongji_root / "reference.csv"]
        },
    }
    (args.output_dir / "protocol.json").write_text(json.dumps(report, indent=2) + "\n")
    report["apollo"] = run_apollo(args.apollo_root, args.output_dir, planar)
    report["zhongji"] = run_zhongji(
        args.zhongji_root, args.output_dir, planar, args.eskf_runner, commands,
    )
    report["output_sha256"] = {
        str(p.relative_to(args.output_dir)): digest(p)
        for p in sorted(args.output_dir.rglob("*.csv"))
    }
    (args.output_dir / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"apollo": {name: r["paired_samples"] for name, r in report["apollo"].items()},
                      "zhongji": report["zhongji"]}, indent=2))


if __name__ == "__main__":
    main()
