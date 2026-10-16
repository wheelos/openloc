#!/usr/bin/env python3
"""Conditional Apollo ESKF replay: late quiet start and offline full-route seed."""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

from compare_planar_replay import digest, export_inputs, require, write_csv
from record_dr import quat_yaw
from wheel_odometry import Series, eligible_intervals, load, statistics, wrap


def series(rows, time_key, keys, offset=0):
    return Series(
        [float(r[time_key]) + offset for r in rows],
        [[float(r[k]) for k in keys] for r in rows], 0.05,
    )


def export_events(chassis, imu, segment, path):
    wheels = [
        r for r in chassis
        if segment["start"] <= float(r["header_time"]) <= segment["end"]
    ]
    require(wheels, "No wheel rows in selected continuous segment")
    start = float(wheels[0]["header_time"])
    imus = [
        r for r in imu
        if start <= float(r["measurement_time"]) + 315964782 <= segment["end"]
    ]
    require(imus, "No complete IMU rows in selected segment")
    rows = []
    for index, row in enumerate(wheels):
        rows.append({
            "kind": "W", "measurement_time": float(row["header_time"]),
            "receive_time": float(row["header_time"]), "sequence": index + 1,
            **{k: "" for k in ("ax", "ay", "az", "gx", "gy", "gz")},
            "speed_mps": float(row["speed"]),
        })
    for index, row in enumerate(imus):
        time = float(row["measurement_time"]) + 315964782
        rows.append({
            "kind": "I", "measurement_time": time, "receive_time": time,
            "sequence": index + 1,
            **{k: float(row[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz")},
            "speed_mps": "",
        })
    rows.sort(key=lambda r: (r["measurement_time"], r["kind"] == "I"))
    require(
        all(b["measurement_time"] - a["measurement_time"] <= 0.05 for stream in ("W", "I")
            for a, b in zip([r for r in rows if r["kind"] == stream],
                            [r for r in rows if r["kind"] == stream][1:])),
        "Selected sensor stream spans a gap",
    )
    write_csv(path, list(rows[0]), rows)
    return rows


def numeric_reference(path):
    unique = []
    for row in load(path):
        value = [float(row[k]) for k in ("px", "py", "heading")]
        time = float(row["measurement_time"])
        if unique and unique[-1][0] == time:
            require(unique[-1][1] == value, "Conflicting duplicate reference")
        else:
            unique.append((time, value))
    return Series([t for t, _ in unique], [v for _, v in unique], 0.05)


def make_seed(quiet_state, speed, path):
    fields = ["qw", "qx", "qy", "qz", "bgx", "bgy", "bgz", "bax", "bay", "baz", "speed"]
    fields += [f"cov_{i}" for i in range(15)]
    values = [float(quiet_state[k]) for k in ("qw", "qx", "qy", "qz")]
    values += [float(quiet_state[f"gyro_bias_{axis}"]) for axis in "xyz"]
    values += [float(quiet_state[f"accel_bias_{axis}"]) for axis in "xyz"]
    values += [speed] + [float(quiet_state[f"cov_{i}"]) for i in range(15)]
    write_csv(path, fields, [dict(zip(fields, values))])


def compare(states, events, reference, output, planar_runner, run):
    valid = [r for r in states if r["valid"] == "1"]
    require(valid, "No initialized ESKF states")
    speed_rows = [r for r in events if r["kind"] == "W"]
    imu_rows = [r for r in events if r["kind"] == "I"]
    speed = series(speed_rows, "measurement_time", ["speed_mps"])
    gyro = series(imu_rows, "measurement_time", ["gz"])
    start, finish = float(valid[0]["time"]), float(valid[-1]["time"])
    times = [float(r["time"]) for r in valid]
    grid_source = Series(times, [[0] for _ in times], 0.05)
    intervals = []
    for _, _, grid, _ in eligible_intervals(grid_source, gyro, 0.05):
        inputs = [(speed.at(t), gyro.at(t)) for t in grid]
        require(all(a is not None and b is not None for a, b in inputs), "Unbracketed common inputs")
        intervals.append((0, quat_yaw(valid[0]), grid, inputs))
    require(intervals, "Insufficient initialized coverage")
    input_path = output / "planar_inputs.csv"
    export_inputs(input_path, intervals)
    candidates = {"eskf": valid}
    for mode, bias in (("planar_raw", 0), ("planar_fixed", float(valid[0]["gyro_bias_z"]))):
        path = output / f"{mode}.csv"
        run([str(planar_runner), "--input", str(input_path), "--output", str(path), "--gyro-bias", str(bias)])
        candidates[mode] = load(path)
    require(
        all([float(r["time"]) for r in rows] == times for rows in candidates.values()),
        "Candidate time pairing mismatch",
    )
    origin = reference.at(start, 2)
    require(origin is not None, "No common reference at first initialized time")
    yaw_offset = origin[2] - quat_yaw(valid[0])
    c, s = math.cos(yaw_offset), math.sin(yaw_offset)
    first_x, first_y = float(valid[0]["px"]), float(valid[0]["py"])
    rows, unscored = [], []
    for index, time in enumerate(times):
        ref = reference.at(time, 2)
        if ref is None:
            unscored.append({"time": time, "reason": "reference_gap_or_unbracketed"})
            continue
        row = {"time": time, "elapsed_s": time - start,
               "wheel_distance_m": float(candidates["planar_raw"][index]["wheel_distance_m"])}
        for mode, trajectory in candidates.items():
            actual = trajectory[index]
            if mode == "eskf":
                x, y = float(actual["px"]) - first_x, float(actual["py"]) - first_y
                yaw = quat_yaw(actual)
            else:
                x, y = float(actual["local_x"]), float(actual["local_y"])
                yaw = float(actual["yaw"])
            row[f"{mode}_error_m"] = math.hypot(
                c * x - s * y - (ref[0] - origin[0]),
                s * x + c * y - (ref[1] - origin[1]),
            )
            row[f"{mode}_heading_error_deg"] = math.degrees(wrap(yaw + yaw_offset - ref[2]))
        row["eskf_update_reason"] = valid[index]["update_reason"]
        row["eskf_precision_reason"] = valid[index]["precision_reason"]
        rows.append(row)
    require(rows, "No common scored rows")
    write_csv(output / "paired_scores.csv", list(rows[0]), rows)
    yaw_rates = [r["gz"] for r in imu_rows if start <= r["measurement_time"] <= finish]
    return {
        "first_valid_time": start, "integrated_duration_s": finish - start,
        "initialized_states": len(valid), "scored_states": len(rows),
        "unscored": unscored, "wheel_distance_m": rows[-1]["wheel_distance_m"],
        "max_abs_raw_yaw_rate_radps": max(abs(rate) for rate in yaw_rates),
        "reference_heading_change_deg": math.degrees(wrap(reference.at(rows[-1]["time"], 2)[2] - origin[2])),
        "precision_qualified_states": sum(r["precision_reason"] == "NONE" for r in valid),
        "results": {
            mode: {
                "horizontal_error_m": statistics([r[f"{mode}_error_m"] for r in rows]),
                "endpoint_error_m": rows[-1][f"{mode}_error_m"],
                "endpoint_heading_error_deg": rows[-1][f"{mode}_heading_error_deg"],
                "distance_checkpoints": [
                    {"requested_m": threshold, **next(r for r in rows if r["wheel_distance_m"] >= threshold)}
                    for threshold in (50, 100, 200, 300, 400, 500)
                    if rows[-1]["wheel_distance_m"] >= threshold
                ],
            } for mode in candidates
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "output-dir", "eskf-runner", "planar-runner"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    full_root = args.root / "full_imu_20261014"
    manifest = json.loads((full_root / "manifest.json").read_text())
    audit = json.loads((args.root / "audit.json").read_text())
    commands = []

    def run(command):
        commands.append(command)
        (args.output_dir / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        subprocess.run(command, check=True)

    protocol = {
        "invocation": sys.argv, "classification": "Conditional same-record diagnostic, not accuracy or online qualification.",
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "source_sha256": {
            str(p): digest(p) for p in [
                Path(__file__), Path(__file__).with_name("local_estimator_replay.cc"),
                Path(__file__).with_name("compare_planar_replay.py"),
                Path(__file__).with_name("wheel_odometry.py"), Path(__file__).with_name("record_dr.py"),
                Path("src/localization/candidates/local/local_estimator.cc"),
                Path("src/localization/candidates/local/local_estimator.h"),
                Path("src/localization/candidates/local/planar_wheel_dr.cc"), Path("BUILD"),
            ]
        },
        "binary_sha256": {str(p): digest(p) for p in (args.eskf_runner, args.planar_runner)},
        "inputs_sha256": {},
        "config": {"stationary_duration_s": 1, "stationary_samples": 50, "wheel_variance": 0.01,
                   "nhc": False, "zupt": False, "bias_feedback": "all", "other_parameters": "unchanged runner defaults"},
        "limits": [
            "FLU interpretation from matching local schema; actual calibration/reference/lever arms unresolved.",
            "Recorded speed is forward-only; demo parking labels remain diagnostic exception.",
            "Events sorted by measurement time; receive_time equals measurement_time for offline replay, not arrival-time validation.",
            "A: causal 1s quiet initialization, only subsequent motion is compared.",
            "B: future A-startup tilt/gyro bias and initial chassis speed explicitly seed the first IMU; noncausal offline experiment.",
            "B: attitude from later quiet period assumes unchanged mounting/gravity attitude; no independent ground truth.",
            "Pose supplies one yaw/translation gauge per compared interval and offline scoring only.",
        ],
    }
    for name in ("sensor_rgb", "demo_3.5"):
        for suffix in ("chassis", "pose"):
            path = args.root / f"{name}_{suffix}.csv"
            require(digest(path) == audit["artifact_sha256"][path.name], "Frozen CSV mismatch")
            protocol["inputs_sha256"][str(path)] = digest(path)
        path = full_root / f"{name}_imu.csv"
        require(digest(path) == manifest[name]["full_csv_sha256"], "Completed gyro hash mismatch")
        protocol["inputs_sha256"][str(path)] = digest(path)
    (args.output_dir / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    report = {}
    for name in ("sensor_rgb", "demo_3.5"):
        destination = args.output_dir / name
        destination.mkdir()
        chassis = load(args.root / f"{name}_chassis.csv")
        imu = load(full_root / f"{name}_imu.csv")
        reference = numeric_reference(args.root / f"{name}_pose.csv")
        baseline = json.loads((args.root / "wheel_odometry_distance_20261014" / name / "summary.json").read_text())
        segments = baseline["wheel_raw_gyro"]["segments"]
        quiet = manifest[name]["quiet_gate_1s_diagnostic_only"][0]
        quiet_segment = next(s for s in segments if s["start"] <= quiet["start"] < quiet["qualified_time"] <= s["end"])
        late = destination / "A_late_quiet"
        late.mkdir()
        event_path = late / "events.csv"
        events = export_events(chassis, imu, quiet_segment, event_path)
        trajectory = late / "eskf.csv"
        common_options = ["--wheel-variance", "0.01", "--use-nhc", "false", "--diagnostic-stationary-duration", "1"]
        run([str(args.eskf_runner), "--events", str(event_path), "--trajectory", str(trajectory), *common_options])
        states = load(trajectory)
        late_report = compare(states, events, reference, late, args.planar_runner, run)
        quiet_state = next(r for r in states if r["valid"] == "1")
        main_segment = segments[baseline["wheel_raw_gyro"]["longest_segment"]]
        whole = destination / "B_offline_main"
        whole.mkdir()
        event_path = whole / "events.csv"
        events = export_events(chassis, imu, main_segment, event_path)
        first_imu = next(r for r in events if r["kind"] == "I")
        speed_series = series(chassis, "header_time", ["speed"])
        seed_speed = speed_series.at(first_imu["measurement_time"])
        require(seed_speed is not None, "Unbracketed initial speed")
        seed_path = whole / "seed.csv"
        make_seed(quiet_state, seed_speed[0], seed_path)
        trajectory = whole / "eskf.csv"
        run([str(args.eskf_runner), "--events", str(event_path), "--trajectory", str(trajectory),
             *common_options, "--offline-diagnostic-seed", str(seed_path)])
        full_report = compare(load(trajectory), events, reference, whole, args.planar_runner, run)
        report[name] = {"A_late_quiet": late_report, "B_offline_main": full_report,
                        "offline_seed_source_time": float(quiet_state["time"]), "offline_seed_speed_mps": seed_speed[0]}
        (args.output_dir / "results.json").write_text(json.dumps(report, indent=2) + "\n")
        print(name, json.dumps(report[name], indent=2), flush=True)
    protocol["results"] = report
    protocol["commands"] = commands
    protocol["output_sha256"] = {
        str(p.relative_to(args.output_dir)): digest(p) for p in sorted(args.output_dir.rglob("*.csv"))
    }
    (args.output_dir / "summary.json").write_text(json.dumps(protocol, indent=2) + "\n")


if __name__ == "__main__":
    main()
