#!/usr/bin/env python3
"""Create diagnostic plots and time-series exports from a bounded DR replay."""

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from record_dr import interpolate_rows, local_enu, quat_yaw


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def numeric_rows(rows, keys):
    return [row | {key: float(row[key]) for key in keys} for row in rows]


def quaternion_yaw(row):
    return quat_yaw(row)


def aligned_states(trajectory, reference):
    ref_times = [row["time"] for row in reference]
    valid = [row for row in trajectory if int(row["valid"]) == 1]
    first = valid[0]
    initial_ref = interpolate_rows(reference, ref_times, first["time"], 0.05)
    if initial_ref is None:
        raise ValueError("first valid estimator state is not bracketed by RTK")
    yaw_offset = math.radians(90.0 - initial_ref["heading_deg"]) - quaternion_yaw(
        first
    )
    cosine, sine = math.cos(yaw_offset), math.sin(yaw_offset)
    rotated_x = cosine * first["px"] - sine * first["py"]
    rotated_y = sine * first["px"] + cosine * first["py"]
    origin = (
        initial_ref["latitude"],
        initial_ref["longitude"],
        initial_ref["height_msl"] + initial_ref["undulation"],
    )
    initial_east, initial_north = local_enu(origin, initial_ref)
    translation_east = initial_east - rotated_x
    translation_north = initial_north - rotated_y
    aligned = []
    previous = None
    for state in trajectory:
        if int(state["valid"]) != 1:
            continue
        ref = interpolate_rows(reference, ref_times, state["time"], 0.05)
        if ref is None:
            continue
        east = cosine * state["px"] - sine * state["py"] + translation_east
        north = sine * state["px"] + cosine * state["py"] + translation_north
        yaw = quaternion_yaw(state) + yaw_offset
        yaw_error = math.degrees(
            (yaw - math.radians(90.0 - ref["heading_deg"]) + math.pi)
            % (2.0 * math.pi)
            - math.pi
        )
        rtk_east, rtk_north = local_enu(origin, ref)
        yaw_global = math.radians(90.0 - ref["heading_deg"])
        error_east, error_north = east - rtk_east, north - rtk_north
        elapsed = state["time"] - first["time"]
        dt = state["time"] - previous["time"] if previous else 0.0
        row = {
            "time": state["time"],
            "elapsed_s": elapsed,
            "east_m": east,
            "north_m": north,
            "rtk_east_m": rtk_east,
            "rtk_north_m": rtk_north,
            "error_east_m": error_east,
            "error_north_m": error_north,
            "horizontal_error_m": math.hypot(error_east, error_north),
            "longitudinal_error_m": error_east * math.cos(yaw_global)
            + error_north * math.sin(yaw_global),
            "lateral_error_m": -error_east * math.sin(yaw_global)
            + error_north * math.cos(yaw_global),
            "aligned_yaw_deg": math.degrees(yaw),
            "rtk_heading_deg": ref["heading_deg"],
            "yaw_error_deg": yaw_error,
            "vx_local_mps": state["vx"],
            "vy_local_mps": state["vy"],
            "vz_local_mps": state["vz"],
            "gyro_bias_x": state["gyro_bias_x"],
            "gyro_bias_y": state["gyro_bias_y"],
            "gyro_bias_z": state["gyro_bias_z"],
            "accel_bias_x": state["accel_bias_x"],
            "accel_bias_y": state["accel_bias_y"],
            "accel_bias_z": state["accel_bias_z"],
            "covariance_diag": [state[f"cov_{i}"] for i in range(15)],
            "integrity_reason": state["integrity_reason"],
            "precision_reason": state["precision_reason"],
            "imu_accepted": int(state["imu_accepted"]),
            "imu_rejected": int(state["imu_rejected"]),
            "wheel_accepted": int(state["wheel_accepted"]),
            "wheel_rejected": int(state["wheel_rejected"]),
            "state_dx_m": east - previous["east_m"] if previous else 0.0,
            "state_dy_m": north - previous["north_m"] if previous else 0.0,
            "state_dyaw_deg": (
                math.degrees(
                    (yaw - previous["yaw_rad"] + math.pi) % (2 * math.pi) - math.pi
                )
                if previous
                else 0.0
            ),
            "dt_s": dt,
        }
        aligned.append(row)
        previous = row | {"yaw_rad": yaw}
    return aligned


def make_wheel_track(events, reference, origin):
    refs = [row for row in reference]
    ref_times = [row["time"] for row in refs]
    wheel = [
        {"time": float(row["measurement_time"]), "speed": float(row["speed_mps"])}
        for row in events
        if row["kind"] == "W"
    ]
    output = []
    east = north = distance = 0.0
    previous = None
    for sample in wheel:
        ref = interpolate_rows(refs, ref_times, sample["time"], 0.05)
        if ref is None:
            continue
        if previous is not None:
            dt = sample["time"] - previous["time"]
            previous_ref = interpolate_rows(refs, ref_times, previous["time"], 0.05)
            if previous_ref is not None and dt > 0:
                yaw_a = math.radians(90.0 - previous_ref["heading_deg"])
                yaw_b = math.radians(90.0 - ref["heading_deg"])
                yaw_mid = yaw_a + ((yaw_b - yaw_a + math.pi) % (2 * math.pi) - math.pi) / 2
                ds = 0.5 * (previous["speed"] + sample["speed"]) * dt
                east += ds * math.cos(yaw_mid)
                north += ds * math.sin(yaw_mid)
                distance += ds
        output.append(
            {
                "time": sample["time"],
                "elapsed_s": sample["time"] - wheel[0]["time"],
                "east_m": east,
                "north_m": north,
                "integrated_distance_m": distance,
                "speed_mps": sample["speed"],
            }
        )
        previous = sample
    return output


def write_timeseries(path, tracks):
    columns = [
        "configuration",
        "time",
        "elapsed_s",
        "east_m",
        "north_m",
        "rtk_east_m",
        "rtk_north_m",
        "error_east_m",
        "error_north_m",
        "horizontal_error_m",
        "longitudinal_error_m",
        "lateral_error_m",
        "aligned_yaw_deg",
        "rtk_heading_deg",
        "yaw_error_deg",
        "vx_local_mps",
        "vy_local_mps",
        "vz_local_mps",
        "gyro_bias_x",
        "gyro_bias_y",
        "gyro_bias_z",
        "accel_bias_x",
        "accel_bias_y",
        "accel_bias_z",
        "state_dx_m",
        "state_dy_m",
        "state_dyaw_deg",
        "dt_s",
        "integrity_reason",
        "precision_reason",
        "imu_accepted",
        "imu_rejected",
        "wheel_accepted",
        "wheel_rejected",
        *[f"cov_{i}" for i in range(15)],
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=columns)
        writer.writeheader()
        for name, rows in tracks.items():
            for row in rows:
                writer.writerow(
                    {
                        **{
                            key: value
                            for key, value in row.items()
                            if key in columns
                        },
                        "configuration": name,
                        **{
                            f"cov_{i}": row["covariance_diag"][i]
                            for i in range(15)
                        },
                    }
                )


def interpolate_scalar(rows, times, timestamp, key):
    index = bisect.bisect_left(times, timestamp)
    if index < len(times) and times[index] == timestamp:
        return rows[index][key]
    if index == 0 or index == len(times):
        return None
    left, right = rows[index - 1], rows[index]
    ratio = (timestamp - times[index - 1]) / (times[index] - times[index - 1])
    return left[key] + ratio * (right[key] - left[key])


def pearson_correlation(xs, ys):
    if len(xs) < 3:
        return None
    mean_x, mean_y = statistics.mean(xs), statistics.mean(ys)
    centered_x = [value - mean_x for value in xs]
    centered_y = [value - mean_y for value in ys]
    denominator = math.sqrt(
        sum(value * value for value in centered_x)
        * sum(value * value for value in centered_y)
    )
    return (
        sum(x * y for x, y in zip(centered_x, centered_y)) / denominator
        if denominator > 0
        else None
    )


def imudirection_check(events):
    wheel = sorted(
        (
            {"time": float(row["measurement_time"]), "speed": float(row["speed_mps"])}
            for row in events
            if row["kind"] == "W"
        ),
        key=lambda row: row["time"],
    )
    wheel_times = [row["time"] for row in wheel]
    imu = [
        {
            "time": float(row["measurement_time"]),
            "ax": float(row["ax"]),
            "ay": float(row["ay"]),
            "az": float(row["az"]),
        }
        for row in events
        if row["kind"] == "I"
    ]
    pairs = {"ax": [], "ay": [], "az": []}
    for row in imu:
        before = interpolate_scalar(wheel, wheel_times, row["time"] - 0.1, "speed")
        after = interpolate_scalar(wheel, wheel_times, row["time"] + 0.1, "speed")
        speed = interpolate_scalar(wheel, wheel_times, row["time"], "speed")
        if before is None or after is None or speed is None or speed < 0.1:
            continue
        longitudinal_acceleration = (after - before) / 0.2
        for axis in pairs:
            pairs[axis].append((row[axis], longitudinal_acceleration))
    return {
        "method": (
            "correlate raw IMU acceleration axes with the central 0.2 s derivative "
            "of signed chassis speed, restricted to forward speed >= 0.1 m/s"
        ),
        "samples": len(pairs["ax"]),
        "pearson_correlation": {
            axis: pearson_correlation(
                [pair[0] for pair in values], [pair[1] for pair in values]
            )
            for axis, values in pairs.items()
        },
        "interpretation_limit": (
            "This is only a within-record axis-consistency diagnostic; it cannot "
            "prove IMU mounting orientation, scale, timing calibration, or "
            "independent forward direction."
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    source = args.input_dir
    reference_raw = read_csv(source / "reference.csv")
    reference = numeric_rows(
        reference_raw,
        (
            "time",
            "latitude",
            "longitude",
            "height_msl",
            "undulation",
            "heading_deg",
            "heading_std_dev",
            "latitude_std_dev",
            "longitude_std_dev",
        ),
    )
    origin_state = next(
        row
        for row in read_csv(source / "trajectory_var_0p01.csv")
        if int(row["valid"]) == 1
    )
    origin_ref = interpolate_rows(
        reference, [row["time"] for row in reference], float(origin_state["time"]), 0.05
    )
    if origin_ref is None:
        raise ValueError("initial DR state is not bracketed by RTK")
    origin = (
        origin_ref["latitude"],
        origin_ref["longitude"],
        origin_ref["height_msl"] + origin_ref["undulation"],
    )
    rtk = []
    ref_times = [row["time"] for row in reference]
    for ref in reference:
        east, north = local_enu(origin, ref)
        rtk.append(
            {"time": ref["time"], "elapsed_s": ref["time"] - reference[0]["time"],
             "east_m": east, "north_m": north}
        )
    events = read_csv(source / "events.csv")
    wheel = make_wheel_track(events, reference, origin)
    tracks = {}
    for label, suffix in (
        ("wheel+imu_nhc_off", ""),
        ("wheel+imu_nhc_on", "_nhc"),
    ):
        states_raw = read_csv(source / f"trajectory_var_0p01{suffix}.csv")
        states = numeric_rows(
            states_raw,
            (
                "time",
                "px",
                "py",
                "pz",
                "vx",
                "vy",
                "vz",
                "gyro_bias_x",
                "gyro_bias_y",
                "gyro_bias_z",
                "accel_bias_x",
                "accel_bias_y",
                "accel_bias_z",
                "qw",
                "qx",
                "qy",
                "qz",
                *[f"cov_{i}" for i in range(15)],
            ),
        )
        tracks[label] = aligned_states(states, reference)
    write_timeseries(args.output_dir / "state_diagnostics.csv", tracks)
    with (args.output_dir / "wheel_heading_integral.csv").open(
        "w", newline="", encoding="utf-8"
    ) as output:
        writer = csv.DictWriter(
            output,
            fieldnames=(
                "time",
                "elapsed_s",
                "east_m",
                "north_m",
                "integrated_distance_m",
                "speed_mps",
            ),
        )
        writer.writeheader()
        writer.writerows(wheel)

    fig, axes = plt.subplots(2, 2, figsize=(15, 11), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot([r["east_m"] for r in rtk], [r["north_m"] for r in rtk],
            color="black", linewidth=2, label="RTK/INS BestPose")
    ax.plot([r["east_m"] for r in wheel], [r["north_m"] for r in wheel],
            color="tab:orange", linewidth=1.5, label="wheel speed × RTK heading")
    for label, color in (("wheel+imu_nhc_off", "tab:red"),
                         ("wheel+imu_nhc_on", "tab:blue")):
        rows = tracks[label]
        ax.plot([r["east_m"] for r in rows], [r["north_m"] for r in rows],
                linewidth=1.2, alpha=0.85, color=color, label=label)
    ax.set_title("Trajectory overlay (single initial alignment)")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    for label, color in (("wheel+imu_nhc_off", "tab:red"),
                         ("wheel+imu_nhc_on", "tab:blue")):
        rows = tracks[label]
        ax.plot([r["elapsed_s"] for r in rows],
                [r["horizontal_error_m"] for r in rows],
                linewidth=1, color=color, label=label)
    ax.set_title("Horizontal position residual")
    ax.set_xlabel("Elapsed since initialization [s]")
    ax.set_ylabel("Error [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1, 0]
    for label, color in (("wheel+imu_nhc_off", "tab:red"),
                         ("wheel+imu_nhc_on", "tab:blue")):
        rows = tracks[label]
        ax.plot([r["elapsed_s"] for r in rows], [r["yaw_error_deg"] for r in rows],
                linewidth=1, color=color, label=label)
    ax.set_title("Estimator yaw minus GNSS dual-antenna heading")
    ax.set_xlabel("Elapsed since initialization [s]")
    ax.set_ylabel("Heading difference [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1, 1]
    ax.plot([r["elapsed_s"] for r in wheel], [r["speed_mps"] for r in wheel],
            color="tab:orange", linewidth=1, label="chassis speed")
    ax2 = ax.twinx()
    for label, color in (("wheel+imu_nhc_off", "tab:red"),
                         ("wheel+imu_nhc_on", "tab:blue")):
        rows = tracks[label]
        ax2.plot([r["elapsed_s"] for r in rows],
                 [r["precision_reason"] == "NONE" for r in rows],
                 linewidth=0.8, alpha=0.75, color=color, label=f"{label} precision pass")
    ax.set_title("Forward speed and precision-policy status")
    ax.set_xlabel("Elapsed time [s]")
    ax.set_ylabel("Wheel speed [m/s]")
    ax2.set_ylabel("Precision qualified")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left", fontsize=8)
    ax2.legend(loc="upper right", fontsize=8)
    fig.suptitle("1010 single-segment DR baseline — same-record diagnostic only")
    fig.savefig(args.output_dir / "dr_rtk_comparison.png", dpi=160)
    plt.close(fig)

    orientation_statistics = {}
    for label, rows in tracks.items():
        signed = [row["yaw_error_deg"] for row in rows]
        absolute = sorted(abs(value) for value in signed)
        orientation_statistics[label] = {
            "first_heading_error_deg": signed[0],
            "last_heading_error_deg": signed[-1],
            "median_absolute_heading_error_deg": statistics.median(absolute),
            "p95_absolute_heading_error_deg": absolute[
                min(len(absolute) - 1, math.ceil(0.95 * len(absolute)) - 1)
            ],
            "maximum_absolute_heading_error_deg": max(absolute),
        }
    baseline_summary = json.loads(
        (source / "evaluation_var_0p01_final/evaluation_summary.json").read_text(
            encoding="utf-8"
        )
    )
    nhc_summary = json.loads(
        (source / "evaluation_var_0p01_nhc_final/evaluation_summary.json").read_text(
            encoding="utf-8"
        )
    )
    common_config = (
        "wheel_speed_variance_m2ps2",
        "zero_velocity_update_enabled",
        "stationary_duration_seconds",
    )
    common_inputs = (
        "source_record_sha256",
        "events_csv_sha256",
        "reference_sha256",
    )
    ablation_check = {
        "same_common_config": all(
            baseline_summary["estimator_config"][key]
            == nhc_summary["estimator_config"][key]
            for key in common_config
        ),
        "same_input_hashes": all(
            baseline_summary["input_artifacts"][key]
            == nhc_summary["input_artifacts"][key]
            for key in common_inputs
        ),
        "only_nhc_config_differs": (
            baseline_summary["estimator_config"]["nonholonomic_constraint_enabled"]
            is False
            and nhc_summary["estimator_config"][
                "nonholonomic_constraint_enabled"
            ]
            is True
            and baseline_summary["estimator_config"][
                "nonholonomic_variance_m2ps2"
            ]
            is None
            and nhc_summary["estimator_config"][
                "nonholonomic_variance_m2ps2"
            ]
            == 0.01
        ),
    }
    ablation_check["passed"] = all(ablation_check.values())
    summary = {
        "plot": "dr_rtk_comparison.png",
        "time_series_csv": "state_diagnostics.csv",
        "wheel_heading_integral_csv": "wheel_heading_integral.csv",
        "wheel_track_definition": (
            "integrated chassis speed projected along the same-record GNSS "
            "dual-antenna heading; not an independent wheel-only odometry"
        ),
        "orientation_check": (
            "each DR trajectory is aligned once in translation/yaw at its first "
            "valid state; subsequent yaw errors compare propagated estimator yaw "
            "with same-record dual-antenna GNSS heading"
        ),
        "limits": [
            "same-record RTK/INS and heading are correlated with initial alignment",
            "single forward-only segment; no turning, reverse, slope or slip evidence",
            "GNSS antenna/base/IMU lever arms are not surveyed",
            "covariance fields are estimator-reported and not calibrated",
        ],
        "wheel_integrated_distance_m": wheel[-1]["integrated_distance_m"],
        "rtk_track_distance_m": math.hypot(
            rtk[-1]["east_m"] - rtk[0]["east_m"],
            rtk[-1]["north_m"] - rtk[0]["north_m"],
        ),
        "track_counts": {name: len(rows) for name, rows in tracks.items()},
        "orientation_error_statistics": orientation_statistics,
        "imu_forward_acceleration_axis_check": imudirection_check(events),
        "nhc_ablation_single_variable_check": ablation_check,
    }
    (args.output_dir / "diagnostic_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
