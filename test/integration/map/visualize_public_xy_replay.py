#!/usr/bin/env python3
"""Render diagnostic plots from public_xy_dataset_replay CSV evidence."""

import argparse
import csv
import datetime as dt
import hashlib
import html
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyproj import Transformer


def read_csv(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_gga(path, capture, origin):
    capture_date = dt.datetime.strptime(capture[:8], "%Y%m%d").date()
    transformer = Transformer.from_crs(
        "EPSG:4326", f"EPSG:{origin['epsg']}", always_xy=True
    )
    valid = []
    counts = Counter()
    previous_second = None
    day_offset = 0

    for line_number, line in enumerate(path.read_text().splitlines(), start=1):
        counts["lines"] += 1
        if not line.strip():
            counts["blank"] += 1
            continue
        try:
            sentence, checksum_text = line.strip().split("*", 1)
            checksum = 0
            for character in sentence[1:]:
                checksum ^= ord(character)
            if checksum != int(checksum_text[:2], 16):
                counts["bad_checksum"] += 1
                continue
            fields = sentence.split(",")
            if fields[0] not in ("$GNGGA", "$GPGGA") or len(fields) < 10:
                counts["unsupported_or_short"] += 1
                continue
            if not fields[1] or not fields[6]:
                counts["empty_timestamp_or_quality"] += 1
                continue
            time_text = fields[1]
            hour = int(time_text[0:2])
            minute = int(time_text[2:4])
            second = float(time_text[4:])
            second_of_day = hour * 3600 + minute * 60 + second
            if previous_second is not None and second_of_day < previous_second - 43200:
                day_offset += 1
            previous_second = second_of_day
            timestamp = (
                dt.datetime.combine(
                    capture_date + dt.timedelta(days=day_offset),
                    dt.time(hour, minute, int(second)),
                    tzinfo=dt.timezone.utc,
                ).timestamp()
                + second % 1
            )
            quality = int(fields[6])
            if quality == 0:
                counts["no_fix_quality_0"] += 1
                continue
            if any(not fields[index] for index in (2, 3, 4, 5, 7, 8)):
                counts["empty_required_fields"] += 1
                continue

            def coordinate(value, hemisphere):
                encoded = float(value)
                degrees = int(encoded // 100)
                decimal = degrees + (encoded - degrees * 100) / 60
                return -decimal if hemisphere in ("S", "W") else decimal

            latitude = coordinate(fields[2], fields[3])
            longitude = coordinate(fields[4], fields[5])
            easting, northing = transformer.transform(longitude, latitude)
            valid.append(
                {
                    "timestamp": timestamp,
                    "x": easting - origin["easting_m"],
                    "y": northing - origin["northing_m"],
                    "quality": quality,
                    "satellites": int(fields[7]),
                    "hdop": float(fields[8]),
                    "line": line_number,
                }
            )
            counts["valid_position"] += 1
            counts[f"quality_{quality}"] += 1
        except (ValueError, IndexError):
            counts["malformed"] += 1

    if not valid:
        raise ValueError(f"no valid GGA fixes found in {path}")
    return valid, counts


def quaternion_slerp(first, second, amount):
    first = np.asarray(first, dtype=float)
    second = np.asarray(second, dtype=float)
    dot = float(np.dot(first, second))
    if dot < 0:
        second = -second
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        result = first + amount * (second - first)
        return result / np.linalg.norm(result)
    angle = math.acos(dot)
    denominator = math.sin(angle)
    return (
        math.sin((1.0 - amount) * angle) / denominator * first
        + math.sin(amount * angle) / denominator * second
    )


def rotate_vector_wxyz(quaternion, vector):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    return np.array(
        [
            (1 - 2 * (y * y + z * z)) * vx
            + 2 * (x * y - z * w) * vy
            + 2 * (x * z + y * w) * vz,
            2 * (x * y + z * w) * vx
            + (1 - 2 * (x * x + z * z)) * vy
            + 2 * (y * z - x * w) * vz,
            2 * (x * z - y * w) * vx
            + 2 * (y * z + x * w) * vy
            + (1 - 2 * (x * x + y * y)) * vz,
        ]
    )


def interpolate_antenna(odom, timestamps, lever):
    odom_times = np.asarray([int(row["timestamp_ns"]) for row in odom], dtype=np.int64)
    odom_position = np.asarray(
        [[float(row[key]) for key in ("px", "py", "pz")] for row in odom]
    )
    odom_quaternion = np.asarray(
        [[float(row[key]) for key in ("qw", "qx", "qy", "qz")] for row in odom]
    )
    output = np.full((len(timestamps), 2), np.nan)
    valid_indices = []
    for index, timestamp in enumerate(timestamps):
        right = int(np.searchsorted(odom_times, timestamp))
        if right == 0 or right == len(odom_times):
            continue
        left = right - 1
        span = int(odom_times[right] - odom_times[left])
        if span <= 0:
            raise ValueError("ODOM timestamps are not strictly increasing")
        amount = (int(timestamp) - int(odom_times[left])) / span
        position = (
            (1.0 - amount) * odom_position[left] + amount * odom_position[right]
        )
        orientation = quaternion_slerp(
            odom_quaternion[left], odom_quaternion[right], amount
        )
        antenna = position + rotate_vector_wxyz(orientation, lever)
        output[index] = antenna[:2]
        valid_indices.append(index)
    if len(valid_indices) == 0:
        raise ValueError("no GNSS measurements could be bracketed by ODOM")
    return output, np.asarray(valid_indices, dtype=int)


def fit_se2(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    left, _, right = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    sign = np.eye(2)
    sign[-1, -1] = np.linalg.det(left @ right)
    rotation = left @ sign @ right
    translation = target_mean - source_mean @ rotation
    return rotation, translation


def residual_stats(residual):
    magnitude = np.linalg.norm(residual, axis=1)
    return {
        "count": int(len(magnitude)),
        "rms_m": float(np.sqrt(np.mean(magnitude**2))),
        "median_m": float(np.median(magnitude)),
        "p95_m": float(np.quantile(magnitude, 0.95)),
        "max_m": float(np.max(magnitude)),
    }


def plot_trajectory(path, events, factors, raw_gga, odom, fixes, lever):
    fig, axis = plt.subplots(figsize=(11, 8))
    if raw_gga:
        quality_groups = sorted({point["quality"] for point in raw_gga})
        colors = {4: "#1f77b4", 2: "#ff7f0e", 5: "#9467bd"}
        for quality in quality_groups:
            points = [point for point in raw_gga if point["quality"] == quality]
            axis.scatter(
                [point["x"] for point in points],
                [point["y"] for point in points],
                s=8,
                alpha=0.48,
                color=colors.get(quality, "#7f7f7f"),
                label=f"raw GGA quality={quality} ({len(points)})",
                zorder=1,
            )

    fix_times = np.asarray([int(row["timestamp_ns"]) for row in fixes], dtype=np.int64)
    antenna, valid_indices = interpolate_antenna(odom, fix_times, lever)
    fix_xy = np.asarray(
        [[float(row["gx"]), float(row["gy"])] for row in fixes], dtype=float
    )
    fit_indices = valid_indices[
        fix_times[valid_indices] <= fix_times[0] + 60_000_000_000
    ]
    if len(fit_indices) < 3:
        raise ValueError("fewer than three prepared fixes in the initial 60 seconds")
    rotation, translation = fit_se2(antenna[fit_indices], fix_xy[fit_indices])
    aligned_odom = antenna @ rotation + translation
    fit_residual = aligned_odom[fit_indices] - fix_xy[fit_indices]
    post_fit_indices = valid_indices[
        fix_times[valid_indices] > fix_times[0] + 60_000_000_000
    ]
    post_fit_residual = (
        aligned_odom[post_fit_indices] - fix_xy[post_fit_indices]
        if len(post_fit_indices)
        else np.empty((0, 2))
    )
    axis.plot(
        aligned_odom[:, 0],
        aligned_odom[:, 1],
        color="#6f6f6f",
        linewidth=1.6,
        linestyle="--",
        label="ODOM antenna, SE(2) fit on first 60 s (diagnostic only)",
        zorder=2,
    )

    position_factors = [
        factor for factor in factors if factor["source"] != "gnss_recovery"
    ]
    if len(position_factors) != len(fixes):
        raise ValueError("position decision count does not match prepared GNSS fixes")
    factor_by_index = dict(enumerate(position_factors))
    groups = {
        "accepted": [],
        "bootstrap_validated": [],
        "nis_rejected": [],
        "quality_rejected": [],
        "pending": [],
        "other_rejected": [],
    }
    for index, row in enumerate(fixes):
        factor = factor_by_index.get(index)
        if factor is None:
            raise ValueError(f"missing final factor record for fix index {index}")
        if factor["type"] == "GLOBAL_XY_YAW":
            if factor["accepted"] == "1":
                groups["bootstrap_validated"].append(index)
        elif factor["accepted"] == "1":
            groups["accepted"].append(index)
        elif factor["reason"] == "pending_odometry_bracket":
            groups["pending"].append(index)
        elif "innovation exceeds" in factor["reason"]:
            groups["nis_rejected"].append(index)
        elif "fixed_solution_not_stable" in factor["reason"]:
            groups["quality_rejected"].append(index)
        else:
            groups["other_rejected"].append(index)

    styles = {
        "accepted": ("#2ca02c", "o", "accepted factor"),
        "bootstrap_validated": (
            "#17becf",
            "D",
            "single-antenna bootstrap validated (not a graph factor)",
        ),
        "nis_rejected": ("#d62728", "x", "NIS rejected"),
        "quality_rejected": ("#ff7f0e", "+", "fixed stability rejected"),
        "pending": ("#7f7f7f", ".", "pending at end"),
        "other_rejected": ("#9467bd", "1", "other rejection"),
    }
    for name, indices in groups.items():
        if not indices:
            continue
        color, marker, label = styles[name]
        axis.scatter(
            fix_xy[indices, 0],
            fix_xy[indices, 1],
            s=30,
            marker=marker,
            color=color,
            linewidths=1.1,
            label=f"prepared GNSS {label} ({len(indices)})",
            zorder=4,
        )

    odom_events = [row for row in events if row["event"] == "odom"]
    valid_events = [row for row in odom_events if row["global_valid"] == "1"]
    if valid_events:
        axis.plot(
            [float(row["backend_ax"]) for row in valid_events],
            [float(row["backend_ay"]) for row in valid_events],
            color="#17becf",
            linewidth=1.5,
            label="OpenLoc backend antenna",
            zorder=3,
        )
        axis.plot(
            [float(row["published_ax"]) for row in valid_events],
            [float(row["published_ay"]) for row in valid_events],
            color="#e377c2",
            linewidth=1.3,
            linestyle=":",
            label="OpenLoc published antenna",
            zorder=3,
        )

    origin_x = float(fix_xy[0, 0])
    origin_y = float(fix_xy[0, 1])
    axis.scatter([origin_x], [origin_y], marker="*", s=120, color="black", label="GNSS origin")
    axis.set_title(
        "Trajectory comparison — diagnostic only\n"
        "GNSS tracks share one receiver source; not an independent truth reference"
    )
    axis.set_xlabel("Local projected X (m)")
    axis.set_ylabel("Local projected Y (m)")
    axis.axis("equal")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return {
        "initial_60s_odom_fit_in_sample": residual_stats(fit_residual),
        "post_60s_residual_using_initial_fit": (
            residual_stats(post_fit_residual) if len(post_fit_residual) else None
        ),
        "initial_60s_fit_rotation_rad": float(
            math.atan2(rotation[0, 1], rotation[0, 0])
        ),
        "initial_60s_fit_translation_xy_m": translation.tolist(),
    }


def plot_covariance(path, events):
    rows = [
        row
        for row in events
        if row["event"] == "odom" and row["global_valid"] == "1"
    ]
    if not rows:
        raise ValueError("replay contains no globally initialized ODOM events")
    time_seconds = np.asarray(
        [int(row["measurement_ns"]) for row in rows], dtype=np.int64
    )
    time_seconds = (time_seconds - time_seconds[0]) / 1e9
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    components = [
        ("cov_00", "X standard deviation", "m"),
        ("cov_11", "Y standard deviation", "m"),
        ("cov_22", "Z standard deviation (unobserved by XY fixes)", "m"),
        ("cov_33", "Yaw standard deviation", "rad"),
    ]
    for axis, (field, title, unit) in zip(axes, components):
        variance = np.asarray([float(row[field]) for row in rows])
        sigma = np.sqrt(np.maximum(variance, np.finfo(float).tiny))
        axis.plot(time_seconds, sigma, linewidth=1.15, color="#1f77b4")
        axis.set_yscale("log")
        axis.set_ylabel(unit)
        axis.set_title(title, loc="left", fontsize=10)
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel("Seconds from first ODOM sample")
    fig.suptitle(
        "Backend marginal covariance after first accepted position — diagnostic and uncalibrated\n"
        "Log scale; this is not a covariance for held/rate-limited published output",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_factor_diagnostics(path, factors, capture_start_ns):
    categories = {
        "accepted": [],
        "NIS rejected": [],
        "fixed stability rejected": [],
        "pending at end": [],
        "other rejected": [],
    }
    for factor in factors:
        if factor["type"] != "GLOBAL_XY_YAW" and factor["accepted"] == "1":
            categories["accepted"].append(factor)
        elif factor["type"] == "GLOBAL_XY_YAW":
            continue
        elif "innovation exceeds" in factor["reason"]:
            categories["NIS rejected"].append(factor)
        elif "fixed_solution_not_stable" in factor["reason"]:
            categories["fixed stability rejected"].append(factor)
        elif factor["reason"] == "pending_odometry_bracket":
            categories["pending at end"].append(factor)
        else:
            categories["other rejected"].append(factor)

    fig, axis = plt.subplots(figsize=(12, 6))
    colors = {
        "accepted": "#2ca02c",
        "NIS rejected": "#d62728",
        "fixed stability rejected": "#ff7f0e",
        "pending at end": "#7f7f7f",
        "other rejected": "#9467bd",
    }
    markers = {
        "accepted": "o",
        "NIS rejected": "x",
        "fixed stability rejected": "1",
        "pending at end": ".",
        "other rejected": "s",
    }
    for category, rows in categories.items():
        if category in ("fixed stability rejected", "pending at end"):
            y_value = -0.8 if category == "fixed stability rejected" else -1.1
            axis.scatter(
                [
                    (int(row["timestamp_ns"]) - capture_start_ns) / 1e9
                    for row in rows
                ],
                np.full(len(rows), y_value),
                marker=markers[category],
                color=colors[category],
                s=35,
                label=f"{category} ({len(rows)})",
            )
            continue
        if category == "pending at end":
            axis.scatter(
                [
                    (int(row["timestamp_ns"]) - capture_start_ns) / 1e9
                    for row in rows
                ],
                np.full(len(rows), -1.1),
                marker=".",
                color="#7f7f7f",
                s=35,
                label=f"{category} ({len(rows)})",
            )
            continue
        axis.scatter(
            [
                (int(row["timestamp_ns"]) - capture_start_ns) / 1e9
                for row in rows
            ],
            [float(row["nis"]) for row in rows],
            marker=markers[category],
            color=colors[category],
            s=30,
            label=f"{category} ({len(rows)})",
        )
    axis.axhline(
        9.210340371976184,
        color="#d62728",
        linestyle="--",
        linewidth=1.1,
        label="99% chi-square threshold, 2D (9.21)",
    )
    bootstraps = [
        factor
        for factor in factors
        if factor["source"] == "gnss_single_antenna_bootstrap"
        and factor["accepted"] == "1"
    ]
    recoveries = [
        factor
        for factor in factors
        if factor["source"] == "gnss_recovery" and factor["accepted"] == "1"
    ]
    axis.scatter(
        [
            (int(factor["timestamp_ns"]) - capture_start_ns) / 1e9
            for factor in bootstraps
        ],
        np.full(len(bootstraps), -1.35),
        marker="D",
        color="#17becf",
        s=38,
        label=f"single-antenna bootstrap validated ({len(bootstraps)})",
    )
    axis.scatter(
        [
            (int(factor["timestamp_ns"]) - capture_start_ns) / 1e9
            for factor in recoveries
        ],
        np.full(len(recoveries), -1.55),
        marker="P",
        color="#9467bd",
        s=42,
        label=f"validated GNSS recovery re-anchor ({len(recoveries)})",
    )
    axis.set_ylim(bottom=-1.65)
    axis.set_xlabel("Seconds from first ODOM sample")
    axis.set_ylabel("Normalized innovation squared (NIS)")
    axis.set_title(
        "GNSS factor decisions — rejected stability checks are shown below axis\n"
        "Synthetic receiver fields in this runner are not raw GNSS receiver measurements"
    )
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_factor_graph(path, factors, capture_start_ns):
    states = {}
    graph_factors = [
        factor for factor in factors if factor["reason"] != "pending_odometry_bracket"
    ]
    for factor in graph_factors:
        state = int(factor["state_id"])
        timestamp = int(factor["timestamp_ns"])
        states.setdefault(state, []).append(timestamp)
    ordered_states = sorted(states)
    state_x = {
        state: float(np.median(states[state]) - capture_start_ns) / 1e9
        for state in ordered_states
    }
    fig, axis = plt.subplots(figsize=(14, 6))
    for first, second in zip(ordered_states, ordered_states[1:]):
        axis.plot(
            [state_x[first], state_x[second]],
            [0, 0],
            color="#7f7f7f",
            linewidth=1,
            zorder=1,
        )
    axis.scatter(
        [state_x[state] for state in ordered_states],
        np.zeros(len(ordered_states)),
        s=24,
        color="#1f77b4",
        label=f"alignment states inferred from state_id ({len(ordered_states)})",
        zorder=3,
    )
    accepted = [
        factor
        for factor in factors
        if factor["accepted"] == "1" and factor["type"] != "GLOBAL_XY_YAW"
    ]
    rejected = [
        factor
        for factor in factors
        if factor["accepted"] != "1"
        and factor["reason"] != "pending_odometry_bracket"
        and factor["type"] != "GLOBAL_XY_YAW"
    ]
    bootstraps = [
        factor
        for factor in factors
        if factor["source"] == "gnss_single_antenna_bootstrap"
        and factor["accepted"] == "1"
    ]
    recoveries = [
        factor
        for factor in factors
        if factor["source"] == "gnss_recovery" and factor["accepted"] == "1"
    ]
    pending = [
        factor for factor in factors if factor["reason"] == "pending_odometry_bracket"
    ]
    for factor in accepted:
        timestamp = int(factor["timestamp_ns"])
        x = (timestamp - capture_start_ns) / 1e9
        state = int(factor["state_id"])
        axis.plot(
            [state_x[state], x],
            [0.025, 0.96],
            color="#2ca02c",
            alpha=0.55,
            linewidth=0.9,
            zorder=1,
        )
    axis.scatter(
        [(int(factor["timestamp_ns"]) - capture_start_ns) / 1e9 for factor in accepted],
        np.ones(len(accepted)),
        s=28,
        color="#2ca02c",
        marker="o",
        label=f"accepted GNSS graph factors ({len(accepted)})",
        zorder=3,
    )
    axis.scatter(
        [
            (int(factor["timestamp_ns"]) - capture_start_ns) / 1e9
            for factor in bootstraps
        ],
        np.full(len(bootstraps), 0.5),
        s=48,
        color="#17becf",
        marker="D",
        label="validated alignment bootstrap (not an inserted factor)",
        zorder=4,
    )
    axis.scatter(
        [
            (int(factor["timestamp_ns"]) - capture_start_ns) / 1e9
            for factor in recoveries
        ],
        np.full(len(recoveries), 0.5),
        s=52,
        color="#9467bd",
        marker="P",
        label="validated recovery re-anchor (not a graph factor)",
        zorder=4,
    )
    axis.scatter(
        [(int(factor["timestamp_ns"]) - capture_start_ns) / 1e9 for factor in rejected],
        np.full(len(rejected), 1.0),
        s=20,
        color="#d62728",
        marker="x",
        label=f"rejected GNSS attempts (not graph factors) ({len(rejected)})",
        zorder=3,
    )
    axis.scatter(
        [(int(factor["timestamp_ns"]) - capture_start_ns) / 1e9 for factor in pending],
        np.full(len(pending), 1.0),
        s=25,
        color="#7f7f7f",
        marker=".",
        label=f"pending observations ({len(pending)})",
        zorder=3,
    )
    axis.set_yticks([0, 1], ["alignment state", "GNSS observation"])
    axis.set_xlabel("Seconds from first ODOM sample")
    axis.set_title(
        "Alignment factor-graph schematic (not a serialized optimizer graph)\n"
        "States are inferred from factor state_id; only accepted observations connect to states"
    )
    axis.grid(axis="x", alpha=0.2)
    axis.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def match_raw_gga(raw_gga, fixes):
    raw_times = np.asarray([point["timestamp"] for point in raw_gga])
    raw_xy = np.asarray([[point["x"], point["y"]] for point in raw_gga])
    residuals = []
    unmatched = 0
    for fix in fixes:
        timestamp = int(fix["timestamp_ns"]) / 1e9
        right = int(np.searchsorted(raw_times, timestamp))
        candidates = [index for index in (right - 1, right) if 0 <= index < len(raw_times)]
        if not candidates:
            unmatched += 1
            continue
        nearest = min(candidates, key=lambda index: abs(raw_times[index] - timestamp))
        if abs(raw_times[nearest] - timestamp) > 1.01:
            unmatched += 1
            continue
        residuals.append(
            raw_xy[nearest] - np.array([float(fix["gx"]), float(fix["gy"])])
        )
    if not residuals:
        return {"matched": 0, "unmatched": unmatched}
    residuals = np.asarray(residuals)
    return {
        "matched": int(len(residuals)),
        "unmatched": int(unmatched),
        "max_abs_xy_difference_m": float(np.max(np.abs(residuals))),
        "rms_difference_m": float(
            np.sqrt(np.mean(np.sum(residuals**2, axis=1)))
        ),
        "interpretation": "same GNSS source cross-check; not independent accuracy",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence_dir", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--prefix", default="replay_baseline")
    parser.add_argument("--gga", type=Path)
    args = parser.parse_args()

    evidence = args.evidence_dir
    output = args.output_dir
    if output.exists():
        raise FileExistsError(f"refusing to overwrite visualization directory: {output}")
    input_manifest_path = evidence / "inputs/input_manifest.json"
    input_manifest = json.loads(input_manifest_path.read_text())
    events_path = evidence / args.prefix
    factors_path = Path(str(events_path) + ".factors.csv")
    summary_path = Path(str(events_path) + ".summary.csv")
    odom_path = evidence / "inputs/odom.csv"
    fixes_path = evidence / "inputs/fixes.csv"
    lever_path = evidence / "inputs/lever.csv"
    required = (
        input_manifest_path,
        events_path,
        factors_path,
        summary_path,
        odom_path,
        fixes_path,
        lever_path,
    )
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(path)

    events = read_csv(events_path)
    factors_raw = read_csv(factors_path)
    summary_rows = read_csv(summary_path)
    odom = read_csv(odom_path)
    fixes = read_csv(fixes_path)
    lever_rows = read_csv(lever_path)
    if len(summary_rows) != 1 or len(lever_rows) != 1:
        raise ValueError("expected one summary row and one antenna lever row")
    if len(events) != int(summary_rows[0]["output_events"]):
        raise ValueError("replay event count does not match summary")
    if len(odom) != int(summary_rows[0]["odom_epochs"]):
        raise ValueError("ODOM input count does not match replay summary")

    final_factors = {}
    for row in factors_raw:
        final_factors[int(row["record_index"])] = row
    factors = [final_factors[index] for index in sorted(final_factors)]
    position_factors = [
        factor for factor in factors if factor["source"] != "gnss_recovery"
    ]
    if len(position_factors) != len(fixes):
        raise ValueError("position decision count does not match prepared GNSS input")
    if int(summary_rows[0]["fix_submissions"]) != len(fixes):
        raise ValueError("fix submission count does not match prepared GNSS input")
    if any(
        int(factor["timestamp_ns"]) != int(fix["timestamp_ns"])
        for factor, fix in zip(position_factors, fixes)
    ):
        raise ValueError("position decisions are not ordered with GNSS input")

    raw_gga = None
    gga_counts = None
    raw_match = None
    if args.gga:
        origin = input_manifest["gnss_contract"]["projected_origin"]
        raw_gga, gga_counts = parse_gga(args.gga, input_manifest["capture"], origin)
        raw_match = match_raw_gga(raw_gga, fixes)

    output.mkdir(parents=True)
    figure_paths = {
        "trajectory": output / "trajectory_comparison.png",
        "covariance": output / "alignment_covariance.png",
        "factors": output / "factor_innovations.png",
        "graph": output / "factor_graph_schematic.png",
    }
    lever = np.asarray(
        [float(lever_rows[0][key]) for key in ("lx", "ly", "lz")]
    )
    capture_start_ns = min(int(row["measurement_ns"]) for row in events)
    fit_summary = plot_trajectory(
        figure_paths["trajectory"],
        events,
        factors,
        raw_gga,
        odom,
        fixes,
        lever,
    )
    plot_covariance(figure_paths["covariance"], events)
    plot_factor_diagnostics(figure_paths["factors"], factors, capture_start_ns)
    plot_factor_graph(figure_paths["graph"], factors, capture_start_ns)

    reason_counts = Counter(factor["reason"] for factor in factors)
    position_factors = [
        factor for factor in factors if factor["type"] in ("ABS_POS2", "ABS_POS3")
    ]
    accepted_count = sum(factor["accepted"] == "1" for factor in position_factors)
    bootstrap_count = sum(
        factor["accepted"] == "1"
        and factor["source"] == "gnss_single_antenna_bootstrap"
        for factor in factors
    )
    recovery_count = sum(
        factor["accepted"] == "1" and factor["source"] == "gnss_recovery"
        for factor in factors
    )
    figure_manifest = {
        path.name: {"sha256": sha256(path), "bytes": path.stat().st_size}
        for path in figure_paths.values()
    }
    summary = {
        "schema": "wheelos.openloc-public-xy-visualization.v1",
        "status": "diagnostic_only",
        "capture": input_manifest["capture"],
        "replay": {
            "event_file_sha256": sha256(events_path),
            "factor_file_sha256": sha256(factors_path),
            "input_manifest_sha256": sha256(input_manifest_path),
            "odometry_events": len(odom),
            "prepared_gnss_fixes": len(fixes),
            "accepted_factors": accepted_count,
            "rejected_factors": len(position_factors) - accepted_count,
            "validated_alignment_bootstraps": bootstrap_count,
            "validated_recovery_reanchors": recovery_count,
            "factor_reason_counts": dict(sorted(reason_counts.items())),
            "initial_60s_fit": fit_summary,
        },
        "original_gga": (
            {
                "path": str(args.gga),
                "sha256": sha256(args.gga),
                "line_counts": dict(sorted(gga_counts.items())),
                "prepared_fix_cross_check": raw_match,
            }
            if args.gga
            else None
        ),
        "figures": figure_manifest,
        "limitations": [
            "Raw GGA and prepared GNSS fixes originate from the same receiver; neither is independent truth.",
            "The initial 60-second SE(2) ODOM fit is for visualization only and is in-sample.",
            "Covariance is the backend marginal and is not calibrated or valid for a held published mean.",
            "The factor graph image is a schematic reconstructed from factor state_id records, not an optimizer graph export.",
            "Rejected factor attempts are shown for diagnosis but are not factors in the optimized graph.",
            "Single-antenna bootstrap and recovery re-anchors are alignment events, not inserted graph factors.",
            "The replay uses idealized arrival equal to measurement time and synthetic receiver-quality fields.",
        ],
        "visualization_conventions": {
            "trajectory_comparison": "evo-style trajectory overlays and same-input residual summaries",
            "factor_graph": "GTSAM-style state/factor bipartite schematic",
            "references": [
                "https://github.com/MichaelGrupp/evo",
                "https://github.com/borglab/gtsam",
            ],
        },
    }
    summary_path_out = output / "visualization_summary.json"
    summary_path_out.write_text(json.dumps(summary, indent=2) + "\n")

    image_sections = "\n".join(
        f'<section><h2>{html.escape(label)}</h2><img src="{path.name}" alt="{html.escape(label)}"></section>'
        for label, path in (
            ("Trajectory comparison", figure_paths["trajectory"]),
            ("Alignment covariance", figure_paths["covariance"]),
            ("Factor innovation decisions", figure_paths["factors"]),
            ("Factor graph schematic", figure_paths["graph"]),
        )
    )
    index = output / "index.html"
    index.write_text(
        "<!doctype html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
        "<title>OpenLoc diagnostic visualization</title>"
        "<style>body{font:16px sans-serif;max-width:1400px;margin:2rem auto;padding:0 1rem}"
        "img{max-width:100%;height:auto;border:1px solid #ddd}section{margin:2rem 0}"
        ".warning{background:#fff3cd;padding:1rem}</style></head><body>"
        f"<h1>OpenLoc diagnostic visualization — {html.escape(input_manifest['capture'])}</h1>"
        "<p class=\"warning\">Diagnostic only: no independent truth or accuracy "
        "qualification. Raw and prepared GNSS share a receiver source; the covariance "
        "is uncalibrated backend covariance; the factor graph is reconstructed "
        "schematically from decision logs.</p>"
        f"{image_sections}<p>Structured summary: "
        f"<a href=\"{summary_path_out.name}\">{summary_path_out.name}</a></p>"
        "<p>Visualization conventions: "
        "<a href=\"https://github.com/MichaelGrupp/evo\">evo trajectory tools</a>; "
        "<a href=\"https://github.com/borglab/gtsam\">GTSAM factor graphs</a>.</p>"
        "</body></html>\n"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
