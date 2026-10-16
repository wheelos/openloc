#!/usr/bin/env python3
"""Reconstruct OpenLoc's horizontal GNSS/ODOM distance-gate rejections."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def read_odometry(path):
    samples = []
    with path.open(newline="") as stream:
        for row in csv.DictReader(stream):
            quaternion = tuple(
                float(row[name]) for name in ("qw", "qx", "qy", "qz")
            )
            norm = math.sqrt(sum(value * value for value in quaternion))
            if not math.isfinite(norm) or norm <= 0.0:
                raise ValueError("ODOM contains an invalid quaternion")
            samples.append(
                (
                    float(row["timestamp"]),
                    float(row["x"]),
                    float(row["y"]),
                    tuple(value / norm for value in quaternion),
                )
            )
    if len(samples) < 2 or any(
        right[0] <= left[0] for left, right in zip(samples, samples[1:])
    ):
        raise ValueError("ODOM timestamps must contain at least two increasing rows")
    return samples


def interpolate_pose(samples, times, timestamp):
    index = bisect.bisect_left(times, timestamp)
    if index < len(samples) and times[index] == timestamp:
        before = after = samples[index]
        fraction = 0.0
        bracket_gap = 0.0
    else:
        if index == 0 or index == len(samples):
            raise ValueError("GNSS epoch is outside the ODOM time range")
        before, after = samples[index - 1], samples[index]
        bracket_gap = after[0] - before[0]
        fraction = (timestamp - before[0]) / bracket_gap

    x = before[1] + fraction * (after[1] - before[1])
    y = before[2] + fraction * (after[2] - before[2])
    q0, q1 = before[3], after[3]
    dot = sum(left * right for left, right in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        quaternion = tuple(
            left + fraction * (right - left)
            for left, right in zip(q0, q1)
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        quaternion = tuple(value / norm for value in quaternion)
    else:
        angle = math.acos(dot)
        denominator = math.sin(angle)
        weight0 = math.sin((1.0 - fraction) * angle) / denominator
        weight1 = math.sin(fraction * angle) / denominator
        quaternion = tuple(
            weight0 * left + weight1 * right
            for left, right in zip(q0, q1)
        )
    return x, y, quaternion, bracket_gap


def quaternion_yaw(quaternion):
    qw, qx, qy, qz = quaternion
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compare_epochs(odometry, times, first, second, config):
    interval = second["timestamp"] - first["timestamp"]
    if interval <= 0.0:
        return None
    first_pose = interpolate_pose(odometry, times, first["timestamp"])
    second_pose = interpolate_pose(odometry, times, second["timestamp"])
    odometry_distance = math.hypot(
        second_pose[0] - first_pose[0],
        second_pose[1] - first_pose[1],
    )
    gnss_distance = math.hypot(
        second["east_m"] - first["east_m"],
        second["north_m"] - first["north_m"],
    )
    tolerance = (
        config["absolute_distance_tolerance_m"]
        + config["relative_distance_tolerance"] * odometry_distance
    )
    yaw_delta = wrap_angle(
        quaternion_yaw(second_pose[2]) - quaternion_yaw(first_pose[2])
    )
    return {
        "from_timestamp_utc_s": first["timestamp"],
        "to_timestamp_utc_s": second["timestamp"],
        "interval_s": interval,
        "odom_distance_m": odometry_distance,
        "gga_distance_m": gnss_distance,
        "signed_distance_error_gga_minus_odom_m": (
            gnss_distance - odometry_distance
        ),
        "distance_gate_tolerance_m": tolerance,
        "odom_speed_mps": odometry_distance / interval,
        "odom_yaw_rate_radps_signed": yaw_delta / interval,
        "odom_bracket_gap_ms": 1000.0 * second_pose[3],
        "distance_gate_would_reject": abs(gnss_distance - odometry_distance)
        > tolerance,
    }


def read_gnss(csv_path, reference_path):
    with reference_path.open() as stream:
        reference = [
            tuple(float(value) for value in line.split())
            for line in stream
            if line.strip() and not line.lstrip().startswith("#")
        ]
    fixes = []
    with csv_path.open(newline="") as stream:
        for row, coordinates in zip(csv.DictReader(stream), reference):
            timestamp = float(row["timestamp"])
            if len(coordinates) != 4 or abs(timestamp - coordinates[3]) > 1e-6:
                raise ValueError("GNSS CSV/reference rows do not match")
            fixes.append(
                {
                    "timestamp": timestamp,
                    "east_m": coordinates[0],
                    "north_m": coordinates[1],
                    "status": row["status"],
                    "mode": row["mode"],
                    "quality": int(row["quality"]),
                    "age_s": float(row["correction_age_s"]),
                    "satellites": int(row["satellites_used"]),
                    "hdop": float(row["hdop"]),
                }
            )
    if len(fixes) != len(reference) or len(fixes) < 2:
        raise ValueError("GNSS CSV/reference row counts differ or are insufficient")
    if any(
        right["timestamp"] <= left["timestamp"]
        for left, right in zip(fixes, fixes[1:])
    ):
        raise ValueError("GNSS timestamps must be strictly increasing")
    return fixes


def read_rtk_context(path, gnss_epoch_count):
    if path is None:
        return None
    with path.open() as stream:
        report = json.load(stream)
    summary = report["summary"]
    correction = summary["correction_rtcm"]
    gps = summary["gps_ros_bag"]
    matched_epochs = correction["gps_1074_epoch_matches_to_gga_bag"]
    return {
        "source": str(path),
        "gnss_epochs": gnss_epoch_count,
        "bag_gga_epochs": gps["count"],
        "msm4_gga_epoch_matches_reported": matched_epochs,
        "all_selected_epochs_reported_matched": (
            gps["count"] == gnss_epoch_count
            and matched_epochs == gnss_epoch_count
        ),
        "per_epoch_timestamps_in_report": False,
        "gps_msm4_gap_after_overlap": correction["gps_1074_gaps_gt_1_1s"],
    }


def attribute(odometry, fixes, rtk_context=None):
    odometry_times = [sample[0] for sample in odometry]
    config = {
        "fixed_quality": 4,
        "maximum_differential_age_s": 3.1,
        "minimum_satellites": 12,
        "maximum_hdop": 1.5,
        "horizontal_sigma_assumption_m": 1.0,
        "stable_fixed_duration_s": 3.0,
        "maximum_fixed_gap_s": 1.5,
        "absolute_distance_tolerance_m": 0.1,
        "relative_distance_tolerance": 0.02,
    }
    stable_start = None
    previous = None
    stable_rejections = 0
    quality_rejections = 0
    rejections = []
    accepted_indices = []
    decisions = {}
    for fix_index, fix in enumerate(fixes):
        eligible = (
            fix["status"] == "A"
            and fix["mode"] == "D"
            and fix["quality"] == config["fixed_quality"]
            and fix["age_s"] <= config["maximum_differential_age_s"]
            and fix["satellites"] >= config["minimum_satellites"]
            and fix["hdop"] <= config["maximum_hdop"]
        )
        if not eligible:
            stable_start = None
            quality_rejections += 1
            decisions[fix_index] = "quality_rejected"
            continue
        if stable_start is None:
            stable_start = fix["timestamp"]
        if fix["timestamp"] - stable_start < config["stable_fixed_duration_s"]:
            stable_rejections += 1
            decisions[fix_index] = "stable_fixed_initialization"
            continue

        current_pose = interpolate_pose(odometry, odometry_times, fix["timestamp"])
        if previous is not None:
            interval = fix["timestamp"] - previous["timestamp"]
            if 0.0 < interval <= config["maximum_fixed_gap_s"]:
                previous_pose = interpolate_pose(
                    odometry, odometry_times, previous["timestamp"]
                )
                odometry_distance = math.hypot(
                    current_pose[0] - previous_pose[0],
                    current_pose[1] - previous_pose[1],
                )
                gnss_distance = math.hypot(
                    fix["east_m"] - previous["east_m"],
                    fix["north_m"] - previous["north_m"],
                )
                tolerance = (
                    config["absolute_distance_tolerance_m"]
                    + config["relative_distance_tolerance"] * odometry_distance
                )
                error = abs(gnss_distance - odometry_distance)
                if error > tolerance:
                    quaternion_dot = abs(
                        sum(
                            left * right
                            for left, right in zip(current_pose[2], previous_pose[2])
                        )
                    )
                    angular_speed = (
                        2.0 * math.acos(min(1.0, quaternion_dot)) / interval
                    )
                    yaw_rate = wrap_angle(
                        quaternion_yaw(current_pose[2])
                        - quaternion_yaw(previous_pose[2])
                    ) / interval
                    rejections.append(
                        {
                            "_fix_index": fix_index,
                            "timestamp_utc_s": fix["timestamp"],
                            "previous_admitted_timestamp_utc_s": previous[
                                "timestamp"
                            ],
                            "interval_s": interval,
                            "odom_distance_m": odometry_distance,
                            "gga_distance_m": gnss_distance,
                            "distance_error_m": error,
                            "distance_gate_tolerance_m": tolerance,
                            "odom_speed_mps": odometry_distance / interval,
                            "odom_angular_speed_radps": angular_speed,
                            "odom_yaw_rate_radps_signed": yaw_rate,
                            "quality": fix["quality"],
                            "status": fix["status"],
                            "mode": fix["mode"],
                            "correction_age_s": fix["age_s"],
                            "satellites_used": fix["satellites"],
                            "hdop": fix["hdop"],
                            "odom_bracket_gap_ms": 1000.0
                            * current_pose[3],
                            "reproduced_reason": "gnss_odometry_distance_disagreement",
                        }
                    )
                    decisions[fix_index] = "distance_rejected"
                    continue
        previous = fix
        accepted_indices.append(fix_index)
        decisions[fix_index] = "accepted"

    def epoch_summary(index):
        if index is None:
            return None
        fix = fixes[index]
        return {
            "timestamp_utc_s": fix["timestamp"],
            "decision": decisions[index],
            "quality": fix["quality"],
            "status": fix["status"],
            "mode": fix["mode"],
            "correction_age_s": fix["age_s"],
            "satellites_used": fix["satellites"],
            "hdop": fix["hdop"],
        }

    for rejection in rejections:
        fix_index = rejection.pop("_fix_index")
        prior_accepted = next(
            (index for index in reversed(accepted_indices) if index < fix_index),
            None,
        )
        following_accepted = next(
            (index for index in accepted_indices if index > fix_index),
            None,
        )
        previous_epoch = fix_index - 1 if fix_index > 0 else None
        next_epoch = fix_index + 1 if fix_index + 1 < len(fixes) else None
        current = fixes[fix_index]
        rejection["adjacent_epochs"] = {
            "previous": epoch_summary(previous_epoch),
            "next": epoch_summary(next_epoch),
        }
        rejection["adjacent_accepted_comparisons"] = {
            "previous_accepted_to_rejected": (
                compare_epochs(
                    odometry,
                    odometry_times,
                    fixes[prior_accepted],
                    current,
                    config,
                )
                if prior_accepted is not None
                else None
            ),
            "rejected_to_next_accepted": (
                compare_epochs(
                    odometry,
                    odometry_times,
                    current,
                    fixes[following_accepted],
                    config,
                )
                if following_accepted is not None
                else None
            ),
        }
        rejection["motion_diagnostics"] = {
            "odom_horizontal_speed_mps": rejection["odom_speed_mps"],
            "odom_rotation_rate_radps_abs": rejection[
                "odom_angular_speed_radps"
            ],
            "odom_yaw_rate_radps_signed": rejection[
                "odom_yaw_rate_radps_signed"
            ],
            "turn_direction_from_odom_yaw": (
                "positive_yaw"
                if rejection["odom_yaw_rate_radps_signed"] > 0.0
                else "negative_yaw"
                if rejection["odom_yaw_rate_radps_signed"] < 0.0
                else "zero_yaw"
            ),
            "linear_acceleration_mps2": "unavailable_no_imu_export",
        }
        rejection["odometry_diagnostics"] = {
            "runtime_health": "unavailable_no_runtime_health_telemetry",
            "valid_point_count": "unavailable_not_exported",
            "covariance": "unavailable_not_exported",
        }
        rejection["residual_structure"] = {
            "signed_horizontal_distance_error_gga_minus_odom_m": (
                rejection["gga_distance_m"] - rejection["odom_distance_m"]
            ),
            "along_track_residual_m": (
                "unavailable_no_verified_T_enu_odom_heading"
            ),
            "cross_track_residual_m": (
                "unavailable_no_verified_T_enu_odom_heading"
            ),
            "vertical_residual_m": (
                "unavailable_vertical_datum_and_frame_not_verified"
            ),
        }
        rejection["rtcm_continuity"] = (
            "all_selected_epochs_reported_matched_by_dataset_crosscheck"
            if rtk_context is not None
            and rtk_context["all_selected_epochs_reported_matched"]
            else "not_joined_per_epoch_in_available_host_export"
        )
        rejection["root_cause"] = "undetermined_from_available_exports"
        rejection["attribution"] = {
            "category": "UNRESOLVED",
            "evidence": (
                "Distance gate reproduced; available exports do not "
                "distinguish GNSS, ODOM, or model error."
            ),
        }

    return {
        "status": "read_only_reconstruction",
        "classification": "same_record_consistency_not_independent_truth",
        "valid": "not_evaluated_read_only_attribution",
        "covariance": "unavailable_in_stored_odom_and_gnss_exports",
        "observability": "not_assessed_by_distance_gate_reconstruction",
        "rejection_reason_codes": sorted(
            {record["reproduced_reason"] for record in rejections}
        ),
        "calibration_state": "uncalibrated",
        "odometry_samples": len(odometry),
        "gnss_epochs": len(fixes),
        "stable_fixed_rejections": stable_rejections,
        "quality_rejections": quality_rejections,
        "distance_gate_rejections": len(rejections),
        "accepted_after_reconstruction": len(accepted_indices),
        "attribution_counts": {"UNRESOLVED": len(rejections)},
        "configuration": config,
        "rtcm_context": rtk_context,
        "rejections": rejections,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odometry", required=True, type=Path)
    parser.add_argument("--gnss-csv", required=True, type=Path)
    parser.add_argument("--gnss-reference", required=True, type=Path)
    parser.add_argument("--rtk-quality-report", type=Path)
    args = parser.parse_args()
    fixes = read_gnss(args.gnss_csv, args.gnss_reference)
    report = attribute(
        read_odometry(args.odometry),
        fixes,
        read_rtk_context(args.rtk_quality_report, len(fixes)),
    )
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
