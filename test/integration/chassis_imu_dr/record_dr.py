#!/usr/bin/env python3
"""Export and score one bounded Cyber Record chassis-speed/IMU DR run."""

import argparse
import bisect
import csv
from collections import Counter
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

GPS_TO_UNIX_SECONDS = 315964782.0
MAX_INPUT_GAP_SECONDS = 0.05
MAX_FUTURE_SECONDS = 0.02
MAX_SENSOR_SKEW_SECONDS = 0.015
MAX_REFERENCE_BRACKET_SECONDS = 0.05
MIN_RTK_STDDEV_METERS = 0.10
EARTH_SEMI_MAJOR_METERS = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.69437999014e-3

CHASSIS_TOPIC = "/apollo/canbus/chassis"
IMU_TOPIC = "/apollo/sensor/gnss/imu"
BEST_POSE_TOPIC = "/apollo/sensor/gnss/best_pose"
HEADING_TOPIC = "/apollo/sensor/gnss/heading"
REQUIRED_TOPICS = {
    CHASSIS_TOPIC: "apollo.canbus.Chassis",
    IMU_TOPIC: "apollo.drivers.gnss.Imu",
    BEST_POSE_TOPIC: "apollo.drivers.gnss.GnssBestPose",
    HEADING_TOPIC: "apollo.drivers.gnss.Heading",
}


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def enum_value(descriptor, field_name, value_name):
    field = descriptor.fields_by_name[field_name]
    try:
        return field.enum_type.values_by_name[value_name].number
    except (AttributeError, KeyError) as error:
        raise ValueError(
            f"{descriptor.full_name}.{field_name} lacks {value_name}"
        ) from error


def verify_fields(message, required):
    missing = sorted(set(required) - set(message.DESCRIPTOR.fields_by_name))
    if missing:
        raise ValueError(
            f"{message.DESCRIPTOR.full_name} lacks required fields: {missing}"
        )


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def longest_quiet_window(imu_rows, chassis_rows):
    chassis_times = [row["time"] for row in chassis_rows]
    quiet = []
    for row in imu_rows:
        index = bisect.bisect_left(chassis_times, row["time"])
        candidates = [
            chassis_rows[i]
            for i in (index - 1, index)
            if 0 <= i < len(chassis_rows)
        ]
        wheel = min(candidates, key=lambda item: abs(item["time"] - row["time"]))
        acceleration_norm = math.sqrt(sum(x * x for x in row["acceleration"]))
        angular_rate_norm = math.sqrt(sum(x * x for x in row["angular_velocity"]))
        if (
            abs(wheel["time"] - row["time"]) <= MAX_SENSOR_SKEW_SECONDS
            and abs(wheel["speed"]) <= 0.02
            and abs(acceleration_norm - 9.80665) <= 0.3
            and angular_rate_norm <= 0.02
        ):
            quiet.append(row["time"])

    windows = []
    start = previous = None
    count = 0
    for timestamp in quiet:
        if previous is None or timestamp - previous > 0.03:
            if start is not None:
                windows.append((start, previous, count))
            start, count = timestamp, 1
        else:
            count += 1
        previous = timestamp
    if start is not None:
        windows.append((start, previous, count))
    return max(windows, key=lambda item: item[1] - item[0], default=None)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def export_record(args):
    try:
        from cyber_record.record import Record
    except ImportError as error:
        raise RuntimeError(
            "cyber_record is required only for the bounded export step"
        ) from error

    source = args.record_file.resolve(strict=True)
    if not source.is_file() or not source.name.endswith(".record.00000"):
        raise ValueError("input must be one existing .record.00000 segment file")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise ValueError("output directory must be empty to preserve prior evidence")
    wheel_scale = finite(args.wheel_scale, "wheel scale")
    if wheel_scale <= 0.0:
        raise ValueError("wheel scale must be positive")

    events = []
    chassis_rows = []
    imu_rows = []
    best_pose_rows = []
    heading_rows = []
    metadata = {}
    chassis_descriptor = None
    best_pose_descriptor = None
    heading_descriptor = None
    with Record(str(source)) as record:
        start_ns = record.get_start_time()
        end_ns = record.get_end_time()
        record_version = record.version
        metadata = {
            channel.name: {
                "message_type": channel.message_type,
                "message_count": channel.message_number,
            }
            for channel in record.get_channel_cache()
        }
        for topic, message_type in REQUIRED_TOPICS.items():
            channel = metadata.get(topic)
            if channel is None or channel["message_type"] != message_type:
                raise ValueError(
                    f"required channel {topic} must have type {message_type}"
                )

        for topic, message, record_time_ns in record.read_messages(
            topics=list(REQUIRED_TOPICS),
            start_time=start_ns,
            end_time=end_ns,
        ):
            if not message.HasField("header"):
                raise ValueError(f"{topic} sample has no header")
            receive_time = record_time_ns * 1e-9
            timestamp = finite(message.header.timestamp_sec, f"{topic} header time")
            sequence = int(message.header.sequence_num)
            if sequence <= 0:
                raise ValueError(f"{topic} has an invalid sequence number")
            if timestamp - receive_time > MAX_FUTURE_SECONDS:
                raise ValueError(f"{topic} measurement exceeds the future-time gate")
            if receive_time - timestamp > 0.2:
                raise ValueError(f"{topic} measurement exceeds the stale-time gate")
            if topic == CHASSIS_TOPIC:
                chassis_descriptor = message.DESCRIPTOR
                verify_fields(
                    message,
                    ("speed_mps", "gear_location", "driving_mode", "error_code"),
                )
                if not all(
                    message.HasField(name)
                    for name in ("speed_mps", "gear_location", "driving_mode", "error_code")
                ):
                    raise ValueError("chassis sample lacks a required populated field")
                row = {
                    "time": timestamp,
                    "receive_time": receive_time,
                    "sequence": sequence,
                    "raw_speed": finite(message.speed_mps, "chassis speed_mps"),
                    "speed": finite(message.speed_mps, "chassis speed_mps"),
                    "gear": int(message.gear_location),
                    "mode": int(message.driving_mode),
                    "error": int(message.error_code),
                }
                chassis_rows.append(row)
                events.append(("W", row, None))
            elif topic == IMU_TOPIC:
                verify_fields(
                    message,
                    ("linear_acceleration", "angular_velocity"),
                )
                if not all(
                    message.HasField(name)
                    for name in ("linear_acceleration", "angular_velocity")
                ):
                    raise ValueError("raw IMU sample lacks populated acceleration or gyro")
                acceleration = tuple(
                    finite(getattr(message.linear_acceleration, axis), f"IMU accel {axis}")
                    for axis in ("x", "y", "z")
                )
                angular_velocity = tuple(
                    finite(getattr(message.angular_velocity, axis), f"IMU gyro {axis}")
                    for axis in ("x", "y", "z")
                )
                if args.imu_frame == "apollo-rfu":
                    acceleration = (
                        acceleration[1],
                        -acceleration[0],
                        acceleration[2],
                    )
                    angular_velocity = (
                        angular_velocity[1],
                        -angular_velocity[0],
                        angular_velocity[2],
                    )
                row = {
                    "time": timestamp,
                    "receive_time": receive_time,
                    "sequence": sequence,
                    "acceleration": acceleration,
                    "angular_velocity": angular_velocity,
                }
                imu_rows.append(row)
                events.append(("I", row, None))
            elif topic == BEST_POSE_TOPIC:
                best_pose_descriptor = message.DESCRIPTOR
                verify_fields(
                    message,
                    (
                        "measurement_time",
                        "sol_status",
                        "sol_type",
                        "latitude",
                        "longitude",
                        "latitude_std_dev",
                        "longitude_std_dev",
                    ),
                )
                if not all(
                    message.HasField(name)
                    for name in (
                        "measurement_time",
                        "sol_status",
                        "sol_type",
                        "latitude",
                        "longitude",
                        "latitude_std_dev",
                        "longitude_std_dev",
                    )
                ):
                    raise ValueError("RTK best-pose sample lacks required evidence fields")
                gps_time = finite(message.measurement_time, "RTK measurement_time")
                row = {
                    "time": gps_time + GPS_TO_UNIX_SECONDS,
                    "header_time": timestamp,
                    "status": int(message.sol_status),
                    "type": int(message.sol_type),
                    "latitude": finite(message.latitude, "RTK latitude"),
                    "longitude": finite(message.longitude, "RTK longitude"),
                    "height_msl": finite(message.height_msl, "RTK height_msl")
                    if message.HasField("height_msl")
                    else 0.0,
                    "undulation": finite(message.undulation, "RTK undulation")
                    if message.HasField("undulation")
                    else 0.0,
                    "latitude_std_dev": finite(
                        message.latitude_std_dev, "RTK latitude_std_dev"
                    ),
                    "longitude_std_dev": finite(
                        message.longitude_std_dev, "RTK longitude_std_dev"
                    ),
                }
                best_pose_rows.append(row)
            else:
                heading_descriptor = message.DESCRIPTOR
                verify_fields(
                    message,
                    ("measurement_time", "solution_status", "position_type", "heading"),
                )
                if not all(
                    message.HasField(name)
                    for name in ("measurement_time", "solution_status", "position_type", "heading")
                ):
                    raise ValueError("GNSS heading sample lacks required fields")
                row = {
                    "time": finite(message.measurement_time, "heading measurement_time")
                    + GPS_TO_UNIX_SECONDS,
                    "header_time": timestamp,
                    "status": int(message.solution_status),
                    "type": int(message.position_type),
                    "heading_deg": finite(message.heading, "GNSS heading"),
                    "heading_std_dev": finite(
                        message.heading_std_dev, "GNSS heading_std_dev"
                    )
                    if message.HasField("heading_std_dev")
                    else None,
                }
                heading_rows.append(row)

    if not chassis_rows or not imu_rows or not best_pose_rows or not heading_rows:
        raise ValueError("record is missing one or more required data streams")

    for name, rows in (
        ("chassis", chassis_rows),
        ("imu", imu_rows),
        ("best_pose", best_pose_rows),
        ("heading", heading_rows),
    ):
        times = [row["time"] for row in rows]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f"{name} measurement timestamps are not strictly increasing")
        gaps = [b - a for a, b in zip(times, times[1:])]
        if max(gaps, default=0.0) > MAX_INPUT_GAP_SECONDS:
            raise ValueError(f"{name} stream exceeds the 50 ms input-gap gate")

    drive_value = enum_value(chassis_descriptor, "gear_location", "GEAR_DRIVE")
    reverse_value = enum_value(chassis_descriptor, "gear_location", "GEAR_REVERSE")
    auto_value = enum_value(chassis_descriptor, "driving_mode", "COMPLETE_AUTO_DRIVE")
    no_error_value = enum_value(chassis_descriptor, "error_code", "NO_ERROR")
    computed_value = enum_value(best_pose_descriptor, "sol_status", "SOL_COMPUTED")
    fixed_value = enum_value(best_pose_descriptor, "sol_type", "INS_RTKFIXED")
    heading_computed_value = computed_value
    heading_fixed_value = fixed_value

    if args.speed_sign_from_gear:
        if any(
            row["gear"] not in (drive_value, reverse_value)
            for row in chassis_rows
        ):
            raise ValueError(
                "gear-based speed sign needs DRIVE or REVERSE samples only"
            )
        if any(row["raw_speed"] < -0.02 for row in chassis_rows):
            raise ValueError("negative raw speed is ambiguous with gear-based sign")
    elif any(row["gear"] != drive_value for row in chassis_rows):
        raise ValueError("selected segment contains a non-DRIVE gear sample")
    if any(row["mode"] != auto_value for row in chassis_rows):
        raise ValueError("selected segment contains a non-automatic driving mode")
    if any(row["error"] != no_error_value for row in chassis_rows):
        raise ValueError("selected segment contains a chassis error")
    if not args.speed_sign_from_gear and any(
        row["raw_speed"] < -0.02 for row in chassis_rows
    ):
        raise ValueError("negative speed_mps is ambiguous in this DRIVE-only run")
    for row in chassis_rows:
        direction = (
            -1.0
            if args.speed_sign_from_gear and row["gear"] == reverse_value
            else 1.0
        )
        row["speed"] = row["raw_speed"] * direction * wheel_scale
    if any(
        row["status"] != computed_value
        or row["type"] != fixed_value
        or max(row["latitude_std_dev"], row["longitude_std_dev"])
        > MIN_RTK_STDDEV_METERS
        for row in best_pose_rows
    ):
        raise ValueError("RTK quality does not pass the fixed-solution screening gate")
    if any(
        row["status"] != heading_computed_value
        or row["type"] != heading_fixed_value
        or row["heading_std_dev"] is None
        or row["heading_std_dev"] > 0.1
        or not 0.0 <= row["heading_deg"] < 360.0
        for row in heading_rows
    ):
        raise ValueError("GNSS heading does not pass the initialization screening gate")
    if any(abs(row["header_time"] - row["time"]) > MAX_FUTURE_SECONDS for row in best_pose_rows):
        raise ValueError("RTK payload time does not map to its Unix header within 20 ms")
    if any(abs(row["header_time"] - row["time"]) > MAX_FUTURE_SECONDS for row in heading_rows):
        raise ValueError("heading payload time does not map to its Unix header within 20 ms")

    duration_seconds = (end_ns - start_ns) * 1e-9
    if not 45.0 <= duration_seconds <= 75.0:
        raise ValueError("selected segment duration is outside the 45–75 second gate")
    min_samples = int(0.9 * duration_seconds * 100.0)
    for topic, rows in (
        (CHASSIS_TOPIC, chassis_rows),
        (IMU_TOPIC, imu_rows),
        (BEST_POSE_TOPIC, best_pose_rows),
        (HEADING_TOPIC, heading_rows),
    ):
        if len(rows) < min_samples:
            raise ValueError(f"{topic} is below 90% of the expected 100 Hz coverage")
        if len(rows) != metadata[topic]["message_count"]:
            raise ValueError(f"{topic} read count differs from its record index")

    quiet_window = longest_quiet_window(imu_rows, chassis_rows)
    if (
        quiet_window is None
        or quiet_window[2] < 50
        or quiet_window[1] - quiet_window[0] < 2.0
    ):
        raise ValueError("no continuous joint stationary window passes initialization gates")

    events.sort(
        key=lambda event: (
            event[1]["receive_time"],
            0 if event[0] == "W" else 1,
        )
    )
    imu_frontier = None
    late_wheels = 0
    for kind, row, _ in events:
        if kind == "W" and imu_frontier is not None and row["time"] < imu_frontier:
            late_wheels += 1
        elif kind == "I":
            imu_frontier = row["time"]
    if late_wheels:
        raise ValueError(
            f"{late_wheels} wheel samples arrive behind the committed IMU frontier"
        )

    heading_times = [row["time"] for row in heading_rows]
    references = []
    for fix in best_pose_rows:
        index = bisect.bisect_left(heading_times, fix["time"])
        candidates = [
            heading_rows[i]
            for i in (index - 1, index)
            if 0 <= i < len(heading_rows)
        ]
        heading = min(candidates, key=lambda item: abs(item["time"] - fix["time"]))
        if abs(heading["time"] - fix["time"]) > MAX_REFERENCE_BRACKET_SECONDS:
            raise ValueError("best-pose epoch has no nearby fixed heading sample")
        references.append(
            {
                **fix,
                "heading_deg": heading["heading_deg"],
                "heading_std_dev": heading["heading_std_dev"],
                "heading_status": heading["status"],
                "heading_type": heading["type"],
            }
        )

    signed_wheel_displacement = sum(
        0.5 * (current["speed"] + previous["speed"])
        * (current["time"] - previous["time"])
        for previous, current in zip(chassis_rows, chassis_rows[1:])
    )
    wheel_path_length = sum(
        0.5 * (abs(current["speed"]) + abs(previous["speed"]))
        * (current["time"] - previous["time"])
        for previous, current in zip(chassis_rows, chassis_rows[1:])
    )
    moving_samples = sum(abs(row["speed"]) > 0.1 for row in chassis_rows)
    if moving_samples < 100 or wheel_path_length < 20.0:
        raise ValueError("selected segment lacks a sustained moving interval")

    reference_times = [row["time"] for row in best_pose_rows]
    chassis_times = [row["time"] for row in chassis_rows]
    rtk_speed_windows = []
    start_time = best_pose_rows[0]["time"]
    for window_start in range(0, int(duration_seconds), 10):
        first = start_time + window_start
        last = min(start_time + window_start + 10.0, best_pose_rows[-1]["time"])
        if last - first < 5.0:
            continue
        start_fix = interpolate_rows(
            best_pose_rows, reference_times, first, MAX_REFERENCE_BRACKET_SECONDS
        )
        end_fix = interpolate_rows(
            best_pose_rows, reference_times, last, MAX_REFERENCE_BRACKET_SECONDS
        )
        if start_fix is None or end_fix is None:
            raise ValueError("RTK track does not bracket a forward-speed audit window")
        speed_integral = 0.0
        lower = bisect.bisect_left(chassis_times, first)
        upper = bisect.bisect_right(chassis_times, last)
        window_chassis = chassis_rows[max(0, lower - 1) : upper + 1]
        for previous, current in zip(window_chassis, window_chassis[1:]):
            overlap_start = max(first, previous["time"])
            overlap_end = min(last, current["time"])
            if overlap_end <= overlap_start:
                continue
            speed_integral += 0.5 * (previous["speed"] + current["speed"]) * (
                overlap_end - overlap_start
            )
        first_fix = best_pose_rows[0]
        origin = (
            first_fix["latitude"],
            first_fix["longitude"],
            first_fix["height_msl"] + first_fix["undulation"],
        )
        start_enu = local_enu(origin, start_fix)
        end_enu = local_enu(origin, end_fix)
        east_delta = end_enu[0] - start_enu[0]
        north_delta = end_enu[1] - start_enu[1]
        start_heading = min(heading_rows, key=lambda row: abs(row["time"] - first))
        end_heading = min(heading_rows, key=lambda row: abs(row["time"] - last))
        heading_delta = (
            end_heading["heading_deg"] - start_heading["heading_deg"] + 180.0
        ) % 360.0 - 180.0
        mean_heading_degrees = (
            start_heading["heading_deg"] + heading_delta * 0.5
        ) % 360.0
        mean_heading = math.radians(
            90.0 - mean_heading_degrees
        )
        forward_projection = east_delta * math.cos(mean_heading) + north_delta * math.sin(
            mean_heading
        )
        if (
            abs(speed_integral) > 5.0
            and speed_integral * forward_projection <= 0.0
        ):
            raise ValueError("signed chassis speed disagrees with RTK forward direction")
        rtk_speed_windows.append(
            {
                "start_elapsed_seconds": window_start,
                "duration_seconds": last - first,
                "chassis_speed_integral_m": speed_integral,
                "rtk_forward_projection_m": forward_projection,
                "rtk_horizontal_displacement_m": math.hypot(east_delta, north_delta),
            }
        )

    gates = {
        "segment_duration_seconds": duration_seconds,
        "message_counts": {
            CHASSIS_TOPIC: len(chassis_rows),
            IMU_TOPIC: len(imu_rows),
            BEST_POSE_TOPIC: len(best_pose_rows),
            HEADING_TOPIC: len(heading_rows),
        },
        "max_measurement_gaps_seconds": {
            name: max(
                (b["time"] - a["time"] for a, b in zip(rows, rows[1:])),
                default=0.0,
            )
            for name, rows in (
                ("chassis", chassis_rows),
                ("imu", imu_rows),
                ("best_pose", best_pose_rows),
                ("heading", heading_rows),
            )
        },
        "stationary_window": {
            "start_unix_seconds": quiet_window[0],
            "end_unix_seconds": quiet_window[1],
            "duration_seconds": quiet_window[1] - quiet_window[0],
            "imu_samples": quiet_window[2],
        },
        "drive_mode_error_codes": {
            "drive_gear": drive_value,
            "reverse_gear": reverse_value,
            "driving_mode": auto_value,
            "error_code": no_error_value,
        },
        "gear_counts": {
            str(gear): sum(row["gear"] == gear for row in chassis_rows)
            for gear in sorted({row["gear"] for row in chassis_rows})
        },
        "raw_speed_mps_range": [
            min(row["raw_speed"] for row in chassis_rows),
            max(row["raw_speed"] for row in chassis_rows),
        ],
        "speed_mps_range": [
            min(row["speed"] for row in chassis_rows),
            max(row["speed"] for row in chassis_rows),
        ],
        "signed_wheel_displacement_m": signed_wheel_displacement,
        "wheel_path_length_m": wheel_path_length,
        "speed_vs_rtk_forward_windows": rtk_speed_windows,
        "samples_above_0_1_mps": moving_samples,
        "max_horizontal_rtk_stddev_m": max(
            max(row["latitude_std_dev"], row["longitude_std_dev"])
            for row in best_pose_rows
        ),
        "payload_to_header_residual_seconds": {
            "best_pose_median": statistics.median(
                row["header_time"] - row["time"] for row in best_pose_rows
            ),
            "best_pose_max_abs": max(
                abs(row["header_time"] - row["time"]) for row in best_pose_rows
            ),
            "heading_median": statistics.median(
                row["header_time"] - row["time"] for row in heading_rows
            ),
            "heading_max_abs": max(
                abs(row["header_time"] - row["time"]) for row in heading_rows
            ),
        },
        "late_wheel_samples": late_wheels,
        "all_gates_passed": True,
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    event_path = args.output_dir / "events.csv"
    with event_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(
            [
                "kind",
                "measurement_time",
                "receive_time",
                "sequence",
                "ax",
                "ay",
                "az",
                "gx",
                "gy",
                "gz",
                "speed_mps",
            ]
        )
        for kind, row, _ in events:
            if kind == "I":
                writer.writerow(
                    [
                        kind,
                        row["time"],
                        row["receive_time"],
                        row["sequence"],
                        *row["acceleration"],
                        *row["angular_velocity"],
                        "",
                    ]
                )
            else:
                writer.writerow(
                    [
                        kind,
                        row["time"],
                        row["receive_time"],
                        row["sequence"],
                        "",
                        "",
                        "",
                        "",
                        "",
                        "",
                        row["speed"],
                    ]
                )

    reference_path = args.output_dir / "reference.csv"
    with reference_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(
            output,
            fieldnames=[
                "time",
                "latitude",
                "longitude",
                "height_msl",
                "undulation",
                "latitude_std_dev",
                "longitude_std_dev",
                "status",
                "type",
                "heading_deg",
                "heading_std_dev",
                "heading_status",
                "heading_type",
            ],
        )
        writer.writeheader()
        writer.writerows(
            {key: row[key] for key in writer.fieldnames} for row in references
        )

    manifest = {
        "source_record": str(source),
        "source_bytes": source.stat().st_size,
        "source_sha256": sha256_file(source),
        "record_version": record_version,
        "record_start_time_ns": start_ns,
        "record_end_time_ns": end_ns,
        "channels": metadata,
        "time_contract": {
            "estimator_time": "Apollo header Unix seconds",
            "receive_time": "Cyber Record index nanoseconds converted to seconds",
            "gnss_payload_time": "GPS seconds + fixed 2026 GPS-to-Unix offset",
            "gps_to_unix_seconds": GPS_TO_UNIX_SECONDS,
            "max_record_future_tolerance_seconds": MAX_FUTURE_SECONDS,
        },
        "thresholds": {
            "max_input_gap_seconds": MAX_INPUT_GAP_SECONDS,
            "max_sensor_skew_seconds": MAX_SENSOR_SKEW_SECONDS,
            "stationary_speed_mps": 0.02,
            "stationary_gyro_radps": 0.02,
            "stationary_accel_norm_tolerance_mps2": 0.3,
            "stationary_min_samples": 50,
            "stationary_min_duration_seconds": 2.0,
            "max_rtk_horizontal_stddev_m": MIN_RTK_STDDEV_METERS,
            "max_heading_stddev_degrees": 0.1,
            "min_motion_distance_m": 20.0,
        },
        "adapter_configuration": {
            "raw_imu_frame_hypothesis": args.imu_frame,
            "raw_imu_frame_id": (
                "empty in source messages; frame interpretation remains a "
                "hypothesis from Apollo paths and cross-record motion checks"
            ),
            "imu_to_openloc_flu_rotation": (
                [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
                if args.imu_frame == "apollo-rfu"
                else [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            ),
            "signed_speed_from_gear": args.speed_sign_from_gear,
            "wheel_scale": wheel_scale,
            "wheel_scale_units": "multiplier applied to reported chassis speed_mps",
        },
        "data_gate": gates,
        "limitations": [
            "Only this single .record.00000 segment is exported; no trigger "
            "directory or segment concatenation occurs.",
            "GNSS BestPose and dual-antenna heading are offline scoring/"
            "alignment references only and may share receiver/IMU errors with "
            "the estimator input.",
            "RTK-fixed status and reported standard deviations are receiver-"
            "reported, not independent surveyed truth or calibrated "
            "uncertainty.",
            "The source header frame_id is empty. Any input-axis "
            "interpretation remains conditional on Apollo producer "
            "configuration and the cross-record diagnostics.",
            "The interval does not establish slope, rough-road, wheel-slip, "
            "GNSS-outage, LiDAR or scan-to-map behavior.",
            "IMU-to-body translation and GNSS antenna lever arm remain "
            "unmeasured; receiver RTK/INS is not independent surveyed truth.",
        ],
        "artifacts": {
            "events_csv": event_path.name,
            "events_sha256": sha256_file(event_path),
            "reference_csv": reference_path.name,
            "reference_sha256": sha256_file(reference_path),
        },
    }
    with (args.output_dir / "manifest.json").open("w", encoding="utf-8") as output:
        json.dump(manifest, output, indent=2, sort_keys=True, allow_nan=False)
        output.write("\n")
    print(json.dumps(gates, indent=2, sort_keys=True))


def ecef(latitude_deg, longitude_deg, height_m):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    normal_radius = EARTH_SEMI_MAJOR_METERS / math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    return (
        (normal_radius + height_m) * math.cos(latitude) * math.cos(longitude),
        (normal_radius + height_m) * math.cos(latitude) * math.sin(longitude),
        (normal_radius * (1.0 - WGS84_ECCENTRICITY_SQUARED) + height_m)
        * sin_latitude,
    )


def local_enu(origin, fix):
    origin_ecef = ecef(*origin)
    point_ecef = ecef(
        fix["latitude"],
        fix["longitude"],
        fix["height_msl"] + fix["undulation"],
    )
    dx, dy, dz = (point_ecef[i] - origin_ecef[i] for i in range(3))
    latitude = math.radians(origin[0])
    longitude = math.radians(origin[1])
    east = -math.sin(longitude) * dx + math.cos(longitude) * dy
    north = (
        -math.sin(latitude) * math.cos(longitude) * dx
        - math.sin(latitude) * math.sin(longitude) * dy
        + math.cos(latitude) * dz
    )
    return east, north


def interpolate_rows(rows, times, timestamp, max_gap):
    index = bisect.bisect_left(times, timestamp)
    if index < len(times) and times[index] == timestamp:
        return rows[index]
    if index == 0 or index == len(times):
        return None
    left, right = rows[index - 1], rows[index]
    gap = times[index] - times[index - 1]
    if gap > max_gap:
        return None
    fraction = (timestamp - times[index - 1]) / gap
    output = {}
    for key in left:
        if key in ("heading_deg",):
            delta = (right[key] - left[key] + 180.0) % 360.0 - 180.0
            output[key] = (left[key] + fraction * delta) % 360.0
        elif isinstance(left[key], (int, float)) and isinstance(right[key], (int, float)):
            output[key] = left[key] + fraction * (right[key] - left[key])
        else:
            output[key] = left[key]
    return output


def quat_yaw(row):
    w, x, y, z = (float(row[key]) for key in ("qw", "qx", "qy", "qz"))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def summarize(values):
    if not values:
        return {"count": 0, "median": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def evaluate_replay(args):
    with args.trajectory.open(newline="", encoding="utf-8") as source:
        trajectory = list(csv.DictReader(source))
    with args.reference.open(newline="", encoding="utf-8") as source:
        references = list(csv.DictReader(source))
    if not trajectory or not references:
        raise ValueError("trajectory and RTK reference files must be nonempty")

    numeric_trajectory = []
    for row in trajectory:
        numeric_trajectory.append(
            {
                key: float(row[key])
                for key in (
                    "time",
                    "px",
                    "py",
                    "pz",
                    "qw",
                    "qx",
                    "qy",
                    "qz",
                )
            }
            | {
                "valid": int(row["valid"]),
                "integrity_reason": row["integrity_reason"],
                "precision_reason": row["precision_reason"],
            }
        )
    numeric_reference = []
    for row in references:
        numeric_reference.append(
            {
                key: float(row[key])
                for key in (
                    "time",
                    "latitude",
                    "longitude",
                    "height_msl",
                    "undulation",
                    "heading_deg",
                    "heading_std_dev",
                    "latitude_std_dev",
                    "longitude_std_dev",
                )
            }
            | {
                "status": int(row["status"]),
                "type": int(row["type"]),
                "heading_status": int(row["heading_status"]),
                "heading_type": int(row["heading_type"]),
            }
        )
    if any(
        numeric_trajectory[i]["time"] <= numeric_trajectory[i - 1]["time"]
        for i in range(1, len(numeric_trajectory))
    ):
        raise ValueError("trajectory output timestamps are not strictly increasing")

    valid_rows = [row for row in numeric_trajectory if row["valid"]]
    if not valid_rows:
        raise ValueError("LocalEstimator produced no valid state")
    precision_qualified_states = sum(
        row["precision_reason"] == "NONE" for row in trajectory
    )
    reference_times = [row["time"] for row in numeric_reference]
    first_state = valid_rows[0]
    initial_reference = interpolate_rows(
        numeric_reference,
        reference_times,
        first_state["time"],
        MAX_REFERENCE_BRACKET_SECONDS,
    )
    if initial_reference is None:
        raise ValueError("first valid estimator state is not bracketed by RTK")
    origin = (
        initial_reference["latitude"],
        initial_reference["longitude"],
        initial_reference["height_msl"] + initial_reference["undulation"],
    )
    reference_enu = [
        local_enu(origin, row)
        for row in numeric_reference
    ]
    for index, row in enumerate(numeric_reference):
        row["east"], row["north"] = reference_enu[index]

    initial_reference = interpolate_rows(
        numeric_reference,
        reference_times,
        first_state["time"],
        MAX_REFERENCE_BRACKET_SECONDS,
    )
    if initial_reference is None or initial_reference["heading_std_dev"] > 0.1:
        raise ValueError("initial RTK position/heading is not qualified")
    yaw_reference = math.radians(90.0 - initial_reference["heading_deg"])
    yaw_offset = yaw_reference - quat_yaw(first_state)
    cos_yaw, sin_yaw = math.cos(yaw_offset), math.sin(yaw_offset)
    rotated_x = cos_yaw * first_state["px"] - sin_yaw * first_state["py"]
    rotated_y = sin_yaw * first_state["px"] + cos_yaw * first_state["py"]
    translation_x = initial_reference["east"] - rotated_x
    translation_y = initial_reference["north"] - rotated_y

    output_rows = []
    horizontal_errors = []
    longitudinal_errors = []
    lateral_errors = []
    max_bracket = 0.0
    for state in valid_rows:
        ref = interpolate_rows(
            numeric_reference,
            reference_times,
            state["time"],
            MAX_REFERENCE_BRACKET_SECONDS,
        )
        if (
            ref is None
            or ref["status"] != 0
            or ref["type"] != 56
            or max(ref["latitude_std_dev"], ref["longitude_std_dev"])
            > MIN_RTK_STDDEV_METERS
            or ref["heading_status"] != 0
            or ref["heading_type"] != 56
        ):
            continue
        index = bisect.bisect_left(reference_times, state["time"])
        if index == 0:
            bracket = reference_times[0] - state["time"]
        elif index == len(reference_times):
            bracket = state["time"] - reference_times[-1]
        elif reference_times[index] == state["time"]:
            bracket = 0.0
        else:
            bracket = reference_times[index] - reference_times[index - 1]
        max_bracket = max(max_bracket, bracket)
        east = cos_yaw * state["px"] - sin_yaw * state["py"] + translation_x
        north = sin_yaw * state["px"] + cos_yaw * state["py"] + translation_y
        error_east = east - ref["east"]
        error_north = north - ref["north"]
        yaw = math.radians(90.0 - ref["heading_deg"])
        longitudinal = error_east * math.cos(yaw) + error_north * math.sin(yaw)
        lateral = -error_east * math.sin(yaw) + error_north * math.cos(yaw)
        horizontal = math.hypot(error_east, error_north)
        horizontal_errors.append(horizontal)
        longitudinal_errors.append(abs(longitudinal))
        lateral_errors.append(abs(lateral))
        output_rows.append(
            {
                "time": state["time"],
                "elapsed_from_init": state["time"] - first_state["time"],
                "estimate_east": east,
                "estimate_north": north,
                "rtk_east": ref["east"],
                "rtk_north": ref["north"],
                "error_east": error_east,
                "error_north": error_north,
                "horizontal_error": horizontal,
                "absolute_longitudinal_error": abs(longitudinal),
                "absolute_lateral_error": abs(lateral),
                "integrity_reason": state["integrity_reason"],
                "precision_reason": state["precision_reason"],
            }
        )
    if not output_rows:
        raise ValueError("no estimator states passed the RTK scoring gates")

    output_rows.sort(key=lambda row: row["time"])
    score_times = [row["time"] for row in output_rows]
    drift_at_horizons = {}
    first_scored = output_rows[0]
    for horizon in (1.0, 10.0, 30.0, 55.0):
        target = first_scored["time"] + horizon
        later = interpolate_rows(output_rows, score_times, target, 0.05)
        if later is None:
            drift_at_horizons[f"{horizon:g}_seconds"] = None
            continue
        estimate_delta = (
            later["estimate_east"] - first_scored["estimate_east"],
            later["estimate_north"] - first_scored["estimate_north"],
        )
        reference_delta = (
            later["rtk_east"] - first_scored["rtk_east"],
            later["rtk_north"] - first_scored["rtk_north"],
        )
        drift_at_horizons[f"{horizon:g}_seconds"] = math.hypot(
            estimate_delta[0] - reference_delta[0],
            estimate_delta[1] - reference_delta[1],
        )

    elapsed = numeric_trajectory[-1]["time"] - numeric_trajectory[0]["time"]
    error_windows = {}
    for start in range(0, math.ceil(elapsed / 10.0) * 10, 10):
        stop = min(start + 10.0, elapsed)
        values = [
            row["horizontal_error"]
            for row in output_rows
            if start <= row["elapsed_from_init"] < stop
        ]
        error_windows[f"{start}-{stop:g}_seconds"] = summarize(values)

    update_counts = Counter(row["update_reason"] for row in trajectory)
    integrity_counts = Counter(row["integrity_reason"] for row in trajectory)
    precision_counts = Counter(row["precision_reason"] for row in trajectory)
    source_manifest_path = args.reference.parent / "manifest.json"
    source_manifest = None
    if source_manifest_path.is_file():
        with source_manifest_path.open(encoding="utf-8") as source:
            source_manifest = json.load(source)
    event_path = args.reference.parent / "events.csv"
    replay_source_path = Path(__file__).with_name("local_estimator_replay.cc")

    summary = {
        "run_label": args.run_label,
        "estimator_config": {
            "wheel_speed_variance_m2ps2": args.wheel_variance,
            "nonholonomic_constraint_enabled": args.use_nhc,
            "nonholonomic_variance_m2ps2": 0.01 if args.use_nhc else None,
            "zero_velocity_update_enabled": False,
            "stationary_duration_seconds": 2.0,
        },
        "adapter_configuration": source_manifest.get("adapter_configuration")
        if source_manifest is not None
        else None,
        "input_artifacts": {
            "trajectory_csv": str(args.trajectory),
            "trajectory_sha256": sha256_file(args.trajectory),
            "reference_csv": str(args.reference),
            "reference_sha256": sha256_file(args.reference),
            "source_manifest": str(source_manifest_path)
            if source_manifest is not None
            else None,
            "source_record_sha256": source_manifest.get("source_sha256")
            if source_manifest is not None
            else None,
            "events_csv_sha256": sha256_file(event_path)
            if event_path.is_file()
            else None,
        },
        "implementation_artifacts": {
            "adapter_source": str(Path(__file__).resolve()),
            "adapter_source_sha256": sha256_file(Path(__file__)),
            "replay_source": str(replay_source_path.resolve()),
            "replay_source_sha256": sha256_file(replay_source_path),
            "bazel_target": "//:local_estimator_dataset_replay",
        },
        "record_window_seconds": elapsed,
        "first_valid_state_unix_seconds": first_state["time"],
        "initialization_delay_seconds": first_state["time"] - numeric_trajectory[0]["time"],
        "integrity_valid_estimator_states": len(valid_rows),
        "total_estimator_states": len(numeric_trajectory),
        "integrity_valid_state_fraction": len(valid_rows) / len(numeric_trajectory),
        "precision_qualified_states": precision_qualified_states,
        "precision_qualified_state_fraction": precision_qualified_states
        / len(numeric_trajectory),
        "update_reason_counts": dict(sorted(update_counts.items())),
        "integrity_reason_counts": dict(sorted(integrity_counts.items())),
        "precision_reason_counts": dict(sorted(precision_counts.items())),
        "scored_states": len(output_rows),
        "rtk_bracket_max_seconds": max_bracket,
        "initial_alignment": {
            "method": "one-time translation and yaw from first valid RTK/dual-antenna heading; no trajectory fitting",
            "yaw_offset_radians": yaw_offset,
            "translation_east_m": translation_x,
            "translation_north_m": translation_y,
        },
        "horizontal_error_m": summarize(horizontal_errors),
        "absolute_longitudinal_error_m": summarize(longitudinal_errors),
        "absolute_lateral_error_m": summarize(lateral_errors),
        "horizontal_error_by_elapsed_window_m": error_windows,
        "relative_translation_error_m": drift_at_horizons,
        "evidence_limits": [
            "This is same-record receiver RTK/INS consistency, not independent truth or calibrated accuracy.",
            "Initial translation and yaw are aligned once using the first valid RTK position and dual-antenna heading; subsequent trajectory samples are not fitted.",
            "The reported orientation is not independently validated; yaw alignment is conditional on the GNSS heading-to-vehicle frame relationship.",
            "Only a single short forward DRIVE segment is represented; reverse, turns, slope, slip and extended outages are untested.",
            "Position reference points differ unless the IMU, vehicle base and GNSS antenna lever arms are measured.",
        ],
    }

    summary_path = args.output_dir / "evaluation_summary.json"
    frames_path = args.output_dir / "per_frame_errors.csv"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if summary_path.exists() or frames_path.exists():
        raise ValueError("evaluation output already exists; use a fresh result directory")
    with summary_path.open("w", encoding="utf-8") as output:
        json.dump(summary, output, indent=2, sort_keys=True, allow_nan=False)
        output.write("\n")
    with frames_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(output_rows[0]))
        writer.writeheader()
        writer.writerows(output_rows)
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    export_parser = subparsers.add_parser("export")
    export_parser.add_argument("--record-file", type=Path, required=True)
    export_parser.add_argument("--output-dir", type=Path, required=True)
    export_parser.add_argument(
        "--imu-frame",
        choices=("as-recorded", "apollo-rfu"),
        default="as-recorded",
        help="transform Apollo RFU vectors to OpenLoc FLU when selected",
    )
    export_parser.add_argument(
        "--speed-sign-from-gear",
        action="store_true",
        help="interpret speed_mps as magnitude and sign it using DRIVE/REVERSE",
    )
    export_parser.add_argument("--wheel-scale", type=float, default=1.0)
    evaluate_parser = subparsers.add_parser("evaluate")
    evaluate_parser.add_argument("--trajectory", type=Path, required=True)
    evaluate_parser.add_argument("--reference", type=Path, required=True)
    evaluate_parser.add_argument("--output-dir", type=Path, required=True)
    evaluate_parser.add_argument("--run-label", required=True)
    evaluate_parser.add_argument("--wheel-variance", type=float, required=True)
    evaluate_parser.add_argument(
        "--use-nhc", choices=("true", "false"), required=True
    )
    args = parser.parse_args()
    try:
        if args.command == "export":
            export_record(args)
        else:
            args.use_nhc = args.use_nhc == "true"
            evaluate_replay(args)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"record_dr: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
