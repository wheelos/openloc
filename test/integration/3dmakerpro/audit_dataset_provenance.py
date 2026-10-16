#!/usr/bin/env python3
"""Reconcile GNSS-off ODOM exports and associate selected GGA epochs to RTCM."""

import argparse
import csv
import hashlib
import json
import math
import shlex
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

GPS_EPOCH_UNIX_S = 315964800
GPS_UTC_LEAP_SECONDS = 18
GPS_WEEK_S = 604800
MSM4_TYPES = {1074, 1084, 1094, 1114, 1124}
POSE_FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def crc24q(data):
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc = (crc << 1) ^ 0x1864CFB if crc & 0x800000 else crc << 1
            crc &= 0xFFFFFF
    return crc


def bit_value(data, start, width):
    return (int.from_bytes(data, "big") >> (len(data) * 8 - start - width)) & (
        (1 << width) - 1
    )


def signed_value(value, width):
    return value - (1 << width) if value & (1 << (width - 1)) else value


def parse_rtcm(data):
    frames = []
    invalid_candidates = 0
    offset = 0
    while offset + 6 <= len(data):
        if data[offset] != 0xD3 or data[offset + 1] & 0xFC:
            offset += 1
            continue
        payload_size = ((data[offset + 1] & 0x03) << 8) | data[offset + 2]
        frame_end = offset + 3 + payload_size + 3
        if frame_end > len(data):
            invalid_candidates += 1
            offset += 1
            continue
        frame_end_without_crc = offset + 3 + payload_size
        expected_crc = int.from_bytes(data[frame_end_without_crc:frame_end], "big")
        if crc24q(data[offset:frame_end_without_crc]) != expected_crc:
            invalid_candidates += 1
            offset += 1
            continue
        payload = data[offset + 3 : frame_end_without_crc]
        if len(payload) < 2:
            invalid_candidates += 1
            offset += 1
            continue
        message_type = bit_value(payload, 0, 12)
        frame = {
            "offset": offset,
            "message_type": message_type,
        }
        if message_type in MSM4_TYPES:
            if len(payload) < 7:
                raise ValueError(f"RTCM MSM4 frame {message_type} is too short")
            frame["station_id"] = bit_value(payload, 12, 12)
            frame["epoch_raw_ms"] = bit_value(payload, 24, 30)
        elif message_type == 1005:
            if len(payload) < 19:
                raise ValueError("RTCM 1005 frame is too short")
            frame["station_id"] = bit_value(payload, 12, 12)
            frame["ecef_m"] = [
                signed_value(bit_value(payload, start, 38), 38) * 1e-4
                for start in (34, 74, 114)
            ]
        elif message_type == 1006:
            if len(payload) < 3:
                raise ValueError("RTCM 1006 frame is too short")
            frame["station_id"] = bit_value(payload, 12, 12)
        frames.append(frame)
        offset = frame_end
    return frames, invalid_candidates


def associate_gps_epoch(timestamp_unix_s, tow_ms):
    gps_seconds = timestamp_unix_s + GPS_UTC_LEAP_SECONDS - GPS_EPOCH_UNIX_S
    week = round((gps_seconds - tow_ms / 1000.0) / GPS_WEEK_S)
    utc_seconds = (
        GPS_EPOCH_UNIX_S
        + week * GPS_WEEK_S
        + tow_ms / 1000.0
        - GPS_UTC_LEAP_SECONDS
    )
    return week, utc_seconds


def utc_iso(timestamp_unix_s):
    return datetime.fromtimestamp(timestamp_unix_s, timezone.utc).isoformat()


def git_revision(repository):
    return subprocess.check_output(
        ["git", "-C", str(repository), "rev-parse", "HEAD"], text=True
    ).strip()


def manifest_file_hash(manifest, role):
    matches = [item["sha256"] for item in manifest["files"] if item["role"] == role]
    return matches[0] if len(matches) == 1 else None


def reconcile_odometry(baseline_path, full_path, baseline_manifest_path):
    baseline_rows = read_csv(baseline_path)
    full_rows = read_csv(full_path)
    if not baseline_rows or not full_rows:
        raise ValueError("ODOM exports must not be empty")
    baseline_by_time = {row["timestamp"]: row for row in baseline_rows}
    full_by_time = {row["timestamp"]: row for row in full_rows}
    if len(baseline_by_time) != len(baseline_rows):
        raise ValueError("baseline ODOM timestamps are not unique")
    if len(full_by_time) != len(full_rows):
        raise ValueError("full ODOM timestamps are not unique")

    changed = []
    for timestamp, row in baseline_by_time.items():
        candidate = full_by_time.get(timestamp)
        if candidate is None or any(
            row[field] != candidate[field] for field in POSE_FIELDS
        ):
            changed.append(timestamp)
    if changed:
        raise ValueError(f"common ODOM rows differ: {len(changed)}")
    if any(
        float(right["timestamp"]) <= float(left["timestamp"])
        for left, right in zip(full_rows, full_rows[1:])
    ):
        raise ValueError("full ODOM timestamps are not strictly increasing")

    suffix = [row for row in full_rows if row["timestamp"] not in baseline_by_time]
    if any(
        float(row["timestamp"]) <= float(baseline_rows[-1]["timestamp"])
        for row in suffix
    ):
        raise ValueError("full-only rows are not a contiguous suffix")
    baseline_manifest = read_json(baseline_manifest_path)
    baseline_start = float(baseline_rows[0]["timestamp"])
    baseline_end = float(baseline_rows[-1]["timestamp"])
    full_start = float(full_rows[0]["timestamp"])
    suffix_rows = []
    for row in suffix:
        timestamp = float(row["timestamp"])
        suffix_rows.append(
            {
                "timestamp_unix_s": timestamp,
                "utc": utc_iso(timestamp),
                "seconds_after_baseline_last_pose": timestamp - baseline_end,
                "relative_to_full_export_start_s": timestamp - full_start,
                **{field: row[field] for field in POSE_FIELDS},
                "exclusion_reason": (
                    "timestamp is after the 1625-row baseline export end; "
                    "the baseline contains the identical earlier poses"
                ),
            }
        )
    return {
        "baseline_rows": len(baseline_rows),
        "full_rows": len(full_rows),
        "common_rows": len(baseline_by_time),
        "common_pose_rows_with_different_fields": 0,
        "baseline_only_rows": 0,
        "full_only_suffix_rows": suffix_rows,
        "baseline_first_timestamp_unix_s": baseline_start,
        "baseline_last_timestamp_unix_s": baseline_end,
        "full_first_timestamp_unix_s": float(full_rows[0]["timestamp"]),
        "full_last_timestamp_unix_s": float(full_rows[-1]["timestamp"]),
        "baseline_recorded_segment_duration_s": float(
            baseline_manifest["settings"]["segment_duration_s"]
        ),
    }


def selected_gnss_epochs(gnss_csv, reference_path):
    reference = []
    for line in reference_path.read_text().splitlines():
        if line.strip() and not line.lstrip().startswith("#"):
            values = [float(value) for value in line.split()]
            if len(values) != 4:
                raise ValueError("GNSS reference rows must contain four values")
            reference.append(values)
    rows = read_csv(gnss_csv)
    if len(rows) != len(reference):
        raise ValueError("GNSS CSV and reference row counts differ")
    epochs = []
    for row, reference_row in zip(rows, reference):
        timestamp = float(row["timestamp"])
        if not math.isclose(timestamp, reference_row[3], abs_tol=1e-6):
            raise ValueError("GNSS CSV/reference timestamps differ")
        epochs.append(
            {
                "timestamp_unix_s": timestamp,
                "east_m": reference_row[0],
                "north_m": reference_row[1],
                "quality": int(row["quality"]),
                "correction_age_s": float(row["correction_age_s"]),
                "satellites_used": int(row["satellites_used"]),
                "hdop": float(row["hdop"]),
                "gga_station_id": row["station_id"],
            }
        )
    if any(
        right["timestamp_unix_s"] <= left["timestamp_unix_s"]
        for left, right in zip(epochs, epochs[1:])
    ):
        raise ValueError("selected GGA epochs are not strictly increasing")
    return epochs


def create_epoch_associations(epochs, frames, max_offset_ms):
    gps_frames = [frame for frame in frames if frame["message_type"] == 1074]
    if not gps_frames:
        raise ValueError("RTCM stream contains no GPS MSM4 (1074) messages")
    associations = []
    for epoch in epochs:
        candidates = []
        for frame in gps_frames:
            week, normalized_utc = associate_gps_epoch(
                epoch["timestamp_unix_s"], frame["epoch_raw_ms"]
            )
            candidates.append(
                (
                    abs(normalized_utc - epoch["timestamp_unix_s"]),
                    frame,
                    week,
                    normalized_utc,
                )
            )
        delta_s, frame, week, normalized_utc = min(candidates, key=lambda item: item[0])
        if delta_s * 1000.0 > max_offset_ms:
            frame = None
        association = {
            **epoch,
            "gga_payload_time_utc": utc_iso(epoch["timestamp_unix_s"]),
            "rtcm_message_type": 1074 if frame else None,
            "rtcm_station_id": frame["station_id"] if frame else None,
            "rtcm_frame_byte_offset": frame["offset"] if frame else None,
            "rtcm_epoch_time_scale": (
                "GPS week/TOW" if frame else "unavailable_no_associated_message"
            ),
            "rtcm_gps_week": week if frame else None,
            "rtcm_gps_tow_ms": frame["epoch_raw_ms"] if frame else None,
            "rtcm_normalized_epoch_utc": utc_iso(normalized_utc) if frame else None,
            "normalized_epoch_minus_gga_s": normalized_utc - epoch["timestamp_unix_s"]
            if frame
            else None,
            "rtcm_file_arrival_timestamp": None,
            "rtcm_file_arrival_timestamp_status": (
                "unavailable_no_transport_timestamps_in_binary_file"
            ),
            "association_method": (
                "nearest GPS 1074 epoch after GPS-week/UTC normalization"
                if frame
                else "no GPS 1074 epoch within configured threshold"
            ),
            "association_threshold_ms": max_offset_ms,
            "time_precision_note": (
                "GGA payload has 0.01 s resolution and RTCM 1074 TOW has 0.001 s "
                "resolution; matching these fields does not establish PPS lock, "
                "absolute clock accuracy, rover application of this observation, "
                "or message arrival time (not present in the file)"
            ),
            "differential_age_semantics": (
                "receiver-reported GGA age; not the RTCM message epoch or "
                "message-arrival delay"
            ),
        }
        associations.append(association)
    return associations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-odom", required=True, type=Path)
    parser.add_argument("--full-odom", required=True, type=Path)
    parser.add_argument("--baseline-manifest", required=True, type=Path)
    parser.add_argument("--full-manifest", required=True, type=Path)
    parser.add_argument("--gnss-csv", required=True, type=Path)
    parser.add_argument("--gnss-reference", required=True, type=Path)
    parser.add_argument("--rtcm", required=True, type=Path)
    parser.add_argument("--gga-text", required=True, type=Path)
    parser.add_argument("--quality-report", required=True, type=Path)
    parser.add_argument("--map-infra-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-match-offset-ms", type=float, default=1.0)
    args = parser.parse_args()

    paths = (
        args.baseline_odom,
        args.full_odom,
        args.baseline_manifest,
        args.full_manifest,
        args.gnss_csv,
        args.gnss_reference,
        args.rtcm,
        args.gga_text,
        args.quality_report,
    )
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
    if args.max_match_offset_ms < 0.0:
        raise ValueError("maximum association offset must be non-negative")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    baseline_manifest = read_json(args.baseline_manifest)
    full_manifest = read_json(args.full_manifest)
    reconciliation = reconcile_odometry(
        args.baseline_odom, args.full_odom, args.baseline_manifest
    )
    epochs = selected_gnss_epochs(args.gnss_csv, args.gnss_reference)
    if (
        epochs[0]["timestamp_unix_s"]
        < reconciliation["full_first_timestamp_unix_s"]
        or epochs[-1]["timestamp_unix_s"]
        > reconciliation["full_last_timestamp_unix_s"]
    ):
        raise ValueError("selected GGA epochs fall outside the full ODOM export")
    frames, invalid_candidates = parse_rtcm(args.rtcm.read_bytes())
    associations = create_epoch_associations(
        epochs, frames, args.max_match_offset_ms
    )
    quality = read_json(args.quality_report)["summary"]
    message_counts = {}
    station_ids = {}
    reference_ecef = {}
    for frame in frames:
        message_type = str(frame["message_type"])
        message_counts[message_type] = message_counts.get(message_type, 0) + 1
        if "station_id" in frame:
            ids = station_ids.setdefault(message_type, set())
            ids.add(frame["station_id"])
        if frame["message_type"] == 1005:
            key = str(frame["station_id"])
            reference_ecef.setdefault(key, set()).add(
                tuple(round(value, 4) for value in frame["ecef_m"])
            )
    station_ids = {key: sorted(value) for key, value in station_ids.items()}
    reference_ecef = {
        key: [list(value) for value in sorted(values)]
        for key, values in reference_ecef.items()
    }

    output_csv = args.output_dir / "openloc_rtcm_gga_epoch_association.csv"
    output_json = args.output_dir / "openloc_rtcm_gga_epoch_association.json"
    output_odom_json = args.output_dir / "openloc_odometry_reconciliation.json"
    output_odom_csv = args.output_dir / "openloc_odometry_excluded_tail.csv"
    if any(
        path.exists()
        for path in (output_csv, output_json, output_odom_json, output_odom_csv)
    ):
        raise FileExistsError("one or more audit output files already exist")

    with output_csv.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(associations[0]))
        writer.writeheader()
        writer.writerows(associations)
    metadata = {
        "schema_version": 1,
        "dataset": "20261008141912",
        "classification": "source_consistency_only_not_independent_truth",
        "command": shlex.join(
            ["python3", str(Path(__file__).resolve()), *sys.argv[1:]]
        ),
        "diagnostic_script_sha256": sha256(Path(__file__).resolve()),
        "source_sha256": {str(path): sha256(path) for path in paths},
        "code_revision": {
            "openloc": git_revision(Path(__file__).resolve().parents[5]),
            "map_infra": git_revision(args.map_infra_root),
            "fast_lio_baseline": baseline_manifest["git_repositories"][0]["revision"],
            "fast_lio_production_manifest_revision": (
                "not recorded in production manifest; source FAST-LIO digest is "
                + str(manifest_file_hash(full_manifest, "source_fast_lio"))
            ),
        },
        "odometry": reconciliation,
        "gga_count_relationship": {
            "raw_gga_text_checksum_valid_rows": quality["gga_text"]["rows"],
            "unique_gga_text_epochs": quality["gga_text"]["unique_epochs"],
            "duplicate_rows": quality["gga_text"]["duplicate_rows"],
            "unique_epoch_quality_counts": quality["gga_text"][
                "unique_epoch_quality_counts"
            ],
            "selected_gps_bag_epochs": len(epochs),
            "selected_epoch_quality_counts": {
                str(value): sum(epoch["quality"] == value for epoch in epochs)
                for value in sorted({epoch["quality"] for epoch in epochs})
            },
            "selection": (
                "the 162 strictly time-ordered rows from the selected GPS bag "
                "export, paired row-for-row with its ENU reference; all selected "
                "epochs must be inside the 1637-row ODOM time interval"
            ),
        },
        "rtcm": {
            "gps_utc_offset_seconds_used_for_normalization": GPS_UTC_LEAP_SECONDS,
            "gps_utc_offset_is_a_time_conversion_assumption_not_clock_validation": True,
            "valid_frame_count": len(frames),
            "invalid_crc_or_malformed_candidates": invalid_candidates,
            "message_counts": message_counts,
            "station_ids_by_message_type": station_ids,
            "1005_reference_station_ecef_m": reference_ecef,
            "1005_1006_semantics": (
                "station coordinate metadata, not epoch-timestamped correction "
                "messages"
            ),
            "per_epoch_joined_constellation": "GPS 1074 only",
            "message_arrival_timestamp_available": False,
            "message_arrival_timestamp_limit": (
                "The binary RTCM file preserves frame order and byte offsets, "
                "not receiver/network arrival timestamps."
            ),
            "other_msm_types_not_epoch_joined": [1084, 1094, 1114, 1124],
            "association_count": sum(
                row["rtcm_message_type"] == 1074 for row in associations
            ),
            "association_missing_count": sum(
                row["rtcm_message_type"] is None for row in associations
            ),
            "maximum_match_offset_ms": max(
                abs(row["normalized_epoch_minus_gga_s"] or 0.0) * 1000.0
                for row in associations
            ),
            "correction_effect_limit": (
                "RTCM observations and matching station/time fields do not prove "
                "that a rover applied a particular correction or that its "
                "measurement was valid."
            ),
        },
        "outputs": {
            "epoch_csv": str(output_csv),
            "odometry_tail_csv": str(output_odom_csv),
        },
    }
    output_json.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    output_odom_json.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "conclusion": (
                    "The 1625-row baseline is an exact prefix of the 1637-row "
                    "full export. All common timestamps and seven pose fields "
                    "match; the 12 additional rows form a contiguous suffix. "
                    "This is consistent with the shorter recorded baseline "
                    "segment/output horizon, not pose-level filtering."
                ),
                "baseline_odom_path": str(args.baseline_odom),
                "baseline_odom_sha256": sha256(args.baseline_odom),
                "baseline_manifest_path": str(args.baseline_manifest),
                "baseline_manifest_recorded_segment_duration_s": float(
                    baseline_manifest["settings"]["segment_duration_s"]
                ),
                "baseline_recorded_odometry_bag_sha256": manifest_file_hash(
                    baseline_manifest, "odometry_bag"
                ),
                "baseline_recorded_config_sha256": manifest_file_hash(
                    baseline_manifest, "fast_lio_config"
                ),
                "baseline_recorded_executable_sha256": manifest_file_hash(
                    baseline_manifest, "executable_fast_lio"
                ),
                "full_odom_path": str(args.full_odom),
                "full_odom_sha256": sha256(args.full_odom),
                "full_manifest_path": str(args.full_manifest),
                "full_manifest_recorded_segment_duration_s": float(
                    full_manifest["settings"]["segment_duration_s"]
                ),
                "full_recorded_odometry_bag_sha256": manifest_file_hash(
                    full_manifest, "odometry_bag"
                ),
                "full_recorded_config_sha256": manifest_file_hash(
                    full_manifest, "frontend_config"
                ),
                "full_recorded_executable_sha256": manifest_file_hash(
                    full_manifest, "executable_fast_lio"
                ),
                "fast_lio_baseline_revision": baseline_manifest[
                    "git_repositories"
                ][0]["revision"],
                "fast_lio_production_source_digest": manifest_file_hash(
                    full_manifest, "source_fast_lio"
                ),
                **reconciliation,
                "filtering_rule": (
                    "select rows in the common strictly increasing time prefix; "
                    "no row sampling or post-hoc pose filter"
                ),
                "baseline_manifest_does_not_preserve_exact_runner_invocation": True,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n"
    )
    with output_odom_csv.open("w", newline="") as stream:
        suffix = reconciliation["full_only_suffix_rows"]
        fieldnames = list(suffix[0]) if suffix else ["timestamp_unix_s"]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(suffix)
    print(
        json.dumps(
            {
                "association_rows": len(associations),
                "exact_or_within_threshold": metadata["rtcm"]["association_count"],
                "odometry_rows": [
                    reconciliation["baseline_rows"],
                    reconciliation["full_rows"],
                ],
                "full_only_suffix_rows": len(
                    reconciliation["full_only_suffix_rows"]
                ),
                "outputs": metadata["outputs"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
