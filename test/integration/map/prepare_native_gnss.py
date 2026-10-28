#!/usr/bin/env python3
"""Prepare all received GGA events for optional, unqualified horizontal replay."""

import argparse
from collections import Counter
import csv
from decimal import Decimal
import json
import math
from pathlib import Path
import re
import shutil
import sys

from prepare_native_lio import digest


def measurement_ns(value, receive_ns):
    """GGA UTC TOD, resolved to nearest bag UTC day; never use arrival as TOD."""
    text = str(value)
    if not re.fullmatch(r"\d{6}(?:\.\d+)?", text):
        raise ValueError("invalid_gga_utc_time_of_day")
    hour, minute, second = int(text[:2]), int(text[2:4]), Decimal(text[4:])
    if hour >= 24 or minute >= 60 or not 0 <= second < 60:
        raise ValueError("invalid_gga_utc_time_of_day")
    day = receive_ns // 86_400_000_000_000
    tod = (hour * 3600 + minute * 60) * 1_000_000_000 + int(second * 10**9)
    return min(((day + offset) * 86_400_000_000_000 + tod
                for offset in (-1, 0, 1)), key=lambda ns: abs(ns - receive_ns))


def coordinate(value, hemisphere):
    numeric = float(value)
    degrees = int(numeric // 100)
    minutes = numeric - degrees * 100
    if hemisphere not in ("N", "S", "E", "W") or not 0 <= minutes < 60:
        raise ValueError("invalid_gga_coordinate")
    result = degrees + minutes / 60
    return -result if hemisphere in ("S", "W") else result


def nmea_index(path):
    result = {}
    counts = Counter()
    for match in re.finditer(rb"\$((?:GN|GP)GGA,[^\r\n$*]*)\*([0-9A-Fa-f]{2})",
                             path.read_bytes()):
        payload = match[1]
        checksum = 0
        for byte in payload:
            checksum ^= byte
        if checksum != int(match[2], 16):
            counts["bad_checksum"] += 1
            continue
        fields = payload.decode("ascii").split(",")
        result.setdefault(fields[1], []).append(fields)
        counts["checksum_valid"] += 1
    return result, dict(counts)


def event(message, receive_ns, sequence, raw, transformer, origin):
    record = [0, receive_ns, sequence, "", "", int(message.qual),
              int(message.sats), str(message.hdop), str(message.age), "ok"]
    try:
        record[0] = measurement_ns(message.time, receive_ns)
        if message.N not in ("N", "S") or message.E not in ("E", "W"):
            raise ValueError("invalid_gga_hemisphere")
        latitude = float(message.Lat) * (-1 if message.N == "S" else 1)
        longitude = float(message.Lon) * (-1 if message.E == "W" else 1)
        if (not all(math.isfinite(v) for v in (latitude, longitude))
                or not -90 <= latitude <= 90 or not -180 <= longitude <= 180):
            raise ValueError("invalid_gga_position")
        station = str(message.stnID).split("*")[0]
        candidates = raw.get(str(message.time), [])
        matched = [f for f in candidates if len(f) >= 15
                   and int(f[6]) == int(message.qual)
                   and f[14] == station
                   and abs(coordinate(f[2], f[3]) - latitude) <= 1e-8
                   and abs(coordinate(f[4], f[5]) - longitude) <= 1e-8
                   and int(f[7]) == int(message.sats)
                   and abs(float(f[8]) - float(message.hdop)) <= 1e-6
                   and abs(float(f[13]) - float(message.age)) <= 1e-6]
        if len(matched) != 1:
            raise ValueError("raw_nmea_association_not_unique_or_missing")
        easting, northing = transformer.transform(longitude, latitude)
        if not all(math.isfinite(v) for v in (easting, northing)):
            raise ValueError("nonfinite_gga_projection")
        record[3:5] = [easting - origin["easting_m"],
                       northing - origin["northing_m"]]
    except (ValueError, IndexError) as error:
        record[9] = str(error)
    return record


def prepare(args):
    import yaml
    from pyproj import Transformer
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    metadata = json.loads(args.metadata.read_text())
    if digest(args.metadata) != args.metadata_sha256:
        raise ValueError("GNSS map metadata pin mismatch")
    reference = metadata["coordinate_reference"]
    origin = reference["projected_origin"]
    if reference["status"] != "horizontal_only" or origin["epsg"] != 32650:
        raise ValueError("only declared horizontal local UTM32650 supported")
    expected = [[1.0, 0.0, origin["easting_m"]],
                [0.0, 1.0, origin["northing_m"]], [0.0, 0.0, 1.0]]
    if reference["map_xy_to_projected_xy"]["matrix"] != expected:
        raise ValueError("map XY is not declared local projected offsets")
    if not math.isfinite(args.sigma_m) or args.sigma_m <= 0:
        raise ValueError("explicit positive uncalibrated diagnostic sigma required")
    profile = yaml.safe_load(args.profile.read_text())
    antenna = profile["gnss"]["antenna_extrinsic"]
    lidar = profile["extrinsics"]["T_I_L"]
    identity = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    if (antenna["rotation_matrix_row_major"] != identity
            or lidar["rotation_matrix_row_major"] != identity):
        raise ValueError("nominal lever composition requires parallel axes")
    lever = [a + b for a, b in zip(antenna["translation_m"],
                                  lidar["translation_m"])]
    with (args.input / "imu.csv").open() as stream:
        imu = list(csv.DictReader(stream))
    start, stop = int(imu[0]["receive_ns"]), int(imu[-1]["receive_ns"])
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    raw, raw_counts = nmea_index(args.nmea)
    events, sources = [], []
    with Reader(args.bag) as reader:
        connections = [c for c in reader.connections if c.topic == "/GPSGGA"]
        if len(connections) != 1:
            raise ValueError("one /GPSGGA connection required")
        connection = connections[0]
        store = get_typestore(Stores.ROS1_NOETIC)
        store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        for c, receive, data in reader.messages(connections=connections,
                                                start=start, stop=stop + 1):
            message = store.deserialize_ros1(data, c.msgtype)
            events.append(event(message, receive, len(events) + 1,
                                raw, transformer, origin))
            sources.append({"receive_ns": receive, "raw_message_hex": data.hex()})
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output / "events.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow("timestamp_ns,receive_ns,sequence,x,y,quality,satellites,"
                        "hdop,age,adapter_reason".split(","))
        writer.writerows(events)
    policy = [args.sigma_m, *lever, 6, 3.0, 10.0, 2_000_000_000, 2_000_000_000]
    with (args.output / "config.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow("sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,"
                        "max_age_s,dwell_ns,max_gap_ns,gnss_map_construction".split(","))
        writer.writerow([*policy, args.gnss_map_construction])
    with (args.output / "identity.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow("map_id,map_version,map_content_id,calibration_id,map_frame,"
                        "odom_frame,base_frame".split(","))
        writer.writerow([metadata.get("map_id") or metadata["version"],
                         metadata["version"], metadata["content_id"],
                         "mid360-manual-T_I_L", "supplied-prior-map",
                         "native-odom", "imu"])
    (args.output / "source_messages.json").write_text(json.dumps(sources, indent=2))
    (args.output / "embedded_schema.txt").write_text(connection.msgdef.data)
    manifest = {
        "schema": "openloc.native-gnss-diagnostic-input.v1",
        "diagnostic_only": True, "uncertainty_qualified": False,
        "receiver_accuracy_available": False, "heading_available": False,
        "sigma_m": args.sigma_m, "sigma_source": "explicit diagnostic assumption",
        "quality_policy": dict(zip(
            ("minimum_satellites", "maximum_hdop", "maximum_age_s",
             "fixed_stability_ns", "maximum_gap_ns"), policy[4:])),
        "measurement_time": "original GGA UTC TOD; nearest bag UTC day; no time shift",
        "receive_time": "original GPS bag record; not measurement time",
        "all_arrival_events_retained": True, "events": len(events),
        "adapter_reasons": dict(Counter(e[9] for e in events)),
        "receiver_quality_counts": dict(Counter(e[5] for e in events)),
        "bag_interval_ns": [start, stop],
        "bag": str(args.bag.resolve()), "bag_sha256": digest(args.bag),
        "nmea": str(args.nmea.resolve()), "nmea_sha256": digest(args.nmea),
        "nmea_counts": raw_counts, "metadata_sha256": args.metadata_sha256,
        "profile_sha256": digest(args.profile),
        "source_sha256": digest(Path(__file__)),
        "gnss_map_construction": args.gnss_map_construction,
        "reference": reference, "antenna_in_imu_m": lever,
        "lever_provenance": antenna,
        "limitations": ["Nominal lever is not independently surveyed per capture.",
                        "UTM grid XY is not exact ENU; no geodetic Z/attitude.",
                        "No calibrated receiver accuracy or absolute heading.",
                        "Bag arrival and UTC TOD do not prove hardware clock sync.",
                        "Same-source GNSS/map is not independent accuracy evidence."],
        "files": {p.name: digest(p) for p in sorted(args.output.iterdir())},
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"events": len(events), "adapter_reasons": manifest["adapter_reasons"]}))


def inject_bundle(input_dir, output, start_ns, stop_ns, shift_xy=(0.0, 0.0),
                  time_offset_ns=0, dropout=False):
    """Derive a diagnostic prepared-GNSS fault without modifying raw records."""
    if (not isinstance(start_ns, int) or not isinstance(stop_ns, int) or
            not isinstance(time_offset_ns, int) or not 0 < start_ns < stop_ns or
            len(shift_xy) != 2 or not all(math.isfinite(x) for x in shift_xy)):
        raise ValueError("fault requires integer times, valid interval and finite XY")
    if (dropout and (any(shift_xy) or time_offset_ns)) or (
            not dropout and not any(shift_xy) and not time_offset_ns):
        raise ValueError("select dropout or a nonzero measurement perturbation")
    manifest_path = input_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if (manifest["schema"] != "openloc.native-gnss-diagnostic-input.v1" or
            manifest["diagnostic_only"] is not True or
            manifest["uncertainty_qualified"] is not False):
        raise ValueError("fault input must be explicitly diagnostic")
    for relative, expected in manifest["files"].items():
        if Path(relative).name != relative or digest(input_dir / relative) != expected:
            raise ValueError("fault input artifact pin mismatch")
    with (input_dir / "events.csv").open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        original = list(reader)
    if len(original) != manifest["events"]:
        raise ValueError("fault input lost GNSS events")
    derived, mask = [], []
    for row in original:
        stamp = int(row["timestamp_ns"])
        changed = dict(row)
        if start_ns <= stamp < stop_ns:
            if not dropout:
                shifted = stamp + time_offset_ns
                if not 0 < shifted <= 2**63 - 1:
                    raise ValueError("shifted measurement outside int64 time")
                changed["timestamp_ns"] = str(shifted)
                for key, offset in zip(("x", "y"), shift_xy):
                    if offset:
                        value = float(row[key]) + offset
                        if not math.isfinite(value):
                            raise ValueError("nonfinite injected GNSS coordinate")
                        changed[key] = format(value, ".17g")
            mask.append({"original": row, "derived": None if dropout else changed})
            if dropout:
                continue
        derived.append(changed)
    if not mask:
        raise ValueError("fault interval did not trigger any GNSS event")
    output.mkdir(parents=True, exist_ok=False)
    for relative in manifest["files"]:
        if relative != "events.csv":
            shutil.copyfile(input_dir / relative, output / relative)
    with (output / "events.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(derived)
    (output / "fault_mask.json").write_text(json.dumps(mask, indent=2) + "\n")
    manifest.update(
        events=len(derived), all_arrival_events_retained=not dropout,
        measurement_time=("injected measurement-time offset; original in fault_mask.json"
                          if time_offset_ns else manifest.get("measurement_time")),
        fault_injection={
            "input_manifest_sha256": digest(manifest_path),
            "generator_sha256": digest(Path(__file__)),
            "interval_ns": [start_ns, stop_ns], "shift_xy_m": list(shift_xy),
            "time_offset_ns": time_offset_ns, "dropout": dropout,
            "affected_events": len(mask),
            "scope": "prepared GNSS owner-input boundary, not raw receiver or accuracy truth",
        },
        adapter_reasons=dict(Counter(row["adapter_reason"] for row in derived)),
        receiver_quality_counts=dict(Counter(row["quality"] for row in derived)),
        files={p.name: digest(p) for p in sorted(output.iterdir())},
    )
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main():
    if sys.argv[1:2] == ["inject"]:
        parser = argparse.ArgumentParser(description=inject_bundle.__doc__)
        parser.add_argument("--input", type=Path, required=True)
        parser.add_argument("--output", type=Path, required=True)
        parser.add_argument("--start-ns", type=int, required=True)
        parser.add_argument("--stop-ns", type=int, required=True)
        parser.add_argument("--shift-xy", nargs=2, type=float, default=(0.0, 0.0))
        parser.add_argument("--time-offset-ns", type=int, default=0)
        parser.add_argument("--dropout", action="store_true")
        args = parser.parse_args(sys.argv[2:])
        inject_bundle(args.input, args.output, args.start_ns, args.stop_ns,
                      args.shift_xy, args.time_offset_ns, args.dropout)
        return
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("bag", "nmea", "profile", "metadata", "input", "output"):
        parser.add_argument("--" + option, type=Path, required=True)
    parser.add_argument("--metadata-sha256", required=True)
    parser.add_argument("--sigma-m", type=float, required=True)
    parser.add_argument("--gnss-map-construction", choices=("present", "unknown"),
                        required=True)
    prepare(parser.parse_args())


if __name__ == "__main__":
    main()
