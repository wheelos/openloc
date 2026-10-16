#!/usr/bin/env python3
"""Separate planar wheel odometry diagnostics from audited scalar CSVs."""

import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def advance(x, y, yaw, speed, rate, dt):
    half = 0.5 * rate * dt
    distance = speed * dt * (math.sin(half) / half if abs(half) > 1e-12 else 1.0)
    return x + distance * math.cos(yaw + half), y + distance * math.sin(yaw + half), yaw + 2 * half


class Series:
    def __init__(self, times, values, max_gap):
        if not times or len(times) != len(values):
            raise ValueError("Empty or mismatched series")
        if any(not math.isfinite(t) for t in times):
            raise ValueError("Non-finite sensor time")
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError("Series times must be strictly increasing")
        if any(not all(math.isfinite(v) for v in row) for row in values):
            raise ValueError("Non-finite sensor values")
        self.times, self.values, self.max_gap = times, values, max_gap

    def at(self, time, angle_column=None):
        index = bisect.bisect_left(self.times, time)
        if index < len(self.times) and self.times[index] == time:
            return self.values[index]
        if index == 0 or index == len(self.times):
            return None
        a, b = index - 1, index
        span = self.times[b] - self.times[a]
        if span > self.max_gap:
            return None
        weight = (time - self.times[a]) / span
        result = []
        for column, (left, right) in enumerate(zip(self.values[a], self.values[b])):
            delta = wrap(right - left) if column == angle_column else right - left
            result.append(left + weight * delta)
        return result


def load(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def statistics(values):
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    return {
        "count": len(values),
        "median": ordered[len(values) // 2],
        "p95": ordered[math.ceil(0.95 * len(values)) - 1],
        "max": ordered[-1],
        "rmse": math.sqrt(sum(v * v for v in values) / len(values)),
    }


def eligible_intervals(speed, gyro, max_gap):
    for begin, end in zip(speed.times, speed.times[1:]):
        count = max(1, math.ceil((end - begin) / 0.005))
        grid = [begin + (end - begin) * step / count for step in range(count + 1)]
        inputs = [
            (speed.at(time), gyro.at(time)) for time in grid
        ] if end - begin <= max_gap else []
        if any(a is None or b is None for a, b in inputs):
            inputs = []
        yield begin, end, grid, inputs


def replay(chassis_path, imu_path, pose_path, coefficient, output_dir, max_gap=0.05):
    if not math.isfinite(coefficient) or coefficient <= 0:
        raise ValueError("A positive frozen steering coefficient is required")
    if not math.isfinite(max_gap) or max_gap <= 0:
        raise ValueError("Invalid maximum input gap")
    chassis, imu, poses = load(chassis_path), load(imu_path), load(pose_path)
    speed = Series(
        [float(row["header_time"]) for row in chassis],
        [[float(row["speed"]), float(row["steering_percent"])] for row in chassis],
        max_gap,
    )
    if any(row[0] < 0 for row in speed.values):
        raise ValueError("Forward-only diagnostic does not infer reverse direction from gear")
    gyro = Series(
        [float(row["measurement_time"]) + 315964782.0 for row in imu],
        [[float(row["gz"])] for row in imu],
        max_gap,
    )
    unique = []
    duplicates = 0
    for row in poses:
        if unique and row["measurement_time"] == unique[-1]["measurement_time"]:
            if any(row[k] != unique[-1][k] for k in ("px", "py", "heading")):
                raise ValueError("Conflicting duplicate reference pose")
            duplicates += 1
        else:
            unique.append(row)
    reference = Series(
        [float(row["measurement_time"]) for row in unique],
        [[float(row[k]) for k in ("px", "py", "heading")] for row in unique],
        max_gap,
    )
    summaries = {}
    output_dir.mkdir(parents=True, exist_ok=True)
    for mode in ("wheel_raw_gyro", "wheel_transferred_steering"):
        segments, rows, rejected = [], [], []
        active = None
        for begin, end, grid, inputs in eligible_intervals(speed, gyro, max_gap):
            count = len(grid) - 1
            if not inputs:
                rejected.append({"begin": begin, "end": end, "reason": "input_gap_or_unbracketed"})
                active = None
                continue
            if active is None:
                origin = reference.at(begin, 2)
                if origin is None:
                    rejected.append({"begin": begin, "end": end, "reason": "no_reference_for_explicit_segment_seed"})
                    continue
                x = y = 0.0
                yaw = origin[2]
                active = {
                    "segment": len(segments), "start": begin, "end": begin,
                    "origin": origin, "wheel_distance_m": 0.0, "reference_distance_m": 0.0,
                    "previous_reference": origin, "errors": [], "yaw_errors": [],
                    "scored": 0, "unscored": 0,
                }
                segments.append(active)
            for step in range(count):
                left, right = inputs[step], inputs[step + 1]
                v = 0.5 * (left[0][0] + right[0][0])
                if mode == "wheel_raw_gyro":
                    rate = 0.5 * (left[1][0] + right[1][0])
                else:
                    rate = 0.5 * coefficient * (
                        left[0][0] * left[0][1] + right[0][0] * right[0][1]
                    ) / 100.0
                dt = grid[step + 1] - grid[step]
                x, y, yaw = advance(x, y, yaw, v, rate, dt)
                active["wheel_distance_m"] += v * dt
            active["end"] = end
            ref = reference.at(end, 2)
            row = {
                "segment": active["segment"], "time": end,
                "wheel_distance_m": active["wheel_distance_m"],
                "local_x": x, "local_y": y, "yaw": wrap(yaw),
                "reference_x": "", "reference_y": "", "horizontal_error_m": "",
                "heading_error_deg": "",
            }
            if ref is None:
                active["unscored"] += 1
                active["previous_reference"] = None
            else:
                rx, ry = ref[0] - active["origin"][0], ref[1] - active["origin"][1]
                error = math.hypot(x - rx, y - ry)
                yaw_error = math.degrees(wrap(yaw - ref[2]))
                active["errors"].append(error)
                active["yaw_errors"].append(abs(yaw_error))
                active["final_error_m"] = error
                active["final_heading_error_deg"] = yaw_error
                previous = active["previous_reference"]
                if previous is not None:
                    active["reference_distance_m"] += math.hypot(ref[0] - previous[0], ref[1] - previous[1])
                active["previous_reference"] = ref
                active["scored"] += 1
                row.update(reference_x=rx, reference_y=ry, horizontal_error_m=error, heading_error_deg=yaw_error)
            rows.append(row)
        if not rows:
            raise ValueError("No eligible wheel odometry intervals")
        with (output_dir / f"{mode}.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        for segment in segments:
            segment["duration_s"] = segment["end"] - segment["start"]
            segment["horizontal_error_m"] = statistics(segment.pop("errors"))
            segment["absolute_heading_error_deg"] = statistics(segment.pop("yaw_errors"))
            selected = [
                row for row in rows
                if row["segment"] == segment["segment"]
                and row["horizontal_error_m"] != ""
            ]
            segment["distance_checkpoints"] = []
            for distance in (50, 100, 200, 300, 400, 500):
                checkpoint = next(
                    (row for row in selected if row["wheel_distance_m"] >= distance),
                    None,
                )
                if checkpoint is not None:
                    segment["distance_checkpoints"].append({
                        "requested_distance_m": distance,
                        "actual_wheel_distance_m": checkpoint["wheel_distance_m"],
                        "elapsed_s": checkpoint["time"] - segment["start"],
                        "horizontal_error_m": checkpoint["horizontal_error_m"],
                        "heading_error_deg": checkpoint["heading_error_deg"],
                    })
            segment.pop("previous_reference")
        summaries[mode] = {
            "segments": segments, "rejected_intervals": rejected,
            "scored_samples": sum(s["scored"] for s in segments),
            "unscored_samples": sum(s["unscored"] for s in segments),
            "eligible_duration_s": sum(s["duration_s"] for s in segments),
            "longest_segment": max(segments, key=lambda s: s["duration_s"])["segment"],
        }
    summaries["protocol"] = {
        "steering_coefficient": coefficient, "max_gap_s": max_gap,
        "reference_duplicate_rows_removed": duplicates,
        "reference_time": "pose measurement_time",
        "imu_time": "GPS payload +315964782 s",
        "chassis_time": "header_time",
        "heading": "raw IMU Z treated as planar yaw rate; no gyro bias or tilt fit",
        "initialization": "one reference position/yaw per explicitly reported input-continuous segment",
        "integration": "endpoint-average speed/rate, exact constant-twist step, substep <=5ms",
        "limits": [
            "Diagnostic wheel odometry, not LocalEstimator replay or calibrated Ackermann model.",
            "No target-record scale, heading, time-offset or steering fit.",
            "Unknown vehicle geometry, encoder reference and lever arms; planar forward-only assumptions.",
            "Localization reference may share IMU/GNSS data; not independent truth.",
            "Gear invalidity in demo is retained as an explicit diagnostic exception, not production admission.",
            "Interpolation requires future bracket; this is offline sensor-time integration, not online latency validation.",
        ],
    }
    summaries["sha256"] = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (chassis_path, imu_path, pose_path, Path(__file__))
    }
    summaries["output_sha256"] = {
        path.name: hashlib.sha256(path.read_bytes()).hexdigest()
        for path in output_dir.glob("wheel_*.csv")
    }
    (output_dir / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("chassis", "imu", "pose", "output-dir"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--steering-coefficient", type=float, required=True)
    args = parser.parse_args()
    replay(args.chassis, args.imu, args.pose, args.steering_coefficient, args.output_dir)


if __name__ == "__main__":
    main()
