#!/usr/bin/env python3
"""Report native raw LIO execution, without claiming accuracy or qualification."""

import argparse
from collections import Counter
import csv
import html
import json
import math
import os
from pathlib import Path
import re
import statistics
import subprocess

from prepare_native_lio import digest


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def runtime_identity(binary):
    """Pin actual loader dependencies; refuse another Bazel configuration."""
    binary = binary.resolve(strict=True)
    with binary.open("rb") as stream:
        if stream.read(4) != b"\x7fELF":
            raise ValueError("runtime identity requires a built ELF binary")
    result = subprocess.run(["ldd", str(binary)], check=True,
                            capture_output=True, text=True)
    if "not found" in result.stdout:
        raise ValueError("runtime shared library not found")
    bazel_root = next((parent for parent in binary.parents
                       if parent.name == "bin" and parent.parent.parent.name == "bazel-out"),
                      None)
    libraries = {}
    for line in result.stdout.splitlines():
        found = re.match(r"\s*(\S+)\s+=>\s+(/\S+)\s+\(", line)
        if found:
            name, path = found.groups()
        else:
            found = re.match(r"\s*(/\S+)\s+\(", line)
            if not found:
                if "linux-vdso" in line or not line.strip():
                    continue
                raise ValueError("unrecognized runtime loader entry: " + line)
            path = found[1]
            name = Path(path).name
        resolved = Path(path).resolve(strict=True)
        if (bazel_root and "bazel-out" in resolved.parts and
                not resolved.is_relative_to(bazel_root)):
            raise ValueError("foreign Bazel runtime library: " + str(resolved))
        if name in libraries:
            raise ValueError("duplicate runtime shared library: " + name)
        libraries[name] = {"path": str(resolved), "sha256": digest(resolved)}
    if not libraries:
        raise ValueError("runtime shared library evidence is empty")
    return {
        "binary_sha256": digest(binary),
        "shared_libraries": libraries,
        "ld_library_path": os.environ.get("LD_LIBRARY_PATH"),
        "ld_preload": os.environ.get("LD_PRELOAD"),
        "scope": "loader resolution at inspection; not a full build closure",
    }


def summarize_timer_output(path):
    records = rows(path) if path.exists() else []
    last_usable = None
    previous = None
    lateness, intervals = [], []
    fresh, held = 0, 0
    for index, row in enumerate(records, 1):
        target, emitted = int(row["target_steady_ns"]), int(row["emitted_steady_ns"])
        skipped = int(row["skipped_periods"])
        maximum_age = int(row["maximum_age_ns"])
        age = int(row["measurement_age_ns"])
        usable, new, epoch_changed = (row[name] == "1"
                                      for name in ("usable", "new_measurement",
                                                   "epoch_changed"))
        if (int(row["publication_sequence"]) != index or emitted < target or
                not 0 < maximum_age <= 1_000_000_000 or
                skipped != (emitted - target) // 10_000_000 or
                row["diagnostic_only"] != "1" or row["trusted"] != "0" or
                any(row[name] not in ("0", "1")
                    for name in ("usable", "new_measurement", "epoch_changed"))):
            raise ValueError("invalid timer output identity or deadline")
        if previous:
            if target != int(previous["target_steady_ns"]) + (
                    int(previous["skipped_periods"]) + 1) * 10_000_000:
                raise ValueError("timer output burst or period change")
            intervals.append((emitted - int(previous["emitted_steady_ns"])) / 1e6)
        pose = [row[name] for name in
                ("px", "py", "pz", "qx", "qy", "qz", "qw", "vx", "vy", "vz")]
        if usable:
            key = (row["session"], row["generation"],
                   row["measurement_ns"], row["source_sequence"])
            changed = last_usable is not None and key[:2] != last_usable[:2]
            if (not 0 <= age <= maximum_age or
                    not all(value and math.isfinite(float(value)) for value in pose) or
                    int(row["measurement_ns"]) <= 0 or int(row["source_sequence"]) <= 0 or
                    not row["session"] or
                    new != (last_usable != key) or epoch_changed != changed):
                raise ValueError("timer output renewed or lost measurement identity")
            if (last_usable is not None and not changed and new and
                    (int(key[2]) <= int(last_usable[2]) or
                     int(key[3]) <= int(last_usable[3]))):
                raise ValueError("timer output source frontier regressed")
            last_usable = key
            fresh += new
            held += not new
        elif any(pose) or new or epoch_changed:
            raise ValueError("timer output published an unavailable pose")
        previous = row
        lateness.append((emitted - target) / 1e6)
    if path.exists() and not records:
        raise ValueError("timer output journal has no actual ticks")
    return {
        "journal_present": path.exists(), "publications": len(records),
        "new_measurements": fresh, "held_measurements": held,
        "unavailable_publications": len(records) - fresh - held,
        "skipped_periods": sum(int(row["skipped_periods"]) for row in records),
        "deadline_misses": sum(value > 10 for value in lateness),
        "lateness": latency(lateness), "wall_intervals": latency(intervals),
        "period_ns": 10_000_000,
        "maximum_age_ns": sorted({int(row["maximum_age_ns"]) for row in records}),
        "sensor_timestamps_preserved": True,
        "apollo_publication_qualified": False,
    }


def summarize_capability(path):
    records = rows(path) if path.exists() else []
    exceeded = [row for row in records
                if row["capability_reason"] == "CORRECTION_BUDGET_EXCEEDED"]
    return {
        "journal_present": path.exists(),
        "recorded_degraded_rows": len(records),
        "reasons": dict(Counter(row["capability_reason"] for row in records)),
        "first_output_envelope_exceedance": exceeded[0] if exceeded else None,
        "last_output_envelope_exceedance": exceeded[-1] if exceeded else None,
    }


def summarize_host_timing(path):
    records = rows(path) if path.exists() else []
    clock_path = path.with_name("replay_clock.csv")
    rate = 1
    if clock_path.exists():
        declaration = rows(clock_path)
        if (len(declaration) != 1 or declaration[0]["arrival_rate"] not in ("1", "2", "5") or
                declaration[0]["scope"] !=
                "wall_schedule_only_original_sensor_timestamps_preserved"):
            raise ValueError("invalid replay clock declaration")
        rate = int(declaration[0]["arrival_rate"])
        if records and declaration[0]["first_receive_ns"] != records[0]["receive_ns"]:
            raise ValueError("replay clock origin changed")
    valid = [r for r in records if r["valid"] == "1"]
    previous = None
    lateness = []
    intervals = []
    receive_intervals = []
    for row in records:
        values = {name: int(value) for name, value in row.items()}
        if values["deadline_ns"] != 10_000_000 or values["emitted_steady_ns"] < values["target_steady_ns"]:
            raise ValueError("invalid paced host timing")
        if previous is not None and (
                values["timestamp_ns"] <= previous["timestamp_ns"] or
                values["receive_ns"] < previous["receive_ns"] or
                values["emitted_steady_ns"] < previous["emitted_steady_ns"] or
                values["target_steady_ns"] - int(records[0]["target_steady_ns"]) !=
                (values["receive_ns"] - int(records[0]["receive_ns"])) // rate):
            raise ValueError("paced host chronology changed")
        if row["valid"] == "1":
            lateness.append(values["emitted_steady_ns"] - values["target_steady_ns"])
            if previous is not None and previous["valid"] == 1:
                intervals.append(values["emitted_steady_ns"] - previous["emitted_steady_ns"])
                receive_intervals.append(values["receive_ns"] - previous["receive_ns"])
        previous = values
    misses = sum(value > 10_000_000 for value in lateness)
    gaps = sum(value > 10_000_000 for value in intervals)
    return {
        "journal_present": path.exists(), "emitted_rows": len(records),
        "arrival_rate": rate,
        "clock_scope": "wall schedule only; source timestamps unchanged",
        "initialized_emissions": len(valid),
        "deadline_ns": 10_000_000,
        "initialized_deadline_misses": misses,
        "initialized_wall_gaps_over_10ms": gaps,
        "initialized_receive_gaps_over_10ms": sum(
            value > 10_000_000 for value in receive_intervals),
        "initialized_wall_gaps_with_receive_gap_over_10ms": sum(
            emitted > 10_000_000 and received > 10_000_000
            for emitted, received in zip(intervals, receive_intervals)),
        "initialized_wall_gaps_with_receive_gap_at_most_10ms": sum(
            emitted > 10_000_000 and received <= 10_000_000
            for emitted, received in zip(intervals, receive_intervals)),
        "initialized_lateness": latency(value / 1e6 for value in lateness),
        "initialized_wall_intervals": latency(value / 1e6 for value in intervals),
        "initialized_receive_intervals": latency(
            value / 1e6 for value in receive_intervals),
        "paced_replay_10ms_check_passed": bool(valid) and misses == 0 and gaps == 0,
        "apollo_publication_qualified": False,
    }


def summarize_global_scheduler(path, imu_samples, valid_states, scans):
    records = rows(path) if path.exists() else []
    if not records:
        if path.exists():
            raise ValueError("empty global scheduler journal")
        return {"journal_present": False}
    previous_finish = 0
    waits = []
    durations = []
    for identifier, row in enumerate(records, 1):
        start = int(row["start_steady_ns"])
        finish = int(row["finish_steady_ns"])
        enqueue = int(row["enqueue_steady_ns"])
        if (int(row["task_id"]) != identifier or row["status"] != "completed" or
                not enqueue <= start <= finish or start < previous_finish or
                not 1 <= int(row["admitted_tasks"]) <= 256 or
                not 0 <= int(row["payload_bytes"]) <= int(row["admitted_bytes"]) <= 64 * 1024 * 1024):
            raise ValueError("global scheduler bounds/order/completion invariant violated")
        waits.append((start - enqueue) / 1e6)
        durations.append((finish - start) / 1e6)
        previous_finish = finish
    counts = dict(Counter(row["kind"] for row in records))
    if counts != {"odometry": valid_states, "gnss": imu_samples,
                  "drain": imu_samples, "scan": scans, "finish": 1}:
        raise ValueError("global scheduler lost or duplicated native events")
    return {
        "journal_present": True, "completed_tasks": len(records),
        "tasks_by_kind": counts, "maximum_outstanding_tasks": max(
            int(r["admitted_tasks"]) for r in records),
        "maximum_outstanding_payload_bytes": max(
            int(r["admitted_bytes"]) for r in records),
        "task_capacity": 256, "payload_capacity_bytes": 64 * 1024 * 1024,
        "queue_wait": latency(waits), "task_duration": latency(durations),
        "single_global_worker": True,
    }


def summarize_local_lidar_tasks(path, expected_scans):
    records = rows(path) if path.exists() else []
    if not path.exists():
        return {"journal_present": False}
    active = {}
    terminal = {}
    for row in records:
        sequence = int(row["sequence"])
        reference = int(row["reference_ns"])
        frontier = int(row["processing_ns"])
        due = int(row["due_ns"])
        status = row["status"]
        if not 5_000_000 <= due - reference <= 100_000_000 or frontier < reference:
            raise ValueError("invalid local LIO source-time budget")
        if status == "queued":
            if sequence in active or sequence in terminal:
                raise ValueError("duplicate local LIO task")
            active[sequence] = (reference, due)
        elif status in ("rejected_busy", "prepare_rejected"):
            if sequence in active or sequence in terminal:
                raise ValueError("duplicate local LIO rejection")
            terminal[sequence] = status
        elif status == "discarded_after_deadline":
            if terminal.get(sequence) != "deadline_missed":
                raise ValueError("unaccounted expired local LIO task")
        else:
            if active.pop(sequence, None) != (reference, due) or sequence in terminal:
                raise ValueError("local LIO result does not match its snapshot")
            if status not in ("committed", "deadline_missed", "compute_rejected",
                              "commit_rejected", "end_of_input_uncommitted"):
                raise ValueError("unknown local LIO task outcome")
            if status == "committed" and (
                    frontier < due or row["reason"] not in ("NONE", "GEOMETRY_UNAVAILABLE")):
                raise ValueError("invalid delayed local LIO commit")
            terminal[sequence] = status
    if active or len(terminal) != expected_scans:
        raise ValueError("local LIO task journal lost a scan or terminal result")
    return {
        "journal_present": True, "runtime_scan_inputs": expected_scans,
        "terminal_outcomes": dict(Counter(terminal.values())),
        "accepted_corrections": sum(r["status"] == "committed" and
                                    r["reason"] == "NONE" for r in records),
        "source_time_delays_ms": sorted({
            (int(r["due_ns"]) - int(r["reference_ns"])) / 1e6 for r in records}),
        "one_outstanding_snapshot": True, "imu_catchup_capacity": 128,
        "published_history_rewritten": False,
        "uncertainty_qualified": False,
    }


def summarize_lidar_decode(path, decisions):
    records = rows(path) if path.exists() else []
    if not path.exists():
        return {"journal_present": False}
    consumed = [r for r in records if r["status"] == "consumed"]
    scan_rows = {r["sequence"]: r for r in decisions}
    previous = 0
    for row in records:
        if row["status"] not in ("consumed", "not_ready",
                                  "skipped_before_local_ready", "decoded_not_consumed"):
            raise ValueError("unknown scan decoder outcome")
        if not math.isfinite(float(row["decoding_ms"])) or float(row["decoding_ms"]) < 0:
            raise ValueError("invalid scan decoding duration")
        if row["status"] == "consumed":
            sequence = int(row["sequence"])
            decision = scan_rows.get(row["sequence"])
            if (sequence <= previous or int(row["points"]) <= 0 or
                    decision is None or
                    int(row["frontier_ns"]) != int(decision["reference_ns"]) or
                    int(decision["reference_ns"]) < int(decision["raw_end_ns"]) or
                    int(decision["processing_frontier_receive_ns"]) <
                    int(decision["source_receive_ns"])):
                raise ValueError("prefetched points violated scan chronology")
            previous = sequence
    return {
        "journal_present": True, "consumed_scans": len(consumed),
        "readiness_deferrals": sum(r["status"] == "not_ready" for r in records),
        "outcomes": dict(Counter(r["status"] for r in records)),
        "decoding_latency": latency(float(r["decoding_ms"]) for r in consumed),
        "prefetch_slots": 1, "estimator_consumption_arrival_gated": True,
    }


def summarize_local_corrections(records):
    accepted = [r for r in records if r["reason"] == "NONE"]
    cumulative = [0.0, 0.0, 0.0]
    maximum_gap = 0.0
    total_path = 0.0
    maximum_velocity_correction = 0.0
    for row in records:
        values = {name: float(value) for name, value in row.items()
                  if name not in ("sequence", "reference_ns", "reason")}
        if any(not math.isfinite(value) for value in values.values()):
            raise ValueError("nonfinite local correction diagnostic")
        delta = [values["position_d" + a] for a in "xyz"]
        velocity_delta = [values["velocity_d" + a] for a in "xyz"]
        if row["reason"] != "NONE" and any(delta + velocity_delta):
            raise ValueError("rejected LiDAR changed local position or velocity")
        total_path += math.sqrt(sum(x * x for x in delta))
        cumulative = [x + y for x, y in zip(cumulative, delta)]
        maximum_gap = max(maximum_gap, math.sqrt(sum(
            (values["internal_p" + a] - values["output_p" + a]) ** 2
            for a in "xyz")))
        maximum_velocity_correction = max(
            maximum_velocity_correction,
            math.sqrt(sum(x * x for x in velocity_delta)))
    return {
        "recorded_scans": len(records), "accepted_corrections": len(accepted),
        "net_position_correction_m": cumulative,
        "total_position_correction_path_m": total_path,
        "maximum_internal_output_translation_gap_m": maximum_gap,
        "maximum_velocity_correction_mps": maximum_velocity_correction,
        "independent_truth_error": False,
    }


def summarize_imu_trace(records, corrections):
    if not records:
        return {"recorded_propagations": 0, "independent_truth_error": False}
    total_dt = 0.0
    impulse = [0.0] * 3
    displacement = [0.0] * 3
    previous = None
    for row in records:
        values = {name: float(value) for name, value in row.items()}
        if any(not math.isfinite(value) for value in values.values()):
            raise ValueError("nonfinite IMU trace")
        dt = values["dt_s"]
        if dt <= 0 or (previous is not None and (
                values["time_s"] <= previous["time_s"] or
                int(row["sequence"]) <= int(previous["sequence"]) or
                abs(values["time_s"] - previous["time_s"] - dt) > 1e-6)):
            raise ValueError("noncausal IMU trace")
        total_dt += dt
        for axis, name in enumerate("xyz"):
            dv = values["velocity_after_" + name] - values["velocity_before_" + name]
            dp = values["position_after_" + name] - values["position_before_" + name]
            a = values["acceleration_" + name]
            if (abs(dv - a * dt) > 1e-9 or
                    abs(dp - values["velocity_before_" + name] * dt -
                        0.5 * a * dt ** 2) > 1e-9):
                raise ValueError("IMU trace violates propagation equations")
            impulse[axis] += dv
            displacement[axis] += dp
        previous = values
    last = records[-1]
    final_position = [float(last["position_after_" + a]) for a in "xyz"]
    final_velocity = [float(last["velocity_after_" + a]) for a in "xyz"]
    if corrections and int(corrections[-1]["reference_ns"]) / 1e9 >= float(last["time_s"]):
        final_position = [float(corrections[-1]["internal_p" + a]) for a in "xyz"]
        final_velocity = [float(corrections[-1]["internal_v" + a]) for a in "xyz"]
    initial_position = [float(records[0]["position_before_" + a]) for a in "xyz"]
    initial_velocity = [float(records[0]["velocity_before_" + a]) for a in "xyz"]
    net_dp = [sum(float(r["position_d" + a]) for r in corrections) for a in "xyz"]
    net_dv = [sum(float(r["velocity_d" + a]) for r in corrections) for a in "xyz"]
    residual_position = [p1 - p0 - dp - lidar for p1, p0, dp, lidar in zip(
        final_position, initial_position, displacement, net_dp)]
    residual_velocity = [v1 - v0 - dv - lidar for v1, v0, dv, lidar in zip(
        final_velocity, initial_velocity, impulse, net_dv)]
    if max(map(abs, residual_position + residual_velocity)) > 1e-8:
        raise ValueError("IMU/LiDAR trace leaves an unexplained state change")
    return {
        "recorded_propagations": len(records), "duration_s": total_dt,
        "imu_velocity_impulse_mps": impulse,
        "imu_position_displacement_m": displacement,
        "lio_velocity_impulse_mps": net_dv,
        "lio_position_displacement_m": net_dp,
        "mean_linear_acceleration_mps2": [x / total_dt for x in impulse],
        "initial_internal_velocity_mps": initial_velocity,
        "final_internal_velocity_mps": final_velocity,
        "position_accounting_residual_m": residual_position,
        "velocity_accounting_residual_mps": residual_velocity,
        "bias_ranges": {
            prefix + axis: {
                "min": min(float(r[prefix + axis]) for r in records),
                "max": max(float(r[prefix + axis]) for r in records),
                "last": float(records[-1][prefix + axis]),
            }
            for prefix in ("gyro_bias_", "accel_bias_") for axis in "xyz"
        },
        "independent_truth_error": False,
    }


def imu_trace_svg(records):
    if not records:
        return "<p>IMU trace disabled; enable --imu-trace for drift diagnostics.</p>"
    selected = records[::max(1, len(records) // 1000)]
    if selected[-1] is not records[-1]:
        selected.append(records[-1])
    start = float(records[0]["time_s"])
    duration = max(float(records[-1]["time_s"]) - start, 1e-9)
    panels = []
    for prefix, label in (
        ("acceleration_", "Internal linear acceleration (m/s^2)"),
        ("velocity_after_", "Internal velocity (m/s)"),
        ("accel_bias_", "Accelerometer bias (m/s^2)"),
        ("gyro_bias_", "Gyroscope bias (rad/s)"),
    ):
        maximum = max(abs(float(r[prefix + a])) for r in selected for a in "xyz")
        scale = 100 / max(maximum, 1e-9)
        lines = []
        for axis, color in zip("xyz", ("#1864ab", "#d9480f", "#2b8a3e")):
            points = " ".join(
                f'{30 + (float(r["time_s"]) - start) * 480 / duration:.2f},'
                f'{140 - float(r[prefix + axis]) * scale:.2f}' for r in selected)
            lines.append(f'<polyline points="{points}" fill="none" stroke="{color}"/>')
        panels.append(
            '<svg viewBox="0 0 580 290" width="580" height="290">'
            '<path d="M30 140 H510" stroke="#888"/>'
            + "".join(lines)
            + f'<text x="15" y="20">{label}; +/-{maximum:.4g}</text>'
            '<text x="15" y="265">X blue; Y orange; Z green; '
            'uncalibrated internal state, not truth</text></svg>')
    return "".join(panels)


def latency(samples):
    values = sorted(float(value) for value in samples)
    if not values:
        return None
    return {
        "count": len(values),
        "median_ms": statistics.median(values),
        "p95_ms": values[int(0.95 * (len(values) - 1))],
        "max_ms": values[-1],
    }


def summarize(states, decisions):
    if not states or not decisions:
        raise ValueError("native state and scan evidence are required")
    valid = [row for row in states if row["valid"] == "1"]
    times = [int(row["timestamp_ns"]) for row in valid]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("valid ODOM measurement times must increase")
    runtime = [row for row in decisions if row["bootstrap_stage"] == "0"]
    return {
        "imu_samples": len(states),
        "valid_states": len(valid),
        "initialization_delay_s": (
            (times[0] - int(states[0]["timestamp_ns"])) / 1e9 if times else None
        ),
        "valid_measurement_span_s": (
            (times[-1] - times[0]) / 1e9 if times else None
        ),
        "max_valid_measurement_gap_ms": (
            max(b - a for a, b in zip(times, times[1:])) / 1e6
            if len(times) > 1 else None
        ),
        "covariance_model_valid_states": sum(
            row["covariance_model_valid"] == "1" for row in valid
        ),
        "motion_available_states": sum(
            row["motion_available"] == "1" for row in valid
        ),
        "valid_state_capability_reasons": dict(Counter(
            row.get("capability_reason", "not_recorded") for row in valid
        )),
        "runtime_scan_decisions": dict(Counter(row["reason"] for row in runtime)),
        "runtime_scan_submissions": sum(
            row.get("detail") != "Scan overlaps pre-initialization history"
            for row in runtime),
        "startup_scan_decisions": dict(Counter(
            row["reason"] for row in decisions if row["bootstrap_stage"] == "1"
        )),
        "valid_imu_call_latency": latency(row["imu_ms"] for row in valid),
        "runtime_scan_latency": latency(row["processing_ms"] for row in runtime),
        "startup_scan_latency": latency(
            row["processing_ms"] for row in decisions
            if row["bootstrap_stage"] == "1"
        ),
    }


def trajectory_svg(states):
    positions = [(float(row["px"]), float(row["py"])) for row in states
                 if row["valid"] == "1"]
    if not positions:
        return "<p>No initialized native trajectory.</p>"
    xmin = min(x for x, _ in positions)
    xmax = max(x for x, _ in positions)
    ymin = min(y for _, y in positions)
    ymax = max(y for _, y in positions)
    scale = 480 / max(xmax - xmin, ymax - ymin, 1e-6)
    points = " ".join(
        f"{30 + (x - xmin) * scale:.2f},{510 - (y - ymin) * scale:.2f}"
        for x, y in positions
    )
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 560 550" '
        'width="560" height="550">'
        f'<polyline points="{points}" fill="none" stroke="#1864ab"/>'
        f'<text x="15" y="535">Local ODOM XY, equal scale; '
        f'X span {xmax - xmin:.3f} m; Y span {ymax - ymin:.3f} m</text></svg>'
    )


def summarize_initialization(records):
    evaluated = [row for row in records
                 if row["linear_system_evaluated"] == "1"]
    solved = [row for row in records if row["solution_available"] == "1"]
    ranges = {}
    for name, selected in (
        ("minimum_scaled_singular_value", evaluated),
        ("gravity_error_mps2", solved), ("accel_bias_mps2", solved),
        ("gyro_bias_radps", solved), ("normalized_rms", solved),
        ("angular_rms_radps", solved),
    ):
        values = [float(row[name]) for row in selected]
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"nonfinite startup diagnostic: {name}")
        ranges[name] = (
            {"min": min(values), "max": max(values),
             "median": statistics.median(values)} if values else None
        )
    return {
        "recorded_scan_decisions": len(records),
        "linear_systems_evaluated": len(evaluated),
        "solutions_available": len(solved),
        "lidar_stationary_initializations": sum(
            row.get("lidar_stationary_initialization") == "1" for row in records
        ),
        "solution_decisions": dict(Counter(row["reason"] for row in solved)),
        "ranges": ranges,
        "uncertainty_qualified": False,
    }


def initialization_svg(records):
    solved = [row for row in records if row["solution_available"] == "1"]
    if not solved:
        return "<p>No numerical motion-initialization solutions recorded.</p>"
    times = [int(row["reference_ns"]) for row in solved]
    duration = max(times[-1] - times[0], 1)
    maximum = max(float(row[name]) for row in solved
                  for name in ("gravity_error_mps2", "accel_bias_mps2"))
    scale = 240 / max(maximum, 1e-6)
    lines = []
    for name, color in (("gravity_error_mps2", "#1864ab"),
                        ("accel_bias_mps2", "#d9480f")):
        points = " ".join(
            f"{30 + (time - times[0]) * 480 / duration:.2f},"
            f"{270 - float(row[name]) * scale:.2f}"
            for time, row in zip(times, solved)
        )
        lines.append(f'<polyline points="{points}" fill="none" stroke="{color}"/>')
    return (
        '<svg viewBox="0 0 580 330" width="580" height="330">'
        + "".join(lines)
        + f'<text x="15" y="20">Startup solutions; vertical 0..{maximum:.3f} m/s^2'
        '</text><text x="15" y="295">Blue: gravity norm error; orange: accel bias norm'
        f'</text><text x="15" y="320">Elapsed {duration / 1e9:.3f} s; '
        'numerical diagnostics, not accuracy</text></svg>'
    )


def report(input_dir, replay_dir, output, binary, execution_runtime=None):
    manifest_path = input_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    for relative, expected in manifest["files"].items():
        if digest(input_dir / relative) != expected:
            raise ValueError(f"input hash mismatch: {relative}")
    states = rows(replay_dir / "odometry.csv")
    decisions = rows(replay_dir / "lidar.csv")
    summary = summarize(states, decisions)
    summary["host_timing"] = summarize_host_timing(replay_dir / "host_timing.csv")
    summary["timer_output"] = summarize_timer_output(replay_dir / "timer_output.csv")
    summary["local_lidar_tasks"] = summarize_local_lidar_tasks(
        replay_dir / "local_lidar_tasks.csv", summary["runtime_scan_submissions"])
    summary["lidar_decode"] = summarize_lidar_decode(
        replay_dir / "lidar_decode.csv", decisions)
    summary["capability_diagnostics"] = summarize_capability(
        replay_dir / "capability.csv")
    corrections_path = replay_dir / "local_corrections.csv"
    corrections = rows(corrections_path) if corrections_path.exists() else []
    summary["local_corrections"] = summarize_local_corrections(corrections)
    summary["local_corrections"]["journal_present"] = corrections_path.exists()
    trace_path = replay_dir / "imu_trace.csv"
    trace = rows(trace_path) if trace_path.exists() else []
    summary["imu_trace"] = summarize_imu_trace(trace, corrections)
    summary["imu_trace"]["journal_present"] = trace_path.exists()
    initialization_path = replay_dir / "motion_initialization.csv"
    initialization = rows(initialization_path) if initialization_path.exists() else []
    summary["motion_initialization"] = summarize_initialization(initialization)
    summary["motion_initialization"]["journal_present"] = initialization_path.exists()
    evidence = {
        "schema": "openloc.native-lio-execution-report.v1",
        "diagnostic_only": True,
        "input_manifest_sha256": digest(manifest_path),
        "binary_sha256": digest(binary),
        "execution_runtime_identity": execution_runtime,
        "selected_source_sha256": {
            name: digest(Path(__file__).resolve().parents[3] / name)
            for name in (
                "src/localization/candidates/common/types.h",
                "include/openloc/localization/local/native_types.h",
                "include/openloc/localization/local/motion_initialization_types.h",
                "include/openloc/localization/local/native_odometry.h",
                "src/localization/local/native_odometry.cc",
                "include/openloc/localization/local/native_motion_bootstrap.h",
                "src/localization/local/native_motion_bootstrap.cc",
                "src/localization/local/bootstrap_deskew.h",
                "src/localization/candidates/local/local_estimator.cc",
                "src/localization/candidates/local/local_lio.cc",
                "src/localization/candidates/local/continuous_output.h",
                "src/localization/candidates/local/continuous_output.cc",
                "src/localization/candidates/local/motion_initializer.cc",
                "test/integration/map/native_lio_replay.cc",
                "test/integration/map/native_diagnostic_executor.h",
                "test/integration/map/native_lidar_scheduler.h",
                "test/integration/map/native_scan_reader.h",
                "test/integration/map/native_timer_output.h",
                "test/integration/map/native_lio_report.py",
            )
        },
        "replay_sha256": {
            name: digest(replay_dir / name)
            for name in ("odometry.csv", "lidar.csv", "summary.csv",
                         "motion_initialization.csv", "capability.csv",
                         "local_corrections.csv", "imu_trace.csv",
                         "host_timing.csv", "global_scheduler.csv",
                         "local_lidar_tasks.csv", "lidar_decode.csv", "timer_output.csv")
            if (replay_dir / name).exists()
        },
        "metrics": summary,
        "limitations": [
            "Native LocalEstimator/LocalLio, not external odometry.",
            "No truth, accuracy comparison, global alignment or prior-map result.",
            "Covariance eligibility is reported, never promoted by this report.",
            "Measurement frequency is not wall-clock publication frequency.",
            "Synchronous scan work can block IMU calls; no 100 Hz deadline proof.",
            "No qualified covariance ellipses or factor graph are available.",
            "Selected source/binary hashes are not a frozen dependency closure.",
            *manifest["limitations"],
        ],
    }
    output.mkdir(parents=True, exist_ok=False)
    (output / "report.json").write_text(json.dumps(evidence, indent=2) + "\n")
    (output / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><title>Native LIO diagnostic</title>'
        "<h1>Native LIO execution diagnostic</h1>"
        "<p>Local ODOM only; no truth or global-map comparison. "
        "This is not accuracy, integrity, or real-time qualification.</p>"
        + trajectory_svg(states)
        + "<h2>Motion initialization consistency</h2>"
        + initialization_svg(initialization)
        + "<h2>IMU and LiDAR state accounting</h2>"
        + imu_trace_svg(trace)
        + "<h2>Metrics</h2><pre>"
        + html.escape(json.dumps(summary, indent=2))
        + "</pre><h2>Limitations</h2><pre>"
        + html.escape(json.dumps(evidence["limitations"], indent=2))
        + "</pre>"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    args = parser.parse_args()
    report(args.input, args.replay, args.output, args.binary)


if __name__ == "__main__":
    main()
