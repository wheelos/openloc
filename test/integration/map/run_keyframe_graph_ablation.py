#!/usr/bin/env python3
"""Causal fixed-lag controls for a declared model, not accuracy certification."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess


ARMS = ("lio", "gnss", "map", "all")
INPUTS = ("nodes.csv", "measurements.csv", "model.csv")


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def read(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def resource_summary(events):
    return {
        key: max(int(event[key]) for event in events)
        for key in ("buffered_nodes", "graph_nodes", "factor_slots",
                    "frontier_rows", "frontier_dimension", "archived_values",
                    "solve_iterations", "damped_iterations")
    }


def timing_summary(events):
    values = sorted(float(event["elapsed_ms"]) for event in events)
    if any(not math.isfinite(value) or value < 0 for value in values):
        raise ValueError("invalid graph event timing")
    receives = [int(event["receive_ns"]) for event in events]
    span = (receives[-1] - receives[0]) * 1e-6
    return {
        "event_count": len(values),
        "event_p50_ms": values[(len(values) - 1) // 2],
        "event_p95_ms": values[math.ceil(0.95 * len(values)) - 1],
        "event_max_ms": values[-1],
        "summed_event_ms": sum(values),
        "receive_span_ms": span,
        "processing_to_receive_span_ratio": sum(values) / span if span > 0 else None,
        "scope": "unpaced isolated graph events, not full pipeline or deadline proof",
    }


def innovation_summary(decisions):
    grouped = {}
    for row in decisions:
        if row["innovation_available"] != "1":
            continue
        value = float(row["nis"])
        if not math.isfinite(value) or value < 0:
            raise ValueError("invalid pre-update NIS")
        source = grouped.setdefault(row["source"], {"values": [], "rejected": 0})
        source["values"].append(value)
        source["rejected"] += row["accepted"] != "1"
    return {
        key: {
            "available_samples": len(source["values"]),
            "rejected_available_samples": source["rejected"],
            "median_nis": sorted(source["values"])[(len(source["values"]) - 1) // 2],
            "p95_nis": sorted(source["values"])[math.ceil(0.95 * len(source["values"])) - 1],
            "maximum_nis": max(source["values"]),
            "scope": "includes rejected available innovations; no calibration claim",
        }
        for key, source in grouped.items()
    }


def pose_difference(a, b):
    positions = [[float(row[key]) for key in ("px", "py", "pz")] for row in (a, b)]
    quaternions = [[float(row[key]) for key in ("qx", "qy", "qz", "qw")]
                   for row in (a, b)]
    values = [value for vector in positions + quaternions for value in vector]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite graph pose")
    norms = [math.hypot(*q) for q in quaternions]
    if any(norm == 0 for norm in norms):
        raise ValueError("zero graph quaternion")
    normalized = [[value / norm for value in q]
                  for q, norm in zip(quaternions, norms)]
    chord = min(math.sqrt(sum((x - sign * y) ** 2
                             for x, y in zip(*normalized))) for sign in (1, -1))
    return (math.dist(*positions), 4 * math.asin(min(1, chord / 2)))


def compare(window, control, prefix_output=None):
    events, reference = read(window / "events.csv"), read(control / "events.csv")
    if not events or len(events) != len(reference):
        raise ValueError("different causal event prefixes")
    differences = []
    prefixes = []
    for event, other in zip(events, reference):
        identity = ("receive_ns", "kind", "latest_measurement_ns", "initialized")
        if any(event[key] != other[key] for key in identity):
            raise ValueError("different causal event identities")
        if event["initialized"] == "1":
            difference = pose_difference(event, other)
            differences.append(difference)
            delta = [float(event[key]) - float(other[key]) for key in ("px", "py", "pz")]
            prefixes.append({
                **{key: event[key] for key in identity},
                "dx_m": delta[0], "dy_m": delta[1], "dz_m": delta[2],
                "translation_difference_m": difference[0],
                "rotation_difference_rad": difference[1],
                "window_graph_nodes": event["graph_nodes"],
                "control_graph_nodes": other["graph_nodes"],
            })
    if not differences:
        raise ValueError("no initialized comparison")
    decisions, expected = read(window / "decisions.csv"), read(control / "decisions.csv")
    identity = ("timestamp_ns", "receive_ns", "source", "sequence")
    if len(decisions) != len(expected) or any(
            any(a[key] != b[key] for key in identity)
            for a, b in zip(decisions, expected)):
        raise ValueError("different observation identities")
    mismatches = [
        {key: a[key] for key in identity} |
        {"window_reason": a["reason"], "control_reason": b["reason"],
         "window_accepted": a["accepted"], "control_accepted": b["accepted"]}
        for a, b in zip(decisions, expected)
        if (a["accepted"], a["reason"]) != (b["accepted"], b["reason"])
    ]
    counts = {}
    for row in decisions:
        source = counts.setdefault(row["source"], {"accepted": 0, "rejected": 0})
        source["accepted" if row["accepted"] == "1" else "rejected"] += 1
    if prefix_output is not None:
        with Path(prefix_output).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(prefixes[0]))
            writer.writeheader()
            writer.writerows(prefixes)
    return {
        "same_admission_outcomes": not mismatches,
        "admission_mismatches": mismatches,
        "window_decisions": counts,
        "compared_initialized_prefixes": len(differences),
        "maximum_translation_difference_m": max(pair[0] for pair in differences),
        "maximum_rotation_difference_rad": max(pair[1] for pair in differences),
        "final_translation_difference_m": differences[-1][0],
        "final_rotation_difference_rad": differences[-1][1],
        "maximum_horizontal_difference_m": max(
            math.hypot(row["dx_m"], row["dy_m"]) for row in prefixes),
        "maximum_vertical_difference_m": max(abs(row["dz_m"]) for row in prefixes),
        "final_position_delta_m": {
            key: prefixes[-1][key] for key in ("dx_m", "dy_m", "dz_m")
        },
        "window_resources": resource_summary(events),
        "control_resources": resource_summary(reference),
        "window_timing": timing_summary(events),
        "control_timing": timing_summary(reference),
        "window_innovations": innovation_summary(decisions),
        "control_innovations": innovation_summary(expected),
    }


def run(binary, input_path, output, lag):
    binary = Path(binary).resolve(strict=True)
    input_path, output = Path(input_path), Path(output)
    if not math.isfinite(lag) or lag <= 0:
        raise ValueError("lag must be positive and finite")
    inputs = INPUTS + (("temporal_model.csv",)
                       if (input_path / "temporal_model.csv").exists() else ())
    original = {name: digest(input_path / name) for name in inputs}
    original["binary"] = digest(binary)
    nodes = read(input_path / "nodes.csv")
    if not nodes:
        raise ValueError("empty causal node input")
    stamps = [int(row["timestamp_ns"]) for row in nodes]
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError("causal node timestamps must increase")
    span = (stamps[-1] - stamps[0]) * 1e-9
    control_lag = span + max(1.0, span * 0.01)
    output.mkdir()
    frozen = output / "input"
    frozen.mkdir()
    for name in inputs:
        shutil.copy2(input_path / name, frozen / name)
    executable = output / "keyframe_graph_replay"
    shutil.copy2(binary, executable)
    if {name: digest(frozen / name) for name in inputs} != {
            name: original[name] for name in inputs} or digest(executable) != original["binary"]:
        raise ValueError("input changed during freezing")
    protocol = {
        "input_sha256": original, "node_count": len(nodes),
        "window_lag_s": lag, "control_lag_s": control_lag,
        "native_model": False, "trusted": False,
        "driver_sha256": digest(Path(__file__)),
        "scope": "frozen frontend causal replay; shared-source dependence unmodeled",
        "temporal_model": "gauss-markov" if "temporal_model.csv" in inputs else "random-walk",
        "limits": [
            "Common full-pose map anchor in every arm; no GNSS-only cold start.",
            "Control uses the same received prefixes, not future-data batch.",
            "Gate outcomes may differ; inspect admissions before same-factor claims.",
            "Equal admissions do not imply equal robust weights or linearizations.",
            "No physical truth, calibration, health recovery or deadline acceptance.",
            "Executable and CSV hashes are frozen; runtime libraries are not pinned.",
            "No-eviction control retains finite input and may hit existing capacity.",
        ],
    }
    (output / "protocol.json").write_text(
        json.dumps(protocol, indent=2, allow_nan=False) + "\n")
    assessments = {}
    for arm in ARMS:
        for mode, duration in (("window", lag), ("control", control_lag)):
            destination = output / f"{arm}_{mode}"
            command = [str(executable.resolve()), str(frozen.resolve()),
                       str(destination.resolve()), arm, str(duration),
                       "--experimental-independent-keyframes", "--causal-receive-order"]
            with (output / f"{arm}_{mode}.log").open("w") as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
        control = read(output / f"{arm}_control" / "events.csv")
        node_events = [event for event in control if event["kind"] == "node"]
        if len(node_events) != len(nodes) or int(control[-1]["buffered_nodes"]) != len(nodes):
            raise ValueError("control did not retain the complete finite node prefix")
        graph_sizes = [int(event["graph_nodes"]) for event in control
                       if event["initialized"] == "1"]
        if any(a > b for a, b in zip(graph_sizes, graph_sizes[1:])):
            raise ValueError("control unexpectedly retired graph states")
        for mode in ("window", "control"):
            scope = read(output / f"{arm}_{mode}" / "model_scope.csv")
            if len(scope) != 1 or scope[0]["trusted"] != "0" or scope[0]["native_model"] != "0":
                raise ValueError("unexpected graph qualification scope")
        assessments[arm] = compare(
            output / f"{arm}_window", output / f"{arm}_control",
            output / f"{arm}_prefix_comparison.csv")
    if original != ({name: digest(input_path / name) for name in inputs} |
                    {"binary": digest(binary)}):
        raise ValueError("source input changed during replay")
    if any(digest(frozen / name) != original[name] for name in inputs) or (
            digest(executable) != original["binary"]):
        raise ValueError("frozen input changed during replay")
    if digest(Path(__file__)) != protocol["driver_sha256"]:
        raise ValueError("comparison driver changed during replay")
    assessments["artifact_sha256"] = {
        str(path.relative_to(output)): digest(path)
        for path in sorted(output.glob("**/*"))
        if path.is_file()
    }
    (output / "assessment.json").write_text(
        json.dumps(assessments, indent=2, allow_nan=False) + "\n")
    return assessments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--lag-s", required=True, type=float)
    args = parser.parse_args()
    run(args.binary, args.input, args.output, args.lag_s)


if __name__ == "__main__":
    main()
