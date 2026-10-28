#!/usr/bin/env python3
"""Frozen functional scenario acceptance for the real public FixedLag owner."""

import argparse
from collections import Counter
import csv
import html
import json
import math
import os
from pathlib import Path
import subprocess

from analyze_global_innovations import _matrix, _solve
from evaluate_native_fusion import distribution
from native_lio_report import runtime_identity
from prepare_native_lio import digest


SCENARIOS = {
    "indoor_map_no_gnss": "Indoor: full map poses, no GNSS or markers.",
    "indoor_marker_no_map_gnss": "Indoor: known-marker cold acquisition, no map matching/GNSS.",
    "outdoor_gnss_map": "Outdoor: map and antenna XY in one owner.",
    "map_degenerate_gnss_available": "Map loses longitudinal direction; GNSS stays available.",
    "gnss_degraded_map_available": "Receiver quality fails; map remains available.",
    "indoor_corridor_marker": "Indoor: projected corridor map plus intermittent known markers.",
    "joint_degradation_marker": "Map degenerates and GNSS quality fails; known markers remain.",
    "gnss_only_after_anchor": "Map disappears after acquisition; retain GNSS updates with stale full-pose trust.",
    "outdoor_indoor_outdoor": "GNSS loss indoors, markers/map, partial map outdoors, full map reentry.",
    "joint_loss_reentry": "All global observations disappear, then map/GNSS return.",
    "joint_loss_local_drift_reentry": "All absolute factors disappear; known local drift grows, "
                                     "then sources return without resetting ODOM.",
    "fault_rejection": "GNSS position jump and ambiguous marker must not commit.",
    "gnss_only_cold_start": "Position-only cold start cannot invent missing global attitude.",
    "delayed_map": "One-second delayed map observations bind original exact ODOM nodes.",
}


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def covered_intervals(nodes):
    intervals = []
    for node in sorted(set(nodes)):
        if intervals and node == intervals[-1][1]:
            intervals[-1][1] += 1
        else:
            intervals.append([node, node + 1])
    return intervals


def scenario_coverage(states, decisions):
    sources = {}
    for source in sorted({r["source"] for r in decisions}):
        events = [r for r in decisions if r["source"] == source]
        sources[source] = {
            "submitted_node_intervals_half_open": covered_intervals(
                int(r["node"]) for r in events),
            "committed_node_intervals_half_open": covered_intervals(
                int(r["node"]) for r in events if r["updated"] == "1"),
            "rejected_node_intervals_half_open": covered_intervals(
                int(r["node"]) for r in events if r["status"] != "0"),
            "innovation_dimensions_tested": sorted({
                len(_matrix(r["residual"])) for r in events
                if r["innovation_available"] == "1"}),
        }
    return {
        "parameters_are_tested_values_not_continuous_range": True,
        "nodes": len(states),
        "source_step_ns_tested": sorted({
            int(b["timestamp_ns"]) - int(a["timestamp_ns"])
            for a, b in zip(states, states[1:])}),
        "observation_delay_ns_tested": sorted({
            int(r["receive_ns"]) - int(r["measurement_ns"]) for r in decisions}),
        "sources": sources,
        "no_absolute_observation_node_intervals_half_open": covered_intervals(
            int(s["node"]) for s in states
            if not any(r["node"] == s["node"] for r in decisions)),
        "stale_full_pose_node_intervals_half_open": covered_intervals(
            int(s["node"]) for s in states if s["pose_available"] == "1" and
            s["full_pose_fresh"] == "0"),
        "uncovered": [
            "raw sensor/frontend behavior and real field scene labels",
            "local ODOM/IMU failure in this runner",
            "arbitrary dropout duration, delay, covariance, geometry or joint fault combinations",
            "sampled-noise accuracy and covariance calibration",
            "real marker detection, survey/extrinsic shared errors and live scheduling",
        ],
    }


def audit_case(folder):
    name = folder.name
    require(name in SCENARIOS, "unknown scenario")
    states, decisions = rows(folder / "states.csv"), rows(folder / "decisions.csv")
    require([int(r["node"]) for r in states] == list(range(24)), "missing/duplicate state frontier")
    require([int(r["decision_id"]) for r in decisions] == list(range(1, len(decisions) + 1)),
            "missing/duplicate terminal decision")
    stamps = [int(r["timestamp_ns"]) for r in states]
    require(all(b-a == 500_000_000 for a, b in zip(stamps, stamps[1:])),
            "source time changed")
    recomputed = 0
    for row in decisions:
        require(int(row["measurement_ns"]) <= int(row["receive_ns"]), "future observation consumed")
        require(row["status"] == "0" or row["updated"] == "0", "rejected observation committed")
        if row["expected"] == "rejected":
            require(row["status"] != "0", "fault unexpectedly accepted")
        elif row["expected"] == "retained_unanchored":
            require(row["status"] == "0" and row["updated"] == "0", "unanchored evidence became pose")
        elif row["expected"] == "accepted":
            require(row["status"] == "0", "usable source rejected")
        else:
            require(row["expected"] == "accepted_or_verifying" and
                    (row["status"] == "0" or "RELOCALIZATION_VERIFYING" in row["reason"]),
                    "unexpected source rejection")
        if row["innovation_available"] == "1":
            residual = [r[0] for r in _matrix(row["residual"])]
            calculated = sum(a*b for a, b in zip(
                residual, _solve(_matrix(row["covariance"]), residual)))
            require(math.isclose(calculated, float(row["nis"]), abs_tol=1e-8, rel_tol=1e-8),
                    "NIS cannot be recomputed")
            recomputed += 1
        else:
            require(bool(row["innovation_unavailable_reason"]), "innovation missing without reason")
    for state in states:
        require(state["input_unchanged"] == "1", "global owner changed local input")
        require(state["diagnostic_only"] == "1" and state["trust"] != "2", "trust ceiling broken")
        require(state["odom_fresh"] == "1", "synthetic source frontier lost ODOM freshness")
        if state["full_pose_fresh"] == "0":
            require(state["trust"] == "0", "partial observations renewed full-pose trust")
        events = [r for r in decisions if int(r["node"]) <= int(state["node"])]
        rejected = sum(r["status"] != "0" for r in events)
        updated = sum(r["updated"] == "1" for r in events)
        retained = len(events) - rejected - updated
        require([int(state[k]) for k in ("submitted", "rejected", "updates", "retained")] ==
                [len(events), rejected, updated, retained], "source ledger does not close")
        if state["pose_available"] == "1":
            error = math.sqrt(sum((float(state[k])-float(state["truth_"+k]))**2
                                  for k in ("x", "y", "z")))
            require(math.isclose(error, float(state["error_m"]), abs_tol=1e-12),
                    "synthetic pose error differs from logged truth")
            if name != "joint_loss_local_drift_reentry":
                require(error < 1e-8, "exact synthetic geometry composition changed")
        else:
            require(state["error_m"] == "", "missing pose counted as zero error")
    sources = {}
    for source in sorted({r["source"] for r in decisions}):
        events = [r for r in decisions if r["source"] == source]
        sources[source] = {
            "received": len(events),
            "committed": sum(r["updated"] == "1" for r in events),
            "retained": sum(r["status"] == "0" and r["updated"] == "0" for r in events),
            "rejected": sum(r["status"] != "0" for r in events),
            "reasons": dict(Counter(r["reason"] for r in events if r["status"] != "0")),
        }
    pose = [r for r in states if r["pose_available"] == "1"]
    stale = [int(r["node"]) for r in pose if r["full_pose_fresh"] == "0"]
    if name == "gnss_only_cold_start":
        require(not pose and sources["gnss-xy"]["committed"] == 0,
                "position-only cold start invented a full pose")
    else:
        require(states[-1]["pose_available"] == "1", "final pose unavailable")
    if name.startswith("indoor_"):
        require("gnss-xy" not in sources, "indoor test silently used GNSS")
    if name == "indoor_marker_no_map_gnss":
        require(set(sources) == {"known-marker"} and len(pose) == 22,
                "marker-only acquisition used another anchor or failed")
    if name == "map_degenerate_gnss_available":
        require(sources["scan-to-map-projected"]["committed"] == 12 and bool(stale),
                "projected graph did not survive expired full-pose evidence")
    if name == "gnss_only_after_anchor":
        require(bool(stale) and sources["gnss-xy"]["committed"] == 22,
                "established GNSS-only continuation not exercised")
    if name == "gnss_degraded_map_available":
        require(sources["gnss-xy"]["rejected"] == 12 and len(pose) == 22,
                "quality rejection/map continuity not triggered")
    if name in ("indoor_corridor_marker", "joint_degradation_marker"):
        require(sources["known-marker"]["committed"] == 6 and
                sources["scan-to-map-projected"]["committed"] == 18,
                "intermittent marker/partial-map complement not exercised")
    if name == "joint_degradation_marker":
        require(sources["gnss-xy"]["rejected"] == 18,
                "joint GNSS quality loss was not triggered")
    if name == "outdoor_indoor_outdoor":
        require(not any(6 <= int(r["node"]) < 12 and r["source"] == "gnss-xy" for r in decisions),
                "indoor interval used GNSS")
        require(sources["known-marker"]["committed"] == 6 and
                states[-1]["full_pose_fresh"] == "1", "source transition/reentry missing")
    if name in ("joint_loss_reentry", "joint_loss_local_drift_reentry"):
        require(not any(6 <= int(r["node"]) < 18 for r in decisions), "joint-loss interval leaked observations")
        require(any(6 <= node < 18 for node in stale) and
                states[-1]["full_pose_fresh"] == "1",
                "joint-loss degradation/reentry not triggered")
    if name == "joint_loss_local_drift_reentry":
        for node in range(6, 18):
            expected_drift = .02 * (node - 5)
            require(math.isclose(float(states[node]["error_m"]), expected_drift,
                                 abs_tol=1e-8),
                    "known loss-interval drift response missing or altered")
        require(states[-1]["full_pose_fresh"] == "1" and
                float(states[-1]["error_m"]) < float(states[17]["error_m"]),
                "returned absolute sources did not constrain drift")
    if name == "fault_rejection":
        fault = next(r for r in decisions if r["source"] == "gnss-xy" and r["node"] == "10")
        require(float(fault["nis"]) > float(fault["gate"]) > 0 and fault["updated"] == "0",
                "known GNSS jump did not trigger innovation rejection")
        require(sources["known-marker"]["rejected"] == 1 and
                sources["known-marker"]["committed"] == 1, "marker ambiguity/recovery not triggered")
    if name == "delayed_map":
        require(all(int(r["receive_ns"])-int(r["measurement_ns"]) == 1_000_000_000
                    for r in decisions), "delayed observations were retimed")
    return {
        "status": "passed_functional_contract",
        "product_scope": ("unobservable_full_pose_cold_start" if name == "gnss_only_cold_start"
                          else "synthetic_observation_boundary_only"),
        "behavior": ("expected_unobservable_cold_start" if name == "gnss_only_cold_start"
                     else "expected_degradation_and_recovery" if stale
                     else "available_source_contract"),
        "degradation_is_functional_failure": False,
        "description": SCENARIOS[name],
        "states": len(states), "pose_states": len(pose),
        "sources": sources, "nis_recomputed": recomputed,
        "stale_full_pose_nodes": stale,
        "synthetic_exact_pose_error_m": distribution([float(r["error_m"]) for r in pose]),
        "synthetic_error_interpretation": (
            "known injected .02m/node ODOM drift across joint absolute loss; growth expected, "
            "no nominal accuracy requirement; final error must decrease after sources return"
            if name == "joint_loss_local_drift_reentry" else
            "exact nominal geometry composition only"),
        "update_ms": distribution([float(r["update_ms"]) for r in states]),
        "covariance_scope": "alignment trace is not held-pose covariance or calibrated coverage",
        "coverage": scenario_coverage(states, decisions),
    }


def render(summary, output):
    body = [
        '<!doctype html><meta charset="utf-8"><title>OpenLoc indoor/outdoor scene matrix</title>'
        '<style>body{max-width:1100px;margin:2em auto;font:16px sans-serif;line-height:1.5}'
        'table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}.gap{color:#a32222}</style>'
        '<h1>One FixedLag owner: indoor/outdoor functional scenarios</h1>'
        '<p>Actual public graph, conditional 15D motion chain, projected/full map, GNSS antenna XY '
        'and known-marker pose composition. 24 nodes per scenario, 0.5 s source steps, 3 s lag.</p>'
        '<p class="gap">Synthetic observation-boundary evidence only: no raw LiDAR registration, '
        'detector, real receiver, field accuracy or calibrated covariance. Maps and extrinsics are '
        'explicitly deterministic here. No noise Monte Carlo/NEES claim. Missing poses are not zero error.</p>'
        '<p>Expected loss of accuracy/availability after factor loss is not a functional failure. '
        'The contract requires explicit freshness/trust, ODOM isolation and designed recovery, '
        'not nominal accuracy with absent information. Tested values are discrete coverage, not '
        'a guarantee over untested ranges or field conditions.</p>'
        '<table><tr><th>Scene</th><th>Contract</th><th>Pose rows</th>'
        '<th>Stale full-pose rows</th><th>Source commits</th></tr>',
    ]
    for name, result in summary.items():
        counts = " / ".join(f'{html.escape(k)}={v["committed"]}' for k, v in result["sources"].items())
        body.append(f'<tr><td><a href="replay/{name}/decisions.csv">{name}</a><br>'
                    f'{html.escape(result["description"])}</td><td>{result["status"]}<br>'
                    f'{result["product_scope"]}<br>{result["behavior"]}</td><td>{result["pose_states"]}/24</td>'
                    f'<td>{len(result["stale_full_pose_nodes"])}</td><td>{counts}</td></tr>')
    body.append('</table><h2>Availability and freshness</h2><p>'
                'Each cell is one source-time node: green=fresh full pose; orange=retained stale full pose; '
                'gray=no full pose. Green remains diagnostic, not trusted.</p>')
    for name in summary:
        states = rows(output / "replay" / name / "states.csv")
        cells = "".join(f'<rect x="{10+25*i}" y="2" width="23" height="22" fill="'
                        f'{"#999" if r["pose_available"] == "0" else "#e69f00" if r["full_pose_fresh"] == "0" else "#009e73"}"/>'
                        for i, r in enumerate(states))
        body.append(f'<p>{name}</p><svg viewBox="0 0 620 27" width="620">{cells}</svg>')
    body.append('<h2>Product gates still open</h2><p class="gap">'
                'Real native sufficient joint model; actual '
                'indoor/outdoor/marker recordings and independent reference; long-gap recovery and '
                'controlled global jumps; live/overload and actual consumer acceptance. '
                'GNSS XY alone cannot cold-start SE(3); refusing unobservable startup is correct, '
                'not itself a functional failure. The existing optional position+heading path '
                'is horizontal-only. Partial factors do not renew full-pose trust.</p>'
                '<p><a href="summary.json">Machine-readable outcomes</a> / '
                '<a href="protocol.json">Source/runtime and synthetic-model assumptions</a>.</p>')
    (output / "index.html").write_text("".join(body))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    require(os.getuid() != 0, "mapped non-root container user required")
    require(not args.output.exists(), "output must be new")
    args.output.mkdir(parents=True)
    package = Path(__file__).resolve().parents[3]
    paths = [p for directory in ("src", "include", "test/integration")
             for p in (package / directory).rglob("*")
             if p.is_file() and p.suffix in (".cc", ".h", ".py")]
    source = {str(p.relative_to(package)): digest(p) for p in paths}
    runtime = runtime_identity(args.binary)
    save(args.output / "protocol.json", {
        "schema": "openloc.indoor-outdoor-functional-scenes.v1",
        "source_sha256": source, "runtime": runtime,
        "scenarios": SCENARIOS, "nodes_per_scenario": 24, "step_ns": 500_000_000,
        "lag_ns": 3_000_000_000,
        "owner": "FixedLagGlobalAlignment",
        "policy": {"allow_uncalibrated_inputs": True, "retain_graph_on_full_pose_staleness": True},
        "model": "synthetic independent 15D random walk: P_i=(.01+.001*i)I, cross=P_previous; "
                 "exact nominal geometry, no sampled noise, no empirical calibration; "
                 "map/marker survey and extrinsics deterministic; independent detector/receiver noise",
        "motion_evidence": "synthetic model declaration only, not a real native sufficient-model claim",
        "validation_tolerance": "1e-8m exact synthetic composition; not a product accuracy requirement",
        "scope": "functional graph observation boundary, not raw sensors/detector/registration or field acceptance",
        "parameter_coverage": {
            "kind": "discrete values and explicitly triggered intervals; not exhaustive or range-certified",
            "gnss_jump_m": [50, -30],
            "projected_map_rows": 5,
            "projected_map_unobserved_axis": "map x translation",
            "gnss_dimension": 2,
            "gnss_covariance_m2": ".01 I",
            "map_covariance": ".0004 I in translation m and rotation rad chart",
            "nominal_motion": "p=(.5*i,.1*sin(.2*i),0), yaw=.03*i rad",
            "fixed_marker": "survey/extrinsics deterministic; no detector simulation",
            "local_drift_during_joint_loss": "map x: .02m/node at nodes 6..17; "
                                            ".24m offset retained after return; ODOM never reset",
            "details": "per-scene coverage derives source/commit/reject intervals, delays and innovation dimensions from logs",
        },
    })
    with (args.output / "execution.log").open("w") as log:
        execution = subprocess.run([str(args.binary), str(args.output / "replay")],
                                   stdout=log, stderr=subprocess.STDOUT)
    if execution.returncode:
        save(args.output / "summary.json", {"status": "failed_partial_run",
                                           "exit_code": execution.returncode})
        raise RuntimeError((args.output / "execution.log").read_text())
    actual = {p.name for p in (args.output / "replay").iterdir() if p.is_dir()}
    require(actual == set(SCENARIOS), "scenario matrix incomplete")
    summary = {name: audit_case(args.output / "replay" / name) for name in SCENARIOS}
    require(runtime_identity(args.binary) == runtime and
            all(digest(package / name) == expected for name, expected in source.items()),
            "source/runtime changed during scene validation")
    save(args.output / "summary.json", summary)
    render(summary, args.output)
    save(args.output / "artifact_hashes.json", {
        str(p.relative_to(args.output)): digest(p) for p in sorted(args.output.rglob("*"))
        if p.is_file() and p.name != "artifact_hashes.json"})
    print(json.dumps({"completed_scenarios": len(summary),
                      "nis_recomputed": sum(v["nis_recomputed"] for v in summary.values()),
                      "goal_qualified": False}, indent=2))


if __name__ == "__main__":
    main()
