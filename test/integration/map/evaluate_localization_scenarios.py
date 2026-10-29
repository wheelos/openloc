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
    "field_sparse_map_gnss": "Field-derived sparse full-map availability and 1Hz admitted GNSS; "
                           "partial updates cannot conceal expired full-pose evidence.",
    "joint_loss_short": "1.5s absolute-source loss below the 3s window, with declared local drift.",
    "joint_loss_equal": "3s absolute-source loss equal to the window, with declared local drift.",
    "joint_loss_long": "15s absolute-source loss across five windows, with declared local drift.",
    "gnss_outlier_burst": "15s rejected GNSS outliers with map remaining; no latent contamination.",
    "gnss_outlier_burst_omitted": "Matched subset control omitting the same 15s GNSS burst.",
    "map_loss_30s": "Map absent for 30s; GNSS XY remains, full pose expires, map returns.",
    "map_loss_60s": "Map absent for 60s; GNSS XY remains across 20 windows, map returns.",
    "map_loss_120s": "Map absent for 120s; GNSS XY remains across 40 windows, map returns.",
}
FIELD_PROFILE = Path(__file__).with_name("testdata") / "field_sparse_map_gnss.csv"
WINDOW_LOSS_END = {"joint_loss_short": 9, "joint_loss_equal": 12, "joint_loss_long": 36}
BURST_SCENARIOS = ("gnss_outlier_burst", "gnss_outlier_burst_omitted")
MAP_LOSS_END = {"map_loss_30s": 66, "map_loss_60s": 126, "map_loss_120s": 246}


def node_count(name):
    if name in MAP_LOSS_END:
        return MAP_LOSS_END[name] + 12
    return 48 if name == "joint_loss_long" or name in BURST_SCENARIOS else 24


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


def audit_declared_observations(name, decisions):
    """Compare journals with the scenario contract, not their expected column."""
    all_nodes = set(range(node_count(name)))
    before_loss = set(range(6))
    after_loss = set(range(18, 24))
    map_nodes, projected_nodes = set(all_nodes), set()
    gnss_nodes, marker_nodes = set(all_nodes), set()
    rejected_gnss, rejected_marker = set(), set()
    if name == "indoor_map_no_gnss":
        gnss_nodes = set()
    elif name == "indoor_marker_no_map_gnss":
        map_nodes, gnss_nodes, marker_nodes = set(), set(), all_nodes
    elif name == "map_degenerate_gnss_available":
        map_nodes, projected_nodes = before_loss | after_loss, set(range(6, 18))
    elif name == "gnss_degraded_map_available":
        rejected_gnss = set(range(6, 18))
    elif name in ("indoor_corridor_marker", "joint_degradation_marker"):
        map_nodes, projected_nodes = before_loss, set(range(6, 24))
        marker_nodes = set(range(6, 24, 3))
        if name == "indoor_corridor_marker":
            gnss_nodes = set()
        else:
            rejected_gnss = set(range(6, 24))
    elif name == "gnss_only_after_anchor":
        map_nodes = before_loss
    elif name in MAP_LOSS_END:
        map_nodes = before_loss | set(range(MAP_LOSS_END[name], node_count(name)))
    elif name == "outdoor_indoor_outdoor":
        map_nodes, projected_nodes = set(range(12)) | after_loss, set(range(12, 18))
        marker_nodes = set(range(6, 12))
        gnss_nodes = all_nodes - marker_nodes
    elif name in ("joint_loss_reentry", "joint_loss_local_drift_reentry"):
        map_nodes = gnss_nodes = before_loss | after_loss
    elif name in WINDOW_LOSS_END:
        map_nodes = gnss_nodes = before_loss | set(range(WINDOW_LOSS_END[name], node_count(name)))
    elif name in BURST_SCENARIOS:
        if name == "gnss_outlier_burst":
            rejected_gnss = set(range(6, 36))
        else:
            gnss_nodes -= set(range(6, 36))
    elif name == "fault_rejection":
        marker_nodes, rejected_marker, rejected_gnss = {12, 13}, {12}, {10}
    elif name == "gnss_only_cold_start":
        map_nodes = set()
    elif name == "delayed_map":
        map_nodes, gnss_nodes = set(range(2, 24)), set()
    elif name == "field_sparse_map_gnss":
        profile = rows(FIELD_PROFILE)
        map_nodes = {int(r["node"]) for r in profile if r["map_available"] == "1"}
        gnss_nodes = {int(r["node"]) for r in profile if r["gnss_available"] == "1"}
    else:
        require(name == "outdoor_gnss_map", "missing declared scenario contract")

    schedule = {}
    for source, nodes in (("scan-to-map", map_nodes),
                          ("scan-to-map-projected", projected_nodes),
                          ("gnss-xy", gnss_nodes), ("known-marker", marker_nodes)):
        for node in nodes:
            if ((source == "gnss-xy" and node in rejected_gnss) or
                    (source == "known-marker" and node in rejected_marker)):
                expected = "rejected"
            elif source == "gnss-xy":
                expected = ("retained_unanchored"
                            if node < 2 or name == "gnss_only_cold_start" else "accepted")
            elif source == "scan-to-map-projected":
                expected = "accepted"
            else:
                expected = "accepted_or_verifying"
            observed = node - 2 if name == "delayed_map" else node
            schedule[(node, source)] = (expected, observed)
    require(Counter((int(r["node"]), r["source"]) for r in decisions) ==
            Counter(schedule.keys()), "declared scenario source schedule not exercised exactly")
    for event in decisions:
        node = int(event["node"])
        expected, observed = schedule[(node, event["source"])]
        require(event["expected"] == expected, "declared scenario outcome policy changed")
        require(int(event["receive_ns"]) == 10_000_000_000 + node * 500_000_000 and
                int(event["measurement_ns"]) == 10_000_000_000 + observed * 500_000_000,
                "declared scenario measurement/receive time changed")


def rotation_error(state):
    estimate = [float(state[key]) for key in ("qx", "qy", "qz", "qw")]
    truth = [float(state["truth_" + key]) for key in ("qx", "qy", "qz", "qw")]
    for quaternion in (estimate, truth):
        require(all(math.isfinite(value) for value in quaternion) and
                math.isclose(sum(value * value for value in quaternion), 1.0,
                             rel_tol=0.0, abs_tol=1e-9),
                "invalid scenario orientation quaternion")
    half_yaw = 0.2 + 0.015 * int(state["node"])
    declared = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
    require(min(sum((a - b) ** 2 for a, b in zip(truth, declared)),
                sum((a + b) ** 2 for a, b in zip(truth, declared))) < 1e-24,
            "declared scenario orientation truth changed")
    # conjugate(truth) * estimate, evaluated without acos near identity.
    tx, ty, tz, tw = truth
    ex, ey, ez, ew = estimate
    vector = (tw * ex - tx * ew - ty * ez + tz * ey,
              tw * ey - ty * ew - tz * ex + tx * ez,
              tw * ez - tz * ew - tx * ey + ty * ex)
    scalar = tw * ew + tx * ex + ty * ey + tz * ez
    return 2.0 * math.atan2(math.sqrt(sum(value * value for value in vector)),
                            abs(scalar))


def field_availability_profile(map_rows, gnss_rows, start_ns):
    """Bin front-end availability, not owner commits, poses or sensor noise."""
    result = []
    for node in range(24):
        begin = start_ns + node * 500_000_000
        end = begin + 500_000_000
        result.append({
            "node": str(node),
            "map_available": str(int(any(
                begin <= int(r["timestamp_ns"]) < end and r["valid"] == "1"
                for r in map_rows))),
            "gnss_available": str(int(any(
                begin <= int(r["timestamp_ns"]) < end and
                r["gate_outcome"] == "accepted" and r["adapter_reason"] == "ok"
                for r in gnss_rows))),
        })
    return result


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
    require([int(r["node"]) for r in states] == list(range(node_count(name))),
            "missing/duplicate state frontier")
    require([int(r["decision_id"]) for r in decisions] == list(range(1, len(decisions) + 1)),
            "missing/duplicate terminal decision")
    audit_declared_observations(name, decisions)
    require([int(r["timestamp_ns"]) for r in states] ==
            [10_000_000_000 + node * 500_000_000 for node in range(node_count(name))],
            "declared scenario source time changed")
    recomputed = 0
    for row in decisions:
        require(int(row["measurement_ns"]) <= int(row["receive_ns"]), "future observation consumed")
        require(row["status"] == "0" or row["updated"] == "0", "rejected observation committed")
        if row["expected"] == "rejected":
            require(row["status"] != "0", "fault unexpectedly accepted")
        elif row["expected"] == "retained_unanchored":
            require(row["status"] == "0" and row["updated"] == "0", "unanchored evidence became pose")
        elif row["expected"] == "accepted":
            require(row["status"] == "0" and row["updated"] == "1",
                    "usable anchored source did not commit")
        else:
            require(row["expected"] == "accepted_or_verifying" and
                    ((row["status"] == "0" and row["updated"] == "1") or
                     (row["status"] != "0" and row["updated"] == "0" and
                      "RELOCALIZATION_VERIFYING" in row["reason"])),
                    "declared scenario full-pose source neither committed nor verifying")
        require(row["updated"] in ("0", "1") and
                (row["updated"] != "1" or int(row["correction_id"]) > 0),
                "invalid correction commit")
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
        node = int(state["node"])
        px, py = 0.5 * node, 0.1 * math.sin(0.2 * node)
        declared_position = (100 + math.cos(0.4) * px - math.sin(0.4) * py,
                             20 + math.sin(0.4) * px + math.cos(0.4) * py, 1)
        require(all(math.isclose(float(state["truth_" + key]), expected,
                                 rel_tol=0.0, abs_tol=1e-12)
                    for key, expected in zip(("x", "y", "z"), declared_position)),
                "declared scenario position truth changed")
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
            angle = rotation_error(state)
            require(math.isfinite(float(state["rotation_error_rad"])) and
                    math.isclose(angle, float(state["rotation_error_rad"]),
                                 rel_tol=1e-8, abs_tol=1e-12),
                    "synthetic orientation error differs from logged truth")
            if name != "joint_loss_local_drift_reentry" and name not in WINDOW_LOSS_END:
                require(error < 1e-8, "exact synthetic geometry composition changed")
                require(angle < 1e-8, "exact synthetic orientation composition changed")
            covariance = _matrix(state["alignment_covariance"])
            require(len(covariance) == 6 and all(len(row) == 6 for row in covariance) and
                    all(math.isfinite(value) for row in covariance for value in row) and
                    all(covariance[i][i] > 0 for i in range(6)) and
                    all(math.isclose(covariance[i][j], covariance[j][i],
                                     abs_tol=1e-12, rel_tol=1e-9)
                        for i in range(6) for j in range(6)),
                    "invalid alignment covariance")
            require(math.isclose(sum(covariance[i][i] for i in range(6)),
                                 float(state["alignment_covariance_trace"]),
                                 abs_tol=1e-12, rel_tol=1e-9),
                    "alignment covariance trace differs from matrix")
        else:
            require(state["error_m"] == "", "missing pose counted as zero error")
            require(all(state[key] == "" for key in
                        ("qx", "qy", "qz", "qw", "rotation_error_rad")),
                    "missing pose contains invented orientation")
            require(state["alignment_covariance"] == "", "missing pose contains covariance")
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
    loss_metrics = None
    if name in MAP_LOSS_END:
        end = MAP_LOSS_END[name]
        require(all(states[node]["full_pose_fresh"] == ("1" if node <= 9 else "0")
                    for node in range(6, end)),
                "long map-loss full-pose lease mismatch")
        require(all(states[node]["pose_available"] == "1" for node in range(6, len(states))),
                "long map-loss graph lost established pose")
        require(states[end]["full_pose_fresh"] == "1",
                "long map-loss return failed to renew full pose")
        gnss = [r for r in decisions if r["source"] == "gnss-xy" and 6 <= int(r["node"]) < end]
        returned = [r for r in decisions if r["source"] == "scan-to-map" and int(r["node"]) >= end]
        require(len(gnss) == end - 6 and all(r["updated"] == "1" for r in gnss),
                "long map-loss usable GNSS ceased updating")
        require(len(returned) == 12 and all(r["updated"] == "1" for r in returned),
                "long map-loss return has no actual full-pose commit")
        loss_metrics = {
            "map_loss_duration_ns": (end - 6) * 500_000_000,
            "lag_ns": 3_000_000_000,
            "partial_commits_during_loss": len(gnss),
            "first_commit_after_return_ns": 0,
            "first_return_absolute_update_m": float(states[end]["absolute_update_translation_m"]),
            "first_return_absolute_update_rad": float(states[end]["absolute_update_rotation_rad"]),
            "scope": "exact public graph observation boundary; no raw registration/search-basin "
                     "or calibrated accuracy claim",
        }
    if name == "joint_loss_local_drift_reentry" or name in WINDOW_LOSS_END:
        loss_end = WINDOW_LOSS_END.get(name, 18)
        for node in range(6, loss_end):
            expected_drift = .02 * (node - 5)
            require(math.isclose(float(states[node]["error_m"]), expected_drift,
                                 abs_tol=1e-8),
                    "known loss-interval drift response missing or altered")
            require(float(states[node]["absolute_update_translation_m"]) < 1e-12 and
                    float(states[node]["absolute_update_rotation_rad"]) < 1e-12,
                    "absolute-source correction invented during joint loss")
        require(states[-1]["full_pose_fresh"] == "1" and
                float(states[-1]["error_m"]) < float(states[loss_end - 1]["error_m"]),
                "returned absolute sources did not constrain drift")
        # Freshness expires only after its lease; a short outage need not be stale.
        for node in range(6, loss_end):
            require(states[node]["full_pose_fresh"] == ("1" if node <= 9 else "0"),
                    "loss-interval full-pose lease mismatch")
        require(states[loss_end]["full_pose_fresh"] == "1",
                "returned full-pose observation did not renew freshness")
        returned = [r for r in decisions if int(r["node"]) >= loss_end and r["updated"] == "1"]
        require(bool(returned), "source return has no actual commit")
        first_commit = min(int(r["node"]) for r in returned)
        peak_error = float(states[loss_end - 1]["error_m"])
        reduced = next(int(r["node"]) for r in states[loss_end:]
                       if float(r["error_m"]) < peak_error)
        loss_metrics = {
            "loss_duration_ns": (loss_end - 6) * 500_000_000,
            "lag_ns": 3_000_000_000,
            "declared_local_offset_m": .02 * (loss_end - 6),
            "first_commit_after_return_ns": (first_commit - loss_end) * 500_000_000,
            "first_error_reduction_after_return_ns": (reduced - loss_end) * 500_000_000,
            "first_return_output_step_m": math.sqrt(sum(
                (float(states[loss_end][key]) - float(states[loss_end - 1][key])) ** 2
                for key in ("x", "y", "z"))),
            "return_error_m": float(states[loss_end]["error_m"]),
            "first_return_absolute_update_m": float(
                states[loss_end]["absolute_update_translation_m"]),
            "first_return_absolute_update_rad": float(
                states[loss_end]["absolute_update_rotation_rad"]),
            "final_error_m": float(states[-1]["error_m"]),
            "scope": "deterministic drift response; absolute update compares before/after sources "
                     "at the same ODOM node; output step also includes motion; "
                     "not a stochastic sensor envelope or product T/D",
        }
    if name == "gnss_outlier_burst":
        faults = [r for r in decisions if r["source"] == "gnss-xy" and 6 <= int(r["node"]) < 36]
        require(len(faults) == 30 and all(
            r["updated"] == "0" and float(r["nis"]) > float(r["gate"]) > 0 for r in faults),
            "GNSS burst did not exercise continuous innovation rejection")
    if name == "fault_rejection":
        fault = next(r for r in decisions if r["source"] == "gnss-xy" and r["node"] == "10")
        require(float(fault["nis"]) > float(fault["gate"]) > 0 and fault["updated"] == "0",
                "known GNSS jump did not trigger innovation rejection")
        require(sources["known-marker"]["rejected"] == 1 and
                sources["known-marker"]["committed"] == 1, "marker ambiguity/recovery not triggered")
    if name == "delayed_map":
        require(all(int(r["receive_ns"])-int(r["measurement_ns"]) == 1_000_000_000
                    for r in decisions), "delayed observations were retimed")
    if name == "field_sparse_map_gnss":
        profile = rows(FIELD_PROFILE)
        expected = {
            (r["node"], source)
            for r in profile
            for source, key in (("scan-to-map", "map_available"),
                                ("gnss-xy", "gnss_available"))
            if r[key] == "1"
        }
        require(Counter((r["node"], r["source"]) for r in decisions) ==
                Counter(expected), "field availability profile not exercised exactly")
        require(sources["scan-to-map"]["committed"] == 5 and
                sources["gnss-xy"]["committed"] == 11 and
                states[16]["full_pose_fresh"] == "0" and
                states[17]["full_pose_fresh"] == "1" and
                states[-1]["full_pose_fresh"] == "0",
                "sparse-map staleness/reentry or actual source updates missing")
    return {
        "status": "passed_functional_contract",
        "declared_source_schedule_verified": True,
        "product_scope": ("unobservable_full_pose_cold_start" if name == "gnss_only_cold_start"
                          else "synthetic_observation_boundary_only"),
        "behavior": ("expected_unobservable_cold_start" if name == "gnss_only_cold_start"
                     else "expected_degradation_and_recovery" if stale
                     else "available_source_contract"),
        "degradation_is_functional_failure": False,
        "description": SCENARIOS[name],
        "states": len(states), "pose_states": len(pose),
        "sources": sources, "nis_recomputed": recomputed,
        "loss_recovery_measurements": loss_metrics,
        "stale_full_pose_nodes": stale,
        "synthetic_exact_pose_error_m": distribution([float(r["error_m"]) for r in pose]),
        "synthetic_rotation_error_rad": distribution(
            [float(r["rotation_error_rad"]) for r in pose]),
        "synthetic_error_interpretation": (
            "known injected .02m/node ODOM drift across joint absolute loss; growth expected, "
            "no nominal accuracy requirement; final translation error must decrease after "
            "sources return; rotation error is measured, not required to remain zero"
            if name == "joint_loss_local_drift_reentry" else
            "declared .02m/node local drift for the selected loss duration; no stochastic envelope claim"
            if name in WINDOW_LOSS_END else
            "exact nominal translation and orientation composition only"),
        "update_ms": distribution([float(r["update_ms"]) for r in states]),
        "covariance_scope": "alignment trace is not held-pose covariance or calibrated coverage",
        "coverage": scenario_coverage(states, decisions),
    }


def audit_rejected_subset(replay):
    """Rejected observations must not change estimates, including after eviction."""
    fault, control = (rows(replay / name / "states.csv") for name in BURST_SCENARIOS)
    require(len(fault) == len(control) == 48, "incomplete rejected-factor subset pair")
    keys = ("node", "timestamp_ns", "pose_available", "full_pose_fresh", "odom_fresh",
            "trust", "diagnostic_only", "assessment", "correction_id")
    maximum = 0.0
    for bad, subset in zip(fault, control):
        require(all(bad[key] == subset[key] for key in keys),
                "rejected-factor subset state mismatch")
        if bad["pose_available"] == "0":
            continue
        left = [float(bad[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        right = [float(subset[key]) for key in ("x", "y", "z", "qx", "qy", "qz", "qw")]
        left += [value for row in _matrix(bad["alignment_covariance"]) for value in row]
        right += [value for row in _matrix(subset["alignment_covariance"]) for value in row]
        require(all(math.isfinite(a) and math.isfinite(b) and
                    math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-12)
                    for a, b in zip(left, right)),
                "rejected-factor subset estimate/covariance mismatch")
        maximum = max(maximum, max(abs(a - b) for a, b in zip(left, right)))
    return {"status": "passed_rejected_factor_noninterference", "paired_states": len(fault),
            "rejected_burst_ns": 15_000_000_000, "maximum_component_difference": maximum,
            "scope": "rejected factors only, not accepted contamination or stochastic dominance"}


def render(summary, output):
    body = [
        '<!doctype html><meta charset="utf-8"><title>OpenLoc indoor/outdoor scene matrix</title>'
        '<style>body{max-width:1100px;margin:2em auto;font:16px sans-serif;line-height:1.5}'
        'table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}.gap{color:#a32222}</style>'
        '<h1>One FixedLag owner: indoor/outdoor functional scenarios</h1>'
        '<p>Actual public graph, conditional 15D motion chain, projected/full map, GNSS antenna XY '
        'and known-marker pose composition. Per-scene node counts, 0.5 s source steps, 3 s lag.</p>'
        '<p class="gap">Synthetic observation-boundary evidence only: no raw LiDAR registration, '
        'detector, real receiver, field accuracy or calibrated covariance. Maps and extrinsics are '
        'explicitly deterministic here. No noise Monte Carlo/NEES claim. Missing poses are not zero error.</p>'
        '<p>Expected loss of accuracy/availability after factor loss is not a functional failure. '
        'The contract requires explicit freshness/trust, ODOM isolation and designed recovery, '
        'not nominal accuracy with absent information. Tested values are discrete coverage, not '
        'a guarantee over untested ranges or field conditions. Full-pose results include '
        'independently checked quaternion angular error, not translation alone.</p>'
        '<table><tr><th>Scene</th><th>Contract</th><th>Pose rows</th>'
        '<th>Stale full-pose rows</th><th>Source commits</th></tr>',
    ]
    for name, result in summary.items():
        counts = " / ".join(f'{html.escape(k)}={v["committed"]}' for k, v in result["sources"].items())
        body.append(f'<tr><td><a href="replay/{name}/decisions.csv">{name}</a><br>'
                    f'{html.escape(result["description"])}</td><td>{result["status"]}<br>'
                    f'{result["product_scope"]}<br>{result["behavior"]}</td><td>{result["pose_states"]}/{result["states"]}</td>'
                    f'<td>{len(result["stale_full_pose_nodes"])}</td><td>{counts}</td></tr>')
    body.append('</table><h2>Availability and freshness</h2><p>'
                'Each cell is one source-time node: green=fresh full pose; orange=retained stale full pose; '
                'gray=no full pose. Green remains diagnostic, not trusted.</p>')
    for name in summary:
        states = rows(output / "replay" / name / "states.csv")
        cells = "".join(f'<rect x="{10+25*i}" y="2" width="23" height="22" fill="'
                        f'{"#999" if r["pose_available"] == "0" else "#e69f00" if r["full_pose_fresh"] == "0" else "#009e73"}"/>'
                        for i, r in enumerate(states))
        body.append(f'<p>{name}</p><svg viewBox="0 0 {20+25*len(states)} 27" width="100%">{cells}</svg>')
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
    paths.append(FIELD_PROFILE)
    source = {str(p.relative_to(package)): digest(p) for p in paths}
    runtime = runtime_identity(args.binary)
    save(args.output / "protocol.json", {
        "schema": "openloc.indoor-outdoor-functional-scenes.v1",
        "source_sha256": source, "runtime": runtime,
        "scenarios": SCENARIOS,
        "nodes_per_scenario": {name: node_count(name) for name in SCENARIOS},
        "step_ns": 500_000_000,
        "lag_ns": 3_000_000_000,
        "owner": "FixedLagGlobalAlignment",
        "policy": {"allow_uncalibrated_inputs": True, "retain_graph_on_full_pose_staleness": True},
        "model": "synthetic independent 15D random walk: P_i=(.01+.001*i)I, cross=P_previous; "
                 "exact nominal geometry, no sampled noise, no empirical calibration; "
                 "map/marker survey and extrinsics deterministic; independent detector/receiver noise",
        "motion_evidence": "synthetic model declaration only, not a real native sufficient-model claim",
        "validation_tolerance": "1e-8m and 1e-8rad exact synthetic pose composition "
                                "except the declared local-drift case; not product accuracy requirements",
        "scope": "functional graph observation boundary, not raw sensors/detector/registration or field acceptance",
        "parameter_coverage": {
            "kind": "discrete values and explicitly triggered intervals; not exhaustive or range-certified",
            "gnss_jump_m": [50, -30],
            "window_loss_duration_ns": [1_500_000_000, 3_000_000_000, 15_000_000_000],
            "map_only_loss_duration_ns": [30_000_000_000, 60_000_000_000, 120_000_000_000],
            "rejected_gnss_burst": "30 consecutive half-second nodes, paired with omitted-source control",
            "projected_map_rows": 5,
            "projected_map_unobserved_axis": "map x translation",
            "gnss_dimension": 2,
            "gnss_covariance_m2": ".01 I",
            "map_covariance": ".0004 I in translation m and rotation rad chart",
            "nominal_motion": "p=(.5*i,.1*sin(.2*i),0), yaw=.03*i rad",
            "map_from_odom": "translation=(100,20,1)m, yaw=.4rad; "
                             "independent analytical pose truth, not a call to Odometry()",
            "fixed_marker": "survey/extrinsics deterministic; no detector simulation",
            "local_drift_during_joint_loss": "map x: .02m/node at nodes 6..17; "
                                            ".24m offset retained after return; ODOM never reset",
            "details": "per-scene coverage derives source/commit/reject intervals, delays and innovation dimensions from logs",
            "field_profile": {
                "path": str(FIELD_PROFILE.relative_to(package)),
                "sha256": digest(FIELD_PROFILE),
                "capture": "20261013170722",
                "relative_window_s": [70, 82],
                "source_origin_ns": 1791882552558665752,
                "map_csv_sha256": "a19f3124e8701a30d8340c4e88e0699c91bd1e9e47a13b96de526b682e347ea4",
                "gnss_csv_sha256": "eeb2977ed611cee54f3047c841c18eec3e5e592f5f0fde53b342014f49dc5715",
                "scope": "half-second bins of historical front-end availability only; "
                         "not original event counts/times, native motion, real poses or noise; "
                         "synthetic cold acquisition is not the field's warm initialization",
            },
        },
    })
    with (args.output / "execution.log").open("w") as log:
        execution = subprocess.run([str(args.binary), str(args.output / "replay"),
                                    str(FIELD_PROFILE)],
                                   stdout=log, stderr=subprocess.STDOUT)
    if execution.returncode:
        save(args.output / "summary.json", {"status": "failed_partial_run",
                                           "exit_code": execution.returncode})
        raise RuntimeError((args.output / "execution.log").read_text())
    actual = {p.name for p in (args.output / "replay").iterdir() if p.is_dir()}
    require(actual == set(SCENARIOS), "scenario matrix incomplete")
    summary = {name: audit_case(args.output / "replay" / name) for name in SCENARIOS}
    subset = audit_rejected_subset(args.output / "replay")
    require(runtime_identity(args.binary) == runtime and
            all(digest(package / name) == expected for name, expected in source.items()),
            "source/runtime changed during scene validation")
    save(args.output / "summary.json", summary)
    save(args.output / "subset_comparison.json", subset)
    render(summary, args.output)
    save(args.output / "artifact_hashes.json", {
        str(p.relative_to(args.output)): digest(p) for p in sorted(args.output.rglob("*"))
        if p.is_file() and p.name != "artifact_hashes.json"})
    print(json.dumps({"completed_scenarios": len(summary),
                      "nis_recomputed": sum(v["nis_recomputed"] for v in summary.values()),
                      "goal_qualified": False}, indent=2))


if __name__ == "__main__":
    main()
