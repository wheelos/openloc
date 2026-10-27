#!/usr/bin/env python3
"""Explicit diagnostic raw-native-LIO/map/SE(3)-CI replay; optional GNSS XY.

Run in the managed container after preparing raw inputs with prepare_native_lio.
The map seed is an explicitly supplied first-scan map-from-IMU hypothesis, not
an ODOM trajectory or a local initialization. No global result feeds local LIO.
"""

import argparse
from collections import Counter
import csv
import html
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from native_lio_report import (
    initialization_svg, rows, runtime_identity, summarize, summarize_capability,
    imu_trace_svg, summarize_imu_trace,
    summarize_global_scheduler, summarize_host_timing,
    summarize_local_lidar_tasks,
    summarize_lidar_decode,
    summarize_timer_output,
    summarize_initialization, summarize_local_corrections,
)
from prepare_native_lio import digest


def write_csv(path, header, values):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header.split(","))
        writer.writerow(values)


def checked_artifact(root, relative, expected):
    path = (root / relative).resolve(strict=True)
    if not path.is_relative_to(root.resolve(strict=True)):
        raise ValueError("map artifact escapes map root")
    if digest(path) != expected:
        raise ValueError(f"map artifact hash mismatch: {relative}")
    return path


def prepare_bundle(map_root, metadata_sha256, seed, seed_description, output,
                   window_radius=15.0, window_cell=2.0,
                   seed_reference_ns=None, seed_maximum_offset_ns=None,
                   recovery_policy=None, recovery_policy_sha256=None):
    causal = recovery_policy is not None
    if causal and (seed is not None or seed_description or
                   seed_reference_ns is not None or seed_maximum_offset_ns is not None):
        raise ValueError("causal search and offline seed are mutually exclusive")
    if causal != bool(recovery_policy_sha256):
        raise ValueError("causal search requires an explicit policy hash")
    if causal and digest(recovery_policy) != recovery_policy_sha256:
        raise ValueError("causal search policy hash mismatch")
    if (seed_reference_ns is None) != (seed_maximum_offset_ns is None):
        raise ValueError("seed reference and maximum offset required together")
    if seed_reference_ns is not None and (
            seed_reference_ns <= 0 or seed_maximum_offset_ns < 0):
        raise ValueError("seed reference must be positive and maximum offset nonnegative")
    if not causal and (seed is None or len(seed) != 7 or
                       not all(math.isfinite(x) for x in seed)):
        raise ValueError("seed requires seven finite pose values")
    if not causal and abs(sum(x * x for x in seed[3:]) - 1) > 1e-6:
        raise ValueError("seed quaternion must be normalized")
    if not 5 <= window_radius <= 30 or not 0.5 <= window_cell <= 2:
        raise ValueError("window radius/cell outside diagnostic bounds")
    metadata_path = map_root / "metadata.json"
    if digest(metadata_path) != metadata_sha256:
        raise ValueError("supplied map metadata pin mismatch")
    metadata = json.loads(metadata_path.read_text())
    if metadata.get("schema") != "wheelos.map.v1":
        raise ValueError("unsupported map schema")
    cloud = metadata["global_cloud"]
    pcd = checked_artifact(map_root, cloud["path"], cloud["sha256"])
    frames = checked_artifact(map_root, "frames.csv", metadata["frames_sha256"])
    output.mkdir()
    shutil.copyfile(pcd, output / "global.pcd")
    if digest(output / "global.pcd") != cloud["sha256"]:
        raise ValueError("copied prior map hash mismatch")
    write_csv(
        output / "identity.csv",
        "map_id,map_version,map_content_id,calibration_id,map_frame,"
        "odom_frame,base_frame",
        [metadata.get("map_id") or metadata["version"], metadata["version"],
         metadata["content_id"], "mid360-manual-T_I_L",
         "supplied-prior-map", "native-odom", "imu"],
    )
    if causal:
        shutil.copyfile(recovery_policy, output / "recovery.csv")
        if digest(output / "recovery.csv") != recovery_policy_sha256:
            raise ValueError("copied causal search policy hash mismatch")
    else:
        write_csv(output / "seed.csv", "x,y,z,qx,qy,qz,qw", seed)
    write_csv(output / "window.csv", "radius_m,cell_size_m",
              [window_radius, window_cell])
    if seed_reference_ns is not None:
        write_csv(output / "seed_time.csv", "reference_ns,maximum_offset_ns",
                  [seed_reference_ns, seed_maximum_offset_ns])
    manifest = {
        "schema": "openloc.native-map-diagnostic-input.v1",
        "map_root": str(map_root.resolve()),
        "metadata_sha256": metadata_sha256,
        "frames_sha256": digest(frames),
        "global_pcd_sha256": cloud["sha256"],
        "integrity_scope": "pinned metadata plus selected global PCD and frames",
        "map_qualification": metadata.get("qualification"),
        "map_coordinate_reference": metadata.get("coordinate_reference"),
        "seed_map_from_imu": seed,
        "causal_search": causal,
        "recovery_policy_sha256": recovery_policy_sha256,
        "recovery_policy_provenance": seed_description if not causal else
            "explicit conditional search/coverage/tilt prior; not inferred from PCD",
        "seed_description": seed_description,
        "seed_reference_ns": seed_reference_ns,
        "seed_maximum_offset_ns": seed_maximum_offset_ns,
        "seed_time_enforced": seed_reference_ns is not None,
        "seed_applied_only_to": None if causal else "first eligible scan-to-map request",
        "seed_epoch_semantics": "pose hypothesis at first post-native-startup "
                                "successfully deskewed scan reference, not bag start",
        "deskew_stream": "pre-LIO continuous native output frontier/history; "
                         "no corrected internal-history mixing",
        "seed_is_relocalization": False,
        "global_owner": "DiagnosticGlobalAlignment",
        "target_graph_available": False,
        "gnss_consumed": False,
        "target_window": {
            "implementation": "existing ScanToMapLocalizer local window",
            "radius_m": window_radius, "cell_size_m": window_cell,
            "actual_radius_policy": "cell-center radius plus half cell diagonal",
            "source_overlap_minimum": 0.4, "target_overlap_minimum": 0.4,
            "geometry_gates": "unchanged matcher defaults",
            "full_map_points": cloud["points"] if "points" in cloud else None,
        },
        "files": {p.name: digest(p) for p in sorted(output.iterdir())},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def map_summary(matches, outputs, decisions, allow_unanchored=False):
    if not matches or not outputs:
        raise ValueError("map/global execution evidence is required")
    updates = [d for d in decisions if d["alignment_updated"] == "1"]
    initialized = [row for row in outputs
                   if row["alignment_initialized"] == "1"]
    accepted = sum(row["valid"] == "1" for row in matches)
    established = bool(accepted and updates and initialized)
    if not established and not allow_unanchored:
        raise ValueError("accepted map and committed global output required")
    if any(row["target_graph_available"] != "0" or
           row["global_trusted"] != "0" or row["diagnostic_only"] != "1"
           for row in outputs):
        raise ValueError("diagnostic qualification invariant violated")
    return {
        "map_attempts": len(matches),
        "accepted_map_matches": accepted,
        "map_decisions": dict(Counter(row.get("reason", "unknown")
                                     for row in matches)),
        "global_decisions": dict(Counter(row.get("message", "unknown")
                                        for row in decisions)),
        "global_correction_updates": len(updates),
        "map_alignment_established": established,
        "global_updates_by_source": dict(Counter(
            row.get("source", "unspecified") for row in updates)),
        "initialized_global_output_rows": len(initialized),
        "first_global_timestamp_ns": initialized[0]["timestamp_ns"] if initialized else None,
        "last_global_timestamp_ns": initialized[-1]["timestamp_ns"] if initialized else None,
        "target_graph_available": False,
        "target_graph_decision": "unavailable: native qualified 15D cross-time "
                                 "conditional error chain is not established",
        "gnss_consumed": False,
    }


def gnss_summary(replay, contract):
    events = rows(replay / "gnss.csv")
    decisions = rows(replay / "global_decisions.csv")
    if len(events) != contract["events"]:
        raise ValueError("GNSS event journal must retain every arrival")
    if any(r["receiver_accuracy_available"] != "0" or
           r["heading_available"] != "0" for r in events):
        raise ValueError("GNSS journal invented receiver accuracy or heading")
    owner = [d for d in decisions
             if d["source"] == "raw-gga-horizontal-diagnostic"]
    accepted = [d for d in owner if d["status"] == "0"]
    final = {d["sequence"]: d for d in owner}
    final_accepted_events = []
    event_diagnostics = []
    for event in events:
        diagnostic = {
            "sequence": event["sequence"],
            "measurement_ns": event["timestamp_ns"],
            "receive_ns": event["receive_ns"],
            "gate_outcome": event["gate_outcome"],
            "gate_reason": event["gate_reason"],
            "adapter_reason": event["adapter_reason"],
            "outcome": ("adapter_rejected" if event["adapter_reason"] != "ok"
                        else "gate_not_accepted"),
            "reason": (event["adapter_reason"] if event["adapter_reason"] != "ok"
                       else event["gate_reason"]),
            "decision_id": None, "correction_id": None,
        }
        event_diagnostics.append(diagnostic)
        if event["gate_outcome"] != "accepted" or event["adapter_reason"] != "ok":
            continue
        decision = final.get(event["sequence"])
        if decision is None:
            if (event["owner_status"] == "not_submitted" and
                    event["owner_reason"] == "native_odometry_unavailable"):
                final_accepted_events.append({
                    "sequence": event["sequence"],
                    "measurement_ns": event["timestamp_ns"],
                    "receive_ns": event["receive_ns"],
                    "decision_id": None, "status": "not_submitted",
                    "reason": event["owner_reason"], "correction_id": None,
                    "outcome": "native_odometry_unavailable",
                })
                continue
            raise ValueError("gate-accepted GNSS event lacks final owner decision")
        if (decision["timestamp_ns"], decision["receive_ns"]) != (
                event["timestamp_ns"], event["receive_ns"]):
            raise ValueError("GNSS owner journal changed measurement or arrival time")
        final_accepted_events.append({
            "sequence": event["sequence"],
            "measurement_ns": event["timestamp_ns"],
            "receive_ns": event["receive_ns"],
            "decision_id": decision["decision_id"],
            "status": decision["status"], "reason": decision["message"],
            "correction_id": decision["correction_id"],
            "outcome": ("updated" if decision["alignment_updated"] == "1"
                        else "unanchored" if decision["status"] == "0"
                        else "rejected"),
        })
    accepted_by_sequence = {e["sequence"]: e for e in final_accepted_events}
    for diagnostic in event_diagnostics:
        accepted_event = accepted_by_sequence.get(diagnostic["sequence"])
        if accepted_event is not None:
            diagnostic.update({
                key: accepted_event[key]
                for key in ("decision_id", "correction_id", "outcome")
            })
            diagnostic["reason"] = accepted_event["reason"]
            if diagnostic["outcome"] == "unanchored":
                diagnostic["reason"] = "position_retained_without_global_anchor"
    return {
        "gnss_consumed": True, "arrival_events": len(events),
        "gate_outcomes": dict(Counter(r["gate_outcome"] for r in events)),
        "gate_reasons": dict(Counter(r["gate_reason"] for r in events)),
        "adapter_reasons": dict(Counter(r["adapter_reason"] for r in events)),
        "owner_reasons": dict(Counter(r["message"] for r in owner)),
        "native_odometry_unavailable_events": sum(
            r["owner_reason"] == "native_odometry_unavailable" for r in events
        ),
        "gate_accepted_final_owner_decisions": final_accepted_events,
        "event_diagnostics": event_diagnostics,
        "final_event_outcomes": dict(Counter(
            event["outcome"] for event in event_diagnostics)),
        "accepted_or_retained_owner_decisions": len(accepted),
        "unanchored_owner_decisions": sum(
            d["alignment_updated"] == "0" for d in accepted),
        "committed_correction_updates": sum(
            d["alignment_updated"] == "1" for d in owner),
        "heading_available": False, "receiver_accuracy_available": False,
        "uncertainty_qualified": False,
    }


def fusion_diagnostics(states, outputs, decisions, matches, gnss, map_contract):
    """Explain source admission versus actual correction without granting trust."""
    valid = [r for r in states if r["valid"] == "1"]
    initialized = [r for r in outputs if r["alignment_initialized"] == "1"]
    sources = {}
    for source in sorted({r["source"] for r in decisions}):
        journal = [r for r in decisions if r["source"] == source]
        final = {}
        for row in journal:
            key = (row["sequence"], row["timestamp_ns"], row["receive_ns"])
            final[key] = row
        outcomes = Counter()
        corrections = []
        for row in final.values():
            updated = row["alignment_updated"] == "1"
            if updated and row["status"] != "0":
                raise ValueError("rejected global observation claims a correction")
            outcome = ("updated" if updated else "accepted_without_correction"
                       if row["status"] == "0" else "not_accepted")
            outcomes[outcome] += 1
            if updated:
                corrections.append({
                    "sequence": row["sequence"],
                    "measurement_ns": row["timestamp_ns"],
                    "receive_ns": row["receive_ns"],
                    "decision_id": row["decision_id"],
                    "correction_id": row["correction_id"],
                })
        sources[source] = {
            "journal_entries": len(journal),
            "distinct_observations": len(final),
            "final_outcomes": dict(outcomes),
            "final_reasons": dict(Counter(r["message"] for r in final.values())),
            "corrections": corrections,
        }
    findings = [{
        "code": "DIAGNOSTIC_BACKEND_ONLY",
        "reason": "No qualified native joint error chain or target fixed-lag graph is established.",
    }]
    if not valid:
        findings.append({"code": "NATIVE_ODOMETRY_UNAVAILABLE",
                         "reason": "No valid native ODOM state in the replay."})
    if gnss:
        if gnss["native_odometry_unavailable_events"]:
            findings.append({
                "code": "GNSS_BEFORE_NATIVE_ODOMETRY",
                "count": gnss["native_odometry_unavailable_events"],
                "reason": "Gate-accepted arrivals could not be submitted before native startup.",
            })
        if gnss["unanchored_owner_decisions"]:
            findings.append({
                "code": "GNSS_POSITION_RETAINED_WITHOUT_ANCHOR",
                "count": gnss["unanchored_owner_decisions"],
                "reason": "Owner acceptance is not a correction; position alone did not establish a global pose.",
            })
        if not initialized:
            findings.append({
                "code": "GNSS_GLOBAL_INITIALIZATION_MISSING",
                "reason": "No global output was initialized; no single-antenna heading is inferred.",
            })
    if map_contract:
        findings.append({
            "code": ("MAP_CONDITIONAL_PRIORS" if map_contract.get("causal_search")
                     else "MAP_OFFLINE_SEED"),
            "reason": ("Regional/tilt priors are conditional, not whole-map autonomous recovery."
                       if map_contract.get("causal_search") else
                       "An explicit map-side pose hypothesis is not autonomous relocalization."),
        })
    def first_time(data, key):
        return int(data[0][key]) if data else None

    first_receive = first_time(states, "receive_ns")
    first_valid_receive = first_time(valid, "receive_ns")
    first_global_receive = first_time(initialized, "receive_ns")
    return {
        "schema": "openloc.native-fusion-diagnostics.v1",
        "clock_scope": "original recorded source times; not wall latency or publisher deadline",
        "native_startup": {
            "input_rows": len(states), "valid_rows": len(valid),
            "invalid_rows": len(states) - len(valid),
            "first_valid_measurement_ns": first_time(valid, "timestamp_ns"),
            "first_valid_receive_ns": first_valid_receive,
            "receive_delay_from_first_input_ns": (
                first_valid_receive - first_receive if valid and states else None),
            "invalid_integrity_reasons": dict(Counter(
                r["integrity_reason"] for r in states if r["valid"] != "1")),
            "valid_capability_reasons": dict(Counter(
                r["capability_reason"] for r in valid)),
        },
        "global_initialization": {
            "established": bool(initialized),
            "first_measurement_ns": first_time(initialized, "timestamp_ns"),
            "first_receive_ns": first_global_receive,
            "source_receive_delay_after_native_startup_ns": (
                first_global_receive - first_valid_receive
                if initialized and valid else None),
            "initialized_output_rows": len(initialized),
            "full_pose_fresh_rows": sum(r["full_pose_fresh"] == "1" for r in initialized),
        },
        "sources": sources,
        "map_matching": {
            "attempts": len(matches),
            "accepted": sum(r["valid"] == "1" for r in matches),
            "rejection_reasons": dict(Counter(
                r["reason"] for r in matches if r["valid"] != "1")),
            "accepted_observed_ranks": dict(Counter(
                r["observed_rank"] for r in matches if r["valid"] == "1")),
        },
        "findings": findings,
        "score_boundary": "Correction counts and same-source consistency are not independent accuracy; no universal fusion benefit is asserted.",
    }


def compare_odometry(actual, baseline):
    first, second = rows(actual), rows(baseline)

    def deterministic(data):
        return [{k: v for k, v in row.items() if k != "imu_ms"} for row in data]

    if deterministic(first) != deterministic(second):
        raise ValueError("absolute sources changed deterministic native ODOM")
    return {"identical_ignoring_only": ["imu_ms"], "rows": len(first),
            "baseline_sha256": digest(baseline), "actual_sha256": digest(actual)}


def causal_worker_summary(path):
    if not path.exists():
        return {"journal_present": False}
    records = rows(path)
    active = None
    queued = set()
    for row in records:
        key = (int(row["sequence"]), int(row["reference_ns"]))
        processing = int(row["processing_ns"])
        status = row["status"]
        if min(key) <= 0 or processing < key[1]:
            raise ValueError("invalid causal worker frontier")
        if status == "queued":
            if active is not None or key in queued or processing != key[1]:
                raise ValueError("causal worker exceeded one snapshot")
            active = key
            queued.add(key)
        elif status == "busy":
            if active is None or processing != key[1]:
                raise ValueError("causal worker busy without active snapshot")
        else:
            if active != key or status not in (
                    "proposal_consumed", "no_proposal", "stale_result_rejected",
                    "end_of_input_unconsumed"):
                raise ValueError("causal worker lost snapshot identity")
            if status == "proposal_consumed" and processing - key[1] > 500_000_000:
                raise ValueError("causal worker consumed an expired proposal")
            active = None
    if active is not None:
        raise ValueError("causal worker did not join its snapshot")
    return {
        "journal_present": True, "queries": len(queued),
        "outcomes": dict(Counter(r["status"] for r in records)),
        "maximum_outstanding_snapshots": 1, "maximum_proposal_age_ns": 500_000_000,
        "source_epoch_and_pose_retained": True, "uncertainty_qualified": False,
    }

def horizontal_summary(records):
    initialized = []
    for row in records:
        if (row["diagnostic_only"] != "1" or row["global_trusted"] != "0" or
                row["target_graph_available"] != "0"):
            raise ValueError("horizontal output gained qualification")
        values = ("correction_id", "x", "y", "alignment_x", "alignment_y",
                  "alignment_yaw", "last_observation_ns")
        if row["alignment_initialized"] == "1":
            if not all(row[key] and math.isfinite(float(row[key])) for key in values):
                raise ValueError("horizontal output is incomplete or nonfinite")
            if not (0 < int(row["last_observation_ns"]) <= int(row["timestamp_ns"]) <=
                    int(row["receive_ns"])) or int(row["correction_id"]) <= 0:
                raise ValueError("horizontal output has invalid source frontier")
            initialized.append(row)
        elif row["alignment_initialized"] != "0" or any(row[key] for key in values):
            raise ValueError("uninitialized horizontal output contains an estimate")
    return {
        "enabled": bool(records),
        "established": bool(initialized),
        "initialized_output_rows": len(initialized),
        "first_measurement_ns": (int(initialized[0]["timestamp_ns"])
                                 if initialized else None),
        "first_receive_ns": (int(initialized[0]["receive_ns"])
                             if initialized else None),
        "fresh_output_rows": sum(r["horizontal_fresh"] == "1" for r in initialized),
        "maximum_correction_id": max(
            (int(r["correction_id"]) for r in initialized), default=0),
        "gravity_level_odom": "explicit diagnostic assumption; not qualified",
        "height_or_full_pose_established": False,
        "uncertainty_qualified": False,
    }


def report(input_dir, replay, bundle, binary, output, gnss=None, arm="map",
           execution_runtime=None):
    manifest_path = input_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    for relative, expected in manifest["files"].items():
        checked_artifact(input_dir, relative, expected)
    contract = None
    if bundle:
        contract = json.loads((bundle / "manifest.json").read_text())
        for relative, expected in contract["files"].items():
            checked_artifact(bundle, relative, expected)
    metrics = (map_summary(rows(replay / "map.csv"), rows(replay / "global.csv"),
                           rows(replay / "global_decisions.csv"),
                           allow_unanchored=bool(contract.get("causal_search")))
               if bundle else {"gnss_consumed": False,
                               "target_graph_available": False})
    gnss_contract = None
    global_rows = (rows(replay / "global.csv")
                   if (replay / "global.csv").exists() else [])
    causal_path = replay / "causal_search.csv"
    if causal_path.exists():
        search = rows(causal_path)
        candidates = rows(replay / "causal_candidates.csv")
        if (not contract or not contract.get("causal_search") or
                rows(replay / "seed_application.csv") or
                any(r["diagnostic_only"] != "1" for r in search)):
            raise ValueError("causal search used a seed or gained qualification")
        metrics["causal_search"] = {
            "queries": len(search),
            "outcomes": dict(Counter(r["status"] for r in search)),
            "reasons": dict(Counter(r["reason"] for r in search)),
            "geometry_candidates": len(candidates),
            "candidate_rejections": dict(Counter(
                r["reason"] for r in candidates if r["valid"] == "0")),
            "proposals": sum(r["status"] == "diagnostic_proposal" for r in search),
            "diagnostic_only": True,
            "search_coverage_complete": False,
            "conditional_region_and_tilt_prior": True,
            "worker": causal_worker_summary(replay / "causal_worker.csv"),
        }
        tracking = rows(replay / "map_tracking.csv")
        if any(r["diagnostic_only"] != "1" for r in tracking):
            raise ValueError("map tracking gained qualification")
        metrics["causal_search"]["tracking"] = {
            "events": dict(Counter(r["event"] for r in tracking)),
            "last_state": tracking[-1]["state"] if tracking else "searching",
            "recovery_generations": max(
                (int(r["generation"]) for r in tracking), default=0),
            "diagnostic_only": True,
        }
    metrics["initialized_global_output_rows"] = sum(
        r["alignment_initialized"] == "1" for r in global_rows)
    horizontal_path = replay / "horizontal.csv"
    horizontal_rows = rows(horizontal_path) if horizontal_path.exists() else []
    if horizontal_path.exists():
        metrics["horizontal_alignment"] = horizontal_summary(horizontal_rows)
    metrics["full_pose_anchor_expected"] = bool(bundle)
    admission_path = replay / "global_admission.csv"
    admission = rows(admission_path) if admission_path.exists() else []
    for entry in admission:
        if not (int(entry["source_receive_ns"]) <=
                int(entry["enqueue_processing_ns"]) <=
                int(entry["submission_processing_ns"])):
            raise ValueError("global observation processed before source arrival")
        if (int(entry["measurement_ns"]) <= int(entry["source_receive_ns"]) and
                int(entry["submission_processing_ns"]) -
                int(entry["measurement_ns"]) < int(entry["holdback_ns"])):
            raise ValueError("global observation submitted before holdback watermark")
    metrics["global_admission"] = {
        "journal_present": admission_path.exists(),
        "submissions": len(admission),
        "holdback_ns": sorted({int(entry["holdback_ns"]) for entry in admission}),
        "maximum_queue_residence_ms": max(
            ((int(entry["submission_processing_ns"]) -
              int(entry["enqueue_processing_ns"])) / 1e6 for entry in admission),
            default=None),
    }
    if gnss:
        gnss_contract = json.loads((gnss / "manifest.json").read_text())
        for relative, expected in gnss_contract["files"].items():
            checked_artifact(gnss, relative, expected)
        metrics["gnss"] = gnss_summary(replay, gnss_contract)
        metrics["gnss_consumed"] = True
        metrics["three_source_fusion_established"] = (
            metrics.get("map_alignment_established", False) and
            metrics["gnss"]["committed_correction_updates"] > 0)
    states = rows(replay / "odometry.csv")
    initialization_path = replay / "motion_initialization.csv"
    initialization = rows(initialization_path) if initialization_path.exists() else []
    local_decisions = rows(replay / "lidar.csv")
    native_metrics = summarize(states, local_decisions)
    native_metrics["lidar_decode"] = summarize_lidar_decode(
        replay / "lidar_decode.csv", local_decisions)
    native_metrics["host_timing"] = summarize_host_timing(replay / "host_timing.csv")
    native_metrics["local_lidar_tasks"] = summarize_local_lidar_tasks(
        replay / "local_lidar_tasks.csv", native_metrics["runtime_scan_submissions"])
    native_metrics["capability_diagnostics"] = summarize_capability(
        replay / "capability.csv")
    corrections_path = replay / "local_corrections.csv"
    corrections = rows(corrections_path) if corrections_path.exists() else []
    native_metrics["local_corrections"] = summarize_local_corrections(corrections)
    native_metrics["local_corrections"]["journal_present"] = corrections_path.exists()
    native_metrics["global_scheduler"] = summarize_global_scheduler(
        replay / "global_scheduler.csv", native_metrics["imu_samples"],
        native_metrics["valid_states"], native_metrics["runtime_scan_submissions"])
    trace_path = replay / "imu_trace.csv"
    trace = rows(trace_path) if trace_path.exists() else []
    native_metrics["imu_trace"] = summarize_imu_trace(trace, corrections)
    native_metrics["imu_trace"]["journal_present"] = trace_path.exists()
    native_metrics["motion_initialization"] = summarize_initialization(initialization)
    native_metrics["motion_initialization"]["journal_present"] = initialization_path.exists()
    health = rows(replay / "health.csv")
    if len(health) != len(states):
        raise ValueError("health journal must cover every native IMU output")
    if any(row["health_trusted"] != "0" or
           row["assessment_covariance_valid"] != "0" for row in health):
        raise ValueError("health qualification invariant violated")
    metrics["health_rows"] = len(health)
    metrics["health_trusted_rows"] = 0
    metrics["health_availability_states"] = dict(Counter(
        row["availability_state"] for row in health))
    metrics["health_capabilities"] = dict(Counter(
        row["capabilities"] for row in health))
    metrics["health_primary_reasons"] = dict(Counter(
        row["primary_reason"] for row in health))
    evidence = {
        "schema": "openloc.native-map-diagnostic-report.v1",
        "diagnostic_only": True,
        "global_trusted": False,
        "global_owner": "DiagnosticGlobalAlignment" if arm != "odom" else None,
        "external_odometry_used": False,
        "raw_input_manifest_sha256": digest(manifest_path),
        "map_input_manifest_sha256": digest(bundle / "manifest.json") if bundle else None,
        "gnss_input_manifest_sha256": digest(gnss / "manifest.json") if gnss else None,
        "binary_sha256": digest(binary),
        "execution_runtime_identity": execution_runtime,
        "timer_output": summarize_timer_output(replay / "timer_output.csv"),
        "native_boundary_source_sha256": {
            name: digest(Path(__file__).resolve().parents[3] / name)
            for name in (
                "include/openloc/localization/local/native_types.h",
                "include/openloc/localization/local/motion_initialization_types.h",
                "include/openloc/localization/local/native_odometry.h",
                "src/localization/local/native_odometry.cc",
                "include/openloc/localization/local/native_motion_bootstrap.h",
                "src/localization/local/native_motion_bootstrap.cc",
                "src/localization/local/bootstrap_deskew.h",
            )
        },
        "integration_source_sha256": {
            name: digest(Path(__file__).with_name(name))
            for name in ("native_lio_replay.cc", "native_diagnostic_executor.h",
                         "native_lidar_scheduler.h",
                         "native_scan_reader.h",
                         "native_timer_output.h",
                         "native_causal_map_search.h",
                         "native_map_diagnostic.h",
                         "native_gnss_diagnostic.h", "prepare_native_gnss.py",
                         "prepare_native_lio.py", "run_native_map_diagnostic.py")
        },
        "global_contract_source_sha256": {
            name: digest(Path(__file__).resolve().parents[3] / name)
            for name in (
                "src/localization/online/gnss_quality_gate.h",
                "src/localization/online/gnss_quality_gate.cc",
                "src/localization/online/diagnostic_alignment.cc",
                "src/localization/online/alignment_recovery.cc",
                "src/localization/online/alignment_recovery.h",
                "src/localization/candidates/global/global_alignment.cc",
                "src/localization/candidates/global/global_types.h",
                "include/openloc/localization/global/diagnostic_alignment.h",
                "include/openloc/localization/global/observation_types.h",
                "src/localization/candidates/map/particle_relocalizer.cc",
                "src/localization/candidates/map/particle_relocalizer.h",
                "src/localization/candidates/map/likelihood_field.cc",
                "src/localization/candidates/map/likelihood_field.h",
                "src/localization/candidates/map/particle_hypothesis_tracker.cc",
                "src/localization/candidates/map/particle_hypothesis_tracker.h",
            )
        },
        "map_contract": contract,
        "gnss_contract": gnss_contract,
        "source_arm": arm,
        "seed_application": rows(replay / "seed_application.csv") if bundle else [],
        "native_metrics": native_metrics,
        "metrics": metrics,
        "outputs_sha256": {p.name: digest(p) for p in sorted(replay.glob("*.csv"))},
        "limitations": [
            ("Conditional causal particle/geometry diagnostic; explicit region, "
             "coverage and tilt prior; not qualified autonomous relocalization."
             if contract and contract.get("causal_search") else
             "Functional map-seeded diagnostic, not autonomous relocalization."),
            "One SE(3) diagnostic CI owner, not the target fixed-lag graph.",
            ("Optional GNSS XY consumed with uncalibrated explicit sigma; "
             "no receiver accuracy or heading inferred." if gnss else
             "No GNSS stream consumed; this is not a three-source replay."),
            "Native ODOM marginal and map covariance remain uncalibrated.",
            "Map matching shares native scans and native ODOM deskew.",
            "Supplied map may share the capture and GNSS construction; "
            "accepted matches are not independent accuracy evidence.",
            "Raw-point deskew interpolates bounded native output poses only; "
            "this does not create exact graph nodes or qualified motion.",
            "No truth, physical-up, road readiness or runtime deadlines proven.",
            "Optional bounded holdback delays only causal global observations; "
            "future-on-arrival inputs retain immediate owner validation. "
            "Replay tail advances processing time only. Beyond-budget arrivals "
            "can still be rejected; this is not historical graph refusion.",
            "Selected source/binary hashes are not a frozen dependency closure.",
        ],
    }
    decisions_path = replay / "global_decisions.csv"
    evidence["fusion_diagnostics"] = fusion_diagnostics(
        states, global_rows,
        rows(decisions_path) if decisions_path.exists() else [],
        rows(replay / "map.csv") if bundle else [],
        metrics.get("gnss"), contract)
    if horizontal_rows:
        partial = metrics["horizontal_alignment"]
        evidence["fusion_diagnostics"]["horizontal_initialization"] = partial
        if partial["established"]:
            findings = evidence["fusion_diagnostics"]["findings"]
            findings[:] = [f for f in findings
                           if f["code"] != "GNSS_GLOBAL_INITIALIZATION_MISSING"]
            findings.append({
                "code": "GNSS_HORIZONTAL_ALIGNMENT_ONLY",
                "reason": "Motion-derived XY/yaw acquired with later holdouts; "
                          "height, full pose and trusted output remain unavailable.",
            })
    output.mkdir()
    (output / "report.json").write_text(json.dumps(evidence, indent=2) + "\n")
    initialized = ([r for r in rows(replay / "global.csv")
                    if r["alignment_initialized"] == "1"]
                   if (replay / "global.csv").exists() else [])
    positions = [(float(r["x"]), float(r["y"])) for r in initialized]
    if not positions:
        positions = [(float(r["x"]), float(r["y"])) for r in horizontal_rows
                     if r["alignment_initialized"] == "1"]
    xmin, ymin = (min(p[0] for p in positions), min(p[1] for p in positions)) if positions else (0, 0)
    span = (max(max(p[0] for p in positions) - xmin,
                max(p[1] for p in positions) - ymin, 1e-6) if positions else 1)
    points = " ".join(f"{20 + (x-xmin)*480/span:.2f},"
                      f"{500 - (y-ymin)*480/span:.2f}" for x, y in positions)
    (output / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><h1>Native map diagnostic</h1>'
        '<p>SE(3) diagnostic CI; target graph unavailable; '
        + ('GNSS XY present; ' if gnss else 'GNSS absent; ')
        + 'no trusted global output or independent accuracy.</p>'
        '<svg viewBox="0 0 540 540" width="540" height="540">'
        f'<polyline points="{points}" fill="none" stroke="#1864ab"/></svg>'
        '<h2>Global map XY (equal scale)</h2>'
        + initialization_svg(initialization)
        + imu_trace_svg(trace)
        + '<h2>Fusion diagnostics: admission is not correction</h2><pre>'
        + html.escape(json.dumps(evidence["fusion_diagnostics"], indent=2))
        + '</pre><h2>Full evidence</h2><pre>'
        + html.escape(json.dumps(evidence, indent=2)) + "</pre>"
    )
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--map-root", type=Path)
    parser.add_argument("--map-metadata-sha256")
    parser.add_argument("--seed", type=float, nargs=7,
                        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    parser.add_argument("--seed-description")
    parser.add_argument("--seed-reference-ns", type=int)
    parser.add_argument("--seed-maximum-offset-ns", type=int)
    parser.add_argument("--arm", choices=["odom", "gnss", "map", "both"], default="map")
    parser.add_argument("--gnss-input", type=Path)
    parser.add_argument("--gnss-manifest-sha256")
    parser.add_argument("--diagnostic-level-odom-gnss-bootstrap", action="store_true",
                        help="opt-in XY/yaw acquisition; explicitly assumes level "
                             "ODOM/global XY, never initializes height or full pose")
    parser.add_argument("--compare-odometry", type=Path)
    parser.add_argument("--global-owner", choices=["diagnostic-ci"], required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion-bootstrap", action="store_true")
    parser.add_argument("--imu-trace", action="store_true",
                        help="explicit internal propagation/bias drift diagnostics")
    parser.add_argument("--arrival-paced", action="store_true",
                        help="pace actual host emission against recorded receive times")
    parser.add_argument("--async-global", action="store_true",
                        help="bounded single-worker map/GNSS execution; requires pacing")
    parser.add_argument("--async-local-lio-ms", type=int,
                        help="explicit [5,100] ms source-time local commit delay")
    parser.add_argument("--prefetch-lidar", action="store_true",
                        help="one bounded point-file decoding slot; arrival-gated consumption")
    parser.add_argument("--timer-output", action="store_true",
                        help="independent diagnostic 10 ms publication; preserved measurement identity")
    parser.add_argument("--recovery-policy", type=Path,
                        help="explicit region/coverage/tilt CSV; replaces offline seed")
    parser.add_argument("--recovery-policy-sha256")
    parser.add_argument("--motion-translation-deskew", action="store_true")
    parser.add_argument("--lidar-stationary-initialization", action="store_true")
    parser.add_argument("--motion-bootstrap-voxel-m", type=float)
    parser.add_argument("--global-holdback-ns", type=int, default=0)
    parser.add_argument("--map-window-radius-m", type=float, default=15.0)
    parser.add_argument("--map-window-cell-m", type=float, default=2.0)
    parser.add_argument("--container",
                        help="existing managed container; execute as current uid/gid")
    args = parser.parse_args()
    use_map = args.arm in ("map", "both")
    use_gnss = args.arm in ("gnss", "both")
    if args.diagnostic_level_odom_gnss_bootstrap and not use_gnss:
        parser.error("horizontal bootstrap requires a GNSS arm")
    if args.async_global and (not args.arrival_paced or args.arm == "odom"):
        parser.error("async global requires arrival pacing and a global source")
    if args.async_local_lio_ms is not None and (
            not args.arrival_paced or not 5 <= args.async_local_lio_ms <= 100):
        parser.error("async local LIO requires arrival pacing and [5,100] ms")
    if args.prefetch_lidar and not args.arrival_paced:
        parser.error("LiDAR prefetch requires arrival pacing")
    if args.timer_output and not args.arrival_paced:
        parser.error("timer output requires arrival pacing")
    if args.motion_translation_deskew and not args.motion_bootstrap:
        parser.error("translation deskew requires motion bootstrap")
    if args.lidar_stationary_initialization and not args.motion_bootstrap:
        parser.error("LiDAR rest initialization requires motion bootstrap")
    if args.motion_bootstrap_voxel_m is not None:
        if not args.motion_bootstrap or not 0.1 <= args.motion_bootstrap_voxel_m <= 0.5:
            parser.error("bootstrap voxel requires motion bootstrap and [0.1,0.5] m")
    if not 0 <= args.global_holdback_ns <= 500_000_000:
        parser.error("global holdback must be in [0,500000000] ns")
    if args.global_holdback_ns and args.arm == "odom":
        parser.error("global holdback requires map or GNSS input")
    if bool(args.recovery_policy) != bool(args.recovery_policy_sha256):
        parser.error("causal recovery policy and explicit hash required together")
    if args.recovery_policy and (
            not use_map or args.seed or args.seed_description or
            args.seed_reference_ns is not None or args.seed_maximum_offset_ns is not None):
        parser.error("causal map search requires a map arm and no offline seed")
    if use_map and (not all((args.map_root, args.map_metadata_sha256)) or
                    not (args.recovery_policy or (args.seed and args.seed_description))):
        parser.error("map arm requires pinned metadata and seed provenance or recovery policy")
    if use_gnss != bool(args.gnss_input and args.gnss_manifest_sha256):
        parser.error("GNSS arm requires input and explicit manifest pin")
    if not use_gnss and (args.gnss_input or args.gnss_manifest_sha256):
        parser.error("GNSS arguments require gnss or both arm")
    if use_gnss:
        if digest(args.gnss_input / "manifest.json") != args.gnss_manifest_sha256:
            raise ValueError("GNSS manifest pin mismatch")
        contract = json.loads((args.gnss_input / "manifest.json").read_text())
        if contract.get("uncertainty_qualified") is not False:
            raise ValueError("only explicitly unqualified diagnostic GNSS supported")
        for relative, expected in contract["files"].items():
            checked_artifact(args.gnss_input, relative, expected)
    args.output.mkdir(parents=True, exist_ok=False)
    bundle = args.output / "map_input" if use_map else None
    if bundle:
        prepare_bundle(args.map_root, args.map_metadata_sha256, args.seed,
                       args.seed_description, bundle, args.map_window_radius_m,
                       args.map_window_cell_m, args.seed_reference_ns,
                       args.seed_maximum_offset_ns, args.recovery_policy,
                       args.recovery_policy_sha256)
    # Validate raw hashes before executing, not merely when reporting.
    raw = json.loads((args.input / "manifest.json").read_text())
    if raw.get("external_odometry_used") is not False:
        raise ValueError("explicit raw/native input contract required")
    for relative, expected in raw["files"].items():
        checked_artifact(args.input, relative, expected)
    args.binary.resolve(strict=True)
    paths = [args.binary, args.input, args.output / "replay"]
    if bundle:
        paths.append(bundle)
    if use_gnss:
        paths.append(args.gnss_input)
    if args.container:
        workspace = Path(__file__).resolve().parents[5]
        paths = [Path("/apollo") / p.absolute().relative_to(workspace)
                 for p in paths]
    command = [str(paths[0]), str(paths[1]), str(paths[2])]
    if bundle:
        names = ["global.pcd", "identity.csv",
                 "recovery.csv" if args.recovery_policy else "seed.csv", "window.csv"]
        if args.seed_reference_ns is not None:
            names.append("seed_time.csv")
        command += ["--diagnostic-map", str(paths[3]), "--map-input-sha256",
                    "".join(digest(bundle / name) for name in names)]
    if use_gnss:
        command += ["--diagnostic-gnss", str(paths[-1]), "--gnss-input-sha256",
                    "".join(digest(args.gnss_input / name) for name in
                            ("events.csv", "config.csv", "identity.csv"))]
    if args.motion_bootstrap:
        command.append("--motion-bootstrap")
    if args.diagnostic_level_odom_gnss_bootstrap:
        command.append("--diagnostic-level-odom-gnss-bootstrap")
    if args.imu_trace:
        command.append("--imu-trace")
    if args.arrival_paced:
        command.append("--arrival-paced")
    if args.async_global:
        command.append("--async-global")
    if args.async_local_lio_ms is not None:
        command += ["--async-local-lio-ms", str(args.async_local_lio_ms)]
    if args.prefetch_lidar:
        command.append("--prefetch-lidar")
    if args.timer_output:
        command.append("--timer-output")
    if args.motion_translation_deskew:
        command.append("--motion-translation-deskew")
    if args.lidar_stationary_initialization:
        command.append("--lidar-stationary-initialization")
    if args.motion_bootstrap_voxel_m is not None:
        command += ["--motion-bootstrap-voxel-m", str(args.motion_bootstrap_voxel_m)]
    if args.global_holdback_ns:
        command += ["--global-holdback-ns", str(args.global_holdback_ns)]
    if args.container:
        command = ["docker", "exec", "--user", f"{os.getuid()}:{os.getgid()}",
                   "-w", "/apollo/pkgs/openloc", args.container,
                   "env", "-u", "LD_LIBRARY_PATH", *command]
    def inspect_runtime():
        if not args.container:
            return runtime_identity(args.binary)
        probe = [
            "docker", "exec", "--user", f"{os.getuid()}:{os.getgid()}",
            "-w", "/apollo/pkgs/openloc", args.container,
            "env", "-u", "LD_LIBRARY_PATH", "python3", "-c",
            "import json,sys;sys.path.insert(0,'test/integration/map');"
            "from native_lio_report import runtime_identity;"
            "from pathlib import Path;"
            "print(json.dumps(runtime_identity(Path(sys.argv[1]))))",
            str(paths[0]),
        ]
        return json.loads(subprocess.run(probe, check=True, capture_output=True,
                                         text=True).stdout)
    execution_runtime = inspect_runtime()
    (args.output / "runtime_identity.json").write_text(
        json.dumps(execution_runtime, indent=2) + "\n")
    (args.output / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    with (args.output / "replay.log").open("w") as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    if inspect_runtime() != execution_runtime:
        raise ValueError("native runtime dependencies changed during replay")
    evidence = report(args.input, args.output / "replay", bundle, args.binary,
                      args.output / "report",
                      args.gnss_input if use_gnss else None, args.arm,
                      execution_runtime)
    if args.compare_odometry:
        evidence["odometry_isolation"] = compare_odometry(
            args.output / "replay/odometry.csv", args.compare_odometry)
        (args.output / "report/report.json").write_text(
            json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence["metrics"], indent=2))
    if (args.arm == "both" and not args.recovery_policy and
            not evidence["metrics"]["three_source_fusion_established"]):
        raise ValueError("three-source fusion requires committed GNSS owner correction; "
                         "partial functional report retained")


if __name__ == "__main__":
    main()
