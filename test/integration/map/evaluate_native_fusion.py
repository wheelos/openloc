#!/usr/bin/env python3
"""Audit pinned native diagnostic replays without ground-truth claims."""

import argparse
from collections import Counter
import csv
import html
import json
import math
from pathlib import Path

from analyze_global_innovations import analyze
from prepare_native_lio import digest


MAP_SOURCE = "native-raw-scan-to-prior-map"
GNSS_SOURCE = "raw-gga-horizontal-diagnostic"
PENDING = "retained in bounded diagnostic global holdback"
DETERMINISTIC_FILES = {
    "odometry.csv": {"imu_ms"},
    "lidar.csv": {"processing_ms"},
    "map.csv": {"processing_ms"},
    "global.csv": set(),
    "global_decisions.csv": set(),
    "global_admission.csv": set(),
    "gnss.csv": set(),
}


def records(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError(f"missing or duplicate CSV columns: {path}")
        result = list(reader)
    if any(None in row or None in row.values() for row in result):
        raise ValueError(f"incomplete CSV row: {path}")
    return result


def identity(row, source=None, admission=False):
    return (
        source or row["source"], int(row["sequence"]),
        int(row["measurement_ns"] if admission else row["timestamp_ns"]),
        int(row["source_receive_ns"] if admission else row["receive_ns"]),
    )


def distribution(values):
    if not values:
        return {"count": 0, "p50": None, "p95": None, "p99": None, "max": None}
    ordered = sorted(values)
    result = {"count": len(values), "max": ordered[-1]}
    for name, probability in (("p50", 0.5), ("p95", 0.95), ("p99", 0.99)):
        position = probability * (len(ordered) - 1)
        lower = int(position)
        result[name] = ordered[lower] + (ordered[min(lower + 1, len(ordered) - 1)] -
                                        ordered[lower]) * (position - lower)
    return result


def compare_replays(actual, reference, filenames):
    result = {}
    for filename in filenames:
        left, right = actual / filename, reference / filename
        if not left.exists() and not right.exists():
            continue
        if not left.exists() or not right.exists():
            raise ValueError(f"deterministic journal presence mismatch: {filename}")
        ignored = DETERMINISTIC_FILES[filename]
        with left.open(newline="") as stream:
            left_fields = set(next(csv.reader(stream))) - ignored
        with right.open(newline="") as stream:
            right_fields = set(next(csv.reader(stream))) - ignored
        if left_fields != right_fields:
            raise ValueError(f"deterministic schema mismatch: {filename}")
        first, second = records(left), records(right)
        if len(first) != len(second):
            raise ValueError(f"deterministic row count mismatch: {filename}")
        for index, (candidate, baseline) in enumerate(zip(first, second)):
            candidate = {k: v for k, v in candidate.items() if k not in ignored}
            baseline = {k: v for k, v in baseline.items() if k not in ignored}
            if candidate != baseline:
                raise ValueError(f"deterministic mismatch: {filename} row {index + 2}")
        result[filename] = {
            "status": "passed", "rows": len(first),
            "ignored_columns": sorted(ignored), "numeric_tolerance": 0,
            "actual_sha256": digest(left), "reference_sha256": digest(right),
        }
    if not result:
        raise ValueError("no deterministic comparison journals")
    return result


def scenario_correspondence(events):
    """Nominate shared mechanisms, never pass a field scene from source counts."""
    maps = [e for e in events if e["source"] == MAP_SOURCE and
            e["outcome"] == "committed"]
    gnss = [e for e in events if e["source"] == GNSS_SOURCE]
    fixes = [e for e in gnss if e["outcome"] == "committed"]
    quality = [e for e in gnss if e["outcome"] == "gate_not_accepted" and
               e["reason"] in ("carrier_solution_not_fixed", "hdop_exceeds_limit")]
    stamps = sorted(e["measurement_ns"] for e in maps)
    gaps = [(a, b) for a, b in zip(stamps, stamps[1:]) if b - a > 2_000_000_000]
    supported_gaps = [
        {"begin_ns": a, "end_ns": b, "gnss_commits": sum(
            a < e["measurement_ns"] < b for e in fixes)}
        for a, b in gaps
        if any(a < e["measurement_ns"] < b for e in fixes)
    ]
    mechanisms = {
        "S01": bool(maps and not gnss),
        "S03": bool(maps and fixes),
        "S05": bool(maps and quality),
        "S08": bool(supported_gaps),
        "S12": any(e["outcome"] == "retained_without_correction" for e in gnss),
        "S15": bool(supported_gaps),
    }
    return {
        "status": "mechanism_correspondence_only_not_field_acceptance",
        "cases": {
            f"S{number:02}": ("partial" if mechanisms.get(f"S{number:02}", False)
                             else "not_observed_in_this_arm")
            for number in range(1, 16)
        },
        "evidence": {
            "map_commits": len(maps), "gnss_commits": len(fixes),
            "gnss_quality_rejections": len(quality),
            "sparse_map_intervals_with_gnss_commits": supported_gaps,
            "sparse_map_gap_definition_ns": 2_000_000_000,
            "gap_scope": "between actual map commits only; excludes startup/tail; "
                         "not a measured freshness policy or synthetic timing match",
        },
        "limits": [
            "DiagnosticGlobalAlignment is not the synthetic FixedLag owner",
            "disabled or absent sources do not establish indoor/outdoor scene labels",
            "source counts do not prove observability, recovery, accuracy or matching parameters",
            "map matcher failures are not projected factors with known unobserved directions",
            "quality/stability front-end gates are separate from graph source qualification",
            "unobserved cases are uncovered, not functional failures",
        ],
    }


def audit_replay(replay, report):
    """Audit the diagnostic runner's actual event boundary, not graph factors."""
    if (report["schema"] != "openloc.native-map-diagnostic-report.v1" or
            report["global_owner"] not in (None, "DiagnosticGlobalAlignment") or
            report["diagnostic_only"] is not True or report["global_trusted"] is not False):
        raise ValueError("unsupported owner or diagnostic trust promotion")
    paths = report["outputs_sha256"]
    for relative, expected in paths.items():
        path = replay / relative
        if path.parent.resolve() != replay.resolve() or digest(path) != expected:
            raise ValueError(f"replay artifact hash mismatch: {relative}")
    required = {"odometry.csv", "lidar.csv", "health.csv"}
    if report["global_owner"] is not None:
        required.update(("global_decisions.csv", "global_admission.csv", "global.csv"))
    if report["gnss_contract"] is not None:
        required.add("gnss.csv")
    if report["map_contract"] is not None:
        required.add("map.csv")
    if not required.issubset(paths):
        raise ValueError("report lacks required journal pins")

    def load(name):
        return records(replay / name) if name in paths else []

    odometry, health = load("odometry.csv"), load("health.csv")
    if not odometry or len(health) != len(odometry):
        raise ValueError("health journal does not cover ODOM")
    if any((row["timestamp_ns"], row["receive_ns"]) !=
           (state["timestamp_ns"], state["receive_ns"])
           for row, state in zip(health, odometry)):
        raise ValueError("health journal changed ODOM time identities")
    if any(row["health_trusted"] != "0" or row["assessment_covariance_valid"] != "0"
           for row in health):
        raise ValueError("unqualified input gained health trust")
    if any(row["diagnostic_only"] != "1" or row["global_trusted"] != "0" or
           row["target_graph_available"] != "0" for row in load("global.csv")):
        raise ValueError("diagnostic output gained qualification")

    expected, ledger = {}, []
    def event(row, source, outcome, reason, submitted):
        key = identity(row, source)
        if min(key[1:]) <= 0 or key in expected:
            raise ValueError("duplicate or invalid source event identity")
        entry = {
            "source": source, "sequence": key[1],
            "measurement_ns": key[2], "receive_ns": key[3],
            "outcome": outcome, "reason": reason, "submitted": submitted,
            "decision_id": None, "correction_id": None,
            "trust": "diagnostic", "epoch": None,
            "epoch_unavailable_reason": "not_exported_by_legacy_diagnostic_journal",
            "robust_weight": None, "robust_weight_status": "not_applicable_ci_owner",
        }
        expected[key] = entry
        ledger.append(entry)

    gnss = load("gnss.csv")
    if report["gnss_contract"] is not None and len(gnss) != report["gnss_contract"]["events"]:
        raise ValueError("GNSS journal lost an arrival")
    for row in gnss:
        if (row["receiver_accuracy_available"] != "0" or row["heading_available"] != "0" or
                row["target_graph_available"] != "0"):
            raise ValueError("GNSS journal invented source qualification")
        if row["adapter_reason"] != "ok":
            outcome, reason, submitted = "adapter_rejected", row["adapter_reason"], False
        elif row["gate_outcome"] != "accepted":
            outcome, reason, submitted = "gate_not_accepted", row["gate_reason"], False
        elif row["owner_status"] == "not_submitted":
            if row["owner_reason"] != "native_odometry_unavailable":
                raise ValueError("unknown GNSS non-submission")
            outcome, reason, submitted = "odometry_unavailable", row["owner_reason"], False
        else:
            outcome, reason, submitted = "pending", row["owner_reason"], True
        event(row, GNSS_SOURCE, outcome, reason, submitted)
    for row in load("map.csv"):
        if row["valid"] not in ("0", "1"):
            raise ValueError("invalid map admission flag")
        submitted = row["valid"] == "1"
        event(row, MAP_SOURCE, "pending" if submitted else "matcher_not_accepted",
              row["reason"], submitted)

    decisions, final, previous_id, revision = load("global_decisions.csv"), {}, 0, 0
    for row in decisions:
        key = identity(row)
        decision_id = int(row["decision_id"])
        if decision_id != previous_id + 1:
            raise ValueError("global decision journal gap")
        previous_id = decision_id
        if key not in expected or not expected[key]["submitted"]:
            raise ValueError("owner decision has no submitted source event")
        if row["target_graph_available"] != "0":
            raise ValueError("CI decision claims target graph")
        if row["message"] == PENDING:
            if key in final:
                raise ValueError("pending decision after terminal decision")
            continue
        if key in final:
            raise ValueError("duplicate terminal decision")
        updated = row["alignment_updated"]
        if updated not in ("0", "1") or (updated == "1" and row["status"] != "0"):
            raise ValueError("rejected observation claims correction")
        if updated == "1":
            if int(row["correction_id"]) != revision + 1:
                raise ValueError("non-consecutive committed correction revision")
            revision += 1
        elif int(row["correction_id"]) != revision:
            raise ValueError("non-committed observation changed correction revision")
        final[key] = row
        expected[key].update(
            outcome=("committed" if updated == "1" else "retained_without_correction"
                     if row["status"] == "0" else "owner_not_accepted"),
            reason=row["message"], decision_id=decision_id,
            correction_id=int(row["correction_id"]),
        )

    admissions, seen, queue_ns = load("global_admission.csv"), set(), []
    submission_order = {}
    previous_submission = 0
    for row in admissions:
        key = identity(row, admission=True)
        if key in seen or key not in expected or not expected[key]["submitted"]:
            raise ValueError("duplicate or orphan admission")
        seen.add(key)
        enqueue, submit = int(row["enqueue_processing_ns"]), int(row["submission_processing_ns"])
        holdback = int(row["holdback_ns"])
        if not key[3] <= enqueue <= submit or holdback < 0:
            raise ValueError("submission predates source arrival")
        if submit < previous_submission:
            raise ValueError("global submission processing time regressed")
        previous_submission = submit
        if key[2] <= key[3] and submit - key[2] < holdback:
            raise ValueError("submission predates holdback watermark")
        queue_ns.append(submit - enqueue)
        submission_order.setdefault(key[0], []).append(key[2])
        terminal = final.get(key)
        if terminal is not None and (row["status"], row["message"]) != (
                terminal["status"], terminal["message"]):
            raise ValueError("admission and owner terminal outcome disagree")
    if seen != {key for key, entry in expected.items() if entry["submitted"]}:
        raise ValueError("submitted source event lacks admission")
    pending = sum(entry["outcome"] == "pending" for entry in ledger)
    sources = {}
    for source in sorted({entry["source"] for entry in ledger}):
        group = [entry for entry in ledger if entry["source"] == source]
        measurement = submission_order.get(source, [])
        sources[source] = {
            "received": len(group),
            "outcomes": dict(Counter(entry["outcome"] for entry in group)),
            "source_delay_ns": distribution([
                entry["receive_ns"] - entry["measurement_ns"] for entry in group]),
            "measurement_regressions_at_submission": sum(
                right < left for left, right in zip(measurement, measurement[1:])),
            "receive_regressions": sum(
                right["receive_ns"] < left["receive_ns"]
                for left, right in zip(group, group[1:])),
        }
    return {
        "schema": "openloc.native-fusion-audit.v1",
        "checks": {
            "artifact_integrity": {"status": "passed", "files": len(paths)},
            "event_accounting": {"status": "failed" if pending else "passed",
                                 "received": len(ledger), "terminal": len(ledger) - pending,
                                 "pending": pending},
            "diagnostic_trust_ceiling": {"status": "passed", "health_rows": len(health)},
            "source_receive_order": {
                "status": ("failed" if any(source["receive_regressions"]
                                          for source in sources.values())
                           else "passed" if ledger else "not_run"),
                "scope": "per-source recorded arrival order, not measurement-time monotonicity"},
            "default_consumer_rejection": {
                "status": "not_run", "reason": "requires actual consumer subscription"},
        },
        "sources": sources, "events": ledger,
        "scenario_correspondence": scenario_correspondence(ledger),
        "queue_residence_ns": distribution(queue_ns),
        "committed_corrections": revision,
        "scope": "Diagnostic source events; not a complete graph-factor ledger. "
                 "Recorded source clocks are not wall latency. No accuracy or calibration claim.",
    }


def evaluate(report_path, replay, baseline=None, repeat=None):
    report = json.loads(report_path.read_text())
    audit = audit_replay(replay, report)
    audit["report_sha256"] = digest(report_path)
    audit["analyzer_sha256"] = digest(Path(__file__).with_name("analyze_global_innovations.py"))
    audit["evaluator_sha256"] = digest(Path(__file__))
    audit["innovations"] = (analyze(replay / "global_decisions.csv", window_ns=5_000_000_000)
                            if report["global_owner"] is not None else None)
    audit["odometry_isolation"] = (
        compare_replays(replay, baseline, ("odometry.csv", "lidar.csv"))
        if baseline is not None else {"status": "not_run"})
    audit["determinism"] = (
        compare_replays(replay, repeat, DETERMINISTIC_FILES)
        if repeat is not None else {"status": "not_run"})
    audit["remaining_evidence"] = {
        "fixed_lag_real_data": "not_run",
        "same_factor_batch_comparison": "not_run",
        "semi_synthetic_nees": "not_run",
        "physical_weak_reference": "not_run",
        "live_overload": "not_run",
    }
    return audit


def _plot(points):
    finite = [(float(x), float(y)) for x, y in points
              if x is not None and y is not None and math.isfinite(x) and math.isfinite(y)]
    if not finite:
        return "<p>Unavailable: insufficient variation/samples.</p>"
    xs, ys = zip(*finite)
    xmin, ymin = min(xs), min(ys)
    dx, dy = max(max(xs) - xmin, 1e-9), max(max(ys) - ymin, 1e-9)
    dots = "".join(f'<circle cx="{20 + 300 * (x-xmin)/dx:.3f}" '
                   f'cy="{160 - 140 * (y-ymin)/dy:.3f}" r="2"/>'
                   for x, y in finite)
    return f'<svg viewBox="0 0 340 180" width="340" role="img">{dots}</svg>'


def render(result):
    rows = []
    for group in (result.get("innovations") or {}).get("groups", []):
        label = html.escape(f'{group["source"]} / {group["outcome"]} / '
                            f'dim={group["observed_dimension"]}, n={group["sample_count"]}')
        for component, values in enumerate(group["whitened_residual"]):
            qq = _plot([(q["theoretical_normal_quantile"], q["sample_whitened_quantile"])
                        for q in values["qq"]])
            acf = _plot([(a["lag"], a["autocorrelation"]) for a in values["autocorrelation"]])
            rows.append(f"<h3>{label}, component {component}</h3>"
                        f"<p>NIS mean={group['nis_mean']:.4g}; "
                        f"p99={group['nis_p99']:.4g}; max={group['nis_max']:.4g}.</p>"
                        "<p>QQ: normal quantile → whitened quantile; "
                        "ACF: selected-sample lag → correlation. Axes auto-scaled.</p>"
                        f"{qq}{acf}")
    gate_rows = []
    for decision in (result.get("innovations") or {}).get("gate_diagnostics", []):
        gate_rows.append("<tr>" + "".join(
            f"<td>{html.escape(str(decision[key]))}</td>"
            for key in ("source", "sequence", "status", "nis", "gate_threshold",
                        "residual", "innovation_variance_diagonal",
                        "covariance_contribution_diagonals")) + "</tr>")
    gates = ("<h2>Pre-update gate explanation</h2>"
             "<p>Small NIS can result from large uncertainty, not small physical error. "
             "Contributions describe the logged model, not calibrated independent errors. "
             "None means unavailable, never zero.</p><table border='1'><tr>"
             "<th>Source</th><th>Sequence</th><th>Status</th><th>NIS</th><th>Gate</th>"
             "<th>Residual</th><th>Innovation diagonal</th>"
             "<th>Prior / observation / local contributions</th></tr>" +
             "".join(gate_rows) + "</table>")
    detail = html.escape(json.dumps(result, indent=2, allow_nan=False))
    return ('<!doctype html><meta charset="utf-8"><title>OpenLoc diagnostic evaluation</title>'
            '<h1>OpenLoc: no-ground-truth diagnostic evaluation</h1>'
            '<p>Functional audit and descriptive consistency only. NIS is not calibration; '
            'smoothness is not accuracy. Unavailable evidence is not a pass.</p>' +
            "".join(rows) + gates + f"<details><summary>Full audit, events and limitations</summary>"
            f"<pre>{detail}</pre></details>")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, help="local-only replay for ODOM isolation")
    parser.add_argument("--repeat", type=Path, help="same-config replay for exact determinism")
    parser.add_argument("--output", type=Path, required=True, help="new report-only directory")
    args = parser.parse_args(argv)
    result = evaluate(args.report, args.replay, args.baseline, args.repeat)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "evaluation.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    (args.output / "index.html").write_text(render(result))
    (args.output / "artifact_hashes.json").write_text(json.dumps({
        name: digest(args.output / name) for name in ("evaluation.json", "index.html")
    }, indent=2) + "\n")
    return 1 if any(check["status"] == "failed"
                    for check in result["checks"].values()) else 0


if __name__ == "__main__":
    raise SystemExit(main())
