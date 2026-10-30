#!/usr/bin/env python3
"""Frozen real native/ICP replay with explicitly injected diagnostic faults.

The prepared bundle retains original nodes, map poses, covariance/root, and
the single usable recorded GNSS. Repeated GNSS is a synthetic fixed-anchor
model, not real receiver loss/reacquisition or field qualification.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


FAULTS = ("map_loss", "map_outlier", "recorded_gnss_omitted",
          "synthetic_nominal", "gnss_loss",
          "gnss_omitted", "gnss_outlier", "conflict", "both_loss")
ARMS = ("lio", "gnss", "map", "all")
MODES = ("lag", "no_eviction")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if any(None in row or None in row.values() for row in rows):
        raise ValueError(f"invalid schema: {path}")
    return rows


def write(path, fields, rows):
    with path.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields.split(","))
        writer.writeheader()
        writer.writerows(rows)


def prepare(canonical, output):
    output.mkdir()
    replay = canonical / "replay"
    nodes = read(replay / "structured_nodes.csv")
    maps = read(replay / "structured_map.csv")
    if len(nodes) != 5 or len(maps) != 4:
        raise ValueError("fixed fault schedule needs five nodes/four actual maps")
    for row in maps:
        if (row["valid"], row["observed_rank"],
                row["structured_error_available"], row["model_reason"]) != (
                    "1", "6", "1", "NONE"):
            raise ValueError("selected original map is not valid structured ICP")
    for source, target in (
            (replay / "structured_nodes.csv", "nodes.csv"),
            (replay / "structured_family_root.csv", "root.csv"),
            (replay / "structured_family_covariance.csv", "covariance.csv"),
            (canonical / "gnss_prefix/identity.csv", "identity.csv")):
        shutil.copyfile(source, output / target)
    map_rows = [{
        "timestamp_ns": row["reference_ns"], "receive_ns": row["receive_ns"],
        "sequence": row["scan_sequence"],
        **{key: row[key] for key in ("px", "py", "pz", "qx", "qy", "qz", "qw")}
    } for row in maps]
    write(output / "maps.csv",
          "timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw", map_rows)
    times = {row["timestamp_ns"] for row in nodes}
    gnss = [row for row in read(replay / "gnss.csv")
            if row["timestamp_ns"] in times and row["gate_outcome"] == "accepted"]
    if len(gnss) != 1:
        raise ValueError("expected exactly one original usable GNSS, not recovery")
    fields = "timestamp_ns,receive_ns,sequence,x,y,sigma_m,lever_x,lever_y,lever_z"
    write(output / "gnss.csv", fields,
          [{key: row[key] for key in fields.split(",")} for row in gnss])
    sources = [replay / name for name in (
        "structured_nodes.csv", "structured_map.csv", "structured_family.csv",
        "structured_family_covariance.csv", "structured_family_root.csv",
        "gnss.csv", "odometry.csv")]
    sources += [canonical / "gnss_prefix" / name
                for name in ("identity.csv", "events.csv", "config.csv")]
    manifest = {
        "canonical": str(canonical.resolve()),
        "source_sha256": {str(p.resolve()): digest(p) for p in sources},
        "bundle_sha256": {p.name: digest(p) for p in output.iterdir()},
        "recorded_usable_gnss": 1,
        "native_nodes": 5, "actual_icp": 4,
        "epoch": "replay-scoped frozen-selected-native-family/1; "
                 "original node journal does not serialize epoch",
        "synthetic_gnss": {
            "nodes": [r["timestamp_ns"] for r in nodes[2:]],
            "model": "deterministic first-map/native anchor, original lever and sigma",
            "cross_source_independence": "assumed diagnostic only, not qualified",
            "recorded_receiver_loss_recovery": False},
        "fault_schedule": {
            "map_loss": "omit third map; return fourth map",
            "map_outlier": "third map +100m x; fourth map unchanged",
            "recorded_gnss_omitted": "omit sole usable recorded GNSS; no GNSS return",
            "synthetic_nominal": "synthetic GNSS at last three original nodes",
            "gnss_loss": "synthetic nominal, omit middle, return last",
            "gnss_omitted": "omit first synthetic GNSS; rejected-control comparator",
            "gnss_outlier": "first synthetic GNSS +100m x; next two nominal",
            "conflict": "first two synthetic GNSS +100m x; actual map unchanged",
            "both_loss": "omit third map and first two synthetic GNSS; return last"},
        "unchanged": ["native inputs", "native node times", "map return times",
                      "two common map anchors", "99x99 covariance", "99x96 root",
                      "graph gates", "noise", "300ms lag", "2 verification frames"],
        "not_claimed": ["real GNSS reacquisition", "accuracy", "calibration",
                        "long run", "indefinite streaming", "full 4x4 coverage"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def check_pins(bundle):
    manifest = json.loads((bundle / "manifest.json").read_text())
    for name, expected in manifest["bundle_sha256"].items():
        if digest(bundle / name) != expected:
            raise ValueError(f"frozen input pin changed: {name}")
    for path, expected in manifest["source_sha256"].items():
        if digest(Path(path)) != expected:
            raise ValueError(f"original source pin changed: {path}")
    return manifest


def verify(bundle, output):
    manifest = check_pins(bundle)
    failures = []
    checks = []
    summaries = []
    statuses = {row["run"]: row for row in read(output / "run_status.csv")}

    def require(condition, label):
        checks.append({"check": label, "passed": bool(condition)})
        if not condition:
            failures.append(label)

    for fault in ("", *FAULTS):
        for arm in ARMS:
            for mode in MODES:
                prefix = (f"{fault}_" if fault else "") + f"{arm}_{mode}"
                decisions = read(output / f"{prefix}_decisions.csv")
                states = read(output / f"{prefix}_states.csv")
                events = read(output / f"{prefix}_interventions.csv")
                status = statuses[prefix]
                require(status["status"] == "completed",
                        prefix + ": graph execution completed")
                if status["status"] != "completed":
                    nodes = read(bundle / "nodes.csv")
                    summaries.append({
                        "run": prefix, "status": "failed", "error": status["error"],
                        "last_successful_ns": states[-1]["timestamp_ns"] if states else None,
                        "first_failure_ns": nodes[len(states)]["timestamp_ns"]
                        if len(states) < len(nodes) else None,
                        "actual_decisions_before_failure": decisions,
                        "interventions_before_failure": events,
                    })
                    continue
                require(len(states) == 5, prefix + ": all original nodes")
                require(states[0]["estimate_available"] == "0" and
                        states[0]["verification_pending"] == "1",
                        prefix + ": first anchor requires distinct frame")
                require(all(s["estimate_available"] == "1" and
                            s["diagnostic_only"] == "1" and s["trust_level"] != "2"
                            for s in states[1:]), prefix + ": diagnostic estimates")
                require([s["timestamp_ns"] for s in states] ==
                        [s["timestamp_ns"] for s in read(bundle / "nodes.csv")],
                        prefix + ": original times")
                require(sum(d["source"] == "actual-structured-ICP"
                            for d in decisions[:2]) == 2,
                        prefix + ": common map anchors")
                omitted = [e for e in events if e["action"] == "omitted"]
                for e in omitted:
                    require(not any(d["source"] == e["source"] and
                                    d["measurement_ns"] == e["measurement_ns"]
                                    for d in decisions),
                            prefix + ": omission not submitted " + e["source"])
                injected = [e for e in events if e["action"] == "injected"]
                for e in events:
                    if e["action"] not in ("original", "synthetic_nominal"):
                        continue
                    if e["measurement_ns"] == states[0]["timestamp_ns"]:
                        continue  # First common anchor is a verifying hypothesis.
                    nominal = [d for d in decisions if d["source"] == e["source"]
                               and d["measurement_ns"] == e["measurement_ns"]]
                    require(len(nominal) == 1 and nominal[0]["status"] == "0" and
                            nominal[0]["alignment_updated"] == "1",
                            prefix + ": nominal source update " + e["source"] +
                            "@" + e["measurement_ns"])
                for e in injected:
                    found = [d for d in decisions if d["source"] == e["source"]
                             and d["measurement_ns"] == e["measurement_ns"]]
                    require(len(found) == 1 and
                            found[0]["alignment_updated"] == "0" and
                            found[0]["innovation_available"] == "1" and
                            float(found[0]["preupdate_nis"]) >
                            float(found[0]["gate_threshold"]),
                            prefix + ": injected fault actually gated " +
                            e["source"])
                if fault in ("gnss_loss", "gnss_outlier", "conflict", "both_loss"):
                    if arm in ("gnss", "all"):
                        final = [d for d in decisions if d["source"].startswith(
                            "SYNTHETIC") and d["measurement_ns"] ==
                            states[-1]["timestamp_ns"]]
                        require(len(final) == 1 and final[0]["alignment_updated"] == "1",
                                prefix + ": later distinct-frame synthetic GNSS update")
                if fault in ("map_loss", "map_outlier", "both_loss"):
                    if arm in ("map", "all"):
                        returned = [d for d in decisions if
                                    d["source"] == "actual-structured-ICP" and
                                    d["measurement_ns"] == states[-1]["timestamp_ns"]]
                        require(len(returned) == 1 and
                                returned[0]["alignment_updated"] == "1",
                                prefix + ": unchanged later actual map update")
                summaries.append({
                    "run": prefix, "status": "completed", "submissions": len(decisions),
                    "updates": sum(d["alignment_updated"] == "1" for d in decisions),
                    "omitted": len(omitted), "injected": len(injected),
                    "gate_rejections": sum(
                        d["innovation_available"] == "1" and
                        float(d["preupdate_nis"]) > float(d["gate_threshold"])
                        for d in decisions),
                    "verification_pending_final": states[-1]["verification_pending"],
                    "first_injected_decision": next(
                        (d for d in decisions if any(
                            d["source"] == e["source"] and
                            d["measurement_ns"] == e["measurement_ns"]
                            for e in injected)), None),
                    "first_nonzero_decision": next(
                        (d for d in decisions if d["status"] != "0"), None)})
    # Replayed controls must retain the original graph behavior, not just names.
    canonical = Path(manifest["canonical"]) / "replay"
    for arm in ARMS:
        for mode in MODES:
            prefix = f"{arm}_{mode}"
            original = read(canonical / f"{prefix}_states.csv")
            current = read(output / f"{prefix}_states.csv")
            require(statuses[prefix]["status"] == "completed" and
                    len(original) == len(current) and
                    all(a["estimate_available"] == b["estimate_available"] and
                        a["correction_id"] == b["correction_id"] and
                        all(abs(float(a[k]) - float(b[k])) < 1e-10
                            for k in ("px", "py", "pz") if a[k])
                        for a, b in zip(original, current)),
                    prefix + ": original baseline graph reproduced")
            original_decisions = read(canonical / f"{prefix}_decisions.csv")
            current_decisions = read(output / f"{prefix}_decisions.csv")
            require(statuses[prefix]["status"] == "completed" and
                    len(original_decisions) == len(current_decisions) and all(
                all(a[k] == b[k] for k in (
                    "source", "sequence", "measurement_ns", "receive_ns",
                    "status", "alignment_updated", "innovation_available"))
                for a, b in zip(original_decisions, current_decisions)),
                prefix + ": original baseline decisions reproduced")
    # A rejected horizontal outlier must behave like omitting that observation,
    # not like accepting it. Full-pose outliers also affect recovery policy.
    for source, arms, omit in (("gnss", ("gnss", "all"), "gnss_omitted"),
                               ("map", ("map", "all"), "map_loss")):
        for arm in arms:
            for mode in MODES:
                rejected = read(output / f"{source}_outlier_{arm}_{mode}_states.csv")
                omitted = read(output / f"{omit}_{arm}_{mode}_states.csv")
                require(len(rejected) == len(omitted) == 5 and
                        all(all(abs(float(a[k]) - float(b[k])) < 1e-10
                                for k in ("px", "py", "pz"))
                            for a, b in zip(rejected[1:], omitted[1:])),
                        f"{arm}_{mode}: rejected {source} equals omitted control")
    report = {
        "status": "passed" if not failures else "failed",
        "graph_runs": len(summaries), "checks": len(checks),
        "completed_graph_runs": sum(r["status"] == "completed" for r in summaries),
        "failed_graph_runs": sum(r["status"] == "failed" for r in summaries),
        "failures": failures, "first_failure": failures[0] if failures else None,
        "runs": summaries, "assertions": checks,
        "model_boundary": manifest["not_claimed"],
        "frozen_source_pins_unchanged": True,
    }
    (output / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    if failures:
        raise AssertionError(failures[0])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("canonical", type=Path)
    parser.add_argument("binary", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir()
    bundle = args.output / "bundle"
    prepare(args.canonical, bundle)
    check_pins(bundle)
    started = time.monotonic()
    binary = args.binary.resolve(strict=True)
    root = Path(__file__).resolve().parents[3]
    source_files = [root / "BUILD"]
    for directory in ("include", "src", "test/integration/map"):
        source_files += [p for p in (root / directory).rglob("*")
                         if p.suffix in (".h", ".cc", ".py", ".json")]
    sources = {str(p.relative_to(root)): digest(p) for p in source_files}
    command = [str(binary), str(bundle.resolve()),
               str((args.output / "graphs").resolve())]
    result = subprocess.run(command, capture_output=True, text=True)
    (args.output / "stdout.log").write_text(result.stdout)
    (args.output / "stderr.log").write_text(result.stderr)
    execution = {
        "command": command, "binary_sha256": digest(binary),
        "returncode": result.returncode, "wall_s": time.monotonic() - started,
        "source_sha256": sources,
        "sources_unchanged_during_execution": all(
            digest(root / name) == expected for name, expected in sources.items()),
    }
    (args.output / "execution.json").write_text(json.dumps(execution, indent=2) + "\n")
    result.check_returncode()
    if not execution["sources_unchanged_during_execution"]:
        raise ValueError("source changed during execution; result not accepted")
    try:
        report = verify(bundle, args.output / "graphs")
    except AssertionError:
        report = json.loads((args.output / "graphs/verification.json").read_text())
    print(json.dumps({k: report[k] for k in (
        "status", "graph_runs", "completed_graph_runs", "failed_graph_runs",
        "checks", "first_failure")}))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
