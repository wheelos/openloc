"""Validate a frozen public XY engineering baseline without retuning the engine."""

import argparse
import bisect
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

import public_xy_validation as replay


NAME = "openloc-xy-tracking-baseline-v1"
REPEATS = 2


def held_out_indices(count):
    return list(range(20, count, 10))


def coverage(rows):
    seconds = dict.fromkeys(("uninitialized", "stale", "transition", "aligned"), 0.0)
    for row, following in zip(rows, rows[1:]):
        elapsed = (int(following["arrival_ns"]) - int(row["arrival_ns"])) / 1e9
        replay.require(elapsed >= 0, "output arrival time moved backwards")
        if row["global_valid"] == "0":
            state = "uninitialized"
        elif row["source_fresh"] == "0":
            state = "stale"
        elif row["pending"] == "1":
            state = "transition"
        else:
            state = "aligned"
        seconds[state] += elapsed
    seconds["total"] = sum(seconds.values())
    return seconds


def clean_scores(rows, nominal, odom):
    """Score every nominal epoch, including masked/corrupted-input epochs."""
    first = next(row for row in rows if row["global_valid"] == "1")
    by_time = {int(row["arrival_ns"]): row for row in rows}
    times = sorted(by_time)
    stamps = [int(row["timestamp_ns"]) for row in odom]
    seed = [float(first["published_" + key]) for key in ("tx", "ty", "yaw")]
    scores = []
    for sample in nominal:
        timestamp = int(sample["timestamp_ns"])
        processing = max(timestamp, stamps[bisect.bisect_left(stamps, timestamp)])
        row = by_time[times[bisect.bisect_left(times, processing)]]
        ax, ay = float(sample["ax"]), float(sample["ay"])
        gx, gy = float(sample["gx"]), float(sample["gy"])
        score = {
            "timestamp_ns": timestamp, "processing_ns": processing,
            "global_valid": int(row["global_valid"]),
            "source_fresh": int(row["source_fresh"]),
            "pending": int(row["pending"]),
        }
        for prefix in ("backend", "published"):
            yaw = float(row[prefix + "_yaw"])
            x = float(row[prefix + "_tx"]) + math.cos(yaw) * ax - math.sin(yaw) * ay
            y = float(row[prefix + "_ty"]) + math.sin(yaw) * ax + math.cos(yaw) * ay
            score[prefix + "_clean_xy_m"] = math.hypot(x - gx, y - gy)
        # A one-time initialization, not a trajectory-wide best fit. Before
        # initialization there is no global ODOM-only estimate to compare.
        score["seeded_odom_xy_m"] = (
            math.hypot(seed[0] + math.cos(seed[2]) * ax - math.sin(seed[2]) * ay - gx,
                       seed[1] + math.sin(seed[2]) * ax + math.cos(seed[2]) * ay - gy)
            if row["global_valid"] == "1" else ""
        )
        replay.require(all(math.isfinite(value) for value in score.values()
                           if value != ""), "nonfinite clean score")
        scores.append(score)
    initialized = [row for row in scores if row["global_valid"] == 1]
    metrics = {
        "all_epoch_count": len(scores),
        "preinitialization_epoch_count": len(scores) - len(initialized),
        "all_epoch_backend_median_p95_max_m": replay.quantiles(
            [row["backend_clean_xy_m"] for row in scores]),
        "all_epoch_published_median_p95_max_m": replay.quantiles(
            [row["published_clean_xy_m"] for row in scores]),
        "initialized_published_median_p95_max_m": replay.quantiles(
            [row["published_clean_xy_m"] for row in initialized]),
        "initialized_seeded_odom_median_p95_max_m": replay.quantiles(
            [row["seeded_odom_xy_m"] for row in initialized]),
    }
    return scores, metrics


def recovery_timeline(rows, nominal, factors):
    resumption = int(nominal[160]["timestamp_ns"])
    ordinary = sorted({int(row["timestamp_ns"]) for row in factors
                       if row["type"] == "ABS_POS2" and row["accepted"] == "1"
                       and int(row["timestamp_ns"]) >= resumption})
    reanchors = [int(row["arrival_ns"]) for previous, row in zip(rows, rows[1:])
                 if previous["alignment_revision"] != row["alignment_revision"]
                 and int(row["arrival_ns"]) >= resumption]
    caught_up = next(
        (int(row["arrival_ns"]) for row in rows
         if int(row["arrival_ns"]) >= resumption and row["global_valid"] == "1"
         and row["source_fresh"] == "1" and row["pending"] == "0"), None)
    return {
        "resumption_measurement_ns": resumption,
        "first_ordinary_readmission_s": (
            (ordinary[0] - resumption) / 1e9 if ordinary else None),
        "backend_reanchor_after_resumption_s": [
            (stamp - resumption) / 1e9 for stamp in reanchors],
        "first_fresh_publication_caught_up_s": (
            (caught_up - resumption) / 1e9 if caught_up else None),
        "caught_up_definition": "fresh and backend/publication means equal, "
                               "not independent accuracy or integrity",
    }


def execute(package, reference, output, container):
    replay.require(reference.is_relative_to(package) and output.is_relative_to(package),
                   "reference/output must be inside the package")
    replay.require(not output.exists(), "refuse existing baseline output")
    output.mkdir(parents=True)
    try:
        return validate(package, reference, output, container)
    except Exception as error:
        replay.save(output / "failure.json", {"error": str(error), "disposition": "STOP"})
        raise


def validate(package, reference, output, container):
    protocol = json.loads((reference / "protocol.json").read_text())
    artifacts = json.loads((reference / "artifacts.json").read_text())
    for name, expected in artifacts.items():
        replay.require(replay.digest(reference / name) == expected,
                       f"reference artifact changed: {name}")
    for name, expected in protocol["hashes_before"].items():
        replay.require(replay.digest(Path(name)) == expected,
                       f"frozen source/input changed: {name}")
    prefix = ["docker", "exec", "-u", f"{os.getuid()}:{os.getgid()}", container,
              "env", "-u", "LD_LIBRARY_PATH"]
    container_root = Path("/apollo/pkgs/openloc")

    def mapped(path):
        return str(container_root / path.relative_to(package))

    binary = output / "bin/public_xy_dataset_replay"
    binary.parent.mkdir()
    subprocess.run(prefix + ["cp", str(container_root / "bazel-bin/public_xy_dataset_replay"),
                             mapped(binary)], check=True)
    replay.require(replay.digest(binary) == protocol["binary_sha256"],
                   "candidate binary differs from frozen reference")
    source_files = [Path(__file__).resolve(), Path(replay.__file__).resolve(),
                    package / "test/integration/map/public_xy_baseline_test.py",
                    package / "MODULE.bazel", package / "MODULE.bazel.lock",
                    package / ".bazelrc"]
    source_pins = {str(path): replay.digest(path) for path in source_files}
    for path in source_files:
        destination = output / "verifier_source" / path.relative_to(package)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, destination)
    acceptance = {
        "name": NAME, "classification": "two-capture sparse XY engineering baseline",
        "production_allowed": False, "map_fusion_validated": False,
        "candidate_binary_sha256": protocol["binary_sha256"],
        "frozen_algorithm_protocol": str(reference / "protocol.json"),
        "source_pins": source_pins,
        "predeclared_checks": [
            "Two repeats of eight tracking runs exactly match frozen reference CSV/ledger",
            "Unmodified nominal reference scores every epoch, including faults/outage",
            "All tracking outputs account for original ODOM poses/epochs, no Z invented",
            "Every tracking event obeys 0.3 m/s and 1 degree/s; stale output cannot move",
            "No post-initialization publication reset; degraded if pending or stale",
            "Each scenario P95 beats frozen conservative publication and seeded ODOM",
            "Holdout fixes absent from input/ledger; holdout P95 beats seeded ODOM",
            "Terminal GNSS loss preserves publication and exposes stale/degraded",
        ],
        "repetitions_per_primary_and_holdout": REPEATS,
        "holdout": "omit indices 20,30,40,... before public submission; receiver "
                   "continuity is allowed to reset under unchanged 1.5 s gap gate",
        "terminal_loss": "submit only nominal indices0..99, retain entire ODOM",
        "reference_limits": "unmodified prepared same-source GNSS, not independent "
                            "truth; holdout is absent from factors/recovery but "
                            "does not remove shared receiver/frame errors",
        "comparison": "ODOM-only freezes first accepted nominal publication; no "
                      "future trajectory fit or online GNSS after that one seed",
        "regression_policy": "byte-exact frozen primary repeats, not a new accuracy threshold",
    }
    replay.save(output / "acceptance.json", acceptance)
    commands, results = [], []
    for cell in replay.CELLS:
        destination = output / cell
        destination.mkdir()
        inputs = reference / cell
        prepared = package / ".agents/notes/two_capture_validation_20261015/horizontal_v2" / cell
        nominal = replay.read(prepared / "input.csv")
        odom = replay.read(inputs / "odom.csv")
        fixes = replay.read(inputs / "nominal_fixes.csv")
        withheld = set(held_out_indices(len(nominal)))
        replay.write(destination / "holdout_fixes.csv", replay.FIX_HEADER,
                     [list(row.values()) for index, row in enumerate(fixes)
                      if index not in withheld])
        replay.write(destination / "terminal_loss_fixes.csv", replay.FIX_HEADER,
                     [list(row.values()) for row in fixes[:100]])
        control_hashes = {str(path): replay.digest(path)
                          for path in destination.glob("*_fixes.csv")}
        first_outputs = {}
        for scenario in (*replay.SCENARIOS, "holdout", "terminal_loss"):
            repeats = 1 if scenario == "terminal_loss" else REPEATS
            fix_path = ((destination if scenario in ("holdout", "terminal_loss")
                         else inputs) / (scenario + "_fixes.csv"))
            scenario_samples = (replay.read(prepared / replay.SCENARIOS[scenario])
                                if scenario in replay.SCENARIOS else nominal)
            for repetition in range(repeats):
                path = destination / f"{scenario}_repeat{repetition}_tracking.csv"
                command = prefix + [mapped(binary), mapped(inputs / "odom.csv"),
                                    mapped(fix_path), mapped(path), "tracking",
                                    "--diagnostic-synthetic-quality",
                                    mapped(inputs / "lever.csv")]
                commands.append(command)
                replay.save(output / "commands.json", commands)
                run = subprocess.run(command, text=True, capture_output=True, check=False)
                path.with_suffix(".log").write_text(run.stdout + run.stderr)
                replay.require(run.returncode == 0, f"{cell}/{scenario}: {run.stderr}")
                rows, software = replay.evaluate(path, scenario_samples, inputs)
                for row in rows:
                    if row["pending"] == "1" or row["source_fresh"] == "0":
                        replay.require(row["global_degraded"] == "1",
                                       "pending/stale output not marked degraded")
                for extension in ("", ".factors.csv"):
                    expected = (inputs / (scenario + "_tracking.csv" + extension)
                                if scenario in replay.SCENARIOS else
                                Path(str(first_outputs.get(scenario, path)) + extension))
                    replay.require(replay.digest(Path(str(path) + extension)) ==
                                   replay.digest(expected), "deterministic replay mismatch")
                first_outputs.setdefault(scenario, path)
                scores, metrics = clean_scores(rows, nominal, odom)
                replay.write(Path(str(path) + ".clean_scores.csv"), ",".join(scores[0]),
                             [list(row.values()) for row in scores])
                factors = replay.read(Path(str(path) + ".factors.csv"))
                replay.require(len(scores) == len(nominal), "score coverage differs")
                result = {
                    "query": cell.split("_query_")[1], "scenario": scenario,
                    "repetition": repetition, "software": software,
                    "clean_reference": metrics, "time_weighted_state_seconds": coverage(rows),
                    "csv_sha256": replay.digest(path),
                }
                if scenario in replay.SCENARIOS:
                    _, conservative = clean_scores(
                        replay.read(inputs / (scenario + "_recovery.csv")), nominal, odom)
                    replay.require(metrics["all_epoch_published_median_p95_max_m"][1] <
                                   conservative["all_epoch_published_median_p95_max_m"][1],
                                   "tracking P95 did not improve over conservative publication")
                    replay.require(metrics["initialized_published_median_p95_max_m"][1] <
                                   metrics["initialized_seeded_odom_median_p95_max_m"][1],
                                   "tracking P95 did not improve over seeded ODOM")
                    result["conservative_published_median_p95_max_m"] = conservative[
                        "all_epoch_published_median_p95_max_m"]
                if scenario == "outage":
                    result["recovery_timeline"] = recovery_timeline(rows, nominal, factors)
                    replay.require(result["recovery_timeline"][
                        "first_fresh_publication_caught_up_s"] is not None,
                        "outage never returned to fresh backend/publication agreement")
                if scenario == "holdout":
                    held_stamps = {nominal[index]["timestamp_ns"] for index in withheld}
                    replay.require(not any(row["timestamp_ns"] in held_stamps
                                           for row in factors), "holdout leaked into factors")
                    held_scores = [scores[index] for index in sorted(withheld)]
                    comparison = {
                        "epochs": len(held_scores),
                        "tracking_median_p95_max_m": replay.quantiles(
                            [row["published_clean_xy_m"] for row in held_scores]),
                        "seeded_odom_median_p95_max_m": replay.quantiles(
                            [row["seeded_odom_xy_m"] for row in held_scores]),
                        "absent_from_public_submission_and_factor_ledger": True,
                    }
                    replay.require(comparison["tracking_median_p95_max_m"][1] <
                                   comparison["seeded_odom_median_p95_max_m"][1],
                                   "withheld-epoch P95 did not improve over seeded ODOM")
                    result["withheld_reference"] = comparison
                if scenario == "terminal_loss":
                    last_fix = int(fixes[99]["timestamp_ns"])
                    stale = [row for row in rows
                             if int(row["odom_ns"]) > last_fix + 1_500_000_000]
                    replay.require(stale and all(
                        row["source_fresh"] == "0" and row["global_degraded"] == "1"
                        and float(row["correction_jump_m"]) < 1e-8 for row in stale),
                        "terminal loss did not stop publication with explicit degradation")
                    result["last_submitted_fix_ns"] = last_fix
                results.append(result)
                replay.save(output / "progress.json", {"completed_executions": len(results)})
        for name, expected in control_hashes.items():
            replay.require(replay.digest(Path(name)) == expected, "control input changed")
    for name, expected in {**protocol["hashes_before"], **source_pins}.items():
        replay.require(replay.digest(Path(name)) == expected, f"input/source changed: {name}")
    replay.require(replay.digest(binary) == protocol["binary_sha256"], "binary changed")
    replay.save(output / "evaluation.json", {
        "name": NAME, "status": "PASS_ENGINEERING_BASELINE",
        "production_allowed": False, "map_fusion_validated": False,
        "new_executions": len(results),
        "new_output_events": sum(row["software"]["events"] for row in results),
        "repeated_primary_executions": 16, "holdout_executions": 4,
        "terminal_loss_executions": 2, "results": results,
    })
    replay.save(output / "artifacts.json", {
        str(path.relative_to(output)): replay.digest(path)
        for path in sorted(output.rglob("*")) if path.is_file()
    })
    print(json.dumps({"output": str(output), "baseline": NAME,
                      "executions": len(results), "status": "PASS_ENGINEERING_BASELINE"}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--container", required=True)
    args = parser.parse_args()
    execute(args.package.resolve(), args.reference.resolve(),
            args.output.resolve(), args.container)


if __name__ == "__main__":
    main()
