#!/usr/bin/env python3
"""Pinned same-factor batch and declared-model semi-synthetic evaluation."""

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


MODELS = ("independent", "ar_modeled", "ar_unmodeled", "shared_modeled", "shared_unmodeled",
          "independent_overconfident", "independent_conservative")
COVARIANCE_SCALES = {"independent": 1.0, "independent_overconfident": .25,
                     "independent_conservative": 4.0}


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def chi_square_quantile(probability, dimension):
    """Invert the exact even-degree chi-square survival series."""
    if not 0 < probability < 1 or dimension <= 0 or dimension % 2:
        raise ValueError("chi-square requires probability in (0,1) and even dimension")

    def cdf(x):
        if x <= 0:
            return 0.0
        z = x / 2
        terms = [k * math.log(z) - math.lgamma(k + 1)
                 for k in range(dimension // 2)]
        largest = max(terms)
        log_tail = min(0.0, -z + largest + math.log(sum(
            math.exp(value - largest) for value in terms)))
        return -math.expm1(log_tail)

    low, high = 0.0, float(dimension)
    while cdf(high) < probability:
        high *= 2
    for _ in range(90):
        middle = (low + high) / 2
        if cdf(middle) < probability:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def wilson(successes, count):
    if count <= 0 or not 0 <= successes <= count:
        raise ValueError("invalid binomial count")
    z = 1.959963984540054
    mean = successes / count
    denominator = 1 + z * z / count
    center = (mean + z * z / (2 * count)) / denominator
    half = z * math.sqrt(mean * (1 - mean) / count + z * z / (4 * count**2)) / denominator
    return [center - half, center + half]


def summarize(output, trials):
    with (output / "trials.csv").open() as stream:
        records = list(csv.DictReader(stream))
    with (output / "decisions.csv").open() as stream:
        decisions = list(csv.DictReader(stream))
    with (output / "factors.csv").open() as stream:
        factors = list(csv.DictReader(stream))
    with (output / "failures.csv").open() as stream:
        failures = list(csv.DictReader(stream))
    if len(records) + len(failures) != len(MODELS) * trials:
        raise ValueError("trial accounting mismatch")
    summary = {}
    lower, upper = (chi_square_quantile(p, 6) for p in (0.025, 0.975))
    for model in MODELS:
        selected = [r for r in records if r["model"] == model]
        failed = [r for r in failures if r["model"] == model]
        if (len(selected) + len(failed) != trials or
                {int(r["trial"]) for r in selected + failed} != set(range(trials))):
            raise ValueError("missing or duplicate trial")
        nis_checks = 0
        for row in (d for d in decisions if d["model"] == model):
            if row["innovation_available"] != "1":
                if row["status"] == "0":
                    raise ValueError("accepted generated observation lacks innovation")
                continue
            residual = [r[0] for r in _matrix(row["residual"])]
            calculated = sum(a * b for a, b in zip(residual, _solve(
                _matrix(row["covariance"]), residual)))
            if not math.isclose(calculated, float(row["nis"]), rel_tol=1e-8, abs_tol=1e-9):
                raise ValueError("unreproducible NIS")
            nis_checks += 1
        values = []
        for row in selected:
            dimension = len(_matrix(row["covariance"]))
            if dimension != 6:
                raise ValueError("NEES must use the declared full-rank 6D map chart")
            error = [r[0] for r in _matrix(row["pose_error"])]
            calculated = sum(a * b for a, b in zip(
                error, _solve(_matrix(row["covariance"]), error)))
            if not math.isclose(calculated, float(row["nees"]), rel_tol=1e-8, abs_tol=1e-9):
                raise ValueError("unreproducible NEES")
            values.append(calculated)
            ledger = [f for f in factors if f["model"] == model and f["trial"] == row["trial"]]
            full = [f for f in ledger if f["scope"] == "full_batch"]
            actual_observations = sum(f["type"].startswith("observation_") for f in full)
            journal_decisions = sum(d["model"] == model and d["trial"] == row["trial"]
                                    for d in decisions)
            if (len(full) != int(row["archived_factors"]) or
                    actual_observations != int(row["accepted"]) or
                    actual_observations != journal_decisions + 1 or int(row["rejected"])):
                raise ValueError("committed factor/decision accounting mismatch")
        if not values:
            summary[model] = {
                "trials": trials, "successful_trials": 0, "failed_trials": failed,
                "nees_mean": None, "mean_consistency": "unavailable_solver_failures",
                "nis_recomputed": nis_checks,
                "failure_stages": dict(Counter(r["stage"] for r in failed)),
                "failure_reasons": dict(Counter(r["reason"] for r in failed)),
            }
            continue
        # Three specified correctly modeled hypotheses; trials, not time points,
        # are the independent samples. Bonferroni controls the familywise rate.
        alpha = 0.05 / 3
        interval = [chi_square_quantile(p, 6 * len(values)) / len(values)
                    for p in (alpha / 2, 1 - alpha / 2)]
        mean = sum(values) / len(values)
        modeled = model in ("independent", "ar_modeled", "shared_modeled")
        direction = ("upper_tail_overconfidence" if mean > interval[1] else
                     "lower_tail_conservatism" if mean < interval[0] else
                     "inside_reference_interval")
        covered = sum(lower <= value <= upper for value in values)
        summary[model] = {
            "trials": trials, "nees": distribution(values), "nees_mean": mean,
            "successful_trials": len(values), "failed_trials": failed,
            "mean_reference_interval": interval,
            "mean_consistency": ("unavailable_selection_by_solver_failure" if failed else
                                 "not_observed_inconsistent" if interval[0] <= mean <= interval[1]
                                 else "inconsistency_observed"),
            "declared_correct_model": modeled,
            "mean_test_direction": direction,
            "mean_test_caveat": "lower-tail conservatism is not automatically a functional bug; "
                                "this is a Gaussian covariance-equality study, not bound qualification",
            "interpretation": ("only under this specified synthetic model; reality not calibrated"
                               if modeled else "deliberately misspecified observation covariance"
                               if model in COVARIANCE_SCALES else
                               "deliberately misspecified temporal/source independence"),
            "covariance_scale": COVARIANCE_SCALES.get(model, 1.0),
            "synthetic_translation_error_m": distribution([
                math.sqrt(sum(row[0] ** 2 for row in _matrix(r["pose_error"])[:3]))
                for r in selected]),
            "synthetic_rotation_error_rad": distribution([
                math.sqrt(sum(row[0] ** 2 for row in _matrix(r["pose_error"])[3:]))
                for r in selected]),
            "single_trial_95_percent_nees_interval": [lower, upper],
            "observed_coverage": covered / len(values),
            "coverage_wilson_95_percent_interval": wilson(covered, len(values)),
            "nis_recomputed": nis_checks,
            "batch_translation_difference_m": distribution([
                float(r["batch_translation_difference_m"]) for r in selected]),
            "batch_rotation_difference_rad": distribution([
                float(r["batch_rotation_difference_rad"]) for r in selected]),
            "maximum_batch_exact_residual": max(float(r["batch_exact_residual"]) for r in selected),
            "maximum_lag_exact_residual": max(float(r["lag_exact_residual"]) for r in selected),
            "retained_relinearizations": distribution([
                int(r["retained_relinearizations"]) for r in selected])
                if all("retained_relinearizations" in r for r in selected) else None,
            "retained_nodes": sorted({int(r["retained_nodes"]) for r in selected}),
            "archived_nodes": sorted({int(r["nodes"]) for r in selected}),
            "factor_types": dict(Counter(f["type"] for f in factors
                                         if f["model"] == model and f["scope"] == "full_batch")),
            "maximum_posterior_information_condition": max(
                float(r["posterior_information_condition"]) for r in selected),
            "update_max_ms": distribution([float(r["update_max_ms"]) for r in selected]),
        }
    return summary


def audit_covariance_pairs(output, trials):
    with (output / "generated.csv").open() as stream:
        generated = list(csv.DictReader(stream))
    with (output / "trials.csv").open() as stream:
        records = list(csv.DictReader(stream))
    reference = {}
    for row in generated:
        if row["model"] != "independent":
            continue
        key = (row["trial"], row["node"], row["source"])
        if key in reference:
            raise ValueError("duplicate covariance-study reference measurement")
        reference[key] = row
    expected_nodes = {int(key[1]) for key in reference}
    expected = {(str(trial), str(node), source) for trial in range(trials)
                for node in range(len(expected_nodes))
                for source in ("synthetic-map", "synthetic-gnss")}
    if len(expected_nodes) < 4 or set(reference) != expected:
        raise ValueError("incomplete covariance-study reference schedule")
    for model, scale in COVARIANCE_SCALES.items():
        selected = [r for r in generated if r["model"] == model]
        if (len(selected) != len(reference) or
                {(r["trial"], r["node"], r["source"]) for r in selected} != expected):
            raise ValueError("incomplete covariance-study paired schedule")
        for row in selected:
            base = reference[(row["trial"], row["node"], row["source"])]
            if row["measurement_error"] != base["measurement_error"]:
                raise ValueError("covariance-study changed actual sampled error")
            covariance, nominal = (_matrix(r["declared_covariance"]) for r in (row, base))
            dimension = 2 if row["source"] == "synthetic-gnss" else 6
            if (len(covariance) != dimension or len(nominal) != dimension or
                    any(len(r) != dimension for r in covariance + nominal) or
                    any(not math.isclose(covariance[i][j], scale * nominal[i][j],
                                         rel_tol=1e-12, abs_tol=1e-15)
                        for i in range(dimension) for j in range(dimension))):
                raise ValueError("covariance-study declared scale mismatch")
    by_trial = {(row["model"], int(row["trial"])): row for row in records}
    paired = {}
    for model in COVARIANCE_SCALES:
        differences = []
        for trial in range(trials):
            base, row = by_trial.get(("independent", trial)), by_trial.get((model, trial))
            if base is None or row is None:
                raise ValueError("covariance-study cannot select away failed trials")
            if base["seed"] != row["seed"]:
                raise ValueError("covariance-study independent trials are not paired")
            errors = [_matrix(value["pose_error"]) for value in (base, row)]
            differences.append(sum(r[0] ** 2 for r in errors[1][:3]) -
                               sum(r[0] ** 2 for r in errors[0][:3]))
        paired[model] = distribution(differences)
    return {
        "status": "verified_same_sampled_errors_and_declared_scales",
        "trials_per_scale": trials, "observation_covariance_scales": COVARIANCE_SCALES,
        "paired_translation_squared_error_difference_m2": paired,
        "scope": "fixed-admission synthetic backend study; not production robustness or field calibration",
    }


def audit_admission_schedule(output, nominal, trials, map_loss=None):
    with nominal.open() as stream:
        stamps = [int(row["timestamp_ns"]) for row in csv.DictReader(stream)]
    with (output / "generated.csv").open() as stream:
        generated = list(csv.DictReader(stream))
    with (output / "decisions.csv").open() as stream:
        decisions = list(csv.DictReader(stream))
    expected, flags = set(), {}
    for model in MODELS:
        for trial in range(trials):
            for node, stamp in enumerate(stamps):
                elapsed = (stamp - stamps[0]) / 1e9
                for source in ("synthetic-map", "synthetic-gnss"):
                    submitted = source != "synthetic-map" or not map_loss or not (
                        map_loss[0] <= elapsed < map_loss[1])
                    key = (model, str(trial), str(node), source)
                    flags[key] = str(int(submitted))
                    if submitted and not (node == 0 and source == "synthetic-map"):
                        expected.add(key)
    observed_flags = {(r["model"], r["trial"], r["node"], r["source"]): r["submitted"]
                      for r in generated}
    actual = [(r["model"], r["trial"], r["node"], r["source"]) for r in decisions]
    if len(generated) != len(flags) or observed_flags != flags:
        raise ValueError("declared generated/admission schedule differs")
    if map_loss and not any(value == "0" for value in flags.values()):
        raise ValueError("declared map-loss schedule never triggered")
    if len(actual) != len(expected) or set(actual) != expected:
        raise ValueError("declared decision/admission schedule differs")
    return {"status": "verified_declared_source_schedule", "decisions": len(actual),
            "map_loss_seconds_half_open": map_loss,
            "generated_but_not_submitted": sum(value == "0" for value in flags.values()),
            "scope": "all sampled noise retained for pairing; omitted maps never reach the graph"}


def run(args):
    if os.getuid() == 0:
        raise ValueError("run in the mapped-user managed container")
    if args.output.exists() or not 1 <= args.trials <= 10000:
        raise ValueError("output must be new and trials in 1..10000")
    if args.map_loss and not (0 < args.map_loss[0] < args.map_loss[1] and
                             all(math.isfinite(value) for value in args.map_loss)):
        raise ValueError("map loss must follow the anchor and have positive duration")
    args.output.mkdir(parents=True)
    with args.odometry.open() as stream:
        odometry = [row for row in csv.DictReader(stream) if row["valid"] == "1"]
    if len(odometry) < args.nodes or not 4 <= args.nodes <= 64:
        raise ValueError("insufficient valid nominal nodes")
    nominal = args.output / "nominal.csv"
    with nominal.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["timestamp_ns", "px", "py", "pz", "qx", "qy", "qz", "qw"])
        for index in range(args.nodes):
            row = odometry[index * (len(odometry) - 1) // (args.nodes - 1)]
            writer.writerow([row[key] for key in
                             ("timestamp_ns", "px", "py", "pz", "qx", "qy", "qz", "qw")])
    runtime = runtime_identity(args.binary)
    package = Path(__file__).resolve().parents[3]
    source = {
        str(path.relative_to(package)): digest(path)
        for directory in ("src", "include", "test/integration/map")
        for path in (package / directory).rglob("*")
        if path.is_file() and path.suffix in (".h", ".cc", ".py", ".csv")
    }
    manifest = {
        "schema": "openloc.fixed-lag-no-truth-evaluation.v1",
        "runtime": runtime, "source_sha256": source,
        "nominal_source": str(args.odometry), "nominal_source_sha256": digest(args.odometry),
        "nominal_sha256": digest(nominal), "nodes": args.nodes,
        "trials_per_model": args.trials, "seed_start": args.seed,
        "split": "development nominal sequence; frozen validation seeds, not tuned to pass",
        "models": list(MODELS),
        "map_loss_seconds_half_open": args.map_loss,
        "generator": {
            "nominal": "real replay trajectory treated as fictional truth; not physical truth",
            "local_error": "correction error [p_odom,v_odom,right_att_body,bg,ba]; "
                           "p0 sigma .02m, attitude0 .002rad; independent random-walk "
                           "increments .005m/sqrt(s), .0005rad/sqrt(s)",
            "observations": "map and GNSS XY sampled each node; maps in declared loss interval "
                            "are journaled but not submitted; independent noise sigma .02m, "
                            "map rotation .002rad, no antenna lever",
            "ar": "stationary 3D sigma .04m, rho .85 per selected node; shared by both sources",
            "shared": "one 3D sigma .04m constant bias shared across time and map/GNSS",
            "modeled": "hidden 3D joint state, exact cross-covariance, conditional factors",
            "unmodeled": "same marginal variance but false source/temporal independence",
            "covariance_misspecification": "independent sampled errors unchanged; "
                                          "declared map/GNSS covariance multiplied by .25/1/4; "
                                          "local error covariance unchanged",
        },
        "solver": {"lag_s": args.lag_seconds, "batch": "GaussNewton on all archived committed factors; "
                   "no timestamps/eviction in full covariance elimination",
                   "gaussian_limit_huber": 1e6, "fixed_admission_nis_gate": 1e6,
                   "max_batch_iterations": 30, "tolerance": 1e-10,
                   "retained_window": "bounded nonlinear relinearization with Gaussian "
                   "marginal A,b/noise frozen at its original chart; Pose3 chart derivatives "
                   "transport the prior, no resurrected observations",
                   "incremental_delta": "complete Bayes-tree solve; wildfire truncation disabled",
                   "scope": "evaluation-only fixed-admission experiment, not production gates"},
        "statistics": {"rank": 6, "chart": "translation_map metres, left_rotation_map radians",
                       "independent_units": "one final endpoint per independent trial",
                       "mean_interval": "exact chi-square, familywise alpha .05 over 3 matched models",
                       "coverage_interval": "Wilson 95%; descriptive, not calibrated field coverage"},
        "posterior_information": "inverse of latest full-rank pose marginal after nuisance "
                                 "elimination; scales 1m and 1rad; not raw geometry Hessian",
        "evidence_ceiling": "L2 semi-synthetic only; no real NEES, accuracy or live deadline claim",
    }
    save(args.output / "protocol.json", manifest)
    command = [str(args.binary), str(nominal), str(args.output / "replay"),
               str(args.trials), str(args.seed), str(args.lag_seconds)]
    if args.map_loss:
        command.extend(str(value) for value in args.map_loss)
    save(args.output / "command.json", command)
    with (args.output / "execution.log").open("w") as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    if runtime_identity(args.binary) != runtime or any(
            digest(package / relative) != expected for relative, expected in source.items()):
        raise ValueError("source/runtime changed during evaluation")
    result = summarize(args.output / "replay", args.trials)
    pairs = audit_covariance_pairs(args.output / "replay", args.trials)
    schedule = audit_admission_schedule(args.output / "replay", nominal, args.trials, args.map_loss)
    save(args.output / "summary.json", result)
    save(args.output / "covariance_pairs.json", pairs)
    save(args.output / "admission_schedule.json", schedule)
    body = []
    for model, report in result.items():
        body.append(f"<h2>{model}</h2><pre>{html.escape(json.dumps(report, indent=2))}</pre>")
    (args.output / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><h1>FixedLag / full-batch / NEES</h1>'
        '<p>Actual shared graph factors, same admission/cutoff. Nominal real trajectory is '
        'fictional synthetic truth. No real-data accuracy or calibrated covariance claim.</p>' +
        "".join(body))
    save(args.output / "artifact_hashes.json", {
        str(p.relative_to(args.output)): digest(p) for p in sorted(args.output.rglob("*"))
        if p.is_file() and p.name != "artifact_hashes.json"})
    print(json.dumps({m: {"nees_mean": r["nees_mean"], "finding": r["mean_consistency"]}
                      for m, r in result.items()}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--odometry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trials", type=int, default=128)
    parser.add_argument("--seed", type=int, default=270021)
    parser.add_argument("--nodes", type=int, default=12)
    parser.add_argument("--lag-seconds", type=float, default=5.0)
    parser.add_argument("--map-loss", type=float, nargs=2, metavar=("BEGIN_S", "END_S"),
                        help="omit only map submissions during this source-relative interval")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
