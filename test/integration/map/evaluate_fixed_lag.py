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


MODELS = ("independent", "ar_modeled", "ar_unmodeled", "shared_modeled", "shared_unmodeled")


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
        covered = sum(lower <= value <= upper for value in values)
        summary[model] = {
            "trials": trials, "nees": distribution(values), "nees_mean": mean,
            "successful_trials": len(values), "failed_trials": failed,
            "mean_reference_interval": interval,
            "mean_consistency": ("unavailable_selection_by_solver_failure" if failed else
                                 "not_observed_inconsistent" if interval[0] <= mean <= interval[1]
                                 else "inconsistency_observed"),
            "declared_correct_model": modeled,
            "interpretation": ("only under this specified synthetic model; reality not calibrated"
                               if modeled else "deliberately misspecified temporal/source independence"),
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


def run(args):
    if os.getuid() == 0:
        raise ValueError("run in the mapped-user managed container")
    if args.output.exists() or not 1 <= args.trials <= 10000:
        raise ValueError("output must be new and trials in 1..10000")
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
        "generator": {
            "nominal": "real replay trajectory treated as fictional truth; not physical truth",
            "local_error": "correction error [p_odom,v_odom,right_att_body,bg,ba]; "
                           "p0 sigma .02m, attitude0 .002rad; independent random-walk "
                           "increments .005m/sqrt(s), .0005rad/sqrt(s)",
            "observations": "full map and GNSS XY each node; independent noise sigma .02m, "
                            "map rotation .002rad, no antenna lever",
            "ar": "stationary 3D sigma .04m, rho .85 per selected node; shared by both sources",
            "shared": "one 3D sigma .04m constant bias shared across time and map/GNSS",
            "modeled": "hidden 3D joint state, exact cross-covariance, conditional factors",
            "unmodeled": "same marginal variance but false source/temporal independence",
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
    save(args.output / "command.json", command)
    with (args.output / "execution.log").open("w") as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    if runtime_identity(args.binary) != runtime or any(
            digest(package / relative) != expected for relative, expected in source.items()):
        raise ValueError("source/runtime changed during evaluation")
    result = summarize(args.output / "replay", args.trials)
    save(args.output / "summary.json", result)
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
    run(parser.parse_args())


if __name__ == "__main__":
    main()
