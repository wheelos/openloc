#!/usr/bin/env python3
"""Statistical reference checks for the declared-model evaluator."""

import math
import csv
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from evaluate_fixed_lag import (
    MODELS, audit_admission_schedule, audit_covariance_pairs, chi_square_quantile, summarize, wilson,
)


class FixedLagEvaluationTest(unittest.TestCase):
    def test_map_loss_changes_admission_not_sampled_noise_and_cannot_hide_a_commit(self):
        nominal = Path(__file__).parent / "testdata/fixed_lag_turning_nominal.csv"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for begin, end in (("30", "0"), ("0", "15"), ("15", "3")):
                rejected = root / f"invalid_{begin}_{end}"
                result = subprocess.run([
                    "bazel-bin/fixed_lag_evaluation", str(nominal), str(rejected),
                    "1", "270021", "5", begin, end],
                    capture_output=True, text=True)
                self.assertNotEqual(0, result.returncode)
                self.assertFalse(rejected.exists())
            for name, extra in (("normal", []), ("loss", ["3", "15"])):
                subprocess.run(["bazel-bin/fixed_lag_evaluation", str(nominal),
                                str(root / name), "1", "270021", "5", *extra],
                               check=True, capture_output=True, text=True)
            normal, loss = root / "normal", root / "loss"
            for directory, interval in ((normal, None), (loss, [3, 15])):
                audit_admission_schedule(directory, nominal, 1, interval)
                audit_covariance_pairs(directory, 1)
                self.assertTrue(all(not r["failed_trials"] for r in summarize(directory, 1).values()))
            def generated(directory):
                with (directory / "generated.csv").open() as stream:
                    return list(csv.DictReader(stream))
            left, right = generated(normal), generated(loss)
            self.assertEqual([{k: v for k, v in row.items() if k != "submitted"} for row in left],
                             [{k: v for k, v in row.items() if k != "submitted"} for row in right])
            self.assertTrue(any(row["submitted"] == "0" for row in right))
            with self.assertRaisesRegex(ValueError, "schedule"):
                audit_admission_schedule(loss, nominal, 1)
            path = loss / "decisions.csv"
            with path.open() as stream:
                decisions = list(csv.DictReader(stream))
            # A fully self-consistent generated flag is not an independent oracle.
            absent = next(row for row in right if row["submitted"] == "0")
            forged = dict(decisions[-1])
            forged.update({key: absent[key] for key in ("model", "trial", "node", "source")})
            decisions.append(forged)
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=decisions[0].keys())
                writer.writeheader()
                writer.writerows(decisions)
            with self.assertRaisesRegex(ValueError, "schedule"):
                audit_admission_schedule(loss, nominal, 1, [3, 15])

    def test_retained_turning_graph_and_previous_no_eviction_failure_seeds(self):
        # Recorded geometry is fictional nominal truth, not a sensor reference.
        nominal = Path(__file__).parent / "testdata/fixed_lag_turning_nominal.csv"
        with tempfile.TemporaryDirectory() as temporary:
            for lag in (5, 60):
                for seed in (270099, 270143):
                    with self.subTest(lag=lag, seed=seed):
                        output = Path(temporary) / f"{lag}_{seed}"
                        subprocess.run([
                            "bazel-bin/fixed_lag_evaluation", str(nominal),
                            str(output), "1", str(seed), str(lag)],
                            check=True, capture_output=True, text=True)
                        reports = summarize(output, 1)
                        self.assertEqual("verified_same_sampled_errors_and_declared_scales",
                                         audit_covariance_pairs(output, 1)["status"])
                        self.assertTrue(all(r["successful_trials"] == 1 and
                                            not r["failed_trials"] for r in reports.values()))
                        self.assertTrue(all(r["maximum_lag_exact_residual"] <= 1e-6
                                            for r in reports.values()))
                        if lag == 60:
                            self.assertTrue(all(r["retained_relinearizations"]["max"] == 0
                                                for r in reports.values()))
                        else:
                            self.assertGreater(sum(r["retained_relinearizations"]["max"]
                                                   for r in reports.values()), 0)

    def test_covariance_pairs_reject_changed_errors_scales_or_missing_trials(self):
        nominal = Path(__file__).parent / "testdata/fixed_lag_turning_nominal.csv"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            original = root / "original"
            subprocess.run(["bazel-bin/fixed_lag_evaluation", str(nominal),
                            str(original), "1", "270021", "5"],
                           check=True, capture_output=True, text=True)
            for defect in ("noise", "scale", "missing_measurement", "failed_trial"):
                with self.subTest(defect=defect):
                    target = root / defect
                    shutil.copytree(original, target)
                    path = target / ("trials.csv" if defect == "failed_trial" else "generated.csv")
                    with path.open() as stream:
                        records = list(csv.DictReader(stream))
                    row = next(r for r in records if r["model"] == "independent_overconfident")
                    if defect in ("missing_measurement", "failed_trial"):
                        records.remove(row)
                    elif defect == "noise":
                        row["measurement_error"] = "6x1:0;0;0;0;0;0"
                    else:
                        row["declared_covariance"] = row["declared_covariance"].replace(
                            "0.0001", "0.0002", 1)
                    with path.open("w", newline="") as stream:
                        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
                        writer.writeheader()
                        writer.writerows(records)
                    with self.assertRaisesRegex(ValueError, "covariance-study"):
                        audit_covariance_pairs(target, 1)

    def test_exact_even_chi_square_reference_and_large_degrees(self):
        self.assertAlmostEqual(-2 * math.log(0.05), chi_square_quantile(0.95, 2), places=10)
        self.assertAlmostEqual(12.591587243743977, chi_square_quantile(0.95, 6), places=9)
        lower = chi_square_quantile(0.025, 768)
        upper = chi_square_quantile(0.975, 768)
        self.assertLess(lower, 768)
        self.assertGreater(upper, 768)
        with self.assertRaises(ValueError):
            chi_square_quantile(0.95, 3)

    def test_wilson_binomial_boundary_and_interval(self):
        self.assertAlmostEqual(0, wilson(0, 10)[0])
        self.assertAlmostEqual(1, wilson(10, 10)[1])
        lower, upper = wilson(95, 100)
        self.assertLess(lower, 0.95)
        self.assertGreater(upper, 0.95)
        with self.assertRaises(ValueError):
            wilson(0, 0)

    def test_solver_failures_are_counted_not_zero_nees_or_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for name in ("trials", "decisions", "factors"):
                (root / f"{name}.csv").write_text("model,trial\n")
            with (root / "failures.csv").open("w") as stream:
                writer = csv.DictWriter(stream, fieldnames=["model", "trial", "stage", "reason"])
                writer.writeheader()
                writer.writerows({"model": model, "trial": 0, "stage": "map",
                                  "reason": "exact constraint did not converge"}
                                 for model in MODELS)
            result = summarize(root, 1)
            self.assertTrue(all(r["nees_mean"] is None and r["successful_trials"] == 0
                                for r in result.values()))
            with self.assertRaisesRegex(ValueError, "accounting"):
                summarize(root, 2)


if __name__ == "__main__":
    unittest.main()
