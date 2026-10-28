#!/usr/bin/env python3
"""Statistical reference checks for the declared-model evaluator."""

import math
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest

from evaluate_fixed_lag import MODELS, chi_square_quantile, summarize, wilson


class FixedLagEvaluationTest(unittest.TestCase):
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
