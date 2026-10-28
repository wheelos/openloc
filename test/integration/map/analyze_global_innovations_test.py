#!/usr/bin/env python3
"""Focused tests for innovation replay analysis."""

import csv
import math
from pathlib import Path
import tempfile
import unittest

from analyze_global_innovations import analyze


FIELDS = [
    "decision_id", "timestamp_ns", "sequence", "status", "message", "source",
    "innovation_available", "innovation_unavailable_reason",
    "observed_dimension", "pre_update_residual", "innovation_covariance", "nis",
    "residual_basis",
    "gate_threshold", "covariance_model", "covariance_decomposition_available",
    "innovation_prior_contribution", "innovation_observation_contribution",
    "innovation_local_contribution",
]


def event(index, residual, status="0", available="1"):
    return {
        "decision_id": str(index + 1),
        "timestamp_ns": str(1_000_000_000 + index),
        "sequence": str(index + 1),
        "status": status,
        "message": "INNOVATION_REJECTED" if status != "0" else "NONE",
        "source": "gnss",
        "innovation_available": available,
        "innovation_unavailable_reason": "" if available == "1" else "no_prior",
        "observed_dimension": "1" if available == "1" else "0",
        "pre_update_residual": f"1x1:{residual}" if available == "1" else "0x1:",
        "innovation_covariance": "1x1:1" if available == "1" else "0x0:",
        "nis": str(float(residual) ** 2) if available == "1" else "0",
    }


class InnovationAnalysisTest(unittest.TestCase):
    def analyze_events(self, events, **kwargs):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "decisions.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows(events)
            return analyze(path, **kwargs)

    def test_whitening_uses_full_covariance_and_reports_scale_and_tail(self):
        row = dict(event(0, 0), observed_dimension="2",
                   pre_update_residual="2x1:2;3",
                   innovation_covariance="2x2:4;2;2;5", nis="2")
        result = self.analyze_events([row])
        group = result["groups"][0]
        self.assertEqual([1, 1], [c["mean"] for c in group["whitened_residual"]])
        self.assertEqual([], group["whitened_residual"][0]["autocorrelation"])
        self.assertEqual([6, 3 * math.sqrt(5)],
                         group["innovation_three_sigma_component_max"])
        self.assertEqual(2, group["nis_p99"])
        self.assertAlmostEqual(0, group["whitened_residual"][0]["qq"][0][
            "theoretical_normal_quantile"])

    def test_gate_and_covariance_terms_are_recomputed_not_guessed(self):
        row = dict(event(0, 2), innovation_covariance="1x1:4", nis="1",
                   gate_threshold="22.458", covariance_model="diagnostic_ci_partial_bound",
                   covariance_decomposition_available="1",
                   innovation_prior_contribution="1x1:1",
                   innovation_observation_contribution="1x1:1",
                   innovation_local_contribution="1x1:2")
        gate = self.analyze_events([row])["gate_diagnostics"][0]
        self.assertAlmostEqual(1 / 22.458, gate["nis_over_gate"])
        self.assertEqual({"prior": [1], "observation": [1], "local": [2]},
                         gate["covariance_contribution_diagonals"])
        for change, message in (
                ({"innovation_local_contribution": "1x1:3"}, "do not sum"),
                ({"gate_threshold": "0.5"}, "exceeds recorded gate"),
                ({"gate_threshold": "nan"}, "invalid gate"),
                ({"innovation_prior_contribution": "2x1:1;0"}, "dimension")):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, message):
                self.analyze_events([dict(row, **change)])
        legacy = self.analyze_events([event(0, 1)])["gate_diagnostics"][0]
        self.assertFalse(legacy["gate_available"])
        self.assertIsNone(legacy["covariance_contribution_diagonals"])

    def test_time_windows_preserve_integer_nanoseconds(self):
        base = 1_791_882_552_000_000_000
        data = [dict(event(i, i + 1), timestamp_ns=str(base + i))
                for i in range(3)]
        result = self.analyze_events(data, window_ns=2)
        self.assertEqual([2, 1], [w["sample_count"] for w in result["time_windows"]])
        self.assertEqual(base + 2, result["time_windows"][-1]["window_start_ns"])
        self.assertEqual(3, len(result["groups"][0]["whitened_residual"][0]["qq"]))

    def test_cross_source_pairing_requires_explicit_same_basis(self):
        data = [dict(event(i, i + 1), source=source)
                for source in ("gnss", "map") for i in range(3)]
        result = self.analyze_events(data)
        self.assertFalse(result["cross_source_correlations"][0]["available"])
        for row in data:
            row["residual_basis"] = "map.xy_m.v1"
        result = self.analyze_events(data)
        pair = result["cross_source_correlations"][0]
        self.assertEqual(3, pair["pair_count"])
        self.assertEqual([1], pair["residual_component_correlation"])
        data[-1]["residual_basis"] = "body.xy_m.v1"
        self.assertFalse(self.analyze_events(data)[
            "cross_source_correlations"][0]["available"])
        data[2]["residual_basis"] = data[-1]["residual_basis"]
        self.assertFalse(self.analyze_events(data)[
            "cross_source_correlations"][0]["available"])

    def test_rejects_bad_shapes_nonfinite_and_nonpositive_covariances(self):
        cases = [
            {"observed_dimension": "0"},
            {"innovation_available": "yes"},
            {"pre_update_residual": "1x1:nan"},
            {"innovation_covariance": "1x1:0"},
            {"innovation_covariance": "1x1:-1"},
            {"innovation_covariance": "2x2:1;0;0;1"},
            {"observed_dimension": "2", "pre_update_residual": "2x1:1;1",
             "innovation_covariance": "2x2:1;1;0;1"},
        ]
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.analyze_events([dict(event(0, 1), **changes)])

    def test_recomputes_nis_and_separates_rejected_samples(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "decisions.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                writer.writerows([
                    event(0, 1), event(1, 2), event(2, 4),
                    event(3, 10, status="10"),
                    event(4, 0, available="0"),
                ])
            result = analyze(path, max_lag=2)
        self.assertEqual(5, result["decision_count"])
        self.assertEqual(4, result["innovation_sample_count"])
        self.assertEqual({"no_prior": 1}, result["unavailable_innovation_reasons"])
        accepted, rejected = result["groups"]
        self.assertEqual(("accepted", 3), (accepted["outcome"], accepted["sample_count"]))
        self.assertAlmostEqual(7.0, accepted["nis_mean"])
        self.assertEqual("rejected", rejected["outcome"])
        self.assertEqual(1, rejected["sample_count"])
        self.assertEqual(2, len(accepted["residual_autocorrelation"][0]))

    def test_rejects_non_recomputable_nis(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "decisions.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=FIELDS)
                writer.writeheader()
                row = event(0, 1)
                row["nis"] = "2"
                writer.writerow(row)
            with self.assertRaisesRegex(ValueError, "NIS mismatch"):
                analyze(path)


if __name__ == "__main__":
    unittest.main()
