#!/usr/bin/env python3
"""Exercise scene acceptance and ensure corrupted evidence cannot pass."""

import csv
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from evaluate_localization_scenarios import SCENARIOS, audit_case, rows


class LocalizationScenarioTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        cls.replay = cls.root / "replay"
        subprocess.run(["bazel-bin/localization_scenarios", str(cls.replay)],
                       check=True, capture_output=True, text=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def copied(self, name):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        target = Path(temporary.name) / name
        shutil.copytree(self.replay / name, target)
        return target

    def write(self, path, values):
        with path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=values[0].keys())
            writer.writeheader()
            writer.writerows(values)

    def test_all_scenarios_have_actual_triggers_and_cold_start_gap(self):
        results = {name: audit_case(self.replay / name) for name in SCENARIOS}
        self.assertEqual(0, results["gnss_only_cold_start"]["pose_states"])
        self.assertIsNone(results["gnss_only_cold_start"]["synthetic_exact_pose_error_m"]["max"])
        self.assertEqual(22, results["indoor_marker_no_map_gnss"]["pose_states"])
        self.assertGreater(sum(r["nis_recomputed"] for r in results.values()), 0)

    def test_missing_decision_and_incorrect_nis_are_rejected(self):
        target = self.copied("fault_rejection")
        decisions = rows(target / "decisions.csv")
        self.write(target / "decisions.csv", decisions[:5] + decisions[6:])
        with self.assertRaisesRegex(ValueError, "missing/duplicate terminal"):
            audit_case(target)
        fault = next(r for r in decisions if r["source"] == "gnss-xy" and r["node"] == "10")
        fault["nis"] = str(float(fault["nis"]) + 1)
        self.write(target / "decisions.csv", decisions)
        with self.assertRaisesRegex(ValueError, "NIS cannot be recomputed"):
            audit_case(target)

    def test_normal_degradation_passes_with_exact_parameter_coverage(self):
        joint = audit_case(self.replay / "joint_loss_reentry")
        self.assertEqual("passed_functional_contract", joint["status"])
        self.assertEqual("expected_degradation_and_recovery", joint["behavior"])
        self.assertFalse(joint["degradation_is_functional_failure"])
        coverage = joint["coverage"]
        self.assertEqual([[6, 18]], coverage["no_absolute_observation_node_intervals_half_open"])
        self.assertEqual([500_000_000], coverage["source_step_ns_tested"])
        self.assertTrue(coverage["parameters_are_tested_values_not_continuous_range"])
        self.assertTrue(coverage["uncovered"])
        delayed = audit_case(self.replay / "delayed_map")
        self.assertEqual([1_000_000_000], delayed["coverage"]["observation_delay_ns_tested"])
        partial = audit_case(self.replay / "map_degenerate_gnss_available")
        projected = partial["coverage"]["sources"]["scan-to-map-projected"]
        self.assertEqual([[6, 18]], projected["committed_node_intervals_half_open"])
        self.assertEqual([5], projected["innovation_dimensions_tested"])
        cold = audit_case(self.replay / "gnss_only_cold_start")
        self.assertEqual("expected_unobservable_cold_start", cold["behavior"])
        self.assertEqual("unobservable_full_pose_cold_start", cold["product_scope"])
        drift = audit_case(self.replay / "joint_loss_local_drift_reentry")
        self.assertEqual("passed_functional_contract", drift["status"])
        self.assertEqual("expected_degradation_and_recovery", drift["behavior"])
        self.assertAlmostEqual(.24, drift["synthetic_exact_pose_error_m"]["max"], places=8)
        self.assertFalse(drift["degradation_is_functional_failure"])

    def test_joint_loss_requires_degradation_in_the_actual_loss_interval(self):
        target = self.copied("joint_loss_reentry")
        states = rows(target / "states.csv")
        # A stale state after source return cannot prove the loss interval was
        # diagnosed; the retained pose must become stale during the actual gap.
        for state in states:
            if 6 <= int(state["node"]) < 18:
                state["full_pose_fresh"] = "1"
        states[20]["full_pose_fresh"] = "0"
        states[20]["trust"] = "0"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "joint-loss degradation/reentry not triggered"):
            audit_case(target)

    def test_false_zero_drift_cannot_claim_expected_degradation_coverage(self):
        target = self.copied("joint_loss_local_drift_reentry")
        states = rows(target / "states.csv")
        for state in states:
            if 6 <= int(state["node"]) < 18:
                state["x"] = state["truth_x"]
                state["y"] = state["truth_y"]
                state["z"] = state["truth_z"]
                state["error_m"] = "0"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "known loss-interval drift response"):
            audit_case(target)

    def test_stale_trust_and_missing_pose_cannot_be_scored_as_success(self):
        target = self.copied("map_degenerate_gnss_available")
        states = rows(target / "states.csv")
        stale = next(r for r in states if r["full_pose_fresh"] == "0" and r["pose_available"] == "1")
        stale["trust"] = "1"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "partial observations renewed"):
            audit_case(target)
        cold = self.copied("gnss_only_cold_start")
        states = rows(cold / "states.csv")
        states[0]["error_m"] = "0"
        self.write(cold / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "missing pose counted as zero"):
            audit_case(cold)


if __name__ == "__main__":
    unittest.main()
