#!/usr/bin/env python3
"""Exercise scene acceptance and ensure corrupted evidence cannot pass."""

import csv
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from evaluate_localization_scenarios import (
    BURST_SCENARIOS, FIELD_PROFILE, MAP_LOSS_END, SCENARIOS, WINDOW_LOSS_END, audit_case,
    audit_rejected_subset, field_availability_profile, rows,
)


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

    def rewrite_ledger(self, target, decisions):
        for index, event in enumerate(decisions, 1):
            event["decision_id"] = str(index)
        states = rows(target / "states.csv")
        for state in states:
            events = [r for r in decisions if int(r["node"]) <= int(state["node"])]
            rejected = sum(r["status"] != "0" for r in events)
            updated = sum(r["updated"] == "1" for r in events)
            for key, value in zip(
                ("submitted", "rejected", "updates", "retained"),
                (len(events), rejected, updated, len(events) - rejected - updated),
            ):
                state[key] = str(value)
        self.write(target / "states.csv", states)
        self.write(target / "decisions.csv", decisions)

    def test_self_consistent_ledger_cannot_hide_missing_source_or_update(self):
        for defect in ("missing_source", "uncommitted_map", "hidden_uncommitted_map",
                       "retimed_map", "retimed_receive"):
            with self.subTest(defect=defect):
                target = self.copied("outdoor_gnss_map")
                decisions = rows(target / "decisions.csv")
                event = next(r for r in decisions if r["node"] == "10" and
                             r["source"] == ("gnss-xy" if defect == "missing_source"
                                             else "scan-to-map"))
                if defect == "missing_source":
                    decisions.remove(event)
                elif defect in ("uncommitted_map", "hidden_uncommitted_map"):
                    event["updated"] = "0"
                    if defect == "uncommitted_map":
                        event["expected"] = "retained_unanchored"
                elif defect == "retimed_map":
                    event["measurement_ns"] = str(int(event["measurement_ns"]) - 1)
                else:
                    event["receive_ns"] = str(int(event["receive_ns"]) + 1)
                self.rewrite_ledger(target, decisions)
                with self.assertRaisesRegex(ValueError, "declared scenario"):
                    audit_case(target)

    def test_uniformly_shifted_state_clock_is_not_the_declared_source_clock(self):
        target = self.copied("outdoor_gnss_map")
        states = rows(target / "states.csv")
        for state in states:
            state["timestamp_ns"] = str(int(state["timestamp_ns"]) + 1)
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "declared scenario source time"):
            audit_case(target)

    def test_rotation_cannot_hide_behind_exact_translation(self):
        for defect in ("unlogged_error", "logged_error", "rewritten_truth",
                       "nonunit", "nonfinite"):
            with self.subTest(defect=defect):
                target = self.copied("outdoor_gnss_map")
                states = rows(target / "states.csv")
                state = states[10]
                if defect in ("unlogged_error", "logged_error", "rewritten_truth"):
                    half_yaw = 0.5 * (0.4 + 0.03 * 10 + 0.2)
                    state["qz"], state["qw"] = str(math.sin(half_yaw)), str(math.cos(half_yaw))
                    if defect == "logged_error":
                        state["rotation_error_rad"] = "0.2"
                    elif defect == "rewritten_truth":
                        state["truth_qz"], state["truth_qw"] = state["qz"], state["qw"]
                elif defect == "nonunit":
                    state["qw"] = "2"
                else:
                    state["qx"] = "nan"
                self.write(target / "states.csv", states)
                with self.assertRaisesRegex(ValueError, "orientation"):
                    audit_case(target)
        cold = self.copied("gnss_only_cold_start")
        states = rows(cold / "states.csv")
        states[0]["rotation_error_rad"] = "0"
        self.write(cold / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "invented orientation"):
            audit_case(cold)

    def test_self_consistent_rewritten_position_truth_is_rejected(self):
        target = self.copied("outdoor_gnss_map")
        states = rows(target / "states.csv")
        states[10]["x"] = states[10]["truth_x"] = "0"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "declared scenario position truth"):
            audit_case(target)

    def test_quaternion_sign_does_not_change_physical_rotation(self):
        target = self.copied("outdoor_gnss_map")
        states = rows(target / "states.csv")
        for key in ("qx", "qy", "qz", "qw"):
            states[10][key] = str(-float(states[10][key]))
            states[11]["truth_" + key] = str(-float(states[11]["truth_" + key]))
        self.write(target / "states.csv", states)
        self.assertEqual("passed_functional_contract", audit_case(target)["status"])

    def test_all_scenarios_have_actual_triggers_and_cold_start_gap(self):
        results = {name: audit_case(self.replay / name) for name in SCENARIOS}
        self.assertEqual(0, results["gnss_only_cold_start"]["pose_states"])
        self.assertIsNone(results["gnss_only_cold_start"]["synthetic_exact_pose_error_m"]["max"])
        self.assertEqual(22, results["indoor_marker_no_map_gnss"]["pose_states"])
        self.assertGreater(sum(r["nis_recomputed"] for r in results.values()), 0)

    def test_long_map_loss_requires_continuing_partial_commits_and_full_return(self):
        for name, end in MAP_LOSS_END.items():
            with self.subTest(name=name):
                result = audit_case(self.replay / name)
                self.assertEqual(end - 6, result["loss_recovery_measurements"][
                    "partial_commits_during_loss"])
                self.assertEqual(0, result["loss_recovery_measurements"][
                    "first_commit_after_return_ns"])
        for defect in ("false_freshness", "uncommitted_partial", "uncommitted_return"):
            with self.subTest(defect=defect):
                target = self.copied("map_loss_120s")
                if defect == "false_freshness":
                    states = rows(target / "states.csv")
                    states[200]["full_pose_fresh"] = "1"
                    self.write(target / "states.csv", states)
                else:
                    decisions = rows(target / "decisions.csv")
                    event = next(r for r in decisions if r["node"] ==
                                 ("200" if defect == "uncommitted_partial" else "246") and
                                 r["source"] == ("gnss-xy" if defect == "uncommitted_partial"
                                                 else "scan-to-map"))
                    event["updated"] = "0"
                    self.rewrite_ledger(target, decisions)
                with self.assertRaises(ValueError):
                    audit_case(target)

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

    def test_field_profile_requires_exact_source_cadence_and_commits(self):
        result = audit_case(self.replay / "field_sparse_map_gnss")
        self.assertEqual(5, result["sources"]["scan-to-map"]["committed"])
        self.assertEqual(11, result["sources"]["gnss-xy"]["committed"])
        self.assertIn(16, result["stale_full_pose_nodes"])
        target = self.copied("field_sparse_map_gnss")
        decisions = rows(target / "decisions.csv")
        event = next(r for r in decisions if r["expected"] == "accepted")
        event["updated"] = "0"
        self.write(target / "decisions.csv", decisions)
        with self.assertRaisesRegex(ValueError, "anchored source did not commit"):
            audit_case(target)

    def test_field_profile_bins_are_half_open_and_use_admission_not_commit(self):
        profile = field_availability_profile(
            [{"timestamp_ns": "100", "valid": "1"},
             {"timestamp_ns": "500000100", "valid": "0"}],
            [{"timestamp_ns": "100", "gate_outcome": "accepted", "adapter_reason": "ok"},
             {"timestamp_ns": "500000100", "gate_outcome": "waiting_for_stability",
              "adapter_reason": "ok"},
             {"timestamp_ns": "1000000100", "gate_outcome": "accepted",
              "adapter_reason": "invalid"}], 100)
        self.assertEqual({"node": "0", "map_available": "1", "gnss_available": "1"},
                         profile[0])
        self.assertEqual("0", profile[1]["map_available"])
        self.assertEqual("0", profile[1]["gnss_available"])
        self.assertEqual("0", profile[2]["gnss_available"])
        self.assertEqual(24, len(rows(FIELD_PROFILE)))

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

    def test_loss_duration_and_recovery_across_window_boundaries(self):
        for name, end in WINDOW_LOSS_END.items():
            with self.subTest(name=name):
                result = audit_case(self.replay / name)
                self.assertEqual([[6, end]], result["coverage"][
                    "no_absolute_observation_node_intervals_half_open"])
                self.assertAlmostEqual(.02 * (end - 6),
                                       result["synthetic_exact_pose_error_m"]["max"], places=8)
                measured = result["loss_recovery_measurements"]
                self.assertEqual((end - 6) * 500_000_000, measured["loss_duration_ns"])
                self.assertEqual(0, measured["first_commit_after_return_ns"])
                self.assertLess(measured["final_error_m"], measured["declared_local_offset_m"])
        short = audit_case(self.replay / "joint_loss_short")
        self.assertFalse(short["stale_full_pose_nodes"])
        self.assertTrue(audit_case(self.replay / "joint_loss_long")["stale_full_pose_nodes"])

    def test_window_loss_cannot_hide_false_freshness_or_source_leak(self):
        target = self.copied("joint_loss_long")
        states = rows(target / "states.csv")
        states[10]["full_pose_fresh"] = "1"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "loss-interval full-pose lease"):
            audit_case(target)
        states[10]["full_pose_fresh"] = "0"
        states[10]["absolute_update_translation_m"] = "0.01"
        self.write(target / "states.csv", states)
        with self.assertRaisesRegex(ValueError, "correction invented during joint loss"):
            audit_case(target)
        target = self.copied("joint_loss_equal")
        decisions = rows(target / "decisions.csv")
        event = dict(next(r for r in decisions if r["node"] == "12"))
        event["node"] = "10"
        decisions.append(event)
        self.rewrite_ledger(target, decisions)
        with self.assertRaisesRegex(ValueError, "declared scenario source schedule"):
            audit_case(target)

    def test_rejected_burst_matches_omitted_source_after_marginalization(self):
        self.assertEqual("passed_rejected_factor_noninterference",
                         audit_rejected_subset(self.replay)["status"])
        result = audit_case(self.replay / "gnss_outlier_burst")
        self.assertEqual(30, result["sources"]["gnss-xy"]["rejected"])
        self.assertEqual([[6, 36]], result["coverage"]["sources"]["gnss-xy"][
            "rejected_node_intervals_half_open"])
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        replay = Path(temporary.name)
        for name in BURST_SCENARIOS:
            shutil.copytree(self.replay / name, replay / name)
        path = replay / BURST_SCENARIOS[0] / "states.csv"
        states = rows(path)
        # Alter a symmetric off-diagonal after the burst; trace-only comparison
        # would miss retained covariance pollution.
        values = states[40]["alignment_covariance"].split(":")[1].split(";")
        values[1] = values[6] = str(float(values[1]) + 1e-5)
        states[40]["alignment_covariance"] = "6x6:" + ";".join(values)
        self.write(path, states)
        with self.assertRaisesRegex(ValueError, "subset estimate/covariance"):
            audit_rejected_subset(replay)

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
