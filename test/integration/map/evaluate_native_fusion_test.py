#!/usr/bin/env python3
"""Contract tests for real diagnostic-journal evaluation."""

import csv
import json
from pathlib import Path
import tempfile
import unittest

from evaluate_native_fusion import (
    GNSS_SOURCE, MAP_SOURCE, audit_replay, compare_replays, main, render,
    scenario_correspondence,
)
from prepare_native_lio import digest


class ReplayEvaluationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.report = {
            "schema": "openloc.native-map-diagnostic-report.v1",
            "global_owner": "DiagnosticGlobalAlignment",
            "diagnostic_only": True, "global_trusted": False,
            "outputs_sha256": {}, "gnss_contract": {"events": 2},
            "map_contract": {},
        }
        self.state = {"timestamp_ns": "100", "receive_ns": "110", "x": "1", "imu_ms": "1"}
        self.write("odometry.csv", [self.state])
        self.write("lidar.csv", [{"sequence": "1", "processing_ms": "10"}])
        self.write("health.csv", [{"timestamp_ns": "100", "receive_ns": "110",
                                  "health_trusted": "0", "assessment_covariance_valid": "0"}])
        self.write("global.csv", [{"diagnostic_only": "1", "global_trusted": "0",
                                  "target_graph_available": "0"}])
        self.gnss = [
            {"sequence": "1", "timestamp_ns": "100", "receive_ns": "110",
             "adapter_reason": "ok", "gate_outcome": "accepted", "gate_reason": "",
             "owner_status": "0", "owner_reason": "", "receiver_accuracy_available": "0",
             "heading_available": "0", "target_graph_available": "0"},
            {"sequence": "2", "timestamp_ns": "200", "receive_ns": "210",
             "adapter_reason": "ok", "gate_outcome": "rejected", "gate_reason": "float",
             "owner_status": "not_submitted", "owner_reason": "gate_not_accepted",
             "receiver_accuracy_available": "0", "heading_available": "0",
             "target_graph_available": "0"},
        ]
        self.write("gnss.csv", self.gnss)
        self.map = [{"sequence": "1", "timestamp_ns": "100", "receive_ns": "110",
                     "valid": "1", "reason": ""},
                    {"sequence": "2", "timestamp_ns": "200", "receive_ns": "210",
                     "valid": "0", "reason": "rank"}]
        self.write("map.csv", self.map)
        self.decisions = [
            {"source": GNSS_SOURCE, "sequence": "1", "timestamp_ns": "100", "receive_ns": "110",
             "decision_id": "1", "correction_id": "0", "status": "0", "message": "",
             "alignment_updated": "0", "target_graph_available": "0"},
            {"source": MAP_SOURCE, "sequence": "1", "timestamp_ns": "100", "receive_ns": "110",
             "decision_id": "2", "correction_id": "1", "status": "0", "message": "",
             "alignment_updated": "1", "target_graph_available": "0"},
        ]
        self.write("global_decisions.csv", self.decisions)
        self.admissions = [
            {"source": row["source"], "sequence": "1", "measurement_ns": "100",
             "source_receive_ns": "110", "enqueue_processing_ns": "110",
             "submission_processing_ns": "120", "holdback_ns": "20",
             "status": "0", "message": ""}
            for row in self.decisions
        ]
        self.write("global_admission.csv", self.admissions)

    def write(self, name, rows):
        with (self.root / name).open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        self.report["outputs_sha256"][name] = digest(self.root / name)

    def audit(self):
        return audit_replay(self.root, self.report)

    def test_closed_accounting_separates_events_and_actual_corrections(self):
        result = self.audit()
        self.assertEqual(4, result["checks"]["event_accounting"]["terminal"])
        self.assertEqual(1, result["committed_corrections"])
        self.assertEqual({"retained_without_correction": 1, "gate_not_accepted": 1},
                         result["sources"][GNSS_SOURCE]["outcomes"])
        self.assertEqual(10, result["queue_residence_ns"]["max"])
        self.assertEqual("not_run", result["checks"]["default_consumer_rejection"]["status"])
        self.assertIsNone(result["events"][0]["robust_weight"])
        self.assertIn("no-ground-truth", render(result))
        correspondence = result["scenario_correspondence"]
        self.assertEqual("partial", correspondence["cases"]["S12"])
        self.assertEqual("not_observed_in_this_arm", correspondence["cases"]["S03"])
        self.assertEqual(0, correspondence["evidence"]["gnss_commits"])

    def test_field_mapping_requires_actual_commits_and_quality_not_stability_wait(self):
        events = [
            {"source": MAP_SOURCE, "outcome": "committed", "reason": "",
             "measurement_ns": stamp}
            for stamp in (1, 4_000_000_001)
        ] + [
            {"source": GNSS_SOURCE, "outcome": "committed", "reason": "",
             "measurement_ns": 2_000_000_001},
            {"source": GNSS_SOURCE, "outcome": "gate_not_accepted",
             "reason": "fixed_solution_not_stable", "measurement_ns": 2},
        ]
        result = scenario_correspondence(events)
        self.assertEqual("partial", result["cases"]["S03"])
        self.assertEqual("partial", result["cases"]["S15"])
        self.assertEqual("not_observed_in_this_arm", result["cases"]["S05"])
        events[-1]["reason"] = "carrier_solution_not_fixed"
        self.assertEqual("partial", scenario_correspondence(events)["cases"]["S05"])
        events[2]["outcome"] = "retained_without_correction"
        result = scenario_correspondence(events)
        self.assertEqual("not_observed_in_this_arm", result["cases"]["S15"])
        self.assertEqual("not_observed_in_this_arm", result["cases"]["S03"])

    def test_original_event_identity_not_sequence_alone(self):
        self.gnss[1].update(sequence="1")
        self.write("gnss.csv", self.gnss)
        self.assertEqual(4, len(self.audit()["events"]))
        self.decisions[0]["timestamp_ns"] = "101"
        self.write("global_decisions.csv", self.decisions)
        with self.assertRaisesRegex(ValueError, "no submitted source"):
            self.audit()

    def test_missing_arrival_terminal_gap_and_duplicate_fail(self):
        self.write("gnss.csv", self.gnss[:1])
        with self.assertRaisesRegex(ValueError, "lost an arrival"):
            self.audit()
        self.write("gnss.csv", self.gnss)
        self.write("global_decisions.csv", self.decisions[1:])
        with self.assertRaisesRegex(ValueError, "journal gap"):
            self.audit()
        self.write("global_decisions.csv", self.decisions + [
            dict(self.decisions[-1], decision_id="3")])
        with self.assertRaisesRegex(ValueError, "duplicate terminal"):
            self.audit()

    def test_missing_terminal_is_explicit_pending_not_pass(self):
        self.write("global_decisions.csv", self.decisions[:1])
        result = self.audit()
        self.assertEqual("failed", result["checks"]["event_accounting"]["status"])
        self.assertEqual(1, result["checks"]["event_accounting"]["pending"])

    def test_timing_revision_trust_and_hash_fail_closed(self):
        self.write("global_admission.csv", [
            dict(self.admissions[0], submission_processing_ns="109"), self.admissions[1]])
        with self.assertRaisesRegex(ValueError, "predates source arrival"):
            self.audit()
        self.write("global_admission.csv", self.admissions)
        self.write("global_decisions.csv", [
            self.decisions[0], dict(self.decisions[1], correction_id="2")])
        with self.assertRaisesRegex(ValueError, "correction revision"):
            self.audit()
        self.write("global_decisions.csv", self.decisions)
        self.write("global.csv", [{"diagnostic_only": "1", "global_trusted": "1",
                                  "target_graph_available": "0"}])
        with self.assertRaisesRegex(ValueError, "gained qualification"):
            self.audit()
        (self.root / "global.csv").write_text("corrupted")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            self.audit()

    def test_determinism_ignores_only_declared_timing_fields(self):
        reference = self.root / "reference"
        reference.mkdir()
        for name in ("odometry.csv", "lidar.csv"):
            (reference / name).write_bytes((self.root / name).read_bytes())
        self.write("odometry.csv", [dict(self.state, imu_ms="20")])
        self.assertEqual("passed", compare_replays(
            self.root, reference, ("odometry.csv", "lidar.csv"))["odometry.csv"]["status"])
        self.write("odometry.csv", [dict(self.state, timestamp_ns="101")])
        with self.assertRaisesRegex(ValueError, "deterministic mismatch"):
            compare_replays(self.root, reference, ("odometry.csv",))

    def test_cli_writes_new_pinned_report_and_rejects_reuse(self):
        self.report.update(global_owner=None, gnss_contract=None, map_contract=None)
        # A local-only report has no absolute source journals.
        for name in ("global.csv", "gnss.csv", "map.csv",
                     "global_decisions.csv", "global_admission.csv"):
            del self.report["outputs_sha256"][name]
        report_path = self.root / "report.json"
        report_path.write_text(json.dumps(self.report))
        output = self.root / "evaluation"
        arguments = ["--report", str(report_path), "--replay", str(self.root),
                     "--output", str(output)]
        self.assertEqual(0, main(arguments))
        pins = json.loads((output / "artifact_hashes.json").read_text())
        for name, pin in pins.items():
            self.assertEqual(pin, digest(output / name))
        with self.assertRaises(FileExistsError):
            main(arguments)


if __name__ == "__main__":
    unittest.main()
