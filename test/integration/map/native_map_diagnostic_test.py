#!/usr/bin/env python3
"""Focused diagnostic map admission/output and raw-to-global regression."""

import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

from native_lio_replay_test import write_csv
from native_lio_report import (
    summarize_global_scheduler, summarize_host_timing, summarize_imu_trace,
)
from run_native_map_diagnostic import (
    causal_worker_summary, fusion_diagnostics,
    global_odom_fault_arguments, map_summary, prepare_bundle,
)
from prepare_native_lio import digest


class MapReportTest(unittest.TestCase):
    def test_global_odom_fault_arguments_are_explicit_and_bounded(self):
        drift = global_odom_fault_arguments(
            "both", ["1791882567558665752", "0.1", "-0.05"], None)
        self.assertEqual(drift["kind"], "drift")
        self.assertEqual(drift["offset"], [0.1, -0.05])
        self.assertEqual(drift["arguments"][:2],
                         ["--global-odom-drift", "1791882567558665752"])
        step = global_odom_fault_arguments(
            "gnss", None, ["1791882570558665752", "50", "-30"])
        self.assertEqual(step["kind"], "step")
        self.assertEqual(step["arguments"][:2],
                         ["--global-odom-step", "1791882570558665752"])
        self.assertIsNone(global_odom_fault_arguments("both", None, None))
        invalid_cases = (
            ("both", ["1", "0", "0"], ["1", "0", "0"], "mutually exclusive"),
            ("odom", ["1", "0", "0"], None, "global source arm"),
            ("gnss", ["0", "0", "0"], None, "positive time"),
            ("gnss", ["1", "nan", "0"], None, "finite offset"),
        )
        for arm, drift_args, step_args, message in invalid_cases:
            with self.subTest(message=message), self.assertRaisesRegex(
                    ValueError, message):
                global_odom_fault_arguments(arm, drift_args, step_args)

    def test_source_diagnostics_count_final_outcomes_not_holdback_entries(self):
        base = {
            "source": "map", "sequence": "7", "timestamp_ns": "100",
            "receive_ns": "110", "status": "11", "message": "pending",
            "alignment_updated": "0", "decision_id": "1", "correction_id": "0",
        }
        committed = dict(base, status="0", message="", alignment_updated="1",
                         decision_id="2", correction_id="1")
        retained = dict(base, source="gnss", sequence="8", status="0",
                        message="", decision_id="3", correction_id="1")
        states = [
            {"timestamp_ns": "1", "receive_ns": "10", "valid": "0",
             "integrity_reason": "WAITING_FOR_STATIONARY", "capability_reason": "NONE"},
            {"timestamp_ns": "90", "receive_ns": "100", "valid": "1",
             "integrity_reason": "NONE", "capability_reason": "CORRELATION_UNQUALIFIED"},
        ]
        outputs = [{"timestamp_ns": "100", "receive_ns": "110",
                    "alignment_initialized": "1", "full_pose_fresh": "1"}]
        result = fusion_diagnostics(
            states, outputs, [base, committed, retained],
            [{"valid": "1", "observed_rank": "6", "reason": ""}],
            None, {"causal_search": True})
        self.assertEqual(result["sources"]["map"]["journal_entries"], 2)
        self.assertEqual(result["sources"]["map"]["distinct_observations"], 1)
        self.assertEqual(result["sources"]["map"]["final_outcomes"], {"updated": 1})
        self.assertEqual(result["sources"]["map"]["corrections"][0]["correction_id"], "1")
        self.assertEqual(result["sources"]["gnss"]["final_outcomes"],
                         {"accepted_without_correction": 1})
        self.assertEqual(result["native_startup"]["receive_delay_from_first_input_ns"], 90)
        self.assertEqual(result["global_initialization"][
            "source_receive_delay_after_native_startup_ns"], 10)
        self.assertIn("MAP_CONDITIONAL_PRIORS", [r["code"] for r in result["findings"]])
        with self.assertRaisesRegex(ValueError, "rejected.*correction"):
            fusion_diagnostics([], [], [dict(committed, status="6")], [], None, None)

    def test_unavailable_and_unanchored_are_explicit_without_invented_times(self):
        result = fusion_diagnostics([], [], [], [], {
            "native_odometry_unavailable_events": 2,
            "unanchored_owner_decisions": 3,
        }, None)
        self.assertIsNone(result["native_startup"]["first_valid_measurement_ns"])
        self.assertIsNone(result["global_initialization"][
            "source_receive_delay_after_native_startup_ns"])
        self.assertFalse(result["global_initialization"]["established"])
        self.assertEqual({r["code"] for r in result["findings"]}, {
            "DIAGNOSTIC_BACKEND_ONLY", "NATIVE_ODOMETRY_UNAVAILABLE",
            "GNSS_BEFORE_NATIVE_ODOMETRY", "GNSS_POSITION_RETAINED_WITHOUT_ANCHOR",
            "GNSS_GLOBAL_INITIALIZATION_MISSING",
        })

    def test_causal_worker_snapshot_identity_and_age_fail_closed(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            path = Path(temporary) / "worker.csv"
            header = "sequence,reference_ns,processing_ns,status"
            events = [[1, 1_000_000_000, 1_000_000_000, "queued"],
                      [2, 1_100_000_000, 1_100_000_000, "busy"],
                      [1, 1_000_000_000, 1_200_000_000, "proposal_consumed"]]
            write_csv(path, header, events)
            summary = causal_worker_summary(path)
            self.assertEqual(summary["queries"], 1)
            self.assertEqual(summary["outcomes"]["busy"], 1)
            events[-1][2] = 1_600_000_000
            write_csv(path, header, events)
            with self.assertRaisesRegex(ValueError, "expired"):
                causal_worker_summary(path)
            events[-1][-1] = "stale_result_rejected"
            write_csv(path, header, events)
            self.assertEqual(causal_worker_summary(path)["queries"], 1)
            write_csv(path, header, events[:-1])
            with self.assertRaisesRegex(ValueError, "did not join"):
                causal_worker_summary(path)
            write_csv(path, header, [events[0], events[0]])
            with self.assertRaisesRegex(ValueError, "one snapshot"):
                causal_worker_summary(path)

    def test_rejections_are_not_functional_global_success(self):
        with self.assertRaisesRegex(ValueError, "committed global"):
            map_summary([{"valid": "1"}],
                        [{"alignment_initialized": "0"}], [])
        output = {"alignment_initialized": "0", "target_graph_available": "0",
                  "global_trusted": "0", "diagnostic_only": "1"}
        summary = map_summary([{"valid": "0", "reason": "causal_search_pending"}],
                              [output], [], allow_unanchored=True)
        self.assertFalse(summary["map_alignment_established"])
        self.assertIsNone(summary["first_global_timestamp_ns"])
        output["global_trusted"] = "1"
        with self.assertRaisesRegex(ValueError, "invariant"):
            map_summary([{"valid": "0"}], [output], [], allow_unanchored=True)

    def test_target_graph_or_trust_promotion_is_rejected(self):
        output = {"alignment_initialized": "1", "timestamp_ns": "1",
                  "target_graph_available": "1", "global_trusted": "0",
                  "diagnostic_only": "1"}
        with self.assertRaisesRegex(ValueError, "invariant"):
            map_summary([{"valid": "1"}], [output],
                        [{"alignment_updated": "1"}])

    def test_seed_and_selected_map_hashes_are_verified(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            (root / "global.pcd").write_text("selected map")
            (root / "frames.csv").write_text("selected frame inventory")
            metadata = {
                "schema": "wheelos.map.v1", "version": "fixture",
                "content_id": "fixture-hash",
                "global_cloud": {"path": "global.pcd",
                                 "sha256": digest(root / "global.pcd")},
                "frames_sha256": digest(root / "frames.csv"),
            }
            (root / "metadata.json").write_text(json.dumps(metadata))
            pin = digest(root / "metadata.json")
            result = prepare_bundle(root, pin, [0, 0, 0, 0, 0, 0, 1],
                                    "manual map-side hypothesis", root / "ok")
            self.assertFalse(result["target_graph_available"])
            self.assertFalse(result["gnss_consumed"])
            self.assertEqual(result["seed_applied_only_to"],
                             "first eligible scan-to-map request")
            policy = root / "recovery.csv"
            write_csv(policy, "name,value", [["field_resolution_m", 0.5]])
            policy_pin = digest(policy)
            conditional = prepare_bundle(
                root, pin, None, None, root / "causal_bundle",
                recovery_policy=policy, recovery_policy_sha256=policy_pin)
            self.assertTrue(conditional["causal_search"])
            self.assertNotIn("seed.csv", conditional["files"])
            self.assertEqual(conditional["files"]["recovery.csv"], policy_pin)
            with self.assertRaisesRegex(ValueError, "mutually exclusive"):
                prepare_bundle(root, pin, [0, 0, 0, 0, 0, 0, 1], "seed",
                               root / "conflicting", recovery_policy=policy,
                               recovery_policy_sha256=policy_pin)
            with self.assertRaisesRegex(ValueError, "policy hash mismatch"):
                prepare_bundle(root, pin, None, None, root / "wrong_policy",
                               recovery_policy=policy,
                               recovery_policy_sha256="0" * 64)
            self.assertFalse((root / "wrong_policy").exists())
            with self.assertRaisesRegex(ValueError, "required together"):
                prepare_bundle(root, pin, [0, 0, 0, 0, 0, 0, 1],
                               "manual seed", root / "missing_time_policy",
                               seed_reference_ns=1)
            (root / "global.pcd").write_text("corrupted")
            with self.assertRaisesRegex(ValueError, "hash mismatch"):
                prepare_bundle(root, pin, [0, 0, 0, 0, 0, 0, 1],
                               "manual seed", root / "rejected")
            self.assertFalse((root / "rejected").exists())


class NativeMapReplayTest(unittest.TestCase):
    binary = None

    def test_raw_native_lio_map_single_diagnostic_owner(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            config = json.loads(Path(__file__).with_name(
                "native_lio_diagnostic_config.json").read_text())
            write_csv(root / "config.csv", "name,value", config.items())
            write_csv(root / "extrinsic.csv", "tx,ty,tz", [[0, 0, 0]])
            write_csv(
                root / "imu.csv", "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                [[1_000_000_000 + i * 5_000_000,
                  1_000_100_000 + i * 5_000_000,
                  0, 0, 9.80665, 0, 0, 0] for i in range(801)],
            )
            points = []
            for a in range(1, 25):
                for b in range(1, 25):
                    x, y = a * 0.2, b * 0.2
                    points.extend([(x, y, 0), (x, 0, y), (0, x, y)])
            scans = []
            for index in range(12):
                end = 3_200_000_000 + index * 100_000_000
                name = f"scan{index}.csv"
                write_csv(root / name, "timestamp_ns,x,y,z",
                          [[end - 10_000_000, *p] for p in points])
                scans.append([end, end + 2_000_000, name])
            write_csv(root / "scans.csv", "end_ns,receive_ns,points", scans)
            bundle = root / "map"
            bundle.mkdir()
            write_csv(bundle / "identity.csv",
                      "map_id,map_version,map_content_id,calibration_id,"
                      "map_frame,odom_frame,base_frame",
                      [["synthetic", "v1", "synthetic-content", "synthetic",
                        "map", "odom", "imu"]])
            write_csv(bundle / "seed.csv", "x,y,z,qx,qy,qz,qw",
                      [[0, 0, 0, 0, 0, 0, 1]])
            write_csv(bundle / "window.csv", "radius_m,cell_size_m", [[15, 2]])
            map_points = points + [(x + 100, y + 100, z) for x, y, z in points]
            with (bundle / "global.pcd").open("w") as stream:
                stream.write(
                    "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    f"COUNT 1 1 1\nWIDTH {len(map_points)}\nHEIGHT 1\n"
                    f"POINTS {len(map_points)}\nDATA ascii\n")
                for point in map_points:
                    stream.write(" ".join(str(v) for v in point) + "\n")
            output = root / "output"
            completed = subprocess.run(
                [str(self.binary), str(root), str(output),
                 "--imu-trace",
                 "--diagnostic-map", str(bundle), "--map-input-sha256",
                 "".join(digest(bundle / name) for name in
                         ("global.pcd", "identity.csv", "seed.csv", "window.csv"))],
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            def read(name):
                with (output / name).open() as stream:
                    reader = csv.DictReader(stream)
                    result = list(reader)
                    self.assertTrue(all(None not in row and
                                        None not in row.values()
                                        for row in result), name)
                    return result
            metrics = map_summary(read("map.csv"), read("global.csv"),
                                  read("global_decisions.csv"))
            self.assertGreater(metrics["accepted_map_matches"], 0)
            self.assertGreater(metrics["global_correction_updates"], 0)
            measurements = read("graph_measurements.csv")
            self.assertTrue(measurements)
            self.assertEqual(len(measurements), sum(
                row["valid"] == "1" for row in read("map.csv")))
            self.assertTrue(all(row["kind"] in ("map", "projected_map")
                                for row in measurements))
            self.assertTrue(all(int(row["receive_ns"]) >= int(row["timestamp_ns"])
                                for row in measurements))
            self.assertTrue(all(row["ambiguous"] == "0" for row in measurements))
            self.assertTrue(all(int(row["map_window_points"]) <= len(points)
                                for row in read("map.csv")))
            baseline = subprocess.run(
                [str(self.binary), str(root), str(root / "local_only")],
                capture_output=True, text=True,
            )
            self.assertEqual(baseline.returncode, 0, baseline.stderr)
            with (root / "local_only/odometry.csv").open() as stream:
                baseline_rows = list(csv.DictReader(stream))
            def deterministic(states):
                return [{k: v for k, v in row.items() if k != "imu_ms"}
                        for row in states]
            self.assertEqual(deterministic(read("odometry.csv")),
                             deterministic(baseline_rows))
            heldout = root / "map_holdout"
            command = [str(self.binary), str(root), str(heldout),
                       "--diagnostic-map", str(bundle), "--map-input-sha256",
                       "".join(digest(bundle / name) for name in
                               ("global.pcd", "identity.csv", "seed.csv", "window.csv")),
                       "--global-map-holdout", "3500000000", "3800000000"]
            checked = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(0, checked.returncode, checked.stderr)
            with (heldout / "map.csv").open() as stream:
                excluded = [r for r in csv.DictReader(stream)
                            if r["reason"] == "EVALUATION_HOLDOUT"]
            self.assertEqual(3, len(excluded))
            self.assertTrue(all(r["valid"] == "0" and r["global_status"] == "not_submitted"
                                for r in excluded))
            with (heldout / "odometry.csv").open() as stream:
                self.assertEqual(deterministic(baseline_rows),
                                 deterministic(list(csv.DictReader(stream))))
            trace = summarize_imu_trace(read("imu_trace.csv"),
                                        read("local_corrections.csv"))
            self.assertGreater(trace["recorded_propagations"], 0)
            asynchronous = subprocess.run(
                [str(self.binary), str(root), str(root / "async"),
                 "--diagnostic-map", str(bundle), "--map-input-sha256",
                 "".join(digest(bundle / name) for name in
                         ("global.pcd", "identity.csv", "seed.csv", "window.csv")),
                 "--async-global", "--arrival-paced"],
                capture_output=True, text=True,
            )
            self.assertEqual(asynchronous.returncode, 0, asynchronous.stderr)
            with (root / "async/odometry.csv").open() as stream:
                self.assertEqual(deterministic(list(csv.DictReader(stream))),
                                 deterministic(baseline_rows))
            with (root / "async/global_decisions.csv").open() as stream:
                self.assertEqual(list(csv.DictReader(stream)),
                                 read("global_decisions.csv"))
            with (root / "async/global_scheduler.csv").open() as stream:
                scheduled = list(csv.DictReader(stream))
            self.assertTrue(scheduled)
            self.assertTrue(all(row["status"] == "completed" for row in scheduled))
            self.assertEqual([int(row["task_id"]) for row in scheduled],
                             list(range(1, len(scheduled) + 1)))
            self.assertLessEqual(max(int(row["admitted_tasks"]) for row in scheduled), 256)
            summary = summarize_global_scheduler(
                root / "async/global_scheduler.csv", len(baseline_rows),
                sum(r["valid"] == "1" for r in baseline_rows), len(scans))
            self.assertTrue(summary["single_global_worker"])
            timing = summarize_host_timing(root / "async/host_timing.csv")
            self.assertEqual(timing["emitted_rows"], len(baseline_rows))
            with (root / "async/host_timing.csv").open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), len(baseline_rows))
            corrections = [r for r in read("lidar.csv") if r["reason"] == "NONE"]
            self.assertTrue(corrections)
            # The state CSV is emitted before AddLidar; check only states
            # strictly after a successful correction.
            first = int(corrections[0]["reference_ns"])
            self.assertTrue(all(r["covariance_model_valid"] == "0"
                                for r in read("odometry.csv")
                                if int(r["timestamp_ns"]) > first))
            self.assertTrue(all(row["odometry_deskew"] == "present" and
                                row["shared_local_scans"] == "present"
                                for row in read("global_decisions.csv")))
            self.assertTrue(all(r["health_trusted"] == "0"
                                for r in read("health.csv")))
            causal_bundle = root / "causal_map"
            causal_bundle.mkdir()
            for name in ("global.pcd", "identity.csv", "window.csv"):
                shutil.copyfile(bundle / name, causal_bundle / name)
            policy = {}
            for prefix, coordinates in (
                    ("field_min_", [-1, -1, -1]),
                    ("field_max_", [6, 6, 6]),
                    ("observed_min_", [-1, -1, -1]),
                    ("observed_max_", [6, 6, 6]),
                    ("search_min_", [-0.1, -0.1, -0.1]),
                    ("search_max_", [0.1, 0.1, 0.1])):
                policy.update({prefix + axis: value
                               for axis, value in zip("xyz", coordinates)})
            policy.update({
                "yaw_min_rad": -0.01, "yaw_max_rad": 0.01,
                "tilt_qx": 0, "tilt_qy": 0, "tilt_qz": 0, "tilt_qw": 1,
                "field_resolution_m": 0.25,
                "distance_sigma_m": 0.3, "inlier_distance_m": 0.6,
            })
            write_csv(causal_bundle / "recovery.csv", "name,value", policy.items())
            causal_command = [
                str(self.binary), str(root), str(root / "causal"),
                "--async-global", "--arrival-paced",
                "--diagnostic-map", str(causal_bundle), "--map-input-sha256",
                "".join(digest(causal_bundle / name) for name in
                        ("global.pcd", "identity.csv", "recovery.csv", "window.csv")),
            ]
            causal = subprocess.run(causal_command, capture_output=True, text=True)
            self.assertEqual(causal.returncode, 0, causal.stderr)
            with (root / "causal/causal_search.csv").open() as stream:
                search_rows = list(csv.DictReader(stream))
            proposals = [r for r in search_rows
                         if r["status"] == "diagnostic_proposal"]
            self.assertEqual(len(proposals), 1)
            self.assertGreaterEqual(int(proposals[0]["stable_tracks"]), 1)
            self.assertTrue(all(r["diagnostic_only"] == "1" for r in search_rows))
            with (root / "causal/causal_candidates.csv").open() as stream:
                candidate_sequences = {r["sequence"] for r in csv.DictReader(stream)}
            for row in search_rows:
                if row["truncated"] == "1":
                    self.assertEqual(row["verified_modes"], "0")
                    self.assertNotIn(row["sequence"], candidate_sequences)
            worker = causal_worker_summary(root / "causal/causal_worker.csv")
            self.assertEqual(worker["outcomes"]["proposal_consumed"], 1)
            with (root / "causal/seed_application.csv").open() as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])
            with (root / "causal/odometry.csv").open() as stream:
                self.assertEqual(deterministic(list(csv.DictReader(stream))),
                                 deterministic(baseline_rows))
            with (root / "causal/global.csv").open() as stream:
                causal_global = list(csv.DictReader(stream))
            self.assertTrue(any(r["alignment_initialized"] == "1"
                                for r in causal_global))
            self.assertTrue(all(r["global_trusted"] == "0"
                                for r in causal_global))
            broad_policy = dict(policy)
            broad_policy.update({
                "yaw_min_rad": -math.pi, "yaw_max_rad": math.pi,
                "distance_sigma_m": 1.0, "inlier_distance_m": 1.5,
            })
            for axis in "xyz":
                broad_policy["search_min_" + axis] = -0.8
                broad_policy["search_max_" + axis] = 4.8
            write_csv(causal_bundle / "recovery.csv", "name,value",
                      broad_policy.items())
            broad_command = [
                str(self.binary), str(root), str(root / "causal_truncated"),
                "--diagnostic-map", str(causal_bundle), "--map-input-sha256",
                "".join(digest(causal_bundle / name) for name in
                        ("global.pcd", "identity.csv", "recovery.csv", "window.csv")),
            ]
            broad = subprocess.run(broad_command, capture_output=True, text=True)
            self.assertEqual(broad.returncode, 0, broad.stderr)
            with (root / "causal_truncated/causal_search.csv").open() as stream:
                truncated = [r for r in csv.DictReader(stream)
                             if r["truncated"] == "1"]
            self.assertTrue(truncated)
            with (root / "causal_truncated/causal_candidates.csv").open() as stream:
                candidate_sequences = {r["sequence"] for r in csv.DictReader(stream)}
            for row in truncated:
                self.assertEqual(row["verified_modes"], "0")
                self.assertNotIn(row["sequence"], candidate_sequences)
            write_csv(causal_bundle / "recovery.csv", "name,value", policy.items())
            gap_input = root / "gap_input"
            gap_input.mkdir()
            for name in ("config.csv", "extrinsic.csv"):
                shutil.copyfile(root / name, gap_input / name)
            write_csv(
                gap_input / "imu.csv",
                "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                [[1_000_000_000 + i * 5_000_000,
                  1_000_100_000 + i * 5_000_000,
                  0, 0, 9.80665, 0, 0, 0] for i in range(1801)])
            gap_scans = []
            for index in range(24):
                start = 3_200_000_000 if index < 12 else 7_200_000_000
                end = start + (index % 12) * 100_000_000
                name = f"scan{index}.csv"
                write_csv(gap_input / name, "timestamp_ns,x,y,z",
                          [[end - 10_000_000, *p] for p in points])
                gap_scans.append([end, end + 2_000_000, name])
            write_csv(gap_input / "scans.csv",
                      "end_ns,receive_ns,points", gap_scans)
            gap_local = root / "gap_local"
            gap_baseline = subprocess.run(
                [str(self.binary), str(gap_input), str(gap_local)],
                capture_output=True, text=True)
            self.assertEqual(gap_baseline.returncode, 0, gap_baseline.stderr)
            with (gap_local / "odometry.csv").open() as stream:
                gap_baseline_rows = list(csv.DictReader(stream))
            for name, options in (("gap_sync", []),
                                  ("gap_async", ["--async-global",
                                                 "--arrival-paced"])):
                gap_output = root / name
                recovery = subprocess.run(
                    [str(self.binary), str(gap_input), str(gap_output),
                     "--diagnostic-map", str(causal_bundle),
                     "--map-input-sha256", causal_command[-1],
                     "--global-holdback-ns", "100000000", *options],
                    capture_output=True, text=True)
                self.assertEqual(recovery.returncode, 0, recovery.stderr)
                def gap_read(filename):
                    with (gap_output / filename).open() as stream:
                        return list(csv.DictReader(stream))
                transitions = gap_read("map_tracking.csv")
                lost = [r for r in transitions
                        if r["event"] == "full_pose_lease_expired"]
                self.assertEqual(len(lost), 1, transitions)
                self.assertEqual(lost[0]["generation"], "1")
                self.assertGreater(int(lost[0]["timestamp_ns"]) -
                                   int(lost[0]["last_full_pose_ns"]),
                                   2_000_000_000)
                committed = [r for r in transitions
                             if r["event"] == "alignment_committed"]
                self.assertEqual([r["generation"] for r in committed], ["0", "1"])
                self.assertEqual(transitions[-1]["state"], "tracking")
                self.assertTrue(all(r["diagnostic_only"] == "1"
                                    for r in transitions))
                search = gap_read("causal_search.csv")
                self.assertEqual({r["generation"] for r in search}, {"0", "1"})
                self.assertTrue(all(r["attempt"] == "1" for r in search))
                resumed = [r for r in gap_read("global_decisions.csv")
                           if int(r["timestamp_ns"]) >= 7_200_000_000]
                self.assertTrue(resumed)
                self.assertEqual(resumed[0]["alignment_updated"], "0")
                self.assertIn("distinct consistent frames",
                              resumed[0]["message"])
                self.assertTrue(any(r["alignment_updated"] == "1"
                                    for r in resumed))
                between = [r for r in gap_read("global.csv")
                           if 6_500_000_000 <= int(r["timestamp_ns"]) <
                           7_200_000_000]
                self.assertTrue(between)
                self.assertTrue(all(r["full_pose_fresh"] == "0"
                                    for r in between))
                self.assertTrue(all(r["global_trusted"] == "0"
                                    for r in gap_read("global.csv")))
                recovering_health = [
                    r for r in gap_read("health.csv")
                    if 7_200_000_000 <= int(r["timestamp_ns"]) < 8_300_000_000]
                self.assertTrue(any(r["diagnostic_global_initialized"] == "0"
                                    for r in recovering_health))
                self.assertTrue(all(r["health_trusted"] == "0"
                                    for r in recovering_health))
                self.assertEqual(deterministic(gap_read("odometry.csv")),
                                 deterministic(gap_baseline_rows))
                if options:
                    worker = causal_worker_summary(gap_output / "causal_worker.csv")
                    self.assertEqual(worker["outcomes"]["proposal_consumed"], 2)
            shutil.copyfile(bundle / "seed.csv", causal_bundle / "seed.csv")
            causal_command[2] = str(root / "causal_seed_conflict")
            conflict = subprocess.run(causal_command, capture_output=True, text=True)
            self.assertNotEqual(conflict.returncode, 0)
            self.assertIn("cannot consume an offline seed", conflict.stderr)
            (causal_bundle / "seed.csv").unlink()
            for axis in "xyz":
                policy["search_min_" + axis] = 4.5
                policy["search_max_" + axis] = 5.5
            write_csv(causal_bundle / "recovery.csv", "name,value", policy.items())
            causal_command[2] = str(root / "causal_unanchored")
            causal_command[-1] = "".join(digest(causal_bundle / name) for name in
                ("global.pcd", "identity.csv", "recovery.csv", "window.csv"))
            unanchored = subprocess.run(causal_command, capture_output=True, text=True)
            self.assertEqual(unanchored.returncode, 0, unanchored.stderr)
            with (root / "causal_unanchored/global.csv").open() as stream:
                self.assertTrue(all(r["alignment_initialized"] == "0"
                                    for r in csv.DictReader(stream)))
            with (root / "causal_unanchored/causal_search.csv").open() as stream:
                failures = list(csv.DictReader(stream))
            self.assertTrue(failures)
            self.assertFalse(any(r["status"] == "diagnostic_proposal" for r in failures))
            seed_rows = read("seed_application.csv")
            self.assertEqual(len(seed_rows), 1)
            self.assertGreater(int(seed_rows[0]["reference_ns"]), 3_000_000_000)
            write_csv(bundle / "seed_time.csv", "reference_ns,maximum_offset_ns",
                      [[11_000_000_000, 100_000_000]])
            stale = subprocess.run(
                [str(self.binary), str(root), str(root / "stale_seed"),
                 "--diagnostic-map", str(bundle), "--map-input-sha256",
                 "".join(digest(bundle / name) for name in
                         ("global.pcd", "identity.csv", "seed.csv", "window.csv",
                          "seed_time.csv"))],
                capture_output=True, text=True,
            )
            self.assertNotEqual(stale.returncode, 0)
            self.assertIn("seed reference outside explicit time tolerance",
                          stale.stderr)
            with (root / "stale_seed/seed_application.csv").open() as stream:
                rejection = next(csv.DictReader(stream))
            self.assertEqual(rejection["reason"], "seed_reference_outside_tolerance")
            self.assertEqual(rejection["local_initialization_used"], "0")
            (bundle / "seed_time.csv").unlink()
            missing_pin = subprocess.run(
                [str(self.binary), str(root), str(root / "missing_pin"),
                 "--diagnostic-map", str(bundle)],
                capture_output=True, text=True,
            )
            self.assertNotEqual(missing_pin.returncode, 0)
            self.assertIn("explicit input hashes", missing_pin.stderr)
            pinned = "".join(digest(bundle / name) for name in
                             ("global.pcd", "identity.csv", "seed.csv",
                              "window.csv"))
            with (bundle / "seed.csv").open("a") as stream:
                stream.write("\n")
            corrupt = subprocess.run(
                [str(self.binary), str(root), str(root / "corrupt"),
                 "--diagnostic-map", str(bundle), "--map-input-sha256", pinned],
                capture_output=True, text=True,
            )
            self.assertNotEqual(corrupt.returncode, 0)
            self.assertIn("map input hash mismatch: seed.csv", corrupt.stderr)
            write_csv(bundle / "seed.csv", "x,y,z,qx,qy,qz,qw",
                      [[1000, 1000, 1000, 0, 0, 0, 1]])
            wrong_pose = subprocess.run(
                [str(self.binary), str(root), str(root / "wrong_pose"),
                 "--diagnostic-map", str(bundle), "--map-input-sha256",
                 "".join(digest(bundle / name) for name in
                         ("global.pcd", "identity.csv", "seed.csv", "window.csv"))],
                capture_output=True, text=True,
            )
            self.assertNotEqual(wrong_pose.returncode, 0)
            self.assertIn("alignment not established", wrong_pose.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: native_map_diagnostic_test.py NATIVE_LIO_BINARY")
    NativeMapReplayTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
