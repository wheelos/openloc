#!/usr/bin/env python3
"""Raw adapter and native LIO runner contracts; no dataset qualification."""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from prepare_native_lio import imu_values, scan_decision, scan_values
from run_native_map_diagnostic import diagnostic_mean_policy
from native_lio_report import (
    imu_trace_svg, initialization_svg, runtime_identity, summarize, summarize_capability,
    summarize_imu_trace,
    summarize_global_scheduler, summarize_host_timing,
    summarize_local_lidar_tasks,
    summarize_lidar_decode,
    summarize_timer_output,
    summarize_initialization, summarize_local_corrections, trajectory_svg,
)


def header(stamp):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=stamp // 1_000_000_000,
                              nanosec=stamp % 1_000_000_000)
    )


def write_csv(path, fields, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields.split(","))
        writer.writerows(rows)


class RawContractTest(unittest.TestCase):
    def test_no_return_filter_precedes_stride_and_keeps_raw_end(self):
        message = SimpleNamespace(
            header=header(1_000_000_000), timebase=1_000_000_000, point_num=6,
            points=[SimpleNamespace(offset_time=i * 10, x=x, y=0, z=0)
                    for i, x in enumerate([0, 1, 0, 2, 3, 0])],
        )
        end, points = scan_values(message, 2)
        self.assertEqual(end, 1_000_000_050)
        self.assertEqual(points, [[1_000_000_010, 1.0, 0.0, 0.0],
                                  [1_000_000_040, 3.0, 0.0, 0.0]])
        self.assertEqual(scan_decision(message, 1_100_000_000, 7, end, points),
                         [7, end, 1_100_000_000, 6, 3, 2, "selected"])
        for point in message.points:
            point.x = 0
        self.assertEqual(scan_values(message, 2), (end, []))
        self.assertEqual(scan_decision(message, 1_100_000_000, 8, end, []),
                         [8, end, 1_100_000_000, 6, 6, 0, "no_valid_returns"])

    def test_imu_converts_g_and_ignores_orientation(self):
        imu = SimpleNamespace(
            header=header(1_000_000_000),
            linear_acceleration=SimpleNamespace(x=0, y=0, z=1),
            angular_velocity=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        )
        self.assertEqual(
            imu_values(imu, 1_000_100_000, 9.80665),
            [1_000_000_000, 1_000_100_000, 0, 0, 9.80665, 0.1, 0.2, 0.3],
        )

    def test_livox_uses_timebase_offsets_not_header_or_external_deskew(self):
        message = SimpleNamespace(
            header=header(1_000_000_050), timebase=1_000_000_000, point_num=5,
            points=[SimpleNamespace(offset_time=i * 10, x=i, y=0, z=1)
                    for i in range(5)],
        )
        end, points = scan_values(message, 2)
        self.assertEqual(end, 1_000_000_040)
        self.assertEqual([point[0] for point in points],
                         [1_000_000_000, 1_000_000_030])
        message.points[2].offset_time = 0
        with self.assertRaisesRegex(ValueError, "regressed"):
            scan_values(message, 2)


class NativeReportTest(unittest.TestCase):
    def test_mean_policy_is_explicit_and_never_promotes_trust(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            self.assertEqual(diagnostic_mean_policy(root), "covariance_intersection")
            write_csv(root / "global_mean_policy.csv",
                      "mode,covariance,trust_ceiling",
                      [["fixed_half_residual", "unknown_correlation_bound",
                        "diagnostic"]])
            self.assertEqual(diagnostic_mean_policy(root), "fixed_half_residual")
            write_csv(root / "global_mean_policy.csv",
                      "mode,covariance,trust_ceiling",
                      [["fixed_half_residual", "qualified", "trusted"]])
            with self.assertRaisesRegex(ValueError, "invalid diagnostic mean policy"):
                diagnostic_mean_policy(root)
            (root / "global_mean_policy.csv").unlink()
            write_csv(root / "global_decisions.csv", "covariance_model",
                      [["diagnostic_fixed_gain_full_pose_bound"]])
            with self.assertRaisesRegex(ValueError, "disagrees with innovations"):
                diagnostic_mean_policy(root)

    def test_timer_distinguishes_held_pose_from_new_measurement_and_stale(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            journal = Path(temporary) / "timer.csv"
            fields = (
                "publication_sequence,target_steady_ns,emitted_steady_ns,"
                "skipped_periods,usable,new_measurement,epoch_changed,"
                "measurement_ns,receive_ns,source_sequence,session,generation,"
                "px,py,pz,qx,qy,qz,qw,vx,vy,vz,measurement_age_ns,"
                "maximum_age_ns,diagnostic_only,trusted")
            common = [1_000_000_000, 1_000_100_000, 1, "session", 0]
            pose = [1, 2, 3, 0, 0, 0, 1, 0, 0, 0]
            events = [
                [1, 10_000_000, 11_000_000, 0, 1, 1, 0, *common, *pose,
                 1_000_000, 200_000_000, 1, 0],
                [2, 20_000_000, 21_000_000, 0, 1, 0, 0, *common, *pose,
                 11_000_000, 200_000_000, 1, 0],
                [3, 30_000_000, 31_000_000, 0, 0, 0, 0, *common, *([""] * 10),
                 201_000_000, 200_000_000, 1, 0],
            ]
            write_csv(journal, fields, events)
            summary = summarize_timer_output(journal)
            self.assertEqual(summary["new_measurements"], 1)
            self.assertEqual(summary["held_measurements"], 1)
            self.assertEqual(summary["unavailable_publications"], 1)
            events[1][5] = 1
            write_csv(journal, fields, events)
            with self.assertRaisesRegex(ValueError, "measurement identity"):
                summarize_timer_output(journal)
            events[1][5] = 0
            events[2][12] = 1
            write_csv(journal, fields, events)
            with self.assertRaisesRegex(ValueError, "unavailable pose"):
                summarize_timer_output(journal)

    def test_runtime_identity_rejects_foreign_bazel_and_missing_libraries(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            output = root / "bazel-out/k8-opt/bin"
            output.mkdir(parents=True)
            binary = output / "native_lio_replay"
            binary.write_bytes(b"\x7fELFfixture")
            library = output / "libestimator.so"
            library.write_bytes(b"selected runtime")
            loaded = SimpleNamespace(stdout=f"libestimator.so => {library.absolute()} (0x1)\n")
            with patch("native_lio_report.subprocess.run", return_value=loaded):
                first = runtime_identity(binary)
                library.write_bytes(b"changed runtime")
                second = runtime_identity(binary)
                self.assertEqual(first["binary_sha256"], second["binary_sha256"])
                self.assertNotEqual(first["shared_libraries"], second["shared_libraries"])
                foreign = root / "other/bazel-out/k8-opt/bin/libestimator.so"
                foreign.parent.mkdir(parents=True)
                foreign.write_bytes(b"foreign runtime")
                loaded.stdout = f"libestimator.so => {foreign.absolute()} (0x1)\n"
                with self.assertRaisesRegex(ValueError, "foreign Bazel"):
                    runtime_identity(binary)
                loaded.stdout = "libestimator.so => not found\n"
                with self.assertRaisesRegex(ValueError, "not found"):
                    runtime_identity(binary)

    def test_delayed_local_task_journal_retains_rejections_and_tail(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            journal = Path(temporary) / "local_lidar_tasks.csv"
            fields = "sequence,reference_ns,processing_ns,due_ns,status,reason,detail"
            events = [
                [1, 1_000_000_000, 1_000_000_000, 1_050_000_000,
                 "queued", "HISTORY_UNAVAILABLE", "queued"],
                [1, 1_000_000_000, 1_050_000_000, 1_050_000_000,
                 "committed", "NONE", ""],
                [2, 1_100_000_000, 1_100_000_000, 1_150_000_000,
                 "queued", "HISTORY_UNAVAILABLE", "queued"],
                [2, 1_100_000_000, 1_150_000_000, 1_150_000_000,
                 "deadline_missed", "LIDAR_STALE", "late"],
                [2, 1_100_000_000, 1_155_000_000, 1_150_000_000,
                 "discarded_after_deadline", "NONE", ""],
                [3, 1_200_000_000, 1_200_000_000, 1_250_000_000,
                 "rejected_busy", "LIDAR_STALE", "busy"],
                [4, 1_300_000_000, 1_300_000_000, 1_350_000_000,
                 "queued", "HISTORY_UNAVAILABLE", "queued"],
                [4, 1_300_000_000, 1_310_000_000, 1_350_000_000,
                 "end_of_input_uncommitted", "NONE", ""],
            ]
            write_csv(journal, fields, events)
            result = summarize_local_lidar_tasks(journal, 4)
            self.assertEqual(result["accepted_corrections"], 1)
            self.assertEqual(result["terminal_outcomes"]["deadline_missed"], 1)
            self.assertFalse(result["published_history_rewritten"])
            write_csv(journal, fields, events[:-1])
            with self.assertRaisesRegex(ValueError, "lost a scan"):
                summarize_local_lidar_tasks(journal, 4)
            events[1][2] = 1_020_000_000
            write_csv(journal, fields, events)
            with self.assertRaisesRegex(ValueError, "invalid delayed"):
                summarize_local_lidar_tasks(journal, 4)

    def test_wall_deadline_and_scheduler_evidence_fail_closed(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            timing = root / "host_timing.csv"
            write_csv(
                timing,
                "timestamp_ns,receive_ns,valid,target_steady_ns,emitted_steady_ns,deadline_ns",
                [[1_000_000_000, 1_000_000_000, 1, 2_000_000_000, 2_000_000_001, 10_000_000],
                 [1_005_000_000, 1_005_000_000, 1, 2_005_000_000, 2_020_000_000, 10_000_000],
                 [1_025_000_000, 1_025_000_000, 1, 2_025_000_000, 2_025_000_000, 10_000_000],
                 [1_045_000_000, 1_045_000_000, 1, 2_045_000_000, 2_045_000_000, 10_000_000]],
            )
            summary = summarize_host_timing(timing)
            self.assertEqual(summary["initialized_deadline_misses"], 1)
            self.assertEqual(summary["initialized_wall_gaps_over_10ms"], 2)
            self.assertEqual(summary["initialized_receive_gaps_over_10ms"], 2)
            self.assertEqual(summary["initialized_wall_gaps_with_receive_gap_over_10ms"], 1)
            self.assertEqual(summary["initialized_wall_gaps_with_receive_gap_at_most_10ms"], 1)
            self.assertFalse(summary["paced_replay_10ms_check_passed"])
            self.assertFalse(summary["apollo_publication_qualified"])
            journal = root / "global_scheduler.csv"
            fields = ("task_id,kind,payload_bytes,admitted_tasks,admitted_bytes,"
                      "enqueue_steady_ns,start_steady_ns,finish_steady_ns,status")
            scheduled = [[i, kind, 1, 1, 1, 10 * i, 10 * i + 1,
                          10 * i + 2, "completed"]
                         for i, kind in enumerate(
                             ("odometry", "gnss", "drain", "scan", "finish"), 1)]
            write_csv(journal, fields, scheduled)
            result = summarize_global_scheduler(journal, 1, 1, 1)
            self.assertEqual(result["completed_tasks"], 5)
            self.assertTrue(result["single_global_worker"])
            write_csv(journal, fields, scheduled[:-1])
            with self.assertRaisesRegex(ValueError, "lost or duplicated"):
                summarize_global_scheduler(journal, 1, 1, 1)
            scheduled[-1][-1] = "failed"
            write_csv(journal, fields, scheduled)
            with self.assertRaisesRegex(ValueError, "invariant"):
                summarize_global_scheduler(journal, 1, 1, 1)
            self.assertFalse(summarize_host_timing(root / "missing")["journal_present"])
            self.assertFalse(summarize_global_scheduler(root / "missing", 1, 1, 1)["journal_present"])

    def test_imu_and_lidar_state_accounting_detects_hidden_changes(self):
        records = []
        for sequence, time, p0, p1, v0, v1 in (
                (2, 1.1, 0, 0.01, 0, 0.2),
                (3, 1.2, 0.04, 0.074, 0.24, 0.44)):
            row = {"sequence": str(sequence), "time_s": str(time), "dt_s": "0.1"}
            for prefix in ("position_before_", "position_after_", "velocity_before_",
                           "velocity_after_", "acceleration_", "gyro_bias_", "accel_bias_"):
                for axis in "xyz":
                    row[prefix + axis] = "0"
            for prefix, value in (
                    ("position_before_", p0), ("position_after_", p1),
                    ("velocity_before_", v0), ("velocity_after_", v1),
                    ("acceleration_", 2)):
                row[prefix + "x"] = str(value)
            records.append(row)
        corrections = [{"reference_ns": "1100000000",
                        **{"position_d" + a: str(0.03 if a == "x" else 0)
                           for a in "xyz"},
                        **{"velocity_d" + a: str(0.04 if a == "x" else 0)
                           for a in "xyz"}}]
        summary = summarize_imu_trace(records, corrections)
        self.assertEqual(summary["recorded_propagations"], 2)
        self.assertAlmostEqual(summary["imu_velocity_impulse_mps"][0], 0.4)
        self.assertEqual(summary["lio_velocity_impulse_mps"], [0.04, 0.0, 0.0])
        self.assertLess(max(map(abs, summary["position_accounting_residual_m"])), 1e-12)
        self.assertIn("Internal linear acceleration", imu_trace_svg(records))
        records[-1]["position_after_x"] = "0.084"
        with self.assertRaisesRegex(ValueError, "propagation equations"):
            summarize_imu_trace(records, corrections)
        records[-1]["position_before_x"] = "0.05"
        with self.assertRaisesRegex(ValueError, "unexplained state change"):
            summarize_imu_trace(records, corrections)
        records[-1]["time_s"] = records[0]["time_s"]
        with self.assertRaisesRegex(ValueError, "noncausal"):
            summarize_imu_trace(records, corrections)
        records[-1]["gyro_bias_z"] = "nan"
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            summarize_imu_trace(records, corrections)
        self.assertEqual(summarize_imu_trace([], [])["recorded_propagations"], 0)
        self.assertIn("disabled", imu_trace_svg([]))

    def test_rejected_startup_solutions_keep_numerical_evidence(self):
        records = [
            {"linear_system_evaluated": "0", "solution_available": "0",
             "reason": "HISTORY_UNAVAILABLE", "reference_ns": "1000000000"},
            {"linear_system_evaluated": "1", "solution_available": "1",
             "reason": "INNOVATION_REJECTED", "reference_ns": "3000000000",
             "minimum_scaled_singular_value": "0.002",
             "gravity_error_mps2": "0.8", "accel_bias_mps2": "0.14",
             "gyro_bias_radps": "0.02", "normalized_rms": "0.01",
             "angular_rms_radps": "0.001"},
        ]
        result = summarize_initialization(records)
        self.assertEqual(result["solutions_available"], 1)
        self.assertEqual(result["solution_decisions"],
                         {"INNOVATION_REJECTED": 1})
        self.assertEqual(result["ranges"]["gravity_error_mps2"]["max"], 0.8)
        self.assertFalse(result["uncertainty_qualified"])
        self.assertIn("polyline", initialization_svg(records))
        self.assertIsNone(summarize_initialization([])["ranges"]["normalized_rms"])
        records[1]["gravity_error_mps2"] = "nan"
        with self.assertRaisesRegex(ValueError, "nonfinite startup diagnostic"):
            summarize_initialization(records)

    def test_no_initialization_is_not_reported_as_qualified_covariance(self):
        state = {
            "timestamp_ns": "1000000000", "valid": "0",
            "covariance_model_valid": "1", "motion_available": "0",
            "imu_ms": "0.1",
        }
        decisions = [{"bootstrap_stage": "1",
                      "reason": "DEGENERATE", "processing_ms": "200"}]
        result = summarize([state], decisions)
        self.assertEqual(result["valid_states"], 0)
        self.assertEqual(result["covariance_model_valid_states"], 0)
        self.assertIsNone(result["initialization_delay_s"])
        self.assertIsNone(result["runtime_scan_latency"])
        self.assertIn("No initialized", trajectory_svg([state]))

    def test_scan_latency_is_separate_from_imu_rate_and_startup_success(self):
        states = [{
            "timestamp_ns": str(1000000000 + i * 5000000), "valid": "1",
            "covariance_model_valid": "0", "motion_available": "0",
            "imu_ms": "0.5",
        } for i in range(3)]
        decisions = [
            {"bootstrap_stage": "1", "reason": "NONE", "processing_ms": "400"},
            {"bootstrap_stage": "0", "reason": "NONE", "processing_ms": "300"},
        ]
        result = summarize(states, decisions)
        self.assertEqual(result["max_valid_measurement_gap_ms"], 5)
        self.assertEqual(result["runtime_scan_decisions"], {"NONE": 1})
        self.assertEqual(result["runtime_scan_latency"]["max_ms"], 300)
        self.assertEqual(result["valid_imu_call_latency"]["max_ms"], 0.5)
        with self.assertRaisesRegex(ValueError, "must increase"):
            summarize(states[::-1], decisions)


class NativeReplayTest(unittest.TestCase):
    binary = None

    def test_structured_closure_cannot_bypass_native_or_source_policy(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            common = ["--offline-structured-closure", "selected.csv"]
            declared = [
                "--native-joint-structured",
                "--native-joint-audit-period-s", "0.005",
                "--diagnostic-map", "map", "--map-input-sha256", "pin",
                "--diagnostic-gnss", "gnss", "--gnss-input-sha256", "pin",
            ]
            for flags, message in (
                (common, "requires structured native audit"),
                (common + declared[:-4], "requires structured native audit"),
                (common + declared + ["--diagnostic-fixed-gain"],
                 "synchronous unmodified native replay"),
                (common + declared + ["--diagnostic-lidar-tracking-gain"],
                 "synchronous unmodified native replay"),
                (common + declared + ["--arrival-paced"],
                 "synchronous unmodified native replay"),
                (common + common, "unknown or duplicate"),
            ):
                with self.subTest(flags=flags):
                    completed = subprocess.run(
                        [str(self.binary), str(root), str(root / "output"),
                         *flags], capture_output=True, text=True)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertFalse((root / "output").exists())

    def test_fixed_gain_requires_explicit_owner_and_unique_option(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for flags, message in (
                (["--diagnostic-fixed-gain"], "requires a global owner"),
                (["--diagnostic-fixed-gain", "--diagnostic-fixed-gain"],
                 "unknown or duplicate"),
            ):
                with self.subTest(flags=flags):
                    completed = subprocess.run(
                        [str(self.binary), str(root), str(root / "output"),
                         *flags], capture_output=True, text=True)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertFalse((root / "output").exists())

    def test_local_profiling_rejects_duplicate_option(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for flags, message in (
                (["--profile-local-timing"], "unknown or duplicate"),
                (["--prefetch-lidar"], "requires synchronous decode and LIO"),
                (["--async-local-lio-ms", "5"],
                 "requires synchronous decode and LIO"),
            ):
                with self.subTest(flags=flags):
                    completed = subprocess.run(
                        [str(self.binary), str(root), str(root / "output"),
                         "--profile-local-timing", *flags],
                        capture_output=True, text=True)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertFalse((root / "output").exists())

    def test_map_profiling_requires_map_before_creating_output(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for flags, message in (
                (["--profile-map-timing"], "map timing profiling requires map"),
                (["--profile-map-timing", "--profile-map-timing"],
                 "unknown or duplicate"),
            ):
                with self.subTest(flags=flags):
                    completed = subprocess.run(
                        [str(self.binary), str(root), str(root / "output"),
                         *flags], capture_output=True, text=True)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertFalse((root / "output").exists())

    def test_native_joint_limits_require_audit_and_valid_unique_counts(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for name in ("--native-joint-max-dimension",
                         "--native-joint-max-memory-rows",
                         "--native-joint-max-snapshots"):
                cases = [
                    ([name, "64"], "limits require native joint audit"),
                    (["--native-joint-audit-period-s", "0.005", name, "0"],
                     "positive"),
                    (["--native-joint-audit-period-s", "0.005", name,
                      "4294967296"], "positive"),
                    (["--native-joint-audit-period-s", "0.005", name, "64",
                      name, "64"], "unknown or duplicate"),
                ]
                for flags, message in cases:
                    with self.subTest(flags=flags):
                        completed = subprocess.run(
                            [str(self.binary), str(root), str(root / "output"),
                             *flags], capture_output=True, text=True,
                        )
                        self.assertNotEqual(completed.returncode, 0)
                        self.assertIn(message, completed.stderr)
                        self.assertFalse((root / "output").exists())

    def test_native_joint_audit_requires_deterministic_local_only_input(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for flags in (["--arrival-paced"], ["--diagnostic-map", "map",
                                               "--map-input-sha256", "pin"]):
                completed = subprocess.run(
                    [str(self.binary), str(root), str(root / "output"),
                     "--native-joint-audit-period-s", "0.005", *flags],
                    capture_output=True, text=True,
                )
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn("requires deterministic local-only replay",
                              completed.stderr)
                self.assertFalse((root / "output").exists())

    def test_native_query_probe_requires_audit_and_unique_flag(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            for flags, message in (
                (["--native-joint-query-probe"], "requires native joint audit"),
                (["--native-joint-query-probe", "--native-joint-query-probe"],
                 "unknown or duplicate"),
                (["--native-joint-structured"], "requires native joint audit"),
                (["--native-joint-structured", "--native-joint-structured"],
                 "unknown or duplicate"),
                (["--native-joint-structured", "--native-joint-query-probe",
                  "--native-joint-audit-period-s", "0.005"],
                 "cannot export a dense query basis"),
            ):
                with self.subTest(flags=flags):
                    completed = subprocess.run(
                        [str(self.binary), str(root), str(root / "output"),
                         *flags], capture_output=True, text=True)
                    self.assertNotEqual(completed.returncode, 0)
                    self.assertIn(message, completed.stderr)
                    self.assertFalse((root / "output").exists())

    def test_native_query_probe_does_not_bypass_dense_model_refusal(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            config = json.loads(Path(__file__).with_name(
                "native_lio_diagnostic_config.json").read_text())
            write_csv(root / "config.csv", "name,value", config.items())
            write_csv(root / "extrinsic.csv", "tx,ty,tz", [[0, 0, 0]])
            write_csv(
                root / "imu.csv", "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                [[1_000_000_000 + i * 5_000_000,
                  1_000_100_000 + i * 5_000_000, 0, 0, 9.80665, 0, 0, 0]
                 for i in range(420)])
            write_csv(root / "scan.csv", "timestamp_ns,x,y,z",
                      [[3_000_000_000, 0.25 * x + 0.025, 0.25 * y + 0.025, 2.25]
                       for x in range(1, 26) for y in range(1, 26)])
            write_csv(root / "second_scan.csv", "timestamp_ns,x,y,z",
                      [[3_050_000_000, 0.25 * x + 0.025, 0.25 * y + 0.025, 2.26]
                       for x in range(1, 26) for y in range(1, 26)])
            write_csv(root / "scans.csv", "end_ns,receive_ns,points",
                      [[3_000_000_000, 3_000_100_000, "scan.csv"],
                       [3_050_000_000, 3_050_100_000, "second_scan.csv"]])
            outputs = []
            for name, flags in (("plain", []),
                                ("probe", ["--native-joint-query-probe"])):
                completed = subprocess.run(
                    [str(self.binary), str(root), str(root / name),
                     "--native-joint-audit-period-s", "0.005",
                     "--native-joint-max-dimension", "128",
                     "--native-joint-max-memory-rows", "256", *flags],
                    capture_output=True, text=True)
                self.assertEqual(1, completed.returncode, completed.stderr)
                self.assertIn("capacity exceeded before allocation",
                              completed.stderr)
                output = {}
                for filename in ("native_joint.csv", "odometry.csv"):
                    with (root / name / filename).open() as stream:
                        output[filename] = [
                            {k: v for k, v in row.items() if k != "imu_ms"}
                            for row in csv.DictReader(stream)]
                outputs.append(output)
            self.assertEqual(outputs[0], outputs[1])
            with (root / "probe/native_query_scope.csv").open() as stream:
                scope = next(csv.DictReader(stream))
            self.assertEqual("625", scope["points"])
            self.assertEqual("0", scope["graph_motion_exported"])
            with (root / "probe/native_query_points.csv").open() as stream:
                points = list(csv.DictReader(stream))
            self.assertEqual(list(range(625)),
                             [int(point["index"]) for point in points])
            self.assertFalse((root / "plain/native_query_scope.csv").exists())
            completed = subprocess.run(
                [str(self.binary), str(root), str(root / "structured"),
                 "--native-joint-audit-period-s", "0.005",
                 "--native-joint-max-dimension", "128",
                 "--native-joint-max-memory-rows", "256",
                 "--native-joint-structured", "--profile-local-timing"],
                capture_output=True, text=True)
            self.assertEqual(0, completed.returncode, completed.stderr)
            with (root / "structured/native_joint.csv").open() as stream:
                audit = list(csv.DictReader(stream))
            self.assertEqual("420", audit[-1]["sequence"])
            self.assertTrue(all(row["reason"] == "NONE" for row in audit))
            self.assertTrue(all(row["dimension"] == "0" for row in audit))
            self.assertTrue(all(row["representation"] ==
                                "structured_sources_not_dense_graph_state"
                                for row in audit))
            self.assertTrue(all(row["trusted"] == "0" for row in audit))
            with (root / "structured/native_resources.csv").open() as stream:
                resources = list(csv.DictReader(stream))
            self.assertEqual([row["sequence"] for row in audit],
                             [row["sequence"] for row in resources])
            self.assertGreater(int(resources[-1]["primitive_sources"]),
                               int(resources[0]["primitive_sources"]))
            query_ms = [float(row["publish_query_ms"]) for row in resources]
            self.assertEqual(query_ms, sorted(query_ms))
            self.assertTrue(all(int(row["snapshots"]) <= 64 for row in resources))
            with (root / "structured/odometry.csv").open() as stream:
                nominal = [{k: v for k, v in row.items() if k != "imu_ms"}
                           for row in csv.DictReader(stream)]
            self.assertEqual(outputs[0]["odometry.csv"],
                             nominal[:len(outputs[0]["odometry.csv"])])

    def test_timer_requires_arrival_pacing_before_creating_output(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            completed = subprocess.run(
                [str(self.binary), str(root), str(root / "output"),
                 "--timer-output"],
                capture_output=True, text=True,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("timer output requires arrival-paced replay",
                          completed.stderr)
            self.assertFalse((root / "output").exists())

    def test_uninitialized_raw_replay_fails_and_preserves_startup_evidence(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            config = json.loads(Path(__file__).with_name(
                "native_lio_diagnostic_config.json"
            ).read_text())
            write_csv(root / "config.csv", "name,value", config.items())
            write_csv(root / "extrinsic.csv", "tx,ty,tz",
                      [[-0.011, -0.02329, 0.04412]])
            write_csv(
                root / "imu.csv",
                "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                [[1_000_000_000 + i * 5_000_000,
                  1_000_100_000 + i * 5_000_000,
                  0, 0, 9.80665, 0, 0, 0.1] for i in range(601)],
            )
            # The cloud must not be read while the native owner is uninitialized.
            write_csv(root / "scans.csv", "end_ns,receive_ns,points",
                      [[2_000_000_000, 2_000_100_000, "not-yet-needed.csv"]])
            completed = subprocess.run(
                [str(self.binary), str(root), str(root / "output")],
                capture_output=True, text=True,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("initialization/LIO update not established",
                          completed.stderr)
            with (root / "output/summary.csv").open() as stream:
                summary = next(csv.DictReader(stream))
            self.assertEqual(summary["valid_states"], "0")
            self.assertEqual(summary["lidar_updates"], "0")
            self.assertEqual(summary["warmup_scans"], "1")
            with (root / "output/lidar.csv").open() as stream:
                decision = next(csv.DictReader(stream))
            self.assertEqual(decision["reason"], "WAITING_FOR_STATIONARY")
            with (root / "output/motion_initialization.csv").open() as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])

    def test_raw_lio_without_external_odometry_and_no_qualification_override(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            config = json.loads(Path(__file__).with_name(
                "native_lio_diagnostic_config.json"
            ).read_text())
            write_csv(root / "config.csv", "name,value", config.items())
            write_csv(root / "extrinsic.csv", "tx,ty,tz",
                      [[-0.011, -0.02329, 0.04412]])
            imu = [
                [1_000_000_000 + i * 5_000_000,
                 1_000_100_000 + i * 5_000_000, 0, 0, 9.80665, 0, 0, 0]
                for i in range(601)
            ]
            write_csv(root / "imu.csv",
                      "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz", imu)
            scans = []
            for index in range(4):
                end = 3_200_000_000 + index * 100_000_000
                points = [
                    [end - 10_000_000, cell_x + 0.1 * x,
                     cell_y + 0.1 * y, 2.0]
                    for cell_x in range(5) for cell_y in range(5)
                    for x in range(1, 6) for y in range(1, 6)
                ]
                name = f"scan{index}.csv"
                write_csv(root / name, "timestamp_ns,x,y,z", points)
                scans.append([end, end + 2_000_000, name])
            write_csv(root / "scans.csv", "end_ns,receive_ns,points", scans)
            completed = subprocess.run(
                [str(self.binary), str(root), str(root / "output")],
                capture_output=True, text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            with (root / "output/lidar.csv").open() as stream:
                decisions = list(csv.DictReader(stream))
            self.assertEqual(len(decisions), 4)
            self.assertTrue(any(row["reason"] == "NONE" for row in decisions))
            for row in decisions:
                self.assertEqual(int(row["reference_ns"]) -
                                 int(row["raw_end_ns"]), 5_000_000)
                self.assertEqual(int(row["source_receive_ns"]) -
                                 int(row["raw_end_ns"]), 2_000_000)
                self.assertGreaterEqual(
                    int(row["processing_frontier_receive_ns"]),
                    int(row["source_receive_ns"]),
                )
            with (root / "output/odometry.csv").open() as stream:
                states = list(csv.DictReader(stream))
            self.assertTrue(any(row["valid"] == "1" for row in states))
            self.assertEqual(states[-1]["covariance_model_valid"], "0")
            self.assertEqual(states[-1]["motion_available"], "0")
            self.assertEqual(states[-1]["capability_reason"],
                             "CORRELATION_UNQUALIFIED")
            self.assertEqual(int(states[-1]["timestamp_ns"]), imu[-1][0])
            paced = root / "rate2"
            replay = subprocess.run(
                [str(self.binary), str(root), str(paced),
                 "--arrival-paced", "--arrival-rate", "2", "--timer-output"],
                capture_output=True, text=True)
            self.assertEqual(0, replay.returncode, replay.stderr)
            with (paced / "odometry.csv").open() as stream:
                accelerated = list(csv.DictReader(stream))
            self.assertEqual(
                [{k: v for k, v in row.items() if k != "imu_ms"} for row in states],
                [{k: v for k, v in row.items() if k != "imu_ms"} for row in accelerated])
            with (paced / "host_timing.csv").open() as stream:
                timing = list(csv.DictReader(stream))
            origin = int(timing[0]["target_steady_ns"])
            self.assertTrue(all(int(row["target_steady_ns"]) - origin ==
                                (int(row["receive_ns"]) - imu[0][1]) // 2 for row in timing))
            with (paced / "replay_clock.csv").open() as stream:
                self.assertEqual("2", next(csv.DictReader(stream))["arrival_rate"])
            self.assertEqual(2, summarize_host_timing(paced / "host_timing.csv")["arrival_rate"])
            with (paced / "replay_clock.csv").open("w") as stream:
                stream.write("first_receive_ns,arrival_rate,scope\n"
                             f"{imu[0][1]},5,wall_schedule_only_original_sensor_timestamps_preserved\n")
            with self.assertRaisesRegex(ValueError, "chronology changed"):
                summarize_host_timing(paced / "host_timing.csv")
            with (root / "output/local_corrections.csv").open() as stream:
                corrections = list(csv.DictReader(stream))
            self.assertEqual(len(corrections), len(scans))
            summary = summarize_local_corrections(corrections)
            self.assertEqual(summary["accepted_corrections"],
                             sum(r["reason"] == "NONE" for r in decisions))
            self.assertLess(summary["total_position_correction_path_m"], 1e-10)
            self.assertFalse((root / "output/imu_trace.csv").exists())
            self.assertFalse((root / "output/local_covariance.csv").exists())
            tracking = subprocess.run(
                [str(self.binary), str(root), str(root / "tracking"),
                 "--diagnostic-lidar-tracking-gain"],
                capture_output=True, text=True)
            self.assertEqual(0, tracking.returncode, tracking.stderr)
            with (root / "tracking/local_gain_policy.csv").open() as stream:
                declaration = next(csv.DictReader(stream))
            self.assertEqual("diagnostic", declaration["trust_ceiling"])
            self.assertEqual("original_conservative_marginal", declaration["uncertainty_export"])
            with (root / "tracking/odometry.csv").open() as stream:
                tracking_states = list(csv.DictReader(stream))
            self.assertEqual("0", tracking_states[-1]["covariance_model_valid"])
            self.assertEqual("0", tracking_states[-1]["motion_available"])
            audited = subprocess.run(
                [str(self.binary), str(root), str(root / "joint-audit"),
                 "--native-joint-audit-period-s", "0.005",
                 "--native-joint-max-dimension", "256",
                 "--native-joint-max-memory-rows", "4096",
                 "--native-joint-max-snapshots", "16"],
                capture_output=True, text=True,
            )
            self.assertNotEqual(audited.returncode, 0)
            self.assertIn("native joint audit:", audited.stderr)
            with (root / "joint-audit/native_joint.csv").open() as stream:
                assessments = list(csv.DictReader(stream))
            self.assertTrue(assessments)
            self.assertNotEqual(assessments[-1]["reason"], "NONE")
            self.assertTrue(assessments[-1]["detail"])
            self.assertTrue(all(row["diagnostic_only"] == "1" and
                                row["trusted"] == "0"
                                for row in assessments))
            self.assertTrue(all(row["maximum_dimension"] == "256" and
                                row["maximum_memory_rows"] == "4096" and
                                row["maximum_snapshots"] == "16" and
                                row["history_duration_s"] == "3"
                                for row in assessments))
            with (root / "joint-audit/odometry.csv").open() as stream:
                audited_states = list(csv.DictReader(stream))
            self.assertEqual(
                [{k: v for k, v in r.items() if k != "imu_ms"}
                 for r in states[:len(audited_states)]],
                [{k: v for k, v in r.items() if k != "imu_ms"}
                 for r in audited_states],
            )
            traced = subprocess.run(
                [str(self.binary), str(root), str(root / "traced"),
                 "--imu-trace", "--profile-local-timing"],
                capture_output=True, text=True,
            )
            self.assertEqual(traced.returncode, 0, traced.stderr)
            with (root / "traced/odometry.csv").open() as stream:
                traced_states = list(csv.DictReader(stream))
            self.assertEqual(
                [{k: v for k, v in r.items() if k != "imu_ms"} for r in states],
                [{k: v for k, v in r.items() if k != "imu_ms"} for r in traced_states],
            )
            self.assertFalse((root / "output/local_profile.csv").exists())
            with (root / "traced/local_profile.csv").open() as stream:
                profile = list(csv.DictReader(stream))
            self.assertEqual(len(scans), len(profile))
            for record in profile:
                self.assertGreater(int(record["points"]), 0)
                self.assertGreaterEqual(
                    int(record["decode_end_ns"]), int(record["decode_start_ns"]))
                self.assertGreaterEqual(
                    int(record["estimator_end_ns"]), int(record["estimator_start_ns"]))
            with (root / "traced/imu_trace.csv").open() as stream:
                trace = list(csv.DictReader(stream))
            self.assertTrue(trace)
            with (root / "traced/local_covariance.csv").open() as stream:
                covariance_trace = list(csv.DictReader(stream))
            self.assertEqual([row["sequence"] for row in trace],
                             [row["sequence"] for row in covariance_trace])
            last = covariance_trace[-1]
            matrices = {}
            for name in ("internal_before", "internal_after", "output_before",
                         "output_after", "transition", "process_noise"):
                values = list(map(float, last[name].split(";")))
                self.assertEqual(len(values), 225)
                matrices[name] = [values[i:i + 15] for i in range(0, 225, 15)]
            transition = matrices["transition"]
            for before, after in (("internal_before", "internal_after"),
                                  ("output_before", "output_after")):
                for axis in range(15):
                    expected = sum(
                        transition[axis][i] * matrices[before][i][j] *
                        transition[axis][j]
                        for i in range(15) for j in range(15))
                    expected += matrices["process_noise"][axis][axis]
                    if before == "output_before" and axis < 9:
                        correction = list(map(float, last[
                            ("translation_correction", "velocity_correction",
                             "rotation_correction")[axis // 3]].split(";")))
                        expected += (float(last["correction_variance_scale"]) *
                                     correction[axis % 3] ** 2)
                    self.assertAlmostEqual(expected, matrices[after][axis][axis],
                                           places=10)
            self.assertAlmostEqual(float(trace[-1]["force_z"]), 9.80665)
            self.assertAlmostEqual(float(trace[-1]["accel_bias_z"]), 0, places=12)
            accounted = summarize_imu_trace(trace, corrections)
            self.assertLess(max(map(abs, accounted["imu_velocity_impulse_mps"])), 1e-12)
            self.assertFalse(accounted["independent_truth_error"])
            delayed = subprocess.run(
                [str(self.binary), str(root), str(root / "delayed"),
                 "--arrival-paced", "--async-local-lio-ms", "50", "--imu-trace",
                 "--prefetch-lidar", "--timer-output"],
                capture_output=True, text=True,
            )
            self.assertEqual(delayed.returncode, 0, delayed.stderr)
            task_metrics = summarize_local_lidar_tasks(
                root / "delayed/local_lidar_tasks.csv", len(scans))
            self.assertGreater(task_metrics["accepted_corrections"], 0)
            with (root / "delayed/lidar.csv").open() as stream:
                delayed_decisions = list(csv.DictReader(stream))
            decode = summarize_lidar_decode(
                root / "delayed/lidar_decode.csv", delayed_decisions)
            self.assertEqual(decode["consumed_scans"], len(scans))
            self.assertTrue(decode["estimator_consumption_arrival_gated"])
            with (root / "delayed/imu_trace.csv").open() as stream:
                delayed_trace = list(csv.DictReader(stream))
            with (root / "delayed/local_corrections.csv").open() as stream:
                delayed_corrections = list(csv.DictReader(stream))
            summarize_imu_trace(delayed_trace, delayed_corrections)
            with (root / "delayed/odometry.csv").open() as stream:
                delayed_states = list(csv.DictReader(stream))
            timer = summarize_timer_output(root / "delayed/timer_output.csv")
            self.assertGreater(timer["new_measurements"], 0)
            self.assertGreater(timer["unavailable_publications"], 0)
            self.assertFalse(timer["apollo_publication_qualified"])
            with (root / "delayed/timer_output.csv").open() as stream:
                for published in csv.DictReader(stream):
                    if published["usable"] != "1":
                        continue
                    source = delayed_states[int(published["source_sequence"]) - 1]
                    self.assertEqual(published["measurement_ns"], source["timestamp_ns"])
                    for name in ("px", "py", "pz", "qx", "qy", "qz", "qw",
                                 "vx", "vy", "vz"):
                        self.assertAlmostEqual(float(published[name]),
                                               float(source[name]), places=12)
            self.assertEqual([(r["timestamp_ns"], r["valid"]) for r in states],
                             [(r["timestamp_ns"], r["valid"]) for r in delayed_states])
            with (root / "delayed/health.csv").open() as stream:
                self.assertTrue(all(r["health_trusted"] == "0"
                                    for r in csv.DictReader(stream)))
            config["max_translation_correction"] = 1e-9
            write_csv(root / "config.csv", "name,value", config.items())
            for index in range(1, 4):
                path = root / f"scan{index}.csv"
                with path.open() as stream:
                    points = list(csv.reader(stream))[1:]
                write_csv(path, "timestamp_ns,x,y,z",
                          [[t, x, y, float(z) + 0.02] for t, x, y, z in points])
            degraded = subprocess.run(
                [str(self.binary), str(root), str(root / "degraded"), "--imu-trace"],
                capture_output=True, text=True,
            )
            self.assertEqual(degraded.returncode, 0, degraded.stderr)
            with (root / "degraded/odometry.csv").open() as stream:
                states = list(csv.DictReader(stream))
            over_budget = [row for row in states
                           if row["capability_reason"] == "CORRECTION_BUDGET_EXCEEDED"]
            self.assertTrue(over_budget)
            with (root / "degraded/local_corrections.csv").open() as stream:
                corrections = list(csv.DictReader(stream))
            summary = summarize_local_corrections(corrections)
            self.assertGreater(summary["total_position_correction_path_m"], 0)
            with (root / "degraded/imu_trace.csv").open() as stream:
                degraded_trace = list(csv.DictReader(stream))
            accounting = summarize_imu_trace(degraded_trace, corrections)
            self.assertEqual(accounting["recorded_propagations"], len(trace))
            rejected = dict(next(r for r in corrections if r["reason"] == "NONE"))
            rejected["reason"] = "INNOVATION_REJECTED"
            with self.assertRaisesRegex(ValueError, "rejected LiDAR"):
                summarize_local_corrections([rejected])
            with (root / "degraded/capability.csv").open() as stream:
                diagnostic = next(row for row in csv.DictReader(stream)
                                  if row["capability_reason"] ==
                                  "CORRECTION_BUDGET_EXCEEDED")
            self.assertIn("translation_limit_m=", diagnostic["detail"])
            self.assertIn("rotation_limit_rad=", diagnostic["detail"])
            self.assertIn("velocity_limit_mps=", diagnostic["detail"])
            detail = dict(field.split("=") for field in
                          diagnostic["detail"].split(": ", 1)[1].split())
            self.assertEqual(float(detail["translation_limit_m"]), 1e-9)
            self.assertGreater(float(detail["translation_m"]), 1e-9)
            capability = summarize_capability(root / "degraded/capability.csv")
            self.assertEqual(
                capability["reasons"]["CORRECTION_BUDGET_EXCEEDED"],
                len(over_budget))
            self.assertEqual(
                capability["first_output_envelope_exceedance"], diagnostic)
            self.assertTrue(all(row["integrity_reason"] == "NONE" and
                                row["valid"] == "1" and
                                row["covariance_model_valid"] == "0" and
                                row["motion_available"] == "0"
                                for row in over_budget))
            self.assertEqual(int(states[-1]["timestamp_ns"]), imu[-1][0])
            with (root / "degraded/health.csv").open() as stream:
                self.assertTrue(all(row["health_trusted"] == "0"
                                    for row in csv.DictReader(stream)))
            write_csv(root / "imu.csv",
                      "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                      imu[:550] + imu[570:])
            gap = subprocess.run(
                [str(self.binary), str(root), str(root / "gap")],
                capture_output=True, text=True,
            )
            self.assertNotEqual(gap.returncode, 0)
            self.assertIn("IMU gap", gap.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: native_lio_replay_test.py NATIVE_LIO_BINARY")
    NativeReplayTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
