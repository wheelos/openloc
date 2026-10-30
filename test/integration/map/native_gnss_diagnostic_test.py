#!/usr/bin/env python3
"""Raw GNSS timing/admission and immutable-native-ODOM regression."""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

from native_lio_replay_test import write_csv
from prepare_native_gnss import event, inject_bundle, measurement_ns
from prepare_native_lio import digest
from run_native_map_diagnostic import compare_odometry, gnss_summary, horizontal_summary


def rows(path):
    with path.open() as source:
        return list(csv.DictReader(source))


class GnssPreparationTest(unittest.TestCase):
    def test_injected_bundle_preserves_original_identity_and_pins(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            write_csv(source / "events.csv",
                      "timestamp_ns,receive_ns,sequence,x,y,quality,adapter_reason",
                      [[100, 110, 1, 0, 1, 4, "ok"], [200, 210, 2, 1, 2, 4, "ok"]])
            (source / "config.csv").write_text("unchanged policy")
            (source / "identity.csv").write_text("unchanged frame")
            manifest = {
                "schema": "openloc.native-gnss-diagnostic-input.v1",
                "diagnostic_only": True, "uncertainty_qualified": False, "events": 2,
                "files": {p.name: digest(p) for p in source.iterdir()},
            }
            (source / "manifest.json").write_text(json.dumps(manifest))
            original_hash = digest(source / "events.csv")
            injected = inject_bundle(source, root / "jump", 100, 200, (10, -5))
            self.assertEqual(1, injected["fault_injection"]["affected_events"])
            changed = rows(root / "jump/events.csv")
            self.assertEqual(("10", "-4", "100", "110", "1"),
                             tuple(changed[0][key] for key in
                                   ("x", "y", "timestamp_ns", "receive_ns", "sequence")))
            self.assertEqual(original_hash, digest(source / "events.csv"))
            for name, pin in injected["files"].items():
                self.assertEqual(pin, digest(root / "jump" / name))
            inject_bundle(source, root / "offset", 100, 200, time_offset_ns=20)
            self.assertEqual("120", rows(root / "offset/events.csv")[0]["timestamp_ns"])
            dropped = inject_bundle(source, root / "drop", 100, 200, dropout=True)
            self.assertEqual(1, dropped["events"])
            self.assertFalse(dropped["all_arrival_events_retained"])
            mask = json.loads((root / "drop/fault_mask.json").read_text())
            self.assertEqual("1", mask[0]["original"]["sequence"])
            self.assertIsNone(mask[0]["derived"])
            for name, options in [
                    ("none", {}), ("not_triggered", {"start_ns": 300, "stop_ns": 400,
                                                    "shift_xy": (1, 0)}),
                    ("bad", {"shift_xy": (float("nan"), 0)}),
                    ("mixed", {"dropout": True, "time_offset_ns": 1})]:
                arguments = dict(start_ns=100, stop_ns=200)
                arguments.update(options)
                with self.subTest(name=name), self.assertRaises(ValueError):
                    inject_bundle(source, root / name, **arguments)
                self.assertFalse((root / name).exists())

    def test_original_measurement_and_arrival_are_not_interchangeable(self):
        receive = 1791772774037985641
        self.assertEqual(measurement_ns("023934.00", receive),
                         1791772774000000000)
        self.assertEqual(measurement_ns("235959.900", 86_400_100_000_000),
                         86_399_900_000_000)
        with self.assertRaisesRegex(ValueError, "utc_time"):
            measurement_ns("2026-10-12 10:39:34", receive)

    def test_bad_checksum_association_is_retained_not_a_quality_prefilter(self):
        message = SimpleNamespace(time="023934.00", Lat=22.7, Lon=114.3,
                                  N="N", E="E", qual=5, sats=19,
                                  hdop=0.7, age=2.0, stnID="271*44")
        class Transformer:
            def transform(self, lon, lat):
                raise AssertionError("unmatched raw epoch must not be projected")
        result = event(message, 1791772774037985641, 7, {}, Transformer(),
                       {"easting_m": 0, "northing_m": 0})
        self.assertEqual(result[:3], [1791772774000000000,
                                     1791772774037985641, 7])
        self.assertEqual(result[5], 5)
        self.assertIn("association", result[9])


class GnssReportTest(unittest.TestCase):
    def test_horizontal_result_is_partial_and_rejects_false_qualification(self):
        missing = dict(
            timestamp_ns="120", receive_ns="130", alignment_initialized="0",
            correction_id="", x="", y="", alignment_x="", alignment_y="",
            alignment_yaw="", last_observation_ns="", odom_fresh="1",
            horizontal_fresh="0", diagnostic_only="1", global_trusted="0",
            target_graph_available="0")
        acquired = dict(
            missing, alignment_initialized="1", correction_id="1", x="2", y="3",
            alignment_x="10", alignment_y="20", alignment_yaw="0.3",
            last_observation_ns="110", horizontal_fresh="1")
        summary = horizontal_summary([missing, acquired])
        self.assertTrue(summary["established"])
        self.assertFalse(summary["height_or_full_pose_established"])
        self.assertFalse(summary["uncertainty_qualified"])
        self.assertEqual(summary["first_measurement_ns"], 120)
        self.assertFalse(horizontal_summary([missing])["established"])
        with self.assertRaisesRegex(ValueError, "qualification"):
            horizontal_summary([dict(acquired, global_trusted="1")])
        with self.assertRaisesRegex(ValueError, "frontier"):
            horizontal_summary([dict(acquired, last_observation_ns="125")])
        with self.assertRaisesRegex(ValueError, "uninitialized"):
            horizontal_summary([dict(missing, x="0")])

    def test_every_arrival_has_an_explicit_final_diagnostic_outcome(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            events = []
            for sequence in range(1, 8):
                events.append([
                    sequence, sequence * 100, sequence * 100 + 10,
                    "accepted", "", "ok", "0", "", 0, 0,
                ])
            events[0][3:5] = ["rejected", "carrier_solution_not_fixed"]
            events[1][3:5] = ["waiting_for_stability", "fixed_solution_not_stable"]
            events[2][5] = "invalid_gga_utc_time_of_day"
            events[3][6:8] = ["not_submitted", "native_odometry_unavailable"]
            header = ("sequence,timestamp_ns,receive_ns,gate_outcome,gate_reason,"
                      "adapter_reason,owner_status,owner_reason,"
                      "receiver_accuracy_available,heading_available")
            write_csv(root / "gnss.csv", header, events)
            decisions = []
            for sequence, status, updated, message in (
                    (5, 0, 0, ""), (6, 0, 1, ""), (7, 6, 0, "too old")):
                decisions.append([
                    "raw-gga-horizontal-diagnostic", sequence, sequence * 100,
                    sequence * 100 + 10, status, updated, message, sequence,
                    1 if sequence >= 6 else 0,
                ])
            decision_header = ("source,sequence,timestamp_ns,receive_ns,status,"
                               "alignment_updated,message,decision_id,correction_id")
            write_csv(root / "global_decisions.csv", decision_header, decisions)
            result = gnss_summary(root, {"events": 7})
            self.assertEqual(result["final_event_outcomes"], {
                "gate_not_accepted": 2, "adapter_rejected": 1,
                "native_odometry_unavailable": 1, "unanchored": 1,
                "updated": 1, "rejected": 1,
            })
            self.assertEqual(len(result["event_diagnostics"]), 7)
            self.assertEqual(result["event_diagnostics"][4]["reason"],
                             "position_retained_without_global_anchor")
            self.assertEqual(result["event_diagnostics"][5]["correction_id"], "1")
            self.assertEqual(result["event_diagnostics"][6]["reason"], "too old")
            write_csv(root / "global_decisions.csv", decision_header, decisions[:-1])
            with self.assertRaisesRegex(ValueError, "lacks final owner decision"):
                gnss_summary(root, {"events": 7})


class NativeGnssReplayTest(unittest.TestCase):
    binary = None

    def test_four_arms_all_arrivals_no_heading_no_local_feedback(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            config = json.loads(Path(__file__).with_name(
                "native_lio_diagnostic_config.json").read_text())
            write_csv(root / "config.csv", "name,value", config.items())
            write_csv(root / "extrinsic.csv", "tx,ty,tz", [[0, 0, 0]])
            write_csv(root / "imu.csv",
                      "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                      [[10**9 + i * 5_000_000, 10**9 + i * 5_000_000 + 100_000,
                        0, 0, 9.80665, 0, 0, 0] for i in range(801)])
            points = []
            for a in range(1, 25):
                for b in range(1, 25):
                    x, y = a * 0.2, b * 0.2
                    points.extend([(x, y, 0), (x, 0, y), (0, x, y)])
            scans = []
            for i in range(12):
                end = 3_200_000_000 + i * 100_000_000
                name = f"scan{i}.csv"
                write_csv(root / name, "timestamp_ns,x,y,z",
                          [[end - 10_000_000, *p] for p in points])
                scans.append([end, end + 2_000_000, name])
            write_csv(root / "scans.csv", "end_ns,receive_ns,points", scans)
            map_bundle = root / "map_input"
            gnss = root / "gnss_input"
            map_bundle.mkdir()
            gnss.mkdir()
            identity = ["synthetic", "v1", "synthetic-content", "synthetic",
                        "map", "odom", "imu"]
            for bundle in (map_bundle, gnss):
                write_csv(bundle / "identity.csv",
                          "map_id,map_version,map_content_id,calibration_id,"
                          "map_frame,odom_frame,base_frame", [identity])
            write_csv(map_bundle / "seed.csv", "x,y,z,qx,qy,qz,qw",
                      [[0, 0, 0, 0, 0, 0, 1]])
            write_csv(map_bundle / "window.csv", "radius_m,cell_size_m", [[15, 2]])
            with (map_bundle / "global.pcd").open("w") as stream:
                stream.write(
                    "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                    f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n"
                    f"POINTS {len(points)}\nDATA ascii\n")
                stream.writelines(" ".join(map(str, p)) + "\n" for p in points)
            write_csv(gnss / "config.csv",
                      "sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,"
                      "max_age_s,dwell_ns,max_gap_ns,gnss_map_construction",
                      [[0.5, 2, -1, 0.4, 6, 3, 10, 100_000_000,
                        1_000_000_000, "unknown"]])
            epochs = [(1_220_000_000, 1_221_000_000, 4),
                      (1_320_000_000, 1_321_000_000, 4),
                      (3_220_000_000, 3_221_000_000, 4),
                      (3_320_000_000, 3_321_000_000, 4),
                      (3_310_000_000, 3_340_000_000, 4),
                      (3_420_000_000, 3_421_000_000, 5),
                      (3_520_000_000, 3_521_000_000, 4),
                      (3_620_000_000, 3_621_000_000, 4),
                      (4_220_000_000, 4_221_000_000, 4),
                      (4_800_000_000, 4_310_000_000, 4),
                      (0, 4_420_000_000, 4)]
            write_csv(gnss / "events.csv",
                      "timestamp_ns,receive_ns,sequence,x,y,quality,satellites,"
                      "hdop,age,adapter_reason",
                      [[t, r, i + 1, 2, -1, q, 19, 0.7, 2,
                        "ok" if t else "invalid_gga_utc_time_of_day"]
                       for i, (t, r, q) in enumerate(epochs)])
            for arm in ("odom", "gnss", "map", "both"):
                command = [str(self.binary), str(root), str(root / arm)]
                if arm in ("map", "both"):
                    command += ["--diagnostic-map", str(map_bundle),
                                "--map-input-sha256",
                                "".join(digest(map_bundle / name) for name in
                                        ("global.pcd", "identity.csv",
                                         "seed.csv", "window.csv"))]
                if arm in ("gnss", "both"):
                    command += ["--diagnostic-gnss", str(gnss),
                                "--gnss-input-sha256",
                                "".join(digest(gnss / name) for name in
                                        ("events.csv", "config.csv", "identity.csv"))]
                result = subprocess.run(command, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, (arm, result.stderr))
                compare_odometry(root / arm / "odometry.csv",
                                 root / "odom/odometry.csv")
            self.assertFalse((root / "odom/gnss.csv").exists())
            self.assertFalse((root / "map/gnss.csv").exists())
            horizontal_command = [
                str(self.binary), str(root), str(root / "horizontal"),
                "--diagnostic-gnss", str(gnss), "--gnss-input-sha256",
                "".join(digest(gnss / name) for name in
                        ("events.csv", "config.csv", "identity.csv")),
                "--diagnostic-level-odom-gnss-bootstrap",
            ]
            horizontal_run = subprocess.run(horizontal_command, capture_output=True,
                                            text=True)
            self.assertEqual(horizontal_run.returncode, 0, horizontal_run.stderr)
            compare_odometry(root / "horizontal/odometry.csv",
                             root / "odom/odometry.csv")
            partial = horizontal_summary(rows(root / "horizontal/horizontal.csv"))
            self.assertTrue(partial["enabled"])
            self.assertFalse(partial["established"])
            self.assertFalse((root / "gnss/horizontal.csv").exists())
            events = rows(root / "both/gnss.csv")
            self.assertEqual(len(events), len(epochs))
            self.assertEqual([int(r["timestamp_ns"]) for r in events],
                             [t for t, _, _ in epochs])
            self.assertEqual([int(r["receive_ns"]) for r in events],
                             [r for _, r, _ in epochs])
            self.assertEqual([r["gate_outcome"] for r in events],
                             ["waiting_for_stability", "accepted",
                              "waiting_for_stability", "accepted", "invalid",
                              "rejected", "waiting_for_stability", "accepted",
                              "accepted", "accepted", "invalid"])
            self.assertIn("did not advance", events[4]["gate_reason"])
            self.assertEqual(events[5]["gate_reason"], "carrier_solution_not_fixed")
            self.assertEqual(events[1]["owner_status"], "not_submitted")
            self.assertEqual(events[1]["owner_reason"],
                             "native_odometry_unavailable")
            self.assertNotEqual(events[-2]["owner_status"], "0")
            self.assertEqual(events[-1]["owner_reason"], "adapter_rejected")
            self.assertTrue(all(r["heading_available"] == "0" and
                                r["receiver_accuracy_available"] == "0"
                                for r in events))
            submitted = [row for row in rows(root / "both/graph_measurements.csv")
                         if row["kind"] == "gnss"]
            self.assertTrue(submitted)
            source = {row["sequence"]: row for row in events}
            for measurement in submitted:
                original = source[measurement["sequence"]]
                for key in ("timestamp_ns", "receive_ns", "lever_x", "lever_y", "lever_z"):
                    self.assertEqual(original[key], measurement[key])
                self.assertEqual(original["x"], measurement["px"])
                self.assertEqual(original["y"], measurement["py"])
                self.assertEqual("0", measurement["ambiguous"])
            self.assertEqual([(r["lever_x"], r["lever_y"], r["lever_z"])
                              for r in events],
                             [("2", "-1", "0.40000000000000002")] * len(epochs))
            gnss_only = gnss_summary(root / "gnss", {"events": len(epochs)})
            self.assertEqual(gnss_only["native_odometry_unavailable_events"], 1)
            self.assertGreater(gnss_only["unanchored_owner_decisions"], 0)
            self.assertEqual(gnss_only["committed_correction_updates"], 0)
            self.assertTrue(all(r["alignment_initialized"] == "0"
                                for r in rows(root / "gnss/global.csv")))
            decisions = rows(root / "both/global_decisions.csv")
            self.assertEqual([int(r["decision_id"]) for r in decisions],
                             list(range(1, len(decisions) + 1)))
            self.assertTrue(any(r["source"] == "raw-gga-horizontal-diagnostic"
                                for r in decisions))
            self.assertTrue(any(r["source"] == "native-raw-scan-to-prior-map"
                                and r["alignment_updated"] == "1"
                                for r in decisions))
            self.assertGreater(gnss_summary(root / "both", {"events": len(epochs)})[
                "committed_correction_updates"], 0)
            revisions = [int(r["correction_id"]) for r in decisions]
            self.assertEqual(revisions, sorted(revisions))
            self.assertTrue(all(r["gnss_map_construction"] == "unknown"
                                for r in decisions))
            future_check = command.copy()
            future_check[2] = str(root / "holdback_future")
            future_check += ["--global-holdback-ns", "100000000"]
            result = subprocess.run(future_check, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            future = [r for r in rows(root / "holdback_future/global_admission.csv")
                      if r["source"] == "raw-gga-horizontal-diagnostic" and
                      r["sequence"] == "10"]
            self.assertEqual(len(future), 1)
            self.assertNotEqual(future[0]["status"], "0")
            self.assertIn("future", future[0]["message"])
            delayed_epochs = [
                (3_300_000_000 + i * 100_000_000,
                 3_335_000_000 + i * 100_000_000) for i in range(9)
            ] + [(4_200_000_000, 4_800_000_000)]
            write_csv(gnss / "events.csv",
                      "timestamp_ns,receive_ns,sequence,x,y,quality,satellites,"
                      "hdop,age,adapter_reason",
                      [[t, r, i + 1, 2, -1, 4, 19, 0.7, 2, "ok"]
                       for i, (t, r) in enumerate(delayed_epochs)])
            for arm, holdback in (("immediate", 0), ("holdback", 100_000_000)):
                delayed = command.copy()
                delayed[2] = str(root / arm)
                delayed[-1] = "".join(digest(gnss / name) for name in
                                     ("events.csv", "config.csv", "identity.csv"))
                delayed += ["--global-holdback-ns", str(holdback)]
                result = subprocess.run(delayed, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                compare_odometry(root / arm / "odometry.csv",
                                 root / "odom/odometry.csv")
            immediate = gnss_summary(root / "immediate",
                                     {"events": len(delayed_epochs)})
            held = gnss_summary(root / "holdback",
                                {"events": len(delayed_epochs)})
            self.assertGreater(held["committed_correction_updates"],
                               immediate["committed_correction_updates"])
            admission = rows(root / "holdback/global_admission.csv")
            for entry in admission:
                self.assertGreaterEqual(int(entry["enqueue_processing_ns"]),
                                        int(entry["source_receive_ns"]))
                self.assertGreaterEqual(int(entry["submission_processing_ns"]),
                                        int(entry["enqueue_processing_ns"]))
                self.assertGreaterEqual(int(entry["submission_processing_ns"]) -
                                        int(entry["measurement_ns"]), 100_000_000)
            late = [entry for entry in admission
                    if entry["source"] == "raw-gga-horizontal-diagnostic"
                    and entry["sequence"] == "10"]
            self.assertEqual(len(late), 1)
            self.assertEqual(late[0]["status"], "6")
            # Restore the original events for the lever-arm ablation below.
            write_csv(gnss / "events.csv",
                      "timestamp_ns,receive_ns,sequence,x,y,quality,satellites,"
                      "hdop,age,adapter_reason",
                      [[t, r, i + 1, 2, -1, q, 19, 0.7, 2,
                        "ok" if t else "invalid_gga_utc_time_of_day"]
                       for i, (t, r, q) in enumerate(epochs)])
            write_csv(gnss / "config.csv",
                      "sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,"
                      "max_age_s,dwell_ns,max_gap_ns,gnss_map_construction",
                      [[0.5, 0, 0, 0, 6, 3, 10, 100_000_000,
                        1_000_000_000, "unknown"]])
            wrong_lever = command.copy()
            wrong_lever[2] = str(root / "wrong_lever")
            wrong_lever[-1] = "".join(digest(gnss / name) for name in
                                     ("events.csv", "config.csv", "identity.csv"))
            result = subprocess.run(wrong_lever, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            baseline = rows(root / "map/global.csv")[-1]
            correct = rows(root / "both/global.csv")[-1]
            wrong = rows(root / "wrong_lever/global.csv")[-1]
            # Compare identical map/native evidence, not an invented cm bound.
            def distance(value):
                return sum((float(value[k]) - float(baseline[k]))**2
                           for k in ("x", "y"))
            self.assertLess(distance(correct), distance(wrong))
            write_csv(gnss / "config.csv",
                      "sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,"
                      "max_age_s,dwell_ns,max_gap_ns,gnss_map_construction",
                      [[0.5, 0, 0, 0, 2**32 + 6, 3, 10, 100_000_000,
                        1_000_000_000, "unknown"]])
            overflow = command.copy()
            overflow[2] = str(root / "overflow")
            overflow[-1] = "".join(digest(gnss / name) for name in
                                  ("events.csv", "config.csv", "identity.csv"))
            result = subprocess.run(overflow, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("count outside uint32 range", result.stderr)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: native_gnss_diagnostic_test.py NATIVE_LIO_BINARY")
    NativeGnssReplayTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
