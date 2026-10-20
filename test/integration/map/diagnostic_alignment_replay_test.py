"""Prepared-observation replay contracts, not recorded-data accuracy evidence."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PARSER = argparse.ArgumentParser()
PARSER.add_argument("binary", type=Path)
PARSER.add_argument("--artifacts", type=Path)
ARGS, TEST_ARGS = PARSER.parse_known_args()
BINARY = ARGS.binary.resolve()
if ARGS.artifacts:
    ARGS.artifacts.mkdir(parents=True, exist_ok=False)

START = 10_000_000_000
STEP = 10_000_000
ODOM_HEADER = (
    "arrival_ns,timestamp_ns,session,generation,px,py,pz,qx,qy,qz,qw,covariance"
)
MAP_HEADER = (
    "arrival_ns,timestamp_ns,session,generation,sequence,source,kind,"
    "px,py,pz,qx,qy,qz,qw,covariance,projection,projected_covariance,"
    "covariance_source,ambiguous,recovery"
)
XY_HEADER = (
    "arrival_ns,timestamp_ns,session,generation,sequence,source,"
    "x,y,covariance,lx,ly,lz,covariance_source"
)
CONTRACT_HEADER = (
    "map_id,map_version,map_content_id,calibration_id,map_frame,odom_frame,"
    "base_frame"
)
PROVENANCE_HEADER = (
    "acquisition_id,preparation_id,odometry_initialization,odometry_deskew,"
    "shared_local_scans,gnss_map_construction"
)


def matrix(size, diagonal):
    return "|".join(
        ";".join(str(diagonal if row == col else 0) for col in range(size))
        for row in range(size)
    )


def odom(index, generation=1):
    return [
        START + (index if index < 3 else index + 2) * STEP,
        START + index * STEP,
        "synthetic-local",
        generation,
        index, 0, 0, 0, 0, 0, -1 if index == 0 else 1,
        matrix(6, 0.01),
    ]


def map_pose(index, **changes):
    row = [
        START + index * STEP, START + index * STEP,
        "synthetic-local", 1, index + 1, "map-matcher", "full",
        100 + index, 20, 0, 0, 0, 0, 1, matrix(6, 0.01),
        "-", "-", "synthetic-not-calibrated", 0, 0,
    ]
    for column, value in changes.items():
        row[MAP_HEADER.split(",").index(column)] = value
    return row


def fix(index, arrival):
    return [
        arrival, START + index * STEP, "synthetic-local", 1, index + 1,
        "receiver", 100 + index, 20, matrix(2, 0.1), 0, 0, 0,
        "synthetic-not-calibrated",
    ]


def write(path, header, rows):
    with path.open("w", newline="") as stream:
        output = csv.writer(stream)
        output.writerow(header.split(","))
        output.writerows(rows)


def read(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if any(None in row or any(v is None for v in row.values()) for row in rows):
        raise AssertionError(f"invalid output shape: {path}")
    return rows


class ReplayTest(unittest.TestCase):
    def setUp(self):
        if ARGS.artifacts:
            self.root = ARGS.artifacts / self._testMethodName
            self.root.mkdir()
        else:
            temporary = tempfile.TemporaryDirectory()
            self.addCleanup(temporary.cleanup)
            self.root = Path(temporary.name)
        write(self.root / "contract.csv", CONTRACT_HEADER, [
            ["synthetic-site", "v1", "synthetic-map-not-a-file-hash",
             "synthetic-calibration", "map", "odom", "body"],
        ])

    def inputs(self, odometry, maps=(), fixes=()):
        write(self.root / "odom.csv", ODOM_HEADER, odometry)
        write(self.root / "map.csv", MAP_HEADER, maps)
        write(self.root / "xy.csv", XY_HEADER, fixes)

    def invoke(self, arm="all", name=None, flag="--diagnostic-only",
               map_path="map.csv", xy_path="xy.csv"):
        output = self.root / (name or arm)
        process = subprocess.run(
            [str(BINARY), str(self.root / "odom.csv"),
             str(self.root / map_path) if map_path != "-" else "-",
             str(self.root / xy_path) if xy_path != "-" else "-",
             str(self.root / "contract.csv"), str(output), arm, flag],
            capture_output=True, text=True, check=False,
        )
        if ARGS.artifacts:
            (self.root / f"{name or arm}.log").write_text(
                f"command={json.dumps(process.args)}\n"
                f"returncode={process.returncode}\n{process.stdout}"
                f"{process.stderr}"
            )
        return process, output

    def successful(self, **kwargs):
        process, output = self.invoke(**kwargs)
        self.assertEqual(process.returncode, 0, process.stderr)
        events = read(output / "events.csv")
        decisions = read(output / "decisions.csv")
        summary = read(output / "summary.csv")[0]
        self.assertEqual(
            int(summary["selected_observations"]),
            sum(int(summary[key])
                for key in ("updates", "rejected", "retained", "pending")),
        )
        return events, decisions, summary

    def test_four_arms_delay_nullspace_and_unresolved_tail(self):
        projected = map_pose(
            3, arrival_ns=START + 35_000_000, kind="projected", px=103.1,
            py=5000, covariance="-", projection="1;0;0;0;0;0",
            projected_covariance="0.01",
        )
        maps = [map_pose(i) for i in range(3)] + [
            projected,
            map_pose(5, arrival_ns=START + 72_000_000, px=5000),
            map_pose(8, arrival_ns=START + 100_000_000),
        ]
        self.inputs(
            [odom(i) for i in range(8)], maps,
            [fix(0, START + 1), fix(4, START + 65_000_000)],
        )
        odom_outputs = []
        for arm in ("odom", "gnss", "map", "all"):
            events, decisions, summary = self.successful(arm=arm)
            self.assertEqual(len(events), 16)
            self.assertTrue(all(r["diagnostic_only"] == "1" for r in events))
            self.assertTrue(all(r["global_trusted"] == "0" for r in events))
            odom_outputs.append([
                tuple(r[key] for key in (
                    "result_timestamp_ns", "session", "generation",
                    "odom_px", "odom_py", "odom_pz",
                    "odom_qx", "odom_qy", "odom_qz", "odom_qw",
                ))
                for r in events if r["kind"] == "odom"
            ])
            if arm in ("odom", "gnss"):
                self.assertTrue(all(r["alignment_initialized"] == "0"
                                    for r in events))
                self.assertTrue(all(r["x"] == "" for r in events))
                self.assertEqual(summary["updates"], "0")
            else:
                delayed = next(r for r in events
                               if r["kind"] == "map" and r["sequence"] == "4")
                self.assertEqual(delayed["event_status"], "11")
                self.assertEqual(delayed["pending_observations"], "1")
                final = next(r for r in decisions
                             if r["source"] == "map-matcher"
                             and r["sequence"] == "4")
                self.assertEqual(final["timestamp_ns"], str(START + 3 * STEP))
                self.assertEqual(final["arrival_ns"], str(START + 35_000_000))
                self.assertEqual(final["alignment_updated"], "1")
                self.assertEqual(final["decision_time_ns"],
                                 str(START + 50_000_000))
                self.assertEqual(final["queue_wait_ns"], "15000000")
                self.assertEqual(summary["maximum_queue_wait_ns"], "15000000")
                self.assertEqual(summary["peak_pending"], "1")
                self.assertAlmostEqual(float(events[-1]["y"]), 20.0)
                self.assertEqual(summary["pending"], "1")
                self.assertEqual(summary["rejected"], "3")
                self.assertEqual(summary["updates"], "3" if arm == "all" else "2")
                self.assertEqual([int(r["decision_id"]) for r in decisions],
                                 list(range(1, len(decisions) + 1)))
            if arm in ("gnss", "all"):
                self.assertEqual(decisions[1 if arm == "all" else 0]["status"],
                                 "0")
                self.assertEqual(summary["retained"], "1" if arm == "all" else "2")
        self.assertTrue(all(rows == odom_outputs[0] for rows in odom_outputs))
        self.assertEqual(odom_outputs[0][0][-1], "-1")

    def test_correction_metrics_exclude_odometry_motion(self):
        self.inputs(
            [odom(i) for i in range(5)],
            [map_pose(i) for i in range(3)] +
            [map_pose(3, arrival_ns=odom(3)[0], px=103.2)],
        )
        self.successful()
        metrics = read(self.root / "all" / "metrics.csv")
        events = read(self.root / "all" / "events.csv")
        summary = read(self.root / "all" / "summary.csv")[0]
        self.assertEqual(len(metrics), len(events))
        motion = [r for r in metrics if r["kind"] == "odom"
                  and r["correction_translation_m"] != ""]
        self.assertTrue(motion)
        self.assertTrue(all(abs(float(r["correction_translation_m"])) < 1e-12
                            for r in motion))
        correction = next(r for r in metrics
                          if r["kind"] == "map"
                          and r["correction_translation_m"] != "")
        self.assertGreater(float(correction["correction_translation_m"]), 0)
        self.assertLess(float(correction["correction_translation_m"]), 0.2)
        self.assertAlmostEqual(float(summary["maximum_translation_m"]),
                               float(correction["correction_translation_m"]))
        waits = read(self.root / "all" / "decisions.csv")
        self.assertTrue(all(r["queue_wait_ns"] == "0" for r in waits))
        self.assertEqual(metrics[-1]["arrival_delay_ns"], "20000000")

    def test_epoch_change_retires_delayed_observation(self):
        self.inputs(
            [odom(i, 1 if i < 3 else 2) for i in range(4)],
            [map_pose(4, arrival_ns=START + 25_000_000)],
        )
        events, decisions, summary = self.successful()
        self.assertEqual(decisions[-1]["status"], "14")
        self.assertEqual(summary["pending"], "0")
        self.assertEqual(events[-1]["generation"], "2")
        self.assertEqual(events[-1]["alignment_initialized"], "0")

    def test_recovery_restarts_after_ambiguity(self):
        maps = [map_pose(i) for i in range(3)]
        maps += [
            map_pose(i, arrival_ns=odom(i)[0], px=150 + i,
                     recovery=1, ambiguous=int(i == 4))
            for i in range(3, 8)
        ]
        self.inputs([odom(i) for i in range(8)], maps)
        events, decisions, summary = self.successful()
        recovering = next(r for r in events
                          if r["kind"] == "map" and r["sequence"] == "4")
        self.assertEqual(recovering["alignment_initialized"], "0")
        self.assertEqual(recovering["verification_pending"], "1")
        ambiguous = next(r for r in events
                         if r["kind"] == "map" and r["sequence"] == "5")
        self.assertEqual(ambiguous["verification_pending"], "0")
        self.assertEqual(events[-1]["correction_id"], "2")
        self.assertAlmostEqual(float(events[-1]["x"]), 157.0)
        self.assertEqual(decisions[-1]["alignment_updated"], "1")
        self.assertEqual(summary["updates"], "2")

    def test_duplicate_late_and_stale_events_are_logged_not_filtered(self):
        maps = [map_pose(i) for i in range(3)] + [
            map_pose(2, arrival_ns=START + 25_000_000),
            map_pose(1, sequence=10, arrival_ns=START + 26_000_000),
            map_pose(0, sequence=11, arrival_ns=START + 3_000_000_000),
        ]
        self.inputs([odom(i) for i in range(4)], maps)
        events, decisions, summary = self.successful()
        self.assertEqual(len(events), 10)
        self.assertEqual([r["status"] for r in decisions[-3:]], ["5", "6", "2"])
        self.assertEqual(summary["updates"], "1")
        self.assertEqual(events[-1]["alignment_initialized"], "1")
        self.assertEqual(events[-1]["odom_fresh"], "0")
        self.assertEqual(events[-1]["full_pose_evidence_fresh"], "0")

    def test_covariance_cross_terms_and_invalid_geometry_reach_owner(self):
        covariance = [
            [0.01 if row == col else 0 for col in range(6)]
            for row in range(6)
        ]
        covariance[0][1] = covariance[1][0] = 0.002
        encoded = "|".join(";".join(map(str, row)) for row in covariance)
        maps = [map_pose(i, covariance=encoded) for i in range(3)]
        maps.append(map_pose(
            3, arrival_ns=odom(3)[0], covariance=matrix(6, -0.01),
        ))
        self.inputs([odom(i) for i in range(4)], maps)
        events, decisions, summary = self.successful()
        output_covariance = [
            [float(value) for value in row.split(";")]
            for row in events[-1]["alignment_covariance"].split("|")
        ]
        self.assertGreater(abs(output_covariance[0][1]), 0)
        self.assertAlmostEqual(output_covariance[0][1], output_covariance[1][0])
        self.assertEqual(decisions[-1]["status"], "1")
        self.assertEqual(decisions[-1]["alignment_updated"], "0")
        self.assertEqual(summary["updates"], "1")

    def test_empty_optional_streams_and_no_overwrite(self):
        self.inputs([odom(0)])
        self.successful(arm="odom", map_path="-", xy_path="-")
        original = (self.root / "odom" / "events.csv").read_bytes()
        process, _ = self.invoke(arm="odom", map_path="-", xy_path="-")
        self.assertNotEqual(process.returncode, 0)
        self.assertTrue(process.stderr.strip())
        self.assertEqual(original, (self.root / "odom" / "events.csv").read_bytes())
        process, output = self.invoke(name="production", flag="--production")
        self.assertNotEqual(process.returncode, 0)
        self.assertFalse(output.exists())

    def test_invalid_arrival_order_covariance_and_odometry_fail_explicitly(self):
        self.inputs([odom(1), odom(0)])
        process, output = self.invoke(name="arrival-regression")
        self.assertNotEqual(process.returncode, 0)
        self.assertFalse(output.exists())
        invalid = odom(0)
        invalid[-1] = "1"
        self.inputs([invalid])
        process, output = self.invoke(name="matrix-shape")
        self.assertNotEqual(process.returncode, 0)
        self.assertFalse(output.exists())
        invalid = odom(0)
        invalid[10] = 0
        self.inputs([invalid])
        process, _ = self.invoke(name="invalid-odom")
        self.assertNotEqual(process.returncode, 0)
        self.assertTrue(process.stderr.strip())

    def test_observation_before_first_odom_is_not_lost(self):
        self.inputs([odom(0)], [map_pose(0, arrival_ns=START - 1)])
        events, decisions, summary = self.successful()
        self.assertEqual(events[0]["kind"], "map")
        self.assertEqual(events[0]["event_status"], "11")
        self.assertEqual(len(decisions), 1)
        self.assertEqual(summary["rejected"], "1")
        self.assertEqual(summary["pending"], "0")
        self.assertEqual(summary["peak_pending"], "1")

    def test_early_rejection_does_not_acquire_artificial_queue_delay(self):
        invalid = map_pose(0, arrival_ns=START - 30_000_000)
        self.inputs([odom(0)], [invalid])
        _, decisions, summary = self.successful()
        self.assertEqual(decisions[0]["status"], "2")
        self.assertEqual(decisions[0]["decision_time_ns"],
                         str(START - 30_000_000))
        self.assertEqual(decisions[0]["queue_wait_ns"], "0")
        self.assertEqual(summary["peak_pending"], "0")

    def test_provenance_survives_queue_rejection_and_excluded_arms(self):
        self.inputs([odom(i) for i in range(4)])
        provenance = [
            "synthetic-shared-capture", "synthetic-preparation-v1",
            "present", "present", "present", "unknown",
        ]
        maps = [map_pose(i) + provenance for i in range(3)] + [
            map_pose(3, arrival_ns=START + 35_000_000) + provenance,
        ]
        write(self.root / "map.csv", MAP_HEADER + "," + PROVENANCE_HEADER, maps)
        fixes = [
            fix(2, START + 45_000_000) +
            ["synthetic-receiver", "synthetic-xy-v1",
             "absent", "absent", "unknown", "present"],
        ]
        write(self.root / "xy.csv", XY_HEADER + "," + PROVENANCE_HEADER, fixes)
        events, decisions, summary = self.successful()
        queued = next(r for r in events
                      if r["kind"] == "map" and r["sequence"] == "4")
        self.assertEqual(queued["event_status"], "11")
        self.assertEqual(queued["acquisition_id"], provenance[0])
        self.assertEqual(queued["shared_local_scans"], "present")
        applied = decisions[-1]
        self.assertEqual(applied["source"], "map-matcher")
        self.assertEqual(applied["alignment_updated"], "1")
        self.assertEqual(applied["preparation_id"], provenance[1])
        self.assertEqual(applied["covariance_source"],
                         "synthetic-not-calibrated")
        self.assertEqual(applied["gnss_map_construction"], "unknown")
        self.assertTrue(all(r["global_trusted"] == "0" for r in events))
        excluded, excluded_decisions, _ = self.successful(arm="odom")
        self.assertFalse(excluded_decisions)
        excluded_map = next(r for r in excluded if r["kind"] == "map")
        self.assertEqual(excluded_map["event_status"], "not_requested")
        self.assertEqual(excluded_map["odometry_initialization"], "present")
        self.assertEqual(summary["updates"], "3")

    def test_unknown_provenance_is_not_inferred_and_bad_tag_fails(self):
        self.inputs([odom(0)], [map_pose(0)])
        events, decisions, _ = self.successful()
        self.assertEqual(events[-1]["shared_local_scans"], "unknown")
        self.assertEqual(decisions[-1]["odometry_deskew"], "unknown")
        self.assertEqual(decisions[-1]["acquisition_id"], "")
        invalid = map_pose(0) + ["capture", "preparation", "independent",
                                 "unknown", "unknown", "unknown"]
        write(self.root / "map.csv", MAP_HEADER + "," + PROVENANCE_HEADER,
              [invalid])
        process, output = self.invoke(name="invalid-dependency")
        self.assertNotEqual(process.returncode, 0)
        self.assertFalse(output.exists())
        self.assertIn("dependency", process.stderr)


if __name__ == "__main__":
    program = unittest.main(argv=[sys.argv[0], *TEST_ARGS], exit=False)
    if ARGS.artifacts:
        (ARGS.artifacts / "replay_test_source.py").write_text(
            Path(__file__).read_text()
        )
        manifest = {
            "kind": "synthetic_software_contract_evidence_not_dataset_baseline",
            "passed": program.result.wasSuccessful(),
            "tests_run": program.result.testsRun,
            "binary_sha256": hashlib.sha256(BINARY.read_bytes()).hexdigest(),
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "files": {
                str(path.relative_to(ARGS.artifacts)):
                    hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(ARGS.artifacts.rglob("*")) if path.is_file()
            },
        }
        (ARGS.artifacts / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
    sys.exit(not program.result.wasSuccessful())
