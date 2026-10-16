"""Public-engine diagnostic contracts; pass the built runner as argv[1]."""

import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


RUNNER = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None


def write_csv(path, header, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header.split(","))
        writer.writerows(rows)


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


class PublicReplayContracts(unittest.TestCase):
    def invoke(self, root, odom, fixes, mode="recovery"):
        write_csv(root / "odom.csv", "timestamp_ns,px,py,pz,qx,qy,qz,qw", odom)
        write_csv(
            root / "fixes.csv", "arrival_ns,timestamp_ns,gx,gy,sigma,quality", fixes
        )
        write_csv(root / "lever.csv", "lx,ly,lz", [[0, 0, 0.5]])
        output = root / "result.csv"
        result = subprocess.run(
            [
                str(RUNNER), str(root / "odom.csv"), str(root / "fixes.csv"),
                str(output), mode, "--diagnostic-synthetic-quality",
                str(root / "lever.csv"),
            ],
            capture_output=True, text=True, check=False,
        )
        return result, output

    def test_moving_recovery_then_synthetic_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            odom = []
            fixes = []
            for index in range(36):
                stamp = (index + 1) * 10**9
                x = min(index, 22) * 2
                odom.append([stamp, x, 0, 3, 0, 0, 0, 1])
                fixes.append([stamp, stamp, x + (10 if index >= 5 else 0), 0, 1, 1])
            result, output = self.invoke(root, odom, fixes)
            self.assertEqual(result.returncode, 0, result.stderr)
            rows = read_csv(output)
            moving = [
                row for row in rows
                if row["alignment_revision"] != "0"
                and int(row["odom_ns"]) <= 23 * 10**9
            ]
            self.assertTrue(moving)
            for row in moving:
                self.assertEqual(row["reset_counter"], "0")
                self.assertEqual(row["publication_state"], "held_for_reset")
                self.assertEqual(row["pending"], "1")
                self.assertEqual(row["vertical_initialized"], "0")
            self.assertEqual(rows[-1]["reset_counter"], "1")
            self.assertEqual(rows[-1]["mean_matches"], "1")
            self.assertEqual(rows[-1]["backend_tz"], "0")
            for row in rows:
                if row["global_valid"] == "1" and row["mean_matches"] == "0":
                    self.assertNotEqual(row["publication_state"], "aligned")
            resets = [
                row for row in rows
                if float(row["correction_jump_m"]) > 1.5
            ]
            self.assertEqual(len(resets), 1)
            self.assertLessEqual(float(resets[0]["speed_proxy_mps"]), 0.2)
            self.assertGreaterEqual(int(resets[0]["odom_ns"]), 28 * 10**9)
            self.assertEqual(read_csv(Path(str(output) + ".summary.csv"))[0]
                             ["pending_at_end"], "0")
            emitted = [row for row in rows if row["event"] == "odom"]
            self.assertEqual(len(emitted), len(odom))
            for expected, actual in zip(odom, emitted):
                self.assertEqual(int(actual["odom_ns"]), expected[0])
                self.assertEqual(float(actual["px"]), expected[1])
            self.assertTrue(all(row["epoch_generation"] == "1" for row in rows))

    def test_pending_duplicate_delay_and_unresolved_are_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            odom = [
                [(index + 1) * 10**9, 0, 0, 3, 0, 0, 0, 1]
                for index in range(25)
            ]
            fixes = [[index * 10**9, index * 10**9, 0, 0, 1, 1]
                     for index in range(1, 5)]
            fixes.extend([
                [4 * 10**9, 4 * 10**9, 0, 0, 1, 1],
                [24 * 10**9, 5 * 10**9, 0, 0, 1, 1],
                [26 * 10**9, 26 * 10**9, 0, 0, 1, 1],
            ])
            result, output = self.invoke(Path(tmp), odom, fixes)
            self.assertEqual(result.returncode, 0, result.stderr)
            factors = read_csv(Path(str(output) + ".factors.csv"))
            reasons = [row["reason"] for row in factors]
            self.assertIn("out_of_order_gnss_timestamp", reasons)
            self.assertIn("odometry extrapolation is not permitted", reasons)
            self.assertIn("pending_odometry_bracket", reasons)
            summary = read_csv(Path(str(output) + ".summary.csv"))[0]
            self.assertEqual(summary["pending_at_end"], "1")
            self.assertEqual(int(summary["output_events"]), len(odom) + len(fixes))

    def test_pending_resolves_at_right_bracket_and_bad_input_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            odom = [[10**9, 0, 0, 3, 0, 0, 0, 1],
                    [2 * 10**9, 1, 0, 3, 0, 0, 0, 1]]
            fixes = [[1500000000, 1500000000, 0.5, 0, 1, 1]]
            result, output = self.invoke(root, odom, fixes)
            self.assertEqual(result.returncode, 0, result.stderr)
            factors = read_csv(Path(str(output) + ".factors.csv"))
            self.assertEqual(factors[0]["reason"], "pending_odometry_bracket")
            self.assertEqual(factors[-1]["timestamp_ns"], "1500000000")
            self.assertEqual(factors[-1]["arrival_ns"], "2000000000")
            self.assertEqual(read_csv(Path(str(output) + ".summary.csv"))[0]
                             ["pending_at_end"], "0")
        for bad in ("nan", "0"):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as tmp:
                malformed = [list(odom[0]), list(odom[1])]
                malformed[1][7] = bad
                result, _ = self.invoke(Path(tmp), malformed, fixes)
                self.assertEqual(result.returncode, 2)
                self.assertIn("failed:", result.stderr)

    def test_tracking_bounds_recovery_outage_and_outlier(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            odom, fixes = [], []
            for index in range(65):
                stamp = (index + 1) * 10**9
                odom.append([stamp, 2 * index, 0, 3, 0, 0, 0, 1])
                if 25 <= index < 35:
                    continue
                offset = 10 if index >= 5 else 0
                if index == 45:
                    offset += 1000
                fixes.append([stamp, stamp, 2 * index + offset, 0, 1, 1])
            result, output = self.invoke(root, odom, fixes, mode="tracking")
            self.assertEqual(result.returncode, 0, result.stderr)
            rows = read_csv(output)
            pending = [row for row in rows if row["pending"] == "1"]
            self.assertTrue(pending)
            self.assertTrue(all(row["global_degraded"] == "1" for row in pending))
            for previous, current in zip(rows, rows[1:]):
                if previous["global_valid"] != "1":
                    continue
                seconds = (int(current["odom_ns"])-int(previous["odom_ns"]))/1e9
                self.assertLessEqual(float(current["correction_jump_m"]),
                                     0.3 * seconds + 1e-8)
                self.assertEqual(current["reset_counter"], previous["reset_counter"])
                if current["source_fresh"] == "0":
                    self.assertLess(float(current["correction_jump_m"]), 1e-8)
                    self.assertEqual(current["global_degraded"], "1")
            self.assertGreater(float(rows[-1]["published_tx"]), 1.0)
            emitted = [row for row in rows if row["event"] == "odom"]
            self.assertEqual([float(row["px"]) for row in emitted],
                             [row[1] for row in odom])
            self.assertTrue(all(row["vertical_initialized"] == "0" for row in rows))
            factors = read_csv(Path(str(output) + ".factors.csv"))
            outlier = [row for row in factors
                       if row["timestamp_ns"] == str(46 * 10**9)]
            self.assertTrue(outlier)
            self.assertTrue(all(row["accepted"] == "0" for row in outlier))


if __name__ == "__main__":
    if RUNNER is None:
        raise SystemExit("provide the built public_xy_dataset_replay")
    unittest.main()
