#!/usr/bin/env python3
"""Opt-in real-input bridge contracts; fixtures do not qualify field noise."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from native_lio_replay_test import write_csv
from prepare_native_lio import digest


def read(path):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    if any(None in row or None in row.values() for row in rows):
        raise ValueError(f"malformed output schema: {path.name}")
    return rows


class StructuredClosureTest(unittest.TestCase):
    binary = None

    def fixture(self, root, adjacent_gnss=False):
        config = json.loads(Path(__file__).with_name(
            "native_lio_diagnostic_config.json").read_text())
        write_csv(root / "config.csv", "name,value", config.items())
        write_csv(root / "extrinsic.csv", "tx,ty,tz", [[0, 0, 0]])
        write_csv(root / "imu.csv",
                  "timestamp_ns,receive_ns,ax,ay,az,gx,gy,gz",
                  [[1_000_000_000 + i * 5_000_000,
                    1_000_100_000 + i * 5_000_000,
                    0, 0, 9.80665, 0, 0, 0] for i in range(601)])
        points = []
        for a in range(24):
            for b in range(24):
                x, y = 0.2 * a + 0.03713, 0.2 * b + 0.04317
                points.extend([(x, y, 0.45119), (5.73713, x, y),
                               (x, 5.34317, y)])
        scans = []
        for i in range(4):
            stamp = 3_200_000_000 + i * 100_000_000
            if adjacent_gnss and i == 3:
                stamp = 3_450_000_000
            name = f"scan{i}.csv"
            write_csv(root / name, "timestamp_ns,x,y,z",
                      [[stamp - 10_000_000, *p] for p in points])
            scans.append([stamp, stamp + 2_000_000, name])
        write_csv(root / "scans.csv", "end_ns,receive_ns,points", scans)
        bundle = root / "map"
        bundle.mkdir()
        write_csv(bundle / "identity.csv",
                  "map_id,map_version,map_content_id,calibration_id,"
                  "map_frame,odom_frame,base_frame",
                  [["fixture", "v1", "fixture-content", "fixture",
                    "map", "odom", "imu"]])
        write_csv(bundle / "seed.csv", "x,y,z,qx,qy,qz,qw",
                  [[0, 0, 0, 0, 0, 0, 1]])
        write_csv(bundle / "seed_time.csv", "reference_ns,maximum_offset_ns",
                  [[3_205_000_000, 0]])
        write_csv(bundle / "window.csv", "radius_m,cell_size_m", [[15, 2]])
        with (bundle / "global.pcd").open("w") as stream:
            stream.write(
                "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n"
                f"POINTS {len(points)}\nDATA ascii\n")
            for p in points:
                stream.write(" ".join(map(str, p)) + "\n")
        gnss = root / "gnss"
        gnss.mkdir()
        (gnss / "identity.csv").write_bytes((bundle / "identity.csv").read_bytes())
        write_csv(gnss / "config.csv",
                  "sigma_m,lever_x,lever_y,lever_z,min_satellites,max_hdop,"
                  "max_age_s,dwell_ns,max_gap_ns,gnss_map_construction",
                  [[1, 0, 0, 0, 6, 3, 10, 2_000_000_000, 2_000_000_000,
                    "present"]])
        write_csv(gnss / "events.csv",
                  "timestamp_ns,receive_ns,sequence,x,y,quality,"
                  "satellites,hdop,age,adapter_reason",
                  [[452_500_000 + i * 1_000_000_000,
                    470_000_000 + i * 1_000_000_000,
                    i + 1, 0.01, -0.02, 4, 28, 0.6, 1, "ok"]
                   for i in range(4)])
        selection = root / "selection.csv"
        write_csv(selection, "scan_sequence", [[1], [2], [3], [4]])
        return [
            "--native-joint-audit-period-s", "0.005",
            "--native-joint-max-dimension", "2048",
            "--native-joint-max-snapshots", "64", "--native-joint-structured",
            "--diagnostic-map", str(bundle), "--map-input-sha256",
            "".join(digest(bundle / n) for n in
                    ("global.pcd", "identity.csv", "seed.csv", "window.csv",
                     "seed_time.csv")),
            "--diagnostic-gnss", str(gnss), "--gnss-input-sha256",
            "".join(digest(gnss / n) for n in
                    ("events.csv", "config.csv", "identity.csv")),
            "--offline-structured-closure", str(selection),
        ]

    def test_selected_original_time_family_reaches_all_four_graph_arms(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            flags = self.fixture(root)
            output = root / "result"
            result = subprocess.run(
                [str(self.binary), str(root), str(output), *flags],
                capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            family = read(output / "structured_family.csv")[0]
            self.assertEqual(("5", "4", "1", "99"),
                             tuple(family[k] for k in
                                   ("nodes", "map_queries", "gnss_queries", "rows")))
            self.assertEqual("0", family["trusted"])
            nodes = read(output / "structured_nodes.csv")
            original = next(row for row in nodes
                            if row["timestamp_ns"] == "3452500000")
            self.assertEqual(("3450000000", "3455000000", "1"),
                             tuple(original[k] for k in
                                   ("left_imu_ns", "right_imu_ns", "interpolated")))
            for arm, expected_map, expected_gnss in (
                    ("lio", 2, 0), ("gnss", 2, 1),
                    ("map", 4, 0), ("all", 4, 1)):
                for suffix in ("lag", "no_eviction"):
                    prefix = f"{arm}_{suffix}"
                    decisions = read(output / f"{prefix}_decisions.csv")
                    self.assertEqual(expected_map, sum(
                        d["source"] == "actual-structured-ICP" for d in decisions))
                    self.assertEqual(expected_gnss, sum(
                        d["source"] == "raw-gga-horizontal-diagnostic"
                        for d in decisions))
                    gnss = [d for d in decisions
                            if d["source"] == "raw-gga-horizontal-diagnostic"]
                    self.assertTrue(all(d["measurement_ns"] == "3452500000"
                                        for d in gnss))
                    self.assertTrue(all(d["status"] == "0" and
                                        d["alignment_updated"] == "1"
                                        for d in gnss))
                    self.assertTrue(any(d["alignment_updated"] == "1"
                                        for d in decisions))
                    states = read(output / f"{prefix}_states.csv")
                    self.assertEqual("1", states[-1]["estimate_available"])
                    self.assertEqual("1", states[-1]["diagnostic_only"])
                    self.assertNotEqual("2", states[-1]["trust_level"])
            scope = read(output / "structured_scope.csv")[0]
            self.assertEqual("1", scope[
                "gnss_cross_source_independence_assumed_not_proven"])
            self.assertTrue(all(d["gnss_map_construction"] == "present"
                                for d in read(output / "gnss.csv")))

    def test_adjacent_gnss_bracket_keeps_resolved_singular_family(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            flags = self.fixture(root, adjacent_gnss=True)
            output = root / "result"
            result = subprocess.run(
                [str(self.binary), str(root), str(output), *flags],
                capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            family = read(output / "structured_family.csv")[0]
            self.assertEqual(("99", "96"), (family["rows"], family["rank"]))

            def matrix(path):
                shape, values = path.read_text().strip().split(":")
                height, width = map(int, shape.split("x"))
                entries = list(map(float, values.split(";")))
                self.assertEqual(height * width, len(entries))
                return [entries[i * width:(i + 1) * width] for i in range(height)]

            covariance = matrix(output / "structured_family_covariance.csv")
            loading = matrix(output / "structured_family_root.csv")
            error = max(abs(sum(a * b for a, b in zip(loading[i], loading[j])) -
                            covariance[i][j]) /
                        (covariance[i][i] * covariance[j][j]) ** 0.5
                        for i in range(99) for j in range(99))
            self.assertLess(error, 1e-11)
            for arm in ("gnss", "all"):
                for mode in ("lag", "no_eviction"):
                    decisions = read(output / f"{arm}_{mode}_decisions.csv")
                    gnss = [d for d in decisions
                            if d["source"] == "raw-gga-horizontal-diagnostic"]
                    self.assertEqual(1, len(gnss))
                    self.assertEqual(("0", "1"),
                                     (gnss[0]["status"], gnss[0]["alignment_updated"]))

    def test_selection_schema_and_reused_query_identities_fail_before_replay(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            flags = self.fixture(root)
            for name, text, message in (
                    ("header", "timestamp_ns\n1\n2\n3\n", "invalid input/header"),
                    ("repeat", "scan_sequence\n1\n2\n2\n", "strictly increase"),
                    ("few", "scan_sequence\n1\n2\n", "3..16 selected scans")):
                with self.subTest(name=name):
                    (root / "selection.csv").write_text(text)
                    output = root / name
                    result = subprocess.run(
                        [str(self.binary), str(root), str(output), *flags],
                        capture_output=True, text=True)
                    self.assertNotEqual(0, result.returncode)
                    self.assertIn(message, result.stderr)
                    self.assertFalse((output / "odometry.csv").exists())


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: native_structured_closure_test.py NATIVE_LIO_BINARY")
    StructuredClosureTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
