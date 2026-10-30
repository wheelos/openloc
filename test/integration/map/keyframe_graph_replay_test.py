#!/usr/bin/env python3
"""Declared-model graph closure, not native covariance or physical accuracy."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


HEADER = ("kind,timestamp_ns,receive_ns,sequence,px,py,pz,qx,qy,qz,qw,"
          "projection,covariance,lever_x,lever_y,lever_z,ambiguous")


def matrix(rows, columns, values):
    return f"{rows}x{columns}:" + ";".join(str(value) for value in values)


def diagonal(size, value):
    return matrix(size, size, [
        value if row == column else 0
        for row in range(size) for column in range(size)])


def write(path, header, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header.split(","))
        writer.writerows(rows)


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


class KeyframeGraphReplayTest(unittest.TestCase):
    def fixture(self, root):
        nodes = [[(i + 1) * 1_000_000_000, 0.1 * i, 0, 0, 0, 0, 0, 1]
                 for i in range(8)]
        write(root / "nodes.csv", "timestamp_ns,px,py,pz,qx,qy,qz,qw", nodes)
        write(root / "model.csv",
              "initial_variances,random_walk_variance_rates,"
              "innovation_gate,robust_threshold",
              [[matrix(15, 1, [0.01] * 15),
                matrix(15, 1, [0.01] * 15), 100, 1e6]])
        measurements = []
        for i, node in enumerate(nodes):
            stamp = node[0]
            measurements.append(
                ["map", stamp, stamp + 100, i + 1, 0, 0, 0, 0, 0, 0, 1,
                 diagonal(6, 1), diagonal(6, 0.001), 0, 0, 0, 0])
            if i:
                # The antenna is 0.5 m forward of the base. These observations
                # should bring the BASE to zero, not to the antenna position.
                measurements.append(
                    ["gnss", stamp, stamp + 100, i + 1,
                     0.5, 0, 0, 0, 0, 0, 1,
                     diagonal(2, 1), diagonal(2, 0.0025), 0.5, 0, 0, 0])
        write(root / "measurements.csv", HEADER, measurements)
        return nodes, measurements

    def run_arm(self, root, arm, lag=2):
        output = root / f"{arm}{lag}"
        result = subprocess.run(
            [str(self.binary), str(root), str(output), arm, str(lag),
             "--experimental-independent-keyframes"],
            capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)
        return output

    def test_bottom_up_additions_and_same_factor_batch(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            errors = {}
            for arm in ("lio", "gnss", "map", "all"):
                output = self.run_arm(root, arm)
                states = read(output / "states.csv")
                self.assertEqual([str(node[0]) for node in nodes],
                                 [row["timestamp_ns"] for row in states])
                errors[arm] = abs(float(states[-1]["px"]))
                comparison = read(output / "comparison.csv")[0]
                self.assertLess(int(comparison["retained_nodes"]), len(nodes))
                self.assertLess(float(comparison["batch_exact_residual"]), 1e-8)
                self.assertLess(float(comparison["retained_exact_residual"]), 1e-8)
                self.assertLess(float(comparison["translation_difference_m"]), 1e-6,
                                arm)
                decisions = read(output / "decisions.csv")
                self.assertTrue(all(row["accepted"] == "1" for row in decisions))
                factors = read(output / "factors.csv")
                self.assertTrue(any(row["scope"] == "batch" for row in factors))
                scope = read(output / "model_scope.csv")[0]
                self.assertEqual("0", scope["native_model"])
                self.assertEqual("0", scope["trusted"])
                self.assertEqual("1", scope["source_correlation_unmodeled"])
            self.assertAlmostEqual(0.7, errors["lio"], places=8)
            self.assertLess(errors["gnss"], 0.1)
            self.assertLess(errors["map"], errors["gnss"])
            self.assertLess(errors["all"], errors["lio"])
            full = self.run_arm(root, "all", 100)
            comparison = read(full / "comparison.csv")[0]
            self.assertEqual(8, int(comparison["retained_nodes"]))
            self.assertLess(float(comparison["translation_difference_m"]), 1e-6)

    def test_missing_model_opt_in_rejects_before_output(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            output = root / "output"
            for flag in (None, "--diagnostic-only"):
                arguments = [str(self.binary), str(root), str(output), "all", "2"]
                if flag:
                    arguments.append(flag)
                result = subprocess.run(arguments, capture_output=True, text=True)
                self.assertNotEqual(0, result.returncode)
                self.assertIn("NOT a native model qualification", result.stderr)
                self.assertFalse(output.exists())

    def test_projected_map_does_not_invent_vertical_observations(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, measurements = self.fixture(root)
            for i, node in enumerate(nodes):
                node[1] = 0
                node[3] = 0.1 * i
            write(root / "nodes.csv", "timestamp_ns,px,py,pz,qx,qy,qz,qw", nodes)
            kept = [row for row in measurements if row[0] == "map"]
            for measurement in kept[1:]:
                measurement[0] = "projected_map"
                measurement[11] = matrix(2, 6, [1, 0, 0, 0, 0, 0,
                                                0, 1, 0, 0, 0, 0])
                measurement[12] = diagonal(2, 0.001)
            write(root / "measurements.csv", HEADER, kept)
            states = read(self.run_arm(root, "map") / "states.csv")
            self.assertAlmostEqual(0.7, float(states[-1]["pz"]), places=7)
            self.assertLess(abs(float(states[-1]["px"])), 0.1)

    def test_wrong_times_duplicate_sources_and_ambiguity_reject(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            _, measurements = self.fixture(root)
            for mutation, message in (
                ("off_node", "exact node"),
                ("future", "causal reception"),
                ("duplicate", "duplicate measurement identity"),
                ("ambiguous", "ambiguous observations"),
            ):
                changed = [row.copy() for row in measurements]
                if mutation == "off_node":
                    changed[-1][1] += 1
                elif mutation == "future":
                    changed[-1][2] = changed[-1][1] - 1
                elif mutation == "duplicate":
                    changed.append(changed[-1].copy())
                else:
                    changed[-1][-1] = 1
                write(root / "measurements.csv", HEADER, changed)
                output = root / mutation
                result = subprocess.run(
                    [str(self.binary), str(root), str(output), "all", "2",
                     "--experimental-independent-keyframes"],
                    capture_output=True, text=True)
                self.assertNotEqual(0, result.returncode)
                self.assertIn(message, result.stderr)
                self.assertFalse(output.exists())

    def run_causal(self, root, name="causal", lag=2):
        output = root / name
        result = subprocess.run(
            [str(self.binary), str(root), str(output), "all", str(lag),
             "--experimental-independent-keyframes", "--causal-receive-order"],
            capture_output=True, text=True)
        self.assertEqual(0, result.returncode, result.stderr)
        return output

    def causal_nodes(self, root, nodes, delays=None):
        write(root / "nodes.csv",
              "timestamp_ns,px,py,pz,qx,qy,qz,qw,receive_ns",
              [node + [node[0] + (delays or {}).get(node[0], 0)] for node in nodes])

    def test_causal_incremental_matches_same_order_gaussian_control(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            control = read(self.run_arm(root, "all") / "states.csv")
            self.causal_nodes(root, nodes)
            output = self.run_causal(root)
            events = read(output / "events.csv")
            final = {event["latest_measurement_ns"]: event for event in events}
            for state in control:
                for axis in ("px", "py", "pz", "qx", "qy", "qz", "qw"):
                    self.assertAlmostEqual(float(state[axis]),
                        float(final[state["timestamp_ns"]][axis]), places=6)
            self.assertTrue(all(row["archived_values"] == "0" for row in events))
            self.assertTrue(all(int(row["frontier_dimension"]) == 15 for row in events))
            self.assertEqual("0", read(output / "model_scope.csv")[0]["native_model"])

    def test_causal_late_missing_and_retired_nodes_are_not_retargeted(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, measurements = self.fixture(root)
            # Node 2 has not arrived when its GNSS measurement arrives. Node 3's
            # GNSS arrives late but within the window; node 4's arrives too late.
            second = nodes[1][0]
            self.causal_nodes(root, nodes, {second: 1000})
            for row in measurements:
                if row[0] == "gnss" and row[1] == nodes[2][0]:
                    row[2] = nodes[3][0] + 200
                if row[0] == "gnss" and row[1] == nodes[3][0]:
                    row[2] = nodes[-1][0] + 200
            write(root / "measurements.csv", HEADER,
                  sorted(measurements, key=lambda row: row[2]))
            output = self.run_causal(root)
            decisions = {int(row["sequence"]): row for row in
                         read(output / "decisions.csv") if row["source"] == "gnss"}
            self.assertEqual("HISTORY_UNAVAILABLE", decisions[2]["reason"])
            self.assertEqual("0", decisions[2]["accepted"])
            self.assertEqual("1", decisions[3]["accepted"])
            self.assertEqual(str(nodes[2][0]), decisions[3]["timestamp_ns"])
            self.assertEqual("HISTORY_UNAVAILABLE", decisions[4]["reason"])
            events = read(output / "events.csv")
            received = [int(row["receive_ns"]) for row in events]
            self.assertEqual(sorted(received), received)

    def test_causal_repeated_eviction_has_bounded_frontier_and_no_archive(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            _, measurements = self.fixture(root)
            nodes = [[1_000_000_000 + i * 100_000_000,
                      0, 0, 0, 0, 0, 0, 1] for i in range(320)]
            self.causal_nodes(root, nodes)
            template = measurements[0]
            observations = []
            for i, node in enumerate(nodes):
                row = template.copy()
                row[1], row[2], row[3] = node[0], node[0] + 100, i + 1
                observations.append(row)
            write(root / "measurements.csv", HEADER, observations)
            events = read(self.run_causal(root, lag=0.3) / "events.csv")
            self.assertEqual(640, len(events))
            self.assertTrue(all(int(row["buffered_nodes"]) <= 5 for row in events))
            self.assertTrue(all(int(row["graph_nodes"]) <= 5 for row in events))
            self.assertTrue(all(row["archived_values"] == "0" for row in events))
            self.assertTrue(all(row["frontier_rows"] == "15" for row in events))
            first = max(int(row["factor_slots"]) for row in events[100:320])
            second = max(int(row["factor_slots"]) for row in events[320:])
            self.assertLessEqual(second, first + 2)

    def test_horizontal_fixes_preserve_feasibility_in_weak_vertical_direction(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, measurements = [], []
            for i in range(240):
                time = i * 0.1
                stamp = 1_000_000_000 + i * 100_000_000
                x, y, z = (30 * time, 2 * math.sin(time / 4),
                           0.1 * math.sin(time / 5))
                nodes.append([stamp, x, y, z, 0, 0, 0, 1])
                if i == 0:
                    measurements.append(
                        ["map", stamp, stamp + 100, 1, x, y, z, 0, 0, 0, 1,
                         diagonal(6, 1), diagonal(6, 0.01), 0, 0, 0, 0])
                elif i % 10 == 0:
                    # A smooth horizontal bias exercises weak-axis curvature,
                    # not a physical-accuracy or covariance-calibration claim.
                    measurements.append(
                        ["gnss", stamp, stamp + 100, i + 1,
                         x + 0.1 * math.sin(time),
                         y + 2 * (1 - math.exp(-time / 10)), 0, 0, 0, 0, 1,
                         diagonal(2, 1), diagonal(2, 1), 0, 0, 0, 0])
            self.causal_nodes(root, nodes)
            variances = [0.01] * 3 + [0.0001] * 6 + [1e-6] * 6
            write(root / "model.csv",
                  "initial_variances,random_walk_variance_rates,"
                  "innovation_gate,robust_threshold",
                  [[matrix(15, 1, variances), matrix(15, 1, variances),
                    22.458, 2.5]])
            write(root / "measurements.csv", HEADER, measurements)
            output = root / "gnss"
            result = subprocess.run(
                [str(self.binary), str(root), str(output), "gnss", "3",
                 "--experimental-independent-keyframes", "--causal-receive-order"],
                capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            decisions = read(output / "decisions.csv")
            self.assertEqual(len(measurements), len(decisions))
            self.assertTrue(all(row["accepted"] == "1" for row in decisions))
            events = read(output / "events.csv")
            self.assertEqual(240, sum(row["kind"] == "node" for row in events))
            self.assertTrue(all(row["archived_values"] == "0" for row in events))
            self.assertTrue(any(int(row["damped_iterations"]) > 0 for row in events))
            self.assertTrue(all(int(row["solve_iterations"]) <= 5 for row in events))
            self.assertTrue(all(float(row["maximum_motion_residual"]) <= 1e-6
                                for row in events))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: keyframe_graph_replay_test.py BINARY")
    KeyframeGraphReplayTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
