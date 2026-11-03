#!/usr/bin/env python3
"""Declared-model graph closure, not native covariance or physical accuracy."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from prepare_keyframe_graph_input import prepare
from run_keyframe_graph_ablation import run as run_ablation


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


def parse_matrix(value):
    shape, fields = value.split(":")
    height, width = map(int, shape.split("x"))
    values = list(map(float, fields.split(";")))
    if len(values) != height * width:
        raise ValueError("invalid test matrix")
    return [values[i * width:(i + 1) * width] for i in range(height)]


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

    def test_prepared_native_outputs_feed_the_same_causal_graph(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, measurements = self.fixture(root)
            offset = 1_791_882_563_000_000_001
            for node in nodes:
                node[0] += offset
            for row in measurements:
                row[1] += offset
                row[2] += offset
            write(root / "measurements.csv", HEADER, measurements)
            self.causal_nodes(root, nodes)
            control = self.run_causal(root)
            write(root / "odometry.csv",
                  "timestamp_ns,receive_ns,valid,px,py,pz,qx,qy,qz,qw",
                  [[node[0], node[0], 1, *node[1:]] for node in nodes])
            write(root / "lidar.csv", "reference_ns,bootstrap_stage",
                  [[node[0], 0] for node in nodes])
            write(root / "graph_measurements.csv", HEADER, measurements)
            bundle = root / "prepared"
            protocol = prepare(root, root / "model.csv", bundle, 1_000_000_000)
            self.assertFalse(protocol["native_model"])
            prepared = self.run_causal(bundle)
            for name in ("decisions.csv", "model_scope.csv"):
                self.assertEqual(read(control / name), read(prepared / name))
            expected, actual = read(control / "events.csv"), read(prepared / "events.csv")
            self.assertEqual(len(expected), len(actual))
            for a, b in zip(expected, actual):
                self.assertEqual({k: v for k, v in a.items() if k != "elapsed_ms"},
                                 {k: v for k, v in b.items() if k != "elapsed_ms"})

    def test_causal_ablation_records_same_prefix_controls_and_gate_divergence(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            self.causal_nodes(root, nodes)
            assessment = run_ablation(self.binary, root, root / "ablation", 2)
            for arm in ("lio", "gnss", "map", "all"):
                row = assessment[arm]
                self.assertTrue(row["same_admission_outcomes"])
                self.assertEqual(8, row["control_resources"]["graph_nodes"])
                self.assertLess(row["window_resources"]["graph_nodes"], 8)
                self.assertLess(row["maximum_translation_difference_m"], 1e-6)
                self.assertLess(row["maximum_rotation_difference_rad"], 1e-6)
                self.assertEqual(0, row["control_resources"]["archived_values"])
                self.assertEqual(23, row["window_timing"]["event_count"])
                self.assertGreaterEqual(row["window_timing"]["event_max_ms"],
                                        row["window_timing"]["event_p95_ms"])
            gnss = assessment["all"]["window_innovations"]["gnss"]
            self.assertEqual(7, gnss["available_samples"])
            self.assertEqual(0, gnss["rejected_available_samples"])
            rows = read(root / "measurements.csv")
            header = ",".join(rows[0])
            late = [list(row.values()) for row in rows]
            for row in late:
                if row[0] == "gnss" and int(row[1]) == nodes[1][0]:
                    row[2] = nodes[-1][0] + 200
            write(root / "measurements.csv", header, sorted(late, key=lambda row: int(row[2])))
            changed = run_ablation(self.binary, root, root / "late", 2)
            for arm in ("gnss", "all"):
                self.assertFalse(changed[arm]["same_admission_outcomes"])
                mismatch = changed[arm]["admission_mismatches"]
                self.assertEqual(1, len(mismatch))
                self.assertEqual("HISTORY_UNAVAILABLE", mismatch[0]["window_reason"])
                self.assertEqual("1", mismatch[0]["control_accepted"])

    def test_causal_source_loss_and_outlier_do_not_prevent_later_valid_factors(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, measurements = self.fixture(root)
            self.causal_nodes(root, nodes)
            kept = []
            for row in measurements:
                if row[0] == "gnss" and row[3] in (3, 4, 5):
                    continue
                if row[0] == "map" and row[3] == 4:
                    row[4] = 1e6
                kept.append(row)
            write(root / "measurements.csv", HEADER, kept)
            output = self.run_causal(root)
            decisions = read(output / "decisions.csv")
            maps = {int(row["sequence"]): row for row in decisions if row["source"] == "map"}
            self.assertEqual("INNOVATION_REJECTED", maps[4]["reason"])
            self.assertEqual("0", maps[4]["accepted"])
            self.assertTrue(all(maps[i]["accepted"] == "1" for i in (5, 6, 7, 8)))
            gnss = {int(row["sequence"]): row for row in decisions if row["source"] == "gnss"}
            self.assertEqual({2, 6, 7, 8}, set(gnss))
            self.assertTrue(all(row["accepted"] == "1" for row in gnss.values()))
            self.assertEqual("0", read(output / "model_scope.csv")[0]["trusted"])

    def test_causal_ablation_failure_retains_logs_without_success_assessment(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            self.causal_nodes(root, nodes)
            write(root / "model.csv",
                  "initial_variances,random_walk_variance_rates,"
                  "innovation_gate,robust_threshold",
                  [[matrix(15, 1, [-0.01] * 15),
                    matrix(15, 1, [0.01] * 15), 100, 1e6]])
            output = root / "ablation"
            with self.assertRaises(subprocess.CalledProcessError):
                run_ablation(self.binary, root, output, 2)
            self.assertIn("invalid causal random-walk model",
                          (output / "lio_window.log").read_text())
            self.assertFalse((output / "assessment.json").exists())
            self.assertFalse((output / "lio_control.log").exists())

    def test_finite_temporal_model_matches_joint_covariance_known_answer(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            for i, node in enumerate(nodes):
                node[6], node[7] = math.sin(i * 0.005), math.cos(i * 0.005)
            self.causal_nodes(root, nodes)
            write(root / "temporal_model.csv", "decay_rate_per_s", [[0.25]])
            output = self.run_causal(root)
            model = read(output / "motion_model.csv")
            self.assertEqual(7, len(model))
            prior = 0.01
            for row in model:
                decay = math.exp(-0.25)
                noise = 0.01 * (1 - math.exp(-0.5)) / 0.5
                variance = decay ** 2 * prior + noise
                transition = parse_matrix(row["transition"])
                process = parse_matrix(row["process_covariance"])
                covariance = parse_matrix(row["state_covariance"])
                cross = parse_matrix(row["previous_state_cross_covariance"])
                rotation = [[math.cos(0.01), math.sin(0.01), 0],
                            [-math.sin(0.01), math.cos(0.01), 0], [0, 0, 1]]
                for i in range(15):
                    for j in range(15):
                        expected = float(i == j)
                        if 6 <= i < 9 and 6 <= j < 9:
                            expected = rotation[i - 6][j - 6]
                        self.assertAlmostEqual(decay * expected, transition[i][j], places=12)
                        self.assertAlmostEqual(noise * (i == j), process[i][j], places=12)
                        self.assertAlmostEqual(variance * (i == j), covariance[i][j], places=12)
                        self.assertAlmostEqual(prior * transition[j][i], cross[i][j], places=12)
                prior = variance
            self.assertEqual("closed-frontier-keyframe-gauss-markov",
                             read(output / "model_scope.csv")[0]["model"])
            self.assertEqual("0", read(output / "model_scope.csv")[0]["trusted"])
            assessment = run_ablation(self.binary, root, root / "ablation", 2)
            self.assertTrue(all(assessment[arm]["same_admission_outcomes"]
                                for arm in ("lio", "gnss", "map", "all")))
            self.assertTrue((root / "ablation/input/temporal_model.csv").is_file())

    def test_temporal_model_zero_decay_limit_and_invalid_rate(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temporary:
            root = Path(temporary)
            nodes, _ = self.fixture(root)
            self.causal_nodes(root, nodes)
            baseline = read(self.run_causal(root) / "motion_model.csv")
            write(root / "temporal_model.csv", "decay_rate_per_s", [[1e-15]])
            limiting = read(self.run_causal(root, "limit") / "motion_model.csv")
            unsupported = subprocess.run(
                [str(self.binary), str(root), str(root / "noncausal"), "all", "2",
                 "--experimental-independent-keyframes"],
                capture_output=True, text=True)
            self.assertNotEqual(0, unsupported.returncode)
            self.assertIn("requires causal", unsupported.stderr)
            self.assertFalse((root / "noncausal").exists())
            for a, b in zip(baseline, limiting):
                for key in ("transition", "process_covariance", "state_covariance",
                            "previous_state_cross_covariance"):
                    for x, y in zip(parse_matrix(a[key]), parse_matrix(b[key])):
                        for v, w in zip(x, y):
                            self.assertAlmostEqual(v, w, places=12)
            write(root / "temporal_model.csv", "decay_rate_per_s", [[-1]])
            with self.assertRaises(subprocess.CalledProcessError):
                run_ablation(self.binary, root, root / "invalid", 2)
            self.assertFalse((root / "invalid/assessment.json").exists())

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
            write(root / "temporal_model.csv", "decay_rate_per_s", [[0.25]])
            temporal = read(self.run_causal(root, "temporal", 0.3) / "events.csv")
            self.assertEqual(len(events), len(temporal))
            self.assertTrue(all(int(row["graph_nodes"]) <= 5 for row in temporal))
            self.assertTrue(all(row["frontier_dimension"] == "15" for row in temporal))
            self.assertTrue(all(row["archived_values"] == "0" for row in temporal))
            first = max(int(row["factor_slots"]) for row in temporal[100:320])
            second = max(int(row["factor_slots"]) for row in temporal[320:])
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
