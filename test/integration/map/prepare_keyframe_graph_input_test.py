#!/usr/bin/env python3
"""Original-time and dependency-boundary checks for keyframe preparation."""

import json
import math
from pathlib import Path
import tempfile
import unittest

from prepare_keyframe_graph_input import MEASUREMENTS, prepare, read, write


class PrepareKeyframeGraphInputTest(unittest.TestCase):
    def fixture(self, root):
        stamp = 1_791_882_563_000_000_001
        poses = [[stamp, stamp + 10, 1, 0, 0, 0, 0, 0, 0, 1],
                 [stamp + 100, stamp + 150, 1, 2, 0, 0, 0, 0, 1, 0]]
        write(root / "odometry.csv",
              "timestamp_ns,receive_ns,valid,px,py,pz,qx,qy,qz,qw", poses)
        write(root / "lidar.csv", "reference_ns,bootstrap_stage",
              [[stamp, 0], [stamp + 100, 0]])
        observations = [
            ["map", stamp, stamp + 20, 1, 0, 0, 0, 0, 0, 0, 1,
             "6x6:1", "6x6:1", 0, 0, 0, 0],
            ["gnss", stamp + 25, stamp + 30, 2, 1, 0, 0, 0, 0, 0, 1,
             "2x2:1", "2x2:1", .5, 0, 0, 0],
        ]
        write(root / "graph_measurements.csv", MEASUREMENTS, observations)
        model = root / "declared_model.csv"
        model.write_text("caller-supplied-model\n")
        return stamp, poses, observations, model

    def test_integer_brackets_and_causal_availability_preserve_measurement(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stamp, _, observations, model = self.fixture(root)
            output = root / "bundle"
            protocol = prepare(root, model, output, 100)
            nodes = read(output / "nodes.csv")
            self.assertEqual([stamp, stamp + 25, stamp + 100],
                             [int(row["timestamp_ns"]) for row in nodes])
            middle = nodes[1]
            self.assertEqual(stamp + 150, int(middle["receive_ns"]))
            self.assertAlmostEqual(.5, float(middle["px"]))
            self.assertAlmostEqual(math.sin(math.pi / 8), float(middle["qz"]))
            self.assertAlmostEqual(math.cos(math.pi / 8), float(middle["qw"]))
            prepared = read(output / "measurements.csv")[1]
            self.assertEqual(str(stamp + 25), prepared["timestamp_ns"])
            self.assertEqual(str(stamp + 30), prepared["receive_ns"])
            self.assertEqual(str(observations[1][13]), prepared["lever_x"])
            self.assertFalse(protocol["native_model"])
            self.assertEqual(model.read_bytes(), (output / "model.csv").read_bytes())

    def test_invalid_tail_and_interpolation_gap_fail_before_output(self):
        for invalid_tail in (False, True):
            with tempfile.TemporaryDirectory() as directory, self.subTest(
                invalid_tail=invalid_tail
            ):
                root = Path(directory)
                _, poses, _, model = self.fixture(root)
                if invalid_tail:
                    poses[-1][2] = 0
                    write(root / "odometry.csv",
                          "timestamp_ns,receive_ns,valid,px,py,pz,qx,qy,qz,qw", poses)
                output = root / "bundle"
                with self.assertRaisesRegex(ValueError, "segment|interpolation gap"):
                    prepare(root, model, output, 50)
                self.assertFalse(output.exists())

    def test_exclusions_are_accounted_and_missing_anchor_is_not_invented(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stamp, _, observations, model = self.fixture(root)
            outside = observations[1].copy()
            outside[1] = stamp - 1
            ambiguous = observations[1].copy()
            ambiguous[-1] = 1
            write(root / "graph_measurements.csv", MEASUREMENTS,
                  [outside, *observations, ambiguous])
            output = root / "bundle"
            prepare(root, model, output, 100)
            protocol = json.loads((output / "protocol.json").read_text())
            self.assertEqual(["outside_valid_native_support", "ambiguous"],
                             [row["reason"] for row in protocol["excluded_measurements"]])
            write(root / "graph_measurements.csv", MEASUREMENTS, observations[1:])
            with self.assertRaisesRegex(ValueError, "full-pose map anchor"):
                prepare(root, model, root / "missing_anchor", 100)
            self.assertFalse((root / "missing_anchor").exists())

    def test_native_keyframes_before_the_first_map_are_retained(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stamp, _, observations, model = self.fixture(root)
            observations[0][1] = stamp + 100
            observations[0][2] = stamp + 160
            write(root / "graph_measurements.csv", MEASUREMENTS, observations)
            output = root / "bundle"
            protocol = prepare(root, model, output, 100)
            self.assertEqual(stamp, int(read(output / "nodes.csv")[0]["timestamp_ns"]))
            self.assertEqual(stamp + 100, protocol["common_full_pose_map_anchor"])


if __name__ == "__main__":
    unittest.main()
