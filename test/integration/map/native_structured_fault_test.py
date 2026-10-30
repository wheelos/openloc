#!/usr/bin/env python3
"""Focused frozen-family preparation and input-integrity regressions."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from native_structured_fault_experiment import check_pins, prepare, read


class FrozenFaultInputTest(unittest.TestCase):
    canonical = None
    binary = None

    def test_original_nodes_maps_and_single_recorded_gnss_are_preserved(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            output = Path(directory) / "bundle"
            manifest = prepare(self.canonical, output)
            self.assertEqual(1, manifest["recorded_usable_gnss"])
            self.assertEqual(
                (self.canonical / "replay/structured_nodes.csv").read_bytes(),
                (output / "nodes.csv").read_bytes())
            actual = read(self.canonical / "replay/structured_map.csv")
            prepared = read(output / "maps.csv")
            self.assertEqual([r["reference_ns"] for r in actual],
                             [r["timestamp_ns"] for r in prepared])
            self.assertEqual([r["px"] for r in actual],
                             [r["px"] for r in prepared])
            self.assertFalse(manifest["synthetic_gnss"][
                "recorded_receiver_loss_recovery"])
            self.assertEqual(manifest, check_pins(output))

    def test_changed_frozen_root_fails_pin_check_before_execution(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            output = Path(directory) / "bundle"
            prepare(self.canonical, output)
            with (output / "root.csv").open("a") as stream:
                stream.write("\n")
            with self.assertRaisesRegex(ValueError, "root.csv"):
                check_pins(output)

    def test_non_original_map_is_not_silently_admitted_as_actual_icp(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            canonical = root / "canonical"
            (canonical / "replay").mkdir(parents=True)
            for name in ("structured_nodes.csv", "structured_map.csv"):
                content = (self.canonical / "replay" / name).read_text()
                if name == "structured_map.csv":
                    content = content.replace(",NONE,", ",CORRELATION_UNQUALIFIED,", 1)
                (canonical / "replay" / name).write_text(content)
            with self.assertRaisesRegex(ValueError, "valid structured ICP"):
                prepare(canonical, root / "bundle")

    def test_native_loader_rejects_changed_root_and_reordered_nodes(self):
        if self.binary is None:
            self.skipTest("standalone binary not provided")
        for fault in ("root", "nodes"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory(
                    dir=Path.cwd()) as directory:
                root = Path(directory)
                bundle = root / "bundle"
                prepare(self.canonical, bundle)
                if fault == "root":
                    path = bundle / "root.csv"
                    prefix, entries = path.read_text().split(":", 1)
                    first, rest = entries.split(";", 1)
                    path.write_text(f"{prefix}:{float(first) + 1};{rest}")
                    expected = "does not reconstruct covariance"
                else:
                    path = bundle / "nodes.csv"
                    lines = path.read_text().splitlines()
                    path.write_text("\n".join([*lines, lines[1]]) + "\n")
                    expected = "original-time node order/bracket"
                result = subprocess.run(
                    [str(self.binary), str(bundle), str(root / "graphs")],
                    capture_output=True, text=True)
                self.assertNotEqual(0, result.returncode)
                self.assertIn(expected, result.stderr)
                self.assertFalse((root / "graphs/run_status.csv").exists())

    def test_receive_prefixes_do_not_submit_future_queries_or_factors(self):
        if self.binary is None:
            self.skipTest("standalone binary not provided")
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as directory:
            root = Path(directory)
            bundle = root / "bundle"
            prepare(self.canonical, bundle)
            output = root / "prefixes"
            result = subprocess.run(
                [str(self.binary), str(bundle), str(output), "--causal-prefixes"],
                capture_output=True, text=True)
            self.assertEqual(0, result.returncode, result.stderr)
            prefixes = read(output / "prefixes.csv")
            self.assertGreater(len(prefixes), 1)
            self.assertLess(int(prefixes[0]["rows"]), int(prefixes[-1]["rows"]))
            nodes, maps, gnss = (read(bundle / name) for name in
                                ("nodes.csv", "maps.csv", "gnss.csv"))
            for prefix in prefixes:
                watermark = int(prefix["watermark_ns"])
                available_nodes = {r["timestamp_ns"] for r in nodes
                                   if int(r["native_receive_ns"]) <= watermark}
                available_maps = [r for r in maps
                                  if int(r["receive_ns"]) <= watermark and
                                  r["timestamp_ns"] in available_nodes]
                available_gnss = [r for r in gnss
                                  if int(r["receive_ns"]) <= watermark and
                                  r["timestamp_ns"] in available_nodes]
                self.assertEqual(len(available_nodes), int(prefix["nodes"]))
                self.assertEqual(len(available_maps), int(prefix["maps"]))
                self.assertEqual(len(available_gnss), int(prefix["gnss"]))
                self.assertLess(float(prefix["root_error"]), 1e-11)
                for arm in ("lio", "gnss", "map", "all"):
                    for window in ("lag", "no_eviction"):
                        name = output / str(watermark) / f"{arm}_{window}"
                        decisions = read(Path(str(name) + "_decisions.csv"))
                        expected = available_maps if arm in ("map", "all") else available_maps[:2]
                        if arm in ("gnss", "all"):
                            expected = [*expected, *available_gnss]
                        self.assertCountEqual(
                            [(r["timestamp_ns"], r["receive_ns"]) for r in expected],
                            [(r["measurement_ns"], r["receive_ns"]) for r in decisions])
                        states = read(Path(str(name) + "_states.csv"))
                        self.assertEqual(available_nodes,
                                         {r["timestamp_ns"] for r in states})
                        self.assertTrue(all(r["diagnostic_only"] == "1"
                                            and r["trust_level"] != "2" for r in states))


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: native_structured_fault_test.py FROZEN_CANONICAL [FAULT_BINARY]")
    if len(sys.argv) == 3:
        FrozenFaultInputTest.binary = Path(sys.argv.pop()).resolve(strict=True)
    FrozenFaultInputTest.canonical = Path(sys.argv.pop()).resolve(strict=True)
    unittest.main()
