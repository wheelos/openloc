import subprocess
import tempfile
import unittest
from pathlib import Path

from apollo_ekf_diagnostic import make_seed
from compare_planar_replay import write_csv
from wheel_odometry import load


class DiagnosticInitializationTest(unittest.TestCase):
    runner = Path("bazel-bin/local_estimator_dataset_replay")

    def events(self, path, seconds, speed=0, gyro=0):
        rows = []
        for index in range(int(seconds * 100) + 1):
            time = 1000 + index / 100
            for kind in ("W", "I"):
                rows.append({
                    "kind": kind, "measurement_time": time, "receive_time": time,
                    "sequence": index + 1, "ax": 0, "ay": 0, "az": 9.80665,
                    "gx": 0, "gy": 0, "gz": gyro, "speed_mps": speed,
                })
        write_csv(path, list(rows[0]), rows)

    def run_replay(self, root, name, *options):
        output = root / f"{name}.csv"
        subprocess.run(
            [str(self.runner), "--events", str(root / "events.csv"),
             "--trajectory", str(output), "--wheel-variance", "0.01",
             "--use-nhc", "false", *options],
            check=True, capture_output=True, text=True,
        )
        return load(output)

    def test_diagnostic_dwell_does_not_change_default(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.events(root / "events.csv", 3)
            default = self.run_replay(root, "default")
            shorter = self.run_replay(root, "shorter", "--diagnostic-stationary-duration", "1")
            first_default = float(next(r for r in default if r["valid"] == "1")["time"])
            first_shorter = float(next(r for r in shorter if r["valid"] == "1")["time"])
            self.assertEqual(first_default, 1002)
            self.assertEqual(first_shorter, 1001)

    def test_explicit_seed_sets_speed_and_bias_without_position_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.events(root / "events.csv", 1, speed=2, gyro=0.1)
            seed = {
                "qw": 1, "qx": 0, "qy": 0, "qz": 0,
                **{f"gyro_bias_{axis}": 0.1 if axis == "z" else 0 for axis in "xyz"},
                **{f"accel_bias_{axis}": 0 for axis in "xyz"},
                **{f"cov_{i}": 1e-4 for i in range(15)},
            }
            make_seed(seed, 2, root / "seed.csv")
            states = self.run_replay(root, "seeded", "--offline-diagnostic-seed", str(root / "seed.csv"))
            self.assertTrue(all(r["valid"] == "1" for r in states))
            self.assertEqual(float(states[0]["px"]), 0)
            self.assertEqual(float(states[0]["vx"]), 2)
            self.assertAlmostEqual(float(states[-1]["px"]), 2, places=9)
            self.assertAlmostEqual(float(states[-1]["py"]), 0, places=9)
            self.assertAlmostEqual(float(states[-1]["gyro_bias_z"]), 0.1, places=9)
            self.assertAlmostEqual(float(states[-1]["qw"]), 1, places=9)


if __name__ == "__main__":
    unittest.main()
