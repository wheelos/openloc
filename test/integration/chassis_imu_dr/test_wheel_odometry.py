import csv
import math
import tempfile
import unittest
from pathlib import Path

from wheel_odometry import Series, advance, eligible_intervals, replay


class WheelOdometryTest(unittest.TestCase):
    def test_straight_and_constant_turn(self):
        self.assertEqual(advance(0, 0, 0, 2, 0, 3), (6, 0, 0))
        x, y, yaw = advance(0, 0, 0, 2, 0.5, math.pi)
        self.assertAlmostEqual(x, 4)
        self.assertAlmostEqual(y, 4)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_gap_and_bounds_are_not_extrapolated(self):
        series = Series([0, 0.01, 0.2], [[1], [2], [3]], 0.05)
        self.assertIsNone(series.at(-0.01))
        self.assertIsNone(series.at(0.1))
        self.assertIsNone(series.at(0.21))
        self.assertEqual(series.at(0.005), [1.5])

    def test_heading_interpolation_crosses_wrap(self):
        series = Series([0, 0.01], [[3.13], [-3.13]], 0.05)
        self.assertAlmostEqual(series.at(0.005, 0)[0], math.pi)

    def test_invalid_times_rejected(self):
        with self.assertRaises(ValueError):
            Series([0, 0], [[1], [2]], 0.05)

    def test_shared_replay_grid_rejects_unbracketed_interiors(self):
        speed = Series([0, 0.01, 0.02], [[1], [1], [1]], 0.05)
        gyro = Series([0, 0.01], [[0], [0]], 0.05)
        first, last = list(eligible_intervals(speed, gyro, 0.05))
        self.assertEqual(first[2], [0, 0.005, 0.01])
        self.assertEqual(len(first[3]), 3)
        self.assertEqual(last[3], [])

    def test_reference_after_seed_does_not_feed_back(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)

            def write(name, fields, rows):
                path = root / name
                with path.open("w") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(fields)
                    writer.writerows(rows)
                return path

            times = [1.0, 1.01, 1.02, 1.03, 1.04]
            chassis = write(
                "chassis.csv", ["header_time", "speed", "steering_percent"],
                [[t, 2, 5] for t in times],
            )
            imu = write(
                "imu.csv", ["measurement_time", "gz"],
                [[t - 315964782.0, 0.1] for t in times],
            )
            fields = ["measurement_time", "px", "py", "heading"]
            first = write(
                "pose.csv", fields, [[t, 2 * (t - 1), 0, 0] for t in times],
            )
            changed = write(
                "changed.csv", fields,
                [[1, 0, 0, 0]] + [[t, 100, 200, 1] for t in times[1:]],
            )
            replay(chassis, imu, first, 0.2, root / "first")
            replay(chassis, imu, changed, 0.2, root / "changed")
            for mode in ("wheel_raw_gyro", "wheel_transferred_steering"):
                with (root / "first" / f"{mode}.csv").open() as stream:
                    original = list(csv.DictReader(stream))
                with (root / "changed" / f"{mode}.csv").open() as stream:
                    modified = list(csv.DictReader(stream))
                self.assertEqual(len(original), len(modified))
                for left, right in zip(original, modified):
                    for key in ("local_x", "local_y", "yaw"):
                        self.assertEqual(left[key], right[key])
                distances = [float(row["wheel_distance_m"]) for row in original]
                self.assertEqual(distances, sorted(distances))
                self.assertAlmostEqual(distances[-1], 0.08)


if __name__ == "__main__":
    unittest.main()
