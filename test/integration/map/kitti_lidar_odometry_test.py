#!/usr/bin/env python3
"""Focused report regressions for the bounded KITTI LiDAR diagnostic."""

import unittest

import run_kitti_lidar_odometry as diagnostic


def row(frame, status, estimate_x, reference_x):
    value = {
        "frame": str(frame),
        "timestamp_s": str(frame / 10),
        "status": status,
        "fitness": "0",
        "valid_points": "1",
        "filtered_points": "1",
    }
    for prefix, x in (("estimate", estimate_x), ("reference", reference_x)):
        matrix = [1, 0, 0, x, 0, 1, 0, 0, 0, 0, 1, 0]
        value.update({
            f"{prefix}_{r}{c}": str(matrix[r * 4 + c])
            for r in range(3) for c in range(4)
        })
    return value


class KittiLidarOdometryReportTest(unittest.TestCase):
    def test_fixed_anchor_errors_and_path_lengths(self):
        rows = [
            row(0, "ANCHOR", 0, 0),
            row(1, "ICP_CONVERGED", 1.0, 1.1),
        ]

        report, completed = diagnostic.summarize(rows)

        self.assertEqual(report["status"], "complete")
        self.assertEqual(report["completed_scans"], 2)
        self.assertAlmostEqual(report["translation_error_m"]["rmse"], 0.1 / 2**0.5)
        self.assertAlmostEqual(report["estimated_path_length_m"], 1.0)
        self.assertAlmostEqual(report["reference_path_length_m"], 1.1)
        self.assertEqual(len(completed), 2)

    def test_failed_registration_is_reported_without_scoring_empty_pose(self):
        rows = [
            row(0, "ANCHOR", 0, 0),
            {
                "frame": "1",
                "timestamp_s": "0.1",
                "status": "ICP_REJECTED",
                "estimate_00": "",
            },
        ]

        report, completed = diagnostic.summarize(rows)

        self.assertEqual(report["status"], "failed_registration")
        self.assertEqual(report["first_failed_frame"], 1)
        self.assertEqual(report["completed_scans"], 1)
        self.assertEqual(len(completed), 1)

    def test_visualization_keeps_xy_equal_scale_view(self):
        svg = diagnostic.svg_trajectory([
            row(0, "ANCHOR", 0, 0),
            row(1, "ICP_CONVERGED", 1, 1),
        ])

        self.assertIn("viewBox=", svg)
        self.assertIn("Equal XY scale", svg)
        self.assertIn("Green: reference", svg)


if __name__ == "__main__":
    unittest.main()
