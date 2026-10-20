#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0

"""Focused geometry regressions for the Boreas scan preparation adapter."""

import math
import unittest

import numpy as np

import boreas_openloc_baseline as baseline


class BoreasOpenLocBaselineTest(unittest.TestCase):
    def test_pose_at_interpolates_translation_and_heading(self):
        poses = np.array(
            (
                (0, 0, 0, 0, 0, 0, 0),
                (10, 10, 20, 30, 0, 0, math.pi / 2),
            ),
            dtype=np.float64,
        )

        translation, quaternion = baseline.pose_at(poses, np.array((5,)))

        np.testing.assert_allclose(translation[0], (5, 10, 15))
        expected = baseline.quaternion_from_rpy(0, 0, math.pi / 4)
        np.testing.assert_allclose(quaternion[0], expected, atol=1e-12)

    def test_rotate_points_uses_xyzw_quaternion(self):
        quaternion = baseline.quaternion_from_rpy(
            np.array(0.0), np.array(0.0), np.array(math.pi / 2)
        )

        rotated = baseline.rotate_points(
            quaternion, np.array((1.0, 0.0, 0.0))
        )

        np.testing.assert_allclose(rotated, (0.0, 1.0, 0.0), atol=1e-12)

    def test_map_bias_is_applied_in_map_frame(self):
        translation, quaternion = baseline.apply_map_bias(
            np.array((10.0, 20.0, 30.0)),
            baseline.quaternion_from_rpy(
                np.array(0.0), np.array(0.0), np.array(0.0)
            ),
        )

        np.testing.assert_allclose(translation, (11.0, 20.0, 30.0))
        np.testing.assert_allclose(
            quaternion,
            baseline.quaternion_from_rpy(
                np.array(0.0),
                np.array(0.0),
                np.array(baseline.BIAS_YAW_RAD),
            ),
        )

    def test_pose_interpolation_rejects_extrapolation(self):
        poses = np.array(
            ((1, 0, 0, 0, 0, 0, 0), (2, 0, 0, 0, 0, 0, 0)),
            dtype=np.float64,
        )

        with self.assertRaisesRegex(ValueError, "outside the pose table"):
            baseline.pose_at(poses, np.array((0,)))

    def test_voxel_centroids_average_points_in_same_cell(self):
        points = np.array(
            ((0.1, 0.1, 0.1), (0.2, 0.2, 0.2), (0.8, 0.8, 0.8))
        )

        centroids = baseline.voxel_centroids(points, 0.5)

        self.assertEqual(len(centroids), 2)
        np.testing.assert_allclose(centroids[0], (0.15, 0.15, 0.15))
        np.testing.assert_allclose(centroids[1], (0.8, 0.8, 0.8))


if __name__ == "__main__":
    unittest.main()
