#!/usr/bin/env python3
"""Focused report semantics; no estimator replay or external dependencies."""

import math
import unittest

from compare_native_lio_baselines import (
    antenna, display_trajectories, distribution, event_ledger, fixed_anchor,
    nearest_causal, select_revision,
)


class ComparisonTest(unittest.TestCase):
    def test_source_epoch_never_uses_future_or_outside_tolerance(self):
        self.assertEqual(nearest_causal([100, 110, 125], 119, 10), 110)
        self.assertIsNone(nearest_causal([100, 125], 119, 10))
        self.assertIsNone(nearest_causal([125], 119, 10))

    def test_revisions_require_emission_and_source_causality(self):
        first = dict(timestamp_ns="100", receive_ns="105",
                     alignment_initialized="1", correction_id="1")
        later = dict(first, receive_ns="120", correction_id="2")
        future_source = dict(first, receive_ns="108", correction_id="3")
        self.assertEqual(select_revision([first, later, future_source], 100, 110,
                                         {1: 90, 2: 99, 3: 101}), first)
        self.assertEqual(select_revision([first, later], 100, 125,
                                         {1: 90, 2: 99}), later)
        self.assertIsNone(select_revision([first], 100, 104, {1: 90}))

    def test_lever_arm_rotates_before_single_anchor_transform(self):
        local = dict(px="1", py="2", pz="3", qx="0", qy="0", qz="0", qw="1")
        target = dict(x="10", y="20", z="30", qx="0", qy="0",
                      qz=str(math.sqrt(.5)), qw=str(math.sqrt(.5)))
        result = fixed_anchor(local, target)(antenna(local, (2, 0, 0), local=True))
        for actual, expected in zip(result, (10, 22, 30)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(antenna(target, (2, 0, 0)), result):
            self.assertAlmostEqual(actual, expected)

    def test_missing_coverage_is_not_zero_error(self):
        self.assertIsNone(distribution([])["rmse_m"])
        self.assertEqual(distribution([1, 3])["p95_m"], 2.9)
        self.assertAlmostEqual(distribution([1, 3])["rmse_m"], math.sqrt(5))

    def test_source_free_arm_has_no_global_decision_journal(self):
        self.assertEqual(event_ledger({"odometry": []}, "odom"), [])

    def test_retrospective_display_collapses_duplicates_without_changing_causal_selection(self):
        local = dict(timestamp_ns="100", receive_ns="105", valid="1",
                     px="0", py="0", pz="0", qx="0", qy="0", qz="0", qw="1")
        first = dict(local, alignment_initialized="1", correction_id="1",
                     x="1", y="0", z="0")
        retrospective = dict(first, receive_ns="140", correction_id="2", x="9")
        arms = dict(odom={"odometry": [local]},
                    map={"global": [first, retrospective]},
                    both={"global": [first]}, gnss={})
        display = display_trajectories(arms, (0, 0, 0), lambda p: p)
        mapped = [r for r in display if r["arm"] == "map"]
        self.assertEqual(len(mapped), 1)
        self.assertEqual(mapped[0]["x"], 9)
        self.assertEqual(select_revision([first, retrospective], 100, 110,
                                         {1: 90, 2: 99})["x"], "1")


if __name__ == "__main__":
    unittest.main()
