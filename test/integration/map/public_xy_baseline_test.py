"""Focused scoring/coverage regressions for the data baseline verifier."""

import unittest

from public_xy_baseline import clean_scores, coverage, held_out_indices


def event(stamp, *, valid="1", fresh="1", pending="0", tx="0"):
    return {
        "arrival_ns": str(stamp), "global_valid": valid, "source_fresh": fresh,
        "pending": pending, "backend_tx": tx, "backend_ty": "0", "backend_yaw": "0",
        "published_tx": tx, "published_ty": "0", "published_yaw": "0",
    }


class BaselineScoringContracts(unittest.TestCase):
    def test_clean_reference_keeps_missing_and_unmodified_epochs(self):
        rows = [event(10**9), event(2 * 10**9), event(3 * 10**9)]
        nominal = [
            {"timestamp_ns": str(index * 10**9), "ax": str(index), "ay": "0",
             "gx": str(index), "gy": "0", "gnss_present": "0" if index == 2 else "1"}
            for index in range(1, 4)
        ]
        odom = [{"timestamp_ns": row["timestamp_ns"]} for row in nominal]
        scores, metrics = clean_scores(rows, nominal, odom)
        self.assertEqual(len(scores), 3)
        self.assertEqual(metrics["all_epoch_published_median_p95_max_m"], [0, 0, 0])
        # Scoring is driven by the clean reference, not corrupted input fixes.
        corrupted = [dict(row) for row in nominal]
        corrupted[1]["gx"] = "1002"
        bad_scores, _ = clean_scores(rows, corrupted, odom)
        self.assertEqual(bad_scores[1]["published_clean_xy_m"], 1000)
        self.assertEqual(scores[1]["published_clean_xy_m"], 0)

    def test_coverage_weights_time_and_same_time_last_event(self):
        rows = [
            event(10**9, valid="0"), event(2 * 10**9, fresh="0"),
            event(2 * 10**9, pending="1"), event(4 * 10**9),
            event(5 * 10**9, fresh="0"), event(8 * 10**9, fresh="0"),
        ]
        self.assertEqual(coverage(rows), {
            "uninitialized": 1, "stale": 3, "transition": 2,
            "aligned": 1, "total": 7,
        })

    def test_seeded_odom_does_not_use_future_corrections_or_preinit(self):
        rows = [
            event(10**9, valid="0"), event(2 * 10**9, tx="2"),
            event(3 * 10**9, tx="4"),
        ]
        nominal = [
            {"timestamp_ns": str(index * 10**9), "ax": "0", "ay": "0",
             "gx": str(index * 2 - 2), "gy": "0"}
            for index in range(1, 4)
        ]
        scores, metrics = clean_scores(
            rows, nominal, [{"timestamp_ns": row["timestamp_ns"]} for row in nominal])
        self.assertEqual(scores[0]["seeded_odom_xy_m"], "")
        self.assertEqual(scores[2]["seeded_odom_xy_m"], 2)
        self.assertEqual(metrics["preinitialization_epoch_count"], 1)
        self.assertEqual(held_out_indices(42), [20, 30, 40])


if __name__ == "__main__":
    unittest.main()
