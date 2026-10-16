"""Test-only diagnostic CSV contracts; run with the built runner as argv[1]."""

import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


RUNNER = Path(sys.argv.pop(1)).resolve() if len(sys.argv) > 1 else None
HEADER = (
    "timestamp_ns,px,py,pz,ax,ay,az,gx,gy,sigma,distance_m,"
    "gnss_present,map_rejected"
).split(",")
ROWS = [
    [1000000000, 0, 0, 3, 0, 0, 4, 0, 0, 1, 0, 1, 1],
    [2000000000, 1, 0, 3, 1, 0, 4, 1, 0, 1, 1, 1, 0],
    [3000000000, 2, 0, 3, 2, 0, 4, 2, 0, 1, 1, 1, 0],
]


class ReplayContracts(unittest.TestCase):
    def invoke(self, directory, name, rows, newline="\r\n", mode="gnss"):
        source = directory / (name + "_input.csv")
        output = directory / (name + "_output.csv")
        with source.open("w", newline="") as stream:
            writer = csv.writer(stream, lineterminator=newline)
            writer.writerow(HEADER)
            writer.writerows(rows)
        command = [str(RUNNER), str(source), str(output), mode]
        if mode == "gnss-recovery":
            command.append(str(directory / (name + "_recovery.csv")))
        result = subprocess.run(
            command,
            capture_output=True, text=True, check=False,
        )
        with output.open(newline="") as stream:
            parsed = list(csv.DictReader(stream))
        return result, parsed

    def test_standard_csv_newlines_and_rejected_map_preserve_estimate(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            lf, lf_rows = self.invoke(directory, "lf", ROWS, newline="\n")
            crlf, crlf_rows = self.invoke(directory, "crlf", ROWS)
            rejected, rejected_rows = self.invoke(
                directory, "map", ROWS, mode="gnss-map-rejected"
            )
            for result in (lf, crlf, rejected):
                self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(len(lf_rows), len(ROWS))
            self.assertEqual(lf_rows, crlf_rows)
            self.assertEqual(rejected_rows[0]["map_rejected"], "1")
            for before, after in zip(lf_rows, rejected_rows):
                after["map_rejected"] = before["map_rejected"]
                self.assertEqual(before, after)
                self.assertEqual(after["tz"], "0")

    def test_duplicate_epoch_and_nonfinite_input_fail_explicitly(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            duplicate, _ = self.invoke(directory, "duplicate", [ROWS[0], ROWS[0]])
            self.assertEqual(duplicate.returncode, 2)
            self.assertIn("nonmonotonic", duplicate.stderr)
            rows = [list(ROWS[0])]
            rows[0][8] = "nan"
            invalid, _ = self.invoke(directory, "invalid", rows)
            self.assertEqual(invalid.returncode, 2)
            self.assertIn("invalid replay scalar", invalid.stderr)

    def test_recovery_holdouts_and_total_window_age(self):
        for interval in (1, 4):
            with self.subTest(interval=interval), tempfile.TemporaryDirectory() as tmp:
                directory = Path(tmp)
                rows = [list(ROWS[0])]
                for index in range(1, 12):
                    x = index * 2
                    rows.append(
                        [(1 + index * interval) * 1000000000,
                         x, 0, 3, x, 0, 4, x + 100, 0, 1, 2, 1, 0]
                    )
                result, output = self.invoke(
                    directory, "recovery", rows, mode="gnss-recovery"
                )
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(len(output), len(rows))
                with (directory / "recovery_recovery.csv").open(newline="") as stream:
                    diagnostics = list(csv.DictReader(stream))
                events = [row for row in diagnostics if row["reanchored"] == "1"]
                self.assertEqual(len(diagnostics), len(rows))
                for sample, actual in zip(rows, output):
                    for position, key in enumerate(("px", "py", "pz"), start=1):
                        self.assertEqual(float(actual[key]), sample[position])
                    self.assertEqual(float(actual["tz"]), 0.0)
                if interval == 1:
                    self.assertEqual(len(events), 1)
                    self.assertEqual(events[0]["training_count"], "8")
                    self.assertEqual(events[0]["validation_count"], "3")
                    self.assertGreaterEqual(
                        float(events[0]["post_var_z"]),
                        float(events[0]["pre_var_z"]) - 1e-8,
                    )
                else:
                    self.assertEqual(events, [])
                    self.assertIn(
                        "recovery_window_timeout",
                        [row["recovery_reason"] for row in diagnostics],
                    )


if __name__ == "__main__":
    if RUNNER is None:
        raise SystemExit("provide the built horizontal_alignment_dataset_replay")
    unittest.main()
