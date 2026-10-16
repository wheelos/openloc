"""CLI and ODOM identity contracts; not recorded-data fusion evidence."""

import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


BINARY = str(Path(sys.argv.pop(1)).resolve())
HEADER = (
    "timestamp_ns,px,py,pz,qx,qy,qz,qw,gx,gy,sigma,gnss_present,"
    "scan_path,local_position_variance,local_rotation_variance"
)


class ReplayTest(unittest.TestCase):
    def invoke(self, root, rows, mode="odom", flag="--diagnostic-only", crlf=False):
        events = root / "events.csv"
        events.write_bytes(
            (HEADER + "\n" + "\n".join(rows) + "\n")
            .replace("\n", "\r\n" if crlf else "\n")
            .encode()
        )
        prior = root / "prior.csv"
        prior.write_text("100,200,0,0,0,0,1\n")
        output = root / "result.csv"
        result = subprocess.run(
            [BINARY, "-", str(events), str(prior), str(output), mode, flag],
            capture_output=True, text=True, check=False,
        )
        return result, output

    def test_crlf_and_lf_are_identical_and_preserve_odom(self):
        rows = [
            "1000000000,0,0,0,0,0,0,-1,100.1,200,1,1,-,0.01,0.001",
            "1100000000,1,0,0,0,0,0,1,101.1,200,1,1,-,0.02,0.002",
        ]
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            outputs = []
            for ending in (False, True):
                subdir = root / str(ending)
                subdir.mkdir()
                process, output = self.invoke(subdir, rows, mode="gnss",
                                              crlf=ending)
                self.assertEqual(process.returncode, 0, process.stderr)
                outputs.append(output.read_bytes())
                with output.open() as stream:
                    result = list(csv.DictReader(stream))
                self.assertEqual([r["px"] for r in result], ["0", "1"])
                self.assertEqual([r["odom_qw"] for r in result], ["-1", "1"])
                self.assertTrue(all(r["diagnostic_only"] == "1" for r in result))
                self.assertTrue(all(r["gnss_accepted"] == "1" for r in result))
            self.assertEqual(outputs[0], outputs[1])

    def test_duplicate_and_invalid_covariance_are_errors(self):
        valid = "1000000000,0,0,0,0,0,0,1,0,0,1,0,-,0.01,0.001"
        for rows in ([valid, valid], [valid.replace(",0.01,0.001", ",-1,0.001")]):
            with tempfile.TemporaryDirectory() as folder:
                process, _ = self.invoke(Path(folder), rows)
                self.assertNotEqual(process.returncode, 0)
                self.assertTrue(process.stderr.strip())

    def test_explicit_diagnostic_flag_and_no_overwrite(self):
        row = "1000000000,0,0,0,0,0,0,1,0,0,1,0,-,0.01,0.001"
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            process, output = self.invoke(root, [row], flag="--production")
            self.assertNotEqual(process.returncode, 0)
            self.assertFalse(output.exists())
            process, output = self.invoke(root, [row])
            self.assertEqual(process.returncode, 0, process.stderr)
            original = output.read_bytes()
            process, _ = self.invoke(root, [row])
            self.assertNotEqual(process.returncode, 0)
            self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
