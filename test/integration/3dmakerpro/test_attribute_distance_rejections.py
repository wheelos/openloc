import csv
import tempfile
import unittest
from pathlib import Path

from attribute_distance_rejections import attribute, read_gnss, read_odometry


class AttributeDistanceRejectionsTest(unittest.TestCase):
    def test_rejection_includes_neighbor_and_unavailable_evidence(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            odometry_path = directory / "odometry.csv"
            gnss_path = directory / "gnss.csv"
            reference_path = directory / "reference.txt"

            with odometry_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")
                )
                for timestamp in range(6):
                    writer.writerow(
                        (timestamp, timestamp, 0, 0, 0, 0, 0, 1)
                    )

            with gnss_path.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    (
                        "timestamp",
                        "status",
                        "mode",
                        "quality",
                        "correction_age_s",
                        "satellites_used",
                        "hdop",
                    )
                )
                for timestamp in range(6):
                    writer.writerow((timestamp, "A", "D", 4, 1, 20, 0.7))

            reference_path.write_text(
                "".join(
                    f"{position} 0 0 {timestamp}\n"
                    for timestamp, position in enumerate((0, 1, 2, 3, 6, 5))
                )
            )

            report = attribute(
                read_odometry(odometry_path),
                read_gnss(gnss_path, reference_path),
            )

        self.assertEqual(report["distance_gate_rejections"], 1)
        rejection = report["rejections"][0]
        self.assertEqual(rejection["attribution"]["category"], "UNRESOLVED")
        self.assertEqual(
            rejection["adjacent_epochs"]["previous"]["decision"], "accepted"
        )
        self.assertEqual(rejection["adjacent_epochs"]["next"]["decision"], "accepted")
        self.assertEqual(
            rejection["adjacent_accepted_comparisons"][
                "rejected_to_next_accepted"
            ]["distance_gate_would_reject"],
            False,
        )
        self.assertEqual(
            rejection["odometry_diagnostics"]["valid_point_count"],
            "unavailable_not_exported",
        )
        self.assertEqual(
            rejection["residual_structure"]["vertical_residual_m"],
            "unavailable_vertical_datum_and_frame_not_verified",
        )


if __name__ == "__main__":
    unittest.main()
