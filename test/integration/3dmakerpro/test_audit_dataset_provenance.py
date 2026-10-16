import csv
import tempfile
import unittest
from pathlib import Path

from audit_dataset_provenance import (
    associate_gps_epoch,
    create_epoch_associations,
    crc24q,
    parse_rtcm,
    reconcile_odometry,
)


def set_bits(target, start, width, value):
    for offset in range(width):
        bit = (value >> (width - offset - 1)) & 1
        target_bit = start + offset
        target[target_bit // 8] |= bit << (7 - target_bit % 8)


def rtcm_frame(payload):
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    body = header + payload
    return body + crc24q(body).to_bytes(3, "big")


class DatasetProvenanceTest(unittest.TestCase):
    def test_gps_1074_epoch_normalizes_to_gga_payload_time(self):
        timestamp = 1791440876.0
        week, tow = associate_gps_epoch(timestamp, 368894000)
        payload = bytearray(8)
        set_bits(payload, 0, 12, 1074)
        set_bits(payload, 12, 12, 271)
        set_bits(payload, 24, 30, 368894000)

        frames, invalid = parse_rtcm(rtcm_frame(payload))
        associations = create_epoch_associations(
            [
                {
                    "timestamp_unix_s": timestamp,
                    "quality": 4,
                    "correction_age_s": 1.0,
                    "satellites_used": 30,
                    "hdop": 0.6,
                    "gga_station_id": "271",
                }
            ],
            frames,
            1.0,
        )

        self.assertEqual(invalid, 0)
        self.assertEqual(week, 2439)
        self.assertEqual(tow, timestamp)
        self.assertEqual(associations[0]["rtcm_station_id"], 271)
        self.assertEqual(associations[0]["normalized_epoch_minus_gga_s"], 0.0)
        self.assertEqual(associations[0]["rtcm_epoch_time_scale"], "GPS week/TOW")
        self.assertIsNone(associations[0]["rtcm_file_arrival_timestamp"])
        self.assertIn(
            "not the RTCM message epoch",
            associations[0]["differential_age_semantics"],
        )

    def test_odometry_difference_is_contiguous_suffix(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline_path = root / "baseline.csv"
            full_path = root / "full.csv"
            manifest_path = root / "manifest.json"
            columns = ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")
            rows = [
                (0, 0, 0, 0, 0, 0, 0, 1),
                (1, 1, 0, 0, 0, 0, 0, 1),
                (2, 2, 0, 0, 0, 0, 0, 1),
            ]
            for path, selected in (
                (baseline_path, rows[:2]),
                (full_path, rows),
            ):
                with path.open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                    writer.writerows(selected)
            manifest_path.write_text(
                '{"settings":{"segment_duration_s":"1.0"}}'
            )

            report = reconcile_odometry(
                baseline_path, full_path, manifest_path
            )

        self.assertEqual(report["common_rows"], 2)
        self.assertEqual(report["common_pose_rows_with_different_fields"], 0)
        self.assertEqual(report["baseline_only_rows"], 0)
        self.assertEqual(len(report["full_only_suffix_rows"]), 1)
        self.assertIn(
            "identical earlier poses",
            report["full_only_suffix_rows"][0]["exclusion_reason"],
        )


if __name__ == "__main__":
    unittest.main()
