from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from audit_tum_trajectory import audit_tum_trajectory


class TestTumTrajectoryAudit(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.path = Path(self.temp_dir.name) / "Dynamic03_GT.txt"

    def _write(self, content: str):
        self.path.write_text(content, encoding="utf-8")
        return audit_tum_trajectory(self.path)

    def test_valid_tum_with_comment(self):
        report = self._write(
            "# timestamp tx ty tz qx qy qz qw\n"
            "1733500000.0 0 0 0 0 0 0 1\n"
            "1733500000.1 0.1 0 0 0 0 0 1\n"
            "1733500000.2 0.2 0 0 0 0 0 1\n"
        )
        self.assertEqual(report["status"], "valid_tum_trajectory")
        self.assertEqual(report["valid_pose_rows"], 3)
        self.assertAlmostEqual(report["timestamp"]["average_rate_hz"], 10.0)
        self.assertAlmostEqual(report["path_length_in_file_order_m"], 0.2)
        self.assertEqual(report["comment_rows"], 1)
        self.assertEqual(len(report["sha256"]), 64)

    def test_rejects_wrong_column_count(self):
        report = self._write("1 0 0 0 0 0 1\n2 0 0 0 0 0 0 1\n")
        self.assertEqual(report["status"], "blocked_review_required")
        self.assertEqual(report["malformed_row_count"], 1)

    def test_rejects_non_finite_values(self):
        report = self._write(
            "1 0 0 0 0 0 0 1\n"
            "2 nan 0 0 0 0 0 1\n"
            "3 0 0 0 0 0 0 1\n"
        )
        self.assertEqual(report["non_finite_row_count"], 1)
        self.assertEqual(report["status"], "blocked_review_required")

    def test_detects_duplicate_and_backwards_timestamps(self):
        report = self._write(
            "3 0 0 0 0 0 0 1\n"
            "3 1 0 0 0 0 0 1\n"
            "2 2 0 0 0 0 0 1\n"
        )
        self.assertEqual(report["timestamp"]["duplicate_count"], 1)
        self.assertEqual(report["timestamp"]["backwards_count"], 1)
        self.assertFalse(report["timestamp"]["strictly_increasing"])
        self.assertEqual(report["status"], "blocked_review_required")

    def test_rejects_non_normalized_quaternion(self):
        report = self._write(
            "1 0 0 0 0 0 0 2\n"
            "2 1 0 0 0 0 0 1\n"
        )
        self.assertEqual(report["quaternion_norm"]["invalid_count"], 1)
        self.assertEqual(report["status"], "blocked_review_required")

    def test_rejects_zero_quaternion(self):
        report = self._write(
            "1 0 0 0 0 0 0 0\n"
            "2 1 0 0 0 0 0 1\n"
        )
        self.assertEqual(report["quaternion_norm"]["invalid_count"], 1)

    def test_requires_at_least_two_valid_rows(self):
        report = self._write("1 0 0 0 0 0 0 1\n")
        self.assertEqual(report["valid_pose_rows"], 1)
        self.assertEqual(report["status"], "blocked_review_required")


if __name__ == "__main__":
    unittest.main()
