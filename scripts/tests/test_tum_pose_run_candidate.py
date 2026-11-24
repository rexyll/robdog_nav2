from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from audit_tum_trajectory import audit_tum_trajectory
from create_tum_pose_run_candidate import create_pose_run_candidate


class TestTumPoseRunCandidate(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.source = self.root / "source.tum"
        self.output = self.root / "candidate"

    def _write_source(self, text: str):
        self.source.write_text(text, encoding="utf-8")
        return hashlib.sha256(self.source.read_bytes()).hexdigest()

    def test_orders_and_deduplicates_only_inside_identical_pose_runs(self):
        source_text = (
            "# TUM trajectory\n"
            "1.0 0 0 0 0 0 0 1\n"
            "0.9 0 0 0 0 0 0 1\n"
            "0.9 0 0 0 0 0 0 1\n"
            "1.3 1 0 0 0 0 0 1\n"
            "1.2 1 0 0 0 0 0 1\n"
            "1.2 1 0 0 0 0 0 1\n"
            "1.5 2 0 0 0 0 0 1\n"
        )
        before_sha = self._write_source(source_text)

        manifest = create_pose_run_candidate(self.source, self.output)

        self.assertEqual(hashlib.sha256(self.source.read_bytes()).hexdigest(), before_sha)
        candidate = self.output / "trajectory.tum"
        report = audit_tum_trajectory(candidate)
        self.assertEqual(report["status"], "valid_tum_trajectory")
        self.assertEqual(report["valid_pose_rows"], 5)
        self.assertTrue(report["timestamp"]["strictly_increasing"])
        self.assertEqual(
            [line.split()[0] for line in candidate.read_text().splitlines() if line and not line.startswith("#")],
            ["0.9", "1.0", "1.2", "1.3", "1.5"],
        )
        self.assertEqual(manifest["status"], "ordered_tum_candidate_not_validated_ground_truth")
        self.assertEqual(manifest["input"]["rows"], 7)
        self.assertEqual(manifest["input"]["duplicate_timestamps"], 2)
        self.assertEqual(manifest["input"]["backwards_timestamps"], 2)
        self.assertEqual(manifest["input"]["consecutive_pose_run_count"], 3)
        self.assertEqual(manifest["output"]["rows"], 5)
        self.assertEqual(manifest["output"]["exact_duplicate_timestamp_pose_rows_removed"], 2)
        self.assertTrue((self.output / "manifest.json").is_file())
        self.assertEqual(json.loads((self.output / "manifest.json").read_text())["source_sha256"], before_sha)

    def test_refuses_timestamp_range_overlap_between_different_pose_runs(self):
        self._write_source(
            "1.0 0 0 0 0 0 0 1\n"
            "1.2 0 0 0 0 0 0 1\n"
            "1.1 1 0 0 0 0 0 1\n"
            "1.3 1 0 0 0 0 0 1\n"
        )
        with self.assertRaisesRegex(ValueError, "pose boundary"):
            create_pose_run_candidate(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_refuses_timestamp_collision_between_different_pose_runs(self):
        self._write_source(
            "1.0 0 0 0 0 0 0 1\n"
            "1.1 0 0 0 0 0 0 1\n"
            "1.1 1 0 0 0 0 0 1\n"
            "1.2 1 0 0 0 0 0 1\n"
        )
        with self.assertRaisesRegex(ValueError, "pose boundary"):
            create_pose_run_candidate(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_refuses_malformed_or_nonfinite_rows(self):
        self._write_source(
            "1.0 0 0 0 0 0 0 1\n"
            "2.0 nan 0 0 0 0 0 1\n"
            "3.0 1 0 0 0 0 0\n"
        )
        with self.assertRaisesRegex(ValueError, "structurally valid TUM"):
            create_pose_run_candidate(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_refuses_invalid_quaternion(self):
        self._write_source(
            "1.0 0 0 0 0 0 0 2\n"
            "2.0 1 0 0 0 0 0 1\n"
        )
        with self.assertRaisesRegex(ValueError, "invalid_quaternion=1"):
            create_pose_run_candidate(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_refuses_existing_output_directory(self):
        self._write_source(
            "1.0 0 0 0 0 0 0 1\n"
            "2.0 1 0 0 0 0 0 1\n"
        )
        self.output.mkdir()
        with self.assertRaises(FileExistsError):
            create_pose_run_candidate(self.source, self.output)


if __name__ == "__main__":
    unittest.main()
