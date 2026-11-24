from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

from audit_m3dgr_ros1_bag import audit_ros1_bag, stamp_to_ns, summarize_frequency


class TestM3DGRBagAuditHelpers(unittest.TestCase):
    def test_stamp_to_ns(self):
        class Stamp:
            sec = 12
            nanosec = 345

        class Header:
            stamp = Stamp()

        class Message:
            header = Header()

        self.assertEqual(stamp_to_ns(Message()), 12_000_000_345)

    def test_stamp_to_ns_accepts_ros1_nsec_name(self):
        class Stamp:
            sec = 7
            nsec = 9

        class Header:
            stamp = Stamp()

        class Message:
            header = Header()

        self.assertEqual(stamp_to_ns(Message()), 7_000_000_009)

    def test_frequency(self):
        self.assertAlmostEqual(summarize_frequency(11, 0, 1_000_000_000), 10.0)
        self.assertIsNone(summarize_frequency(1, 0, 0))

    def test_audits_ros1_bag_and_fails_closed_for_wrong_sensor_types(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag_path = Path(temp_dir) / "Dynamic03.bag"
            typestore = get_typestore(Stores.ROS1_NOETIC)
            String = typestore.types["std_msgs/msg/String"]

            with Writer(bag_path) as writer:
                lidar_connection = writer.add_connection(
                    "/livox/mid360/lidar",
                    String.__msgtype__,
                    typestore=typestore,
                )
                imu_connection = writer.add_connection(
                    "/livox/mid360/imu",
                    String.__msgtype__,
                    typestore=typestore,
                )
                for index in range(3):
                    timestamp = 1_000_000_000 + index * 100_000_000
                    payload = typestore.serialize_ros1(
                        String(f"sample-{index}"), String.__msgtype__
                    )
                    writer.write(lidar_connection, timestamp, payload)
                    writer.write(imu_connection, timestamp, payload)

            report = audit_ros1_bag(bag_path)
            self.assertEqual(report["input_format"], "ROS1_bag")
            self.assertEqual(report["bag_message_count"], 6)
            self.assertEqual(
                report["topics"]["/livox/mid360/lidar"]["message_count"], 3
            )
            self.assertEqual(
                report["topics"]["/livox/mid360/imu"]["message_count"], 3
            )
            self.assertEqual(report["status"], "blocked_review_required")
            self.assertTrue(
                any("unexpected LiDAR message type" in item for item in report["issues"])
            )
            self.assertTrue(
                any("unexpected IMU message type" in item for item in report["issues"])
            )
            self.assertEqual(
                report["sample_deserialization"]["/livox/mid360/lidar"]["decoded"],
                3,
                report["sample_deserialization"]["/livox/mid360/lidar"],
            )

    def test_rejects_db3_disguised_as_ros1_bag(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            disguised = Path(temp_dir) / "not_really_ros1.bag"
            disguised.write_bytes(b"SQLite format 3\x00")
            with self.assertRaises(ValueError):
                audit_ros1_bag(disguised)


if __name__ == "__main__":
    unittest.main()
