from __future__ import annotations

import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from audit_m3dgr_ros1_bag import audit_ros1_bag
from convert_m3dgr_dynamic03_to_ros2 import (
    CUSTOM_MSG_MSG,
    CUSTOM_POINT_MSG,
    convert_m3dgr_lio_streams,
)


SOURCE_LIDAR_TYPE = "livox_ros_driver/msg/CustomMsg"
SOURCE_IMU_TYPE = "sensor_msgs/msg/Imu"
TARGET_LIDAR_TYPE = "livox_ros_driver2/msg/CustomMsg"

class TestM3DGRRos2Conversion(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.source_bag = self.root / "Dynamic03.bag"
        self.output_bag = self.root / "Dynamic03_lio_ros2"
        self.audit_path = self.root / "audit.json"
        self.checksum_path = self.root / "checksums.txt"

    def _create_source_bag(self):
        typestore = get_typestore(Stores.ROS1_NOETIC)
        typestore.register(
            get_types_from_msg(CUSTOM_POINT_MSG, "livox_ros_driver/msg/CustomPoint")
        )
        typestore.register(get_types_from_msg(CUSTOM_MSG_MSG, SOURCE_LIDAR_TYPE))

        header_type = typestore.types["std_msgs/msg/Header"]
        time_type = typestore.types["builtin_interfaces/msg/Time"]
        point_type = typestore.types["livox_ros_driver/msg/CustomPoint"]
        lidar_type = typestore.types[SOURCE_LIDAR_TYPE]
        imu_type = typestore.types[SOURCE_IMU_TYPE]
        quaternion_type = typestore.types["geometry_msgs/msg/Quaternion"]
        vector3_type = typestore.types["geometry_msgs/msg/Vector3"]

        header_lidar = header_type(
            seq=1,
            stamp=time_type(sec=100, nanosec=10),
            frame_id="mid360",
        )
        points = [
            point_type(
                offset_time=0,
                x=1.0,
                y=2.0,
                z=3.0,
                reflectivity=80,
                tag=1,
                line=2,
            ),
            point_type(
                offset_time=100_000,
                x=1.1,
                y=2.1,
                z=3.1,
                reflectivity=81,
                tag=2,
                line=3,
            ),
        ]
        lidar_message = lidar_type(
            header=header_lidar,
            timebase=100_000_000_010,
            point_num=2,
            lidar_id=4,
            rsvd=[0, 0, 0],
            points=points,
        )

        header_imu = header_type(
            seq=2,
            stamp=time_type(sec=100, nanosec=20),
            frame_id="mid360_imu",
        )
        imu_message = imu_type(
            header=header_imu,
            orientation=quaternion_type(x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=[0.0] * 9,
            angular_velocity=vector3_type(x=0.1, y=0.2, z=0.3),
            angular_velocity_covariance=[0.01] * 9,
            linear_acceleration=vector3_type(x=1.0, y=2.0, z=9.81),
            linear_acceleration_covariance=[0.02] * 9,
        )

        with Rosbag1Writer(self.source_bag) as writer:
            lidar_connection = writer.add_connection(
                LIDAR_TOPIC := "/livox/mid360/lidar",
                SOURCE_LIDAR_TYPE,
                typestore=typestore,
            )
            imu_connection = writer.add_connection(
                IMU_TOPIC := "/livox/mid360/imu",
                SOURCE_IMU_TYPE,
                typestore=typestore,
            )
            writer.write(
                lidar_connection,
                100_000_000_010,
                typestore.serialize_ros1(lidar_message, SOURCE_LIDAR_TYPE),
            )
            writer.write(
                imu_connection,
                100_000_000_020,
                typestore.serialize_ros1(imu_message, SOURCE_IMU_TYPE),
            )

    def _write_preflight(self):
        report = audit_ros1_bag(self.source_bag)
        self.audit_path.write_text(json.dumps(report), encoding="utf-8")
        digest = hashlib.sha256(self.source_bag.read_bytes()).hexdigest()
        self.checksum_path.write_text(
            f"m3dgr/Dynamic03.bag {digest}\n",
            encoding="utf-8",
        )

    def test_maps_legacy_livox_type_and_preserves_fields(self):
        self._create_source_bag()
        self._write_preflight()
        before_hash = hashlib.sha256(self.source_bag.read_bytes()).hexdigest()

        manifest = convert_m3dgr_lio_streams(
            self.source_bag,
            self.output_bag,
            self.audit_path,
            self.checksum_path,
        )

        self.assertTrue(self.output_bag.is_dir())
        self.assertEqual(hashlib.sha256(self.source_bag.read_bytes()).hexdigest(), before_hash)
        self.assertEqual(
            manifest["message_counts"],
            {"/livox/mid360/lidar": 1, "/livox/mid360/imu": 1},
        )
        self.assertEqual(
            manifest["mapping"]["/livox/mid360/lidar"]["target_type"],
            TARGET_LIDAR_TYPE,
        )
        self.assertTrue((self.output_bag / "conversion_manifest.json").is_file())

        with AnyReader([self.output_bag]) as reader:
            types = {connection.topic: connection.msgtype for connection in reader.connections}
            self.assertEqual(types["/livox/mid360/lidar"], TARGET_LIDAR_TYPE)
            self.assertEqual(types["/livox/mid360/imu"], "sensor_msgs/msg/Imu")

            lidar_connection = next(
                c for c in reader.connections if c.topic == "/livox/mid360/lidar"
            )
            imu_connection = next(
                c for c in reader.connections if c.topic == "/livox/mid360/imu"
            )
            lidar_messages = list(
                reader.messages(connections=[lidar_connection])
            )
            imu_messages = list(reader.messages(connections=[imu_connection]))
            lidar = reader.deserialize(lidar_messages[0][2], lidar_connection.msgtype)
            imu = reader.deserialize(imu_messages[0][2], imu_connection.msgtype)

        self.assertEqual(lidar.timebase, 100_000_000_010)
        self.assertEqual(lidar.point_num, 2)
        self.assertEqual(lidar.points[1].offset_time, 100_000)
        self.assertAlmostEqual(lidar.points[1].x, 1.1, places=6)
        self.assertEqual(lidar.points[1].reflectivity, 81)
        self.assertEqual(lidar.points[1].tag, 2)
        self.assertEqual(lidar.points[1].line, 3)
        self.assertEqual(lidar.header.frame_id, "mid360")
        self.assertAlmostEqual(imu.angular_velocity.z, 0.3, places=6)
        self.assertAlmostEqual(imu.linear_acceleration.z, 9.81, places=6)
        self.assertEqual(imu.header.frame_id, "mid360_imu")

    def test_refuses_unverified_audit_report(self):
        self._create_source_bag()
        self._write_preflight()
        audit = json.loads(self.audit_path.read_text())
        audit["status"] = "blocked_review_required"
        audit["issues"] = ["simulated failure"]
        self.audit_path.write_text(json.dumps(audit), encoding="utf-8")

        with self.assertRaises(ValueError):
            convert_m3dgr_lio_streams(
                self.source_bag,
                self.output_bag,
                self.audit_path,
                self.checksum_path,
            )
        self.assertFalse(self.output_bag.exists())

    def test_refuses_existing_output_directory(self):
        self._create_source_bag()
        self._write_preflight()
        self.output_bag.mkdir()
        with self.assertRaises(FileExistsError):
            convert_m3dgr_lio_streams(
                self.source_bag,
                self.output_bag,
                self.audit_path,
                self.checksum_path,
            )


if __name__ == "__main__":
    unittest.main()
