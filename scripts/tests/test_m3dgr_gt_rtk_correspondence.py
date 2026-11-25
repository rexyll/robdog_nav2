from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.typesys import Stores, get_typestore

from audit_m3dgr_gt_rtk_correspondence import (
    audit_gt_rtk_correspondence,
    geodetic_to_enu,
    rigid_trajectory_correspondence,
    message_snapshot,
)


class TestM3DGRGTRTKCorrespondence(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.bag_path = self.root / "Dynamic03.bag"
        self.gt_path = self.root / "Dynamic03.txt"

    def _make_bag(self, points):
        typestore = get_typestore(Stores.ROS1_NOETIC)
        nav_type = typestore.types["sensor_msgs/msg/NavSatFix"]
        status_type = typestore.types["sensor_msgs/msg/NavSatStatus"]
        header_type = typestore.types["std_msgs/msg/Header"]
        time_type = typestore.types["builtin_interfaces/msg/Time"]

        with Rosbag1Writer(self.bag_path) as writer:
            connection = writer.add_connection(
                "/mavros/global_position/raw/fix",
                "sensor_msgs/msg/NavSatFix",
                typestore=typestore,
            )
            for index, (sec, lat, lon, alt) in enumerate(points, start=1):
                message = nav_type(
                    header=header_type(
                        seq=index,
                        stamp=time_type(sec=sec, nanosec=0),
                        frame_id="gps",
                    ),
                    status=status_type(status=2, service=1),
                    latitude=lat,
                    longitude=lon,
                    altitude=alt,
                    position_covariance=np.zeros(9, dtype=np.float64),
                    position_covariance_type=3,
                )
                typestore_msg = typestore.serialize_ros1(
                    message, "sensor_msgs/msg/NavSatFix"
                )
                writer.write(connection, sec * 1_000_000_000 + 5_000_000, typestore_msg)

    def test_matches_equal_length_rtk_stream_by_original_row_order(self):
        base_lat = 37.0
        base_lon = -122.0
        radius = 6_378_137.0
        samples_enu = np.asarray(
            [[0.0, 0.0, 0.0], [1.0, 2.0, 0.0], [2.0, 1.0, 0.1], [3.0, 4.0, 0.2]],
            dtype=np.float64,
        )
        points = []
        for index, (east, north, up) in enumerate(samples_enu, start=10):
            lat = base_lat + np.degrees(north / radius)
            lon = base_lon + np.degrees(east / (radius * np.cos(np.radians(base_lat))))
            points.append((index, lat, lon, 100.0 + up))
        self._make_bag(points)

        enu = geodetic_to_enu(
            [item[1] for item in points],
            [item[2] for item in points],
            [item[3] for item in points],
        )
        angle = np.radians(25.0)
        rotation = np.asarray(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        tum_positions = enu @ rotation + np.asarray([4.0, -2.0, 0.5])
        rows = []
        for index, position in enumerate(tum_positions, start=10):
            rows.append(
                f"{index}.000000 {position[0]:.9f} {position[1]:.9f} "
                f"{position[2]:.9f} 0 0 0 1"
            )
        self.gt_path.write_text("\n".join(rows) + "\n", encoding="utf-8")

        report = audit_gt_rtk_correspondence(self.bag_path, self.gt_path)

        self.assertEqual(report["status"], "association_report_complete")
        self.assertTrue(report["row_count_equal"])
        pairing = report["ordinal_timestamp_comparison"]["gt_minus_rtk_header"]
        self.assertEqual(pairing["absolute_ms"]["maximum"], 0.0)
        position = report["ordinal_position_comparison"]
        self.assertLess(position["rmse_3d_m"], 1e-5)
        self.assertEqual(position["scale_fitted"], False)
        self.assertEqual(report["rtk_status_distribution"], {"2": 4})

    def test_does_not_pair_rows_when_counts_differ(self):
        points = [
            (10, 37.0, -122.0, 100.0),
            (11, 37.00001, -122.00001, 100.1),
            (12, 37.00002, -122.00002, 100.2),
        ]
        self._make_bag(points)
        self.gt_path.write_text(
            "10.0 0 0 0 0 0 0 1\n"
            "11.0 1 1 0 0 0 0 1\n",
            encoding="utf-8",
        )
        report = audit_gt_rtk_correspondence(self.bag_path, self.gt_path)
        self.assertFalse(report["row_count_equal"])
        self.assertEqual(
            report["ordinal_timestamp_comparison"]["status"], "unavailable"
        )
        self.assertEqual(
            report["ordinal_position_comparison"]["status"], "unavailable"
        )

    def test_navsatfix_report_includes_frame_service_and_covariance_metadata(self):
        points = [
            (10, 37.0, -122.0, 100.0),
            (11, 37.00001, -122.00001, 100.1),
            (12, 37.00002, -122.00002, 100.2),
        ]
        self._make_bag(points)
        rows = [
            "10.0 0 0 0 0 0 0 1",
            "11.0 1 1 0 0 0 0 1",
            "12.0 2 2 0 0 0 0 1",
        ]
        self.gt_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
        report = audit_gt_rtk_correspondence(self.bag_path, self.gt_path)
        metadata = report["rtk_navsatfix_metadata"]
        self.assertEqual(metadata["frame_id_distribution"], {"gps": 3})
        self.assertEqual(metadata["service_bitfield_distribution"], {"1": 3})
        self.assertEqual(metadata["covariance_finite_matrix_count"], 3)
        self.assertEqual(report["related_gnss_topics"]["/ublox_driver/receiver_lla"]["status"], "topic_absent")

    def test_message_snapshot_bounds_arrays(self):
        snapshot = message_snapshot({"values": list(range(40))}, max_items=8)
        self.assertEqual(snapshot["values"]["length"], 40)
        self.assertTrue(snapshot["values"]["truncated"])
        self.assertEqual(len(snapshot["values"]["items"]), 8)

    def test_rigid_fit_reports_diagnostic_residual_without_scale(self):
        source = np.asarray(
            [[0, 0, 0], [1, 0, 0], [0, 2, 0], [1, 2, 1]],
            dtype=np.float64,
        )
        angle = np.radians(60.0)
        rotation = np.asarray(
            [
                [np.cos(angle), -np.sin(angle), 0.0],
                [np.sin(angle), np.cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        target = source @ rotation + np.asarray([10.0, -3.0, 2.0])
        result = rigid_trajectory_correspondence(source, target)
        self.assertLess(result["rmse_3d_m"], 1e-10)
        self.assertFalse(result["scale_fitted"])


if __name__ == "__main__":
    unittest.main()
