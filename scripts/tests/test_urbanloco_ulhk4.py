import math
import sqlite3
import struct
import tempfile
import unittest
from pathlib import Path

from extract_urbanloco_ulhk4_gt import extract_samples
from urbanloco_gt_helpers import (
    geodetic_to_enu,
    order_and_deduplicate,
    parse_inspvax_position,
)


class TestInspvaxCdrParsing(unittest.TestCase):
    def _payload(self, lat=22.3, lon=114.2, height=35.0):
        payload = bytearray(84)
        struct.pack_into("<ddd", payload, 60, lat, lon, height)
        return bytes(payload)

    def test_reads_fixed_urbanloco_fields(self):
        self.assertEqual(parse_inspvax_position(self._payload()), (22.3, 114.2, 35.0))

    def test_rejects_short_payload(self):
        with self.assertRaises(ValueError):
            parse_inspvax_position(b"short")

    def test_rejects_invalid_latitude(self):
        with self.assertRaises(ValueError):
            parse_inspvax_position(self._payload(lat=95.0))

    def test_rejects_non_finite_height(self):
        with self.assertRaises(ValueError):
            parse_inspvax_position(self._payload(height=math.nan))


class TestGeodeticToEnu(unittest.TestCase):
    def test_origin_maps_to_zero(self):
        result = geodetic_to_enu(22.3, 114.2, 35.0, (22.3, 114.2, 35.0))
        for coordinate in result:
            self.assertAlmostEqual(coordinate, 0.0, places=6)

    def test_eastward_displacement_is_positive(self):
        origin = (22.3, 114.2, 35.0)
        east, north, up = geodetic_to_enu(22.3, 114.20001, 35.0, origin)
        self.assertGreater(east, 0.5)
        self.assertAlmostEqual(north, 0.0, delta=0.01)
        self.assertAlmostEqual(up, 0.0, delta=0.01)

    def test_height_change_is_up(self):
        origin = (22.3, 114.2, 35.0)
        east, north, up = geodetic_to_enu(22.3, 114.2, 36.0, origin)
        self.assertAlmostEqual(east, 0.0, places=5)
        self.assertAlmostEqual(north, 0.0, places=5)
        self.assertAlmostEqual(up, 1.0, places=5)


class TestOrdering(unittest.TestCase):
    def test_sorts_and_deduplicates_timestamps(self):
        samples = [
            (20, 22.0, 114.0, 30.0),
            (10, 22.1, 114.1, 31.0),
            (20, 22.2, 114.2, 32.0),
        ]
        result, duplicates = order_and_deduplicate(samples)
        self.assertEqual([row[0] for row in result], [10, 20])
        self.assertEqual(duplicates, 1)
        self.assertEqual(result[1][1], 22.0)


class TestRosbagSqliteExtraction(unittest.TestCase):
    @staticmethod
    def _payload(lat, lon, height):
        payload = bytearray(84)
        struct.pack_into("<ddd", payload, 60, lat, lon, height)
        return bytes(payload)

    def test_extracts_inspvax_messages_from_sqlite_bag(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag_dir = Path(temp_dir)
            db3 = bag_dir / "data_0.db3"
            with sqlite3.connect(db3) as connection:
                connection.execute(
                    "CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT)"
                )
                connection.execute(
                    "CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)"
                )
                connection.execute(
                    "INSERT INTO topics (id, name, type) VALUES (?, ?, ?)",
                    (1, "/novatel_data/inspvax", "novatel_oem7_msgs/msg/INSPVAX"),
                )
                connection.executemany(
                    "INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                    [
                        (1, 1_000_000_000, self._payload(22.3, 114.2, 35.0)),
                        (1, 2_000_000_000, self._payload(22.30001, 114.2, 35.5)),
                    ],
                )

            samples, topic_type = extract_samples(
                bag_dir, "/novatel_data/inspvax"
            )
            self.assertEqual(topic_type, "novatel_oem7_msgs/msg/INSPVAX")
            self.assertEqual(len(samples), 2)
            self.assertEqual(samples[0], (1_000_000_000, 22.3, 114.2, 35.0))
            self.assertEqual(samples[1], (2_000_000_000, 22.30001, 114.2, 35.5))

    def test_rejects_non_inspvax_topic_type(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            bag_dir = Path(temp_dir)
            db3 = bag_dir / "data_0.db3"
            with sqlite3.connect(db3) as connection:
                connection.execute("CREATE TABLE topics (id INTEGER, name TEXT, type TEXT)")
                connection.execute("CREATE TABLE messages (id INTEGER, topic_id INTEGER, timestamp INTEGER, data BLOB)")
                connection.execute(
                    "INSERT INTO topics VALUES (?, ?, ?)",
                    (1, "/novatel_data/inspvax", "sensor_msgs/msg/NavSatFix"),
                )
            with self.assertRaises(TypeError):
                extract_samples(bag_dir, "/novatel_data/inspvax")


if __name__ == "__main__":
    unittest.main()
