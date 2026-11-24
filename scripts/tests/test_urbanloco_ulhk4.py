import math
import struct
import unittest

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


if __name__ == "__main__":
    unittest.main()
