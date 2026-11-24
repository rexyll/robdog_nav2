import unittest

from livox_time_audit_helpers import (
    nearest_neighbor_deltas_ns,
    percentile,
    stamp_to_ns,
    summarize_time_series,
    summarize_values,
)


class TestPercentile(unittest.TestCase):
    def test_linear_interpolation(self):
        self.assertEqual(percentile([1, 2, 3, 4], 50), 2.5)
        self.assertEqual(percentile([1, 2, 3, 4], 100), 4.0)

    def test_empty_values(self):
        self.assertIsNone(percentile([], 95))

    def test_invalid_percentile_rejected(self):
        with self.assertRaises(ValueError):
            percentile([1, 2], 101)


class TestTimestampConversion(unittest.TestCase):
    def test_converts_seconds_and_nanoseconds(self):
        class Stamp:
            sec = 12
            nanosec = 345

        self.assertEqual(stamp_to_ns(Stamp()), 12_000_000_345)

    def test_invalid_nanoseconds_rejected(self):
        class Stamp:
            sec = 1
            nanosec = 1_000_000_000

        with self.assertRaises(ValueError):
            stamp_to_ns(Stamp())


class TestTimeSeries(unittest.TestCase):
    def test_counts_duplicates_and_backwards(self):
        result = summarize_time_series([0, 10, 20, 20, 15, 30])
        self.assertEqual(result["message_count"], 6)
        self.assertEqual(result["duplicate_timestamp_count"], 1)
        self.assertEqual(result["backward_timestamp_count"], 1)
        self.assertEqual(result["positive_period_ns"]["count"], 3)
        self.assertAlmostEqual(result["median_frequency_hz"], 100_000_000.0)

    def test_single_timestamp_has_no_frequency(self):
        result = summarize_time_series([100])
        self.assertIsNone(result["median_frequency_hz"])

    def test_empty_summary(self):
        result = summarize_time_series([])
        self.assertEqual(result["message_count"], 0)
        self.assertIsNone(result["median_frequency_hz"])


class TestNearestNeighbor(unittest.TestCase):
    def test_nearest_sample_deltas(self):
        self.assertEqual(
            nearest_neighbor_deltas_ns([0, 10, 22], [3, 14, 30]),
            [3, 4, 8],
        )

    def test_missing_samples(self):
        self.assertEqual(nearest_neighbor_deltas_ns([1, 2], []), [])


class TestValueSummary(unittest.TestCase):
    def test_summary_statistics(self):
        result = summarize_values([4, 1, 3, 2])
        self.assertEqual(result["min"], 1.0)
        self.assertEqual(result["max"], 4.0)
        self.assertEqual(result["p50"], 2.5)


if __name__ == "__main__":
    unittest.main()
