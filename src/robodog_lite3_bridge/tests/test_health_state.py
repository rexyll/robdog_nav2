import unittest

from robodog_lite3_bridge.health_state import StreamState, classify_stream


class TestClassifyStream(unittest.TestCase):
    def test_not_received(self):
        self.assertEqual(
            classify_stream(None, now_monotonic=10.0, timeout_sec=1.0),
            StreamState.NOT_RECEIVED,
        )

    def test_fresh_inside_timeout(self):
        self.assertEqual(
            classify_stream(9.5, now_monotonic=10.0, timeout_sec=1.0),
            StreamState.FRESH,
        )

    def test_fresh_at_timeout_boundary(self):
        self.assertEqual(
            classify_stream(9.0, now_monotonic=10.0, timeout_sec=1.0),
            StreamState.FRESH,
        )

    def test_stale_after_timeout(self):
        self.assertEqual(
            classify_stream(8.99, now_monotonic=10.0, timeout_sec=1.0),
            StreamState.STALE,
        )

    def test_future_receive_time_is_not_received(self):
        self.assertEqual(
            classify_stream(11.0, now_monotonic=10.0, timeout_sec=1.0),
            StreamState.NOT_RECEIVED,
        )

    def test_non_positive_timeout_is_rejected(self):
        with self.assertRaises(ValueError):
            classify_stream(9.5, now_monotonic=10.0, timeout_sec=0.0)


if __name__ == "__main__":
    unittest.main()
