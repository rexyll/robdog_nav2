import unittest

from livox_udp_sequence_audit import LivoxUdpSequenceAudit


def observe(audit, counter, frame=7, data_type=1, timestamp=None):
    return audit.observe(
        udp_cnt=counter, frame_cnt=frame, data_type=data_type,
        timestamp_ns=(counter if timestamp is None else timestamp),
    )


class TestLivoxUdpSequenceAudit(unittest.TestCase):
    def test_contiguous_and_uint16_wrap(self):
        audit = LivoxUdpSequenceAudit()
        self.assertEqual(observe(audit, 65534), "first_packet")
        self.assertEqual(observe(audit, 65535), "in_order")
        self.assertEqual(observe(audit, 0, frame=7), "in_order")
        self.assertEqual(observe(audit, 1, frame=7), "in_order")
        report = audit.report()
        self.assertEqual(report["missing_counter_values_estimated"], 0)
        self.assertEqual(report["events"].get("frame_reset_candidate", 0), 0)

    def test_forward_gap_and_late_recovery(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 10)
        self.assertEqual(observe(audit, 13), "forward_gap")
        self.assertEqual(observe(audit, 11), "reordered_gap_recovered")
        self.assertEqual(observe(audit, 12), "reordered_gap_recovered")
        self.assertEqual(observe(audit, 14), "in_order")
        report = audit.report()
        self.assertEqual(report["missing_counter_values_estimated"], 2)
        self.assertEqual(report["reordered_gap_packets_recovered"], 2)
        self.assertEqual(report["unresolved_gap_values_at_end"], 0)

    def test_duplicate_and_untracked_out_of_order(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 10)
        observe(audit, 11)
        self.assertEqual(observe(audit, 11), "duplicate")
        self.assertEqual(observe(audit, 9), "out_of_order")
        self.assertEqual(audit.report()["events"]["duplicate"], 1)

    def test_frame_reset_requires_frame_counter_change(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 0, frame=10)
        observe(audit, 5, frame=10)
        self.assertEqual(observe(audit, 0, frame=11), "frame_reset_candidate")
        self.assertEqual(observe(audit, 0, frame=11), "duplicate")
        self.assertEqual(audit.report()["events"]["frame_reset_candidate"], 1)

    def test_same_frame_counter_value_at_udp_zero_is_reset_candidate(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 0, frame=10)
        observe(audit, 5, frame=10)
        self.assertEqual(observe(audit, 0, frame=10), "frame_reset_candidate")
        self.assertEqual(audit.report()["events"]["frame_counter_unchanged_at_reset"], 1)

    def test_frame_counter_change_without_udp_reset(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 0, frame=2)
        observe(audit, 4, frame=2)
        self.assertEqual(observe(audit, 0, frame=2), "frame_reset_candidate")
        self.assertEqual(observe(audit, 1, frame=3), "forward_gap")
        report = audit.report()
        self.assertEqual(report["events"]["frame_counter_unchanged_at_reset"], 1)
        self.assertEqual(report["events"]["frame_counter_changed_without_udp_reset"], 1)

    def test_large_gap_is_bounded(self):
        audit = LivoxUdpSequenceAudit(max_pending_gap=2)
        observe(audit, 1)
        self.assertEqual(observe(audit, 1000), "forward_gap")
        self.assertEqual(audit.report()["unresolved_gap_values_at_end"], 0)
        self.assertEqual(audit.report()["events"]["untracked_large_gap_values"], 998)

    def test_timestamp_regression_is_capture_order_observation(self):
        audit = LivoxUdpSequenceAudit()
        observe(audit, 1, timestamp=100)
        observe(audit, 2, timestamp=90)
        self.assertEqual(
            audit.report()["events"]["sensor_timestamp_regressions_in_capture_order"], 1
        )


if __name__ == "__main__":
    unittest.main()
