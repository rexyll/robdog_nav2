import unittest

from robodog_lite3_bridge.telemetry_contract import (
    EnvelopeValidationError,
    SequenceDisposition,
    StreamEnvelope,
    StreamSequenceTracker,
    require_finite_values,
    require_vector_length,
    validate_envelope,
    validate_covariance,
)


def valid_envelope(**overrides):
    values = dict(
        protocol_major=1,
        protocol_minor=0,
        message_type="imu",
        robot_id="lite3-test-unit",
        session_id="session-001",
        sequence=10,
        source_timestamp_ns=123456789,
        source_clock_id="vendor_steady",
        gateway_monotonic_ns=987654321,
        frame_id="imu_link",
        unit_convention="REP103_SI_v1",
        payload_length=128,
    )
    values.update(overrides)
    return StreamEnvelope(**values)


class TestEnvelopeValidation(unittest.TestCase):
    def test_valid_envelope(self):
        validate_envelope(valid_envelope())

    def test_unknown_protocol_major_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(protocol_major=2))

    def test_missing_robot_id_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(robot_id=" "))

    def test_missing_session_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(session_id=""))

    def test_source_timestamp_requires_clock_id(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(source_clock_id=None))

    def test_missing_source_time_is_explicit(self):
        validate_envelope(
            valid_envelope(source_timestamp_ns=None, source_clock_id="no_source_time")
        )

    def test_negative_payload_length_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(payload_length=-1))

    def test_missing_frame_is_allowed_for_non_spatial_status(self):
        validate_envelope(valid_envelope(frame_id=None, message_type="robot_status"))


    def test_contact_state_with_frame_is_valid(self):
        validate_envelope(
            valid_envelope(message_type="contact_state", frame_id="base_link")
        )

    def test_contact_state_requires_frame_id(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(
                valid_envelope(message_type="contact_state", frame_id=None)
            )

    def test_unknown_message_type_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(message_type="unknown"))

    def test_imu_requires_frame_id(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(frame_id=None))

    def test_base_odometry_requires_child_frame_id(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(
                valid_envelope(
                    message_type="base_odometry",
                    frame_id="odom",
                    child_frame_id=None,
                )
            )

    def test_base_odometry_with_frames_is_valid(self):
        validate_envelope(
            valid_envelope(
                message_type="base_odometry",
                frame_id="odom",
                child_frame_id="base_link",
            )
        )

    def test_sequence_over_uint64_rejected(self):
        with self.assertRaises(EnvelopeValidationError):
            validate_envelope(valid_envelope(sequence=1 << 64))


class TestSequenceTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = StreamSequenceTracker()

    def test_first_packet(self):
        result = self.tracker.observe("s1", 100)
        self.assertEqual(result.disposition, SequenceDisposition.FIRST)

    def test_in_order_packet(self):
        self.tracker.observe("s1", 100)
        result = self.tracker.observe("s1", 101)
        self.assertEqual(result.disposition, SequenceDisposition.IN_ORDER)

    def test_gap_is_reported_and_tracker_advances(self):
        self.tracker.observe("s1", 100)
        result = self.tracker.observe("s1", 104)
        self.assertEqual(result.disposition, SequenceDisposition.GAP)
        self.assertEqual(result.missing_count, 3)
        self.assertEqual(self.tracker.last_sequence, 104)

    def test_duplicate_is_reported_without_advancing(self):
        self.tracker.observe("s1", 100)
        result = self.tracker.observe("s1", 100)
        self.assertEqual(result.disposition, SequenceDisposition.DUPLICATE)
        self.assertEqual(self.tracker.last_sequence, 100)

    def test_out_of_order_is_reported_without_advancing(self):
        self.tracker.observe("s1", 100)
        result = self.tracker.observe("s1", 99)
        self.assertEqual(result.disposition, SequenceDisposition.OUT_OF_ORDER)
        self.assertEqual(self.tracker.last_sequence, 100)

    def test_new_session_resets_sequence(self):
        self.tracker.observe("s1", 100)
        result = self.tracker.observe("s2", 0)
        self.assertEqual(result.disposition, SequenceDisposition.SESSION_RESET)
        self.assertEqual(self.tracker.session_id, "s2")
        self.assertEqual(self.tracker.last_sequence, 0)


class TestNumericValidation(unittest.TestCase):
    def test_finite_values(self):
        self.assertEqual(require_finite_values([1.0, -2.0], "vector"), (1.0, -2.0))

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):
            require_finite_values([0.0, float("nan")], "vector")

    def test_infinity_rejected(self):
        with self.assertRaises(ValueError):
            require_finite_values([float("inf")], "vector")

    def test_vector_length_and_values(self):
        self.assertEqual(
            require_vector_length([1.0, 2.0, 3.0], 3, "rpy"),
            (1.0, 2.0, 3.0),
        )

    def test_wrong_vector_length_rejected(self):
        with self.assertRaises(ValueError):
            require_vector_length([1.0, 2.0], 3, "rpy")


    def test_valid_6x6_covariance(self):
        matrix = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            matrix[index] = 0.01
        self.assertEqual(validate_covariance(matrix, 6, "pose_covariance"), tuple(matrix))

    def test_covariance_wrong_length_rejected(self):
        with self.assertRaises(ValueError):
            validate_covariance([0.0] * 9, 6, "pose_covariance")

    def test_negative_covariance_diagonal_rejected(self):
        matrix = [0.0] * 36
        matrix[14] = -0.1
        with self.assertRaises(ValueError):
            validate_covariance(matrix, 6, "pose_covariance")

    def test_unknown_imu_covariance_sentinel_allowed(self):
        matrix = [0.0] * 9
        matrix[0] = -1.0
        self.assertEqual(
            validate_covariance(matrix, 3, "orientation_covariance", allow_unknown=True),
            tuple(matrix),
        )

    def test_unknown_covariance_sentinel_rejected_when_not_allowed(self):
        matrix = [0.0] * 9
        matrix[0] = -1.0
        with self.assertRaises(ValueError):
            validate_covariance(matrix, 3, "position_covariance")


if __name__ == "__main__":
    unittest.main()
