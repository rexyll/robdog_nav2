import math
import unittest

from robodog_gnss_quality import (
    CarrierSolution,
    CovarianceType,
    GnssObservation,
    GateConfig,
    RejectionReason,
    RtkQualityGate,
)


def make_observation(**overrides):
    values = dict(
        source_timestamp_ns=1_000_000_000,
        receive_monotonic_ns=5_000_000_000,
        latitude_deg=37.0,
        longitude_deg=-122.0,
        altitude_m=10.0,
        navsat_status=0,
        gnss_fix_ok=True,
        carrier_solution=CarrierSolution.FIXED,
        position_covariance=(
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.04,
        ),
        covariance_type=CovarianceType.KNOWN,
    )
    values.update(overrides)
    return GnssObservation(**values)


def make_gate(**overrides):
    values = dict(
        max_fix_age_ns=200_000_000,
        max_horizontal_std_m=0.25,
        max_vertical_std_m=0.5,
        max_plausible_speed_mps=5.0,
        required_consecutive_fixed_samples=3,
        max_jump_reference_interval_ns=2_000_000_000,
    )
    values.update(overrides)
    return RtkQualityGate(GateConfig(**values))


class TestGateConfig(unittest.TestCase):
    def test_positive_config(self):
        make_gate()

    def test_zero_max_age_rejected(self):
        with self.assertRaises(ValueError):
            GateConfig(0, 0.2, 0.5, 2.0)

    def test_nonfinite_threshold_rejected(self):
        with self.assertRaises(ValueError):
            GateConfig(100, math.inf, 0.5, 2.0)

    def test_zero_consecutive_samples_rejected(self):
        with self.assertRaises(ValueError):
            GateConfig(100, 0.2, 0.5, 2.0, required_consecutive_fixed_samples=0)


class TestRtkQualityGate(unittest.TestCase):
    def setUp(self):
        self.gate = make_gate()

    def evaluate(self, gate=None, index=0, **overrides):
        observation = make_observation(
            source_timestamp_ns=1_000_000_000 + index * 100_000_000,
            receive_monotonic_ns=5_000_000_000 + index * 100_000_000,
            **overrides,
        )
        return (gate or self.gate).evaluate(
            observation,
            now_monotonic_ns=5_000_000_000 + index * 100_000_000 + 10_000_000,
        )

    def test_requires_three_consecutive_fixed_epochs(self):
        self.assertEqual(self.evaluate(index=0).reason, RejectionReason.RECOVERING)
        self.assertEqual(self.evaluate(index=1).reason, RejectionReason.RECOVERING)
        decision = self.evaluate(index=2)
        self.assertTrue(decision.qualified)
        self.assertEqual(decision.reason, RejectionReason.ACCEPTED)

    def test_float_is_never_qualified(self):
        decision = self.evaluate(carrier_solution=CarrierSolution.FLOAT)
        self.assertFalse(decision.qualified)
        self.assertEqual(decision.reason, RejectionReason.RTK_NOT_FIXED)
        self.assertEqual(self.gate.consecutive_fixed_samples, 0)

    def test_no_carrier_solution_rejected(self):
        decision = self.evaluate(carrier_solution=CarrierSolution.NO_SOLUTION)
        self.assertEqual(decision.reason, RejectionReason.RTK_NOT_FIXED)

    def test_navsat_no_fix_rejected(self):
        decision = self.evaluate(navsat_status=-1)
        self.assertEqual(decision.reason, RejectionReason.GNSS_FIX_INVALID)

    def test_receiver_gnss_fix_ok_must_be_true(self):
        decision = self.evaluate(gnss_fix_ok=False)
        self.assertEqual(decision.reason, RejectionReason.GNSS_FIX_INVALID)

    def test_stale_fix_rejected(self):
        observation = make_observation(receive_monotonic_ns=1_000_000_000)
        decision = self.gate.evaluate(observation, now_monotonic_ns=2_000_000_000)
        self.assertEqual(decision.reason, RejectionReason.FIX_STALE)

    def test_future_receive_timestamp_rejected(self):
        observation = make_observation(receive_monotonic_ns=2_000_000_000)
        decision = self.gate.evaluate(observation, now_monotonic_ns=1_000_000_000)
        self.assertEqual(decision.reason, RejectionReason.RECEIVE_TIME_INVALID)

    def test_invalid_coordinate_rejected(self):
        decision = self.evaluate(latitude_deg=91.0)
        self.assertEqual(decision.reason, RejectionReason.COORDINATE_INVALID)

    def test_nonfinite_coordinate_rejected(self):
        decision = self.evaluate(longitude_deg=float("nan"))
        self.assertEqual(decision.reason, RejectionReason.COORDINATE_INVALID)

    def test_unknown_covariance_type_rejected(self):
        decision = self.evaluate(covariance_type=CovarianceType.UNKNOWN)
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_TYPE_UNKNOWN)

    def test_approximated_covariance_rejected_by_default(self):
        decision = self.evaluate(covariance_type=CovarianceType.APPROXIMATED)
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_APPROXIMATED)

    def test_approximated_covariance_can_be_explicitly_enabled(self):
        gate = make_gate(allow_approximated_covariance=True)
        decision = self.evaluate(gate=gate, covariance_type=CovarianceType.APPROXIMATED)
        self.assertEqual(decision.reason, RejectionReason.RECOVERING)

    def test_zero_covariance_is_not_treated_as_high_confidence(self):
        zero = (0.0,) * 9
        decision = self.evaluate(position_covariance=zero)
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_DEGENERATE)

    def test_nan_covariance_rejected(self):
        values = list(make_observation().position_covariance)
        values[0] = float("nan")
        decision = self.evaluate(position_covariance=tuple(values))
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_INVALID)

    def test_asymmetric_covariance_rejected(self):
        values = (
            0.01, 0.003, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.04,
        )
        decision = self.evaluate(position_covariance=values)
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_INVALID)

    def test_indefinite_covariance_rejected_even_with_positive_diagonal(self):
        values = (
            0.01, 0.03, 0.0,
            0.03, 0.01, 0.0,
            0.0, 0.0, 0.04,
        )
        decision = self.evaluate(position_covariance=values)
        self.assertEqual(decision.reason, RejectionReason.COVARIANCE_INVALID)

    def test_horizontal_uncertainty_threshold(self):
        values = (
            0.16, 0.0, 0.0,
            0.0, 0.16, 0.0,
            0.0, 0.0, 0.04,
        )
        decision = self.evaluate(position_covariance=values)
        self.assertEqual(decision.reason, RejectionReason.HORIZONTAL_UNCERTAINTY)

    def test_vertical_uncertainty_threshold(self):
        values = (
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.49,
        )
        decision = self.evaluate(position_covariance=values)
        self.assertEqual(decision.reason, RejectionReason.VERTICAL_UNCERTAINTY)

    def test_duplicate_or_out_of_order_epoch_rejected_without_reset(self):
        self.evaluate(index=0)
        decision = self.evaluate(index=0)
        self.assertEqual(decision.reason, RejectionReason.OUT_OF_ORDER)
        self.assertEqual(self.gate.consecutive_fixed_samples, 1)

    def test_position_jump_rejected_and_recovery_restarts(self):
        self.evaluate(index=0)
        decision = self.evaluate(index=1, latitude_deg=37.01)
        self.assertEqual(decision.reason, RejectionReason.POSITION_JUMP)
        self.assertEqual(self.gate.consecutive_fixed_samples, 0)

    def test_plausible_small_motion_passes(self):
        self.evaluate(index=0)
        decision = self.evaluate(index=1, longitude_deg=-121.999999)
        self.assertEqual(decision.reason, RejectionReason.RECOVERING)

    def test_long_data_gap_restarts_recovery(self):
        self.evaluate(index=0)
        observation = make_observation(
            source_timestamp_ns=5_000_000_000,
            receive_monotonic_ns=9_000_000_000,
        )
        decision = self.gate.evaluate(observation, now_monotonic_ns=9_010_000_000)
        self.assertEqual(decision.reason, RejectionReason.RECOVERING)
        self.assertEqual(decision.consecutive_fixed_samples, 1)

    def test_antimeridian_crossing_does_not_look_like_global_jump(self):
        gate = make_gate(max_plausible_speed_mps=20.0)
        first = make_observation(longitude_deg=179.99999)
        second = make_observation(
            source_timestamp_ns=1_100_000_000,
            receive_monotonic_ns=5_100_000_000,
            longitude_deg=-179.99999,
        )
        gate.evaluate(first, now_monotonic_ns=5_010_000_000)
        decision = gate.evaluate(second, now_monotonic_ns=5_110_000_000)
        self.assertEqual(decision.reason, RejectionReason.RECOVERING)


if __name__ == "__main__":
    unittest.main()
