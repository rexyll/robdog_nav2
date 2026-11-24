import unittest

from robodog_gnss_quality import CarrierSolution, CovarianceType
from robodog_gnss_quality.ubx_nav_pvt import (
    EpochPairingError,
    NavSatEpochRecord,
    UbxNavPvtQuality,
    decode_nav_pvt_flags,
    pair_navsat_fix_with_nav_pvt,
)


def make_position(**overrides):
    values = dict(
        i_tow_ms=345_000,
        source_timestamp_ns=1_234_500_000_000,
        source_clock_id="zed_f9p_gps_week_2450_receiver_epoch_0",
        receive_monotonic_ns=10_000_000_000,
        latitude_deg=37.0,
        longitude_deg=-122.0,
        altitude_m=15.0,
        navsat_status=0,
        position_covariance=(
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.04,
        ),
        covariance_type=CovarianceType.KNOWN,
    )
    values.update(overrides)
    return NavSatEpochRecord(**values)


def make_quality(**overrides):
    values = dict(
        i_tow_ms=345_000,
        flags=0x81,  # gnssFixOK=1, carrSoln=2 (RTK Fixed)
        receive_monotonic_ns=10_002_000_000,
    )
    values.update(overrides)
    return UbxNavPvtQuality(**values)


class TestDecodeNavPvtFlags(unittest.TestCase):
    def test_fixed_solution(self):
        fix_ok, solution = decode_nav_pvt_flags(0x81)
        self.assertTrue(fix_ok)
        self.assertEqual(solution, CarrierSolution.FIXED)

    def test_float_solution(self):
        fix_ok, solution = decode_nav_pvt_flags(0x41)
        self.assertTrue(fix_ok)
        self.assertEqual(solution, CarrierSolution.FLOAT)

    def test_no_carrier_solution(self):
        fix_ok, solution = decode_nav_pvt_flags(0x01)
        self.assertTrue(fix_ok)
        self.assertEqual(solution, CarrierSolution.NO_SOLUTION)

    def test_receiver_fix_invalid_is_preserved(self):
        fix_ok, solution = decode_nav_pvt_flags(0x80)
        self.assertFalse(fix_ok)
        self.assertEqual(solution, CarrierSolution.FIXED)

    def test_reserved_carrier_code_fails_closed(self):
        fix_ok, solution = decode_nav_pvt_flags(0xC1)
        self.assertTrue(fix_ok)
        self.assertEqual(solution, CarrierSolution.UNKNOWN)

    def test_flags_outside_uint8_rejected(self):
        for flags in (-1, 256, True):
            with self.subTest(flags=flags):
                with self.assertRaises(EpochPairingError):
                    decode_nav_pvt_flags(flags)


class TestEpochPairing(unittest.TestCase):
    def test_matching_epoch_produces_observation(self):
        observation = pair_navsat_fix_with_nav_pvt(
            make_position(),
            make_quality(),
            max_pair_receive_skew_ns=5_000_000,
        )
        self.assertEqual(observation.carrier_solution, CarrierSolution.FIXED)
        self.assertTrue(observation.gnss_fix_ok)
        self.assertEqual(observation.source_clock_id, "zed_f9p_gps_week_2450_receiver_epoch_0")
        self.assertEqual(observation.receive_monotonic_ns, 10_002_000_000)

    def test_epoch_mismatch_rejected_without_nearest_timestamp_guess(self):
        with self.assertRaisesRegex(EpochPairingError, "epoch mismatch"):
            pair_navsat_fix_with_nav_pvt(
                make_position(i_tow_ms=345_001),
                make_quality(i_tow_ms=345_000),
                max_pair_receive_skew_ns=10_000_000,
            )

    def test_receive_skew_over_limit_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "pairing skew"):
            pair_navsat_fix_with_nav_pvt(
                make_position(receive_monotonic_ns=10_000_000_000),
                make_quality(receive_monotonic_ns=10_020_000_000),
                max_pair_receive_skew_ns=5_000_000,
            )

    def test_invalid_i_tow_rejected(self):
        with self.assertRaises(EpochPairingError):
            pair_navsat_fix_with_nav_pvt(
                make_position(i_tow_ms=604_800_000),
                make_quality(i_tow_ms=604_800_000),
                max_pair_receive_skew_ns=5_000_000,
            )

    def test_source_clock_epoch_required(self):
        with self.assertRaisesRegex(EpochPairingError, "source_clock_id"):
            pair_navsat_fix_with_nav_pvt(
                make_position(source_clock_id=" "),
                make_quality(),
                max_pair_receive_skew_ns=5_000_000,
            )

    def test_unknown_carrier_solution_is_passed_as_unknown_for_gate_rejection(self):
        observation = pair_navsat_fix_with_nav_pvt(
            make_position(),
            make_quality(flags=0xC1),
            max_pair_receive_skew_ns=5_000_000,
        )
        self.assertEqual(observation.carrier_solution, CarrierSolution.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
