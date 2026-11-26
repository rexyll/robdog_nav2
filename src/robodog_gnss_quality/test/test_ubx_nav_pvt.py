import unittest

from robodog_gnss_quality import CarrierSolution, CovarianceType
from robodog_gnss_quality.ubx_nav_pvt import (
    EpochPairingError,
    NavSatEpochRecord,
    UbxNavPvtQuality,
    UbxNavPvtEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavCovEpoch,
    UbxNavStatusEpoch,
    EpochPairingError,
    build_observation_from_ubx_epochs,
    decode_nav_pvt_flags,
    ned_covariance_to_enu,
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


def make_raw_pvt(**overrides):
    values = dict(
        process_session_id="legacy-ubx-test-session",
        connection_generation=0,
        gps_week=2450,
        i_tow_ms=345_000,
        gnss_fix_ok=True,
        carrier_solution=CarrierSolution.FIXED,
        receive_monotonic_ns=10_000_000_000,
    )
    values.update(overrides)
    return UbxNavPvtEpoch(**values)


def make_hpposllh(**overrides):
    values = dict(
        process_session_id="legacy-ubx-test-session",
        connection_generation=0,
        i_tow_ms=345_000,
        lon=-1_220_000_000,
        lat=370_000_000,
        height_mm=15_000,
        lon_hp=0,
        lat_hp=0,
        height_hp=0,
        invalid_lon=False,
        invalid_lat=False,
        invalid_height=False,
        invalid_lon_hp=False,
        invalid_lat_hp=False,
        invalid_height_hp=False,
        receive_monotonic_ns=10_001_000_000,
    )
    values.update(overrides)
    return UbxNavHpPosLlhEpoch(**values)


def make_nav_cov(**overrides):
    values = dict(
        process_session_id="legacy-ubx-test-session",
        connection_generation=0,
        i_tow_ms=345_000,
        pos_cor_valid=True,
        pos_cov_nn=1.0,
        pos_cov_ne=0.1,
        pos_cov_nd=0.2,
        pos_cov_ee=2.0,
        pos_cov_ed=0.3,
        pos_cov_dd=3.0,
        receive_monotonic_ns=10_002_000_000,
    )
    values.update(overrides)
    return UbxNavCovEpoch(**values)


def make_nav_status(**overrides):
    values = dict(
        process_session_id="legacy-ubx-test-session",
        connection_generation=0,
        i_tow_ms=345_000,
        gnss_fix_ok=True,
        carrier_solution_valid=True,
        carrier_solution=CarrierSolution.FIXED,
        receive_monotonic_ns=10_003_000_000,
    )
    values.update(overrides)
    return UbxNavStatusEpoch(**values)


def make_raw_observation(**overrides):
    values = dict(
        pvt=make_raw_pvt(),
        position=make_hpposllh(),
        covariance=make_nav_cov(),
        status=make_nav_status(),
        source_timestamp_ns=1_234_500_000_000,
        source_clock_id="gps_week_2450_receiver_generation_0",
        max_pair_receive_skew_ns=5_000_000,
    )
    values.update(overrides)
    return build_observation_from_ubx_epochs(**values)


class TestNedToEnuCovariance(unittest.TestCase):
    def test_rotates_all_cross_terms_and_preserves_variances(self):
        self.assertEqual(
            ned_covariance_to_enu(1.0, 0.1, 0.2, 2.0, 0.3, 3.0),
            (
                2.0, 0.1, -0.3,
                0.1, 1.0, -0.2,
                -0.3, -0.2, 3.0,
            ),
        )

    def test_non_finite_covariance_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "NaN or Inf"):
            ned_covariance_to_enu(1.0, 0.0, float("nan"), 1.0, 0.0, 1.0)


class TestRawUbxEpochJoin(unittest.TestCase):
    def test_four_matching_messages_build_one_enu_observation(self):
        observation = make_raw_observation()
        self.assertEqual(observation.source_timestamp_ns, 1_234_500_000_000)
        self.assertEqual(
            observation.source_clock_id,
            "gps_week_2450_receiver_generation_0",
        )
        self.assertEqual(observation.receive_monotonic_ns, 10_003_000_000)
        self.assertAlmostEqual(observation.latitude_deg, 37.0)
        self.assertAlmostEqual(observation.longitude_deg, -122.0)
        self.assertAlmostEqual(observation.altitude_m, 15.0)
        self.assertEqual(observation.navsat_status, 0)
        self.assertTrue(observation.gnss_fix_ok)
        self.assertEqual(observation.carrier_solution, CarrierSolution.FIXED)
        self.assertEqual(observation.covariance_type, CovarianceType.KNOWN)
        self.assertEqual(
            observation.position_covariance,
            (
                2.0, 0.1, -0.3,
                0.1, 1.0, -0.2,
                -0.3, -0.2, 3.0,
            ),
        )

    def test_exact_itow_required_across_each_message_type(self):
        cases = (
            {"pvt": make_raw_pvt(i_tow_ms=345_001)},
            {"position": make_hpposllh(i_tow_ms=345_001)},
            {"covariance": make_nav_cov(i_tow_ms=345_001)},
            {"status": make_nav_status(i_tow_ms=345_001)},
        )
        for overrides in cases:
            with self.subTest(message=next(iter(overrides))):
                with self.assertRaisesRegex(EpochPairingError, "epoch mismatch"):
                    make_raw_observation(**overrides)

    def test_week_boundary_i_tow_out_of_range_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "GPS week"):
            make_raw_observation(
                covariance=make_nav_cov(i_tow_ms=604_800_000)
            )

    def test_maximum_cross_topic_receive_skew_enforced(self):
        with self.assertRaisesRegex(EpochPairingError, "maximum pairing skew"):
            make_raw_observation(
                status=make_nav_status(receive_monotonic_ns=10_010_000_000),
                max_pair_receive_skew_ns=5_000_000,
            )

    def test_invalid_host_monotonic_stamp_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "monotonic timestamp"):
            make_raw_observation(
                status=make_nav_status(receive_monotonic_ns=True)
            )

    def test_invalid_source_clock_mapping_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "source_clock_id"):
            make_raw_observation(source_clock_id=" ")
        with self.assertRaisesRegex(EpochPairingError, "source_timestamp_ns"):
            make_raw_observation(source_timestamp_ns=True)

    def test_invalid_hpposllh_component_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "position components invalid"):
            make_raw_observation(
                position=make_hpposllh(invalid_lat_hp=True)
            )

    def test_invalid_covariance_flag_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "not marked valid"):
            make_raw_observation(
                covariance=make_nav_cov(pos_cor_valid=False)
            )

    def test_non_finite_covariance_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "NaN or Inf"):
            make_raw_observation(
                covariance=make_nav_cov(pos_cov_ne=float("nan"))
            )

    def test_pvt_and_status_quality_must_agree(self):
        with self.assertRaisesRegex(EpochPairingError, "disagree on GNSS fix validity"):
            make_raw_observation(
                status=make_nav_status(gnss_fix_ok=False)
            )
        with self.assertRaisesRegex(EpochPairingError, "disagree on carrier solution"):
            make_raw_observation(
                status=make_nav_status(carrier_solution=CarrierSolution.FLOAT)
            )

    def test_fixed_solution_requires_status_validity_flag(self):
        with self.assertRaisesRegex(EpochPairingError, "carrier solution as valid"):
            make_raw_observation(
                status=make_nav_status(carrier_solution_valid=False)
            )

    def test_invalid_receiver_fix_is_preserved_as_no_fix(self):
        observation = make_raw_observation(
            pvt=make_raw_pvt(gnss_fix_ok=False),
            status=make_nav_status(gnss_fix_ok=False),
        )
        self.assertFalse(observation.gnss_fix_ok)
        self.assertEqual(observation.navsat_status, -1)




if __name__ == "__main__":
    unittest.main()
