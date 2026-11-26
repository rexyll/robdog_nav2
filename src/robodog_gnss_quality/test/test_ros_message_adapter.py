import math
import unittest
from types import SimpleNamespace

from robodog_gnss_quality.quality_gate import CarrierSolution
from robodog_gnss_quality.ros_message_adapter import (
    decode_nav_cov_message,
    decode_nav_hp_pos_llh_message,
    decode_nav_pvt_message,
    decode_nav_status_message,
)
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError


PROCESS_SESSION_ID = "ros-message-test-session"
CONNECTION_GENERATION = 0
GPS_WEEK = 2450


def decode_pvt(message, receive):
    return decode_nav_pvt_message(
        message, receive, process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK,
    )


def decode_hppos(message, receive):
    return decode_nav_hp_pos_llh_message(
        message, receive, process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK,
    )


def decode_cov(message, receive):
    return decode_nav_cov_message(
        message, receive, process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK,
    )


def decode_status(message, receive):
    return decode_nav_status_message(
        message, receive, process_session_id=PROCESS_SESSION_ID,
        connection_generation=CONNECTION_GENERATION, gps_week=GPS_WEEK,
    )


def pvt(status=2):
    return SimpleNamespace(
        itow=1234,
        gnss_fix_ok=True,
        carr_soln=SimpleNamespace(status=status),
        header=SimpleNamespace(stamp="deliberately ignored"),
    )


def hppos():
    return SimpleNamespace(
        itow=1234, lon=100, lat=-200, height=3000,
        lon_hp=-2, lat_hp=3, height_hp=-4,
        invalid_lon=False, invalid_lat=False, invalid_height=False,
        invalid_lon_hp=False, invalid_lat_hp=False, invalid_height_hp=False,
        header=SimpleNamespace(stamp="deliberately ignored"),
    )


def cov():
    return SimpleNamespace(
        itow=1234, pos_cor_valid=True,
        pos_cov_nn=0.04, pos_cov_ne=0.001, pos_cov_nd=0.002,
        pos_cov_ee=0.09, pos_cov_ed=0.003, pos_cov_dd=0.16,
    )


def status(code=2):
    return SimpleNamespace(
        itow=1234, gps_fix_ok=True, carr_soln_valid=True,
        carr_soln=SimpleNamespace(status=code),
    )


class TestUbxRosMessageAdapter(unittest.TestCase):
    def test_receiver_context_is_required_and_validated(self):
        with self.assertRaisesRegex(EpochPairingError, "process_session_id"):
            decode_nav_pvt_message(
                pvt(), 10, process_session_id="", connection_generation=0, gps_week=GPS_WEEK
            )
        with self.assertRaisesRegex(EpochPairingError, "connection_generation"):
            decode_nav_pvt_message(
                pvt(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=True, gps_week=GPS_WEEK
            )

    def test_gps_week_context_must_be_signed_16_bit_non_negative(self):
        with self.assertRaisesRegex(EpochPairingError, "gps_week"):
            decode_nav_pvt_message(
                pvt(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=True
            )
        with self.assertRaisesRegex(EpochPairingError, "gps_week"):
            decode_nav_pvt_message(
                pvt(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=32768
            )


    def test_all_four_decoders_reject_invalid_gps_week_context(self):
        decoders = (
            lambda week: decode_nav_pvt_message(
                pvt(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=week
            ),
            lambda week: decode_nav_hp_pos_llh_message(
                hppos(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=week
            ),
            lambda week: decode_nav_cov_message(
                cov(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=week
            ),
            lambda week: decode_nav_status_message(
                status(), 10, process_session_id=PROCESS_SESSION_ID,
                connection_generation=CONNECTION_GENERATION, gps_week=week
            ),
        )
        for index, decode in enumerate(decoders):
            with self.subTest(decoder=index):
                with self.assertRaisesRegex(EpochPairingError, "gps_week"):
                    decode(True)
                with self.assertRaisesRegex(EpochPairingError, "gps_week"):
                    decode(32768)

    def test_decode_pvt_carrier_solution_values(self):
        self.assertEqual(decode_pvt(pvt(0), 10).carrier_solution,
                         CarrierSolution.NO_SOLUTION)
        self.assertEqual(decode_pvt(pvt(1), 10).carrier_solution,
                         CarrierSolution.FLOAT)
        self.assertEqual(decode_pvt(pvt(2), 10).carrier_solution,
                         CarrierSolution.FIXED)
        self.assertEqual(decode_pvt(pvt(3), 10).carrier_solution,
                         CarrierSolution.UNKNOWN)

    def test_decode_hpposllh_preserves_raw_precision_and_flags(self):
        decoded = decode_hppos(hppos(), 11)
        self.assertEqual((decoded.lon, decoded.lon_hp), (100, -2))
        self.assertEqual((decoded.lat, decoded.lat_hp), (-200, 3))
        self.assertEqual((decoded.height_mm, decoded.height_hp), (3000, -4))
        self.assertFalse(decoded.invalid_lon)
        self.assertEqual(decoded.receive_monotonic_ns, 11)
        self.assertEqual(
            (decoded.process_session_id, decoded.connection_generation, decoded.gps_week),
            (PROCESS_SESSION_ID, CONNECTION_GENERATION, GPS_WEEK),
        )

    def test_decode_covariance_and_status(self):
        decoded_cov = decode_cov(cov(), 12)
        self.assertTrue(decoded_cov.pos_cor_valid)
        self.assertAlmostEqual(decoded_cov.pos_cov_ed, 0.003)
        decoded_status = decode_status(status(2), 13)
        self.assertTrue(decoded_status.gnss_fix_ok)
        self.assertTrue(decoded_status.carrier_solution_valid)
        self.assertEqual(decoded_status.carrier_solution, CarrierSolution.FIXED)
        self.assertEqual(
            (decoded_status.process_session_id, decoded_status.connection_generation, decoded_status.gps_week),
            (PROCESS_SESSION_ID, CONNECTION_GENERATION, GPS_WEEK),
        )

    def test_rejects_invalid_itow_receive_time_and_boolean_types(self):
        invalid = pvt()
        invalid.itow = 604800000
        with self.assertRaises(EpochPairingError):
            decode_pvt(invalid, 1)
        with self.assertRaises(EpochPairingError):
            decode_pvt(pvt(), -1)
        invalid = pvt()
        invalid.gnss_fix_ok = 1
        with self.assertRaises(EpochPairingError):
            decode_pvt(invalid, 1)

    def test_rejects_nonfinite_covariance(self):
        invalid = cov()
        invalid.pos_cov_nn = math.nan
        with self.assertRaises(EpochPairingError):
            decode_cov(invalid, 1)

    def test_rejects_boolean_covariance_field(self):
        invalid = cov()
        invalid.pos_cov_nn = True
        with self.assertRaises(EpochPairingError):
            decode_cov(invalid, 1)

    def test_rejects_missing_nested_carrier_field(self):
        invalid = pvt()
        invalid.carr_soln = None
        with self.assertRaises(EpochPairingError):
            decode_pvt(invalid, 1)


if __name__ == "__main__":
    unittest.main()
