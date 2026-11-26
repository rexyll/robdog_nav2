import struct
import unittest

from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_quality.ubx_time_messages import (
    decode_nav_timegps_payload,
    decode_nav_timels_payload,
)


SESSION = "process-7"
GENERATION = 12


def timegps_payload(*, itow=123456, ftow=-123, week=2450,
                    leap=18, valid=0x07, accuracy=25):
    return struct.pack("<Ii hbBI", itow, ftow, week, leap, valid, accuracy)


def timels_payload(*, itow=123456, version=0, src=2, curr_ls=18,
                   change_src=0, change=0, time_to_event=-1,
                   event_week=0, event_day=0, valid=0x01,
                   reserved1=b"\x00\x00\x00", reserved2=b"\x00\x00\x00"):
    return (
        struct.pack("<IB", itow, version) + reserved1
        + struct.pack("<BbBb i HH", src, curr_ls, change_src, change,
                      time_to_event, event_week, event_day)
        + reserved2 + bytes([valid])
    )


class TestUbxTimeMessageDecoders(unittest.TestCase):
    def test_nav_timegps_signed_fields_and_independent_validity(self):
        decoded = decode_nav_timegps_payload(
            timegps_payload(valid=0x85, ftow=-500000, week=-2, leap=-1),
            receive_monotonic_ns=100,
            process_session_id=SESSION,
            connection_generation=GENERATION,
        )
        self.assertEqual(decoded.i_tow_ms, 123456)
        self.assertEqual(decoded.f_tow_ns, -500000)
        self.assertEqual(decoded.week, -2)
        self.assertEqual(decoded.leap_seconds, -1)
        self.assertTrue(decoded.tow_valid)
        self.assertFalse(decoded.week_valid)
        self.assertTrue(decoded.leap_seconds_valid)
        self.assertEqual(decoded.reserved_validity_bits, 0x80)
        self.assertEqual(decoded.time_accuracy_ns, 25)
        self.assertEqual(decoded.process_session_id, SESSION)
        self.assertEqual(decoded.connection_generation, GENERATION)

    def test_nav_timegps_max_positive_ftow_and_validity_zero(self):
        decoded = decode_nav_timegps_payload(
            timegps_payload(ftow=500000, valid=0),
            receive_monotonic_ns=0,
            process_session_id=SESSION,
            connection_generation=0,
        )
        self.assertEqual(decoded.f_tow_ns, 500000)
        self.assertFalse(decoded.tow_valid)
        self.assertFalse(decoded.week_valid)
        self.assertFalse(decoded.leap_seconds_valid)

    def test_nav_timels_preserves_sources_signed_fields_and_reserved_bytes(self):
        decoded = decode_nav_timels_payload(
            timels_payload(src=255, curr_ls=-18, change_src=6, change=-1,
                           time_to_event=-123, event_week=2200, event_day=7,
                           valid=0x83, reserved1=b"abc", reserved2=b"xyz"),
            receive_monotonic_ns=200,
            process_session_id=SESSION,
            connection_generation=GENERATION,
        )
        self.assertEqual(decoded.source_current_leap_seconds, 255)
        self.assertEqual(decoded.current_leap_seconds, -18)
        self.assertEqual(decoded.source_leap_seconds_change, 6)
        self.assertEqual(decoded.leap_seconds_change, -1)
        self.assertEqual(decoded.time_to_leap_event_seconds, -123)
        self.assertEqual(decoded.leap_event_gps_week, 2200)
        self.assertEqual(decoded.leap_event_gps_day, 7)
        self.assertTrue(decoded.current_leap_seconds_valid)
        self.assertTrue(decoded.time_to_leap_event_valid)
        self.assertEqual(decoded.reserved_validity_bits, 0x80)
        self.assertEqual(decoded.reserved1, b"abc")
        self.assertEqual(decoded.reserved2, b"xyz")

    def test_nav_timels_current_offset_validity_is_independent(self):
        decoded = decode_nav_timels_payload(
            timels_payload(valid=0x01, time_to_event=999),
            receive_monotonic_ns=1,
            process_session_id=SESSION,
            connection_generation=1,
        )
        self.assertTrue(decoded.current_leap_seconds_valid)
        self.assertFalse(decoded.time_to_leap_event_valid)
        self.assertEqual(decoded.time_to_leap_event_seconds, 999)

    def test_exact_payload_lengths_are_required(self):
        for decoder, payload in (
            (decode_nav_timegps_payload, timegps_payload()),
            (decode_nav_timels_payload, timels_payload()),
        ):
            for malformed in (payload[:-1], payload + b"\x00"):
                with self.subTest(decoder=decoder.__name__, length=len(malformed)):
                    with self.assertRaises(EpochPairingError):
                        decoder(malformed, receive_monotonic_ns=1,
                                process_session_id=SESSION, connection_generation=1)

    def test_nav_timels_unknown_version_rejected(self):
        with self.assertRaisesRegex(EpochPairingError, "unsupported NAV-TIMELS version"):
            decode_nav_timels_payload(
                timels_payload(version=1), receive_monotonic_ns=1,
                process_session_id=SESSION, connection_generation=1)

    def test_itow_and_ftow_ranges_are_checked(self):
        with self.assertRaisesRegex(EpochPairingError, "iTOW"):
            decode_nav_timegps_payload(
                timegps_payload(itow=604800000), receive_monotonic_ns=1,
                process_session_id=SESSION, connection_generation=1)
        with self.assertRaisesRegex(EpochPairingError, "fTOW"):
            decode_nav_timegps_payload(
                timegps_payload(ftow=500001), receive_monotonic_ns=1,
                process_session_id=SESSION, connection_generation=1)


    def test_ingress_sequence_is_preserved_and_strictly_typed(self):
        decoded = decode_nav_timegps_payload(
            timegps_payload(), receive_monotonic_ns=1,
            process_session_id=SESSION, connection_generation=GENERATION,
            ingress_sequence=123,
        )
        self.assertEqual(decoded.ingress_sequence, 123)
        with self.assertRaisesRegex(EpochPairingError, "ingress_sequence"):
            decode_nav_timels_payload(
                timels_payload(), receive_monotonic_ns=1,
                process_session_id=SESSION, connection_generation=GENERATION,
                ingress_sequence=True,
            )

    def test_context_is_mandatory_and_strictly_typed(self):
        for kwargs in (
            {"receive_monotonic_ns": True, "process_session_id": SESSION, "connection_generation": 1},
            {"receive_monotonic_ns": 1, "process_session_id": "", "connection_generation": 1},
            {"receive_monotonic_ns": 1, "process_session_id": SESSION, "connection_generation": True},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(EpochPairingError):
                    decode_nav_timels_payload(timels_payload(), **kwargs)


if __name__ == "__main__":
    unittest.main()
