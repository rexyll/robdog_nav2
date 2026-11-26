import struct
import unittest

from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_quality.ubx_ingress_adapter import decode_ubx_time_ingress_frame
from robodog_gnss_quality.ubx_time_messages import (
    UbxNavTimeGpsEpoch,
    UbxNavTimeLsEpoch,
)


SESSION = "raw-f9p-process"
GENERATION = 4


def make_ubx_frame(message_class, message_id, payload):
    header = bytes([message_class, message_id]) + len(payload).to_bytes(2, "little")
    body = header + payload
    ck_a = 0
    ck_b = 0
    for value in body:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xB5\x62" + body + bytes([ck_a, ck_b])


def timegps_payload():
    return struct.pack("<Ii hbBI", 123456, 25, 2450, 18, 0x07, 50)


def timels_payload():
    payload = bytearray(24)
    payload[0:4] = (123456).to_bytes(4, "little")
    payload[4] = 0
    payload[8] = 2
    payload[9] = 18
    payload[10] = 2
    payload[11] = 1
    payload[12:16] = (86400).to_bytes(4, "little", signed=True)
    payload[16:18] = (2451).to_bytes(2, "little")
    payload[18:20] = (7).to_bytes(2, "little")
    payload[23] = 0x03
    return bytes(payload)


def decode(frame, sequence=11, protocol="ubx"):
    return decode_ubx_time_ingress_frame(
        frame,
        protocol=protocol,
        receive_monotonic_ns=123_000,
        process_session_id=SESSION,
        connection_generation=GENERATION,
        ingress_sequence=sequence,
    )


class TestUbxIngressAdapter(unittest.TestCase):
    def test_decodes_nav_timegps_with_event_context(self):
        record = decode(make_ubx_frame(0x01, 0x20, timegps_payload()), sequence=27)
        numeric_protocol = decode_ubx_time_ingress_frame(
            make_ubx_frame(0x01, 0x20, timegps_payload()),
            protocol=0, receive_monotonic_ns=123_000,
            process_session_id=SESSION, connection_generation=GENERATION,
            ingress_sequence=29,
        )
        self.assertIsInstance(numeric_protocol, UbxNavTimeGpsEpoch)
        self.assertEqual(numeric_protocol.ingress_sequence, 29)
        self.assertIsInstance(record, UbxNavTimeGpsEpoch)
        self.assertEqual(record.i_tow_ms, 123456)
        self.assertEqual(record.week, 2450)
        self.assertEqual(record.process_session_id, SESSION)
        self.assertEqual(record.connection_generation, GENERATION)
        self.assertEqual(record.receive_monotonic_ns, 123_000)
        self.assertEqual(record.ingress_sequence, 27)

    def test_decodes_nav_timels_with_event_context(self):
        record = decode(make_ubx_frame(0x01, 0x26, timels_payload()), sequence=28)
        self.assertIsInstance(record, UbxNavTimeLsEpoch)
        self.assertEqual(record.i_tow_ms, 123456)
        self.assertEqual(record.current_leap_seconds, 18)
        self.assertEqual(record.ingress_sequence, 28)

    def test_non_time_ubx_message_is_ignored_after_envelope_validation(self):
        frame = make_ubx_frame(0x01, 0x07, b"\x00\x01")
        self.assertIsNone(decode(frame))

    def test_non_ubx_protocol_is_ignored(self):
        self.assertIsNone(decode(b"not-a-ubx-frame", protocol="rtcm3"))
        self.assertIsNone(decode(b"not-a-ubx-frame", protocol="nmea"))
        self.assertIsNone(decode(b"not-a-ubx-frame", protocol=1))
        self.assertIsNone(decode(b"not-a-ubx-frame", protocol=2))
        with self.assertRaises(EpochPairingError):
            decode(b"not-a-ubx-frame", protocol=9)

    def test_bad_sync_length_and_checksum_are_rejected(self):
        valid = make_ubx_frame(0x01, 0x20, timegps_payload())
        cases = (
            valid[1:],
            valid[:-1],
            valid + b"\x00",
            valid[:-1] + bytes([valid[-1] ^ 0x01]),
        )
        for malformed in cases:
            with self.subTest(length=len(malformed)):
                with self.assertRaises(EpochPairingError):
                    decode(malformed)

    def test_context_and_ingress_sequence_are_mandatory(self):
        valid = make_ubx_frame(0x01, 0x20, timegps_payload())
        for overrides in (
            {"ingress_sequence": True},
            {"receive_monotonic_ns": -1},
            {"process_session_id": " "},
            {"connection_generation": True},
        ):
            kwargs = dict(
                protocol="ubx",
                receive_monotonic_ns=123_000,
                process_session_id=SESSION,
                connection_generation=GENERATION,
                ingress_sequence=1,
            )
            kwargs.update(overrides)
            with self.subTest(overrides=overrides):
                with self.assertRaises(EpochPairingError):
                    decode_ubx_time_ingress_frame(valid, **kwargs)


if __name__ == "__main__":
    unittest.main()
