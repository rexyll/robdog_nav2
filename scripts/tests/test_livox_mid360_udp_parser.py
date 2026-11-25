import struct
import unittest
import zlib

from livox_mid360_udp_parser import (
    HEADER_SIZE,
    Mid360PacketError,
    crc32_livox,
    parse_mid360_udp_payload,
)


def synthetic_packet(data_type=1, dot_num=1, time_type=1, data=None):
    sample_sizes = {0: 24, 1: 14, 2: 8, 3: 10}
    if data is None:
        data = bytes(range(sample_sizes[data_type] * dot_num))
    payload = bytearray(HEADER_SIZE + len(data))
    struct.pack_into(
        "<BHHHHBBB",
        payload,
        0,
        0,  # protocol version
        len(payload),
        123,  # time_interval: 12.3 us
        dot_num,
        7,  # udp_cnt
        2,  # frame_cnt
        data_type,
        time_type,
    )
    payload[12:24] = bytes(range(12))
    struct.pack_into("<Q", payload, 28, 1_700_000_000_123_456_789)
    payload[HEADER_SIZE:] = data
    struct.pack_into("<I", payload, 24, zlib.crc32(payload[28:]) & 0xFFFFFFFF)
    return bytes(payload)


class TestLivoxCrc32(unittest.TestCase):
    def test_standard_crc32_check_value(self):
        # CRC-32/ISO-HDLC check value for ASCII "123456789".
        self.assertEqual(crc32_livox(b"123456789"), 0xCBF43926)


class TestMid360PacketParser(unittest.TestCase):
    def test_parses_cartesian_32_packet_and_little_endian_fields(self):
        packet = parse_mid360_udp_payload(synthetic_packet())
        self.assertEqual(packet.version, 0)
        self.assertEqual(packet.length, HEADER_SIZE + 14)
        self.assertEqual(packet.time_interval, 123)
        self.assertEqual(packet.time_interval_ns, 12_300)
        self.assertEqual(packet.dot_num, 1)
        self.assertEqual(packet.udp_cnt, 7)
        self.assertEqual(packet.frame_cnt, 2)
        self.assertEqual(packet.data_type, 1)
        self.assertEqual(packet.time_type, 1)
        self.assertEqual(packet.timestamp_ns, 1_700_000_000_123_456_789)
        self.assertEqual(packet.reserved, bytes(range(12)))
        self.assertTrue(packet.crc32_valid)

    def test_accepts_all_documented_sample_layouts(self):
        for data_type in (0, 1, 2, 3):
            with self.subTest(data_type=data_type):
                packet = parse_mid360_udp_payload(
                    synthetic_packet(data_type=data_type, dot_num=2)
                )
                self.assertEqual(packet.data_type, data_type)
                self.assertEqual(packet.dot_num, 2)
                self.assertEqual(len(packet.data), packet.sample_size * 2)

    def test_rejects_short_header(self):
        with self.assertRaisesRegex(Mid360PacketError, "shorter"):
            parse_mid360_udp_payload(b"\x00" * (HEADER_SIZE - 1))

    def test_rejects_nonzero_protocol_version(self):
        payload = bytearray(synthetic_packet())
        payload[0] = 1
        with self.assertRaisesRegex(Mid360PacketError, "version"):
            parse_mid360_udp_payload(payload)

    def test_rejects_length_mismatch(self):
        payload = bytearray(synthetic_packet())
        struct.pack_into("<H", payload, 1, len(payload) - 1)
        with self.assertRaisesRegex(Mid360PacketError, "does not match"):
            parse_mid360_udp_payload(payload)

    def test_rejects_zero_dot_count(self):
        payload = bytearray(synthetic_packet())
        struct.pack_into("<H", payload, 5, 0)
        with self.assertRaisesRegex(Mid360PacketError, "dot_num"):
            parse_mid360_udp_payload(payload)

    def test_rejects_unknown_data_type(self):
        payload = bytearray(synthetic_packet())
        payload[10] = 4
        with self.assertRaisesRegex(Mid360PacketError, "data_type"):
            parse_mid360_udp_payload(payload)

    def test_rejects_unknown_time_type(self):
        payload = bytearray(synthetic_packet())
        payload[11] = 3
        with self.assertRaisesRegex(Mid360PacketError, "time_type"):
            parse_mid360_udp_payload(payload)

    def test_rejects_data_size_inconsistent_with_type_and_count(self):
        payload = bytearray(synthetic_packet())
        payload[HEADER_SIZE:] = payload[HEADER_SIZE:-1]
        struct.pack_into("<H", payload, 1, len(payload))
        struct.pack_into("<I", payload, 24, zlib.crc32(payload[28:]) & 0xFFFFFFFF)
        with self.assertRaisesRegex(Mid360PacketError, "data length"):
            parse_mid360_udp_payload(payload)

    def test_rejects_corrupted_payload_crc(self):
        payload = bytearray(synthetic_packet())
        payload[-1] ^= 0x80
        with self.assertRaisesRegex(Mid360PacketError, "CRC-32 mismatch"):
            parse_mid360_udp_payload(payload)

    def test_rejects_corrupted_stored_crc(self):
        payload = bytearray(synthetic_packet())
        payload[24] ^= 0x01
        with self.assertRaisesRegex(Mid360PacketError, "CRC-32 mismatch"):
            parse_mid360_udp_payload(payload)


if __name__ == "__main__":
    unittest.main()
