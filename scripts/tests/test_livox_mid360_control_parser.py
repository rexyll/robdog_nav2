import struct
import unittest
import zlib

from livox_mid360_control_parser import (
    Mid360ControlPacketError, parse_mid360_control_payload,
)


def control_packet(cmd_id, data, *, cmd_type=1, corrupt_crc16=False,
                   corrupt_crc32=False):
    packet = bytearray(24 + len(data))
    packet[0] = 0xAA
    packet[1] = 0
    struct.pack_into("<H", packet, 2, len(packet))
    struct.pack_into("<I", packet, 4, 77)
    struct.pack_into("<H", packet, 8, cmd_id)
    packet[10] = cmd_type
    packet[11] = 1
    packet[24:] = data
    crc16 = 0xFFFF
    for byte in packet[:18]:
        crc16 ^= byte << 8
        for _ in range(8):
            crc16 = ((crc16 << 1) ^ 0x1021) & 0xFFFF if crc16 & 0x8000 else (crc16 << 1) & 0xFFFF
    struct.pack_into("<H", packet, 18, crc16 ^ int(corrupt_crc16))
    crc32 = zlib.crc32(packet[24:]) & 0xFFFFFFFF
    struct.pack_into("<I", packet, 20, crc32 ^ int(corrupt_crc32))
    return bytes(packet)


def kv(key, value):
    return struct.pack("<HH", key, len(value)) + value


class TestMid360ControlParser(unittest.TestCase):
    def test_query_ack_decodes_time_sync_diagnostics(self):
        local = 1_700_000_000_000_000_100
        master = 1_700_000_000_000_000_000
        data = b"\x00" + struct.pack("<H", 4) + b"".join([
            kv(0x8009, struct.pack("<Q", local)),
            kv(0x800A, struct.pack("<Q", master)),
            kv(0x800B, struct.pack("<q", 100)),
            kv(0x800C, b"\x01"),
        ])
        msg = parse_mid360_control_payload(control_packet(0x0101, data))
        self.assertEqual(msg.cmd_id, 0x0101)
        self.assertEqual(msg.return_code, 0)
        values = {v["name"]: v["value"] for v in msg.key_values}
        self.assertEqual(values, {
            "local_time_now": local, "last_sync_time": master,
            "time_offset": 100, "time_sync_type": 1,
        })
        self.assertEqual(msg.key_values[-1]["meaning"], "ptp_ieee1588v2_0")

    def test_status_push_decodes_signed_offset_and_diag_nibbles(self):
        raw_status = (1 << 12) | (2 << 8) | (3 << 4) | 0
        data = struct.pack("<HH", 2, 0) + b"".join([
            kv(0x800B, struct.pack("<q", -25)),
            kv(0x800E, struct.pack("<H", raw_status)),
        ])
        msg = parse_mid360_control_payload(control_packet(
            0x0102, data, cmd_type=0
        ))
        values = {v["name"]: v for v in msg.key_values}
        self.assertEqual(values["time_offset"]["value"], -25)
        self.assertEqual(values["lidar_diag_status"]["modules"]["scan"]["level"], 3)
        self.assertEqual(values["lidar_diag_status"]["modules"]["communication"]["level"], 1)


    def test_hms_code_slots_decode_id_level_and_time_sync_catalog(self):
        codes = [0x04020002, 0x040A0002, 0, 0x01020001, 0, 0, 0, 0]
        data = struct.pack("<HH", 1, 0) + kv(
            0x8011, struct.pack("<8I", *codes)
        )
        msg = parse_mid360_control_payload(control_packet(
            0x0102, data, cmd_type=0
        ))
        item = msg.key_values[0]
        self.assertEqual(item["active_count"], 3)
        self.assertEqual(item["value"][0]["abnormal_id_hex"], "0x0402")
        self.assertEqual(item["value"][0]["level_name"], "warning")
        self.assertIn("PTP synchronization stopped",
                      item["value"][0]["description"])
        self.assertIn("PTP and gPTP", item["value"][1]["description"])
        self.assertEqual(item["value"][2]["slot"], 3)
        self.assertEqual(item["value"][2]["level_name"], "info")

    def test_hms_unknown_id_and_unknown_level_are_not_guessed(self):
        raw = 0x99990009
        data = struct.pack("<HH", 1, 0) + kv(
            0x8011, struct.pack("<8I", raw, 0, 0, 0, 0, 0, 0, 0)
        )
        msg = parse_mid360_control_payload(control_packet(
            0x0102, data, cmd_type=0
        ))
        code, = msg.key_values[0]["value"]
        self.assertIsNone(code["description"])
        self.assertIsNone(code["recommendation"])
        self.assertEqual(code["level_name"], "unknown")
        self.assertEqual(code["reserved"], 0)

    def test_hms_code_requires_exactly_eight_uint32_slots(self):
        data = struct.pack("<HH", 1, 0) + kv(0x8011, b"\\x00" * 28)
        with self.assertRaisesRegex(Mid360ControlPacketError, "32 bytes"):
            parse_mid360_control_payload(control_packet(
                0x0102, data, cmd_type=0
            ))

    def test_unknown_key_is_preserved_without_interpretation(self):
        data = struct.pack("<HH", 1, 0) + kv(0x7777, b"abc")
        msg = parse_mid360_control_payload(control_packet(0x0102, data, cmd_type=0))
        self.assertEqual(msg.key_values[0]["raw_hex"], "616263")
        self.assertNotIn("name", msg.key_values[0])

    def test_bad_crc_and_lengths_are_rejected(self):
        data = b"\x00" + struct.pack("<H", 1) + kv(0x800C, b"\x01")
        with self.assertRaisesRegex(Mid360ControlPacketError, "CRC16"):
            parse_mid360_control_payload(control_packet(
                0x0101, data, corrupt_crc16=True
            ))
        with self.assertRaisesRegex(Mid360ControlPacketError, "CRC32"):
            parse_mid360_control_payload(control_packet(
                0x0101, data, corrupt_crc32=True
            ))
        malformed = control_packet(0x0101, data)[:-1]
        with self.assertRaises(Mid360ControlPacketError):
            parse_mid360_control_payload(malformed)

    def test_malformed_known_key_length_rejected(self):
        data = b"\x00" + struct.pack("<H", 1) + kv(0x800B, b"123")
        with self.assertRaisesRegex(Mid360ControlPacketError, "time_offset"):
            parse_mid360_control_payload(control_packet(0x0101, data))


if __name__ == "__main__":
    unittest.main()
