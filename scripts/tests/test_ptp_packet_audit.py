import ipaddress
import struct
import unittest

from pcapng_reader import PcapngPacket
from ptp_packet_audit import (
    PtpPacketError, audit_ptp_records, parse_ptp_v2_message,
)


CLOCK_A = bytes.fromhex("0011223344556677")
CLOCK_B = bytes.fromhex("8899aabbccddeeff")
PORT_A = CLOCK_A + struct.pack("!H", 1)
PORT_B = CLOCK_B + struct.pack("!H", 2)


def timestamp(seconds=100, nanoseconds=123):
    return seconds.to_bytes(6, "big") + struct.pack("!I", nanoseconds)


def ptp_message(message_type, *, source=PORT_A, sequence=1, domain=0,
                flags=0, body=b"", correction=0, version=2, minor_version=0):
    length = 34 + len(body)
    header = bytearray(34)
    header[0] = message_type & 0x0f
    header[1] = (minor_version << 4) | version
    struct.pack_into("!H", header, 2, length)
    header[4] = domain
    struct.pack_into("!H", header, 6, flags)
    struct.pack_into("!q", header, 8, correction)
    header[20:30] = source
    struct.pack_into("!H", header, 30, sequence)
    header[32] = 0
    header[33] = 0x7f
    return bytes(header) + body


def packet(link_type, frame, timestamp_ns=123456789):
    return PcapngPacket(0, 0, link_type, timestamp_ns, timestamp_ns,
                        len(frame), len(frame), frame, True, False)


def raw_ipv4_udp(payload, source_port, destination_port):
    udp = struct.pack("!HHHH", source_port, destination_port, 8 + len(payload), 0) + payload
    ip = bytearray(20)
    ip[0] = 0x45
    struct.pack_into("!H", ip, 2, 20 + len(udp))
    ip[8] = 64
    ip[9] = 17
    ip[12:16] = ipaddress.ip_address("192.0.2.1").packed
    ip[16:20] = ipaddress.ip_address("192.0.2.2").packed
    return bytes(ip) + udp


def ethernet_ptp(payload, vlan=False):
    header = bytes.fromhex("011b19000000001122334455")
    if vlan:
        return header + struct.pack("!HHH", 0x8100, 7, 0x88f7) + payload
    return header + struct.pack("!H", 0x88f7) + payload


class TestPtpPacketAudit(unittest.TestCase):
    def test_parse_common_header_and_timestamp(self):
        raw = ptp_message(0, flags=0x0200, body=timestamp(123, 456),
                          correction=-65536)
        msg = parse_ptp_v2_message(raw, transport="ethernet_l2")
        self.assertEqual(msg.message_name, "sync")
        self.assertTrue(msg.two_step)
        self.assertEqual(msg.correction_field_ns, -1.0)
        self.assertEqual(msg.timestamp_ns, 123_000_000_456)
        self.assertEqual(msg.source_port_identity, "00:11:22:33:44:55:66:77/1")

    def test_rejects_invalid_length_version_and_timestamp(self):
        with self.assertRaises(PtpPacketError):
            parse_ptp_v2_message(b"short", transport="ethernet_l2")
        with self.assertRaises(PtpPacketError):
            parse_ptp_v2_message(ptp_message(0, version=1, body=timestamp()),
                                 transport="ethernet_l2")
        with self.assertRaises(PtpPacketError):
            parse_ptp_v2_message(
                ptp_message(0, minor_version=1, body=timestamp()),
                transport="ethernet_l2",
            )
        bad_length = bytearray(ptp_message(0, body=timestamp()))
        struct.pack_into("!H", bad_length, 2, 1000)
        with self.assertRaises(PtpPacketError):
            parse_ptp_v2_message(bytes(bad_length), transport="ethernet_l2")
        with self.assertRaises(PtpPacketError):
            parse_ptp_v2_message(ptp_message(0, body=timestamp(1, 1_000_000_000)),
                                 transport="ethernet_l2")

    def test_extracts_udp_ipv4_and_l2_vlan_ptp(self):
        sync = ptp_message(0, flags=0x0200, body=timestamp())
        udp_packet = packet(101, raw_ipv4_udp(sync, 319, 319))
        l2_packet = packet(1, ethernet_ptp(sync, vlan=True))
        udp_msg, l2_msg = audit_ptp_records([udp_packet, l2_packet])["sample_messages"]
        self.assertEqual(udp_msg["transport"], "udp/319")
        self.assertEqual(l2_msg["transport"], "ethernet_l2")
        self.assertEqual(l2_msg["vlan_ids"], [7])

    def test_sync_follow_up_pairing(self):
        sync = ptp_message(0, flags=0x0200, sequence=12, body=timestamp())
        follow = ptp_message(8, sequence=12, body=timestamp(100, 999))
        report = audit_ptp_records([
            packet(101, raw_ipv4_udp(sync, 319, 319)),
            packet(101, raw_ipv4_udp(follow, 320, 320)),
        ])
        self.assertEqual(report["correlations"]["sync_follow_up_pairs"], 1)
        self.assertEqual(report["unmatched_at_end"]["two_step_sync_waiting_follow_up"], 0)

    def test_delay_req_resp_and_pdelay_pairing(self):
        delay_req = ptp_message(1, source=PORT_A, sequence=9, body=timestamp())
        delay_resp = ptp_message(9, source=PORT_B, sequence=9,
                                 body=timestamp(101, 456) + PORT_A)
        pdelay_req = ptp_message(2, source=PORT_A, sequence=10, body=timestamp())
        pdelay_resp = ptp_message(3, source=PORT_B, sequence=10,
                                  body=timestamp(102, 789) + PORT_A)
        pdelay_follow = ptp_message(10, source=PORT_B, sequence=10,
                                    body=timestamp(102, 999) + PORT_A)
        report = audit_ptp_records([
            packet(101, raw_ipv4_udp(delay_req, 319, 319)),
            packet(101, raw_ipv4_udp(delay_resp, 319, 319)),
            packet(101, raw_ipv4_udp(pdelay_req, 319, 319)),
            packet(101, raw_ipv4_udp(pdelay_resp, 319, 319)),
            packet(101, raw_ipv4_udp(pdelay_follow, 320, 320)),
        ])
        self.assertEqual(report["correlations"]["delay_req_resp_pairs"], 1)
        self.assertEqual(report["correlations"]["pdelay_req_resp_pairs"], 1)
        self.assertEqual(report["correlations"]["pdelay_resp_follow_up_pairs"], 1)

    def test_port_mismatch_and_orphan_follow_up(self):
        follow = ptp_message(8, sequence=99, body=timestamp())
        report = audit_ptp_records([packet(101, raw_ipv4_udp(follow, 319, 319))])
        self.assertEqual(report["correlations"]["orphan_follow_up"], 1)
        self.assertEqual(report["correlations"]["udp_port_message_type_mismatch"], 1)

    def test_ignores_non_ptp_and_counts_truncated(self):
        non_ptp = packet(101, raw_ipv4_udp(b"not ptp", 1234, 1234))
        base = packet(101, raw_ipv4_udp(ptp_message(0, body=timestamp()), 319, 319))
        truncated = PcapngPacket(0, 0, 101, 1, 1, 5, 10, b"12345", True, True)
        report = audit_ptp_records([non_ptp, truncated, base])
        self.assertEqual(report["message_count"], 1)
        self.assertEqual(report["truncated_capture_records"], 1)


    def test_announce_extracts_grandmaster_clock_quality(self):
        body = bytearray(30)
        body[:10] = timestamp(200, 456)
        struct.pack_into("!h", body, 10, 37)
        body[13] = 1
        body[14] = 6
        body[15] = 0x20
        struct.pack_into("!H", body, 16, 0x1234)
        body[18] = 2
        body[19:27] = bytes.fromhex("aabbccddeeff0011")
        struct.pack_into("!H", body, 27, 3)
        body[29] = 0x20
        msg = parse_ptp_v2_message(
            ptp_message(11, body=bytes(body)), transport="udp/320"
        )
        self.assertEqual(msg.current_utc_offset, 37)
        self.assertEqual(msg.grandmaster_priority1, 1)
        self.assertEqual(msg.grandmaster_clock_class, 6)
        self.assertEqual(msg.grandmaster_clock_accuracy, 0x20)
        self.assertEqual(msg.grandmaster_priority2, 2)
        self.assertEqual(msg.grandmaster_identity, "aa:bb:cc:dd:ee:ff:00:11")
        self.assertEqual(msg.steps_removed, 3)
        self.assertEqual(msg.time_source, 0x20)

    def test_pending_state_is_bounded(self):
        from ptp_packet_audit import PtpCorrelationAudit
        audit = PtpCorrelationAudit(max_pending=2)
        for seq in (1, 2, 3):
            msg = parse_ptp_v2_message(
                ptp_message(0, flags=0x0200, sequence=seq, body=timestamp()),
                transport="ethernet_l2",
            )
            audit.observe(msg)
        report = audit.report()
        self.assertEqual(report["unmatched_at_end"]["two_step_sync_waiting_follow_up"], 2)
        self.assertEqual(report["correlations"]["expired_pending_sync"], 1)


if __name__ == "__main__":
    unittest.main()
