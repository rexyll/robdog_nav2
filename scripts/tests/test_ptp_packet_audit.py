import ipaddress
import struct
import unittest

from pcapng_reader import PcapngPacket
from ptp_packet_audit import (
    PtpPacketError, PtpCorrelationAudit, audit_ptp_records, parse_ptp_v2_message,
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


def announce_message(*, source=PORT_A, sequence=1, domain=0, gm_identity=None,
                    priority1=128, clock_class=248, priority2=128,
                    steps_removed=0, capture_timestamp_ns=None):
    body = bytearray(30)
    body[:10] = timestamp(200, 456)
    struct.pack_into("!h", body, 10, 37)
    body[13] = priority1
    body[14] = clock_class
    body[15] = 0x20
    struct.pack_into("!H", body, 16, 0x1234)
    body[18] = priority2
    body[19:27] = gm_identity or CLOCK_A
    struct.pack_into("!H", body, 27, steps_removed)
    body[29] = 0x20
    return parse_ptp_v2_message(
        ptp_message(11, source=source, sequence=sequence, domain=domain,
                    body=bytes(body)),
        transport="udp/320", capture_timestamp_ns=capture_timestamp_ns,
    )


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
        pdelay_exchange = next(
            item for item in report["timestamp_exchange_samples"]
            if item["exchange"] == "peer_delay_response_follow_up"
        )
        self.assertEqual(
            set(pdelay_exchange["messages"]),
            {"pdelay_req", "pdelay_resp", "pdelay_resp_follow_up"},
        )
        self.assertEqual(
            pdelay_exchange["messages"]["pdelay_resp"]["timestamp_ns"],
            102_000_000_789,
        )
        self.assertEqual(
            pdelay_exchange["messages"]["pdelay_resp_follow_up"]["timestamp_ns"],
            102_000_000_999,
        )

    def test_pdelay_follow_up_requires_same_responder_as_response(self):
        request = ptp_message(2, source=PORT_A, sequence=77, body=timestamp())
        response = ptp_message(3, source=PORT_B, sequence=77,
                               body=timestamp(700, 1) + PORT_A)
        wrong_responder = bytes.fromhex("1020304050607080") + struct.pack("!H", 3)
        wrong_follow = ptp_message(10, source=wrong_responder, sequence=77,
                                   body=timestamp(700, 2) + PORT_A)
        correct_follow = ptp_message(10, source=PORT_B, sequence=77,
                                     body=timestamp(700, 3) + PORT_A)
        audit = PtpCorrelationAudit()
        for raw, port in [
            (request, 319), (response, 319), (wrong_follow, 320),
            (correct_follow, 320),
        ]:
            from ptp_packet_audit import extract_ptp_message
            audit.observe(extract_ptp_message(packet(
                101, raw_ipv4_udp(raw, port, port)
            )))
        report = audit.report()
        self.assertEqual(
            report["correlations"]["pdelay_resp_follow_up_responder_mismatch"], 1
        )
        self.assertEqual(report["correlations"]["pdelay_resp_follow_up_pairs"], 1)
        self.assertEqual(report["unmatched_at_end"]["pdelay_resp_waiting_follow_up"], 0)

    def test_ptp_capture_windows_are_partitioned_by_domain_and_interface(self):
        audit = PtpCorrelationAudit()
        first = parse_ptp_v2_message(
            ptp_message(0, body=timestamp(), domain=2),
            transport="udp/319", capture_timestamp_ns=100,
            interface_id=1, section_index=0,
        )
        second = parse_ptp_v2_message(
            ptp_message(0, body=timestamp(), domain=3),
            transport="udp/319", capture_timestamp_ns=250,
            interface_id=1, section_index=0,
        )
        third = parse_ptp_v2_message(
            ptp_message(0, body=timestamp(), domain=2),
            transport="udp/319", capture_timestamp_ns=300,
            interface_id=2, section_index=0,
        )
        for message in (first, second, third):
            audit.observe(message)
        windows = audit.report()["ptp_capture_windows"]
        self.assertEqual(len(windows), 3)
        self.assertEqual(
            {(w["domain_number"], w["interface_id"]) for w in windows},
            {(2, 1), (3, 1), (2, 2)},
        )

    def test_timestamp_exchange_samples_preserve_origin_and_correction_fields(self):
        sync = ptp_message(0, flags=0x0200, sequence=44,
                           body=timestamp(0, 0), correction=65536)
        follow = ptp_message(8, sequence=44,
                             body=timestamp(500, 123456789), correction=-32768)
        delay_req = ptp_message(1, source=PORT_A, sequence=45,
                                body=timestamp(501, 111), correction=16384)
        delay_resp = ptp_message(9, source=PORT_B, sequence=45,
                                 body=timestamp(502, 222) + PORT_A,
                                 correction=-16384)
        packets = [
            packet(101, raw_ipv4_udp(sync, 319, 319), 1_000),
            packet(101, raw_ipv4_udp(follow, 320, 320), 2_000),
            packet(101, raw_ipv4_udp(delay_req, 319, 319), 3_000),
            packet(101, raw_ipv4_udp(delay_resp, 319, 319), 4_000),
        ]
        report = audit_ptp_records(packets)
        samples = report["timestamp_exchange_samples"]
        sync_sample = next(x for x in samples
                           if x["exchange"] == "two_step_sync_follow_up")
        self.assertEqual(sync_sample["messages"]["sync"]["timestamp_ns"], 0)
        self.assertEqual(sync_sample["messages"]["follow_up"]["timestamp_ns"],
                         500_123_456_789)
        self.assertEqual(sync_sample["messages"]["sync"]["correction_field_raw"],
                         65536)
        self.assertEqual(sync_sample["messages"]["follow_up"]["correction_field_raw"],
                         -32768)
        delay_sample = next(x for x in samples
                            if x["exchange"] == "end_to_end_delay_req_resp")
        self.assertEqual(delay_sample["messages"]["delay_req"]["timestamp_ns"],
                         501_000_000_111)
        self.assertEqual(delay_sample["messages"]["delay_resp"]["timestamp_ns"],
                         502_000_000_222)
        self.assertEqual(delay_sample["messages"]["delay_req"]["capture_timestamp_ns"],
                         3_000)
        self.assertIn("no offset", sync_sample["interpretation"])

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

    def test_announce_candidates_sequence_and_source_gm_changes(self):
        audit = PtpCorrelationAudit()
        gm_a = bytes.fromhex("aabbccddeeff0011")
        gm_b = bytes.fromhex("aabbccddeeff0022")
        gm_c = bytes.fromhex("aabbccddeeff0033")
        messages = [
            announce_message(sequence=10, gm_identity=gm_a, capture_timestamp_ns=100),
            announce_message(sequence=12, gm_identity=gm_a, capture_timestamp_ns=200),
            announce_message(sequence=11, gm_identity=gm_a, capture_timestamp_ns=300),
            announce_message(sequence=13, gm_identity=gm_b, capture_timestamp_ns=400),
            announce_message(source=PORT_B, sequence=1, gm_identity=gm_c),
            announce_message(sequence=1, domain=1, gm_identity=gm_a),
        ]
        for message in messages:
            audit.observe(message)
        report = audit.report()["announce_audit"]
        self.assertEqual(report["counts"]["announce_sequence_gap_observed"], 1)
        self.assertEqual(report["counts"]["announce_sequence_out_of_order_or_late"], 1)
        self.assertEqual(report["counts"]["announcing_source_gm_identity_changes"], 1)
        self.assertEqual(report["selection_status"], "NOT_INFERRED_FROM_PACKET_CAPTURE")
        domain_zero = [c for c in report["observed_candidates"] if c["domain_number"] == 0]
        domain_one = [c for c in report["observed_candidates"] if c["domain_number"] == 1]
        self.assertEqual({c["grandmaster_identity"] for c in domain_zero},
                         {gm_a.hex(":"), gm_b.hex(":"), gm_c.hex(":")})
        self.assertEqual(len(domain_one), 1)
        self.assertEqual(domain_one[0]["grandmaster_identity"], gm_a.hex(":"))
        self.assertEqual(report["observed_source_gm_identity_changes"][0]
                         ["capture_timestamp_ns"], 400)

    def test_announce_audit_bounds_candidate_and_source_state(self):
        audit = PtpCorrelationAudit(max_pending=1)
        audit.observe(announce_message(source=PORT_A, sequence=1))
        audit.observe(announce_message(source=PORT_B, sequence=1,
                                       gm_identity=bytes.fromhex("0102030405060708")))
        report = audit.report()["announce_audit"]
        self.assertEqual(len(report["announcing_sources"]), 1)
        self.assertEqual(len(report["observed_candidates"]), 1)
        self.assertEqual(report["counts"]["evicted_announce_sources"], 1)
        self.assertEqual(report["counts"]["evicted_announce_candidates"], 1)

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
