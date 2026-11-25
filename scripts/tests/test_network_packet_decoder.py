import ipaddress
import struct
import unittest
import zlib

from livox_mid360_udp_parser import parse_mid360_udp_payload\nfrom network_packet_decoder import (
    LINKTYPE_ETHERNET, LINKTYPE_LINUX_SLL, LINKTYPE_LINUX_SLL2,
    LINKTYPE_RAW, LINKTYPE_IPV4, LINKTYPE_IPV6,
    PacketDecodeError, classify_mid360_datagram, decode_link_packet,
)


def ipv4_udp(payload=b"payload", src="192.0.2.1", dst="192.0.2.2",
             src_port=56300, dst_port=40000, fragment=0, protocol=17):
    udp = struct.pack("!HHHH", src_port, dst_port, 8 + len(payload), 0) + payload
    header = bytearray(20)
    header[0] = 0x45
    struct.pack_into("!H", header, 2, 20 + len(udp))
    struct.pack_into("!H", header, 6, fragment)
    header[8] = 64
    header[9] = protocol
    header[12:16] = ipaddress.ip_address(src).packed
    header[16:20] = ipaddress.ip_address(dst).packed
    return bytes(header) + udp


def ipv6_udp(payload=b"payload", src="2001:db8::1", dst="2001:db8::2",
             src_port=56300, dst_port=40000, extension=False, fragment=None):
    udp = struct.pack("!HHHH", src_port, dst_port, 8 + len(payload), 0) + payload
    ext = b""
    next_header = 17
    if fragment is not None:
        # Fragment header: next header UDP, reserved, offset/reserved/M, identification.
        ext = struct.pack("!BBHI", 17, 0, fragment, 0x12345678)
        next_header = 44
    elif extension:
        ext = struct.pack("!BB6s", 17, 0, b"\x00" * 6)
        next_header = 0
    body = ext + udp
    header = bytearray(40)
    header[0] = 0x60
    struct.pack_into("!H", header, 4, len(body))
    header[6] = next_header
    header[7] = 64
    header[8:24] = ipaddress.ip_address(src).packed
    header[24:40] = ipaddress.ip_address(dst).packed
    return bytes(header) + body


def ethernet(payload, ethertype=0x0800):
    return b"\x00" * 12 + struct.pack("!H", ethertype) + payload


class TestNetworkPacketDecoder(unittest.TestCase):
    def test_ethernet_ipv4_udp_and_capture_metadata(self):
        frame = ethernet(ipv4_udp())
        record = type("Packet", (), {"link_type": LINKTYPE_ETHERNET, "data": frame,
                                     "capture_timestamp_ns": 123, "interface_id": 2,
                                     "section_index": 1})()
        result = decode_link_packet(record)
        self.assertEqual(result.disposition, "udp")
        self.assertEqual(result.datagram.payload, b"payload")
        self.assertEqual(result.datagram.capture_timestamp_ns, 123)
        self.assertEqual(result.datagram.interface_id, 2)
        self.assertEqual(result.datagram.section_index, 1)

    def test_single_and_qinq_vlan_tags(self):
        inner = ipv4_udp()
        single = ethernet(struct.pack("!HH", 7, 0x0800) + inner, 0x8100)
        result = decode_link_packet((LINKTYPE_ETHERNET, single))
        self.assertEqual(result.vlan_ids, (7,))
        qinq = ethernet(struct.pack("!HH", 10, 0x8100) +
                         struct.pack("!HH", 20, 0x0800) + inner, 0x88A8)
        result = decode_link_packet((LINKTYPE_ETHERNET, qinq))
        self.assertEqual(result.vlan_ids, (10, 20))

    def test_linux_cooked_v1_and_v2(self):
        ip = ipv4_udp()
        sll1 = b"\x00" * 14 + struct.pack("!H", 0x0800) + ip
        sll2 = struct.pack("!HHI HBB", 0x0800, 0, 3, 1, 0, 0) + b"\x00" * 8 + ip
        self.assertEqual(decode_link_packet((LINKTYPE_LINUX_SLL, sll1)).disposition, "udp")
        self.assertEqual(decode_link_packet((LINKTYPE_LINUX_SLL2, sll2)).disposition, "udp")

    def test_raw_and_explicit_ip_linktypes(self):
        ip4 = ipv4_udp()
        ip6 = ipv6_udp()
        self.assertEqual(decode_link_packet((LINKTYPE_RAW, ip4)).disposition, "udp")
        self.assertEqual(decode_link_packet((LINKTYPE_IPV4, ip4)).disposition, "udp")
        self.assertEqual(decode_link_packet((LINKTYPE_IPV6, ip6)).disposition, "udp")

    def test_ipv4_options_and_non_udp(self):
        ip = bytearray(ipv4_udp(protocol=6))
        ip[0] = 0x46
        ip[20:20] = b"\x01\x01\x01\x01"
        struct.pack_into("!H", ip, 2, len(ip))
        result = decode_link_packet((LINKTYPE_RAW, bytes(ip)))
        self.assertEqual(result.disposition, "non_udp")
        self.assertEqual(result.ip_version, 4)

    def test_ipv4_fragments_are_not_misreported_as_udp(self):
        result = decode_link_packet((LINKTYPE_RAW, ipv4_udp(fragment=0x2000)))
        self.assertEqual(result.disposition, "fragmented")
        self.assertTrue(result.more_fragments)

    def test_ipv6_extension_header(self):
        result = decode_link_packet((LINKTYPE_RAW, ipv6_udp(extension=True)))
        self.assertEqual(result.disposition, "udp")
        self.assertEqual(result.datagram.payload, b"payload")

    def test_ipv6_fragment_is_not_misreported_as_udp(self):
        result = decode_link_packet((LINKTYPE_RAW, ipv6_udp(fragment=1)))
        self.assertEqual(result.disposition, "fragmented")

    def test_malformed_lengths_and_short_headers(self):
        with self.assertRaises(PacketDecodeError):
            decode_link_packet((LINKTYPE_ETHERNET, b"short"))
        broken = bytearray(ethernet(ipv4_udp()))
        struct.pack_into("!H", broken, 16, 10)
        with self.assertRaises(PacketDecodeError):
            decode_link_packet((LINKTYPE_ETHERNET, bytes(broken)))


    def test_truncated_capture_record_is_not_decoded(self):
        record = type("Packet", (), {"link_type": LINKTYPE_RAW,
                                     "data": ipv4_udp(),
                                     "truncated": True})()
        self.assertEqual(decode_link_packet(record).disposition, "truncated_capture")

    def test_lidar_ip_direction_filter(self):
        from network_packet_decoder import UdpDatagram
        datagram = UdpDatagram(LINKTYPE_RAW, 4, "192.0.2.10", "192.0.2.20",
                               41000, 56301, b"data")
        self.assertIsNone(classify_mid360_datagram(
            datagram, lidar_ip="192.0.2.20", host_ports={56301: "point_cloud"}))
        self.assertEqual(classify_mid360_datagram(
            datagram, lidar_ip="192.0.2.10", host_ports={56301: "point_cloud"}),
            ("point_cloud", "lidar_to_host"))

    def test_network_to_mid360_payload_parser_end_to_end(self):
        payload_data = bytes(range(14))
        payload = bytearray(36 + len(payload_data))
        struct.pack_into("<BHHHHBBB", payload, 0, 0, len(payload), 100,
                         1, 4, 1, 1, 1)
        payload[36:] = payload_data
        struct.pack_into("<Q", payload, 28, 1_700_000_000_000_000_000)
        struct.pack_into("<I", payload, 24, zlib.crc32(payload[28:]) & 0xffffffff)
        decoded = decode_link_packet((LINKTYPE_RAW, ipv4_udp(
            payload=bytes(payload), src_port=56300)))
        self.assertEqual(decoded.disposition, "udp")
        self.assertEqual(classify_mid360_datagram(decoded.datagram),
                         ("point_cloud", "lidar_to_host"))
        mid360 = parse_mid360_udp_payload(decoded.datagram.payload)
        self.assertTrue(mid360.crc32_valid)
        self.assertEqual(mid360.data_type, 1)
        self.assertEqual(mid360.timestamp_ns, 1_700_000_000_000_000_000)

    def test_classifies_official_device_side_ports_and_optional_host_ports(self):
        result = decode_link_packet((LINKTYPE_RAW, ipv4_udp(src_port=56300)))
        self.assertEqual(classify_mid360_datagram(result.datagram),
                         ("point_cloud", "lidar_to_host"))
        self.assertIsNone(classify_mid360_datagram(
            decode_link_packet((LINKTYPE_RAW, ipv4_udp(src_port=56000))).datagram))
        host_to_lidar = decode_link_packet(
            (LINKTYPE_RAW, ipv4_udp(src_port=41000, dst_port=56301))
        )
        self.assertEqual(classify_mid360_datagram(
            host_to_lidar.datagram, host_ports={56301: "point_cloud"}),
            ("point_cloud", "host_to_lidar"))


if __name__ == "__main__":
    unittest.main()
