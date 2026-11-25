import io
import struct
import unittest

from pcapng_reader import PcapngError, iter_pcap_packets, iter_pcapng_packets


def block(kind, body, endian="<"):
    total = 12 + len(body)
    assert total % 4 == 0
    return struct.pack(endian + "II", kind, total) + body + struct.pack(endian + "I", total)


def shb(endian="<"):
    bom = b"\x4d\x3c\x2b\x1a" if endian == "<" else b"\x1a\x2b\x3c\x4d"
    return block(0x0A0D0D0A, bom + struct.pack(endian + "HHq", 1, 0, -1), endian)


def option(code, value, endian="<"):
    return struct.pack(endian + "HH", code, len(value)) + value + b"\x00" * ((-len(value)) % 4)


def idb(endian="<", link=1, snaplen=65535, opts=b""):
    return block(1, struct.pack(endian + "HHI", link, 0, snaplen) + opts, endian)


def epb(data, ticks, endian="<", origlen=None, interface=0):
    origlen = len(data) if origlen is None else origlen
    padded = data + b"\x00" * ((-len(data)) % 4)
    body = struct.pack(
        endian + "IIIII", interface, ticks >> 32, ticks & 0xffffffff,
        len(data), origlen
    ) + padded
    return block(6, body, endian)


class TestPcapngReader(unittest.TestCase):
    def test_little_endian_default_microsecond_timestamp(self):
        raw = shb() + idb() + epb(b"abcd", 1_234_567)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual(packet.link_type, 1)
        self.assertEqual(packet.capture_timestamp_ns, 1_234_567_000)
        self.assertEqual(packet.data, b"abcd")
        self.assertFalse(packet.truncated)

    def test_big_endian_and_decimal_nanosecond_resolution(self):
        opts = option(9, b"\x09", ">")
        raw = shb(">") + idb(">", link=147, opts=opts) + epb(b"1234", 123456789, ">")
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual(packet.link_type, 147)
        self.assertEqual(packet.capture_timestamp_ns, 123456789)

    def test_binary_resolution_and_signed_offset(self):
        opts = option(9, b"\x8a") + option(14, struct.pack("<q", -2))
        raw = shb() + idb(opts=opts) + epb(b"data", 1 << 10)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual(packet.capture_timestamp_ns, -1_000_000_000)

    def test_multi_interface_ids_and_link_types(self):
        raw = shb() + idb(link=1) + idb(link=101) + epb(b"data", 7, interface=1)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual(packet.interface_id, 1)
        self.assertEqual(packet.link_type, 101)

    def test_reports_epb_truncation(self):
        raw = shb() + idb(snaplen=64) + epb(b"short", 0, origlen=100)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertTrue(packet.truncated)
        self.assertEqual(packet.captured_length, 5)
        self.assertEqual(packet.original_length, 100)

    def test_simple_packet_block_has_no_timestamp(self):
        body = struct.pack("<I", 4) + b"data"
        raw = shb() + idb() + block(3, body)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertFalse(packet.timestamp_available)
        self.assertIsNone(packet.capture_timestamp_ns)


    def test_classic_pcap_little_endian_microseconds(self):
        header = b"\xd4\xc3\xb2\xa1" + struct.pack("<HHIIII", 2, 4, 0, 0, 65535, 1)
        record = struct.pack("<IIII", 10, 123456, 4, 6) + b"data"
        packet, = list(iter_pcap_packets(io.BytesIO(header + record)))
        self.assertEqual(packet.link_type, 1)
        self.assertEqual(packet.capture_timestamp_ns, 10_123_456_000)
        self.assertTrue(packet.truncated)

    def test_classic_pcap_big_endian_nanoseconds(self):
        header = b"\xa1\xb2\x3c\x4d" + struct.pack(">HHIIII", 2, 4, 0, 0, 65535, 147)
        record = struct.pack(">IIII", 2, 987654321, 4, 4) + b"data"
        packet, = list(iter_pcap_packets(io.BytesIO(header + record)))
        self.assertEqual(packet.link_type, 147)
        self.assertEqual(packet.capture_timestamp_ns, 2_987_654_321)

    def test_classic_pcap_rejects_invalid_fraction(self):
        header = b"\xd4\xc3\xb2\xa1" + struct.pack("<HHIIII", 2, 4, 0, 0, 65535, 1)
        record = struct.pack("<IIII", 10, 1_000_000, 4, 4) + b"data"
        with self.assertRaisesRegex(PcapngError, "fraction out of range"):
            list(iter_pcap_packets(io.BytesIO(header + record)))


    def test_accepts_repeatable_interface_address_options(self):
        opts = option(4, b"12345678") + option(4, b"abcdefgh")
        raw = shb() + idb(opts=opts) + epb(b"data", 1)
        packet, = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual(packet.data, b"data")

    def test_new_section_resets_interface_table(self):
        raw = shb() + idb(link=1) + epb(b"one!", 1)
        raw += shb(">") + idb(">", link=147) + epb(b"two!", 2, ">")
        packets = list(iter_pcapng_packets(io.BytesIO(raw)))
        self.assertEqual([p.section_index for p in packets], [0, 1])
        self.assertEqual([p.interface_id for p in packets], [0, 0])
        self.assertEqual([p.link_type for p in packets], [1, 147])

    def test_rejects_invalid_trailer(self):
        raw = bytearray(shb() + idb())
        raw[-1] ^= 1
        with self.assertRaisesRegex(PcapngError, "trailer"):
            list(iter_pcapng_packets(io.BytesIO(raw)))

    def test_rejects_unknown_epb_interface(self):
        raw = shb() + idb() + epb(b"data", 0, interface=2)
        with self.assertRaisesRegex(PcapngError, "unknown interface"):
            list(iter_pcapng_packets(io.BytesIO(raw)))

    def test_rejects_short_or_truncated_block(self):
        with self.assertRaises(PcapngError):
            list(iter_pcapng_packets(io.BytesIO(shb()[:-2])))


if __name__ == "__main__":
    unittest.main()
