import ipaddress
import struct
import unittest
import zlib

from audit_mid360_pcap import audit_packet_records
from pcapng_reader import PcapngPacket


def mid360_payload(data_type, dot_num=1, corrupt_crc=False):
    sample_size = {0: 24, 1: 14, 2: 8, 3: 10}[data_type]
    data = bytes(range(sample_size * dot_num))
    payload = bytearray(36 + len(data))
    struct.pack_into("<BHHHHBBB", payload, 0, 0, len(payload), 100,
                     dot_num, 3, 1, data_type, 1)
    struct.pack_into("<Q", payload, 28, 1_700_000_000_000_000_000)
    payload[36:] = data
    crc = zlib.crc32(payload[28:]) & 0xffffffff
    struct.pack_into("<I", payload, 24, crc ^ (1 if corrupt_crc else 0))
    return bytes(payload)


def raw_ipv4_udp(payload, source_port):
    udp = struct.pack("!HHHH", source_port, 45000, 8 + len(payload), 0) + payload
    ip = bytearray(20)
    ip[0] = 0x45
    struct.pack_into("!H", ip, 2, 20 + len(udp))
    ip[8] = 64
    ip[9] = 17
    ip[12:16] = ipaddress.ip_address("192.0.2.10").packed
    ip[16:20] = ipaddress.ip_address("192.0.2.20").packed
    return bytes(ip) + udp


def record(payload, port, *, truncated=False):
    frame = raw_ipv4_udp(payload, port)
    return PcapngPacket(0, 0, 101, 123, 123, len(frame), len(frame) + int(truncated),
                        frame, True, truncated)


class TestMid360CaptureAudit(unittest.TestCase):
    def test_valid_pointcloud_and_imu_streams(self):
        report = audit_packet_records([
            record(mid360_payload(1), 56300),
            record(mid360_payload(0), 56400),
        ])
        self.assertEqual(report["capture_packets"], 2)
        self.assertEqual(report["streams"], {"point_cloud": 1, "imu": 1})
        self.assertEqual(report["valid_payloads"], {"point_cloud": 1, "imu": 1})
        self.assertEqual(report["payload_errors"], {})
        self.assertEqual(report["time_types"], {"1": 2})
        self.assertFalse(report["time_semantics"]["delta_is_network_latency"])

    def test_crc_error_counted_without_aborting_capture(self):
        report = audit_packet_records([
            record(mid360_payload(1, corrupt_crc=True), 56300),
            record(mid360_payload(1), 56300),
        ])
        self.assertEqual(report["payload_errors"], {"point_cloud": 1})
        self.assertEqual(report["valid_payloads"]["point_cloud"], 1)

    def test_truncated_capture_record_is_skipped(self):
        report = audit_packet_records([record(mid360_payload(1), 56300, truncated=True)])
        self.assertEqual(report["truncated_capture_packets"], 1)
        self.assertEqual(report["streams"], {})


if __name__ == "__main__":
    unittest.main()
