import ipaddress
import struct
import unittest
import zlib

from audit_mid360_pcap import audit_packet_records
from pcapng_reader import PcapngPacket


def mid360_payload(data_type, dot_num=1, corrupt_crc=False, udp_cnt=3, frame_cnt=1,
                   time_type=1):
    sample_size = {0: 24, 1: 14, 2: 8, 3: 10}[data_type]
    data = bytes(range(sample_size * dot_num))
    payload = bytearray(36 + len(data))
    struct.pack_into("<BHHHHBBB", payload, 0, 0, len(payload), 100,
                     dot_num, udp_cnt, frame_cnt, data_type, time_type)
    struct.pack_into("<Q", payload, 28, 1_700_000_000_000_000_000)
    payload[36:] = data
    crc = zlib.crc32(payload[28:]) & 0xffffffff
    struct.pack_into("<I", payload, 24, crc ^ (1 if corrupt_crc else 0))
    return bytes(payload)


def raw_ipv4_udp(payload, source_port, destination_port=45000):
    udp = struct.pack("!HHHH", source_port, destination_port, 8 + len(payload), 0) + payload
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


    def test_pointcloud_sequence_gap_and_reordered_recovery(self):
        report = audit_packet_records([
            record(mid360_payload(1, udp_cnt=10), 56300),
            record(mid360_payload(1, udp_cnt=12), 56300),
            record(mid360_payload(1, udp_cnt=11), 56300),
            record(mid360_payload(1, udp_cnt=13), 56300),
        ])
        self.assertEqual(report["sequence_event_counts"]["forward_gap"], 1)
        self.assertEqual(
            report["sequence_event_counts"]["reordered_gap_recovered"], 1
        )
        sequence, = report["point_cloud_sequence_audits"].values()
        self.assertEqual(sequence["missing_counter_values_estimated"], 1)
        self.assertEqual(sequence["reordered_gap_packets_recovered"], 1)
        self.assertEqual(sequence["unresolved_gap_values_at_end"], 0)

    def test_pointcloud_frame_boundary_counter_reset(self):
        report = audit_packet_records([
            record(mid360_payload(1, udp_cnt=65535, frame_cnt=9), 56300),
            record(mid360_payload(1, udp_cnt=0, frame_cnt=10), 56300),
            record(mid360_payload(1, udp_cnt=1, frame_cnt=10), 56300),
        ])
        self.assertEqual(report["sequence_event_counts"]["frame_reset_candidate"], 1)
        sequence, = report["point_cloud_sequence_audits"].values()
        self.assertEqual(sequence["missing_counter_values_estimated"], 0)


    def test_ptp_messages_are_audited_alongside_lidar_streams(self):
        ptp = bytearray(44)
        ptp[0] = 0
        ptp[1] = 2
        struct.pack_into("!H", ptp, 2, len(ptp))
        struct.pack_into("!H", ptp, 6, 0x0200)
        ptp[20:30] = bytes.fromhex("0011223344556677") + struct.pack("!H", 1)
        struct.pack_into("!H", ptp, 30, 42)
        ptp[34:40] = (100).to_bytes(6, "big")
        struct.pack_into("!I", ptp, 40, 123)
        frame = raw_ipv4_udp(bytes(ptp), 319, 319)
        report = audit_packet_records([
            record(mid360_payload(1, udp_cnt=1), 56300),
            PcapngPacket(0, 0, 101, 124, 124, len(frame), len(frame),
                         frame, True, False),
        ])
        self.assertEqual(report["ptp_audit"]["message_count"], 1)
        self.assertEqual(report["ptp_audit"]["message_types"], {"sync": 1})
        self.assertEqual(report["valid_payloads"]["point_cloud"], 1)

    def test_sensor_time_ranges_and_ptp_co_observation_are_not_domain_binding(self):
        sensor_packet = record(mid360_payload(1, time_type=1, udp_cnt=10), 56300)
        ptp = bytearray(44)
        ptp[0] = 0
        ptp[1] = 2
        struct.pack_into("!H", ptp, 2, len(ptp))
        ptp[4] = 7
        ptp[20:30] = bytes.fromhex("0011223344556677") + struct.pack("!H", 1)
        struct.pack_into("!H", ptp, 30, 42)
        ptp[34:40] = (100).to_bytes(6, "big")
        struct.pack_into("!I", ptp, 40, 123)
        frame = raw_ipv4_udp(bytes(ptp), 319, 319)
        ptp_packet = PcapngPacket(
            0, 0, 101, 123, 123, len(frame), len(frame), frame, True, False
        )
        report = audit_packet_records([sensor_packet, ptp_packet])
        observation, = report["sensor_time_observations"]
        self.assertEqual(observation["time_type"], 1)
        self.assertEqual(observation["packet_count"], 1)
        self.assertEqual(observation["sensor_timestamp_min_ns"],
                         1_700_000_000_000_000_000)
        co_observation, = report["ptp_sensor_timebase_co_observations"]
        self.assertEqual(co_observation["ptp_domain_number_observed"], 7)
        self.assertIn("does not bind", co_observation["interpretation"])
        self.assertIn("not proof", report["time_semantics"]["ptp_sensor_co_observation"])

    def test_sensor_observations_are_separated_by_time_type(self):
        report = audit_packet_records([
            record(mid360_payload(1, time_type=1), 56300),
            record(mid360_payload(1, time_type=2), 56300),
        ])
        self.assertEqual(
            {item["time_type"] for item in report["sensor_time_observations"]},
            {1, 2},
        )
        self.assertEqual(len(report["sensor_time_observations"]), 2)

    def test_sensor_time_observation_metadata_is_bounded(self):
        report = audit_packet_records([
            record(mid360_payload(1, time_type=1), 56300),
            record(mid360_payload(1, time_type=2), 56300),
        ], max_sensor_time_observations=1)
        self.assertEqual(len(report["sensor_time_observations"]), 1)
        self.assertEqual(report["sensor_time_observation_evictions"], 1)
        self.assertEqual(report["sensor_time_observation_limit"], 1)

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
