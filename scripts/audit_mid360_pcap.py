#!/usr/bin/env python3
"""Stream PCAP/PCAPNG captures and audit MID-360 UDP streams and payloads."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

from livox_mid360_udp_parser import Mid360PacketError, parse_mid360_udp_payload
from network_packet_decoder import (
    PacketDecodeError, classify_mid360_datagram, decode_link_packet,
)
from pcapng_reader import PcapngError, iter_capture_packets


def audit_packet_records(packets, *, lidar_ip=None, host_ports=None):
    counts = Counter()
    streams = Counter()
    payload_errors = Counter()
    data_types = Counter()
    time_types = Counter()
    interfaces = defaultdict(lambda: {"packets": 0, "capture_timestamp_min_ns": None,
                                       "capture_timestamp_max_ns": None})
    samples = []

    for packet in packets:
        counts["capture_packets"] += 1
        interface_key = f"{packet.section_index}:{packet.interface_id}"
        iface = interfaces[interface_key]
        iface["packets"] += 1
        ts = packet.capture_timestamp_ns
        if ts is not None:
            iface["capture_timestamp_min_ns"] = (
                ts if iface["capture_timestamp_min_ns"] is None
                else min(iface["capture_timestamp_min_ns"], ts)
            )
            iface["capture_timestamp_max_ns"] = (
                ts if iface["capture_timestamp_max_ns"] is None
                else max(iface["capture_timestamp_max_ns"], ts)
            )

        if packet.truncated:
            counts["truncated_capture_packets"] += 1
            continue
        try:
            decoded = decode_link_packet(packet)
        except PacketDecodeError:
            counts["malformed_link_or_network_packets"] += 1
            continue

        counts[f"disposition_{decoded.disposition}"] += 1
        if decoded.disposition != "udp" or decoded.datagram is None:
            continue
        datagram = decoded.datagram
        stream = classify_mid360_datagram(
            datagram, lidar_ip=lidar_ip, host_ports=host_ports
        )
        if stream is None:
            counts["unclassified_udp_datagrams"] += 1
            continue
        stream_name, direction = stream
        streams[stream_name] += 1
        counts[f"direction_{direction}"] += 1

        if stream_name not in {"point_cloud", "imu"}:
            continue
        try:
            payload = parse_mid360_udp_payload(datagram.payload)
        except Mid360PacketError:
            payload_errors[stream_name] += 1
            continue

        counts[f"valid_{stream_name}_payloads"] += 1
        data_types[str(payload.data_type)] += 1
        time_types[str(payload.time_type)] += 1
        expected_type = 0 if stream_name == "imu" else None
        if expected_type is not None and payload.data_type != expected_type:
            counts["stream_data_type_mismatch"] += 1
        elif stream_name == "point_cloud" and payload.data_type == 0:
            counts["stream_data_type_mismatch"] += 1
        if len(samples) < 5:
            samples.append({
                "stream": stream_name,
                "source": f"{datagram.source_ip}:{datagram.source_port}",
                "destination": f"{datagram.destination_ip}:{datagram.destination_port}",
                "data_type": payload.data_type,
                "time_type": payload.time_type,
                "sensor_timestamp_ns": payload.timestamp_ns,
                "capture_timestamp_ns": datagram.capture_timestamp_ns,
                "interface_id": datagram.interface_id,
                "section_index": datagram.section_index,
            })

    return {
        "capture_packets": counts["capture_packets"],
        "dispositions": {k.removeprefix("disposition_"): v for k, v in counts.items()
                         if k.startswith("disposition_")},
        "truncated_capture_packets": counts["truncated_capture_packets"],
        "malformed_link_or_network_packets": counts["malformed_link_or_network_packets"],
        "unclassified_udp_datagrams": counts["unclassified_udp_datagrams"],
        "streams": dict(streams),
        "payload_errors": dict(payload_errors),
        "valid_payloads": {
            "point_cloud": counts["valid_point_cloud_payloads"],
            "imu": counts["valid_imu_payloads"],
        },
        "stream_data_type_mismatch": counts["stream_data_type_mismatch"],
        "data_types": dict(data_types),
        "time_types": dict(time_types),
        "direction_counts": {
            "lidar_to_host": counts["direction_lidar_to_host"],
            "host_to_lidar": counts["direction_host_to_lidar"],
        },
        "interfaces": dict(interfaces),
        "sample_datagrams": samples,
        "time_semantics": {
            "capture_timestamp_ns": "capture-file timestamp; not LiDAR sensor time",
            "sensor_timestamp_ns": "MID-360 payload timestamp; clock domain follows time_type",
            "delta_is_network_latency": False,
        },
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="classic PCAP or PCAPNG file")
    parser.add_argument("--lidar-ip", help="optional exact MID-360 device IP filter")
    parser.add_argument("--host-point-cloud-port", type=int,
                        help="configured host receive port for point-cloud stream")
    parser.add_argument("--host-imu-port", type=int,
                        help="configured host receive port for IMU stream")
    args = parser.parse_args(argv)

    host_ports = {}
    if args.host_point_cloud_port is not None:
        host_ports[args.host_point_cloud_port] = "point_cloud"
    if args.host_imu_port is not None:
        host_ports[args.host_imu_port] = "imu"

    try:
        with args.capture.open("rb") as stream:
            report = audit_packet_records(
                iter_capture_packets(stream), lidar_ip=args.lidar_ip,
                host_ports=host_ports or None,
            )
    except (OSError, PcapngError) as exc:
        print(f"capture audit failed: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
