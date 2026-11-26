#!/usr/bin/env python3
"""Stream PCAP/PCAPNG captures and audit MID-360 UDP streams and payloads."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

from livox_mid360_udp_parser import Mid360PacketError, parse_mid360_udp_payload
from livox_udp_sequence_audit import LivoxUdpSequenceAudit
from network_packet_decoder import (
    PacketDecodeError, classify_mid360_datagram, decode_link_packet,
)
from pcapng_reader import PcapngError, iter_capture_packets
from ptp_packet_audit import PtpCaptureAudit


def audit_packet_records(packets, *, lidar_ip=None, host_ports=None,
                         max_sensor_time_observations=4096,
                         max_ptp_sensor_co_observations=4096):
    if max_sensor_time_observations < 1 or max_ptp_sensor_co_observations < 1:
        raise ValueError("sensor time observation bounds must be positive")
    counts = Counter()
    streams = Counter()
    payload_errors = Counter()
    data_types = Counter()
    time_types = Counter()
    sequence_audits = {}
    sensor_time_observations = {}
    ptp_audit = PtpCaptureAudit()
    interfaces = defaultdict(lambda: {"packets": 0, "capture_timestamp_min_ns": None,
                                       "capture_timestamp_max_ns": None})
    samples = []

    for packet in packets:
        ptp_audit.observe_packet(packet)
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
        observation_key = (
            datagram.section_index, datagram.interface_id, stream_name,
            datagram.source_ip, datagram.source_port, datagram.destination_ip,
            datagram.destination_port, payload.time_type,
        )
        observation = sensor_time_observations.get(observation_key)
        if observation is None:
            if len(sensor_time_observations) >= max_sensor_time_observations:
                sensor_time_observations.pop(next(iter(sensor_time_observations)))
                counts["evicted_sensor_time_observations"] += 1
            observation = {
                "section_index": datagram.section_index,
                "interface_id": datagram.interface_id,
                "stream": stream_name,
                "source": f"{datagram.source_ip}:{datagram.source_port}",
                "destination": f"{datagram.destination_ip}:{datagram.destination_port}",
                "time_type": payload.time_type,
                "packet_count": 0,
                "sensor_timestamp_min_ns": payload.timestamp_ns,
                "sensor_timestamp_max_ns": payload.timestamp_ns,
                "capture_timestamp_min_ns": datagram.capture_timestamp_ns,
                "capture_timestamp_max_ns": datagram.capture_timestamp_ns,
            }
            sensor_time_observations[observation_key] = observation
        observation["packet_count"] += 1
        observation["sensor_timestamp_min_ns"] = min(
            observation["sensor_timestamp_min_ns"], payload.timestamp_ns
        )
        observation["sensor_timestamp_max_ns"] = max(
            observation["sensor_timestamp_max_ns"], payload.timestamp_ns
        )
        if datagram.capture_timestamp_ns is not None:
            old_min = observation["capture_timestamp_min_ns"]
            old_max = observation["capture_timestamp_max_ns"]
            observation["capture_timestamp_min_ns"] = (
                datagram.capture_timestamp_ns if old_min is None
                else min(old_min, datagram.capture_timestamp_ns)
            )
            observation["capture_timestamp_max_ns"] = (
                datagram.capture_timestamp_ns if old_max is None
                else max(old_max, datagram.capture_timestamp_ns)
            )
        sequence_event = None
        if stream_name == "point_cloud" and direction == "lidar_to_host":
            flow_key = (
                datagram.section_index, datagram.interface_id,
                datagram.source_ip, datagram.source_port,
                datagram.destination_ip, datagram.destination_port,
            )
            sequence_audit = sequence_audits.setdefault(
                flow_key, LivoxUdpSequenceAudit()
            )
            sequence_event = sequence_audit.observe(
                udp_cnt=payload.udp_cnt, frame_cnt=payload.frame_cnt,
                data_type=payload.data_type, timestamp_ns=payload.timestamp_ns,
            )
            counts[f"sequence_{sequence_event}"] += 1
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
                "udp_cnt": payload.udp_cnt,
                "frame_cnt": payload.frame_cnt,
                "sequence_event": sequence_event,
                "time_type": payload.time_type,
                "sensor_timestamp_ns": payload.timestamp_ns,
                "capture_timestamp_ns": datagram.capture_timestamp_ns,
                "interface_id": datagram.interface_id,
                "section_index": datagram.section_index,
            })

    ptp_report = ptp_audit.report()
    sensor_observations = [
        dict(value) for _, value in sorted(sensor_time_observations.items())
    ]
    co_observations = []
    for sensor in sensor_observations:
        sensor_min = sensor["capture_timestamp_min_ns"]
        sensor_max = sensor["capture_timestamp_max_ns"]
        if sensor_min is None or sensor_max is None:
            continue
        for ptp_window in ptp_report.get("ptp_capture_windows", []):
            if (sensor["section_index"] != ptp_window["section_index"]
                    or sensor["interface_id"] != ptp_window["interface_id"]):
                continue
            if (sensor_min <= ptp_window["capture_timestamp_max_ns"]
                    and ptp_window["capture_timestamp_min_ns"] <= sensor_max):
                if len(co_observations) >= max_ptp_sensor_co_observations:
                    counts["truncated_ptp_sensor_co_observations"] += 1
                    continue
                co_observations.append({
                    "sensor_stream": sensor["stream"],
                    "sensor_source": sensor["source"],
                    "sensor_destination": sensor["destination"],
                    "sensor_time_type": sensor["time_type"],
                    "section_index": sensor["section_index"],
                    "interface_id": sensor["interface_id"],
                    "ptp_domain_number_observed": ptp_window["domain_number"],
                    "ptp_capture_messages": ptp_window["ptp_messages"],
                    "interpretation": (
                        "Co-observed on the same capture interface with overlapping "
                        "capture-file time windows only; this does not bind the sensor "
                        "clock to this PTP domain or prove synchronization."
                    ),
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
        "ptp_audit": ptp_report,
        "sensor_time_observations": sensor_observations,
        "sensor_time_observation_evictions": counts["evicted_sensor_time_observations"],
        "sensor_time_observation_limit": max_sensor_time_observations,
        "ptp_sensor_timebase_co_observations": co_observations,
        "ptp_sensor_co_observation_truncated": counts[
            "truncated_ptp_sensor_co_observations"
        ],
        "ptp_sensor_co_observation_limit": max_ptp_sensor_co_observations,
        "point_cloud_sequence_audits": {
            "|".join(map(str, key)): audit.report()
            for key, audit in sequence_audits.items()
        },
        "sequence_event_counts": {
            key.removeprefix("sequence_"): value
            for key, value in counts.items() if key.startswith("sequence_")
        },
        "sample_datagrams": samples,
        "time_semantics": {
            "capture_timestamp_ns": "capture-file timestamp; not LiDAR sensor time",
            "sensor_timestamp_ns": "MID-360 payload timestamp; clock domain follows time_type",
            "time_type_1": "sensor declares PTP/gPTP master time; observed PTP domain is not inferred",
            "ptp_sensor_co_observation": (
                "same capture section/interface and overlapping capture-file time windows; "
                "not proof of clock-domain binding, lock, or synchronization"
            ),
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
