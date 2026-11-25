#!/usr/bin/env python3
"""Parse IEEE 1588-2019 PTPv2 messages in UDP/IP or Ethernet L2 captures."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import struct

from network_packet_decoder import (
    ETH_P_IP, ETH_P_IPV6, LINKTYPE_ETHERNET, PacketDecodeError,
    _link_payload, decode_link_packet,
)

ETH_P_PTP = 0x88F7
PTP_EVENT_PORT = 319
PTP_GENERAL_PORT = 320
PTP_HEADER_SIZE = 34
MESSAGE_NAMES = {
    0: "sync",
    1: "delay_req",
    2: "pdelay_req",
    3: "pdelay_resp",
    8: "follow_up",
    9: "delay_resp",
    10: "pdelay_resp_follow_up",
    11: "announce",
    12: "signaling",
    13: "management",
}
EVENT_MESSAGE_TYPES = {0, 1, 2, 3}
GENERAL_MESSAGE_TYPES = {8, 9, 10, 11, 12, 13}


class PtpPacketError(ValueError):
    """Raised for malformed or unsupported PTPv2 message payloads."""


@dataclass(frozen=True)
class PtpMessage:
    message_type: int
    message_name: str
    transport_specific: int
    version_ptp: int
    message_length: int
    domain_number: int
    flags: int
    two_step: bool
    correction_field_raw: int
    correction_field_ns: float
    source_port_identity: str
    sequence_id: int
    control_field: int
    log_message_interval: int
    timestamp_seconds: int | None
    timestamp_nanoseconds: int | None
    timestamp_ns: int | None
    requesting_port_identity: str | None
    transport: str
    capture_timestamp_ns: int | None
    interface_id: int
    section_index: int
    vlan_ids: tuple[int, ...]
    udp_source_port: int | None
    udp_destination_port: int | None


def _port_identity(raw: bytes) -> str:
    if len(raw) != 10:
        raise PtpPacketError("port identity must be exactly 10 bytes")
    clock = raw[:8].hex(":")
    port = struct.unpack_from("!H", raw, 8)[0]
    return f"{clock}/{port}"


def _timestamp(raw: bytes) -> tuple[int, int, int]:
    if len(raw) != 10:
        raise PtpPacketError("PTP timestamp must be exactly 10 bytes")
    seconds = int.from_bytes(raw[:6], "big")
    nanoseconds = struct.unpack_from("!I", raw, 6)[0]
    if nanoseconds >= 1_000_000_000:
        raise PtpPacketError("PTP timestamp nanoseconds field is out of range")
    return seconds, nanoseconds, seconds * 1_000_000_000 + nanoseconds


def parse_ptp_v2_message(payload: bytes, *, transport: str,
                         capture_timestamp_ns: int | None = None,
                         interface_id: int = 0, section_index: int = 0,
                         vlan_ids: tuple[int, ...] = (),
                         udp_source_port: int | None = None,
                         udp_destination_port: int | None = None) -> PtpMessage:
    """Parse the common PTPv2 header and the timing fields needed for correlation.

    PTP timestamps are kept as PTP seconds/nanoseconds; no UTC/GPS conversion is
    attempted. messageLength, not Ethernet padding, bounds the parsed PTP message.
    """
    data = bytes(payload)
    if len(data) < PTP_HEADER_SIZE:
        raise PtpPacketError("PTP payload shorter than 34-byte common header")
    first, second = data[0], data[1]
    message_type = first & 0x0F
    transport_specific = first >> 4
    version_ptp = second & 0x0F
    if version_ptp != 2:
        raise PtpPacketError(f"unsupported PTP versionPTP: {version_ptp}")
    message_length = struct.unpack_from("!H", data, 2)[0]
    if message_length < PTP_HEADER_SIZE or message_length > len(data):
        raise PtpPacketError("PTP messageLength outside captured payload")
    data = data[:message_length]
    domain_number = data[4]
    flags = struct.unpack_from("!H", data, 6)[0]
    correction_raw = struct.unpack_from("!q", data, 8)[0]
    source_identity = _port_identity(data[20:30])
    sequence_id = struct.unpack_from("!H", data, 30)[0]
    control_field = data[32]
    log_interval = struct.unpack_from("!b", data, 33)[0]

    timestamp_seconds = timestamp_nanoseconds = timestamp_ns = None
    requesting_identity = None
    if message_type in {0, 1, 2, 8}:
        if message_length < 44:
            raise PtpPacketError("PTP timestamp message shorter than 44 bytes")
        timestamp_seconds, timestamp_nanoseconds, timestamp_ns = _timestamp(data[34:44])
    elif message_type in {3, 9, 10}:
        if message_length < 54:
            raise PtpPacketError("PTP response message shorter than 54 bytes")
        timestamp_seconds, timestamp_nanoseconds, timestamp_ns = _timestamp(data[34:44])
        requesting_identity = _port_identity(data[44:54])

    return PtpMessage(
        message_type=message_type,
        message_name=MESSAGE_NAMES.get(message_type, f"unknown_{message_type}"),
        transport_specific=transport_specific,
        version_ptp=version_ptp,
        message_length=message_length,
        domain_number=domain_number,
        flags=flags,
        two_step=bool(flags & 0x0200),
        correction_field_raw=correction_raw,
        correction_field_ns=correction_raw / 65536.0,
        source_port_identity=source_identity,
        sequence_id=sequence_id,
        control_field=control_field,
        log_message_interval=log_interval,
        timestamp_seconds=timestamp_seconds,
        timestamp_nanoseconds=timestamp_nanoseconds,
        timestamp_ns=timestamp_ns,
        requesting_port_identity=requesting_identity,
        transport=transport,
        capture_timestamp_ns=capture_timestamp_ns,
        interface_id=interface_id,
        section_index=section_index,
        vlan_ids=vlan_ids,
        udp_source_port=udp_source_port,
        udp_destination_port=udp_destination_port,
    )


def extract_ptp_message(packet) -> PtpMessage | None:
    """Extract PTPv2 over UDP ports 319/320 or Ethernet EtherType 0x88F7."""
    link_type = int(packet.link_type)
    frame = bytes(packet.data)
    if getattr(packet, "truncated", False):
        return None
    timestamp = getattr(packet, "capture_timestamp_ns", None)
    interface_id = getattr(packet, "interface_id", 0)
    section_index = getattr(packet, "section_index", 0)

    ethertype, offset, vlans, reason = _link_payload(link_type, frame)
    if reason:
        return None
    if ethertype == ETH_P_PTP:
        return parse_ptp_v2_message(
            frame[offset:], transport="ethernet_l2",
            capture_timestamp_ns=timestamp, interface_id=interface_id,
            section_index=section_index, vlan_ids=vlans,
        )
    if ethertype not in {ETH_P_IP, ETH_P_IPV6}:
        return None

    decoded = decode_link_packet(packet)
    if decoded.disposition != "udp" or decoded.datagram is None:
        return None
    datagram = decoded.datagram
    if PTP_EVENT_PORT not in {datagram.source_port, datagram.destination_port} and (
        PTP_GENERAL_PORT not in {datagram.source_port, datagram.destination_port}
    ):
        return None
    port = PTP_EVENT_PORT if PTP_EVENT_PORT in {
        datagram.source_port, datagram.destination_port
    } else PTP_GENERAL_PORT
    return parse_ptp_v2_message(
        datagram.payload, transport=f"udp/{port}",
        capture_timestamp_ns=datagram.capture_timestamp_ns,
        interface_id=datagram.interface_id, section_index=datagram.section_index,
        vlan_ids=datagram.vlan_ids, udp_source_port=datagram.source_port,
        udp_destination_port=datagram.destination_port,
    )


class PtpCorrelationAudit:
    """Bounded message-pair correlator; reports presence/pairing, not clock accuracy."""

    def __init__(self, *, max_pending: int = 4096):
        if max_pending < 1:
            raise ValueError("max_pending must be positive")
        self.max_pending = max_pending
        self.counts = Counter()
        self.message_types = Counter()
        self.domains = Counter()
        self.transports = Counter()
        self.pending_sync = {}
        self.pending_delay_req = {}
        self.pending_pdelay_req = {}
        self.pending_pdelay_resp = {}

    def _store(self, table: dict, key: tuple, label: str) -> None:
        if key in table:
            self.counts[f"duplicate_{label}"] += 1
        table[key] = True
        if len(table) > self.max_pending:
            table.pop(next(iter(table)))
            self.counts[f"expired_pending_{label}"] += 1

    def observe(self, message: PtpMessage) -> None:
        self.counts["messages"] += 1
        self.message_types[message.message_name] += 1
        self.domains[str(message.domain_number)] += 1
        self.transports[message.transport] += 1
        if message.transport.startswith("udp/"):
            expected = (
                PTP_EVENT_PORT if message.message_type in EVENT_MESSAGE_TYPES
                else PTP_GENERAL_PORT if message.message_type in GENERAL_MESSAGE_TYPES
                else None
            )
            actual = int(message.transport.split("/", 1)[1])
            if expected is not None and actual != expected:
                self.counts["udp_port_message_type_mismatch"] += 1

        key = (message.domain_number, message.source_port_identity,
               message.sequence_id)
        if message.message_type == 0:
            if message.two_step:
                self._store(self.pending_sync, key, "sync")
            else:
                self.counts["one_step_sync"] += 1
        elif message.message_type == 8:
            if self.pending_sync.pop(key, None) is not None:
                self.counts["sync_follow_up_pairs"] += 1
            else:
                self.counts["orphan_follow_up"] += 1
        elif message.message_type == 1:
            self._store(self.pending_delay_req, key, "delay_req")
        elif message.message_type == 9 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            if self.pending_delay_req.pop(request_key, None) is not None:
                self.counts["delay_req_resp_pairs"] += 1
            else:
                self.counts["orphan_delay_resp"] += 1
        elif message.message_type == 2:
            self._store(self.pending_pdelay_req, key, "pdelay_req")
        elif message.message_type == 3 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            if self.pending_pdelay_req.pop(request_key, None) is not None:
                self.counts["pdelay_req_resp_pairs"] += 1
                self._store(self.pending_pdelay_resp, request_key, "pdelay_resp")
            else:
                self.counts["orphan_pdelay_resp"] += 1
        elif message.message_type == 10 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            if self.pending_pdelay_resp.pop(request_key, None) is not None:
                self.counts["pdelay_resp_follow_up_pairs"] += 1
            else:
                self.counts["orphan_pdelay_resp_follow_up"] += 1

    def report(self) -> dict:
        return {
            "message_count": self.counts["messages"],
            "message_types": dict(self.message_types),
            "domain_counts": dict(self.domains),
            "transport_counts": dict(self.transports),
            "correlations": dict(self.counts),
            "unmatched_at_end": {
                "two_step_sync_waiting_follow_up": len(self.pending_sync),
                "delay_req_waiting_delay_resp": len(self.pending_delay_req),
                "pdelay_req_waiting_response": len(self.pending_pdelay_req),
                "pdelay_resp_waiting_follow_up": len(self.pending_pdelay_resp),
            },
            "interpretation": (
                "PTP message presence and sequence-ID pairing only; not proof of "
                "PTP lock, GM selection, PHC timestamping, offset, path delay, "
                "clock accuracy, or end-to-end synchronization"
            ),
        }


class PtpCaptureAudit:
    """Streaming capture wrapper around the bounded PTP message correlator."""

    def __init__(self):
        self.audit = PtpCorrelationAudit()
        self.counts = Counter()
        self.samples = []

    def observe_packet(self, packet) -> None:
        if getattr(packet, "truncated", False):
            self.counts["truncated_capture_records"] += 1
            return
        try:
            message = extract_ptp_message(packet)
        except (PacketDecodeError, PtpPacketError):
            self.counts["malformed_ptp_candidates"] += 1
            return
        if message is None:
            return
        self.audit.observe(message)
        if len(self.samples) < 8:
            self.samples.append({
                "message_type": message.message_name,
                "domain_number": message.domain_number,
                "transport_specific": message.transport_specific,
                "source_port_identity": message.source_port_identity,
                "sequence_id": message.sequence_id,
                "two_step": message.two_step,
                "transport": message.transport,
                "capture_timestamp_ns": message.capture_timestamp_ns,
                "timestamp_seconds": message.timestamp_seconds,
                "timestamp_nanoseconds": message.timestamp_nanoseconds,
                "vlan_ids": list(message.vlan_ids),
            })

    def report(self) -> dict:
        report = self.audit.report()
        report["malformed_ptp_candidates"] = self.counts["malformed_ptp_candidates"]
        report["truncated_capture_records"] = self.counts["truncated_capture_records"]
        report["sample_messages"] = list(self.samples)
        report["timestamp_semantics"] = (
            "PTP message-body timestamps are not converted to UTC/GPS; capture timestamps "
            "are file timestamps; no packet-time delta is reported as path delay"
        )
        return report


def audit_ptp_records(packets) -> dict:
    audit = PtpCaptureAudit()
    for packet in packets:
        audit.observe_packet(packet)
    return audit.report()
