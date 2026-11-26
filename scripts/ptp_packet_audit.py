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
    minor_version_ptp: int
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
    current_utc_offset: int | None
    grandmaster_priority1: int | None
    grandmaster_clock_class: int | None
    grandmaster_clock_accuracy: int | None
    grandmaster_offset_scaled_log_variance: int | None
    grandmaster_priority2: int | None
    grandmaster_identity: str | None
    steps_removed: int | None
    time_source: int | None
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
    minor_version_ptp = second >> 4
    if version_ptp != 2:
        raise PtpPacketError(f"unsupported PTP versionPTP: {version_ptp}")
    if minor_version_ptp != 0:
        raise PtpPacketError(
            f"unsupported PTP minorVersionPTP: {minor_version_ptp}; parser is v2.0 only"
        )
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
    current_utc_offset = grandmaster_priority1 = None
    grandmaster_clock_class = grandmaster_clock_accuracy = None
    grandmaster_offset_scaled_log_variance = grandmaster_priority2 = None
    grandmaster_identity = steps_removed = time_source = None
    if message_type in {0, 1, 2, 8}:
        if message_length < 44:
            raise PtpPacketError("PTP timestamp message shorter than 44 bytes")
        timestamp_seconds, timestamp_nanoseconds, timestamp_ns = _timestamp(data[34:44])
    elif message_type in {3, 9, 10}:
        if message_length < 54:
            raise PtpPacketError("PTP response message shorter than 54 bytes")
        timestamp_seconds, timestamp_nanoseconds, timestamp_ns = _timestamp(data[34:44])
        requesting_identity = _port_identity(data[44:54])
    elif message_type == 11:
        if message_length < 64:
            raise PtpPacketError("PTP Announce message shorter than 64 bytes")
        current_utc_offset = struct.unpack_from("!h", data, 44)[0]
        grandmaster_priority1 = data[47]
        grandmaster_clock_class = data[48]
        grandmaster_clock_accuracy = data[49]
        grandmaster_offset_scaled_log_variance = struct.unpack_from("!H", data, 50)[0]
        grandmaster_priority2 = data[52]
        grandmaster_identity = data[53:61].hex(":")
        steps_removed = struct.unpack_from("!H", data, 61)[0]
        time_source = data[63]

    return PtpMessage(
        message_type=message_type,
        message_name=MESSAGE_NAMES.get(message_type, f"unknown_{message_type}"),
        transport_specific=transport_specific,
        version_ptp=version_ptp,
        minor_version_ptp=minor_version_ptp,
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
        current_utc_offset=current_utc_offset,
        grandmaster_priority1=grandmaster_priority1,
        grandmaster_clock_class=grandmaster_clock_class,
        grandmaster_clock_accuracy=grandmaster_clock_accuracy,
        grandmaster_offset_scaled_log_variance=grandmaster_offset_scaled_log_variance,
        grandmaster_priority2=grandmaster_priority2,
        grandmaster_identity=grandmaster_identity,
        steps_removed=steps_removed,
        time_source=time_source,
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


class PtpAnnounceAudit:
    """Observe Announce candidates and per-source sequence continuity.

    This deliberately does not run BMCA: packet captures alone do not expose
    the receiver's complete foreign-master records, profile rules, or port state.
    """

    def __init__(self, *, max_sources: int = 4096, max_changes: int = 32):
        if max_sources < 1 or max_changes < 1:
            raise ValueError("Announce audit bounds must be positive")
        self.max_sources = max_sources
        self.max_changes = max_changes
        self.counts = Counter()
        self.sources = {}
        self.candidates = {}
        self.gm_changes = []

    def observe(self, message: PtpMessage) -> None:
        if message.message_type != 11 or message.grandmaster_identity is None:
            return
        self.counts["announce_messages"] += 1
        source_key = (message.domain_number, message.source_port_identity)
        state = self.sources.get(source_key)
        if state is None:
            if len(self.sources) >= self.max_sources:
                self.sources.pop(next(iter(self.sources)))
                self.counts["evicted_announce_sources"] += 1
            state = {"last_sequence_id": None, "last_grandmaster_identity": None,
                     "message_count": 0}
            self.sources[source_key] = state
        previous_sequence = state["last_sequence_id"]
        if previous_sequence is None:
            state["last_sequence_id"] = message.sequence_id
        else:
            delta = (message.sequence_id - previous_sequence) & 0xFFFF
            if delta == 0:
                self.counts["announce_sequence_duplicate"] += 1
            elif delta < 0x8000:
                if delta > 1:
                    self.counts["announce_sequence_gap_observed"] += delta - 1
                    self.counts["announce_sequence_gap_events"] += 1
                else:
                    self.counts["announce_sequence_in_order"] += 1
                # Keep the forward high-water mark; a late/reordered packet
                # must not move it backwards and create a false later gap.
                state["last_sequence_id"] = message.sequence_id
            else:
                self.counts["announce_sequence_out_of_order_or_late"] += 1
        state["message_count"] += 1

        previous_gm = state["last_grandmaster_identity"]
        if previous_gm is not None and previous_gm != message.grandmaster_identity:
            self.counts["announcing_source_gm_identity_changes"] += 1
            if len(self.gm_changes) < self.max_changes:
                self.gm_changes.append({
                    "domain_number": message.domain_number,
                    "source_port_identity": message.source_port_identity,
                    "previous_grandmaster_identity": previous_gm,
                    "new_grandmaster_identity": message.grandmaster_identity,
                    "sequence_id": message.sequence_id,
                    "capture_timestamp_ns": message.capture_timestamp_ns,
                })
        state["last_grandmaster_identity"] = message.grandmaster_identity

        candidate_key = (message.domain_number, message.grandmaster_identity)
        candidate = self.candidates.get(candidate_key)
        if candidate is None:
            if len(self.candidates) >= self.max_sources:
                self.candidates.pop(next(iter(self.candidates)))
                self.counts["evicted_announce_candidates"] += 1
            candidate = {
                "domain_number": message.domain_number,
                "grandmaster_identity": message.grandmaster_identity,
                "announce_count": 0,
                "announcing_sources": set(),
            }
            self.candidates[candidate_key] = candidate
        candidate["announce_count"] += 1
        candidate["announcing_sources"].add(message.source_port_identity)
        candidate.update({
            "priority1": message.grandmaster_priority1,
            "clock_class": message.grandmaster_clock_class,
            "clock_accuracy": message.grandmaster_clock_accuracy,
            "offset_scaled_log_variance": message.grandmaster_offset_scaled_log_variance,
            "priority2": message.grandmaster_priority2,
            "steps_removed": message.steps_removed,
            "time_source": message.time_source,
            "last_sequence_id": message.sequence_id,
            "last_capture_timestamp_ns": message.capture_timestamp_ns,
        })

    def report(self) -> dict:
        candidates = []
        for candidate in self.candidates.values():
            item = dict(candidate)
            item["announcing_sources"] = sorted(item["announcing_sources"])
            candidates.append(item)
        candidates.sort(key=lambda item: (item["domain_number"],
                                          item["grandmaster_identity"]))
        sources = [{
            "domain_number": domain,
            "source_port_identity": source,
            **state,
        } for (domain, source), state in sorted(self.sources.items())]
        return {
            "counts": dict(self.counts),
            "observed_candidates": candidates,
            "announcing_sources": sources,
            "observed_source_gm_identity_changes": list(self.gm_changes),
            "selection_status": "NOT_INFERRED_FROM_PACKET_CAPTURE",
            "interpretation": (
                "Announce candidate declarations and capture-order sequence observations only; "
                "not BMCA execution, active GM selection, port state, or PTP lock. "
                "Sequence gaps may reflect capture loss/filtering/reordering."
            ),
        }


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
        self.grandmasters = Counter()
        self.announce_audit = PtpAnnounceAudit(max_sources=max_pending)
        self.pending_sync = {}
        self.pending_delay_req = {}
        self.pending_pdelay_req = {}
        self.pending_pdelay_resp = {}
        self.exchange_samples = []
        self.max_exchange_samples = 32

    @staticmethod
    def _timestamp_fields(message: PtpMessage) -> dict:
        return {
            "timestamp_seconds": message.timestamp_seconds,
            "timestamp_nanoseconds": message.timestamp_nanoseconds,
            "timestamp_ns": message.timestamp_ns,
            "correction_field_raw": message.correction_field_raw,
            "correction_field_ns": message.correction_field_ns,
            "capture_timestamp_ns": message.capture_timestamp_ns,
            "source_port_identity": message.source_port_identity,
            "sequence_id": message.sequence_id,
            "transport": message.transport,
            "interface_id": message.interface_id,
            "section_index": message.section_index,
        }

    def _record_exchange(self, kind: str, messages: dict[str, PtpMessage],
                         *, domain_number: int, sequence_id: int) -> None:
        self.counts[f"{kind}_timestamp_exchanges"] += 1
        if len(self.exchange_samples) >= self.max_exchange_samples:
            self.counts["evicted_timestamp_exchange_samples"] += 1
            return
        self.exchange_samples.append({
            "exchange": kind,
            "domain_number": domain_number,
            "sequence_id": sequence_id,
            "messages": {
                name: self._timestamp_fields(message)
                for name, message in messages.items()
            },
            "interpretation": (
                "Raw message timestamps and correctionField values only; no offset, "
                "mean path delay, asymmetry, or clock accuracy is calculated."
            ),
        })

    def _store(self, table: dict, key: tuple, label: str,
               message: PtpMessage) -> None:
        if key in table:
            self.counts[f"duplicate_{label}"] += 1
        table[key] = message
        if len(table) > self.max_pending:
            table.pop(next(iter(table)))
            self.counts[f"expired_pending_{label}"] += 1

    def observe(self, message: PtpMessage) -> None:
        self.counts["messages"] += 1
        self.message_types[message.message_name] += 1
        self.domains[str(message.domain_number)] += 1
        self.transports[message.transport] += 1
        if message.grandmaster_identity is not None:
            self.grandmasters[message.grandmaster_identity] += 1
        self.announce_audit.observe(message)
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
                self._store(self.pending_sync, key, "sync", message)
            else:
                self.counts["one_step_sync"] += 1
        elif message.message_type == 8:
            sync_message = self.pending_sync.pop(key, None)
            if sync_message is not None:
                self.counts["sync_follow_up_pairs"] += 1
                self._record_exchange(
                    "two_step_sync_follow_up",
                    {"sync": sync_message, "follow_up": message},
                    domain_number=message.domain_number,
                    sequence_id=message.sequence_id,
                )
            else:
                self.counts["orphan_follow_up"] += 1
        elif message.message_type == 1:
            self._store(self.pending_delay_req, key, "delay_req", message)
        elif message.message_type == 9 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            request_message = self.pending_delay_req.pop(request_key, None)
            if request_message is not None:
                self.counts["delay_req_resp_pairs"] += 1
                self._record_exchange(
                    "end_to_end_delay_req_resp",
                    {"delay_req": request_message, "delay_resp": message},
                    domain_number=message.domain_number,
                    sequence_id=message.sequence_id,
                )
            else:
                self.counts["orphan_delay_resp"] += 1
        elif message.message_type == 2:
            self._store(self.pending_pdelay_req, key, "pdelay_req", message)
        elif message.message_type == 3 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            request_message = self.pending_pdelay_req.pop(request_key, None)
            if request_message is not None:
                self.counts["pdelay_req_resp_pairs"] += 1
                self._store(self.pending_pdelay_resp, request_key,
                            "pdelay_resp", (request_message, message))
                self._record_exchange(
                    "peer_delay_req_resp",
                    {"pdelay_req": request_message, "pdelay_resp": message},
                    domain_number=message.domain_number,
                    sequence_id=message.sequence_id,
                )
            else:
                self.counts["orphan_pdelay_resp"] += 1
        elif message.message_type == 10 and message.requesting_port_identity:
            request_key = (message.domain_number,
                           message.requesting_port_identity, message.sequence_id)
            response_exchange = self.pending_pdelay_resp.pop(request_key, None)
            if response_exchange is not None:
                request_message, response_message = response_exchange
                self.counts["pdelay_resp_follow_up_pairs"] += 1
                self._record_exchange(
                    "peer_delay_response_follow_up",
                    {"pdelay_req": request_message,
                     "pdelay_resp": response_message,
                     "pdelay_resp_follow_up": message},
                    domain_number=message.domain_number,
                    sequence_id=message.sequence_id,
                )
            else:
                self.counts["orphan_pdelay_resp_follow_up"] += 1

    def report(self) -> dict:
        return {
            "message_count": self.counts["messages"],
            "message_types": dict(self.message_types),
            "domain_counts": dict(self.domains),
            "transport_counts": dict(self.transports),
            "grandmaster_identity_counts": dict(self.grandmasters),
            "announce_audit": self.announce_audit.report(),
            "correlations": dict(self.counts),
            "timestamp_exchange_samples": list(self.exchange_samples),
            "timestamp_exchange_sample_limit": self.max_exchange_samples,
            "unmatched_at_end": {
                "two_step_sync_waiting_follow_up": len(self.pending_sync),
                "delay_req_waiting_delay_resp": len(self.pending_delay_req),
                "pdelay_req_waiting_response": len(self.pending_pdelay_req),
                "pdelay_resp_waiting_follow_up": len(self.pending_pdelay_resp),
            },
            "interpretation": (
                "PTP message presence and sequence-ID pairing with raw timestamp/"
                "correctionField evidence only; capture timestamps are not event "
                "timestamps and no offset, path delay, asymmetry, or clock accuracy "
                "is calculated; not proof of PTP lock, GM selection, PHC timestamping, "
                "or end-to-end synchronization"
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
                "minor_version_ptp": message.minor_version_ptp,
                "source_port_identity": message.source_port_identity,
                "sequence_id": message.sequence_id,
                "two_step": message.two_step,
                "grandmaster_identity": message.grandmaster_identity,
                "grandmaster_priority1": message.grandmaster_priority1,
                "grandmaster_clock_class": message.grandmaster_clock_class,
                "grandmaster_priority2": message.grandmaster_priority2,
                "steps_removed": message.steps_removed,
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
