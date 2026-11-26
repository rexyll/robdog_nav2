#!/usr/bin/env python3
"""Decode link-layer captures into UDP datagrams without reassembling IP fragments.

Supported link types: Ethernet, Linux cooked v1/v2, raw IP, LINKTYPE_IPV4,
and LINKTYPE_IPV6. Checksums are not validated because NIC offload and capture
position can make checksum fields appear incomplete or invalid in host captures.
"""
from __future__ import annotations

from dataclasses import dataclass
import ipaddress
import struct

LINKTYPE_ETHERNET = 1
LINKTYPE_RAW = 101
LINKTYPE_LINUX_SLL = 113
LINKTYPE_IPV4 = 228
LINKTYPE_IPV6 = 229
LINKTYPE_LINUX_SLL2 = 276

ETH_P_IP = 0x0800
ETH_P_IPV6 = 0x86DD
VLAN_ETHERTYPES = {0x8100, 0x88A8, 0x9100}
IPPROTO_UDP = 17

MID360_LIDAR_PORTS = {
    56100: "control",
    56200: "push",
    56300: "point_cloud",
    56400: "imu",
    56500: "log",
}


class PacketDecodeError(ValueError):
    """Raised for malformed link/network/transport headers."""


@dataclass(frozen=True)
class UdpDatagram:
    link_type: int
    ip_version: int
    source_ip: str
    destination_ip: str
    source_port: int
    destination_port: int
    payload: bytes
    vlan_ids: tuple[int, ...] = ()
    capture_timestamp_ns: int | None = None
    interface_id: int = 0
    section_index: int = 0


@dataclass(frozen=True)
class FrameDecodeResult:
    disposition: str
    link_type: int
    detail: str = ""
    vlan_ids: tuple[int, ...] = ()
    ip_version: int | None = None
    source_ip: str | None = None
    destination_ip: str | None = None
    ip_protocol: int | None = None
    source_port: int | None = None
    destination_port: int | None = None
    fragment_offset: int = 0
    more_fragments: bool = False
    datagram: UdpDatagram | None = None


def _need(data: bytes, offset: int, size: int, what: str) -> None:
    if offset < 0 or size < 0 or offset + size > len(data):
        raise PacketDecodeError(f"truncated {what}")


def _link_payload(link_type: int, frame: bytes) -> tuple[int | None, int, tuple[int, ...], str]:
    """Return (ethertype, network offset, vlan IDs, unsupported reason)."""
    if link_type == LINKTYPE_ETHERNET:
        _need(frame, 0, 14, "Ethernet header")
        ethertype = struct.unpack_from("!H", frame, 12)[0]
        offset = 14
        vlans: list[int] = []
        while ethertype in VLAN_ETHERTYPES:
            if len(vlans) >= 8:
                raise PacketDecodeError("VLAN nesting exceeds 8 tags")
            _need(frame, offset, 4, "VLAN header")
            tci, next_type = struct.unpack_from("!HH", frame, offset)
            vlans.append(tci & 0x0FFF)
            ethertype = next_type
            offset += 4
        if ethertype < 0x0600:
            return None, offset, tuple(vlans), "IEEE 802.3 length/LLC frame not decoded"
        return ethertype, offset, tuple(vlans), ""

    if link_type == LINKTYPE_LINUX_SLL:
        _need(frame, 0, 16, "Linux cooked v1 header")
        protocol = struct.unpack_from("!H", frame, 14)[0]
        return protocol, 16, (), ""

    if link_type == LINKTYPE_LINUX_SLL2:
        _need(frame, 0, 20, "Linux cooked v2 header")
        protocol = struct.unpack_from("!H", frame, 0)[0]
        return protocol, 20, (), ""

    if link_type == LINKTYPE_RAW:
        _need(frame, 0, 1, "raw IP version")
        version = frame[0] >> 4
        if version == 4:
            return ETH_P_IP, 0, (), ""
        if version == 6:
            return ETH_P_IPV6, 0, (), ""
        raise PacketDecodeError(f"unknown raw IP version nibble: {version}")

    if link_type == LINKTYPE_IPV4:
        return ETH_P_IP, 0, (), ""
    if link_type == LINKTYPE_IPV6:
        return ETH_P_IPV6, 0, (), ""
    return None, 0, (), f"unsupported link_type {link_type}"


def _decode_ipv4(frame: bytes, offset: int, link_type: int, vlans: tuple[int, ...],
                 capture_timestamp_ns: int | None, interface_id: int,
                 section_index: int) -> FrameDecodeResult:
    _need(frame, offset, 20, "IPv4 base header")
    version_ihl = frame[offset]
    version, ihl = version_ihl >> 4, (version_ihl & 0x0F) * 4
    if version != 4 or ihl < 20:
        raise PacketDecodeError("invalid IPv4 version/IHL")
    _need(frame, offset, ihl, "IPv4 options/header")
    total_length = struct.unpack_from("!H", frame, offset + 2)[0]
    if total_length < ihl:
        raise PacketDecodeError("IPv4 total_length smaller than header")
    _need(frame, offset, total_length, "IPv4 packet")
    packet_end = offset + total_length
    flags_fragment = struct.unpack_from("!H", frame, offset + 6)[0]
    more_fragments = bool(flags_fragment & 0x2000)
    fragment_offset = flags_fragment & 0x1FFF
    protocol = frame[offset + 9]
    src = str(ipaddress.IPv4Address(frame[offset + 12:offset + 16]))
    dst = str(ipaddress.IPv4Address(frame[offset + 16:offset + 20]))
    base = dict(link_type=link_type, vlan_ids=vlans, ip_version=4,
                source_ip=src, destination_ip=dst, ip_protocol=protocol,
                fragment_offset=fragment_offset, more_fragments=more_fragments)
    if fragment_offset or more_fragments:
        return FrameDecodeResult("fragmented", detail="IPv4 fragment; reassembly is not implemented", **base)
    if protocol != IPPROTO_UDP:
        return FrameDecodeResult("non_udp", detail=f"IPv4 protocol {protocol}", **base)
    udp_offset = offset + ihl
    udp_len, = struct.unpack_from("!H", _checked_slice(frame, udp_offset, 8, "UDP header"), 4)
    if udp_len < 8 or udp_offset + udp_len > packet_end:
        raise PacketDecodeError("invalid UDP length for IPv4 payload")
    src_port, dst_port = struct.unpack_from("!HH", frame, udp_offset)
    payload = frame[udp_offset + 8:udp_offset + udp_len]
    datagram = UdpDatagram(link_type, 4, src, dst, src_port, dst_port, payload,
                           vlans, capture_timestamp_ns, interface_id, section_index)
    return FrameDecodeResult("udp", source_port=src_port, destination_port=dst_port,
                             datagram=datagram, **base)


def _checked_slice(frame: bytes, offset: int, size: int, what: str) -> bytes:
    _need(frame, offset, size, what)
    return frame[offset:offset + size]


def _decode_ipv6(frame: bytes, offset: int, link_type: int, vlans: tuple[int, ...],
                 capture_timestamp_ns: int | None, interface_id: int,
                 section_index: int) -> FrameDecodeResult:
    _need(frame, offset, 40, "IPv6 base header")
    if frame[offset] >> 4 != 6:
        raise PacketDecodeError("invalid IPv6 version")
    payload_length, next_header = struct.unpack_from("!H B", frame, offset + 4)
    if payload_length == 0:
        raise PacketDecodeError("IPv6 jumbograms (zero payload length) are unsupported")
    packet_end = offset + 40 + payload_length
    _need(frame, offset, 40 + payload_length, "IPv6 packet")
    src = str(ipaddress.IPv6Address(frame[offset + 8:offset + 24]))
    dst = str(ipaddress.IPv6Address(frame[offset + 24:offset + 40]))
    cursor = offset + 40
    fragment_offset = 0
    more_fragments = False
    # Extension headers: Hop-by-Hop, Routing, Fragment, AH, Destination Options,
    # Mobility, HIP, Shim6. ESP is intentionally opaque.
    extension_headers = {0, 43, 44, 51, 60, 135, 139, 140}
    count = 0
    while next_header in extension_headers:
        count += 1
        if count > 16:
            raise PacketDecodeError("IPv6 extension header chain exceeds 16 headers")
        if next_header == 44:
            _need(frame, cursor, 8, "IPv6 Fragment header")
            next_header, frag_field = struct.unpack_from("!B x H", frame, cursor)
            fragment_offset = (frag_field >> 3) & 0x1FFF
            more_fragments = bool(frag_field & 0x1)
            cursor += 8
            if fragment_offset or more_fragments:
                return FrameDecodeResult(
                    "fragmented", detail="IPv6 fragment; reassembly is not implemented",
                    link_type=link_type, vlan_ids=vlans, ip_version=6,
                    source_ip=src, destination_ip=dst, ip_protocol=next_header,
                    fragment_offset=fragment_offset, more_fragments=more_fragments,
                )
            continue
        _need(frame, cursor, 2, "IPv6 extension header")
        following = frame[cursor]
        if next_header == 51:  # AH length is in 32-bit words, minus 2.
            ext_len = (frame[cursor + 1] + 2) * 4
        else:
            ext_len = (frame[cursor + 1] + 1) * 8
        if ext_len < 8:
            raise PacketDecodeError("invalid IPv6 extension header length")
        _need(frame, cursor, ext_len, "IPv6 extension header body")
        cursor += ext_len
        next_header = following

    base = dict(link_type=link_type, vlan_ids=vlans, ip_version=6,
                source_ip=src, destination_ip=dst, ip_protocol=next_header,
                fragment_offset=fragment_offset, more_fragments=more_fragments)
    if next_header != IPPROTO_UDP:
        return FrameDecodeResult("non_udp", detail=f"IPv6 next_header {next_header}", **base)
    _need(frame, cursor, 8, "UDP header")
    src_port, dst_port, udp_len = struct.unpack_from("!HHH", frame, cursor)
    if udp_len < 8 or cursor + udp_len > packet_end:
        raise PacketDecodeError("invalid UDP length for IPv6 payload")
    payload = frame[cursor + 8:cursor + udp_len]
    datagram = UdpDatagram(link_type, 6, src, dst, src_port, dst_port, payload,
                           vlans, capture_timestamp_ns, interface_id, section_index)
    return FrameDecodeResult("udp", source_port=src_port, destination_port=dst_port,
                             datagram=datagram, **base)


def decode_link_packet(packet, *, capture_timestamp_ns: int | None = None,
                       interface_id: int = 0, section_index: int = 0) -> FrameDecodeResult:
    """Decode one packet record or raw frame to a UDP datagram/disposition."""
    link_type = int(packet.link_type) if hasattr(packet, "link_type") else int(packet[0])
    frame = bytes(packet.data) if hasattr(packet, "data") else bytes(packet[1])
    if hasattr(packet, "truncated") and packet.truncated:
        return FrameDecodeResult(
            "truncated_capture", link_type,
            "capture record is shorter than its original packet length",
        )
    if hasattr(packet, "capture_timestamp_ns") and capture_timestamp_ns is None:
        capture_timestamp_ns = packet.capture_timestamp_ns
    if hasattr(packet, "interface_id"):
        interface_id = packet.interface_id
    if hasattr(packet, "section_index"):
        section_index = packet.section_index
    ethertype, offset, vlans, reason = _link_payload(link_type, frame)
    if reason:
        return FrameDecodeResult("unsupported", link_type, reason, vlan_ids=vlans)
    if ethertype not in (ETH_P_IP, ETH_P_IPV6):
        return FrameDecodeResult("non_ip", link_type, f"ethertype {ethertype!r}", vlan_ids=vlans)
    if ethertype == ETH_P_IP:
        return _decode_ipv4(frame, offset, link_type, vlans, capture_timestamp_ns,
                            interface_id, section_index)
    return _decode_ipv6(frame, offset, link_type, vlans, capture_timestamp_ns,
                        interface_id, section_index)


def classify_mid360_datagram(datagram: UdpDatagram, *, lidar_ip: str | None = None,
                             host_ports: dict[int, str] | None = None) -> tuple[str, str] | None:
    """Classify known MID-360 device source ports or explicitly supplied host ports.

    The official device-side ports are 56200 push, 56300 point cloud, 56400 IMU,
    and 56500 log. Host receive ports are configurable and must be supplied by
    the caller; discovery port 56000 is not a point-cloud/IMU stream.
    """
    normalized_lidar_ip = str(ipaddress.ip_address(lidar_ip)) if lidar_ip is not None else None
    if normalized_lidar_ip is None or datagram.source_ip == normalized_lidar_ip:
        if datagram.source_port in MID360_LIDAR_PORTS:
            return MID360_LIDAR_PORTS[datagram.source_port], "lidar_to_host"
    if normalized_lidar_ip is None or datagram.destination_ip == normalized_lidar_ip:
        if host_ports and datagram.destination_port in host_ports:
            return host_ports[datagram.destination_port], "host_to_lidar"
    return None
