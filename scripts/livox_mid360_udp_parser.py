#!/usr/bin/env python3
"""Strict, dependency-free parser for Livox MID-360 point-cloud/IMU UDP payloads.

Protocol baseline: Livox MID-360 communication protocol, v1.4.12.
This parses one UDP application payload; it does not parse Ethernet/IP/UDP or PCAP.
"""
from __future__ import annotations

from dataclasses import dataclass
import struct
import zlib


HEADER_SIZE = 36
PROTOCOL_VERSION = 0
TIME_INTERVAL_UNIT_NS = 100  # protocol unit: 0.1 microsecond
SAMPLE_SIZE_BY_DATA_TYPE = {
    0: 24,  # six float32 IMU values
    1: 14,  # int32 XYZ + uint8 reflectivity + uint8 tag
    2: 8,   # int16 XYZ + uint8 reflectivity + uint8 tag
    3: 10,  # uint32 depth + uint16 theta/phi + uint8 reflectivity/tag
}
VALID_TIME_TYPES = {0, 1, 2}


class Mid360PacketError(ValueError):
    """Raised when a MID-360 UDP application payload violates the protocol."""


@dataclass(frozen=True)
class Mid360Packet:
    version: int
    length: int
    time_interval: int
    dot_num: int
    udp_cnt: int
    frame_cnt: int
    data_type: int
    time_type: int
    reserved: bytes
    crc32: int
    timestamp_ns: int
    data: bytes
    crc32_valid: bool

    @property
    def time_interval_ns(self) -> int:
        return self.time_interval * TIME_INTERVAL_UNIT_NS

    @property
    def sample_size(self) -> int:
        return SAMPLE_SIZE_BY_DATA_TYPE[self.data_type]


def crc32_livox(data: bytes) -> int:
    """CRC-32/ISO-HDLC (poly 0x04C11DB7, init/xorout FFFFFFFF, refin/refout)."""
    return zlib.crc32(data) & 0xFFFFFFFF


def parse_mid360_udp_payload(payload: bytes | bytearray | memoryview) -> Mid360Packet:
    """Parse and strictly validate one MID-360 UDP payload, including CRC-32.

    CRC coverage is timestamp (byte 28) concatenated with data (byte 36 onward).
    All multibyte protocol fields are little-endian.
    """
    packet = bytes(payload)
    if len(packet) < HEADER_SIZE:
        raise Mid360PacketError(
            f"payload shorter than {HEADER_SIZE}-byte header: {len(packet)}"
        )

    (
        version,
        length,
        time_interval,
        dot_num,
        udp_cnt,
        frame_cnt,
        data_type,
        time_type,
    ) = struct.unpack_from("<BHHHHBBB", packet, 0)

    if version != PROTOCOL_VERSION:
        raise Mid360PacketError(f"unsupported protocol version: {version}")
    if length != len(packet):
        raise Mid360PacketError(
            f"length field {length} does not match UDP payload size {len(packet)}"
        )
    if dot_num < 1:
        raise Mid360PacketError("dot_num must be at least 1")
    if data_type not in SAMPLE_SIZE_BY_DATA_TYPE:
        raise Mid360PacketError(f"unsupported data_type: {data_type}")
    if time_type not in VALID_TIME_TYPES:
        raise Mid360PacketError(f"unsupported time_type: {time_type}")

    reserved = packet[12:24]
    stored_crc32 = struct.unpack_from("<I", packet, 24)[0]
    timestamp_ns = struct.unpack_from("<Q", packet, 28)[0]
    data = packet[HEADER_SIZE:]

    expected_data_size = dot_num * SAMPLE_SIZE_BY_DATA_TYPE[data_type]
    if len(data) != expected_data_size:
        raise Mid360PacketError(
            f"data length {len(data)} does not match dot_num={dot_num} "
            f"and data_type={data_type} (expected {expected_data_size})"
        )

    calculated_crc32 = crc32_livox(packet[28:])
    if stored_crc32 != calculated_crc32:
        raise Mid360PacketError(
            f"CRC-32 mismatch: stored=0x{stored_crc32:08x}, "
            f"calculated=0x{calculated_crc32:08x}"
        )

    return Mid360Packet(
        version=version,
        length=length,
        time_interval=time_interval,
        dot_num=dot_num,
        udp_cnt=udp_cnt,
        frame_cnt=frame_cnt,
        data_type=data_type,
        time_type=time_type,
        reserved=reserved,
        crc32=stored_crc32,
        timestamp_ns=timestamp_ns,
        data=data,
        crc32_valid=True,
    )
