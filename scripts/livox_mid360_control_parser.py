#!/usr/bin/env python3
"""Strict decoder for MID-360 control/status UDP messages (protocol v1.4.12)."""
from __future__ import annotations

from dataclasses import dataclass
import struct
import zlib

SOF = 0xAA
HEADER_SIZE = 24
CMD_LIDAR_INFO_QUERY = 0x0101
CMD_LIDAR_INFO_PUSH = 0x0102

TIME_KEYS = {
    0x8009: ("local_time_now", 8, "uint64", "ns"),
    0x800A: ("last_sync_time", 8, "uint64", "ns"),
    0x800B: ("time_offset", 8, "int64", "ns"),
    0x800C: ("time_sync_type", 1, "uint8", None),
}
DIAG_KEY = 0x800E
DIAG_STATUS = {
    0: "normal", 1: "warning", 2: "error", 3: "safety_err",
}


class Mid360ControlPacketError(ValueError):
    """Malformed or unsupported MID-360 control/status packet."""


@dataclass(frozen=True)
class Mid360ControlMessage:
    cmd_id: int
    cmd_type: int
    sender_type: int
    sequence_number: int
    return_code: int | None
    key_values: tuple[dict, ...]


def _crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _decode_key_value(key: int, value: bytes) -> dict:
    item = {"key": key, "length": len(value)}
    if key in TIME_KEYS:
        name, expected_length, kind, unit = TIME_KEYS[key]
        if len(value) != expected_length:
            raise Mid360ControlPacketError(
                f"{name} key 0x{key:04x} length {len(value)} != {expected_length}"
            )
        fmt = {"uint64": "<Q", "int64": "<q", "uint8": "<B"}[kind]
        item.update(name=name, value=struct.unpack(fmt, value)[0], data_type=kind)
        if unit:
            item["unit"] = unit
        if key == 0x800C:
            item["meaning"] = {
                0: "no_sync", 1: "ptp_ieee1588v2_0", 2: "gps"
            }.get(item["value"], "unknown")
    elif key == DIAG_KEY:
        if len(value) != 2:
            raise Mid360ControlPacketError("lidar diagnostic status must be 2 bytes")
        raw = struct.unpack("<H", value)[0]
        modules = ("system", "scan", "ranging", "communication")
        item.update(
            name="lidar_diag_status", value=raw, raw_hex=f"0x{raw:04x}",
            modules={
                name: {"level": (raw >> (4 * i)) & 0xF,
                       "meaning": DIAG_STATUS.get((raw >> (4 * i)) & 0xF, "unknown")}
                for i, name in enumerate(modules)
            },
        )
    else:
        item["raw_hex"] = value.hex()
    return item


def _parse_key_values(data: bytes, offset: int, count: int) -> tuple[dict, ...]:
    items = []
    cursor = offset
    for _ in range(count):
        if cursor + 4 > len(data):
            raise Mid360ControlPacketError("truncated key/value header")
        key, length = struct.unpack_from("<HH", data, cursor)
        cursor += 4
        if cursor + length > len(data):
            raise Mid360ControlPacketError(f"truncated value for key 0x{key:04x}")
        items.append(_decode_key_value(key, data[cursor:cursor + length]))
        cursor += length
    if cursor != len(data):
        raise Mid360ControlPacketError("trailing bytes after key/value list")
    return tuple(items)


def parse_mid360_control_payload(payload: bytes) -> Mid360ControlMessage:
    """Validate the 24-byte command envelope and decode time/status key-values.

    Supports 0x0101 ACK query responses and 0x0102 device status pushes.
    The command UDP payload is little-endian; CRC16 covers bytes [0:18],
    CRC32 covers the data segment [24:length].
    """
    if len(payload) < HEADER_SIZE:
        raise Mid360ControlPacketError("control packet shorter than 24-byte header")
    if payload[0] != SOF:
        raise Mid360ControlPacketError("invalid control SOF")
    if payload[1] != 0:
        raise Mid360ControlPacketError(f"unsupported control protocol version {payload[1]}")
    length = struct.unpack_from("<H", payload, 2)[0]
    if length != len(payload) or length > 1400:
        raise Mid360ControlPacketError("control length does not match UDP payload or exceeds 1400")
    expected_crc16 = struct.unpack_from("<H", payload, 18)[0]
    if _crc16_ccitt_false(payload[:18]) != expected_crc16:
        raise Mid360ControlPacketError("control header CRC16 mismatch")
    expected_crc32 = struct.unpack_from("<I", payload, 20)[0]
    if (zlib.crc32(payload[24:]) & 0xFFFFFFFF) != expected_crc32:
        raise Mid360ControlPacketError("control data CRC32 mismatch")

    sequence_number = struct.unpack_from("<I", payload, 4)[0]
    cmd_id = struct.unpack_from("<H", payload, 8)[0]
    cmd_type, sender_type = payload[10], payload[11]
    data = payload[24:]
    return_code = None
    key_values = ()
    if cmd_id == CMD_LIDAR_INFO_QUERY and cmd_type == 1:
        if len(data) < 3:
            raise Mid360ControlPacketError("0x0101 ACK lacks return code/key count")
        return_code = data[0]
        count = struct.unpack_from("<H", data, 1)[0]
        key_values = _parse_key_values(data, 3, count)
    elif cmd_id == CMD_LIDAR_INFO_PUSH:
        if len(data) < 4:
            raise Mid360ControlPacketError("0x0102 push lacks key count/reserved")
        count = struct.unpack_from("<H", data, 0)[0]
        key_values = _parse_key_values(data, 4, count)
    else:
        raise Mid360ControlPacketError(
            f"unsupported diagnostic message cmd_id=0x{cmd_id:04x}, cmd_type={cmd_type}"
        )
    return Mid360ControlMessage(
        cmd_id, cmd_type, sender_type, sequence_number, return_code, key_values
    )
