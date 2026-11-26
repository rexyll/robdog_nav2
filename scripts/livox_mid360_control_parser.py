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
HMS_KEY = 0x8011
HMS_LEVELS = {1: "info", 2: "warning", 3: "error", 4: "fatal"}
HMS_CATALOG = {
    0x0102: ("Environment temperature is slightly high", "Check ambient temperature and heat dissipation."),
    0x0103: ("Environment temperature is relatively high", "Check ambient temperature and heat dissipation."),
    0x0104: ("Window is dirty and point-cloud reliability may be affected", "Inspect and clean the optical window."),
    0x0105: ("Device upgrade process error", "Inspect upgrade state and retry only under the vendor procedure."),
    0x0111: ("Abnormal temperature of internal device components", "Check ambient temperature and heat dissipation."),
    0x0112: ("Abnormal temperature of internal device components", "Check ambient temperature and heat dissipation."),
    0x0113: ("IMU stopped working", "Follow vendor recovery guidance; restart only when safe."),
    0x0114: ("Environment temperature is high", "Check ambient temperature and heat dissipation."),
    0x0115: ("Environment temperature exceeded limit; device stopped", "Check ambient temperature and power/thermal conditions."),
    0x0116: ("Abnormal external voltage", "Check the specified external supply voltage."),
    0x0117: ("Abnormal LiDAR parameters", "Inspect configuration and consult vendor recovery guidance."),
    0x0118: ("Internal device component damage", "Stop relying on the sensor and contact vendor service."),
    0x0201: ("Scan module is heating", "Wait for scan-module heating to complete."),
    0x0210: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0211: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0212: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0213: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0214: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0215: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0216: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0217: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0218: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x0219: ("Scan module abnormality", "Allow recovery; if persistent, follow vendor restart/service guidance."),
    0x021C: ("Scan code disk dirty / abnormal rotation speed (MID-360L)", "Inspect and clean/service according to the MID-360L vendor procedure."),
    0x0304: ("TIA DC abnormal; ranging point cloud may be abnormal", "Follow vendor troubleshooting and recovery guidance."),
    0x0401: ("Communication link recovered after link-down (MID-360S)", "Inspect Ethernet link and switch/host interface logs."),
    0x0402: ("PTP synchronization stopped or time gap too large", "Check PTP source, profile, network path and device sync status."),
    0x0403: ("Unsupported IEEE 1588v2.1 detected", "Use the supported IEEE 1588v2.0 mode; verify network configuration."),
    0x0404: ("PPS time synchronization abnormal", "Check PPS wiring, electrical levels and GPS timing source."),
    0x0405: ("Time synchronization exception", "Correlate device HMS, PTP/GPS logs and network capture."),
    0x0406: ("Time synchronization accuracy is low", "Check time source quality, PHC timestamping and network path."),
    0x0407: ("GPS signal lost during PPS synchronization", "Check GNSS antenna, sky view and receiver status."),
    0x0408: ("PPS signal lost during PPS synchronization", "Check PPS wiring, pulse integrity and receiver output."),
    0x0409: ("GPS signal abnormal", "Check GNSS receiver status and time-source validity."),
    0x040A: ("PTP and gPTP signals detected simultaneously", "Inspect topology and use only the intended synchronization mode."),
}

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
    elif key == HMS_KEY:
        if len(value) != 32:
            raise Mid360ControlPacketError("hms_code key 0x8011 must be 32 bytes")
        codes = []
        for slot in range(8):
            raw = struct.unpack_from("<I", value, slot * 4)[0]
            if raw == 0:
                continue
            abnormal_id = (raw >> 16) & 0xFFFF
            reserved = (raw >> 8) & 0xFF
            level = raw & 0xFF
            description, recommendation = HMS_CATALOG.get(
                abnormal_id, (None, None)
            )
            entry = {
                "slot": slot, "raw": raw, "raw_hex": f"0x{raw:08x}",
                "abnormal_id": abnormal_id,
                "abnormal_id_hex": f"0x{abnormal_id:04x}",
                "reserved": reserved, "level": level,
                "level_name": HMS_LEVELS.get(level, "unknown"),
                "description": description,
                "recommendation": recommendation,
            }
            if abnormal_id == 0x021C:
                entry["model_scope"] = "MID-360L only per vendor table"
            elif abnormal_id == 0x0401:
                entry["model_scope"] = "MID-360S only per vendor table"
            codes.append(entry)
        item.update(name="hms_code", value=codes, slot_count=8,
                    active_count=len(codes), raw_hex=value.hex())
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
