#!/usr/bin/env python3
"""Streaming, dependency-free PCAPNG reader preserving per-interface clock metadata.

Scope: Section Header, Interface Description, Enhanced Packet and Simple Packet
blocks. Network-layer decoding (Ethernet/IP/UDP) is deliberately separate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import BinaryIO, Iterator
import struct


SHB = 0x0A0D0D0A
IDB = 0x00000001
SPB = 0x00000003
EPB = 0x00000006
BYTE_ORDER_MAGIC = b"\x1a\x2b\x3c\x4d"


class PcapngError(ValueError):
    """Malformed or unsupported PCAPNG structure."""


@dataclass(frozen=True)
class PcapngInterface:
    section_index: int
    interface_id: int
    link_type: int
    snaplen: int
    tsresol_raw: int = 6
    tsoffset_seconds: int = 0
    name: str | None = None
    description: str | None = None

    @property
    def timestamp_base(self) -> int:
        return 2 if self.tsresol_raw & 0x80 else 10

    @property
    def timestamp_exponent(self) -> int:
        return self.tsresol_raw & 0x7F if self.tsresol_raw & 0x80 else self.tsresol_raw

    def timestamp_ns(self, raw_ticks: int) -> int:
        """Convert interface ticks to integer ns, truncating sub-ns precision."""
        base = self.timestamp_base
        exponent = self.timestamp_exponent
        if base == 10:
            if exponent <= 9:
                scaled = raw_ticks * (10 ** (9 - exponent))
            else:
                scaled = raw_ticks // (10 ** (exponent - 9))
        else:
            numerator = raw_ticks * (10 ** 9)
            denominator = 1 << exponent
            scaled = numerator // denominator
        return (self.tsoffset_seconds * 1_000_000_000) + scaled


@dataclass(frozen=True)
class PcapngPacket:
    section_index: int
    interface_id: int
    link_type: int
    capture_timestamp_ticks: int | None
    capture_timestamp_ns: int | None
    captured_length: int
    original_length: int
    data: bytes
    timestamp_available: bool
    truncated: bool


def _read_exact(stream: BinaryIO, size: int) -> bytes:
    data = stream.read(size)
    if len(data) != size:
        raise PcapngError(f"unexpected EOF: wanted {size} bytes, got {len(data)}")
    return data


def _parse_options(data: bytes, endian: str) -> dict[int, bytes]:
    options: dict[int, bytes] = {}
    pos = 0
    while pos < len(data):
        if len(data) - pos < 4:
            raise PcapngError("truncated option header")
        code, length = struct.unpack_from(endian + "HH", data, pos)
        pos += 4
        padded = (length + 3) & ~3
        if pos + padded > len(data):
            raise PcapngError("option value exceeds block bounds")
        value = data[pos:pos + length]
        pos += padded
        if code == 0:
            if length != 0:
                raise PcapngError("end-of-options must have zero length")
            if any(data[pos:]):
                raise PcapngError("nonzero bytes after end-of-options")
            break
        if code in options:
            raise PcapngError(f"duplicate singleton option code {code}")
        options[code] = value
    return options


def iter_pcapng_packets(stream: BinaryIO) -> Iterator[PcapngPacket]:
    """Yield packets from a binary stream; each SHB starts a fresh section."""
    endian: str | None = None
    section_index = -1
    interfaces: list[PcapngInterface] = []

    while True:
        block_start = stream.read(8)
        if not block_start:
            return
        if len(block_start) != 8:
            raise PcapngError("truncated block type/length")

        block_type_raw = block_start[:4]
        if block_type_raw == b"\x0a\x0d\x0d\x0a":
            bom = _read_exact(stream, 4)
            if bom == b"\x4d\x3c\x2b\x1a":
                endian = "<"
            elif bom == BYTE_ORDER_MAGIC:
                endian = ">"
            else:
                raise PcapngError(f"invalid byte-order magic: {bom.hex()}")
            block_type = SHB
            total_length = struct.unpack(endian + "I", block_start[4:8])[0]
            section_index += 1
            interfaces = []
            body_prefix = bom
        else:
            if endian is None:
                raise PcapngError("file must begin with a Section Header Block")
            block_type = struct.unpack(endian + "I", block_type_raw)[0]
            total_length = struct.unpack(endian + "I", block_start[4:8])[0]
            body_prefix = b""

        min_length = 28 if block_type == SHB else 12
        if total_length < min_length or total_length % 4:
            raise PcapngError(f"invalid block total length: {total_length}")
        remaining = total_length - 12 - len(body_prefix)
        if remaining < 0:
            raise PcapngError("invalid block length")
        body_and_trailer = _read_exact(stream, remaining + 4)
        body = body_prefix + body_and_trailer[:-4]
        trailer_length = struct.unpack(endian + "I", body_and_trailer[-4:])[0]
        if trailer_length != total_length:
            raise PcapngError(
                f"block length trailer {trailer_length} != header {total_length}"
            )

        if block_type == SHB:
            if len(body) < 16:
                raise PcapngError("short Section Header Block")
            major, minor = struct.unpack_from(endian + "HH", body, 4)
            if major != 1:
                raise PcapngError(f"unsupported PCAPNG major version: {major}")
            # minor is retained only as an accepted compatibility field.
            _ = minor
        elif block_type == IDB:
            if len(body) < 8:
                raise PcapngError("short Interface Description Block")
            link_type, _reserved, snaplen = struct.unpack_from(endian + "HHI", body, 0)
            opts = _parse_options(body[8:], endian)
            tsresol = opts.get(9, b"\x06")
            tsoffset = opts.get(14, b"\x00" * 8)
            if len(tsresol) != 1 or len(tsoffset) != 8:
                raise PcapngError("invalid if_tsresol/if_tsoffset option length")
            try:
                name = opts.get(2, b"").decode("utf-8") or None
                description = opts.get(3, b"").decode("utf-8") or None
            except UnicodeDecodeError as exc:
                raise PcapngError("interface name/description is not UTF-8") from exc
            interfaces.append(PcapngInterface(
                section_index, len(interfaces), link_type, snaplen,
                tsresol[0], struct.unpack(endian + "q", tsoffset)[0],
                name, description,
            ))
        elif block_type == EPB:
            if len(body) < 20:
                raise PcapngError("short Enhanced Packet Block")
            iid, high, low, caplen, origlen = struct.unpack_from(
                endian + "IIIII", body, 0
            )
            if iid >= len(interfaces):
                raise PcapngError(f"EPB references unknown interface {iid}")
            padded_caplen = (caplen + 3) & ~3
            if 20 + padded_caplen > len(body):
                raise PcapngError("EPB captured packet exceeds block bounds")
            packet_data = body[20:20 + caplen]
            interface = interfaces[iid]
            ticks = (high << 32) | low
            yield PcapngPacket(
                section_index, iid, interface.link_type, ticks,
                interface.timestamp_ns(ticks), caplen, origlen, packet_data,
                True, caplen < origlen or (interface.snaplen > 0 and caplen < min(origlen, interface.snaplen)),
            )
        elif block_type == SPB:
            if len(body) < 4:
                raise PcapngError("short Simple Packet Block")
            if not interfaces:
                raise PcapngError("SPB requires interface 0")
            (origlen,) = struct.unpack_from(endian + "I", body, 0)
            interface = interfaces[0]
            caplen = min(origlen, interface.snaplen) if interface.snaplen else origlen
            padded_caplen = (caplen + 3) & ~3
            if 4 + padded_caplen > len(body):
                raise PcapngError("SPB captured packet exceeds block bounds")
            packet_data = body[4:4 + caplen]
            yield PcapngPacket(
                section_index, 0, interface.link_type, None, None,
                caplen, origlen, packet_data, False, caplen < origlen,
            )
        # Unknown block types are skipped by their validated total length.
