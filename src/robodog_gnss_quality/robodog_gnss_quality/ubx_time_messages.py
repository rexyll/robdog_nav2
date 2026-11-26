"""Strict UBX-NAV-TIMEGPS and UBX-NAV-TIMELS payload decoders.

The decoder preserves raw validity bits, source identifiers and reserved bytes.
It performs structural/range validation only; it does not promote receiver time
or leap-second data to trusted mapping inputs.
"""

from dataclasses import dataclass
import struct

from .ubx_nav_pvt import EpochPairingError, UBX_GPS_WEEK_MS


NAV_TIMEGPS_CLASS = 0x01
NAV_TIMEGPS_ID = 0x20
NAV_TIMEGPS_PAYLOAD_LENGTH = 16
NAV_TIMELS_CLASS = 0x01
NAV_TIMELS_ID = 0x26
NAV_TIMELS_PAYLOAD_LENGTH = 24


def _u8(data: bytes, offset: int) -> int:
    return data[offset]


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _i8(data: bytes, offset: int) -> int:
    return struct.unpack_from("<b", data, offset)[0]


def _i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def _i32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<i", data, offset)[0]


def _validate_context(receive_monotonic_ns: int, process_session_id: str,
                      connection_generation: int) -> None:
    if (isinstance(receive_monotonic_ns, bool)
            or not isinstance(receive_monotonic_ns, int)
            or receive_monotonic_ns < 0):
        raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
    if not isinstance(process_session_id, str) or not process_session_id.strip():
        raise EpochPairingError("process_session_id must be a non-empty string")
    if (isinstance(connection_generation, bool)
            or not isinstance(connection_generation, int)
            or connection_generation < 0):
        raise EpochPairingError("connection_generation must be a non-negative integer")


def _validate_payload(payload: bytes, expected_length: int, message_name: str) -> bytes:
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise EpochPairingError(f"{message_name} payload must be bytes-like")
    data = bytes(payload)
    if len(data) != expected_length:
        raise EpochPairingError(
            f"{message_name} payload must be exactly {expected_length} bytes"
        )
    return data


def _validate_itow(value: int, message_name: str) -> None:
    if not 0 <= value < UBX_GPS_WEEK_MS:
        raise EpochPairingError(f"{message_name} iTOW is outside GPS week range")


@dataclass(frozen=True)
class UbxNavTimeGpsEpoch:
    i_tow_ms: int
    f_tow_ns: int
    week: int
    leap_seconds: int
    validity_raw: int
    tow_valid: bool
    week_valid: bool
    leap_seconds_valid: bool
    reserved_validity_bits: int
    time_accuracy_ns: int
    receive_monotonic_ns: int
    process_session_id: str
    connection_generation: int


@dataclass(frozen=True)
class UbxNavTimeLsEpoch:
    i_tow_ms: int
    version: int
    reserved1: bytes
    source_current_leap_seconds: int
    current_leap_seconds: int
    source_leap_seconds_change: int
    leap_seconds_change: int
    time_to_leap_event_seconds: int
    leap_event_gps_week: int
    leap_event_gps_day: int
    reserved2: bytes
    validity_raw: int
    current_leap_seconds_valid: bool
    time_to_leap_event_valid: bool
    reserved_validity_bits: int
    receive_monotonic_ns: int
    process_session_id: str
    connection_generation: int


def decode_nav_timegps_payload(
    payload: bytes, *, receive_monotonic_ns: int,
    process_session_id: str, connection_generation: int,
) -> UbxNavTimeGpsEpoch:
    """Decode the 16-byte UBX-NAV-TIMEGPS payload (class 0x01, ID 0x20).

    This is the u-blox 9 / protocol 27.10 layout documented in UBX-18010854 R06; field layout is cross-checked against the F9 HPG 1.32 Interface Description UBX-22008968. week and leap seconds are
    signed fields; validity is independent for TOW, week and leap seconds.
    """
    data = _validate_payload(payload, NAV_TIMEGPS_PAYLOAD_LENGTH, "NAV-TIMEGPS")
    _validate_context(receive_monotonic_ns, process_session_id, connection_generation)
    i_tow = _u32(data, 0)
    _validate_itow(i_tow, "NAV-TIMEGPS")
    f_tow = _i32(data, 4)
    if not -500_000 <= f_tow <= 500_000:
        raise EpochPairingError("NAV-TIMEGPS fTOW is outside [-500000, 500000] ns")
    validity = _u8(data, 11)
    return UbxNavTimeGpsEpoch(
        i_tow_ms=i_tow,
        f_tow_ns=f_tow,
        week=_i16(data, 8),
        leap_seconds=_i8(data, 10),
        validity_raw=validity,
        tow_valid=bool(validity & 0x01),
        week_valid=bool(validity & 0x02),
        leap_seconds_valid=bool(validity & 0x04),
        reserved_validity_bits=validity & 0xF8,
        time_accuracy_ns=_u32(data, 12),
        receive_monotonic_ns=receive_monotonic_ns,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
    )


def decode_nav_timels_payload(
    payload: bytes, *, receive_monotonic_ns: int,
    process_session_id: str, connection_generation: int,
) -> UbxNavTimeLsEpoch:
    """Decode the 24-byte UBX-NAV-TIMELS payload (class 0x01, ID 0x26).

    Only version 0 is accepted. Current leap-second validity and future/last
    event validity remain separate; source IDs are preserved as raw values.
    """
    data = _validate_payload(payload, NAV_TIMELS_PAYLOAD_LENGTH, "NAV-TIMELS")
    _validate_context(receive_monotonic_ns, process_session_id, connection_generation)
    i_tow = _u32(data, 0)
    _validate_itow(i_tow, "NAV-TIMELS")
    version = _u8(data, 4)
    if version != 0:
        raise EpochPairingError(f"unsupported NAV-TIMELS version {version}")
    validity = _u8(data, 23)
    return UbxNavTimeLsEpoch(
        i_tow_ms=i_tow,
        version=version,
        reserved1=bytes(data[5:8]),
        source_current_leap_seconds=_u8(data, 8),
        current_leap_seconds=_i8(data, 9),
        source_leap_seconds_change=_u8(data, 10),
        leap_seconds_change=_i8(data, 11),
        time_to_leap_event_seconds=_i32(data, 12),
        leap_event_gps_week=_u16(data, 16),
        leap_event_gps_day=_u16(data, 18),
        reserved2=bytes(data[20:23]),
        validity_raw=validity,
        current_leap_seconds_valid=bool(validity & 0x01),
        time_to_leap_event_valid=bool(validity & 0x02),
        reserved_validity_bits=validity & 0xFC,
        receive_monotonic_ns=receive_monotonic_ns,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
    )
