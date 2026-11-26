"""Strict UBX frame-envelope adapter for raw receiver TIMEGPS/TIMELS ingress.

The C++ stream decoder already validates UBX framing/checksum. This Python
boundary deliberately revalidates the envelope before passing payload bytes
to the canonical typed TIMEGPS/TIMELS decoders.
"""

from typing import Optional, Union

from .ubx_nav_pvt import EpochPairingError
from .ubx_time_messages import (
    NAV_TIMEGPS_CLASS,
    NAV_TIMEGPS_ID,
    NAV_TIMELS_CLASS,
    NAV_TIMELS_ID,
    UbxNavTimeGpsEpoch,
    UbxNavTimeLsEpoch,
    decode_nav_timegps_payload,
    decode_nav_timels_payload,
)


UbxTimeIngressRecord = Union[UbxNavTimeGpsEpoch, UbxNavTimeLsEpoch]
PROTOCOL_UBX = 0
PROTOCOL_RTCM3 = 1
PROTOCOL_NMEA = 2


def _validate_ingress_context(
    *,
    receive_monotonic_ns: int,
    process_session_id: str,
    connection_generation: int,
    ingress_sequence: int,
) -> None:
    if (
        isinstance(receive_monotonic_ns, bool)
        or not isinstance(receive_monotonic_ns, int)
        or receive_monotonic_ns < 0
    ):
        raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
    if not isinstance(process_session_id, str) or not process_session_id.strip():
        raise EpochPairingError("process_session_id must be a non-empty string")
    if (
        isinstance(connection_generation, bool)
        or not isinstance(connection_generation, int)
        or connection_generation < 0
    ):
        raise EpochPairingError("connection_generation must be a non-negative integer")
    if (
        isinstance(ingress_sequence, bool)
        or not isinstance(ingress_sequence, int)
        or ingress_sequence < 0
    ):
        raise EpochPairingError("ingress_sequence must be a non-negative integer")


def _validate_ubx_frame(frame_bytes: bytes) -> bytes:
    if not isinstance(frame_bytes, (bytes, bytearray, memoryview)):
        raise EpochPairingError("UBX frame must be bytes-like")
    frame = bytes(frame_bytes)
    if len(frame) < 8:
        raise EpochPairingError("UBX frame is shorter than the 8-byte minimum")
    if frame[0:2] != b"\xB5\x62":
        raise EpochPairingError("UBX frame sync bytes are invalid")
    payload_length = frame[4] | (frame[5] << 8)
    expected_length = payload_length + 8
    if len(frame) != expected_length:
        raise EpochPairingError(
            f"UBX frame length mismatch: declared {expected_length}, received {len(frame)}"
        )
    ck_a = 0
    ck_b = 0
    for value in frame[2:-2]:
        ck_a = (ck_a + value) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    if frame[-2] != ck_a or frame[-1] != ck_b:
        raise EpochPairingError("UBX frame checksum mismatch")
    return frame


def decode_ubx_time_ingress_frame(
    frame_bytes: bytes,
    *,
    protocol: Union[str, int],
    receive_monotonic_ns: int,
    process_session_id: str,
    connection_generation: int,
    ingress_sequence: int,
) -> Optional[UbxTimeIngressRecord]:
    """Decode one ordered ingress event; return None for non-time/non-UBX data.

    protocol accepts C++ enum values 0=UBX, 1=RTCM3, 2=NMEA or the strings
    "ubx", "rtcm3", "nmea". Non-UBX protocols are ignored.
    """
    if isinstance(protocol, bool):
        raise EpochPairingError("protocol must identify UBX, RTCM3, or NMEA")
    protocol_names = {
        "ubx": PROTOCOL_UBX,
        "rtcm3": PROTOCOL_RTCM3,
        "nmea": PROTOCOL_NMEA,
    }
    if isinstance(protocol, str):
        protocol_id = protocol_names.get(protocol)
    elif isinstance(protocol, int):
        protocol_id = protocol if protocol in (
            PROTOCOL_UBX, PROTOCOL_RTCM3, PROTOCOL_NMEA
        ) else None
    else:
        protocol_id = None
    if protocol_id is None:
        raise EpochPairingError("protocol must identify UBX, RTCM3, or NMEA")

    _validate_ingress_context(
        receive_monotonic_ns=receive_monotonic_ns,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
        ingress_sequence=ingress_sequence,
    )
    if protocol_id != PROTOCOL_UBX:
        return None

    frame = _validate_ubx_frame(frame_bytes)
    message_class = frame[2]
    message_id = frame[3]
    payload = frame[6:-2]
    context = {
        "receive_monotonic_ns": receive_monotonic_ns,
        "process_session_id": process_session_id,
        "connection_generation": connection_generation,
        "ingress_sequence": ingress_sequence,
    }
    if (message_class, message_id) == (NAV_TIMEGPS_CLASS, NAV_TIMEGPS_ID):
        return decode_nav_timegps_payload(payload, **context)
    if (message_class, message_id) == (NAV_TIMELS_CLASS, NAV_TIMELS_ID):
        return decode_nav_timels_payload(payload, **context)
    return None
