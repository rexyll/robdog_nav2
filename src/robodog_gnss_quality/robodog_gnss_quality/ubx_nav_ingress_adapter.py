"""Strict raw UBX NAV-frame decoders for the ordered USB ingress path.

The C++ stream parser validates framing/checksums; this boundary revalidates the
UBX envelope and payload layout before constructing typed navigation epochs.
GPS week is supplied only by a matching validated receiver-time mapping.
"""

from dataclasses import dataclass
import struct
from typing import Optional, Union

from .ubx_ingress_adapter import PROTOCOL_UBX, _validate_ubx_frame
from .ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
    UBX_GPS_WEEK_MS,
)


NAV_CLASS = 0x01
NAV_PVT_ID = 0x07
NAV_HPPOSLLH_ID = 0x14
NAV_STATUS_ID = 0x03
NAV_COV_ID = 0x36
NAV_PVT_LENGTH = 92
NAV_HPPOSLLH_LENGTH = 36
NAV_STATUS_LENGTH = 16
NAV_COV_LENGTH = 64

_KIND_BY_ID = {
    NAV_PVT_ID: "pvt",
    NAV_HPPOSLLH_ID: "position",
    NAV_COV_ID: "covariance",
    NAV_STATUS_ID: "status",
}
_LENGTH_BY_KIND = {
    "pvt": NAV_PVT_LENGTH,
    "position": NAV_HPPOSLLH_LENGTH,
    "covariance": NAV_COV_LENGTH,
    "status": NAV_STATUS_LENGTH,
}


@dataclass(frozen=True)
class UbxNavRawIngressRecord:
    kind: str
    i_tow_ms: int
    payload: bytes
    receive_monotonic_ns: int
    process_session_id: str
    connection_generation: int
    ingress_sequence: int


def _uint(name, value, maximum=(1 << 64) - 1):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise EpochPairingError(f"{name} must be an unsigned integer in range")
    return value


def _context(*, receive_monotonic_ns, process_session_id,
             connection_generation, ingress_sequence):
    _uint("receive_monotonic_ns", receive_monotonic_ns)
    _uint("connection_generation", connection_generation)
    _uint("ingress_sequence", ingress_sequence)
    if not isinstance(process_session_id, str) or not process_session_id.strip():
        raise EpochPairingError("process_session_id must be a non-empty string")


def decode_ubx_nav_ingress_frame(
    frame_bytes: bytes,
    *,
    protocol: Union[str, int],
    receive_monotonic_ns: int,
    process_session_id: str,
    connection_generation: int,
    ingress_sequence: int,
) -> Optional[UbxNavRawIngressRecord]:
    """Return a recognized NAV raw record; unrelated/non-UBX frames return None."""
    _context(
        receive_monotonic_ns=receive_monotonic_ns,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
        ingress_sequence=ingress_sequence,
    )
    if isinstance(protocol, bool):
        raise EpochPairingError("protocol must identify UBX, RTCM3, or NMEA")
    if protocol in ("rtcm3", "nmea", 1, 2):
        return None
    if protocol not in ("ubx", PROTOCOL_UBX):
        raise EpochPairingError("protocol must identify UBX, RTCM3, or NMEA")

    frame = _validate_ubx_frame(frame_bytes)
    message_class, message_id = frame[2], frame[3]
    kind = _KIND_BY_ID.get(message_id) if message_class == NAV_CLASS else None
    if kind is None:
        return None
    payload = frame[6:-2]
    expected = _LENGTH_BY_KIND[kind]
    if len(payload) != expected:
        raise EpochPairingError(
            f"NAV-{kind.upper()} payload must be exactly {expected} bytes"
        )
    i_tow = struct.unpack_from("<I", payload, 0 if kind != "position" else 4)[0]
    if not 0 <= i_tow < UBX_GPS_WEEK_MS:
        raise EpochPairingError(f"NAV-{kind.upper()} iTOW is outside GPS week range")
    return UbxNavRawIngressRecord(
        kind=kind,
        i_tow_ms=i_tow,
        payload=payload,
        receive_monotonic_ns=receive_monotonic_ns,
        process_session_id=process_session_id,
        connection_generation=connection_generation,
        ingress_sequence=ingress_sequence,
    )


def decode_ubx_nav_raw_record(
    record: UbxNavRawIngressRecord, *, gps_week: int
):
    """Decode a raw record after a matching valid receiver-time mapping supplies GPS week."""
    if not isinstance(record, UbxNavRawIngressRecord):
        raise EpochPairingError("record must be a UbxNavRawIngressRecord")
    gps_week = _uint("gps_week", gps_week, 32767)
    payload = record.payload
    common = dict(
        process_session_id=record.process_session_id,
        connection_generation=record.connection_generation,
        gps_week=gps_week,
        i_tow_ms=record.i_tow_ms,
        receive_monotonic_ns=record.receive_monotonic_ns,
    )

    if record.kind == "pvt":
        if len(payload) != NAV_PVT_LENGTH:
            raise EpochPairingError("NAV-PVT payload length changed after envelope validation")
        fix_type = payload[20]
        flags = payload[21]
        flags3 = payload[78]
        if fix_type > 5:
            raise EpochPairingError("NAV-PVT gpsFix type is outside defined range")
        if flags3 & 0x01:
            raise EpochPairingError("NAV-PVT marks LLH invalid")
        carrier_code = (flags >> 6) & 0x03
        carrier = _carrier_solution(carrier_code)
        return UbxNavPvtEpoch(
            **common,
            gnss_fix_ok=bool(flags & 0x01),
            carrier_solution=carrier,
        )

    if record.kind == "position":
        if len(payload) != NAV_HPPOSLLH_LENGTH:
            raise EpochPairingError("NAV-HPPOSLLH payload length changed after envelope validation")
        version = payload[0]
        if version != 0:
            raise EpochPairingError(f"unsupported NAV-HPPOSLLH version {version}")
        flags = payload[3]
        lon, lat, height, _hmsl = struct.unpack_from("<iiii", payload, 8)
        lon_hp, lat_hp, height_hp, _hmsl_hp = struct.unpack_from("<bbbb", payload, 24)
        _h_acc, _v_acc = struct.unpack_from("<II", payload, 28)
        return UbxNavHpPosLlhEpoch(
            **common,
            lon=lon,
            lat=lat,
            height_mm=height,
            lon_hp=lon_hp,
            lat_hp=lat_hp,
            height_hp=height_hp,
            invalid_lon=bool(flags & 0x01),
            invalid_lat=bool(flags & 0x02),
            invalid_height=bool(flags & 0x04),
            invalid_lon_hp=bool(flags & 0x10),
            invalid_lat_hp=bool(flags & 0x20),
            invalid_height_hp=bool(flags & 0x40),
        )

    if record.kind == "covariance":
        if len(payload) != NAV_COV_LENGTH:
            raise EpochPairingError("NAV-COV payload length changed after envelope validation")
        version = payload[4]
        if version != 0:
            raise EpochPairingError(f"unsupported NAV-COV version {version}")
        pos_cor_valid = bool(payload[5])
        values = struct.unpack_from("<ffffff", payload, 16)
        return UbxNavCovEpoch(
            **common,
            pos_cor_valid=pos_cor_valid,
            pos_cov_nn=values[0],
            pos_cov_ne=values[1],
            pos_cov_nd=values[2],
            pos_cov_ee=values[3],
            pos_cov_ed=values[4],
            pos_cov_dd=values[5],
        )

    if record.kind == "status":
        if len(payload) != NAV_STATUS_LENGTH:
            raise EpochPairingError("NAV-STATUS payload length changed after envelope validation")
        gps_fix_type = payload[4]
        flags = payload[5]
        fix_stat = payload[6]
        flags2 = payload[7]
        if gps_fix_type > 5:
            raise EpochPairingError("NAV-STATUS gpsFix type is outside defined range")
        carrier_code = (flags2 >> 6) & 0x03
        return UbxNavStatusEpoch(
            **common,
            gnss_fix_ok=bool(flags & 0x01),
            carrier_solution_valid=bool(fix_stat & 0x02),
            carrier_solution=_carrier_solution(carrier_code),
        )

    raise EpochPairingError(f"unsupported raw NAV record kind: {record.kind}")


def _carrier_solution(code: int):
    from .quality_gate import CarrierSolution
    return {
        0: CarrierSolution.NO_SOLUTION,
        1: CarrierSolution.FLOAT,
        2: CarrierSolution.FIXED,
        3: CarrierSolution.UNKNOWN,
    }[code]
