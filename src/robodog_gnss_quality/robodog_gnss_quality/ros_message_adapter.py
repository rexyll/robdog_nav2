"""Strict field adapter for the pinned ublox_ubx_msgs ROS message schema.

The module has no rclpy or ROS dependency. Callbacks must pass one shared
host-monotonic receive timestamp; ROS Header.stamp is intentionally ignored.
"""

import math

from .quality_gate import CarrierSolution
from .ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
    UBX_GPS_WEEK_MS,
)


def _integer(name: str, value: object, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise EpochPairingError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def _boolean(name: str, value: object) -> bool:
    if not isinstance(value, bool):
        raise EpochPairingError(f"{name} must be a boolean")
    return value


def _receive_time(value: object) -> int:
    return _integer("receive_monotonic_ns", value, 0, (1 << 63) - 1)


def _context(process_session_id: object, connection_generation: object) -> tuple:
    if not isinstance(process_session_id, str) or not process_session_id.strip():
        raise EpochPairingError("process_session_id must be a non-empty string")
    generation = _integer("connection_generation", connection_generation, 0, (1 << 63) - 1)
    return process_session_id, generation


def _i_tow(value: object) -> int:
    return _integer("iTOW", value, 0, UBX_GPS_WEEK_MS - 1)


def _carrier_solution(status: object) -> CarrierSolution:
    code = _integer("carr_soln.status", status, 0, 255)
    return {
        0: CarrierSolution.NO_SOLUTION,
        1: CarrierSolution.FLOAT,
        2: CarrierSolution.FIXED,
    }.get(code, CarrierSolution.UNKNOWN)


def decode_nav_pvt_message(message: object, receive_monotonic_ns: int, *, process_session_id: str, connection_generation: int) -> UbxNavPvtEpoch:
    """Decode only the NAV-PVT fields required by the fail-closed quality core."""
    try:
        return UbxNavPvtEpoch(
            i_tow_ms=_i_tow(message.itow),
            gnss_fix_ok=_boolean("NAV-PVT gnss_fix_ok", message.gnss_fix_ok),
            carrier_solution=_carrier_solution(message.carr_soln.status),
            receive_monotonic_ns=_receive_time(receive_monotonic_ns),
            process_session_id=_context(process_session_id, connection_generation)[0],
            connection_generation=_context(process_session_id, connection_generation)[1],
        )
    except AttributeError as exc:
        raise EpochPairingError("NAV-PVT ROS message is missing a required field") from exc


def decode_nav_hp_pos_llh_message(
    message: object, receive_monotonic_ns: int, *,
    process_session_id: str, connection_generation: int
) -> UbxNavHpPosLlhEpoch:
    """Decode HPPOSLLH raw integer and validity fields without using Header.stamp."""
    try:
        return UbxNavHpPosLlhEpoch(
            i_tow_ms=_i_tow(message.itow),
            lon=_integer("HPPOSLLH lon", message.lon, -(1 << 31), (1 << 31) - 1),
            lat=_integer("HPPOSLLH lat", message.lat, -(1 << 31), (1 << 31) - 1),
            height_mm=_integer("HPPOSLLH height", message.height, -(1 << 31), (1 << 31) - 1),
            lon_hp=_integer("HPPOSLLH lon_hp", message.lon_hp, -128, 127),
            lat_hp=_integer("HPPOSLLH lat_hp", message.lat_hp, -128, 127),
            height_hp=_integer("HPPOSLLH height_hp", message.height_hp, -128, 127),
            invalid_lon=_boolean("HPPOSLLH invalid_lon", message.invalid_lon),
            invalid_lat=_boolean("HPPOSLLH invalid_lat", message.invalid_lat),
            invalid_height=_boolean("HPPOSLLH invalid_height", message.invalid_height),
            invalid_lon_hp=_boolean("HPPOSLLH invalid_lon_hp", message.invalid_lon_hp),
            invalid_lat_hp=_boolean("HPPOSLLH invalid_lat_hp", message.invalid_lat_hp),
            invalid_height_hp=_boolean("HPPOSLLH invalid_height_hp", message.invalid_height_hp),
            receive_monotonic_ns=_receive_time(receive_monotonic_ns),
            process_session_id=_context(process_session_id, connection_generation)[0],
            connection_generation=_context(process_session_id, connection_generation)[1],
        )
    except AttributeError as exc:
        raise EpochPairingError("NAV-HPPOSLLH ROS message is missing a required field") from exc


def _finite_float(name: str, value: object) -> float:
    if isinstance(value, bool):
        raise EpochPairingError(f"{name} must be a finite numeric value, not bool")
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise EpochPairingError(f"{name} must be numeric") from exc
    if not math.isfinite(converted):
        raise EpochPairingError(f"{name} must be finite")
    return converted


def decode_nav_cov_message(message: object, receive_monotonic_ns: int, *, process_session_id: str, connection_generation: int) -> UbxNavCovEpoch:
    """Decode the six NED position covariance terms and validity bit."""
    try:
        return UbxNavCovEpoch(
            i_tow_ms=_i_tow(message.itow),
            pos_cor_valid=_boolean("NAV-COV pos_cor_valid", message.pos_cor_valid),
            pos_cov_nn=_finite_float("NAV-COV pos_cov_nn", message.pos_cov_nn),
            pos_cov_ne=_finite_float("NAV-COV pos_cov_ne", message.pos_cov_ne),
            pos_cov_nd=_finite_float("NAV-COV pos_cov_nd", message.pos_cov_nd),
            pos_cov_ee=_finite_float("NAV-COV pos_cov_ee", message.pos_cov_ee),
            pos_cov_ed=_finite_float("NAV-COV pos_cov_ed", message.pos_cov_ed),
            pos_cov_dd=_finite_float("NAV-COV pos_cov_dd", message.pos_cov_dd),
            receive_monotonic_ns=_receive_time(receive_monotonic_ns),
            process_session_id=_context(process_session_id, connection_generation)[0],
            connection_generation=_context(process_session_id, connection_generation)[1],
        )
    except AttributeError as exc:
        raise EpochPairingError("NAV-COV ROS message is missing a required field") from exc


def decode_nav_status_message(
    message: object, receive_monotonic_ns: int, *,
    process_session_id: str, connection_generation: int
) -> UbxNavStatusEpoch:
    """Decode NAV-STATUS cross-check fields required by the four-message joiner."""
    try:
        return UbxNavStatusEpoch(
            i_tow_ms=_i_tow(message.itow),
            gnss_fix_ok=_boolean("NAV-STATUS gps_fix_ok", message.gps_fix_ok),
            carrier_solution_valid=_boolean(
                "NAV-STATUS carr_soln_valid", message.carr_soln_valid
            ),
            carrier_solution=_carrier_solution(message.carr_soln.status),
            receive_monotonic_ns=_receive_time(receive_monotonic_ns),
            process_session_id=_context(process_session_id, connection_generation)[0],
            connection_generation=_context(process_session_id, connection_generation)[1],
        )
    except AttributeError as exc:
        raise EpochPairingError("NAV-STATUS ROS message is missing a required field") from exc
