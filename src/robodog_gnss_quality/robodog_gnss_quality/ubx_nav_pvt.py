"""u-blox UBX-NAV-PVT quality decoding and exact-epoch pairing helpers.

No serial decoder or ROS driver is included. The caller must obtain these
fields from the configured receiver driver and preserve the NAV-PVT iTOW
alongside the corresponding NavSatFix record.
"""

from dataclasses import dataclass
import math

from .quality_gate import CarrierSolution, CovarianceType, GnssObservation


UBX_GPS_WEEK_MS = 604_800_000


class EpochPairingError(ValueError):
    """Raised when position and receiver-quality messages cannot be paired safely."""


@dataclass(frozen=True)
class UbxNavPvtQuality:
    i_tow_ms: int
    flags: int
    receive_monotonic_ns: int


@dataclass(frozen=True)
class NavSatEpochRecord:
    i_tow_ms: int
    source_timestamp_ns: int
    source_clock_id: str
    receive_monotonic_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    navsat_status: int
    position_covariance: tuple
    covariance_type: CovarianceType


def decode_nav_pvt_flags(flags: int) -> tuple:
    """Decode UBX-NAV-PVT flags into (gnss_fix_ok, carrier_solution).

    UBX-NAV-PVT flags bit 0 is gnssFixOK. Bits 7:6 are carrSoln:
      0 = no carrier-phase solution
      1 = RTK float ambiguities
      2 = RTK fixed ambiguities
      3 = reserved/unknown (fail closed)
    """
    if isinstance(flags, bool) or not isinstance(flags, int) or not 0 <= flags <= 0xFF:
        raise EpochPairingError("UBX-NAV-PVT flags must be an unsigned 8-bit integer")

    gnss_fix_ok = bool(flags & 0x01)
    carrier_code = (flags >> 6) & 0x03
    carrier_solution = {
        0: CarrierSolution.NO_SOLUTION,
        1: CarrierSolution.FLOAT,
        2: CarrierSolution.FIXED,
        3: CarrierSolution.UNKNOWN,
    }[carrier_code]
    return gnss_fix_ok, carrier_solution


def pair_navsat_fix_with_nav_pvt(
    position: NavSatEpochRecord,
    quality: UbxNavPvtQuality,
    *,
    max_pair_receive_skew_ns: int,
) -> GnssObservation:
    """Create one gate observation only when both records identify the same iTOW.

    iTOW is a GPS week-relative millisecond counter. source_clock_id must also
    identify the receiver time epoch (at minimum GPS week plus receiver clock
    generation) so the downstream gate can detect week rollover/reboot.
    source_timestamp_ns must use one monotonic receiver time domain; it must
    not be synthesized from host wall time or from iTOW alone.
    """
    if max_pair_receive_skew_ns <= 0:
        raise EpochPairingError("max_pair_receive_skew_ns must be positive")

    for name, value in (("position.i_tow_ms", position.i_tow_ms), ("quality.i_tow_ms", quality.i_tow_ms)):
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < UBX_GPS_WEEK_MS:
            raise EpochPairingError(f"{name} is outside the UBX GPS week millisecond range")

    if position.i_tow_ms != quality.i_tow_ms:
        raise EpochPairingError(
            f"GNSS epoch mismatch: NavSatFix iTOW {position.i_tow_ms} != NAV-PVT iTOW {quality.i_tow_ms}"
        )

    if not isinstance(position.source_clock_id, str) or not position.source_clock_id.strip():
        raise EpochPairingError("source_clock_id must identify the receiver time epoch")
    if (
        isinstance(position.source_timestamp_ns, bool)
        or not isinstance(position.source_timestamp_ns, int)
        or position.source_timestamp_ns < 0
    ):
        raise EpochPairingError("source_timestamp_ns must be a non-negative integer")
    receive_times = (position.receive_monotonic_ns, quality.receive_monotonic_ns)
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in receive_times):
        raise EpochPairingError("receive monotonic timestamps must be non-negative integers")
    if abs(receive_times[0] - receive_times[1]) > max_pair_receive_skew_ns:
        raise EpochPairingError("NavSatFix and NAV-PVT receive times exceed the configured pairing skew")

    gnss_fix_ok, carrier_solution = decode_nav_pvt_flags(quality.flags)
    return GnssObservation(
        source_timestamp_ns=position.source_timestamp_ns,
        source_clock_id=position.source_clock_id,
        receive_monotonic_ns=max(receive_times),
        latitude_deg=position.latitude_deg,
        longitude_deg=position.longitude_deg,
        altitude_m=position.altitude_m,
        navsat_status=position.navsat_status,
        gnss_fix_ok=gnss_fix_ok,
        carrier_solution=carrier_solution,
        position_covariance=tuple(position.position_covariance),
        covariance_type=position.covariance_type,
    )


@dataclass(frozen=True)
class UbxNavPvtEpoch:
    """Fields required from one decoded UBX-NAV-PVT ROS message."""

    i_tow_ms: int
    gnss_fix_ok: bool
    carrier_solution: CarrierSolution
    receive_monotonic_ns: int


@dataclass(frozen=True)
class UbxNavHpPosLlhEpoch:
    """High-precision position fields from one UBX-NAV-HPPOSLLH message."""

    i_tow_ms: int
    lon: int
    lat: int
    height_mm: int
    lon_hp: int
    lat_hp: int
    height_hp: int
    invalid_lon: bool
    invalid_lat: bool
    invalid_height: bool
    invalid_lon_hp: bool
    invalid_lat_hp: bool
    invalid_height_hp: bool
    receive_monotonic_ns: int


@dataclass(frozen=True)
class UbxNavCovEpoch:
    """Position covariance fields from one UBX-NAV-COV message (NED frame)."""

    i_tow_ms: int
    pos_cor_valid: bool
    pos_cov_nn: float
    pos_cov_ne: float
    pos_cov_nd: float
    pos_cov_ee: float
    pos_cov_ed: float
    pos_cov_dd: float
    receive_monotonic_ns: int


@dataclass(frozen=True)
class UbxNavStatusEpoch:
    """Quality cross-check fields from one UBX-NAV-STATUS message."""

    i_tow_ms: int
    gnss_fix_ok: bool
    carrier_solution_valid: bool
    carrier_solution: CarrierSolution
    receive_monotonic_ns: int


def _validate_ubx_epoch_i_tow(name: str, value: int) -> None:
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value < UBX_GPS_WEEK_MS
    ):
        raise EpochPairingError(
            f"{name} is outside the UBX GPS week millisecond range"
        )


def _validate_monotonic_receive_times(named_values: tuple) -> tuple:
    values = []
    for name, value in named_values:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 0
        ):
            raise EpochPairingError(
                f"{name} must be a non-negative host monotonic timestamp"
            )
        values.append(value)
    return tuple(values)


def ned_covariance_to_enu(
    nn: float,
    ne: float,
    nd: float,
    ee: float,
    ed: float,
    dd: float,
) -> tuple:
    """Convert UBX-NAV-COV's symmetric NED covariance to row-major ENU.

    UBX-NAV-COV provides the upper triangle of C_NED. The frame rotation is
    ENU = [E, N, -D], so C_ENU = R * C_NED * R.T. In particular, the
    cross-terms involving Down change sign; merely swapping the diagonal
    variances is incorrect.
    """
    values = (nn, ne, nd, ee, ed, dd)
    try:
        finite_values = tuple(float(value) for value in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise EpochPairingError("NAV-COV position covariance must be numeric") from exc
    if not all(math.isfinite(value) for value in finite_values):
        raise EpochPairingError("NAV-COV position covariance contains NaN or Inf")

    nn, ne, nd, ee, ed, dd = finite_values
    return (
        ee, ne, -ed,
        ne, nn, -nd,
        -ed, -nd, dd,
    )


def build_observation_from_ubx_epochs(
    pvt: UbxNavPvtEpoch,
    position: UbxNavHpPosLlhEpoch,
    covariance: UbxNavCovEpoch,
    status: UbxNavStatusEpoch,
    *,
    source_timestamp_ns: int,
    source_clock_id: str,
    max_pair_receive_skew_ns: int,
) -> GnssObservation:
    """Join four raw UBX-derived messages on exactly one receiver iTOW.

    The caller must supply source_timestamp_ns from a separately validated
    receiver-time mapper (for example, a mapping based on a valid GPS week and
    receiver generation). Do not synthesize it from iTOW alone: iTOW wraps
    every GPS week and the driver's ROS header stamp is a host USB receive
    timestamp, not the receiver measurement time.

    receive_monotonic_ns values must be sampled by the adapter from one host
    steady/monotonic clock. The maximum-to-minimum arrival skew across all four
    records must stay within max_pair_receive_skew_ns.
    """
    if (
        isinstance(max_pair_receive_skew_ns, bool)
        or not isinstance(max_pair_receive_skew_ns, int)
        or max_pair_receive_skew_ns <= 0
    ):
        raise EpochPairingError("max_pair_receive_skew_ns must be a positive integer")

    for name, value in (
        ("pvt.i_tow_ms", pvt.i_tow_ms),
        ("position.i_tow_ms", position.i_tow_ms),
        ("covariance.i_tow_ms", covariance.i_tow_ms),
        ("status.i_tow_ms", status.i_tow_ms),
    ):
        _validate_ubx_epoch_i_tow(name, value)

    i_tows = (
        pvt.i_tow_ms,
        position.i_tow_ms,
        covariance.i_tow_ms,
        status.i_tow_ms,
    )
    if len(set(i_tows)) != 1:
        raise EpochPairingError(
            "UBX epoch mismatch: NAV-PVT, HPPOSLLH, NAV-COV and NAV-STATUS "
            "must have exactly equal iTOW values"
        )

    if not isinstance(source_clock_id, str) or not source_clock_id.strip():
        raise EpochPairingError(
            "source_clock_id must identify the GPS week and receiver clock generation"
        )
    if (
        isinstance(source_timestamp_ns, bool)
        or not isinstance(source_timestamp_ns, int)
        or source_timestamp_ns < 0
    ):
        raise EpochPairingError(
            "source_timestamp_ns must be supplied as a non-negative receiver-time mapping"
        )

    receive_times = _validate_monotonic_receive_times((
        ("pvt.receive_monotonic_ns", pvt.receive_monotonic_ns),
        ("position.receive_monotonic_ns", position.receive_monotonic_ns),
        ("covariance.receive_monotonic_ns", covariance.receive_monotonic_ns),
        ("status.receive_monotonic_ns", status.receive_monotonic_ns),
    ))
    if max(receive_times) - min(receive_times) > max_pair_receive_skew_ns:
        raise EpochPairingError(
            "UBX epoch receive times exceed the configured maximum pairing skew"
        )

    for name, flag in (
        ("PVT gnss_fix_ok", pvt.gnss_fix_ok),
        ("STATUS gnss_fix_ok", status.gnss_fix_ok),
        ("STATUS carrier_solution_valid", status.carrier_solution_valid),
        ("HPPOSLLH invalid_lon", position.invalid_lon),
        ("HPPOSLLH invalid_lat", position.invalid_lat),
        ("HPPOSLLH invalid_height", position.invalid_height),
        ("HPPOSLLH invalid_lon_hp", position.invalid_lon_hp),
        ("HPPOSLLH invalid_lat_hp", position.invalid_lat_hp),
        ("HPPOSLLH invalid_height_hp", position.invalid_height_hp),
        ("NAV-COV pos_cor_valid", covariance.pos_cor_valid),
    ):
        if not isinstance(flag, bool):
            raise EpochPairingError(f"{name} must be a boolean")

    if pvt.gnss_fix_ok != status.gnss_fix_ok:
        raise EpochPairingError(
            "NAV-PVT and NAV-STATUS disagree on GNSS fix validity"
        )
    if pvt.carrier_solution != status.carrier_solution:
        raise EpochPairingError(
            "NAV-PVT and NAV-STATUS disagree on carrier solution"
        )
    if pvt.carrier_solution == CarrierSolution.FIXED and not status.carrier_solution_valid:
        raise EpochPairingError(
            "NAV-STATUS does not mark the carrier solution as valid"
        )
    if not covariance.pos_cor_valid:
        raise EpochPairingError(
            "NAV-COV position covariance is not marked valid"
        )
    if any((
        position.invalid_lon,
        position.invalid_lat,
        position.invalid_height,
        position.invalid_lon_hp,
        position.invalid_lat_hp,
        position.invalid_height_hp,
    )):
        raise EpochPairingError(
            "NAV-HPPOSLLH marks one or more position components invalid"
        )

    for name, value in (
        ("position.lon", position.lon),
        ("position.lat", position.lat),
        ("position.height_mm", position.height_mm),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not -(2 ** 31) <= value < (2 ** 31)
        ):
            raise EpochPairingError(f"{name} must be a signed 32-bit integer")
    for name, value in (
        ("position.lon_hp", position.lon_hp),
        ("position.lat_hp", position.lat_hp),
        ("position.height_hp", position.height_hp),
    ):
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not -128 <= value <= 127
        ):
            raise EpochPairingError(f"{name} must be a signed 8-bit integer")

    latitude_deg = position.lat * 1e-7 + position.lat_hp * 1e-9
    longitude_deg = position.lon * 1e-7 + position.lon_hp * 1e-9
    altitude_m = position.height_mm * 1e-3 + position.height_hp * 1e-4
    if not all(math.isfinite(value) for value in (
        latitude_deg, longitude_deg, altitude_m
    )):
        raise EpochPairingError("HPPOSLLH conversion produced a non-finite position")

    enu_covariance = ned_covariance_to_enu(
        covariance.pos_cov_nn,
        covariance.pos_cov_ne,
        covariance.pos_cov_nd,
        covariance.pos_cov_ee,
        covariance.pos_cov_ed,
        covariance.pos_cov_dd,
    )
    return GnssObservation(
        source_timestamp_ns=source_timestamp_ns,
        source_clock_id=source_clock_id,
        receive_monotonic_ns=max(receive_times),
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        # sensor_msgs/NavSatStatus.STATUS_FIX is 0. The explicit GNSS fix
        # flag remains the quality authority; invalid fixes map to STATUS_NO_FIX.
        navsat_status=0 if pvt.gnss_fix_ok else -1,
        gnss_fix_ok=pvt.gnss_fix_ok,
        carrier_solution=pvt.carrier_solution,
        position_covariance=enu_covariance,
        covariance_type=CovarianceType.KNOWN,
    )
