"""u-blox UBX-NAV-PVT quality decoding and exact-epoch pairing helpers.

No serial decoder or ROS driver is included. The caller must obtain these
fields from the configured receiver driver and preserve the NAV-PVT iTOW
alongside the corresponding NavSatFix record.
"""

from dataclasses import dataclass

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
