"""Fail-closed quality gate for GNSS fixes with explicit RTK carrier status.

This module deliberately has no ROS or receiver-driver dependency. A future ROS
adapter must pair NavSatFix and receiver-specific status (for example, u-blox
NAV-PVT) by GNSS epoch before constructing GnssObservation.
"""

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
from typing import Optional, Sequence, Tuple


EARTH_MEAN_RADIUS_M = 6_371_008.8


class CarrierSolution(str, Enum):
    """Carrier-phase solution state supplied by the receiver adapter."""

    NO_SOLUTION = "no_solution"
    FLOAT = "float"
    FIXED = "fixed"
    UNKNOWN = "unknown"


class CovarianceType(IntEnum):
    """sensor_msgs/NavSatFix position_covariance_type values."""

    UNKNOWN = 0
    APPROXIMATED = 1
    DIAGONAL_KNOWN = 2
    KNOWN = 3


class RejectionReason(str, Enum):
    ACCEPTED = "accepted"
    RECOVERING = "recovering"
    GNSS_FIX_INVALID = "gnss_fix_invalid"
    RTK_NOT_FIXED = "rtk_not_fixed"
    RECEIVE_TIME_INVALID = "receive_time_invalid"
    FIX_STALE = "fix_stale"
    SOURCE_TIME_INVALID = "source_time_invalid"
    OUT_OF_ORDER = "out_of_order"
    COORDINATE_INVALID = "coordinate_invalid"
    COVARIANCE_TYPE_UNKNOWN = "covariance_type_unknown"
    COVARIANCE_APPROXIMATED = "covariance_approximated"
    COVARIANCE_INVALID = "covariance_invalid"
    COVARIANCE_DEGENERATE = "covariance_degenerate"
    HORIZONTAL_UNCERTAINTY = "horizontal_uncertainty"
    VERTICAL_UNCERTAINTY = "vertical_uncertainty"
    POSITION_JUMP = "position_jump"


@dataclass(frozen=True)
class GnssObservation:
    """One epoch-aligned GNSS position and receiver-quality observation.

    position_covariance is the row-major 3x3 ENU covariance from NavSatFix.
    source_timestamp_ns identifies the same measurement epoch as the receiver
    carrier-solution state. receive_monotonic_ns is stamped by the receiving
    host and is used only for transport freshness.
    """

    source_timestamp_ns: int
    source_clock_id: str
    receive_monotonic_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    navsat_status: int
    gnss_fix_ok: bool
    carrier_solution: CarrierSolution
    position_covariance: Tuple[float, ...]
    covariance_type: CovarianceType


@dataclass(frozen=True)
class GateConfig:
    """Explicit project thresholds; no hardware accuracy is assumed by default."""

    max_fix_age_ns: int
    max_horizontal_std_m: float
    max_vertical_std_m: float
    max_plausible_speed_mps: float
    required_consecutive_fixed_samples: int = 3
    max_jump_reference_interval_ns: int = 2_000_000_000
    allow_approximated_covariance: bool = False
    min_covariance_trace_m2: float = 1e-12
    symmetry_relative_tolerance: float = 1e-9
    psd_relative_tolerance: float = 1e-10

    def __post_init__(self) -> None:
        if self.max_fix_age_ns <= 0:
            raise ValueError("max_fix_age_ns must be positive")
        if self.max_jump_reference_interval_ns <= 0:
            raise ValueError("max_jump_reference_interval_ns must be positive")
        if self.required_consecutive_fixed_samples < 1:
            raise ValueError("required_consecutive_fixed_samples must be >= 1")
        for name in (
            "max_horizontal_std_m",
            "max_vertical_std_m",
            "max_plausible_speed_mps",
            "min_covariance_trace_m2",
            "symmetry_relative_tolerance",
            "psd_relative_tolerance",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.max_horizontal_std_m <= 0.0:
            raise ValueError("max_horizontal_std_m must be positive")
        if self.max_vertical_std_m <= 0.0:
            raise ValueError("max_vertical_std_m must be positive")
        if self.max_plausible_speed_mps <= 0.0:
            raise ValueError("max_plausible_speed_mps must be positive")
        if self.min_covariance_trace_m2 <= 0.0:
            raise ValueError("min_covariance_trace_m2 must be positive")
        if self.symmetry_relative_tolerance <= 0.0:
            raise ValueError("symmetry_relative_tolerance must be positive")
        if self.psd_relative_tolerance <= 0.0:
            raise ValueError("psd_relative_tolerance must be positive")


@dataclass(frozen=True)
class QualityDecision:
    qualified: bool
    reason: RejectionReason
    consecutive_fixed_samples: int
    horizontal_std_m: Optional[float] = None
    vertical_std_m: Optional[float] = None
    estimated_speed_mps: Optional[float] = None


@dataclass(frozen=True)
class _AcceptedReference:
    source_timestamp_ns: int
    source_clock_id: str
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


def _finite_number(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _covariance_uncertainty(
    values: Sequence[float],
    covariance_type: CovarianceType,
    config: GateConfig,
) -> Tuple[float, float]:
    if len(values) != 9:
        raise ValueError("NavSatFix position covariance must contain 9 values")

    matrix_values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in matrix_values):
        raise ValueError("position covariance contains NaN or Inf")

    if covariance_type == CovarianceType.UNKNOWN:
        raise LookupError("position covariance type is unknown")
    if covariance_type == CovarianceType.APPROXIMATED and not config.allow_approximated_covariance:
        raise PermissionError("approximated covariance is disabled")
    if covariance_type not in (
        CovarianceType.APPROXIMATED,
        CovarianceType.DIAGONAL_KNOWN,
        CovarianceType.KNOWN,
    ):
        raise ValueError("unsupported position covariance type")

    matrix = [
        list(matrix_values[0:3]),
        list(matrix_values[3:6]),
        list(matrix_values[6:9]),
    ]
    scale = max(abs(value) for value in matrix_values)
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if scale == 0.0 or trace < config.min_covariance_trace_m2:
        raise ArithmeticError("position covariance is degenerate")

    symmetry_tolerance = config.symmetry_relative_tolerance * max(scale, 1e-30)
    if covariance_type == CovarianceType.DIAGONAL_KNOWN:
        for row, column in ((0, 1), (0, 2), (1, 2)):
            if max(abs(matrix[row][column]), abs(matrix[column][row])) > symmetry_tolerance:
                raise ValueError("diagonal-known covariance must have zero off-diagonal entries")
        matrix[0][1] = matrix[1][0] = 0.0
        matrix[0][2] = matrix[2][0] = 0.0
        matrix[1][2] = matrix[2][1] = 0.0
    for row, column in ((0, 1), (0, 2), (1, 2)):
        if abs(matrix[row][column] - matrix[column][row]) > symmetry_tolerance:
            raise ValueError("position covariance must be symmetric")
        average = 0.5 * matrix[row][column] + 0.5 * matrix[column][row]
        matrix[row][column] = average
        matrix[column][row] = average

    a, b, c = matrix[0][0], matrix[1][1], matrix[2][2]
    xy, xz, yz = matrix[0][1], matrix[0][2], matrix[1][2]
    principal_tolerance_1 = config.psd_relative_tolerance * max(scale, 1e-30)
    principal_tolerance_2 = config.psd_relative_tolerance * max(scale * scale, 1e-300)
    principal_tolerance_3 = config.psd_relative_tolerance * max(scale * scale * scale, 1e-300)

    if min(a, b, c) < -principal_tolerance_1:
        raise ValueError("position covariance has a negative principal variance")
    if (
        a * b - xy * xy < -principal_tolerance_2
        or a * c - xz * xz < -principal_tolerance_2
        or b * c - yz * yz < -principal_tolerance_2
    ):
        raise ValueError("position covariance is not positive semidefinite")

    determinant = (
        a * b * c
        + 2.0 * xy * xz * yz
        - a * yz * yz
        - b * xz * xz
        - c * xy * xy
    )
    if determinant < -principal_tolerance_3:
        raise ValueError("position covariance is not positive semidefinite")

    horizontal_largest_eigenvalue = 0.5 * (
        a + b + math.hypot(a - b, 2.0 * xy)
    )
    if horizontal_largest_eigenvalue < -principal_tolerance_1 or c < -principal_tolerance_1:
        raise ValueError("position covariance has a negative variance")

    horizontal_std = math.sqrt(max(0.0, horizontal_largest_eigenvalue))
    vertical_std = math.sqrt(max(0.0, c))
    return horizontal_std, vertical_std


def _distance_3d_m(left: _AcceptedReference, right: GnssObservation) -> float:
    """Haversine surface distance plus altitude delta, robust at the dateline."""
    lat1 = math.radians(left.latitude_deg)
    lat2 = math.radians(right.latitude_deg)
    delta_lat = lat2 - lat1
    delta_lon = math.radians(right.longitude_deg - left.longitude_deg)
    haversine = (
        math.sin(delta_lat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon / 2.0) ** 2
    )
    haversine = min(1.0, max(0.0, haversine))
    surface_distance = 2.0 * EARTH_MEAN_RADIUS_M * math.asin(math.sqrt(haversine))
    vertical_delta = right.altitude_m - left.altitude_m
    return math.hypot(surface_distance, vertical_delta)


class RtkQualityGate:
    """Stateful fail-closed gate with fixed-solution recovery hysteresis."""

    def __init__(self, config: GateConfig) -> None:
        self.config = config
        self._consecutive_fixed_samples = 0
        self._last_reference: Optional[_AcceptedReference] = None

    @property
    def consecutive_fixed_samples(self) -> int:
        return self._consecutive_fixed_samples

    def reset(self) -> None:
        """Clear recovery and jump-reference state after an external reset."""
        self._consecutive_fixed_samples = 0
        self._last_reference = None

    def _reject(
        self,
        reason: RejectionReason,
        *,
        reset: bool = True,
        horizontal_std_m: Optional[float] = None,
        vertical_std_m: Optional[float] = None,
        estimated_speed_mps: Optional[float] = None,
    ) -> QualityDecision:
        if reset:
            self.reset()
        return QualityDecision(
            qualified=False,
            reason=reason,
            consecutive_fixed_samples=self._consecutive_fixed_samples,
            horizontal_std_m=horizontal_std_m,
            vertical_std_m=vertical_std_m,
            estimated_speed_mps=estimated_speed_mps,
        )

    def evaluate(
        self,
        observation: GnssObservation,
        now_monotonic_ns: int,
    ) -> QualityDecision:
        """Evaluate one epoch-paired GNSS observation.

        now_monotonic_ns and receive_monotonic_ns must use the same host
        monotonic clock. Source timestamps are compared only with previous
        source timestamps from the same receiver clock epoch.
        """
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or isinstance(observation.receive_monotonic_ns, bool)
            or not isinstance(observation.receive_monotonic_ns, int)
            or now_monotonic_ns < 0
            or observation.receive_monotonic_ns < 0
        ):
            return self._reject(RejectionReason.RECEIVE_TIME_INVALID)
        if observation.receive_monotonic_ns > now_monotonic_ns:
            return self._reject(RejectionReason.RECEIVE_TIME_INVALID)
        age_ns = now_monotonic_ns - observation.receive_monotonic_ns
        if age_ns > self.config.max_fix_age_ns:
            return self._reject(RejectionReason.FIX_STALE)

        if (
            isinstance(observation.source_timestamp_ns, bool)
            or not isinstance(observation.source_timestamp_ns, int)
            or observation.source_timestamp_ns < 0
            or not isinstance(observation.source_clock_id, str)
            or not observation.source_clock_id.strip()
        ):
            return self._reject(RejectionReason.SOURCE_TIME_INVALID)
        if self._last_reference is not None:
            if observation.source_clock_id != self._last_reference.source_clock_id:
                # A receiver reboot or clock-domain switch starts a new epoch.
                # Never compare timestamps or jump distance across clock epochs.
                self.reset()
            elif observation.source_timestamp_ns <= self._last_reference.source_timestamp_ns:
                return self._reject(RejectionReason.OUT_OF_ORDER, reset=False)

        if (
            not observation.gnss_fix_ok
            or isinstance(observation.navsat_status, bool)
            or not isinstance(observation.navsat_status, int)
            or observation.navsat_status not in (0, 1, 2)
        ):
            return self._reject(RejectionReason.GNSS_FIX_INVALID)
        if (
            not isinstance(observation.carrier_solution, CarrierSolution)
            or observation.carrier_solution != CarrierSolution.FIXED
        ):
            return self._reject(RejectionReason.RTK_NOT_FIXED)

        if (
            not _finite_number(observation.latitude_deg)
            or not _finite_number(observation.longitude_deg)
            or not _finite_number(observation.altitude_m)
        ):
            return self._reject(RejectionReason.COORDINATE_INVALID)
        latitude = float(observation.latitude_deg)
        longitude = float(observation.longitude_deg)
        altitude = float(observation.altitude_m)
        if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            return self._reject(RejectionReason.COORDINATE_INVALID)

        try:
            covariance_type = CovarianceType(observation.covariance_type)
            horizontal_std, vertical_std = _covariance_uncertainty(
                observation.position_covariance,
                covariance_type,
                self.config,
            )
        except LookupError:
            return self._reject(RejectionReason.COVARIANCE_TYPE_UNKNOWN)
        except PermissionError:
            return self._reject(RejectionReason.COVARIANCE_APPROXIMATED)
        except (TypeError, ValueError, ArithmeticError):
            covariance_values = observation.position_covariance
            if (
                isinstance(covariance_values, (tuple, list))
                and len(covariance_values) == 9
                and all(_finite_number(value) for value in covariance_values)
                and max(abs(float(value)) for value in covariance_values) == 0.0
            ):
                return self._reject(RejectionReason.COVARIANCE_DEGENERATE)
            return self._reject(RejectionReason.COVARIANCE_INVALID)

        if horizontal_std > self.config.max_horizontal_std_m:
            return self._reject(
                RejectionReason.HORIZONTAL_UNCERTAINTY,
                horizontal_std_m=horizontal_std,
                vertical_std_m=vertical_std,
            )
        if vertical_std > self.config.max_vertical_std_m:
            return self._reject(
                RejectionReason.VERTICAL_UNCERTAINTY,
                horizontal_std_m=horizontal_std,
                vertical_std_m=vertical_std,
            )

        estimated_speed: Optional[float] = None
        if self._last_reference is not None:
            delta_ns = observation.source_timestamp_ns - self._last_reference.source_timestamp_ns
            if delta_ns <= 0:
                return self._reject(RejectionReason.OUT_OF_ORDER, reset=False)
            if delta_ns > self.config.max_jump_reference_interval_ns:
                self._consecutive_fixed_samples = 0
            else:
                distance = _distance_3d_m(self._last_reference, observation)
                delta_seconds = delta_ns / 1_000_000_000.0
                estimated_speed = distance / delta_seconds
                if estimated_speed > self.config.max_plausible_speed_mps:
                    self.reset()
                    return QualityDecision(
                        qualified=False,
                        reason=RejectionReason.POSITION_JUMP,
                        consecutive_fixed_samples=0,
                        horizontal_std_m=horizontal_std,
                        vertical_std_m=vertical_std,
                        estimated_speed_mps=estimated_speed,
                    )

        self._last_reference = _AcceptedReference(
            source_timestamp_ns=observation.source_timestamp_ns,
            source_clock_id=observation.source_clock_id,
            latitude_deg=latitude,
            longitude_deg=longitude,
            altitude_m=altitude,
        )
        self._consecutive_fixed_samples += 1
        if self._consecutive_fixed_samples < self.config.required_consecutive_fixed_samples:
            return QualityDecision(
                qualified=False,
                reason=RejectionReason.RECOVERING,
                consecutive_fixed_samples=self._consecutive_fixed_samples,
                horizontal_std_m=horizontal_std,
                vertical_std_m=vertical_std,
                estimated_speed_mps=estimated_speed,
            )

        return QualityDecision(
            qualified=True,
            reason=RejectionReason.ACCEPTED,
            consecutive_fixed_samples=self._consecutive_fixed_samples,
            horizontal_std_m=horizontal_std,
            vertical_std_m=vertical_std,
            estimated_speed_mps=estimated_speed,
        )
