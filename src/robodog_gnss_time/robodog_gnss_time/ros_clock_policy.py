"""Fail-closed ROS/SystemTime-to-UTC policy for a future NavSatFix publisher.

This module does not discover host clock synchronization. A trusted runtime
monitor must provide fresh offset/uncertainty measurements; absent evidence is
a rejection, never an implicit synchronized state.
"""

from dataclasses import dataclass
from enum import Enum
import math


INT32_MAX = (1 << 31) - 1
NSEC_PER_SEC = 1_000_000_000


class ClockPolicyReason(str, Enum):
    ACCEPTED = "accepted"
    SIM_TIME_ACTIVE = "sim_time_active"
    UTC_INVALID = "utc_invalid"
    UTC_TIMESTAMP_INVALID = "utc_timestamp_invalid"
    UTC_SECONDS_OUT_OF_RANGE = "utc_seconds_out_of_range"
    LEAP_GUARD_ACTIVE = "leap_guard_active"
    MONOTONIC_TIME_INVALID = "monotonic_time_invalid"
    HOST_SYNC_UNKNOWN = "host_sync_unknown"
    HOST_SYNC_SOURCE_MISSING = "host_sync_source_missing"
    HOST_SYNC_SAMPLE_STALE = "host_sync_sample_stale"
    HOST_SYNC_OFFSET_INVALID = "host_sync_offset_invalid"
    HOST_SYNC_UNCERTAINTY_INVALID = "host_sync_uncertainty_invalid"
    HOST_SYNC_ERROR_EXCEEDED = "host_sync_error_exceeded"


@dataclass(frozen=True)
class ClockPolicyConfig:
    """Thresholds must be allocated by the system timing/integrity budget."""

    max_host_utc_error_ns: int
    max_sync_sample_age_ns: int

    def __post_init__(self):
        for name, value in (
            ("max_host_utc_error_ns", self.max_host_utc_error_ns),
            ("max_sync_sample_age_ns", self.max_sync_sample_age_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class ClockPolicyInput:
    use_sim_time: bool
    utc_stamp_valid: bool
    utc_seconds: object
    utc_nanoseconds: object
    leap_guard_active: bool
    host_sync_state: str
    host_sync_source: str
    host_utc_offset_ns: object
    host_utc_uncertainty_ns: object
    host_sync_sample_monotonic_ns: object


@dataclass(frozen=True)
class ClockPolicyDecision:
    allowed: bool
    reason: ClockPolicyReason
    host_utc_error_bound_ns: object = None


class RosSystemTimeUtcPolicy:
    """Allow a real-world UTC stamp only with explicit fresh sync evidence."""

    def __init__(self, config: ClockPolicyConfig):
        self.config = config

    @staticmethod
    def _unsigned_integer(value):
        return (
            isinstance(value, int)
            and not isinstance(value, bool)
            and value >= 0
        )

    def evaluate(self, sample: ClockPolicyInput, *, now_monotonic_ns: int):
        if not isinstance(sample, ClockPolicyInput):
            raise TypeError("sample must be ClockPolicyInput")
        if not self._unsigned_integer(now_monotonic_ns):
            return ClockPolicyDecision(False, ClockPolicyReason.MONOTONIC_TIME_INVALID)
        if not isinstance(sample.use_sim_time, bool):
            return ClockPolicyDecision(False, ClockPolicyReason.SIM_TIME_ACTIVE)
        if sample.use_sim_time:
            return ClockPolicyDecision(False, ClockPolicyReason.SIM_TIME_ACTIVE)
        if not isinstance(sample.utc_stamp_valid, bool) or not sample.utc_stamp_valid:
            return ClockPolicyDecision(False, ClockPolicyReason.UTC_INVALID)
        if (
            not self._unsigned_integer(sample.utc_seconds)
            or not self._unsigned_integer(sample.utc_nanoseconds)
            or sample.utc_nanoseconds >= NSEC_PER_SEC
        ):
            return ClockPolicyDecision(False, ClockPolicyReason.UTC_TIMESTAMP_INVALID)
        if sample.utc_seconds > INT32_MAX:
            return ClockPolicyDecision(False, ClockPolicyReason.UTC_SECONDS_OUT_OF_RANGE)
        if not isinstance(sample.leap_guard_active, bool) or sample.leap_guard_active:
            return ClockPolicyDecision(False, ClockPolicyReason.LEAP_GUARD_ACTIVE)
        if sample.host_sync_state != "synchronized":
            return ClockPolicyDecision(False, ClockPolicyReason.HOST_SYNC_UNKNOWN)
        if not isinstance(sample.host_sync_source, str) or not sample.host_sync_source.strip():
            return ClockPolicyDecision(False, ClockPolicyReason.HOST_SYNC_SOURCE_MISSING)
        if not self._unsigned_integer(sample.host_sync_sample_monotonic_ns):
            return ClockPolicyDecision(False, ClockPolicyReason.MONOTONIC_TIME_INVALID)
        if sample.host_sync_sample_monotonic_ns > now_monotonic_ns:
            return ClockPolicyDecision(False, ClockPolicyReason.MONOTONIC_TIME_INVALID)
        if now_monotonic_ns - sample.host_sync_sample_monotonic_ns > self.config.max_sync_sample_age_ns:
            return ClockPolicyDecision(False, ClockPolicyReason.HOST_SYNC_SAMPLE_STALE)
        if (
            isinstance(sample.host_utc_offset_ns, bool)
            or not isinstance(sample.host_utc_offset_ns, int)
        ):
            return ClockPolicyDecision(False, ClockPolicyReason.HOST_SYNC_OFFSET_INVALID)
        uncertainty = sample.host_utc_uncertainty_ns
        if not self._unsigned_integer(uncertainty):
            return ClockPolicyDecision(False, ClockPolicyReason.HOST_SYNC_UNCERTAINTY_INVALID)
        error_bound = abs(sample.host_utc_offset_ns) + uncertainty
        if error_bound > self.config.max_host_utc_error_ns:
            return ClockPolicyDecision(
                False,
                ClockPolicyReason.HOST_SYNC_ERROR_EXCEEDED,
                error_bound,
            )
        return ClockPolicyDecision(True, ClockPolicyReason.ACCEPTED, error_bound)
