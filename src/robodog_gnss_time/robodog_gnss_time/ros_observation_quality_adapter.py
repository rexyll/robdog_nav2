"""Strict ROS-message adapter for exact observation/quality identity joins."""

import math

from robodog_gnss_quality import (
    GnssObservationIdentity,
    GnssObservationQualityJoiner,
    QualityDecision,
    RejectionReason,
)

from .gnss_rtk_quality_node import observation_from_ros


def _uint(name, value, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be an unsigned integer in [0, {maximum}]")
    return value


def identity_from_observation_message(message):
    """Build the canonical full identity from a GnssObservation ROS message."""
    observation, context = observation_from_ros(message)
    identity = GnssObservationIdentity(
        process_session_id=context[0],
        connection_generation=context[1],
        gps_week=context[2],
        i_tow_ms=context[3],
        source_timestamp_ns=observation.source_timestamp_ns,
        source_clock_id=observation.source_clock_id,
        receive_monotonic_ns=observation.receive_monotonic_ns,
    )
    return identity, observation


def decision_from_quality_message(message):
    """Convert a GnssQualityStatus without accepting enum or bool aliases."""
    if not isinstance(message.qualified, bool):
        raise ValueError("qualified must be bool")
    if not isinstance(message.reason, str) or not message.reason:
        raise ValueError("reason must be a non-empty string")
    try:
        reason = RejectionReason(message.reason)
    except ValueError as exc:
        raise ValueError(f"unknown quality reason: {message.reason!r}") from exc
    consecutive = _uint("consecutive_fixed_samples", message.consecutive_fixed_samples, (1 << 32) - 1)

    def optional_metric(valid_name, value_name):
        valid = getattr(message, valid_name)
        value = getattr(message, value_name)
        if not isinstance(valid, bool):
            raise ValueError(f"{valid_name} must be bool")
        if not valid:
            if value != 0.0:
                raise ValueError(f"{value_name} must use zero sentinel when invalid")
            return None
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{value_name} must be finite when valid")
        if value < 0:
            raise ValueError(f"{value_name} must be non-negative")
        return float(value)

    decision = QualityDecision(
        qualified=message.qualified,
        reason=reason,
        consecutive_fixed_samples=consecutive,
        horizontal_std_m=optional_metric("horizontal_std_valid", "horizontal_std_m"),
        vertical_std_m=optional_metric("vertical_std_valid", "vertical_std_m"),
        estimated_speed_mps=optional_metric("estimated_speed_valid", "estimated_speed_mps"),
    )
    if decision.qualified != (decision.reason == RejectionReason.ACCEPTED):
        raise ValueError("qualified must be true exactly when reason is accepted")

    identity = GnssObservationIdentity(
        process_session_id=message.process_session_id,
        connection_generation=_uint("connection_generation", message.connection_generation, (1 << 64) - 1),
        gps_week=_uint("gps_week", message.gps_week, 32767),
        i_tow_ms=_uint("i_tow_ms", message.i_tow_ms, 604799999),
        source_timestamp_ns=_uint("source_timestamp_ns", message.source_timestamp_ns, (1 << 64) - 1),
        source_clock_id=message.source_clock_id,
        receive_monotonic_ns=_uint("receive_monotonic_ns", message.receive_monotonic_ns, (1 << 64) - 1),
    )
    return identity, decision


class RosObservationQualityJoinAdapter:
    """Bounded exact-identity adapter; emits only qualified matching pairs."""

    def __init__(self, *, max_pending_epochs=64, max_pending_age_ns=1_000_000_000):
        self._joiner = GnssObservationQualityJoiner(
            max_pending_epochs=max_pending_epochs,
            max_pending_age_ns=max_pending_age_ns,
        )

    def add_observation(self, message, *, now_monotonic_ns):
        identity, observation = identity_from_observation_message(message)
        return self._joiner.add_observation(
            identity, observation, now_monotonic_ns=now_monotonic_ns
        )

    @staticmethod
    def _is_unbound_startup_status(message):
        """Recognize only the quality node's identity-free startup sentinel."""
        return (
            message.reason == "no_data"
            and message.qualified is False
            and message.consecutive_fixed_samples == 0
            and message.process_session_id == ""
            and message.connection_generation == 0
            and message.gps_week == 0
            and message.i_tow_ms == 0
            and message.source_timestamp_ns == 0
            and message.source_clock_id == ""
            and message.receive_monotonic_ns == 0
            and message.horizontal_std_valid is False
            and message.horizontal_std_m == 0.0
            and message.vertical_std_valid is False
            and message.vertical_std_m == 0.0
            and message.estimated_speed_valid is False
            and message.estimated_speed_mps == 0.0
        )

    def add_decision(self, message, *, now_monotonic_ns):
        # The quality node publishes this one unbound sentinel at startup.
        # It has no epoch identity and must not enter the exact-identity join.
        if self._is_unbound_startup_status(message):
            return None
        identity, decision = decision_from_quality_message(message)
        return self._joiner.add_decision(
            identity, decision, now_monotonic_ns=now_monotonic_ns
        )

    def expire_stale(self, *, now_monotonic_ns):
        self._joiner.expire_stale(now_monotonic_ns)

    def stats(self):
        return self._joiner.stats()
