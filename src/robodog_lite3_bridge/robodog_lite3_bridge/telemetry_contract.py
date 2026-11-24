"""Transport-agnostic validation for the proposed Lite3 telemetry envelope.

This module deliberately does not implement UDP, protobuf, CBOR, DDS, or vendor
SDK access. It validates envelope metadata before a later gateway adapter maps
payloads into standard ROS messages.
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Optional, Sequence, Tuple


PROTOCOL_MAJOR_SUPPORTED = 1


class SequenceDisposition(str, Enum):
    FIRST = "first"
    IN_ORDER = "in_order"
    GAP = "gap"
    DUPLICATE = "duplicate"
    OUT_OF_ORDER = "out_of_order"
    SESSION_RESET = "session_reset"


@dataclass(frozen=True)
class StreamEnvelope:
    protocol_major: int
    protocol_minor: int
    message_type: str
    robot_id: str
    session_id: str
    sequence: int
    source_timestamp_ns: Optional[int]
    source_clock_id: Optional[str]
    gateway_monotonic_ns: int
    frame_id: Optional[str]
    unit_convention: str
    payload_length: int


@dataclass(frozen=True)
class SequenceObservation:
    disposition: SequenceDisposition
    missing_count: int = 0


class EnvelopeValidationError(ValueError):
    """Raised when metadata is unsafe or ambiguous."""


def validate_envelope(envelope: StreamEnvelope) -> None:
    """Validate mandatory metadata without interpreting payload semantics."""
    if envelope.protocol_major != PROTOCOL_MAJOR_SUPPORTED:
        raise EnvelopeValidationError(
            f"unsupported protocol major: {envelope.protocol_major}"
        )
    if envelope.protocol_minor < 0:
        raise EnvelopeValidationError("protocol_minor must be non-negative")
    if not envelope.message_type.strip():
        raise EnvelopeValidationError("message_type is required")
    if not envelope.robot_id.strip():
        raise EnvelopeValidationError("robot_id is required")
    if not envelope.session_id.strip():
        raise EnvelopeValidationError("session_id is required")
    if envelope.sequence < 0:
        raise EnvelopeValidationError("sequence must be non-negative")
    if envelope.gateway_monotonic_ns <= 0:
        raise EnvelopeValidationError("gateway_monotonic_ns must be positive")
    if envelope.payload_length < 0:
        raise EnvelopeValidationError("payload_length must be non-negative")
    if not envelope.unit_convention.strip():
        raise EnvelopeValidationError("unit_convention is required")

    if envelope.source_timestamp_ns is None:
        if envelope.source_clock_id not in (None, "", "no_source_time"):
            raise EnvelopeValidationError(
                "source_clock_id must be absent or 'no_source_time' "
                "when source_timestamp_ns is absent"
            )
    else:
        if envelope.source_timestamp_ns < 0:
            raise EnvelopeValidationError("source_timestamp_ns must be non-negative")
        if not envelope.source_clock_id or not envelope.source_clock_id.strip():
            raise EnvelopeValidationError(
                "source_clock_id is required when source timestamp is present"
            )


class StreamSequenceTracker:
    """Tracks message ordering for one logical telemetry stream.

    A changed session_id indicates a gateway restart or source reset. Sequence
    numbers are expected to be monotonically increasing uint64 values within a
    session. Wrap-around is deliberately not inferred by this v1 helper.
    """

    def __init__(self) -> None:
        self._session_id: Optional[str] = None
        self._last_sequence: Optional[int] = None

    @property
    def session_id(self) -> Optional[str]:
        return self._session_id

    @property
    def last_sequence(self) -> Optional[int]:
        return self._last_sequence

    def observe(self, session_id: str, sequence: int) -> SequenceObservation:
        if not session_id or not session_id.strip():
            raise ValueError("session_id is required")
        if sequence < 0:
            raise ValueError("sequence must be non-negative")

        if self._session_id is None:
            self._session_id = session_id
            self._last_sequence = sequence
            return SequenceObservation(SequenceDisposition.FIRST)

        if session_id != self._session_id:
            self._session_id = session_id
            self._last_sequence = sequence
            return SequenceObservation(SequenceDisposition.SESSION_RESET)

        assert self._last_sequence is not None
        if sequence == self._last_sequence:
            return SequenceObservation(SequenceDisposition.DUPLICATE)
        if sequence < self._last_sequence:
            return SequenceObservation(SequenceDisposition.OUT_OF_ORDER)

        missing = sequence - self._last_sequence - 1
        self._last_sequence = sequence
        if missing > 0:
            return SequenceObservation(SequenceDisposition.GAP, missing_count=missing)
        return SequenceObservation(SequenceDisposition.IN_ORDER)


def require_finite_values(values: Iterable[float], field_name: str) -> Tuple[float, ...]:
    """Return a finite tuple or reject NaN/Inf values."""
    converted = tuple(float(value) for value in values)
    for index, value in enumerate(converted):
        if not math.isfinite(value):
            raise ValueError(f"{field_name}[{index}] is not finite")
    return converted


def require_vector_length(
    values: Sequence[float],
    expected_length: int,
    field_name: str,
) -> Tuple[float, ...]:
    """Validate vector size and finite values."""
    if expected_length < 0:
        raise ValueError("expected_length must be non-negative")
    if len(values) != expected_length:
        raise ValueError(
            f"{field_name} length {len(values)} != expected {expected_length}"
        )
    return require_finite_values(values, field_name)
