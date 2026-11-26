"""Bounded ROS-message/time-mapping adapter for exact-iTOW GNSS assembly.

The receiver-time mapper is an independent component. This adapter buffers
decoded ROS message records until a validated per-iTOW mapping is available.
It never derives receiver time from ROS Header.stamp or iTOW alone.
"""

from collections import OrderedDict
from dataclasses import dataclass, replace
from threading import RLock
from typing import Dict, Optional

from .quality_gate import GnssObservation
from .ubx_epoch_joiner import UbxFourMessageEpochJoiner
from .ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
    UBX_GPS_WEEK_MS,
)


@dataclass(frozen=True)
class UbxSourceTimeMapping:
    i_tow_ms: int
    source_timestamp_ns: int
    source_clock_id: str
    receive_monotonic_ns: int
    valid: bool
    utc_stamp_valid: bool
    utc_seconds: Optional[int] = None
    utc_nanoseconds: Optional[int] = None
    validity_reason: str = ""
    gps_week: Optional[int] = None
    source_time_accuracy_ns: Optional[int] = None
    leap_seconds: Optional[int] = None
    leap_source: Optional[int] = None
    reported_leap_seconds: Optional[int] = None
    leap_event_valid: bool = False
    time_to_leap_event_seconds: Optional[int] = None
    process_session_id: Optional[str] = None
    connection_generation: Optional[int] = None


@dataclass(frozen=True)
class UbxRosEpochResult:
    observation: GnssObservation
    time_mapping: UbxSourceTimeMapping


@dataclass
class _PendingEpoch:
    first_receive_monotonic_ns: int
    messages: Dict[str, object]
    mapping: Optional[UbxSourceTimeMapping] = None


class UbxRosEpochAdapter:
    """Join four decoded UBX ROS streams only after validated time mapping arrives.

    begin_generation() is an explicit trust boundary. A changed source_clock_id
    is rejected until the caller has independently validated the new GPS week
    and receiver boot generation.
    """

    _KINDS = ("pvt", "position", "covariance", "status")
    _TYPES = {
        "pvt": UbxNavPvtEpoch,
        "position": UbxNavHpPosLlhEpoch,
        "covariance": UbxNavCovEpoch,
        "status": UbxNavStatusEpoch,
    }

    def __init__(
        self,
        *,
        max_pending_epochs: int,
        max_pair_receive_skew_ns: int,
        max_pending_age_ns: int,
    ) -> None:
        for name, value in (
            ("max_pending_epochs", max_pending_epochs),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns),
            ("max_pending_age_ns", max_pending_age_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        self.max_pending_epochs = max_pending_epochs
        self.max_pair_receive_skew_ns = max_pair_receive_skew_ns
        self.max_pending_age_ns = max_pending_age_ns
        self.joiner = UbxFourMessageEpochJoiner(
            max_pending_epochs=max_pending_epochs,
            max_pair_receive_skew_ns=max_pair_receive_skew_ns,
            max_pending_age_ns=max_pending_age_ns,
        )
        self._lock = RLock()
        self._source_clock_id: Optional[str] = None
        self._process_session_id: Optional[str] = None
        self._connection_generation: Optional[int] = None
        self._gps_week: Optional[int] = None
        self._pending: OrderedDict[int, _PendingEpoch] = OrderedDict()
        self._retired: OrderedDict[int, str] = OrderedDict()
        self._retired_limit = max(8, max_pending_epochs * 4)
        self._latest_receive_monotonic_ns: Optional[int] = None
        self._evicted_capacity = 0
        self._evicted_stale = 0
        self._rejected_conflict = 0
        self._invalid_mappings = 0
        self._stale_context_rejections = 0

    @property
    def source_clock_id(self) -> Optional[str]:
        with self._lock:
            return self._source_clock_id

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._pending)

    @property
    def evicted_capacity(self) -> int:
        with self._lock:
            return self._evicted_capacity

    @property
    def evicted_stale(self) -> int:
        with self._lock:
            return self._evicted_stale

    @property
    def rejected_conflict(self) -> int:
        with self._lock:
            return self._rejected_conflict

    @property
    def invalid_mappings(self) -> int:
        with self._lock:
            return self._invalid_mappings

    @property
    def stale_context_rejections(self) -> int:
        with self._lock:
            return self._stale_context_rejections

    def _retire(self, i_tow_ms: int, reason: str) -> None:
        self._retired[i_tow_ms] = reason
        self._retired.move_to_end(i_tow_ms)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def begin_generation(
        self, source_clock_id: str, *, process_session_id: str,
        connection_generation: int, gps_week: int
    ) -> None:
        with self._lock:
            self._begin_generation_locked(
                source_clock_id, process_session_id, connection_generation, gps_week
            )

    def _begin_generation_locked(
        self, source_clock_id: str, process_session_id: str,
        connection_generation: int, gps_week: int
    ) -> None:
        if not isinstance(process_session_id, str) or not process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")
        if (isinstance(connection_generation, bool)
                or not isinstance(connection_generation, int)
                or connection_generation < 0):
            raise EpochPairingError("connection_generation must be a non-negative integer")
        if isinstance(gps_week, bool) or not isinstance(gps_week, int) or not 0 <= gps_week <= 32767:
            raise EpochPairingError("gps_week must be a non-negative signed-16-bit integer")
        expected_clock_id = (
            f"{process_session_id}:connection:{connection_generation}:gps-week:{gps_week}"
        )
        if source_clock_id != expected_clock_id:
            raise EpochPairingError("source_clock_id does not match generation context")
        self._source_clock_id = source_clock_id
        self._process_session_id = process_session_id
        self._connection_generation = connection_generation
        self._gps_week = gps_week
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_monotonic_ns = None
        self.joiner.begin_generation(source_clock_id)

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()

    def _reset_locked(self) -> None:
        self._source_clock_id = None
        self._process_session_id = None
        self._connection_generation = None
        self._gps_week = None
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_monotonic_ns = None
        self.joiner.reset()

    @staticmethod
    def _validate_i_tow(value: object) -> int:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value < UBX_GPS_WEEK_MS
        ):
            raise EpochPairingError("iTOW is outside the GPS week millisecond range")
        return value

    @staticmethod
    def _validate_receive_time(value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
        return value

    def expire_stale(self, now_monotonic_ns: int) -> int:
        with self._lock:
            return self._expire_stale_locked(now_monotonic_ns)

    def _expire_stale_locked(self, now_monotonic_ns: int) -> int:
        now = self._validate_receive_time(now_monotonic_ns)
        if self._latest_receive_monotonic_ns is None or now > self._latest_receive_monotonic_ns:
            self._latest_receive_monotonic_ns = now
        effective_now = self._latest_receive_monotonic_ns
        stale = [
            i_tow for i_tow, entry in self._pending.items()
            if effective_now - entry.first_receive_monotonic_ns > self.max_pending_age_ns
        ]
        for i_tow in stale:
            self._pending.pop(i_tow, None)
            self._retire(i_tow, "stale")
        self._evicted_stale += len(stale)
        return len(stale)

    def _entry(self, i_tow_ms: int, receive_ns: int) -> Optional[_PendingEpoch]:
        if i_tow_ms in self._retired:
            return None
        latest = self._latest_receive_monotonic_ns
        if latest is not None and receive_ns < latest - self.max_pair_receive_skew_ns:
            self._pending.pop(i_tow_ms, None)
            self._retire(i_tow_ms, "late")
            return None
        if latest is None or receive_ns > latest:
            self._latest_receive_monotonic_ns = receive_ns
        self.expire_stale(self._latest_receive_monotonic_ns)
        if i_tow_ms in self._retired:
            return None

        entry = self._pending.get(i_tow_ms)
        if entry is None:
            entry = _PendingEpoch(receive_ns, {})
            self._pending[i_tow_ms] = entry
        self._pending.move_to_end(i_tow_ms)
        while len(self._pending) > self.max_pending_epochs:
            evicted_i_tow, _ = self._pending.popitem(last=False)
            self._retire(evicted_i_tow, "capacity")
            self._evicted_capacity += 1
        return self._pending.get(i_tow_ms)

    def _add_message(self, kind: str, message: object) -> Optional[UbxRosEpochResult]:
        with self._lock:
            return self._add_message_locked(kind, message)

    def _add_message_locked(self, kind: str, message: object) -> Optional[UbxRosEpochResult]:
        if self._source_clock_id is None:
            raise EpochPairingError("begin_generation() is required before accepting UBX records")
        expected = self._TYPES[kind]
        if not isinstance(message, expected):
            raise EpochPairingError(f"{kind} must use its matching immutable UBX epoch record")
        if (
            message.process_session_id != self._process_session_id
            or message.connection_generation != self._connection_generation
        ):
            self._stale_context_rejections += 1
            raise EpochPairingError(
                "stale process_session_id/connection_generation; discard old ROS callback"
            )
        i_tow = self._validate_i_tow(message.i_tow_ms)
        receive_ns = self._validate_receive_time(message.receive_monotonic_ns)
        entry = self._entry(i_tow, receive_ns)
        if entry is None:
            return None

        previous = entry.messages.get(kind)
        normalized = replace(message, receive_monotonic_ns=0)
        if previous is not None:
            if replace(previous, receive_monotonic_ns=0) != normalized:
                self._pending.pop(i_tow, None)
                self._retire(i_tow, "conflicting-duplicate")
                self._rejected_conflict += 1
                raise EpochPairingError(f"conflicting duplicate {kind} record for iTOW {i_tow}")
            return None
        entry.messages[kind] = message
        return self._try_emit(i_tow, entry)

    def add_pvt(self, message: UbxNavPvtEpoch) -> Optional[UbxRosEpochResult]:
        return self._add_message("pvt", message)

    def add_position(self, message: UbxNavHpPosLlhEpoch) -> Optional[UbxRosEpochResult]:
        return self._add_message("position", message)

    def add_covariance(self, message: UbxNavCovEpoch) -> Optional[UbxRosEpochResult]:
        return self._add_message("covariance", message)

    def add_status(self, message: UbxNavStatusEpoch) -> Optional[UbxRosEpochResult]:
        return self._add_message("status", message)

    def add_time_mapping(
        self, mapping: UbxSourceTimeMapping
    ) -> Optional[UbxRosEpochResult]:
        with self._lock:
            return self._add_time_mapping_locked(mapping)

    def _add_time_mapping_locked(
        self, mapping: UbxSourceTimeMapping
    ) -> Optional[UbxRosEpochResult]:
        if not isinstance(mapping, UbxSourceTimeMapping):
            raise EpochPairingError("mapping must be UbxSourceTimeMapping")
        if self._source_clock_id is None:
            raise EpochPairingError("begin_generation() is required before accepting time mapping")
        i_tow = self._validate_i_tow(mapping.i_tow_ms)
        receive_ns = self._validate_receive_time(mapping.receive_monotonic_ns)
        if mapping.source_clock_id != self._source_clock_id:
            raise EpochPairingError(
                "source_clock_id changed; validate receiver week/reset and call begin_generation()"
            )
        if (
            mapping.process_session_id != self._process_session_id
            or mapping.connection_generation != self._connection_generation
            or mapping.gps_week != self._gps_week
        ):
            self._stale_context_rejections += 1
            raise EpochPairingError("time mapping does not match session/generation/week context")
        expected_clock_id = (
            f"{self._process_session_id}:connection:{self._connection_generation}"
            f":gps-week:{self._gps_week}"
        )
        if mapping.source_clock_id != expected_clock_id:
            raise EpochPairingError("source_clock_id does not match active generation context")
        if not isinstance(mapping.valid, bool) or not isinstance(mapping.utc_stamp_valid, bool):
            raise EpochPairingError("time mapping validity fields must be booleans")
        if (
            isinstance(mapping.source_timestamp_ns, bool)
            or not isinstance(mapping.source_timestamp_ns, int)
            or mapping.source_timestamp_ns < 0
        ):
            raise EpochPairingError("source_timestamp_ns must be a non-negative receiver-time value")
        if mapping.utc_stamp_valid:
            if (
                isinstance(mapping.utc_seconds, bool)
                or not isinstance(mapping.utc_seconds, int)
                or mapping.utc_seconds < 0
                or isinstance(mapping.utc_nanoseconds, bool)
                or not isinstance(mapping.utc_nanoseconds, int)
                or not 0 <= mapping.utc_nanoseconds < 1_000_000_000
            ):
                raise EpochPairingError("valid UTC stamp requires seconds and nanoseconds in range")
        elif mapping.utc_seconds is not None or mapping.utc_nanoseconds is not None:
            raise EpochPairingError("invalid UTC stamp must not carry timestamp values")

        entry = self._entry(i_tow, receive_ns)
        if entry is None:
            return None
        if not mapping.valid:
            self._pending.pop(i_tow, None)
            self._retire(i_tow, "invalid-time-mapping")
            self._invalid_mappings += 1
            return None

        previous = entry.mapping
        normalized = replace(mapping, receive_monotonic_ns=0)
        if previous is not None:
            if replace(previous, receive_monotonic_ns=0) != normalized:
                self._pending.pop(i_tow, None)
                self._retire(i_tow, "conflicting-time-mapping")
                self._rejected_conflict += 1
                raise EpochPairingError(f"conflicting receiver-time mapping for iTOW {i_tow}")
            return None
        entry.mapping = mapping
        return self._try_emit(i_tow, entry)

    def _try_emit(
        self, i_tow: int, entry: _PendingEpoch
    ) -> Optional[UbxRosEpochResult]:
        if entry.mapping is None or not all(kind in entry.messages for kind in self._KINDS):
            return None
        mapping = entry.mapping
        try:
            observation = None
            for kind in self._KINDS:
                add = getattr(self.joiner, "add_" + kind)
                result = add(
                    entry.messages[kind],
                    source_clock_id=mapping.source_clock_id,
                    source_timestamp_ns=mapping.source_timestamp_ns,
                )
                if result is not None:
                    observation = result
            if observation is None:
                raise EpochPairingError("complete mapped epoch did not produce an observation")
        except EpochPairingError:
            self._pending.pop(i_tow, None)
            self._retire(i_tow, "invalid-or-out-of-order")
            raise
        self._pending.pop(i_tow, None)
        self._retire(i_tow, "completed")
        return UbxRosEpochResult(observation, mapping)
