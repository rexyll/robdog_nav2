"""Bounded exact-iTOW joiner for the four u-blox UBX navigation streams.

The ROS adapter is expected to decode each ROS message into the small immutable
records in ubx_nav_pvt.py, stamp callbacks with one host monotonic clock, and
supply a validated receiver-time mapping. This module owns only deterministic
cache, ordering, duplicate and epoch-join policy.
"""

from collections import OrderedDict
from dataclasses import dataclass, replace
from typing import Dict, Optional

from .quality_gate import GnssObservation
from .ubx_nav_pvt import (
    EpochPairingError,
    UbxNavCovEpoch,
    UbxNavHpPosLlhEpoch,
    UbxNavPvtEpoch,
    UbxNavStatusEpoch,
    UBX_GPS_WEEK_MS,
    build_observation_from_ubx_epochs,
)


@dataclass
class _PendingEpoch:
    source_timestamp_ns: int
    messages: Dict[str, object]
    first_receive_monotonic_ns: int
    last_receive_monotonic_ns: int


class UbxFourMessageEpochJoiner:
    """Join NAV-PVT, HPPOSLLH, NAV-COV and NAV-STATUS by exact receiver epoch.

    source_clock_id identifies one receiver generation and GPS week. The
    caller must call begin_generation() when the validated receiver-time
    mapper changes week or detects a receiver reset. The helper intentionally
    refuses implicit generation changes so a late callback from an old
    generation cannot silently clear the new generation's cache.

    The pending cache is LRU-bounded by max_pending_epochs. Incomplete epochs
    are evicted when their oldest host-monotonic receive time exceeds
    max_pending_age_ns. All receive timestamps must share one host monotonic
    clock. Completed source timestamps must be strictly increasing.
    """

    _REQUIRED = ("pvt", "position", "covariance", "status")

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
        self._source_clock_id: Optional[str] = None
        self._requires_explicit_generation = False
        self._pending: OrderedDict[int, _PendingEpoch] = OrderedDict()
        self._retired: OrderedDict[int, tuple] = OrderedDict()
        self._retired_limit = max(8, max_pending_epochs * 4)
        self._latest_receive_monotonic_ns: Optional[int] = None
        self._last_emitted_source_timestamp_ns: Optional[int] = None
        self._evicted_capacity = 0
        self._evicted_stale = 0
        self._rejected_conflict = 0
        self._rejected_out_of_order = 0

    @property
    def source_clock_id(self) -> Optional[str]:
        return self._source_clock_id

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def evicted_capacity(self) -> int:
        return self._evicted_capacity

    @property
    def evicted_stale(self) -> int:
        return self._evicted_stale

    @property
    def rejected_conflict(self) -> int:
        return self._rejected_conflict

    @property
    def rejected_out_of_order(self) -> int:
        return self._rejected_out_of_order

    @property
    def retired_count(self) -> int:
        return len(self._retired)

    def _retire(self, i_tow_ms: int, reason: str, source_timestamp_ns: int) -> None:
        self._retired[i_tow_ms] = (reason, source_timestamp_ns)
        self._retired.move_to_end(i_tow_ms)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def begin_generation(self, source_clock_id: str) -> None:
        """Clear all epoch state and explicitly begin a validated receiver epoch."""
        if not isinstance(source_clock_id, str) or not source_clock_id.strip():
            raise EpochPairingError("source_clock_id must be a non-empty generation ID")
        self._source_clock_id = source_clock_id
        self._requires_explicit_generation = False
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_monotonic_ns = None
        self._last_emitted_source_timestamp_ns = None

    def reset(self) -> None:
        """Clear all state and require begin_generation() before the next sample."""
        self._source_clock_id = None
        self._requires_explicit_generation = True
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_monotonic_ns = None
        self._last_emitted_source_timestamp_ns = None

    def expire_stale(self, now_monotonic_ns: int) -> int:
        """Evict incomplete epochs older than the configured pending-age bound."""
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < 0
        ):
            raise EpochPairingError("now_monotonic_ns must be a non-negative integer")
        if (
            self._latest_receive_monotonic_ns is None
            or now_monotonic_ns > self._latest_receive_monotonic_ns
        ):
            self._latest_receive_monotonic_ns = now_monotonic_ns
        stale_keys = [
            i_tow
            for i_tow, entry in self._pending.items()
            if now_monotonic_ns - entry.first_receive_monotonic_ns
            > self.max_pending_age_ns
        ]
        for i_tow in stale_keys:
            entry = self._pending.pop(i_tow)
            self._retire(i_tow, "stale", entry.source_timestamp_ns)
        self._evicted_stale += len(stale_keys)
        return len(stale_keys)

    def add_pvt(
        self, message: UbxNavPvtEpoch, *, source_clock_id: str,
        source_timestamp_ns: int,
    ) -> Optional[GnssObservation]:
        return self._add(
            "pvt", message, source_clock_id=source_clock_id,
            source_timestamp_ns=source_timestamp_ns,
        )

    def add_position(
        self, message: UbxNavHpPosLlhEpoch, *, source_clock_id: str,
        source_timestamp_ns: int,
    ) -> Optional[GnssObservation]:
        return self._add(
            "position", message, source_clock_id=source_clock_id,
            source_timestamp_ns=source_timestamp_ns,
        )

    def add_covariance(
        self, message: UbxNavCovEpoch, *, source_clock_id: str,
        source_timestamp_ns: int,
    ) -> Optional[GnssObservation]:
        return self._add(
            "covariance", message, source_clock_id=source_clock_id,
            source_timestamp_ns=source_timestamp_ns,
        )

    def add_status(
        self, message: UbxNavStatusEpoch, *, source_clock_id: str,
        source_timestamp_ns: int,
    ) -> Optional[GnssObservation]:
        return self._add(
            "status", message, source_clock_id=source_clock_id,
            source_timestamp_ns=source_timestamp_ns,
        )

    @staticmethod
    def _normalized_message(message: object) -> object:
        """Ignore receive time when comparing duplicate payloads."""
        try:
            return replace(message, receive_monotonic_ns=0)
        except (TypeError, ValueError) as exc:
            raise EpochPairingError("UBX epoch records must be supported dataclasses") from exc

    def _add(
        self,
        kind: str,
        message: object,
        *,
        source_clock_id: str,
        source_timestamp_ns: int,
    ) -> Optional[GnssObservation]:
        expected_types = {
            "pvt": UbxNavPvtEpoch,
            "position": UbxNavHpPosLlhEpoch,
            "covariance": UbxNavCovEpoch,
            "status": UbxNavStatusEpoch,
        }
        if kind not in expected_types or not isinstance(message, expected_types[kind]):
            raise EpochPairingError(f"{kind} must use its matching immutable UBX epoch record")
        if not isinstance(source_clock_id, str) or not source_clock_id.strip():
            raise EpochPairingError("source_clock_id must be a non-empty generation ID")
        process_session_id = getattr(message, "process_session_id", None)
        connection_generation = getattr(message, "connection_generation", None)
        if not isinstance(process_session_id, str) or not process_session_id.strip():
            raise EpochPairingError("UBX epoch record must carry process_session_id")
        if (
            isinstance(connection_generation, bool)
            or not isinstance(connection_generation, int)
            or connection_generation < 0
        ):
            raise EpochPairingError("UBX epoch record must carry a valid connection_generation")
        expected_prefix = (
            f"{process_session_id}:connection:{connection_generation}:gps-week:"
        )
        if not source_clock_id.startswith(expected_prefix):
            raise EpochPairingError(
                "stale process_session_id/connection_generation does not match source_clock_id"
            )
        if (
            isinstance(source_timestamp_ns, bool)
            or not isinstance(source_timestamp_ns, int)
            or source_timestamp_ns < 0
        ):
            raise EpochPairingError("source_timestamp_ns must be a non-negative integer")
        if self._source_clock_id is None:
            if self._requires_explicit_generation:
                raise EpochPairingError(
                    "begin_generation() is required after reset before accepting new receiver data"
                )
            self.begin_generation(source_clock_id)
        elif source_clock_id != self._source_clock_id:
            raise EpochPairingError(
                "source_clock_id changed; call begin_generation() after validating receiver week/reset"
            )

        receive_time = getattr(message, "receive_monotonic_ns", None)
        if (
            isinstance(receive_time, bool)
            or not isinstance(receive_time, int)
            or receive_time < 0
        ):
            raise EpochPairingError("UBX message receive time must be a non-negative monotonic integer")
        i_tow = getattr(message, "i_tow_ms", None)
        if (
            isinstance(i_tow, bool)
            or not isinstance(i_tow, int)
            or not 0 <= i_tow < UBX_GPS_WEEK_MS
        ):
            raise EpochPairingError(
                "UBX message iTOW is outside the GPS week millisecond range"
            )

        latest_receive = self._latest_receive_monotonic_ns
        if (
            latest_receive is not None
            and receive_time < latest_receive - self.max_pair_receive_skew_ns
        ):
            old_entry = self._pending.pop(i_tow, None)
            self._retire(
                i_tow,
                "late",
                old_entry.source_timestamp_ns if old_entry is not None else source_timestamp_ns,
            )
            raise EpochPairingError(
                "UBX message arrived outside the monotonic receive-time window"
            )
        if latest_receive is None or receive_time > latest_receive:
            self._latest_receive_monotonic_ns = receive_time
        self.expire_stale(self._latest_receive_monotonic_ns)

        retired = self._retired.get(i_tow)
        if retired is not None:
            reason, retired_timestamp = retired
            if reason == "completed" and retired_timestamp != source_timestamp_ns:
                raise EpochPairingError(
                    "completed iTOW was supplied with a conflicting receiver-time mapping"
                )
            return None

        entry = self._pending.get(i_tow)
        if entry is None:
            entry = _PendingEpoch(
                source_timestamp_ns=source_timestamp_ns,
                messages={},
                first_receive_monotonic_ns=receive_time,
                last_receive_monotonic_ns=receive_time,
            )
            self._pending[i_tow] = entry
        elif entry.source_timestamp_ns != source_timestamp_ns:
            del self._pending[i_tow]
            self._retire(i_tow, "mapping-conflict", entry.source_timestamp_ns)
            self._rejected_conflict += 1
            raise EpochPairingError(
                "receiver-time mapping changed for messages with the same iTOW"
            )

        previous = entry.messages.get(kind)
        if previous is not None:
            if self._normalized_message(previous) != self._normalized_message(message):
                del self._pending[i_tow]
                self._retire(i_tow, "conflicting-duplicate", entry.source_timestamp_ns)
                self._rejected_conflict += 1
                raise EpochPairingError(
                    f"conflicting duplicate UBX {kind} message for iTOW {i_tow}"
                )
            return None

        entry.messages[kind] = message
        entry.last_receive_monotonic_ns = max(entry.last_receive_monotonic_ns, receive_time)
        self._pending.move_to_end(i_tow)

        while len(self._pending) > self.max_pending_epochs:
            evicted_i_tow, evicted = self._pending.popitem(last=False)
            self._retire(evicted_i_tow, "capacity", evicted.source_timestamp_ns)
            self._evicted_capacity += 1

        if not all(required in entry.messages for required in self._REQUIRED):
            return None

        del self._pending[i_tow]
        try:
            observation = build_observation_from_ubx_epochs(
                entry.messages["pvt"],
                entry.messages["position"],
                entry.messages["covariance"],
                entry.messages["status"],
                source_timestamp_ns=entry.source_timestamp_ns,
                source_clock_id=self._source_clock_id,
                max_pair_receive_skew_ns=self.max_pair_receive_skew_ns,
            )
        except EpochPairingError:
            self._retire(i_tow, "invalid", entry.source_timestamp_ns)
            raise
        last_emitted = self._last_emitted_source_timestamp_ns
        if last_emitted is not None and observation.source_timestamp_ns <= last_emitted:
            self._rejected_out_of_order += 1
            self._retire(i_tow, "out-of-order", entry.source_timestamp_ns)
            raise EpochPairingError(
                "completed GNSS epoch is not newer than the last emitted source timestamp"
            )
        self._last_emitted_source_timestamp_ns = observation.source_timestamp_ns
        self._retire(i_tow, "completed", observation.source_timestamp_ns)
        return observation
