"""Receiver-time mapper consuming session-fenced UBX TIMEGPS/TIMELS epochs.

Raw payload decoding is centralized in ubx_time_messages.py. Every decoded
message carries process_session_id, connection_generation and host monotonic
receive time; the mapper rejects stale-generation callbacks before pairing.
"""

from collections import OrderedDict
from dataclasses import dataclass
from threading import RLock
from typing import Optional, Set

from .ros_epoch_adapter import UbxSourceTimeMapping
from .ubx_nav_pvt import EpochPairingError
from .ubx_time_messages import (
    UbxNavTimeGpsEpoch,
    UbxNavTimeLsEpoch,
    decode_nav_timegps_payload,
    decode_nav_timels_payload,
)


GPS_WEEK_NS = 604_800 * 1_000_000_000
GPS_EPOCH_UNIX_SECONDS = 315_964_800
GPS_EPOCH_UNIX_NS = GPS_EPOCH_UNIX_SECONDS * 1_000_000_000
UINT64_MAX = (1 << 64) - 1
# NAV-TIMELS source 4 is BeiDou-UTC, 14 seconds behind GPS-UTC.
LEAP_SOURCE_TO_GPS_UTC_ADJUSTMENT_SECONDS = {4: 14}


@dataclass(frozen=True)
class UbxTimeMapperStats:
    paired_epochs: int
    invalid_gps_time: int
    utc_valid_epochs: int
    utc_untrusted_epochs: int
    expired_epochs: int
    capacity_evictions: int
    conflicting_duplicates: int
    pair_skew_rejections: int
    stale_context_rejections: int


@dataclass
class _PendingTimePair:
    first_receive_monotonic_ns: int
    timegps: Optional[UbxNavTimeGpsEpoch] = None
    timels: Optional[UbxNavTimeLsEpoch] = None


class UbxReceiverTimeMapper:
    """Build exact-iTOW mappings from session-fenced TIMEGPS and TIMELS epochs.

    trusted_leap_sources is empty by default. Source 0 (firmware default) and
    255 (unknown) are never trusted. Explicitly approved IDs 1..8 are allowed.
    """

    def __init__(
        self,
        *,
        trusted_leap_sources: Set[int] = frozenset(),
        max_pending_epochs: int = 8,
        max_pair_receive_skew_ns: int = 500_000_000,
        max_pending_age_ns: int = 2_000_000_000,
        leap_event_guard_seconds: int = 1,
        require_ingress_sequence: bool = False,
    ) -> None:
        for name, value in (
            ("max_pending_epochs", max_pending_epochs),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns),
            ("max_pending_age_ns", max_pending_age_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if any(
            isinstance(source, bool) or not isinstance(source, int) or not 1 <= source <= 8
            for source in trusted_leap_sources
        ):
            raise ValueError(
                "firmware default 0 is never trusted; allowlist only accepts explicit NAV-TIMELS source IDs 1..8"
            )
        if (
            isinstance(leap_event_guard_seconds, bool)
            or not isinstance(leap_event_guard_seconds, int)
            or leap_event_guard_seconds < 0
        ):
            raise ValueError("leap_event_guard_seconds must be a non-negative integer")

        if not isinstance(require_ingress_sequence, bool):
            raise ValueError("require_ingress_sequence must be a boolean")
        self.require_ingress_sequence = require_ingress_sequence
        self._last_ingress_sequence: Optional[int] = None
        self.leap_event_guard_seconds = leap_event_guard_seconds
        self.trusted_leap_sources = frozenset(trusted_leap_sources)
        self.max_pending_epochs = max_pending_epochs
        self.max_pair_receive_skew_ns = max_pair_receive_skew_ns
        self.max_pending_age_ns = max_pending_age_ns
        self._lock = RLock()
        self._process_session_id: Optional[str] = None
        self._connection_generation: Optional[int] = None
        self._pending: OrderedDict[int, _PendingTimePair] = OrderedDict()
        self._retired: OrderedDict[int, str] = OrderedDict()
        self._retired_limit = max(8, max_pending_epochs * 4)
        self._latest_receive_ns: Optional[int] = None
        self._paired_epochs = 0
        self._invalid_gps_time = 0
        self._utc_valid_epochs = 0
        self._utc_untrusted_epochs = 0
        self._expired_epochs = 0
        self._capacity_evictions = 0
        self._conflicting_duplicates = 0
        self._pair_skew_rejections = 0
        self._stale_context_rejections = 0

    @property
    def process_session_id(self) -> Optional[str]:
        with self._lock:
            return self._process_session_id

    @property
    def connection_generation(self) -> Optional[int]:
        with self._lock:
            return self._connection_generation

    def begin_session(self, process_session_id: str, connection_generation: int) -> None:
        with self._lock:
            self._begin_session_locked(process_session_id, connection_generation)

    def _begin_session_locked(self, process_session_id: str, connection_generation: int) -> None:
        if not isinstance(process_session_id, str) or not process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")
        if (
            isinstance(connection_generation, bool)
            or not isinstance(connection_generation, int)
            or connection_generation < 0
        ):
            raise EpochPairingError("connection_generation must be a non-negative integer")
        if self._process_session_id != process_session_id:
            self._last_ingress_sequence = None
        self._process_session_id = process_session_id
        self._connection_generation = connection_generation
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_ns = None

    def reset(self) -> None:
        with self._lock:
            self._process_session_id = None
            self._connection_generation = None
            self._last_ingress_sequence = None
            self._pending.clear()
            self._retired.clear()
            self._latest_receive_ns = None

    @staticmethod
    def _receive_time(value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
        return value

    def _validate_context(self, message: object) -> None:
        if self._process_session_id is None or self._connection_generation is None:
            raise EpochPairingError("begin_session() is required before TIMEGPS/TIMELS")
        if (
            message.process_session_id != self._process_session_id
            or message.connection_generation != self._connection_generation
        ):
            self._stale_context_rejections += 1
            raise EpochPairingError(
                "stale process_session_id/connection_generation; discard callback from old device generation"
            )
        sequence = getattr(message, "ingress_sequence", None)
        if self.require_ingress_sequence and sequence is None:
            self._stale_context_rejections += 1
            raise EpochPairingError("raw ingress message is missing ingress_sequence")
        if sequence is not None:
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                self._stale_context_rejections += 1
                raise EpochPairingError("ingress_sequence must be a non-negative integer")
            if (self._last_ingress_sequence is not None
                    and sequence <= self._last_ingress_sequence):
                self._stale_context_rejections += 1
                raise EpochPairingError(
                    "stale or reordered ingress_sequence; serialize raw ingress events"
                )
            self._last_ingress_sequence = sequence

    def _retire(self, i_tow: int, reason: str) -> None:
        self._retired[i_tow] = reason
        self._retired.move_to_end(i_tow)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def expire_stale(self, now_monotonic_ns: int) -> int:
        with self._lock:
            now = self._receive_time(now_monotonic_ns)
            if self._latest_receive_ns is None or now > self._latest_receive_ns:
                self._latest_receive_ns = now
            effective_now = self._latest_receive_ns
            stale = [
                i_tow for i_tow, entry in self._pending.items()
                if effective_now - entry.first_receive_monotonic_ns > self.max_pending_age_ns
            ]
            for i_tow in stale:
                self._pending.pop(i_tow, None)
                self._retire(i_tow, "stale")
            self._expired_epochs += len(stale)
            return len(stale)

    def _entry(self, i_tow: int, receive_ns: int) -> Optional[_PendingTimePair]:
        if i_tow in self._retired:
            return None
        if self._latest_receive_ns is None or receive_ns > self._latest_receive_ns:
            self._latest_receive_ns = receive_ns
        elif receive_ns < self._latest_receive_ns - self.max_pair_receive_skew_ns:
            self._pending.pop(i_tow, None)
            self._retire(i_tow, "late")
            return None
        self.expire_stale(self._latest_receive_ns)
        if i_tow in self._retired:
            return None
        entry = self._pending.get(i_tow)
        if entry is None:
            entry = _PendingTimePair(receive_ns)
            self._pending[i_tow] = entry
        entry.first_receive_monotonic_ns = min(entry.first_receive_monotonic_ns, receive_ns)
        self._pending.move_to_end(i_tow)
        while len(self._pending) > self.max_pending_epochs:
            old_i_tow, _ = self._pending.popitem(last=False)
            self._retire(old_i_tow, "capacity")
            self._capacity_evictions += 1
        return self._pending.get(i_tow)

    def add_timegps(self, message: UbxNavTimeGpsEpoch) -> Optional[UbxSourceTimeMapping]:
        with self._lock:
            return self._add_timegps_locked(message)

    def add_timegps_payload(
        self,
        payload: bytes,
        receive_monotonic_ns: int,
        *,
        process_session_id: str,
        connection_generation: int,
        ingress_sequence: Optional[int] = None,
    ) -> Optional[UbxSourceTimeMapping]:
        message = decode_nav_timegps_payload(
            payload,
            receive_monotonic_ns=receive_monotonic_ns,
            process_session_id=process_session_id,
            connection_generation=connection_generation,
            ingress_sequence=ingress_sequence,
        )
        return self.add_timegps(message)

    def _add_timegps_locked(
        self, message: UbxNavTimeGpsEpoch
    ) -> Optional[UbxSourceTimeMapping]:
        if not isinstance(message, UbxNavTimeGpsEpoch):
            raise EpochPairingError("TIMEGPS must be a decoded UbxNavTimeGpsEpoch")
        self._validate_context(message)
        receive_ns = self._receive_time(message.receive_monotonic_ns)
        entry = self._entry(message.i_tow_ms, receive_ns)
        if entry is None:
            return None
        if entry.timegps is not None:
            if entry.timegps != message:
                self._pending.pop(message.i_tow_ms, None)
                self._retire(message.i_tow_ms, "conflicting-timegps")
                self._conflicting_duplicates += 1
                raise EpochPairingError("conflicting NAV-TIMEGPS duplicate")
            return None
        entry.timegps = message
        return self._try_map(message.i_tow_ms, entry)

    def add_timels(self, message: UbxNavTimeLsEpoch) -> Optional[UbxSourceTimeMapping]:
        with self._lock:
            return self._add_timels_locked(message)

    def add_timels_payload(
        self,
        payload: bytes,
        receive_monotonic_ns: int,
        *,
        process_session_id: str,
        connection_generation: int,
        ingress_sequence: Optional[int] = None,
    ) -> Optional[UbxSourceTimeMapping]:
        message = decode_nav_timels_payload(
            payload,
            receive_monotonic_ns=receive_monotonic_ns,
            process_session_id=process_session_id,
            connection_generation=connection_generation,
            ingress_sequence=ingress_sequence,
        )
        return self.add_timels(message)

    def _add_timels_locked(
        self, message: UbxNavTimeLsEpoch
    ) -> Optional[UbxSourceTimeMapping]:
        if not isinstance(message, UbxNavTimeLsEpoch):
            raise EpochPairingError("TIMELS must be a decoded UbxNavTimeLsEpoch")
        self._validate_context(message)
        receive_ns = self._receive_time(message.receive_monotonic_ns)
        entry = self._entry(message.i_tow_ms, receive_ns)
        if entry is None:
            return None
        if entry.timels is not None:
            if entry.timels != message:
                self._pending.pop(message.i_tow_ms, None)
                self._retire(message.i_tow_ms, "conflicting-timels")
                self._conflicting_duplicates += 1
                raise EpochPairingError("conflicting NAV-TIMELS duplicate")
            return None
        entry.timels = message
        return self._try_map(message.i_tow_ms, entry)

    def _try_map(
        self, i_tow: int, entry: _PendingTimePair
    ) -> Optional[UbxSourceTimeMapping]:
        if entry.timegps is None or entry.timels is None:
            return None
        gps = entry.timegps
        leap = entry.timels
        self._pending.pop(i_tow, None)
        self._retire(i_tow, "paired")
        if abs(gps.receive_monotonic_ns - leap.receive_monotonic_ns) > self.max_pair_receive_skew_ns:
            self._pair_skew_rejections += 1
            return None
        if not gps.tow_valid or not gps.week_valid or not 0 <= gps.week <= 32_767:
            self._invalid_gps_time += 1
            return None
        if leap.time_to_leap_event_valid and leap.leap_seconds_change not in (-1, 0, 1):
            raise EpochPairingError("valid NAV-TIMELS lsChange must be -1, 0, or +1")

        raw_tow_ns = gps.i_tow_ms * 1_000_000 + gps.f_tow_ns
        week_adjust, normalized_tow_ns = divmod(raw_tow_ns, GPS_WEEK_NS)
        normalized_week = gps.week + week_adjust
        if normalized_week < 0:
            self._invalid_gps_time += 1
            return None
        source_timestamp_ns = normalized_week * GPS_WEEK_NS + normalized_tow_ns
        if source_timestamp_ns > UINT64_MAX:
            self._invalid_gps_time += 1
            return None

        source_adjustment = LEAP_SOURCE_TO_GPS_UTC_ADJUSTMENT_SECONDS.get(
            leap.source_current_leap_seconds, 0
        )
        gps_utc_leap_seconds = leap.current_leap_seconds + source_adjustment
        utc_valid = (
            gps.leap_seconds_valid
            and leap.current_leap_seconds_valid
            and leap.source_current_leap_seconds in self.trusted_leap_sources
            and 0 <= gps_utc_leap_seconds <= 127
            and gps.leap_seconds == gps_utc_leap_seconds
        )
        utc_seconds = None
        utc_nanoseconds = None
        reason = "GPS week/TOW valid; UTC leap mapping is not trusted"
        leap_event_near = (
            leap.time_to_leap_event_valid
            and abs(leap.time_to_leap_event_seconds) <= self.leap_event_guard_seconds
        )
        if utc_valid and leap_event_near:
            utc_valid = False
            reason = "GPS time is within the leap-second transition guard window"
        if utc_valid:
            utc_ns = (
                GPS_EPOCH_UNIX_NS + source_timestamp_ns
                - gps_utc_leap_seconds * 1_000_000_000
            )
            if utc_ns >= 0:
                utc_seconds, utc_nanoseconds = divmod(utc_ns, 1_000_000_000)
                reason = "GPS week/TOW and trusted NAV-TIMELS leap mapping valid"
            else:
                utc_valid = False
                reason = "computed UTC timestamp is before Unix epoch"
        elif not gps.leap_seconds_valid:
            reason = "GPS week/TOW valid; NAV-TIMEGPS leapSValid is false"
        elif not leap.current_leap_seconds_valid:
            reason = "GPS week/TOW valid; NAV-TIMELS validCurrLs is false"
        elif leap.source_current_leap_seconds not in self.trusted_leap_sources:
            reason = "GPS week/TOW valid; NAV-TIMELS current leap source is not trusted"
        elif not 0 <= gps_utc_leap_seconds <= 127:
            reason = "GPS week/TOW valid; source-adjusted GPS-UTC leap offset is outside 0..127"
        elif gps.leap_seconds != gps_utc_leap_seconds:
            reason = "GPS week/TOW valid; TIMEGPS leapS differs from source-adjusted TIMELS currLs"
        elif leap_event_near:
            reason = "GPS week/TOW valid; UTC stamp withheld inside leap-second transition guard window"

        if utc_valid:
            self._utc_valid_epochs += 1
        else:
            self._utc_untrusted_epochs += 1
        self._paired_epochs += 1
        source_clock_id = (
            f"{self._process_session_id}:connection:{self._connection_generation}"
            f":gps-week:{normalized_week}"
        )
        return UbxSourceTimeMapping(
            i_tow_ms=gps.i_tow_ms,
            source_timestamp_ns=source_timestamp_ns,
            source_clock_id=source_clock_id,
            receive_monotonic_ns=max(
                gps.receive_monotonic_ns, leap.receive_monotonic_ns
            ),
            valid=True,
            utc_stamp_valid=utc_valid,
            utc_seconds=utc_seconds,
            utc_nanoseconds=utc_nanoseconds,
            validity_reason=reason,
            gps_week=normalized_week,
            source_time_accuracy_ns=gps.time_accuracy_ns,
            leap_seconds=gps_utc_leap_seconds,
            leap_source=leap.source_current_leap_seconds,
            reported_leap_seconds=leap.current_leap_seconds,
            leap_event_valid=leap.time_to_leap_event_valid,
            time_to_leap_event_seconds=(
                leap.time_to_leap_event_seconds if leap.time_to_leap_event_valid else None
            ),
            process_session_id=self._process_session_id,
            connection_generation=self._connection_generation,
        )

    def stats(self) -> UbxTimeMapperStats:
        with self._lock:
            return self._stats_locked()

    def _stats_locked(self) -> UbxTimeMapperStats:
        return UbxTimeMapperStats(
            self._paired_epochs, self._invalid_gps_time, self._utc_valid_epochs,
            self._utc_untrusted_epochs, self._expired_epochs, self._capacity_evictions,
            self._conflicting_duplicates, self._pair_skew_rejections,
            self._stale_context_rejections,
        )
