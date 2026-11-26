"""Strict UBX-NAV-TIMEGPS / UBX-NAV-TIMELS decoder and receiver-time mapper.

GPS source time is computed from valid week + iTOW + fTOW. UTC conversion is
separate and requires valid, explicitly trusted NAV-TIMELS current-leap data
that agrees with NAV-TIMEGPS.leapS. No NAV-TIMEUTC fallback is used.
"""

from collections import OrderedDict
from dataclasses import dataclass, replace
from typing import Dict, Optional, Set

from .ros_epoch_adapter import UbxSourceTimeMapping
from .ubx_nav_pvt import EpochPairingError, UBX_GPS_WEEK_MS


GPS_WEEK_NS = 604_800 * 1_000_000_000
GPS_EPOCH_UNIX_SECONDS = 315_964_800
GPS_EPOCH_UNIX_NS = GPS_EPOCH_UNIX_SECONDS * 1_000_000_000
UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class UbxNavTimeGps:
    i_tow_ms: int
    f_tow_ns: int
    week: int
    leap_seconds: int
    valid_flags: int
    time_accuracy_ns: int

    @property
    def tow_valid(self) -> bool:
        return bool(self.valid_flags & 0x01)

    @property
    def week_valid(self) -> bool:
        return bool(self.valid_flags & 0x02)

    @property
    def leap_seconds_valid(self) -> bool:
        return bool(self.valid_flags & 0x04)


@dataclass(frozen=True)
class UbxNavTimeLs:
    i_tow_ms: int
    version: int
    source_of_current_leap: int
    current_leap_seconds: int
    source_of_leap_change: int
    leap_change: int
    time_to_leap_event_seconds: int
    leap_event_gps_week: int
    leap_event_gps_day: int
    valid_flags: int

    @property
    def current_leap_valid(self) -> bool:
        return bool(self.valid_flags & 0x01)

    @property
    def leap_event_valid(self) -> bool:
        return bool(self.valid_flags & 0x02)


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


@dataclass
class _PendingTimePair:
    first_receive_monotonic_ns: int
    timegps: Optional[UbxNavTimeGps] = None
    timegps_receive_ns: Optional[int] = None
    timels: Optional[UbxNavTimeLs] = None
    timels_receive_ns: Optional[int] = None


def decode_nav_timegps(payload: bytes) -> UbxNavTimeGps:
    """Decode the 16-byte UBX-NAV-TIMEGPS payload."""
    if not isinstance(payload, (bytes, bytearray, memoryview)) or len(payload) != 16:
        raise EpochPairingError("NAV-TIMEGPS payload must be exactly 16 bytes")
    data = bytes(payload)
    i_tow = int.from_bytes(data[0:4], "little", signed=False)
    f_tow = int.from_bytes(data[4:8], "little", signed=True)
    week = int.from_bytes(data[8:10], "little", signed=True)
    leap_seconds = int.from_bytes(data[10:11], "little", signed=True)
    valid_flags = data[11]
    time_accuracy_ns = int.from_bytes(data[12:16], "little", signed=False)
    if i_tow >= UBX_GPS_WEEK_MS:
        raise EpochPairingError("NAV-TIMEGPS iTOW is outside GPS week")
    if not -500_000 <= f_tow <= 500_000:
        raise EpochPairingError("NAV-TIMEGPS fTOW is outside +/-500000 ns")
    if not 0 <= week <= 32_767:
        raise EpochPairingError("NAV-TIMEGPS week is outside signed GPS week range")
    return UbxNavTimeGps(
        i_tow, f_tow, week, leap_seconds, valid_flags, time_accuracy_ns
    )


def decode_nav_timels(payload: bytes) -> UbxNavTimeLs:
    """Decode the 24-byte UBX-NAV-TIMELS payload (version 0)."""
    if not isinstance(payload, (bytes, bytearray, memoryview)) or len(payload) != 24:
        raise EpochPairingError("NAV-TIMELS payload must be exactly 24 bytes")
    data = bytes(payload)
    i_tow = int.from_bytes(data[0:4], "little", signed=False)
    version = data[4]
    source_current = data[8]
    current_leap = int.from_bytes(data[9:10], "little", signed=True)
    source_change = data[10]
    leap_change = int.from_bytes(data[11:12], "little", signed=True)
    time_to_event = int.from_bytes(data[12:16], "little", signed=True)
    event_week = int.from_bytes(data[16:18], "little", signed=False)
    event_day = int.from_bytes(data[18:20], "little", signed=False)
    valid_flags = data[23]
    if i_tow >= UBX_GPS_WEEK_MS:
        raise EpochPairingError("NAV-TIMELS iTOW is outside GPS week")
    if version != 0:
        raise EpochPairingError(f"unsupported NAV-TIMELS version {version}")
    return UbxNavTimeLs(
        i_tow, version, source_current, current_leap, source_change,
        leap_change, time_to_event, event_week, event_day, valid_flags
    )


class UbxReceiverTimeMapper:
    """Builds exact-iTOW GPS/UTC mappings from paired TIMEGPS and TIMELS.

    trusted_leap_sources is intentionally empty by default. The application
    must explicitly approve source IDs after checking receiver/constellation
    configuration. Source ID 255 (unknown) cannot be trusted.
    """

    def __init__(
        self,
        *,
        trusted_leap_sources: Set[int] = frozenset(),
        max_pending_epochs: int = 8,
        max_pair_receive_skew_ns: int = 500_000_000,
        max_pending_age_ns: int = 2_000_000_000,
    ) -> None:
        for name, value in (
            ("max_pending_epochs", max_pending_epochs),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns),
            ("max_pending_age_ns", max_pending_age_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if any(
            isinstance(source, bool) or not isinstance(source, int) or not 0 <= source <= 8
            for source in trusted_leap_sources
        ):
            raise ValueError("trusted leap sources must be explicit NAV-TIMELS source IDs 0..8")
        self.trusted_leap_sources = frozenset(trusted_leap_sources)
        self.max_pending_epochs = max_pending_epochs
        self.max_pair_receive_skew_ns = max_pair_receive_skew_ns
        self.max_pending_age_ns = max_pending_age_ns
        self._session_id: Optional[str] = None
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

    def begin_session(self, receiver_session_id: str) -> None:
        if not isinstance(receiver_session_id, str) or not receiver_session_id.strip():
            raise EpochPairingError("receiver_session_id must be non-empty")
        self._session_id = receiver_session_id
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_ns = None

    def reset(self) -> None:
        self._session_id = None
        self._pending.clear()
        self._retired.clear()
        self._latest_receive_ns = None

    @staticmethod
    def _receive_time(value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
        return value

    def _retire(self, i_tow: int, reason: str) -> None:
        self._retired[i_tow] = reason
        self._retired.move_to_end(i_tow)
        while len(self._retired) > self._retired_limit:
            self._retired.popitem(last=False)

    def expire_stale(self, now_monotonic_ns: int) -> int:
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

    def add_timegps(
        self, payload: bytes, receive_monotonic_ns: int
    ) -> Optional[UbxSourceTimeMapping]:
        if self._session_id is None:
            raise EpochPairingError("begin_session() is required before TIMEGPS")
        message = decode_nav_timegps(payload)
        receive_ns = self._receive_time(receive_monotonic_ns)
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
        entry.timegps_receive_ns = receive_ns
        return self._try_map(message.i_tow_ms, entry)

    def add_timels(
        self, payload: bytes, receive_monotonic_ns: int
    ) -> Optional[UbxSourceTimeMapping]:
        if self._session_id is None:
            raise EpochPairingError("begin_session() is required before TIMELS")
        message = decode_nav_timels(payload)
        receive_ns = self._receive_time(receive_monotonic_ns)
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
        entry.timels_receive_ns = receive_ns
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
        if (
            entry.timegps_receive_ns is None
            or entry.timels_receive_ns is None
            or abs(entry.timegps_receive_ns - entry.timels_receive_ns)
               > self.max_pair_receive_skew_ns
        ):
            self._pair_skew_rejections += 1
            return None
        if not gps.tow_valid or not gps.week_valid:
            self._invalid_gps_time += 1
            return None

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

        utc_valid = (
            gps.leap_seconds_valid
            and leap.current_leap_valid
            and leap.source_of_current_leap in self.trusted_leap_sources
            and gps.leap_seconds == leap.current_leap_seconds
        )
        utc_seconds = None
        utc_nanoseconds = None
        reason = "GPS week/TOW valid; UTC leap mapping is not trusted"
        if utc_valid:
            utc_ns = (
                GPS_EPOCH_UNIX_NS + source_timestamp_ns
                - leap.current_leap_seconds * 1_000_000_000
            )
            if utc_ns >= 0:
                utc_seconds, utc_nanoseconds = divmod(utc_ns, 1_000_000_000)
                reason = "GPS week/TOW and trusted NAV-TIMELS leap mapping valid"
            else:
                utc_valid = False
                reason = "computed UTC timestamp is before Unix epoch"
        elif not gps.leap_seconds_valid:
            reason = "GPS week/TOW valid; NAV-TIMEGPS leapSValid is false"
        elif not leap.current_leap_valid:
            reason = "GPS week/TOW valid; NAV-TIMELS validCurrLs is false"
        elif leap.source_of_current_leap not in self.trusted_leap_sources:
            reason = "GPS week/TOW valid; NAV-TIMELS current leap source is not trusted"
        elif gps.leap_seconds != leap.current_leap_seconds:
            reason = "GPS week/TOW valid; TIMEGPS leapS differs from TIMELS currLs"

        if utc_valid:
            self._utc_valid_epochs += 1
        else:
            self._utc_untrusted_epochs += 1
        self._paired_epochs += 1
        source_clock_id = f"{self._session_id}:gps-week:{normalized_week}"
        return UbxSourceTimeMapping(
            i_tow_ms=gps.i_tow_ms,
            source_timestamp_ns=source_timestamp_ns,
            source_clock_id=source_clock_id,
            receive_monotonic_ns=max(
                entry.timegps_receive_ns, entry.timels_receive_ns
            ),
            valid=True,
            utc_stamp_valid=utc_valid,
            utc_seconds=utc_seconds,
            utc_nanoseconds=utc_nanoseconds,
            validity_reason=reason,
            gps_week=normalized_week,
            source_time_accuracy_ns=gps.time_accuracy_ns,
            leap_seconds=leap.current_leap_seconds,
            leap_source=leap.source_of_current_leap,
        )

    def stats(self) -> UbxTimeMapperStats:
        return UbxTimeMapperStats(
            self._paired_epochs, self._invalid_gps_time, self._utc_valid_epochs,
            self._utc_untrusted_epochs, self._expired_epochs, self._capacity_evictions,
            self._conflicting_duplicates, self._pair_skew_rejections,
        )
