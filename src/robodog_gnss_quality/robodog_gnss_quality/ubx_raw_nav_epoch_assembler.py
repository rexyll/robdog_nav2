"""Join raw UBX NAV frames to validated TIMEGPS/TIMELS mappings.

This path consumes the same ordered USB ingress lifecycle as the receiver-time
mapper. It therefore carries the transport's process session and connection
generation directly, unlike third-party decoded ROS topics.
"""

from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from .ros_epoch_adapter import UbxRosEpochAdapter, UbxRosEpochResult, UbxSourceTimeMapping
from .ubx_ingress_event_processor import GnssIngressEvent
from .ubx_nav_ingress_adapter import (
    UbxNavRawIngressRecord,
    decode_ubx_nav_ingress_frame,
    decode_ubx_nav_raw_record,
)
from .ubx_nav_pvt import EpochPairingError


@dataclass
class _PendingRawEpoch:
    first_receive_monotonic_ns: int
    records: Dict[str, UbxNavRawIngressRecord] = field(default_factory=dict)


@dataclass(frozen=True)
class UbxRawNavAssemblerStats:
    decoded_nav_frames: int
    buffered_before_mapping: int
    rejected_receive_skew: int
    rejected_stale_generation: int
    duplicate_frames: int
    conflicting_epochs: int
    expired_raw_epochs: int
    capacity_evictions: int
    emitted_observations: int


class UbxRawNavEpochAssembler:
    """Bounded exact-iTOW join of raw NAV frames and validated receiver time."""

    _EVENT_NAMES = {
        0: "GenerationStarted", 1: "Frame", 2: "DeviceLost",
        3: "TransportFault", 4: "Stopped",
    }

    def __init__(
        self,
        *,
        max_pending_epochs: int,
        max_pair_receive_skew_ns: int,
        max_pending_age_ns: int,
    ):
        for name, value, minimum in (
            ("max_pending_epochs", max_pending_epochs, 1),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns, 1),
            ("max_pending_age_ns", max_pending_age_ns, 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} must be an integer >= {minimum}")
        self.max_pending_epochs = max_pending_epochs
        self.max_pair_receive_skew_ns = max_pair_receive_skew_ns
        self.max_pending_age_ns = max_pending_age_ns
        self.adapter = UbxRosEpochAdapter(
            max_pending_epochs=max_pending_epochs,
            max_pair_receive_skew_ns=max_pair_receive_skew_ns,
            max_pending_age_ns=max_pending_age_ns,
        )
        self._transport_context: Optional[Tuple[str, int]] = None
        self._time_context: Optional[Tuple[str, int, int]] = None
        self._raw_pending: OrderedDict[int, _PendingRawEpoch] = OrderedDict()
        self._mapping_pending: OrderedDict[int, UbxSourceTimeMapping] = OrderedDict()
        self._retired: OrderedDict[int, None] = OrderedDict()
        self._latest_receive_monotonic_ns: Optional[int] = None
        self._decoded_nav_frames = 0
        self._buffered_before_mapping = 0
        self._rejected_receive_skew = 0
        self._rejected_stale_generation = 0
        self._duplicate_frames = 0
        self._conflicting_epochs = 0
        self._expired_raw_epochs = 0
        self._capacity_evictions = 0
        self._emitted_observations = 0

    @staticmethod
    def _event_kind(event):
        value = event.event_type
        if isinstance(value, bool):
            raise EpochPairingError("ingress event_type must not be bool")
        if isinstance(value, int):
            value = UbxRawNavEpochAssembler._EVENT_NAMES.get(value)
        if value not in ("GenerationStarted", "Frame", "DeviceLost", "TransportFault", "Stopped"):
            raise EpochPairingError("unknown GNSS ingress event_type")
        return value

    def reset(self):
        self.adapter.reset()
        self._transport_context = None
        self._time_context = None
        self._raw_pending.clear()
        self._mapping_pending.clear()
        self._retired.clear()
        self._latest_receive_monotonic_ns = None

    def process_event(self, event: GnssIngressEvent) -> Optional[UbxRosEpochResult]:
        if not isinstance(event, GnssIngressEvent):
            raise EpochPairingError("event must be a GnssIngressEvent")
        kind = self._event_kind(event)
        if (
            isinstance(event.connection_generation, bool)
            or not isinstance(event.connection_generation, int)
            or event.connection_generation < 0
        ):
            raise EpochPairingError("connection_generation must be a non-negative integer")
        if (
            isinstance(event.receive_monotonic_ns, bool)
            or not isinstance(event.receive_monotonic_ns, int)
            or event.receive_monotonic_ns < 0
        ):
            raise EpochPairingError("receive_monotonic_ns must be a non-negative integer")
        if not isinstance(event.process_session_id, str) or not event.process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")

        now = event.receive_monotonic_ns
        self._latest_receive_monotonic_ns = now
        self._expire(now)

        context = (event.process_session_id, event.connection_generation)
        if kind == "GenerationStarted":
            self.reset()
            self._transport_context = context
            self._latest_receive_monotonic_ns = now
            return None
        if kind in ("DeviceLost", "TransportFault", "Stopped"):
            if self._transport_context is not None and context != self._transport_context:
                self._rejected_stale_generation += 1
                return None
            self.reset()
            return None

        if self._transport_context != context:
            self._rejected_stale_generation += 1
            return None

        if kind != "Frame":
            return None
        if event.frame_connection_generation != event.connection_generation:
            raise EpochPairingError("frame generation must match event generation")
        raw = decode_ubx_nav_ingress_frame(
            event.frame_bytes,
            protocol=event.protocol,
            receive_monotonic_ns=event.receive_monotonic_ns,
            process_session_id=event.process_session_id,
            connection_generation=event.connection_generation,
            ingress_sequence=event.ingress_sequence,
        )
        if raw is None:
            return None
        self._decoded_nav_frames += 1
        return self._accept_raw(raw)

    def handle_mapping(self, mapping: UbxSourceTimeMapping) -> Optional[UbxRosEpochResult]:
        if not isinstance(mapping, UbxSourceTimeMapping):
            raise EpochPairingError("mapping must be a UbxSourceTimeMapping")
        if not mapping.valid:
            return None
        if (
            mapping.gps_week is None
            or mapping.process_session_id is None
            or mapping.connection_generation is None
        ):
            raise EpochPairingError("valid receiver-time mapping lacks GPS week/session/generation")
        context = (mapping.process_session_id, mapping.connection_generation)
        if self._transport_context != context:
            self._rejected_stale_generation += 1
            return None
        time_context = (
            mapping.process_session_id,
            mapping.connection_generation,
            mapping.gps_week,
        )
        if time_context != self._time_context:
            if self._time_context is None:
                # First trusted week mapping: keep pre-mapping raw records from
                # this already-fenced transport generation. Exact iTOW and
                # receive-skew checks below decide which records can join.
                keep_all = self._raw_pending
                self._raw_pending = OrderedDict()
            else:
                # A GPS-week transition makes all previously buffered raw
                # records ambiguous except the iTOW of this mapping. Retired
                # iTOWs are week-scoped, so clear them only when an earlier
                # trusted week context actually existed.
                keep = self._raw_pending.pop(mapping.i_tow_ms, None)
                keep_all = OrderedDict()
                if keep is not None:
                    keep_all[mapping.i_tow_ms] = keep
                self._retired.clear()
            self.adapter.begin_generation(
                mapping.source_clock_id,
                process_session_id=mapping.process_session_id,
                connection_generation=mapping.connection_generation,
                gps_week=mapping.gps_week,
            )
            self._time_context = time_context
            self._raw_pending.clear()
            self._raw_pending.update(keep_all)
            self._mapping_pending.clear()
        if mapping.i_tow_ms in self._retired:
            return None

        result = self.adapter.add_time_mapping(mapping)
        self._mapping_pending[mapping.i_tow_ms] = mapping
        self._mapping_pending.move_to_end(mapping.i_tow_ms)
        self._enforce_capacity(self._mapping_pending)

        pending = self._raw_pending.pop(mapping.i_tow_ms, None)
        if pending is not None:
            for kind in ("pvt", "position", "covariance", "status"):
                record = pending.records.get(kind)
                if record is not None:
                    candidate = self._feed_raw(record, mapping)
                    if candidate is not None:
                        result = candidate
        return self._finish_result(mapping.i_tow_ms, result)

    def _accept_raw(self, record: UbxNavRawIngressRecord):
        if self._time_context is not None:
            expected = self._time_context[:2]
            if (record.process_session_id, record.connection_generation) != expected:
                self._rejected_stale_generation += 1
                return None
        mapping = self._mapping_pending.get(record.i_tow_ms)
        if mapping is not None:
            if record.i_tow_ms in self._retired:
                return None
            if not self._within_skew(record.receive_monotonic_ns, mapping.receive_monotonic_ns):
                self._rejected_receive_skew += 1
                return None
            result = self._feed_raw(record, mapping)
            return self._finish_result(record.i_tow_ms, result)

        pending = self._raw_pending.get(record.i_tow_ms)
        if pending is None:
            pending = _PendingRawEpoch(record.receive_monotonic_ns)
            self._raw_pending[record.i_tow_ms] = pending
        existing = pending.records.get(record.kind)
        if existing is not None:
            if existing.payload == record.payload:
                self._duplicate_frames += 1
                return None
            self._conflicting_epochs += 1
            self._raw_pending.pop(record.i_tow_ms, None)
            self._retire(record.i_tow_ms)
            return None
        pending.records[record.kind] = record
        self._raw_pending.move_to_end(record.i_tow_ms)
        self._buffered_before_mapping += 1
        self._enforce_capacity(self._raw_pending)
        return None

    def _feed_raw(self, record, mapping):
        if not self._within_skew(record.receive_monotonic_ns, mapping.receive_monotonic_ns):
            self._rejected_receive_skew += 1
            return None
        if (
            record.process_session_id != mapping.process_session_id
            or record.connection_generation != mapping.connection_generation
        ):
            self._rejected_stale_generation += 1
            return None
        epoch = decode_ubx_nav_raw_record(record, gps_week=mapping.gps_week)
        add_method = {
            "pvt": self.adapter.add_pvt,
            "position": self.adapter.add_position,
            "covariance": self.adapter.add_covariance,
            "status": self.adapter.add_status,
        }[record.kind]
        return add_method(epoch)

    def _finish_result(self, i_tow_ms, result):
        if result is not None:
            self._mapping_pending.pop(i_tow_ms, None)
            self._raw_pending.pop(i_tow_ms, None)
            self._retire(i_tow_ms)
            self._emitted_observations += 1
        return result

    def _within_skew(self, first, second):
        return abs(first - second) <= self.max_pair_receive_skew_ns

    def _retire(self, i_tow_ms):
        self._retired[i_tow_ms] = None
        self._retired.move_to_end(i_tow_ms)
        while len(self._retired) > self.max_pending_epochs:
            self._retired.popitem(last=False)

    def _enforce_capacity(self, cache):
        while len(cache) > self.max_pending_epochs:
            evicted_i_tow, _ = cache.popitem(last=False)
            self._capacity_evictions += 1
            self._raw_pending.pop(evicted_i_tow, None)
            self._mapping_pending.pop(evicted_i_tow, None)
            self._retire(evicted_i_tow)

    def _expire(self, now_monotonic_ns):
        for i_tow, pending in list(self._raw_pending.items()):
            if now_monotonic_ns - pending.first_receive_monotonic_ns > self.max_pending_age_ns:
                self._raw_pending.pop(i_tow, None)
                self._retire(i_tow)
                self._expired_raw_epochs += 1
        for i_tow, mapping in list(self._mapping_pending.items()):
            if now_monotonic_ns - mapping.receive_monotonic_ns > self.max_pending_age_ns:
                self._mapping_pending.pop(i_tow, None)
                self._retire(i_tow)
        self.adapter.expire_stale(now_monotonic_ns)

    def expire_stale(self, now_monotonic_ns: int):
        if (
            isinstance(now_monotonic_ns, bool)
            or not isinstance(now_monotonic_ns, int)
            or now_monotonic_ns < 0
        ):
            raise EpochPairingError("now_monotonic_ns must be a non-negative integer")
        self._latest_receive_monotonic_ns = now_monotonic_ns
        self._expire(now_monotonic_ns)

    def stats(self):
        return UbxRawNavAssemblerStats(
            decoded_nav_frames=self._decoded_nav_frames,
            buffered_before_mapping=self._buffered_before_mapping,
            rejected_receive_skew=self._rejected_receive_skew,
            rejected_stale_generation=self._rejected_stale_generation,
            duplicate_frames=self._duplicate_frames,
            conflicting_epochs=self._conflicting_epochs,
            expired_raw_epochs=self._expired_raw_epochs,
            capacity_evictions=self._capacity_evictions,
            emitted_observations=self._emitted_observations,
        )
