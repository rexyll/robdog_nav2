"""Ordered lifecycle consumer for raw GNSS ingress events.

This transport-independent processor mirrors GnssIngressEvent from the C++
USB stream. It serializes lifecycle and frame events before forwarding only
NAV-TIMEGPS/NAV-TIMELS records to the canonical receiver-time mapper.
"""

from dataclasses import dataclass
from typing import Optional, Union

from .ubx_nav_pvt import EpochPairingError
from .ubx_ingress_adapter import (
    PROTOCOL_UBX,
    UbxTimeIngressRecord,
    decode_ubx_time_ingress_frame,
)
from .ubx_time_mapper import UbxReceiverTimeMapper
from .ubx_time_messages import UbxNavTimeGpsEpoch, UbxNavTimeLsEpoch
from .ros_epoch_adapter import UbxSourceTimeMapping


@dataclass(frozen=True)
class GnssIngressEvent:
    """Python representation of one C++ GnssIngressEvent."""

    event_type: str
    process_session_id: str
    connection_generation: int
    ingress_sequence: int
    receive_monotonic_ns: int
    protocol: Optional[Union[str, int]] = None
    frame_bytes: Optional[bytes] = None


@dataclass(frozen=True)
class GnssIngressProcessorStats:
    processed_events: int
    decoded_timegps: int
    decoded_timels: int
    ignored_frames: int
    emitted_mappings: int
    rejected_events: int


class GnssIngressEventProcessor:
    """Consume one ordered event stream and fence mapper state by lifecycle."""

    _EVENT_TYPES = {
        "GenerationStarted", "Frame", "DeviceLost", "TransportFault", "Stopped"
    }

    def __init__(self, mapper: UbxReceiverTimeMapper) -> None:
        if not isinstance(mapper, UbxReceiverTimeMapper):
            raise ValueError("mapper must be an UbxReceiverTimeMapper")
        if not mapper.require_ingress_sequence:
            raise ValueError("mapper must require ingress_sequence for raw ingress")
        self.mapper = mapper
        self._process_session_id: Optional[str] = None
        self._connection_generation: Optional[int] = None
        self._last_ingress_sequence: Optional[int] = None
        self._last_receive_monotonic_ns: Optional[int] = None
        self._active = False
        self._terminal_type: Optional[str] = None
        self._stopped = False
        self._processed_events = 0
        self._decoded_timegps = 0
        self._decoded_timels = 0
        self._ignored_frames = 0
        self._emitted_mappings = 0
        self._rejected_events = 0

    @staticmethod
    def _validate_event(event: GnssIngressEvent) -> None:
        if not isinstance(event, GnssIngressEvent):
            raise EpochPairingError("event must be a GnssIngressEvent")
        if event.event_type not in GnssIngressEventProcessor._EVENT_TYPES:
            raise EpochPairingError("unknown GNSS ingress event_type")
        if not isinstance(event.process_session_id, str) or not event.process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")
        for name, value in (
            ("connection_generation", event.connection_generation),
            ("ingress_sequence", event.ingress_sequence),
            ("receive_monotonic_ns", event.receive_monotonic_ns),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise EpochPairingError(f"{name} must be a non-negative integer")
        if event.event_type == "Frame":
            if event.protocol is None:
                raise EpochPairingError("Frame event requires protocol")
            if not isinstance(event.frame_bytes, (bytes, bytearray, memoryview)):
                raise EpochPairingError("Frame event requires bytes-like frame_bytes")
        elif event.protocol is not None or event.frame_bytes is not None:
            raise EpochPairingError("lifecycle events must not carry frame data")

    def process(self, event: GnssIngressEvent) -> Optional[UbxSourceTimeMapping]:
        """Consume one event. Returns a mapping only when TIMEGPS/TIMELS pair."""
        try:
            self._validate_event(event)
            if (
                self._process_session_id is not None
                and event.process_session_id != self._process_session_id
            ):
                if event.event_type != "GenerationStarted":
                    raise EpochPairingError(
                        "new process_session_id must begin with GenerationStarted"
                    )
                self.mapper.reset()
                self._connection_generation = None
                self._last_ingress_sequence = None
                self._last_receive_monotonic_ns = None
                self._active = False
                self._terminal_type = None
                self._stopped = False

            if (
                self._last_ingress_sequence is not None
                and event.ingress_sequence <= self._last_ingress_sequence
            ):
                raise EpochPairingError(
                    "stale or reordered ingress event sequence"
                )
            if (
                self._last_receive_monotonic_ns is not None
                and event.receive_monotonic_ns < self._last_receive_monotonic_ns
            ):
                raise EpochPairingError(
                    "ingress receive_monotonic_ns moved backwards"
                )

            if event.event_type == "GenerationStarted":
                if self._process_session_id == event.process_session_id:
                    if (
                        self._connection_generation is not None
                        and event.connection_generation <= self._connection_generation
                    ):
                        raise EpochPairingError(
                            "connection_generation must increase on restart"
                        )
                    if self._active:
                        raise EpochPairingError(
                            "cannot start a new generation before prior generation terminates"
                        )
                self.mapper.begin_session(
                    event.process_session_id, event.connection_generation
                )
                self._process_session_id = event.process_session_id
                self._connection_generation = event.connection_generation
                self._active = True
                self._terminal_type = None
                self._stopped = False
            else:
                if (
                    self._process_session_id != event.process_session_id
                    or self._connection_generation != event.connection_generation
                ):
                    raise EpochPairingError(
                        "event process_session_id/connection_generation is stale"
                    )

            # Sequence is consumed even if a well-formed Frame is not a time
            # message or its payload is rejected. The next event must still be
            # strictly newer in the source event stream.
            self._last_ingress_sequence = event.ingress_sequence
            self._last_receive_monotonic_ns = event.receive_monotonic_ns
            self._processed_events += 1

            if event.event_type == "Frame":
                if not self._active or self._terminal_type is not None or self._stopped:
                    raise EpochPairingError("Frame event arrived outside active generation")
                record = decode_ubx_time_ingress_frame(
                    event.frame_bytes,
                    protocol=event.protocol,
                    receive_monotonic_ns=event.receive_monotonic_ns,
                    process_session_id=event.process_session_id,
                    connection_generation=event.connection_generation,
                    ingress_sequence=event.ingress_sequence,
                )
                if record is None:
                    self._ignored_frames += 1
                    return None
                mapping = self._add_time_record(record)
                if mapping is not None:
                    self._emitted_mappings += 1
                return mapping

            if event.event_type in ("DeviceLost", "TransportFault"):
                if not self._active or self._terminal_type is not None:
                    raise EpochPairingError("duplicate or out-of-order terminal event")
                self._terminal_type = event.event_type
                self._active = False
                self.mapper.reset()
                return None

            if event.event_type == "Stopped":
                if self._stopped:
                    raise EpochPairingError("duplicate Stopped event")
                if self._active:
                    self.mapper.reset()
                    self._active = False
                self._stopped = True
                return None

            return None
        except EpochPairingError:
            self._rejected_events += 1
            raise

    def _add_time_record(
        self, record: UbxTimeIngressRecord
    ) -> Optional[UbxSourceTimeMapping]:
        if isinstance(record, UbxNavTimeGpsEpoch):
            self._decoded_timegps += 1
            return self.mapper.add_timegps(record)
        if isinstance(record, UbxNavTimeLsEpoch):
            self._decoded_timels += 1
            return self.mapper.add_timels(record)
        raise EpochPairingError("unsupported decoded GNSS time record")

    def stats(self) -> GnssIngressProcessorStats:
        return GnssIngressProcessorStats(
            processed_events=self._processed_events,
            decoded_timegps=self._decoded_timegps,
            decoded_timels=self._decoded_timels,
            ignored_frames=self._ignored_frames,
            emitted_mappings=self._emitted_mappings,
            rejected_events=self._rejected_events,
        )
