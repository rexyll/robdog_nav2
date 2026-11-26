"""Ordered lifecycle consumer for raw GNSS ingress events.

This transport-independent processor mirrors the C++ GnssIngressEvent and
the ROS robodog_gnss_interfaces/GnssIngressEvent wire contract.
"""

from dataclasses import dataclass
from typing import Optional, Union

from .ubx_nav_pvt import EpochPairingError
from .ubx_ingress_adapter import (
    UbxTimeIngressRecord,
    decode_ubx_time_ingress_frame,
)
from .ubx_time_mapper import UbxReceiverTimeMapper
from .ubx_time_messages import UbxNavTimeGpsEpoch, UbxNavTimeLsEpoch
from .ros_epoch_adapter import UbxSourceTimeMapping


_EVENT_ID_TO_NAME = {
    0: "GenerationStarted",
    1: "Frame",
    2: "DeviceLost",
    3: "TransportFault",
    4: "Stopped",
}
_EVENT_NAME_TO_ID = {value: key for key, value in _EVENT_ID_TO_NAME.items()}


@dataclass(frozen=True)
class GnssIngressEvent:
    """Python representation of one C++/ROS ordered ingress event."""

    event_type: Union[str, int]
    process_session_id: str
    connection_generation: int
    ingress_sequence: int
    receive_monotonic_ns: int
    dropped_frame_events_total: int = 0
    protocol: Optional[Union[str, int]] = None
    checksum_status: Optional[int] = None
    frame_sequence: Optional[int] = None
    frame_connection_generation: Optional[int] = None
    frame_bytes: Optional[bytes] = None

    @staticmethod
    def _ros_uint(name: str, value: object, maximum: int = (1 << 64) - 1) -> int:
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 0 <= value <= maximum
        ):
            raise EpochPairingError(f"ROS ingress {name} must be an unsigned integer")
        return value

    @classmethod
    def from_ros_message(cls, message: object) -> "GnssIngressEvent":
        """Convert generated ROSIDL message without importing its package."""
        event_type = cls._ros_uint("event_type", getattr(message, "event_type", None), 0xFF)
        if event_type not in _EVENT_ID_TO_NAME:
            raise EpochPairingError("ROS ingress message has unknown event_type")
        kind = _EVENT_ID_TO_NAME[event_type]
        process_session_id = getattr(message, "process_session_id", None)
        if not isinstance(process_session_id, str) or not process_session_id.strip():
            raise EpochPairingError("ROS ingress process_session_id must be a non-empty string")
        connection_generation = cls._ros_uint(
            "connection_generation", getattr(message, "connection_generation", None)
        )
        ingress_sequence = cls._ros_uint(
            "ingress_sequence", getattr(message, "ingress_sequence", None)
        )
        receive_monotonic_ns = cls._ros_uint(
            "receive_monotonic_ns", getattr(message, "receive_monotonic_ns", None)
        )
        dropped_frame_events_total = cls._ros_uint(
            "dropped_frame_events_total",
            getattr(message, "dropped_frame_events_total", None),
        )
        protocol = cls._ros_uint("protocol", getattr(message, "protocol", None), 0xFF)
        checksum_status = cls._ros_uint(
            "checksum_status", getattr(message, "checksum_status", None), 0xFF
        )
        frame_sequence = cls._ros_uint(
            "frame_sequence", getattr(message, "frame_sequence", None)
        )
        frame_generation = cls._ros_uint(
            "frame_connection_generation",
            getattr(message, "frame_connection_generation", None),
        )
        try:
            frame_bytes = bytes(getattr(message, "frame_bytes", b""))
        except (TypeError, ValueError, OverflowError) as exc:
            raise EpochPairingError("ROS ingress frame_bytes must be a uint8 sequence") from exc
        if kind == "Frame":
            return cls(
                event_type=kind,
                process_session_id=process_session_id,
                connection_generation=connection_generation,
                ingress_sequence=ingress_sequence,
                receive_monotonic_ns=receive_monotonic_ns,
                dropped_frame_events_total=dropped_frame_events_total,
                protocol=protocol,
                checksum_status=checksum_status,
                frame_sequence=frame_sequence,
                frame_connection_generation=frame_generation,
                frame_bytes=frame_bytes,
            )
        if (
            protocol != 255 or checksum_status != 0 or frame_sequence != 0
            or frame_generation != 0 or frame_bytes
        ):
            raise EpochPairingError(
                "ROS lifecycle event must use empty-frame sentinel fields"
            )
        return cls(
            event_type=kind,
            process_session_id=process_session_id,
            connection_generation=connection_generation,
            ingress_sequence=ingress_sequence,
            receive_monotonic_ns=receive_monotonic_ns,
            dropped_frame_events_total=dropped_frame_events_total,
        )


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

    _EVENT_TYPES = set(_EVENT_NAME_TO_ID)

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
        self._last_dropped_frame_events_total = 0
        self._pending_dropped_frame_events = 0
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
    def _event_kind(value: Union[str, int]) -> str:
        if isinstance(value, bool):
            raise EpochPairingError("event_type must be a known string or uint8 enum")
        if isinstance(value, int):
            kind = _EVENT_ID_TO_NAME.get(value)
        elif isinstance(value, str):
            kind = value if value in _EVENT_NAME_TO_ID else None
        else:
            kind = None
        if kind is None:
            raise EpochPairingError("unknown GNSS ingress event_type")
        return kind

    @classmethod
    def _validate_event(cls, event: GnssIngressEvent) -> str:
        if not isinstance(event, GnssIngressEvent):
            raise EpochPairingError("event must be a GnssIngressEvent")
        kind = cls._event_kind(event.event_type)
        if not isinstance(event.process_session_id, str) or not event.process_session_id.strip():
            raise EpochPairingError("process_session_id must be a non-empty string")
        for name, value in (
            ("connection_generation", event.connection_generation),
            ("ingress_sequence", event.ingress_sequence),
            ("receive_monotonic_ns", event.receive_monotonic_ns),
            ("dropped_frame_events_total", event.dropped_frame_events_total),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise EpochPairingError(f"{name} must be a non-negative integer")
        if kind == "Frame":
            if (
                isinstance(event.protocol, bool)
                or not (
                    (isinstance(event.protocol, int) and event.protocol in (0, 1, 2))
                    or (isinstance(event.protocol, str) and event.protocol in ("ubx", "rtcm3", "nmea"))
                )
            ):
                raise EpochPairingError("Frame event requires a supported protocol")
            if not isinstance(event.frame_bytes, (bytes, bytearray, memoryview)):
                raise EpochPairingError("Frame event requires bytes-like frame_bytes")
            if (
                isinstance(event.checksum_status, bool)
                or not isinstance(event.checksum_status, int)
                or event.checksum_status not in (1, 2)
            ):
                raise EpochPairingError("Frame event requires an integer valid/absent checksum status")
            if event.protocol in (0, 1, "ubx", "rtcm3") and event.checksum_status != 1:
                raise EpochPairingError("UBX/RTCM3 frames must have a valid checksum")
            if isinstance(event.frame_sequence, bool) or not isinstance(event.frame_sequence, int) or event.frame_sequence < 0:
                raise EpochPairingError("Frame event requires a non-negative frame_sequence")
            if (
                isinstance(event.frame_connection_generation, bool)
                or not isinstance(event.frame_connection_generation, int)
                or event.frame_connection_generation != event.connection_generation
            ):
                raise EpochPairingError("frame generation must match event generation")
        elif any(value is not None for value in (
            event.protocol, event.checksum_status, event.frame_sequence,
            event.frame_connection_generation, event.frame_bytes
        )):
            raise EpochPairingError("lifecycle events must not carry frame data")
        return kind

    def process(self, event: GnssIngressEvent) -> Optional[UbxSourceTimeMapping]:
        """Consume one event. Returns a mapping only when TIMEGPS/TIMELS pair."""
        try:
            kind = self._validate_event(event)
            if (
                self._process_session_id is not None
                and event.process_session_id != self._process_session_id
            ):
                if kind != "GenerationStarted":
                    raise EpochPairingError(
                        "new process_session_id must begin with GenerationStarted"
                    )
                self.mapper.reset()
                self._connection_generation = None
                self._last_ingress_sequence = None
                self._last_receive_monotonic_ns = None
                self._last_dropped_frame_events_total = 0
                self._pending_dropped_frame_events = 0
                self._active = False
                self._terminal_type = None
                self._stopped = False

            if (
                self._last_ingress_sequence is not None
                and event.ingress_sequence <= self._last_ingress_sequence
            ):
                raise EpochPairingError("stale or reordered ingress event sequence")
            if (
                self._last_receive_monotonic_ns is not None
                and event.receive_monotonic_ns < self._last_receive_monotonic_ns
            ):
                raise EpochPairingError("ingress receive_monotonic_ns moved backwards")
            if event.dropped_frame_events_total < self._last_dropped_frame_events_total:
                raise EpochPairingError("dropped_frame_events_total moved backwards")

            if event.dropped_frame_events_total > event.ingress_sequence:
                raise EpochPairingError(
                    "dropped-frame watermark exceeds the event sequence"
                )
            if self._last_ingress_sequence is None:
                if kind != "GenerationStarted" or event.ingress_sequence != 0:
                    raise EpochPairingError(
                        "first ingress event must be GenerationStarted with sequence 0"
                    )
                if event.dropped_frame_events_total != 0:
                    raise EpochPairingError(
                        "first GenerationStarted event cannot report prior frame drops"
                    )
                missing_sequences = 0
            else:
                missing_sequences = event.ingress_sequence - self._last_ingress_sequence - 1
            drop_delta = event.dropped_frame_events_total - self._last_dropped_frame_events_total
            pending_drops = self._pending_dropped_frame_events + drop_delta
            if missing_sequences > pending_drops:
                raise EpochPairingError(
                    "ingress sequence gap is not explained by dropped frame events"
                )
            pending_drops -= missing_sequences
            if kind in ("DeviceLost", "TransportFault", "Stopped") and pending_drops:
                raise EpochPairingError(
                    "terminal ingress event leaves unexplained dropped-frame watermark"
                )

            if kind == "GenerationStarted":
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
            elif self._process_session_id != event.process_session_id:
                raise EpochPairingError(
                    "event process_session_id is stale"
                )

            # Consume the ordered source event even when its connection
            # generation is stale. Otherwise one rejected old-generation
            # callback would make every subsequent current-generation event
            # look like an unexplained sequence gap.
            self._last_ingress_sequence = event.ingress_sequence
            self._last_receive_monotonic_ns = event.receive_monotonic_ns
            self._last_dropped_frame_events_total = event.dropped_frame_events_total
            self._pending_dropped_frame_events = pending_drops
            self._processed_events += 1

            if (
                kind != "GenerationStarted"
                and self._connection_generation != event.connection_generation
            ):
                raise EpochPairingError(
                    "event process_session_id/connection_generation is stale"
                )

            if kind == "Frame":
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

            if kind in ("DeviceLost", "TransportFault"):
                if not self._active or self._terminal_type is not None:
                    raise EpochPairingError("duplicate or out-of-order terminal event")
                self._terminal_type = kind
                self._active = False
                self.mapper.reset()
                return None

            if kind == "Stopped":
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
