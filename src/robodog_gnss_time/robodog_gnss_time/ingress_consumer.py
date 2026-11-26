"""Sticky fail-closed wrapper around the ordered transport-independent processor."""

from robodog_gnss_quality.ubx_ingress_event_processor import GnssIngressEvent, GnssIngressEventProcessor


class GnssIngressConsumer:
    def __init__(self, processor, publish_mapping):
        if not isinstance(processor, GnssIngressEventProcessor):
            raise ValueError("processor must be a GnssIngressEventProcessor")
        if not callable(publish_mapping):
            raise ValueError("publish_mapping must be callable")
        self._processor = processor
        self._publish_mapping = publish_mapping
        self._fault: Exception | None = None

    @property
    def faulted(self):
        return self._fault is not None

    @property
    def fault(self):
        return self._fault

    def handle_ros_message(self, message):
        if self._fault is not None:
            raise RuntimeError("GNSS ingress consumer is faulted; restart with a new process session") from self._fault
        try:
            event = GnssIngressEvent.from_ros_message(message)
            mapping = self._processor.process(event)
            if mapping is not None:
                self._publish_mapping(mapping)
            return mapping
        except Exception as exc:
            self._fault = exc
            raise
