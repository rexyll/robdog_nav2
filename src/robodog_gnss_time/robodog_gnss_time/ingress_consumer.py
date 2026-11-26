"""Sticky fail-closed wrapper around ingress, receiver-time and raw NAV consumers."""

from robodog_gnss_quality.ubx_ingress_event_processor import (
    GnssIngressEvent,
    GnssIngressEventProcessor,
)


class GnssIngressConsumer:
    def __init__(
        self,
        processor,
        publish_mapping,
        navigation_assembler=None,
        publish_observation=None,
    ):
        if not isinstance(processor, GnssIngressEventProcessor):
            raise ValueError("processor must be a GnssIngressEventProcessor")
        if not callable(publish_mapping):
            raise ValueError("publish_mapping must be callable")
        if (navigation_assembler is None) != (publish_observation is None):
            raise ValueError(
                "navigation_assembler and publish_observation must be supplied together"
            )
        if publish_observation is not None and not callable(publish_observation):
            raise ValueError("publish_observation must be callable")
        self._processor = processor
        self._publish_mapping = publish_mapping
        self._navigation_assembler = navigation_assembler
        self._publish_observation = publish_observation
        self._fault = None

    @property
    def faulted(self):
        return self._fault is not None

    @property
    def fault(self):
        return self._fault

    def handle_ros_message(self, message):
        if self._fault is not None:
            raise RuntimeError(
                "GNSS ingress consumer is faulted; restart with a new process session"
            ) from self._fault
        try:
            event = GnssIngressEvent.from_ros_message(message)
            mapping = self._processor.process(event)
            result = None
            if self._navigation_assembler is not None:
                result = self._navigation_assembler.process_event(event)
            if mapping is not None:
                self._publish_mapping(mapping)
                if self._navigation_assembler is not None:
                    result = self._navigation_assembler.handle_mapping(mapping) or result
            if result is not None:
                self._publish_observation(result)
            return mapping
        except Exception as exc:
            self._fault = exc
            raise
