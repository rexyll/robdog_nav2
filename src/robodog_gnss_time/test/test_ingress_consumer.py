from types import SimpleNamespace

import pytest

from robodog_gnss_quality.ubx_ingress_event_processor import GnssIngressEventProcessor
from robodog_gnss_quality.ubx_time_mapper import UbxReceiverTimeMapper
from robodog_gnss_time.ingress_consumer import GnssIngressConsumer


def processor():
    return GnssIngressEventProcessor(UbxReceiverTimeMapper(require_ingress_sequence=True))


def lifecycle_message(event_type, sequence):
    return SimpleNamespace(
        event_type=event_type,
        process_session_id="process-A",
        connection_generation=1,
        ingress_sequence=sequence,
        receive_monotonic_ns=1000 + sequence,
        dropped_frame_events_total=0,
        protocol=255,
        checksum_status=0,
        frame_sequence=0,
        frame_connection_generation=0,
        frame_bytes=[],
    )


def test_consumer_accepts_generation_and_stopped_events():
    consumer = GnssIngressConsumer(processor(), lambda mapping: None)
    assert consumer.handle_ros_message(lifecycle_message(0, 0)) is None
    assert consumer.handle_ros_message(lifecycle_message(4, 1)) is None
    assert consumer.faulted is False


def test_consumer_latches_invalid_event_and_rejects_every_later_event():
    consumer = GnssIngressConsumer(processor(), lambda mapping: None)
    with pytest.raises(Exception):
        consumer.handle_ros_message(lifecycle_message(4, 0))
    assert consumer.faulted is True
    with pytest.raises(RuntimeError, match="latched fail-closed"):
        consumer.handle_ros_message(lifecycle_message(0, 0))


def test_consumer_latches_mapping_publisher_failure():
    def fail_publish(_mapping):
        raise RuntimeError("publisher failure")

    consumer = GnssIngressConsumer(processor(), fail_publish)
    assert consumer.handle_ros_message(lifecycle_message(0, 0)) is None
    assert consumer.faulted is False
