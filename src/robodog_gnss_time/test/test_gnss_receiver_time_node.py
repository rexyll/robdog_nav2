import struct
import time

import rclpy
import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from robodog_gnss_interfaces.msg import GnssIngressEvent, GnssObservation, GnssReceiverTime

from robodog_gnss_time.gnss_receiver_time_node import GnssReceiverTimeNode


def ubx_frame(message_class, message_id, payload):
    body = bytes([message_class, message_id]) + len(payload).to_bytes(2, "little") + payload
    a = b = 0
    for value in body:
        a = (a + value) & 0xFF
        b = (b + a) & 0xFF
    return b"\xB5\x62" + body + bytes([a, b])



def nav_pvt_frame():
    payload = bytearray(92)
    payload[0:4] = (123456).to_bytes(4, "little")
    payload[20] = 3
    payload[21] = 0x81
    return ubx_frame(0x01, 0x07, bytes(payload))


def nav_hpposllh_frame():
    payload = bytearray(36)
    payload[0] = 0
    payload[3] = 0
    payload[4:8] = (123456).to_bytes(4, "little")
    struct.pack_into("<iiii", payload, 8, -1220000000, 370000000, 15000, 10000)
    struct.pack_into("<bbbb", payload, 24, 1, -1, 2, 0)
    struct.pack_into("<II", payload, 28, 100, 120)
    return ubx_frame(0x01, 0x14, bytes(payload))


def nav_cov_frame():
    payload = bytearray(64)
    payload[0:4] = (123456).to_bytes(4, "little")
    payload[4] = 0
    payload[5] = 1
    payload[6] = 1
    struct.pack_into("<ffffff", payload, 16, 1.0, 0.0, 0.0, 1.5, 0.0, 2.0)
    return ubx_frame(0x01, 0x36, bytes(payload))


def nav_status_frame():
    payload = bytearray(16)
    payload[0:4] = (123456).to_bytes(4, "little")
    payload[4] = 3
    payload[5] = 0x0F
    payload[6] = 0x03
    payload[7] = 0x80
    return ubx_frame(0x01, 0x03, bytes(payload))


def timegps_frame():
    return ubx_frame(0x01, 0x20, struct.pack("<IihbBI", 123456, 0, 2450, 18, 0x07, 50))


def timels_frame():
    payload = bytearray(24)
    payload[0:4] = (123456).to_bytes(4, "little")
    payload[8] = 2
    payload[9] = 18
    payload[10] = 2
    payload[11] = 1
    payload[12:16] = (86400).to_bytes(4, "little", signed=True)
    payload[16:18] = (2451).to_bytes(2, "little")
    payload[18:20] = (7).to_bytes(2, "little")
    payload[23] = 0x01
    return ubx_frame(0x01, 0x26, bytes(payload))


def make_event(kind, sequence, frame=None):
    message = GnssIngressEvent()
    message.event_type = kind
    message.process_session_id = "ros-node-session"
    message.connection_generation = 1
    message.ingress_sequence = sequence
    message.receive_monotonic_ns = 10_000_000 + sequence * 1_000
    message.dropped_frame_events_total = 0
    message.protocol = 255
    message.checksum_status = 0
    message.frame_sequence = 0
    message.frame_connection_generation = 0
    message.frame_bytes = []
    if frame is not None:
        message.protocol = GnssIngressEvent.PROTOCOL_UBX
        message.checksum_status = GnssIngressEvent.CHECKSUM_VALID
        message.frame_sequence = sequence
        message.frame_connection_generation = 1
        message.frame_bytes = list(frame)
    return message


def test_ros_node_publishes_generation_fenced_receiver_time_mapping():
    if not rclpy.ok():
        rclpy.init()
    suffix = str(time.monotonic_ns())
    ingress_topic = f"/test/gnss_time_ingress_{suffix}"
    mapping_topic = f"/test/gnss_time_mapping_{suffix}"
    observation_topic = f"/test/gnss_time_observation_{suffix}"
    node = GnssReceiverTimeNode(parameter_overrides=[
        Parameter("ingress_topic", value=ingress_topic),
        Parameter("mapping_topic", value=mapping_topic),
        Parameter("observation_topic", value=observation_topic),
    ])
    harness = rclpy.create_node("gnss_time_node_test_harness")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    ingress_publisher = harness.create_publisher(GnssIngressEvent, ingress_topic, qos)
    received = []
    observations = []
    mapping_subscription = harness.create_subscription(
        GnssReceiverTime, mapping_topic, received.append, qos
    )
    observation_subscription = harness.create_subscription(
        GnssObservation, observation_topic, observations.append, qos
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    try:
        deadline = time.monotonic() + 5
        while ingress_publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert ingress_publisher.get_subscription_count() > 0
        mapping_deadline = time.monotonic() + 5
        while node._publisher.get_subscription_count() == 0 and time.monotonic() < mapping_deadline:
            executor.spin_once(timeout_sec=0.01)
        assert node._publisher.get_subscription_count() > 0
        observation_deadline = time.monotonic() + 5
        while node._observation_publisher.get_subscription_count() == 0 and time.monotonic() < observation_deadline:
            executor.spin_once(timeout_sec=0.01)
        assert node._observation_publisher.get_subscription_count() > 0

        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_GENERATION_STARTED, 0))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 1, nav_pvt_frame()))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 2, nav_hpposllh_frame()))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 3, nav_cov_frame()))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 4, nav_status_frame()))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 5, timegps_frame()))
        ingress_publisher.publish(make_event(GnssIngressEvent.EVENT_FRAME, 6, timels_frame()))

        deadline = time.monotonic() + 5
        while (not received or not observations) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(received) == 1
        assert len(observations) == 1
        mapping = received[0]
        assert mapping.valid is True
        assert mapping.utc_stamp_valid is False  # default leap-source allowlist is fail-closed
        assert mapping.process_session_id == "ros-node-session"
        assert mapping.connection_generation == 1
        assert mapping.gps_week_valid is True
        assert mapping.gps_week == 2450
        assert mapping.source_timestamp_ns > 0

        observation = observations[0]
        assert observation.i_tow_ms == 123456
        assert observation.gps_week == 2450
        assert observation.process_session_id == "ros-node-session"
        assert observation.connection_generation == 1
        assert observation.source_timestamp_ns == mapping.source_timestamp_ns
        assert observation.source_clock_id == mapping.source_clock_id
        assert observation.gnss_fix_ok is True
        assert observation.carrier_solution == 2
        assert observation.covariance_type == 3
        assert observation.latitude_deg == pytest.approx(37.0, abs=2e-8)
        assert observation.longitude_deg == pytest.approx(-122.0, abs=2e-8)
        assert observation.utc_stamp_valid is False
    finally:
        executor.remove_node(node)
        executor.remove_node(harness)
        node.destroy_node()
        harness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
