import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from robodog_gnss_interfaces.msg import GnssObservation, GnssQualityStatus

from robodog_gnss_time.gnss_rtk_quality_node import GnssRtkQualityNode


def make_observation(receive_ns, source_ns, generation=2, week=2450):
    message = GnssObservation()
    message.i_tow_ms = 123456 + source_ns // 1_000_000_000
    message.gps_week = week
    message.process_session_id = "quality-test-session"
    message.connection_generation = generation
    message.source_timestamp_ns = source_ns
    message.source_clock_id = (
        f"{message.process_session_id}:connection:{generation}:gps-week:{week}"
    )
    message.receive_monotonic_ns = receive_ns
    message.latitude_deg = 37.0
    message.longitude_deg = -122.0
    message.altitude_m = 10.0
    message.navsat_status = 0
    message.gnss_fix_ok = True
    message.carrier_solution = 2
    message.position_covariance = [
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.04,
    ]
    message.covariance_type = 3
    return message


def setup_nodes(max_age_ns=500_000_000):
    if not rclpy.ok():
        rclpy.init()
    suffix = str(time.monotonic_ns())
    observation_topic = f"/test/gnss_quality_input_{suffix}"
    status_topic = f"/test/gnss_quality_output_{suffix}"
    node = GnssRtkQualityNode(parameter_overrides=[
        Parameter("observation_topic", value=observation_topic),
        Parameter("status_topic", value=status_topic),
        Parameter("max_fix_age_ns", value=max_age_ns),
        Parameter("required_consecutive_fixed_samples", value=3),
        Parameter("max_horizontal_std_m", value=0.25),
        Parameter("max_vertical_std_m", value=0.5),
        Parameter("max_plausible_speed_mps", value=5.0),
    ])
    harness = rclpy.create_node("gnss_quality_test_harness")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = harness.create_publisher(GnssObservation, observation_topic, qos)
    received = []
    subscription = harness.create_subscription(
        GnssQualityStatus, status_topic, received.append, qos
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    deadline = time.monotonic() + 5
    while (
        publisher.get_subscription_count() == 0
        or node._publisher.get_subscription_count() == 0
    ) and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert publisher.get_subscription_count() > 0
    assert node._publisher.get_subscription_count() > 0
    return node, harness, publisher, received, executor


def cleanup_nodes(node, harness, executor):
    executor.remove_node(node)
    executor.remove_node(harness)
    node.destroy_node()
    harness.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate()


def test_quality_node_publishes_recovery_acceptance_and_stale_watchdog():
    node, harness, publisher, received, executor = setup_nodes(max_age_ns=100_000_000)
    try:
        for index in range(3):
            publisher.publish(make_observation(
                time.monotonic_ns(),
                1_000_000_000 * (index + 1),
            ))
            executor.spin_once(timeout_sec=0.02)
        spin_until(executor, lambda: any(item.qualified for item in received))
        accepted = [item for item in received if item.qualified][-1]
        assert accepted.reason == "accepted"
        assert accepted.consecutive_fixed_samples == 3
        assert accepted.source_clock_id.endswith("gps-week:2450")

        spin_until(
            executor,
            lambda: any(item.reason == "fix_stale" for item in received),
            timeout=2.0,
        )
        stale = [item for item in received if item.reason == "fix_stale"][-1]
        assert stale.qualified is False
    finally:
        cleanup_nodes(node, harness, executor)


def test_quality_node_rejects_old_generation_without_resetting_active_gate():
    node, harness, publisher, received, executor = setup_nodes()
    try:
        publisher.publish(make_observation(time.monotonic_ns(), 1_000_000_000, generation=2))
        spin_until(executor, lambda: any(item.reason == "recovering" for item in received))
        publisher.publish(make_observation(time.monotonic_ns(), 2_000_000_000, generation=1))
        spin_until(executor, lambda: any(item.reason == "out_of_order" for item in received))
        publisher.publish(make_observation(time.monotonic_ns(), 3_000_000_000, generation=2))
        spin_until(
            executor,
            lambda: any(
                item.reason == "recovering" and item.consecutive_fixed_samples == 2
                for item in received
            ),
        )
        assert node._gate.consecutive_fixed_samples == 2
        assert node.faulted is False
    finally:
        teardown(node, harness, executor)
