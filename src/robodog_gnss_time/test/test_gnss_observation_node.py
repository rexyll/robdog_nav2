import time

import rclpy
import pytest
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from robodog_gnss_interfaces.msg import (
    GnssNavCov,
    GnssNavHpPosLlh,
    GnssNavPvt,
    GnssNavStatus,
    GnssObservation,
    GnssReceiverTime,
)

from robodog_gnss_time.gnss_observation_node import GnssObservationNode


def test_ros_node_joins_four_ubx_topics_with_receiver_time():
    if not rclpy.ok():
        rclpy.init()
    suffix = str(time.monotonic_ns())
    topics = {
        "nav_pvt_topic": f"/test/gnss_pvt_{suffix}",
        "nav_hp_pos_llh_topic": f"/test/gnss_hppos_{suffix}",
        "nav_cov_topic": f"/test/gnss_cov_{suffix}",
        "nav_status_topic": f"/test/gnss_status_{suffix}",
        "receiver_time_topic": f"/test/gnss_time_{suffix}",
        "observation_topic": f"/test/gnss_observation_{suffix}",
    }
    overrides = [Parameter(name, value=value) for name, value in topics.items()]
    overrides.extend([
        Parameter("max_pair_receive_skew_ns", value=2_000_000_000),
        Parameter("max_pending_age_ns", value=5_000_000_000),
    ])
    node = GnssObservationNode(parameter_overrides=overrides)
    harness = rclpy.create_node("gnss_observation_test_harness")
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pvt_pub = harness.create_publisher(GnssNavPvt, topics["nav_pvt_topic"], qos)
    pos_pub = harness.create_publisher(GnssNavHpPosLlh, topics["nav_hp_pos_llh_topic"], qos)
    cov_pub = harness.create_publisher(GnssNavCov, topics["nav_cov_topic"], qos)
    status_pub = harness.create_publisher(GnssNavStatus, topics["nav_status_topic"], qos)
    time_pub = harness.create_publisher(GnssReceiverTime, topics["receiver_time_topic"], qos)
    received = []
    observation_sub = harness.create_subscription(
        GnssObservation, topics["observation_topic"], received.append, qos
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(harness)
    try:
        publishers = [pvt_pub, pos_pub, cov_pub, status_pub, time_pub]
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            if all(p.get_subscription_count() > 0 for p in publishers) and node._publisher.get_subscription_count() > 0:
                break
            executor.spin_once(timeout_sec=0.01)
        assert all(p.get_subscription_count() > 0 for p in publishers)
        assert node._publisher.get_subscription_count() > 0

        session = "ros-f9p-session"
        generation = 7
        week = 2450
        i_tow = 123456
        clock_id = f"{session}:connection:{generation}:gps-week:{week}"

        pvt = GnssNavPvt()
        pvt.i_tow_ms = i_tow
        pvt.gps_week = week
        pvt.process_session_id = session
        pvt.connection_generation = generation
        pvt.raw_flags = 0x81

        pos = GnssNavHpPosLlh()
        pos.i_tow_ms = i_tow
        pos.gps_week = week
        pos.process_session_id = session
        pos.connection_generation = generation
        pos.lon = -1_220_000_000
        pos.lat = 370_000_000
        pos.height_mm = 15_000
        pos.lon_hp = 1
        pos.lat_hp = -1
        pos.height_hp = 2
        pos.raw_flags = 0

        cov = GnssNavCov()
        cov.i_tow_ms = i_tow
        cov.gps_week = week
        cov.process_session_id = session
        cov.connection_generation = generation
        cov.raw_flags = 1
        cov.pos_cov_nn = 1.0
        cov.pos_cov_ne = 0.0
        cov.pos_cov_nd = 0.0
        cov.pos_cov_ee = 1.5
        cov.pos_cov_ed = 0.0
        cov.pos_cov_dd = 2.0

        status = GnssNavStatus()
        status.i_tow_ms = i_tow
        status.gps_week = week
        status.process_session_id = session
        status.connection_generation = generation
        status.raw_flags = 1
        status.raw_fix_stat = 4

        mapping = GnssReceiverTime()
        mapping.i_tow_ms = i_tow
        mapping.source_timestamp_ns = 1_480_000_000_123_000_000
        mapping.source_clock_id = clock_id
        mapping.receive_monotonic_ns = time.monotonic_ns()
        mapping.valid = True
        mapping.utc_stamp_valid = False
        mapping.utc_seconds_valid = False
        mapping.utc_seconds = 0
        mapping.utc_nanoseconds_valid = False
        mapping.utc_nanoseconds = 0
        mapping.validity_reason = "leap source not trusted"
        mapping.gps_week_valid = True
        mapping.gps_week = week
        mapping.source_time_accuracy_valid = True
        mapping.source_time_accuracy_ns = 40
        mapping.leap_seconds_valid = True
        mapping.leap_seconds = 18
        mapping.leap_source_valid = True
        mapping.leap_source = 2
        mapping.reported_leap_seconds_valid = True
        mapping.reported_leap_seconds = 18
        mapping.leap_event_valid = False
        mapping.time_to_leap_event_seconds_valid = False
        mapping.time_to_leap_event_seconds = 0
        mapping.process_session_id = session
        mapping.connection_generation = generation

        # The order deliberately differs from the mapper output order.
        pvt_pub.publish(pvt)
        cov_pub.publish(cov)
        status_pub.publish(status)
        pos_pub.publish(pos)
        time_pub.publish(mapping)

        deadline = time.monotonic() + 8
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(received) == 1
        result = received[0]
        assert result.i_tow_ms == i_tow
        assert result.gps_week == week
        assert result.process_session_id == session
        assert result.connection_generation == generation
        assert result.source_timestamp_ns == mapping.source_timestamp_ns
        assert result.source_clock_id == clock_id
        assert result.gnss_fix_ok is True
        assert result.carrier_solution == 2
        assert result.covariance_type == 3
        assert result.latitude_deg == pytest.approx(37.0, abs=2e-8)
        assert result.longitude_deg == pytest.approx(-122.0, abs=2e-8)
        assert result.utc_stamp_valid is False
        assert result.source_time_accuracy_valid is True
        assert result.source_time_accuracy_ns == 40
    finally:
        executor.remove_node(node)
        executor.remove_node(harness)
        node.destroy_node()
        harness.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
