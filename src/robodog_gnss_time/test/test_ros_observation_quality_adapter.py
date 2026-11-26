import pytest
from robodog_gnss_interfaces.msg import GnssObservation, GnssQualityStatus
from robodog_gnss_time.ros_observation_quality_adapter import (
    RosObservationQualityJoinAdapter,
    decision_from_quality_message,
    identity_from_observation_message,
)


def make_observation():
    msg = GnssObservation()
    msg.i_tow_ms = 123456
    msg.gps_week = 2450
    msg.process_session_id = "adapter-session"
    msg.connection_generation = 4
    msg.source_timestamp_ns = 1_234_567_890
    msg.source_clock_id = "adapter-session:connection:4:gps-week:2450"
    msg.receive_monotonic_ns = 9_000_000_000
    msg.latitude_deg = 37.0
    msg.longitude_deg = -122.0
    msg.altitude_m = 10.0
    msg.navsat_status = 0
    msg.gnss_fix_ok = True
    msg.carrier_solution = 2
    msg.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.04]
    msg.covariance_type = 3
    return msg


def make_decision(observation=None, *, qualified=True, reason="accepted"):
    observation = observation or make_observation()
    msg = GnssQualityStatus()
    msg.qualified = qualified
    msg.reason = reason
    msg.consecutive_fixed_samples = 3
    msg.horizontal_std_valid = True
    msg.horizontal_std_m = 0.1
    msg.vertical_std_valid = True
    msg.vertical_std_m = 0.2
    msg.estimated_speed_valid = False
    msg.estimated_speed_mps = 0.0
    msg.i_tow_ms = observation.i_tow_ms
    msg.gps_week = observation.gps_week
    msg.process_session_id = observation.process_session_id
    msg.connection_generation = observation.connection_generation
    msg.source_timestamp_ns = observation.source_timestamp_ns
    msg.source_clock_id = observation.source_clock_id
    msg.receive_monotonic_ns = observation.receive_monotonic_ns
    return msg


def test_adapter_emits_only_exact_matching_qualified_pair_in_either_order():
    observation = make_observation()
    decision = make_decision(observation)
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    assert adapter.add_decision(decision, now_monotonic_ns=9_000_000_100) is None
    result = adapter.add_observation(observation, now_monotonic_ns=9_000_000_200)
    assert result is not None
    assert result.identity.i_tow_ms == observation.i_tow_ms
    assert result.observation.source_timestamp_ns == observation.source_timestamp_ns
    assert result.quality_decision.qualified is True
    assert adapter.stats().emitted == 1


def test_observation_first_then_quality_decision_emits():
    observation = make_observation()
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    assert adapter.add_observation(observation, now_monotonic_ns=9_000_000_100) is None
    result = adapter.add_decision(make_decision(observation), now_monotonic_ns=9_000_000_200)
    assert result is not None


def test_identity_mismatch_never_nearest_time_pairs():
    observation = make_observation()
    decision = make_decision(observation)
    decision.connection_generation += 1
    decision.source_clock_id = "adapter-session:connection:5:gps-week:2450"
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    adapter.add_observation(observation, now_monotonic_ns=9_000_000_100)
    assert adapter.add_decision(decision, now_monotonic_ns=9_000_000_200) is None
    assert adapter.stats().emitted == 0


def test_unqualified_decision_retires_matching_observation():
    observation = make_observation()
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    adapter.add_observation(observation, now_monotonic_ns=9_000_000_100)
    decision = make_decision(observation, qualified=False, reason="rtk_not_fixed")
    assert adapter.add_decision(decision, now_monotonic_ns=9_000_000_200) is None
    assert adapter.add_observation(observation, now_monotonic_ns=9_000_000_300) is None
    assert adapter.stats().rejected_unqualified == 1


def test_invalid_boolean_alias_and_unknown_reason_are_rejected():
    decision = make_decision()
    decision.connection_generation = True
    with pytest.raises(ValueError):
        decision_from_quality_message(decision)
    decision = make_decision()
    decision.reason = "not_a_real_reason"
    with pytest.raises(ValueError):
        decision_from_quality_message(decision)


def test_quality_decision_requires_consistent_qualified_reason_and_metric_sentinels():
    decision = make_decision()
    decision.reason = "recovering"
    with pytest.raises(ValueError):
        decision_from_quality_message(decision)
    decision = make_decision()
    decision.estimated_speed_valid = False
    decision.estimated_speed_mps = 1.0
    with pytest.raises(ValueError):
        decision_from_quality_message(decision)


def test_observation_clock_identity_is_strict():
    observation = make_observation()
    observation.source_clock_id = "wrong-clock"
    with pytest.raises(ValueError):
        identity_from_observation_message(observation)



def test_identity_free_startup_no_data_status_is_ignored():
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    startup = GnssQualityStatus()
    startup.qualified = False
    startup.reason = "no_data"
    assert adapter.add_decision(startup, now_monotonic_ns=9_000_000_000) is None
    assert adapter.stats().pending_decisions == 0
    assert adapter.stats().emitted == 0


def test_malformed_no_data_status_is_not_silently_ignored():
    adapter = RosObservationQualityJoinAdapter(max_pending_epochs=4, max_pending_age_ns=1000)
    startup = GnssQualityStatus()
    startup.qualified = False
    startup.reason = "no_data"
    startup.process_session_id = "unexpected-session"
    with pytest.raises(ValueError):
        adapter.add_decision(startup, now_monotonic_ns=9_000_000_000)
