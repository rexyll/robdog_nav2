from types import SimpleNamespace

import pytest

from robodog_gnss_quality.quality_gate import CarrierSolution, CovarianceType
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_time.ubx_ros_epoch_runtime import (
    UbxRosEpochRuntime,
    nav_cov_from_ros,
    nav_cov_from_ublox_ros,
    nav_hp_pos_llh_from_ros,
    nav_hp_pos_llh_from_ublox_ros,
    nav_pvt_from_ros,
    nav_pvt_from_ublox_ros,
    nav_status_from_ros,
    nav_status_from_ublox_ros,
    receiver_time_from_ros,
)


SESSION = "f9p-session-a"
GENERATION = 4
WEEK = 2450
I_TOW = 123456
CLOCK_ID = f"{SESSION}:connection:{GENERATION}:gps-week:{WEEK}"


def pvt(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, gps_fix_type=3, gnss_fix_ok=True,
        diff_soln=True, carrier_solution=2, invalid_llh=False,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def position(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, version=0,
        invalid_lon=False, invalid_lat=False, invalid_height=False,
        invalid_hmsl=False, invalid_lon_hp=False, invalid_lat_hp=False,
        invalid_height_hp=False, invalid_hmsl_hp=False,
        lon=-1220000000, lat=370000000, height_mm=15000, hmsl_mm=10000,
        lon_hp=1, lat_hp=-1, height_hp=2, hmsl_hp=0, h_acc=100, v_acc=120,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def covariance(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, version=0,
        pos_cor_valid=True, vel_cor_valid=True,
        pos_cov_nn=1.0, pos_cov_ne=0.0, pos_cov_nd=0.0,
        pos_cov_ee=1.5, pos_cov_ed=0.0, pos_cov_dd=2.0,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def status(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, gps_fix_type=3,
        gps_fix_ok=True, diff_soln=True, wkn_set=True, tow_set=True,
        diff_corr=True, carrier_solution_valid=True, carrier_solution=2,
        map_matching_status=0, psm_state=1, spoof_det_state=1,
        ttff=700, msss=123456,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def receiver_time(**overrides):
    values = dict(
        i_tow_ms=I_TOW,
        source_timestamp_ns=1_480_000_000_123_000_000,
        source_clock_id=CLOCK_ID,
        receive_monotonic_ns=140,
        valid=True,
        utc_stamp_valid=False,
        utc_seconds_valid=False,
        utc_seconds=0,
        utc_nanoseconds_valid=False,
        utc_nanoseconds=0,
        validity_reason="leap source not trusted",
        gps_week_valid=True,
        gps_week=WEEK,
        source_time_accuracy_valid=True,
        source_time_accuracy_ns=40,
        leap_seconds_valid=True,
        leap_seconds=18,
        leap_source_valid=True,
        leap_source=2,
        reported_leap_seconds_valid=True,
        reported_leap_seconds=18,
        leap_event_valid=False,
        time_to_leap_event_seconds_valid=False,
        time_to_leap_event_seconds=0,
        process_session_id=SESSION,
        connection_generation=GENERATION,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_decoded_nav_pvt_and_status_fields_map_to_quality_records():
    p = nav_pvt_from_ros(pvt(), receive_monotonic_ns=10)
    s = nav_status_from_ros(status(), receive_monotonic_ns=13)
    assert p.gnss_fix_ok is True
    assert p.carrier_solution is CarrierSolution.FIXED
    assert s.gnss_fix_ok is True
    assert s.carrier_solution is CarrierSolution.FIXED
    assert s.carrier_solution_valid is True


def test_nav_pvt_invalid_llh_is_rejected():
    with pytest.raises(EpochPairingError, match="NAV-PVT marks LLH invalid"):
        nav_pvt_from_ros(pvt(invalid_llh=True), receive_monotonic_ns=10)


def test_hpposllh_preserves_individual_component_invalidity():
    record = nav_hp_pos_llh_from_ros(
        position(invalid_lat_hp=True, invalid_hmsl=True), receive_monotonic_ns=11
    )
    assert record.invalid_lat_hp is True
    assert record.invalid_lon is False
    assert record.invalid_height is False


def test_nav_cov_validity_and_finite_values_are_preserved():
    record = nav_cov_from_ros(covariance(pos_cor_valid=False), receive_monotonic_ns=12)
    assert record.pos_cor_valid is False
    with pytest.raises(EpochPairingError, match="finite"):
        nav_cov_from_ros(covariance(pos_cov_nn=float("nan")), receive_monotonic_ns=12)


def test_receiver_time_optional_fields_require_validity_sentinels():
    mapped = receiver_time_from_ros(receiver_time())
    assert mapped.valid is True
    assert mapped.utc_stamp_valid is False
    assert mapped.utc_seconds is None
    assert mapped.time_to_leap_event_seconds is None
    with pytest.raises(EpochPairingError, match="zero when"):
        receiver_time_from_ros(receiver_time(gps_week_valid=False, gps_week=WEEK))


def test_four_stream_join_waits_for_matching_receiver_time_mapping():
    published = []
    clock_values = iter((100, 110, 120, 130, 140))
    runtime = UbxRosEpochRuntime(
        max_pending_epochs=8,
        max_pair_receive_skew_ns=500,
        max_pending_age_ns=10_000,
        publish_observation=published.append,
        monotonic_clock=lambda: next(clock_values),
    )
    assert runtime.handle_nav_pvt(pvt()) is None
    assert runtime.handle_nav_hp_pos_llh(position()) is None
    assert runtime.handle_nav_cov(covariance()) is None
    assert runtime.handle_nav_status(status()) is None
    assert published == []

    result = runtime.handle_receiver_time(receiver_time())
    assert result is not None
    assert len(published) == 1
    observation = published[0].observation
    assert observation.source_timestamp_ns == receiver_time().source_timestamp_ns
    assert observation.source_clock_id == CLOCK_ID
    assert observation.gnss_fix_ok is True
    assert observation.carrier_solution is CarrierSolution.FIXED
    assert observation.covariance_type is CovarianceType.KNOWN
    assert observation.latitude_deg == pytest.approx(37.0, abs=2e-8)
    assert observation.longitude_deg == pytest.approx(-122.0, abs=2e-8)
    assert observation.receive_monotonic_ns == 130


def test_new_valid_mapping_activates_generation_and_old_callbacks_are_dropped():
    published = []
    runtime = UbxRosEpochRuntime(
        max_pending_epochs=8, max_pair_receive_skew_ns=500,
        max_pending_age_ns=10_000, publish_observation=published.append,
        monotonic_clock=lambda: 100,
    )
    runtime.handle_nav_pvt(pvt())
    next_session = "f9p-session-b"
    next_generation = 0
    next_clock = f"{next_session}:connection:{next_generation}:gps-week:{WEEK}"
    new_map = receiver_time(
        source_clock_id=next_clock,
        process_session_id=next_session,
        connection_generation=next_generation,
        receive_monotonic_ns=110,
    )
    runtime.handle_receiver_time(new_map)
    stale = runtime.handle_nav_pvt(pvt())
    assert stale is None
    assert runtime.stale_context_drops == 1
    assert published == []


def test_raw_callback_rejects_invalid_unsigned_and_float_fields():
    with pytest.raises(EpochPairingError, match="i_tow_ms"):
        nav_pvt_from_ros(pvt(i_tow_ms=True), receive_monotonic_ns=10)
    with pytest.raises(EpochPairingError, match="carrier_solution"):
        nav_pvt_from_ros(pvt(carrier_solution=256), receive_monotonic_ns=10)
    with pytest.raises(EpochPairingError, match="connection_generation"):
        nav_pvt_from_ros(pvt(connection_generation=1.0), receive_monotonic_ns=10)


def test_pinned_ublox_nav_pvt_message_adapter_uses_decoded_fields_not_header_stamp():
    upstream = SimpleNamespace(
        itow=I_TOW,
        gps_fix=SimpleNamespace(fix_type=3),
        gnss_fix_ok=True,
        diff_soln=True,
        carr_soln=SimpleNamespace(status=2),
        invalid_llh=False,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=2)),
    )
    record = nav_pvt_from_ublox_ros(
        upstream, process_session_id=SESSION, connection_generation=GENERATION,
        gps_week=WEEK, receive_monotonic_ns=77,
    )
    assert record.i_tow_ms == I_TOW
    assert record.receive_monotonic_ns == 77
    assert record.carrier_solution is CarrierSolution.FIXED


def test_pinned_ublox_hpposllh_adapter_preserves_per_component_invalid_flags():
    upstream = SimpleNamespace(
        itow=I_TOW, version=0, invalid_lon=False, invalid_lat=False,
        invalid_height=False, invalid_hmsl=False, invalid_lon_hp=False,
        invalid_lat_hp=True, invalid_height_hp=False, invalid_hmsl_hp=False,
        lon=-1220000000, lat=370000000, height=15000, hmsl=10000,
        lon_hp=1, lat_hp=-1, height_hp=2, hmsl_hp=0, h_acc=100, v_acc=120,
    )
    record = nav_hp_pos_llh_from_ublox_ros(
        upstream, process_session_id=SESSION, connection_generation=GENERATION,
        gps_week=WEEK, receive_monotonic_ns=78,
    )
    assert record.invalid_lat_hp is True
    assert record.invalid_lon_hp is False
    assert record.height_mm == 15000


def test_pinned_ublox_nav_cov_and_status_adapters_use_semantic_fields():
    upstream_cov = SimpleNamespace(
        itow=I_TOW, version=0, pos_cor_valid=True, vel_cor_valid=True,
        pos_cov_nn=1.0, pos_cov_ne=0.0, pos_cov_nd=0.0,
        pos_cov_ee=1.5, pos_cov_ed=0.0, pos_cov_dd=2.0,
    )
    upstream_status = SimpleNamespace(
        itow=I_TOW, gps_fix=SimpleNamespace(fix_type=3), gps_fix_ok=True,
        diff_soln=True, wkn_set=True, tow_set=True, diff_corr=True,
        carr_soln_valid=True, carr_soln=SimpleNamespace(status=2),
        map_matching=SimpleNamespace(status=0), psm=SimpleNamespace(state=1),
        spoof_det=SimpleNamespace(state=1), ttff=700, msss=123456,
    )
    cov_record = nav_cov_from_ublox_ros(
        upstream_cov, process_session_id=SESSION, connection_generation=GENERATION,
        gps_week=WEEK, receive_monotonic_ns=79,
    )
    status_record = nav_status_from_ublox_ros(
        upstream_status, process_session_id=SESSION, connection_generation=GENERATION,
        gps_week=WEEK, receive_monotonic_ns=80,
    )
    assert cov_record.pos_cor_valid is True
    assert status_record.carrier_solution_valid is True
    assert status_record.carrier_solution is CarrierSolution.FIXED
