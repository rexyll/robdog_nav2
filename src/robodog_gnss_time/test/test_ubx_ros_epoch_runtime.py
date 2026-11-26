from types import SimpleNamespace

import pytest

from robodog_gnss_quality.quality_gate import CarrierSolution, CovarianceType
from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_time.ubx_ros_epoch_runtime import (
    UbxRosEpochRuntime,
    nav_cov_from_ros,
    nav_hp_pos_llh_from_ros,
    nav_pvt_from_ros,
    nav_status_from_ros,
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
        connection_generation=GENERATION, raw_flags=0x81,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def position(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, lon=-1220000000, lat=370000000,
        height_mm=15000, lon_hp=1, lat_hp=-1, height_hp=2, raw_flags=0,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def covariance(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, raw_flags=0x01,
        pos_cov_nn=1.0, pos_cov_ne=0.0, pos_cov_nd=0.0,
        pos_cov_ee=1.5, pos_cov_ed=0.0, pos_cov_dd=2.0,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def status(**overrides):
    values = dict(
        i_tow_ms=I_TOW, gps_week=WEEK, process_session_id=SESSION,
        connection_generation=GENERATION, raw_flags=0x01, raw_fix_stat=0x04,
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


def test_nav_pvt_and_status_adapters_decode_raw_flags():
    p = nav_pvt_from_ros(pvt(), receive_monotonic_ns=10)
    s = nav_status_from_ros(status(), receive_monotonic_ns=13)
    assert p.gnss_fix_ok is True
    assert p.carrier_solution is CarrierSolution.FIXED
    assert s.gnss_fix_ok is True
    assert s.carrier_solution is CarrierSolution.FIXED
    assert s.carrier_solution_valid is True


def test_hpposllh_invalid_flag_invalidates_all_components():
    record = nav_hp_pos_llh_from_ros(position(raw_flags=0x01), receive_monotonic_ns=11)
    assert all((
        record.invalid_lon, record.invalid_lat, record.invalid_height,
        record.invalid_lon_hp, record.invalid_lat_hp, record.invalid_height_hp,
    ))


def test_nav_cov_validity_and_finite_values_are_preserved():
    record = nav_cov_from_ros(covariance(raw_flags=0), receive_monotonic_ns=12)
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
    assert observation.latitude_deg == pytest.approx(37.0)
    assert observation.longitude_deg == pytest.approx(-122.0)
    assert observation.receive_monotonic_ns == 130


def test_new_valid_mapping_activates_generation_and_old_callbacks_are_dropped():
    published = []
    now = [100]
    runtime = UbxRosEpochRuntime(
        max_pending_epochs=8, max_pair_receive_skew_ns=500,
        max_pending_age_ns=10_000, publish_observation=published.append,
        monotonic_clock=lambda: now[0],
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
    with pytest.raises(EpochPairingError, match="raw_flags"):
        nav_pvt_from_ros(pvt(raw_flags=256), receive_monotonic_ns=10)
    with pytest.raises(EpochPairingError, match="connection_generation"):
        nav_pvt_from_ros(pvt(connection_generation=1.0), receive_monotonic_ns=10)
