from types import SimpleNamespace

import pytest

from robodog_gnss_quality.ubx_nav_pvt import EpochPairingError
from robodog_gnss_quality.receiver_time_ros_adapter import decode_receiver_time_mapping_message


def message(**overrides):
    values = dict(
        i_tow_ms=123456,
        source_timestamp_ns=1_480_000_000_000_000_000,
        source_clock_id="session:connection:1:gps-week:2450",
        receive_monotonic_ns=99_000,
        valid=True,
        utc_stamp_valid=False,
        utc_seconds_valid=False,
        utc_seconds=0,
        utc_nanoseconds_valid=False,
        utc_nanoseconds=0,
        validity_reason="leap source not trusted",
        gps_week_valid=True,
        gps_week=2450,
        source_time_accuracy_valid=True,
        source_time_accuracy_ns=50,
        leap_seconds_valid=True,
        leap_seconds=18,
        leap_source_valid=True,
        leap_source=2,
        reported_leap_seconds_valid=True,
        reported_leap_seconds=18,
        leap_event_valid=False,
        time_to_leap_event_seconds_valid=False,
        time_to_leap_event_seconds=0,
        process_session_id="session",
        connection_generation=1,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def test_decodes_gpsts_mapping_with_utc_untrusted():
    decoded = decode_receiver_time_mapping_message(message())
    assert decoded.valid is True
    assert decoded.utc_stamp_valid is False
    assert decoded.utc_seconds is None
    assert decoded.utc_nanoseconds is None
    assert decoded.gps_week == 2450
    assert decoded.source_timestamp_ns == 1_480_000_000_000_000_000
    assert decoded.process_session_id == "session"
    assert decoded.connection_generation == 1


def test_decodes_valid_utc_and_valid_leap_event_countdown():
    decoded = decode_receiver_time_mapping_message(message(
        utc_stamp_valid=True,
        utc_seconds_valid=True,
        utc_seconds=1_480_000_000,
        utc_nanoseconds_valid=True,
        utc_nanoseconds=123,
        leap_event_valid=True,
        time_to_leap_event_seconds_valid=True,
        time_to_leap_event_seconds=86_400,
    ))
    assert decoded.utc_stamp_valid is True
    assert decoded.utc_seconds == 1_480_000_000
    assert decoded.utc_nanoseconds == 123
    assert decoded.leap_event_valid is True
    assert decoded.time_to_leap_event_seconds == 86_400


@pytest.mark.parametrize("overrides", [
    {"gps_week": True},
    {"source_timestamp_ns": -1},
    {"connection_generation": 1.0},
    {"valid": 1},
    {"utc_stamp_valid": 1},
    {"gps_week_valid": 1},
    {"utc_nanoseconds": 1_000_000_000, "utc_nanoseconds_valid": True},
    {"utc_seconds": 4, "utc_seconds_valid": False},
    {"time_to_leap_event_seconds": 1, "time_to_leap_event_seconds_valid": False},
])
def test_rejects_invalid_ros_field_types_ranges_and_sentinels(overrides):
    with pytest.raises(EpochPairingError):
        decode_receiver_time_mapping_message(message(**overrides))


def test_rejects_source_clock_identity_mismatch():
    with pytest.raises(EpochPairingError, match="source_clock_id"):
        decode_receiver_time_mapping_message(message(source_clock_id="other-clock"))


def test_rejects_utc_validity_without_complete_valid_timestamp():
    with pytest.raises(EpochPairingError, match="both UTC fields"):
        decode_receiver_time_mapping_message(message(utc_stamp_valid=True))
