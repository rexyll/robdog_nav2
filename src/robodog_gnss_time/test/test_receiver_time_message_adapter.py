from types import SimpleNamespace

import pytest

from robodog_gnss_quality.ros_epoch_adapter import UbxSourceTimeMapping
from robodog_gnss_time.receiver_time_message_adapter import populate_receiver_time_message


def mapping(**overrides):
    values = dict(
        i_tow_ms=123456,
        source_timestamp_ns=1_480_000_000_000_000_000,
        source_clock_id="session:connection:1:gps-week:2450",
        receive_monotonic_ns=99_000,
        valid=True,
        utc_stamp_valid=True,
        utc_seconds=1_480_000_000,
        utc_nanoseconds=123,
        validity_reason="UTC offset validated",
        gps_week=2450,
        source_time_accuracy_ns=50,
        leap_seconds=18,
        leap_source=2,
        reported_leap_seconds=18,
        leap_event_valid=False,
        time_to_leap_event_seconds=None,
        process_session_id="session",
        connection_generation=1,
    )
    values.update(overrides)
    return UbxSourceTimeMapping(**values)


def message():
    return SimpleNamespace()


def test_populates_receiver_time_and_explicit_optional_fields():
    output = populate_receiver_time_message(mapping(), message())
    assert output.valid is True
    assert output.utc_stamp_valid is True
    assert output.utc_seconds_valid is True
    assert output.utc_seconds == 1_480_000_000
    assert output.utc_nanoseconds_valid is True
    assert output.utc_nanoseconds == 123
    assert output.gps_week_valid is True
    assert output.gps_week == 2450
    assert output.leap_event_valid is False
    assert output.time_to_leap_event_seconds_valid is False
    assert output.time_to_leap_event_seconds == 0
    assert output.process_session_id == "session"
    assert output.connection_generation == 1


def test_invalid_utc_has_no_timestamp_values():
    output = populate_receiver_time_message(
        mapping(utc_stamp_valid=False, utc_seconds=None, utc_nanoseconds=None,
                validity_reason="leap source not trusted"),
        message(),
    )
    assert output.valid is True
    assert output.utc_stamp_valid is False
    assert output.utc_seconds_valid is False
    assert output.utc_seconds == 0
    assert output.utc_nanoseconds_valid is False
    assert output.utc_nanoseconds == 0


@pytest.mark.parametrize("bad_week", [-1, 32768, True, 2450.0])
def test_rejects_invalid_gps_week(bad_week):
    with pytest.raises(ValueError, match="gps_week"):
        populate_receiver_time_message(mapping(gps_week=bad_week), message())


def test_invalid_utc_must_not_carry_values():
    with pytest.raises(ValueError, match="invalid UTC stamp"):
        populate_receiver_time_message(
            mapping(utc_stamp_valid=False, utc_seconds=1_480_000_000,
                    utc_nanoseconds=123),
            message(),
        )
