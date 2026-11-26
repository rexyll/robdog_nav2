import pytest

from robodog_gnss_time.ros_clock_policy import (
    ClockPolicyConfig,
    ClockPolicyInput,
    ClockPolicyReason,
    RosSystemTimeUtcPolicy,
)


def sample(**changes):
    values = dict(
        use_sim_time=False,
        utc_stamp_valid=True,
        utc_seconds=1_800_000_000,
        utc_nanoseconds=123_456_789,
        leap_guard_active=False,
        host_sync_state="synchronized",
        host_sync_source="chrony:tracking",
        host_utc_offset_ns=100_000,
        host_utc_uncertainty_ns=200_000,
        host_sync_sample_monotonic_ns=9_000_000_000,
    )
    values.update(changes)
    return ClockPolicyInput(**values)


def gate():
    return RosSystemTimeUtcPolicy(
        ClockPolicyConfig(
            max_host_utc_error_ns=2_000_000,
            max_sync_sample_age_ns=500_000_000,
        )
    )


def test_accepts_only_fresh_system_time_with_bounded_utc_error():
    result = gate().evaluate(sample(), now_monotonic_ns=9_100_000_000)
    assert result.allowed is True
    assert result.reason == ClockPolicyReason.ACCEPTED
    assert result.host_utc_error_bound_ns == 300_000


@pytest.mark.parametrize(
    "changes,reason",
    [
        ({"use_sim_time": True}, ClockPolicyReason.SIM_TIME_ACTIVE),
        ({"utc_stamp_valid": False}, ClockPolicyReason.UTC_INVALID),
        ({"utc_nanoseconds": 1_000_000_000}, ClockPolicyReason.UTC_TIMESTAMP_INVALID),
        ({"utc_seconds": -1}, ClockPolicyReason.UTC_TIMESTAMP_INVALID),
        ({"utc_seconds": (1 << 31)}, ClockPolicyReason.UTC_SECONDS_OUT_OF_RANGE),
        ({"leap_guard_active": True}, ClockPolicyReason.LEAP_GUARD_ACTIVE),
        ({"host_sync_state": "unknown"}, ClockPolicyReason.HOST_SYNC_UNKNOWN),
        ({"host_sync_source": ""}, ClockPolicyReason.HOST_SYNC_SOURCE_MISSING),
        ({"host_utc_offset_ns": "0"}, ClockPolicyReason.HOST_SYNC_OFFSET_INVALID),
        ({"host_utc_uncertainty_ns": -1}, ClockPolicyReason.HOST_SYNC_UNCERTAINTY_INVALID),
    ],
)
def test_fail_closed_conditions(changes, reason):
    result = gate().evaluate(sample(**changes), now_monotonic_ns=9_100_000_000)
    assert result.allowed is False
    assert result.reason == reason


def test_stale_or_future_sync_measurement_is_rejected():
    policy = gate()
    stale = policy.evaluate(
        sample(), now_monotonic_ns=9_600_000_001
    )
    future = policy.evaluate(
        sample(host_sync_sample_monotonic_ns=9_100_000_001),
        now_monotonic_ns=9_100_000_000,
    )
    assert stale.reason == ClockPolicyReason.HOST_SYNC_SAMPLE_STALE
    assert future.reason == ClockPolicyReason.MONOTONIC_TIME_INVALID


def test_offset_plus_uncertainty_must_fit_budget():
    result = gate().evaluate(
        sample(host_utc_offset_ns=-1_900_000, host_utc_uncertainty_ns=200_001),
        now_monotonic_ns=9_100_000_000,
    )
    assert result.reason == ClockPolicyReason.HOST_SYNC_ERROR_EXCEEDED
    assert result.host_utc_error_bound_ns == 2_100_001


def test_config_rejects_bool_and_nonpositive_thresholds():
    with pytest.raises(ValueError):
        ClockPolicyConfig(max_host_utc_error_ns=True, max_sync_sample_age_ns=1)
    with pytest.raises(ValueError):
        ClockPolicyConfig(max_host_utc_error_ns=1, max_sync_sample_age_ns=0)
