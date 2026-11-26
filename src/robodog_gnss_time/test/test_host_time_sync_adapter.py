import pytest
from robodog_gnss_interfaces.msg import HostTimeSyncStatus

from robodog_gnss_time.host_time_sync_adapter import HostTimeSyncStatusAdapter


def message(**changes):
    msg = HostTimeSyncStatus()
    msg.state = HostTimeSyncStatus.STATE_SYNCHRONIZED
    msg.source_id = "chrony:tracking"
    msg.host_boot_id = "boot-abc"
    msg.utc_offset_ns = -125_000
    msg.uncertainty_ns = 250_000
    msg.sample_monotonic_ns = 10_000_000
    msg.sample_sequence = 1
    msg.reference_id = "203.0.113.10"
    msg.stratum = 3
    msg.leap_status = "Normal"
    for key, value in changes.items():
        setattr(msg, key, value)
    return msg


def test_decodes_synchronized_host_time_evidence():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    sample = adapter.decode(message())
    assert sample.state == "synchronized"
    assert sample.source_id == "chrony:tracking"
    assert sample.utc_offset_ns == -125_000
    assert sample.uncertainty_ns == 250_000
    assert sample.reference_id == "203.0.113.10"
    assert sample.stratum == 3
    assert sample.leap_status == "Normal"
    assert sample.as_policy_kwargs()["host_sync_sample_monotonic_ns"] == 10_000_000


@pytest.mark.parametrize(
    "state,expected",
    [
        (HostTimeSyncStatus.STATE_UNKNOWN, "unknown"),
        (HostTimeSyncStatus.STATE_UNSYNCHRONIZED, "unsynchronized"),
        (HostTimeSyncStatus.STATE_FAULT, "fault"),
    ],
)
def test_preserves_non_synchronized_states_as_fail_closed_evidence(state, expected):
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    assert adapter.decode(message(state=state)).state == expected


def test_rejects_cross_boot_clock_sample():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(ValueError):
        adapter.decode(message(host_boot_id="old-boot"))


def test_rejects_missing_source_and_unknown_state():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(ValueError):
        adapter.decode(message(source_id=""))
    with pytest.raises(ValueError):
        adapter.decode(message(state=99))


def test_rejects_replayed_or_reordered_sequence_and_monotonic_sample():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    adapter.decode(message())
    with pytest.raises(ValueError):
        adapter.decode(message(sample_sequence=1, sample_monotonic_ns=11_000_000))
    with pytest.raises(ValueError):
        adapter.decode(message(sample_sequence=2, sample_monotonic_ns=10_000_000))


def test_rejects_bool_alias_and_rosidl_out_of_range_uint64_fields():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(ValueError):
        adapter.decode(message(uncertainty_ns=True))
    with pytest.raises(ValueError):
        adapter.decode(message(sample_sequence=True))
    # Exercise the adapter's numeric boundary directly: ROSIDL generated
    # uint64 setters differ across supported ROS distributions for negatives.
    with pytest.raises(ValueError):
        adapter._uint("sample_sequence", -1)


def test_rejects_wrong_message_type_and_empty_expected_boot_id():
    with pytest.raises(ValueError):
        HostTimeSyncStatusAdapter(expected_host_boot_id="")
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(TypeError):
        adapter.decode(object())


def test_rejects_missing_chrony_reference_or_leap_evidence():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(ValueError):
        adapter.decode(message(reference_id=""))
    with pytest.raises(ValueError):
        adapter.decode(message(leap_status=""))


def test_synchronized_state_requires_healthy_chrony_diagnostics():
    adapter = HostTimeSyncStatusAdapter(expected_host_boot_id="boot-abc")
    with pytest.raises(ValueError):
        adapter.decode(message(stratum=16))
    with pytest.raises(ValueError):
        adapter.decode(message(leap_status="Not synchronised"))
    with pytest.raises(ValueError):
        adapter.decode(message(reference_id="00000000"))
