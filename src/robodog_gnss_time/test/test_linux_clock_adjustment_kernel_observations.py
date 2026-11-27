import pytest

from robodog_gnss_time.linux_clock_adjustment_kernel_observations import (
    reduce_kernel_clock_adjustment_observations,
)

BOOT = "boot-a"
PID = 0x100000002


def _event(event_type, sequence, *, handler="do_clock_adjtime", pid_tgid=PID,
           monotonic_ns=None, cpu=0, **extra):
    if monotonic_ns is None:
        monotonic_ns = sequence * 100
    event = {
        "event_type": event_type, "handler": handler, "boot_id": BOOT,
        "pid_tgid": pid_tgid, "monotonic_ns": monotonic_ns,
        "cpu": cpu, "sequence": sequence,
    }
    if event_type == "kernel_handler_enter":
        event["raw_modes"] = 0x2100
        if handler == "do_clock_adjtime":
            event["raw_clockid"] = 0
    else:
        event["return_value"] = 0
    event.update(extra)
    return event


def test_pairs_post_copy_kernel_request_and_handler_return():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
        _event("kernel_handler_exit", 2, monotonic_ns=250),
    ])
    assert report["reduction_status"] == "consistent"
    pair = report["pairs"][0]
    assert pair["request_observation_stage"] == "post_copy_kernel_request"
    assert pair["raw_modes"] == 0x2100
    assert pair["raw_clockid"] == 0
    assert pair["outer_syscall_correlation"] == "not_joined"
    assert pair["clock_effect"] == "not_measured"


def test_adjtimex_handler_has_no_clockid_field():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, handler="do_adjtimex"),
        _event("kernel_handler_exit", 2, handler="do_adjtimex"),
    ])
    assert report["reduction_status"] == "consistent"
    assert report["pairs"][0]["raw_clockid"] is None


def test_nonnegative_handler_return_is_not_effect_proof():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
        _event("kernel_handler_exit", 2, return_value=1),
    ])
    assert report["pairs"][0]["handler_return_interpretation"] == "nonnegative_handler_return_not_effect_proof"


def test_negative_efault_handler_result_is_not_copyback_proof():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
        _event("kernel_handler_exit", 2, return_value=-14),
    ])
    assert report["pairs"][0]["handler_return_interpretation"] == "negative_efault_handler_result_not_copyback_proof"


def test_orphan_handler_return_is_partial():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_exit", 1),
    ])
    assert report["reduction_status"] == "partial"
    assert report["counts"]["orphan_handler_returns"] == 1


def test_handler_entry_without_return_is_retained():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
    ])
    assert report["unpaired_entries"][0]["unpaired_reason"] == "no_matching_handler_return"


def test_duplicate_handler_entry_supersedes_old_and_marks_partial():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
        _event("kernel_handler_enter", 2),
        _event("kernel_handler_exit", 3),
    ])
    assert report["counts"]["duplicate_handler_entries"] == 1
    assert report["unpaired_entries"][0]["unpaired_reason"] == "superseded_by_duplicate_handler_entry"


def test_per_cpu_sequence_gap_is_reported():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1),
        _event("kernel_handler_exit", 3),
    ])
    assert report["issues"][0]["code"] == "per_cpu_sequence_gap"


def test_sequence_is_partitioned_by_cpu():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, cpu=0),
        _event("kernel_handler_exit", 1, cpu=1),
    ])
    assert report["reduction_status"] == "consistent"


def test_thread_identity_is_part_of_handler_pair_key():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, pid_tgid=PID),
        _event("kernel_handler_exit", 2, pid_tgid=PID + 1),
    ])
    assert report["reduction_status"] == "partial"
    assert len(report["unpaired_entries"]) == 1
    assert len(report["orphan_returns"]) == 1


def test_boot_identity_is_part_of_handler_pair_key():
    event = _event("kernel_handler_exit", 2)
    event["boot_id"] = "boot-b"
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1), event,
    ])
    assert report["reduction_status"] == "partial"
    assert len(report["unpaired_entries"]) == 1
    assert len(report["orphan_returns"]) == 1


def test_clock_adjtime_requires_signed_32_bit_clockid():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, raw_clockid=1 << 40),
    ])
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "invalid_kernel_raw_clockid"


def test_adjtimex_rejects_unexpected_clockid():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, handler="do_adjtimex", raw_clockid=0),
    ])
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "unexpected_kernel_raw_clockid"


def test_bool_modes_are_rejected():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, raw_modes=True),
    ])
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "invalid_kernel_raw_modes"


def test_handler_return_timestamp_before_entry_is_not_paired():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 10, monotonic_ns=1000),
        _event("kernel_handler_exit", 11, monotonic_ns=999),
    ])
    assert report["reduction_status"] == "partial"
    assert report["unpaired_entries"][0]["unpaired_reason"] == "handler_return_precedes_entry"


def test_pending_capacity_is_bounded():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, pid_tgid=PID),
        _event("kernel_handler_enter", 2, pid_tgid=PID + 1),
    ], max_pending=1)
    assert report["counts"]["pending_capacity_drops"] == 1
    assert report["unpaired_entries"][0]["unpaired_reason"] == "pending_capacity_exceeded"


def test_unhashable_event_type_is_rejected():
    report = reduce_kernel_clock_adjustment_observations([
        {"event_type": ["kernel_handler_enter"]},
    ])
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "unknown_event_type"


def test_unknown_fields_and_raw_pointers_are_not_echoed():
    report = reduce_kernel_clock_adjustment_observations([
        _event("kernel_handler_enter", 1, raw_user_pointer="secret", arbitrary={"x": 1}),
    ])
    assert "raw_user_pointer" not in str(report)
    assert "arbitrary" not in str(report)


def test_invalid_max_pending_is_rejected():
    with pytest.raises(ValueError):
        reduce_kernel_clock_adjustment_observations([], max_pending=0)
    with pytest.raises(TypeError):
        reduce_kernel_clock_adjustment_observations([], max_pending=True)
