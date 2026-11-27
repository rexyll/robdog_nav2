import pytest

from robodog_gnss_time.linux_clock_adjustment_events import (
    reduce_clock_adjustment_events,
)


BOOT = "boot-a"


def _event(event_type, sequence, *, pid_tgid=0x100000002,
           syscall="clock_adjtime", monotonic_ns=None, cpu=0, **extra):
    if monotonic_ns is None:
        monotonic_ns = sequence * 100
    return {
        "event_type": event_type,
        "boot_id": BOOT,
        "monotonic_ns": monotonic_ns,
        "pid_tgid": pid_tgid,
        "syscall": syscall,
        "cpu": cpu,
        "sequence": sequence,
        **extra,
    }


def test_pairs_entry_and_exit_and_keeps_clock_effect_unmeasured():
    events = [
        _event("syscall_enter", 1, raw_clockid=0, raw_modes=0x2100,
               modes_read_status="ok"),
        _event("syscall_exit", 2, monotonic_ns=250, return_value=0),
    ]

    report = reduce_clock_adjustment_events(events)

    assert report["reduction_status"] == "consistent"
    assert report["counts"]["entry_pair_completed"] == 1
    pair = report["pairs"][0]
    assert pair["duration_ns"] == 150
    assert pair["mode_decode"]["requested_operations"] == [
        "direct_time_offset_request",
    ]
    assert pair["return_interpretation"] == "nonnegative_handler_return_not_effect_proof"
    assert pair["clock_effect"] == "not_measured"
    assert pair["clock_target_resolution"] == "not_inferred"


def test_negative_efault_is_not_interpreted_as_no_side_effect():
    events = [
        _event("syscall_enter", 1),
        _event("syscall_exit", 2, return_value=-14),
    ]

    pair = reduce_clock_adjustment_events(events)["pairs"][0]
    assert pair["return_interpretation"] == "negative_efault_copy_stage_ambiguous"


def test_other_negative_errno_is_also_not_an_effect_verdict():
    events = [
        _event("syscall_enter", 1),
        _event("syscall_exit", 2, return_value=-22),
    ]

    pair = reduce_clock_adjustment_events(events)["pairs"][0]
    assert pair["return_interpretation"] == "negative_errno_effect_unresolved"


def test_orphan_exit_makes_reduction_partial():
    report = reduce_clock_adjustment_events([
        _event("syscall_exit", 1, return_value=0),
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["orphan_exit"] == 1
    assert report["issues"][0]["code"] == "orphan_exit"


def test_unmatched_entry_is_retained_at_end_of_input():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1),
    ])

    assert report["reduction_status"] == "partial"
    assert report["unpaired_entries"][0]["unpaired_reason"] == "no_matching_exit_in_input"


def test_duplicate_entry_overwrites_pending_record_but_marks_partial():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1),
        _event("syscall_enter", 2, monotonic_ns=220),
        _event("syscall_exit", 3, monotonic_ns=300, return_value=0),
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["inflight_overwrite"] == 1
    assert report["unpaired_entries"][0]["unpaired_reason"] == "superseded_by_duplicate_entry"
    assert report["pairs"][0]["entry_monotonic_ns"] == 220


def test_per_cpu_sequence_gap_is_reported():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 4),
        _event("syscall_exit", 6, return_value=0),
    ])

    assert report["reduction_status"] == "partial"
    assert report["sequence_gaps"][0]["missing_count"] == 1
    assert "per_cpu_sequence_gap" in [issue["code"] for issue in report["issues"]]


def test_sequence_tracking_is_partitioned_by_cpu():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, cpu=0),
        _event("syscall_exit", 1, cpu=1, return_value=0),
    ])

    assert report["reduction_status"] == "consistent"
    assert report["sequence_gaps"] == []


def test_tid_is_part_of_pair_identity_and_no_threads_are_cross_paired():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, pid_tgid=0x100000002),
        _event("syscall_exit", 2, pid_tgid=0x100000003, return_value=0),
    ])

    assert report["reduction_status"] == "partial"
    assert len(report["unpaired_entries"]) == 1
    assert len(report["orphan_exits"]) == 1


def test_boot_id_is_part_of_pair_identity():
    exit_event = _event("syscall_exit", 2, return_value=0)
    exit_event["boot_id"] = "boot-b"
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1),
        exit_event,
    ])

    assert report["reduction_status"] == "partial"
    assert len(report["unpaired_entries"]) == 1
    assert len(report["orphan_exits"]) == 1


def test_invalid_records_are_counted_and_not_echoed():
    report = reduce_clock_adjustment_events([
        {"event_type": "syscall_enter", "raw_user_pointer": "must-not-escape"},
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["malformed_events"] == 1
    assert "raw_user_pointer" not in str(report)


def test_entry_without_successful_modes_read_is_not_decoded():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, modes_read_status="user_read_failed"),
        _event("syscall_exit", 2, return_value=0),
    ])

    assert report["pairs"][0]["mode_decode"] is None
    assert report["pairs"][0]["modes_read_status"] == "user_read_failed"


def test_inflight_capacity_is_bounded():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, pid_tgid=0x100000002),
        _event("syscall_enter", 2, pid_tgid=0x100000003),
        _event("syscall_exit", 3, pid_tgid=0x100000002, return_value=0),
    ], max_pending=1)

    assert report["reduction_status"] == "partial"
    assert report["counts"]["inflight_capacity_drop"] == 1
    assert report["unpaired_entries"][0]["unpaired_reason"] == "inflight_capacity_exceeded"


def test_exit_timestamp_before_entry_is_not_accepted_as_a_pair():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, monotonic_ns=300),
        _event("syscall_exit", 2, monotonic_ns=200, return_value=0),
    ])

    assert report["reduction_status"] == "partial"
    assert report["pairs"] == []
    assert report["orphan_exits"][0]["return_value"] == 0


def test_invalid_capacity_is_rejected():
    with pytest.raises(ValueError):
        reduce_clock_adjustment_events([], max_pending=0)
    with pytest.raises(TypeError):
        reduce_clock_adjustment_events([], max_pending=True)


def test_unhashable_event_type_is_rejected_without_crashing():
    report = reduce_clock_adjustment_events([
        {"event_type": ["syscall_enter"], "raw_user_pointer": "must-not-escape"},
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "unknown_event_type"
    assert "raw_user_pointer" not in str(report)


def test_invalid_raw_clockid_is_rejected_without_echoing_object():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, raw_clockid={"fd": 3}),
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "invalid_raw_clockid"


def test_unhashable_modes_read_status_is_rejected():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, modes_read_status=["ok"]),
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "invalid_modes_read_status"


def test_invalid_raw_modes_record_is_not_counted_as_accepted():
    report = reduce_clock_adjustment_events([
        _event("syscall_enter", 1, modes_read_status="ok", raw_modes=True),
    ])

    assert report["reduction_status"] == "partial"
    assert report["counts"]["accepted_events"] == 0
    assert report["counts"]["malformed_events"] == 1
    assert report["issues"][0]["code"] == "invalid_raw_modes"
