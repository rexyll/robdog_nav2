import pytest

from robodog_gnss_time.linux_clock_adjustment_invocation_reducer import (
    reduce_clock_adjustment_invocation_events,
)


def event(event_type, event_id, cpu, sequence, monotonic_ns, **overrides):
    payload = {"event_type": event_type, "pid_tgid": 4294967297}
    if event_type.startswith("syscall_"):
        payload["syscall"] = "clock_adjtime"
        if event_type == "syscall_enter":
            payload["modes_read_status"] = "not_requested"
        else:
            payload["return_value"] = 0
    else:
        payload["handler"] = "do_adjtimex"
        if event_type == "kernel_handler_enter":
            payload["raw_modes"] = 0
        else:
            payload["return_value"] = 0
    payload.update(overrides.pop("payload", {}))
    record = {
        "schema_version": 2,
        "capture_session_id": "capture-01",
        "boot_id": "boot-01",
        "producer_id": "producer-01",
        "stream_id": "syscall-stream",
        "stream_kind": "syscall",
        "cpu": cpu,
        "sequence": sequence,
        "monotonic_ns": monotonic_ns,
        "task_instance_id": "task-life-01",
        "event_id": event_id,
        "invocation_id": "syscall-inv-01",
        "payload": payload,
    }
    record.update(overrides)
    return record


def test_pairs_by_invocation_id_across_cpu_migration_and_arrival_reordering():
    exit_record = event("syscall_exit", "event-exit", 3, 1, 250)
    entry_record = event("syscall_enter", "event-entry", 1, 1, 100)
    result = reduce_clock_adjustment_invocation_events(
        [exit_record, entry_record], "syscall"
    )
    assert result["reduction_status"] == "consistent"
    assert result["counts"]["pairs"] == 1
    pair = result["pairs"][0]
    assert pair["entry_cpu"] == 1
    assert pair["exit_cpu"] == 3
    assert pair["entry_monotonic_ns"] == 100
    assert pair["exit_monotonic_ns"] == 250


def test_same_task_can_have_distinct_nested_invocation_ids():
    outer_enter = event("syscall_enter", "e1", 0, 1, 10, invocation_id="outer")
    inner_enter = event("syscall_enter", "e2", 1, 1, 20, invocation_id="inner")
    inner_exit = event("syscall_exit", "e3", 1, 2, 30, invocation_id="inner")
    outer_exit = event("syscall_exit", "e4", 0, 2, 40, invocation_id="outer")
    result = reduce_clock_adjustment_invocation_events(
        [outer_enter, inner_enter, inner_exit, outer_exit], "syscall"
    )
    assert result["reduction_status"] == "consistent"
    assert {p["invocation_id"] for p in result["pairs"]} == {"outer", "inner"}


def test_duplicate_event_id_is_rejected():
    enter = event("syscall_enter", "same-event", 0, 1, 10)
    exit_record = event("syscall_exit", "same-event", 0, 2, 20)
    result = reduce_clock_adjustment_invocation_events([enter, exit_record], "syscall")
    assert result["reduction_status"] == "partial"
    assert "duplicate_event_id" in [x["code"] for x in result["issues"]]


def test_duplicate_entry_for_same_invocation_is_rejected():
    records = [
        event("syscall_enter", "e1", 0, 1, 10),
        event("syscall_enter", "e2", 0, 2, 11),
        event("syscall_exit", "e3", 0, 3, 20),
    ]
    result = reduce_clock_adjustment_invocation_events(records, "syscall")
    assert result["reduction_status"] == "partial"
    assert "duplicate_invocation_side" in [x["code"] for x in result["issues"]]


def test_unmatched_invocation_and_time_reversal_fail_closed():
    missing_exit = event("syscall_enter", "e1", 0, 1, 10)
    unmatched = reduce_clock_adjustment_invocation_events([missing_exit], "syscall")
    assert unmatched["reduction_status"] == "partial"
    assert "unmatched_invocation_side" in [x["code"] for x in unmatched["issues"]]

    reversed_pair = [
        event("syscall_enter", "e2", 0, 2, 30),
        event("syscall_exit", "e3", 0, 3, 20),
    ]
    reversed_result = reduce_clock_adjustment_invocation_events(reversed_pair, "syscall")
    assert reversed_result["reduction_status"] == "partial"
    assert "invocation_time_order_mismatch" in [
        x["code"] for x in reversed_result["issues"]
    ]


def test_handler_parent_identity_must_match_between_entry_and_exit():
    def handler(event_type, event_id, parent):
        return event(
            event_type, event_id, 0, 1 if event_type.endswith("enter") else 2,
            10 if event_type.endswith("enter") else 20,
            stream_id="handler-stream",
            stream_kind="kernel_handler",
            invocation_id="handler-inv-01",
            parent_syscall_invocation_id=parent,
            parent_context_status="matched",
            payload={"event_type": event_type, "pid_tgid": 4294967297,
                     "handler": "do_adjtimex"},
        )

    result = reduce_clock_adjustment_invocation_events(
        [handler("kernel_handler_enter", "h1", "syscall-a"),
         handler("kernel_handler_exit", "h2", "syscall-b")],
        "kernel_handler",
    )
    assert result["reduction_status"] == "partial"
    assert "invocation_parent_context_mismatch" in [
        x["code"] for x in result["issues"]
    ]


def test_schema_v1_is_not_upgraded_or_paired():
    legacy = {
        "schema_version": 1,
        "capture_session_id": "capture-01",
        "boot_id": "boot-01",
        "producer_id": "producer-01",
        "stream_id": "syscall-stream",
        "stream_kind": "syscall",
        "cpu": 0,
        "sequence": 1,
        "monotonic_ns": 10,
        "payload": {"event_type": "syscall_enter", "pid_tgid": 4294967297,
                    "syscall": "clock_adjtime"},
    }
    result = reduce_clock_adjustment_invocation_events([legacy], "syscall")
    assert result["reduction_status"] == "partial"
    assert "legacy_schema_without_invocation_identity" in [
        x["code"] for x in result["issues"]
    ]


def test_invalid_inputs_rejected():
    with pytest.raises(TypeError):
        reduce_clock_adjustment_invocation_events({}, "syscall")
    with pytest.raises(ValueError):
        reduce_clock_adjustment_invocation_events([], "unknown")


def test_invocation_pair_preserves_request_modes_and_return_value():
    records = [
        event("syscall_enter", "e1", 0, 1, 10,
              payload={"event_type": "syscall_enter", "pid_tgid": 4294967297,
                       "syscall": "clock_adjtime", "raw_clockid": 0,
                       "modes_read_status": "ok", "raw_modes": 1}),
        event("syscall_exit", "e2", 1, 1, 20,
              payload={"event_type": "syscall_exit", "pid_tgid": 4294967297,
                       "syscall": "clock_adjtime", "return_value": -14}),
    ]
    result = reduce_clock_adjustment_invocation_events(records, "syscall")
    assert result["reduction_status"] == "consistent"
    pair = result["pairs"][0]
    assert pair["raw_clockid"] == 0
    assert pair["raw_modes"] == 1
    assert pair["mode_decode"] is not None
    assert pair["raw_return_value"] == -14
    assert pair["return_interpretation"] == "negative_efault_copy_stage_ambiguous"


def test_event_id_uniqueness_is_checked_across_streams():
    from robodog_gnss_time.linux_clock_adjustment_invocation_reducer import (
        audit_clock_adjustment_event_id_uniqueness,
    )
    syscall = event("syscall_enter", "shared", 0, 1, 10)
    handler = event(
        "kernel_handler_enter", "shared", 0, 1, 11,
        stream_id="handler-stream",
        stream_kind="kernel_handler",
        invocation_id="handler-inv",
        parent_syscall_invocation_id="syscall-inv-01",
        parent_context_status="matched",
        payload={"event_type": "kernel_handler_enter",
                 "pid_tgid": 4294967297, "handler": "do_adjtimex",
                 "raw_modes": 0},
    )
    result = audit_clock_adjustment_event_id_uniqueness([[syscall], [handler]])
    assert result["audit_status"] == "partial"
    assert "duplicate_event_id_across_streams" in [
        x["code"] for x in result["issues"]
    ]
