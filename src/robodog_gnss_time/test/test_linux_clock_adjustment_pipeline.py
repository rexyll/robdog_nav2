import pytest

from robodog_gnss_time.linux_clock_adjustment_pipeline import (
    audit_clock_adjustment_userspace_pipeline,
    build_clock_adjustment_capture_report,
)


def snapshot(**overrides):
    value = {
        "capture_session_id": "capture-1",
        "boot_id": "boot-a",
        "consumer_records_read": 0,
        "decode_successes": 0,
        "decode_failures": 0,
        "queue_enqueued": 0,
        "queue_admission_drops": 0,
        "writer_records_written": 0,
        "writer_write_failures": 0,
        "snapshot_after_writer_close": True,
        "final_queue_depth": 0,
        "final_inflight_writer_records": 0,
    }
    value.update(overrides)
    return value


def test_pipeline_all_stages_reconcile():
    initial = snapshot()
    final = snapshot(
        consumer_records_read=10,
        decode_successes=9,
        decode_failures=1,
        queue_enqueued=8,
        queue_admission_drops=1,
        writer_records_written=7,
        writer_write_failures=1,
    )
    result = audit_clock_adjustment_userspace_pipeline(initial, final)
    assert result["pipeline_status"] == "consistent"
    assert result["deltas"]["consumer_records_read"] == 10


@pytest.mark.parametrize(
    "field,initial,final",
    [
        ("read_decode", 10, {"consumer_records_read": 10, "decode_successes": 9, "decode_failures": 0}),
        ("decode_queue", 9, {"decode_successes": 9, "queue_enqueued": 7, "queue_admission_drops": 1}),
        ("queue_writer", 8, {"queue_enqueued": 8, "writer_records_written": 6, "writer_write_failures": 1}),
    ],
)
def test_each_stage_requires_explicit_accounting(field, initial, final):
    start = snapshot()
    end = snapshot(**final)
    result = audit_clock_adjustment_userspace_pipeline(start, end)
    assert result["pipeline_status"] == "partial"
    assert any(item.get("stage") == field for item in result["issues"])


def test_queue_admission_drop_and_writer_failure_are_not_conflated():
    start = snapshot()
    end = snapshot(
        consumer_records_read=4, decode_successes=4,
        queue_enqueued=3, queue_admission_drops=1,
        writer_records_written=2, writer_write_failures=1,
    )
    result = audit_clock_adjustment_userspace_pipeline(start, end)
    assert result["pipeline_status"] == "consistent"
    assert result["deltas"]["queue_admission_drops"] == 1
    assert result["deltas"]["writer_write_failures"] == 1


def test_identity_change_and_counter_regression_fail_closed():
    start = snapshot(consumer_records_read=5)
    end = snapshot(boot_id="boot-b", consumer_records_read=4)
    result = audit_clock_adjustment_userspace_pipeline(start, end)
    codes = [item["code"] for item in result["issues"]]
    assert "pipeline_identity_mismatch" in codes
    assert "pipeline_counter_regression" in codes


@pytest.mark.parametrize("field", ["final_queue_depth", "final_inflight_writer_records"])
def test_outstanding_pipeline_work_fails_closed(field):
    result = audit_clock_adjustment_userspace_pipeline(
        snapshot(), snapshot(**{field: 1})
    )
    assert result["pipeline_status"] == "partial"
    assert "pipeline_work_remaining" in [item["code"] for item in result["issues"]]


def test_final_snapshot_must_follow_writer_close():
    result = audit_clock_adjustment_userspace_pipeline(
        snapshot(), snapshot(snapshot_after_writer_close=False)
    )
    assert "final_snapshot_not_after_writer_close" in [x["code"] for x in result["issues"]]


def test_invalid_inputs_rejected():
    with pytest.raises(TypeError):
        audit_clock_adjustment_userspace_pipeline([], snapshot())


def test_unified_report_keeps_independent_gates_and_fails_closed():
    report = build_clock_adjustment_capture_report(
        [], [], [], {"per_cpu": []}, {"per_cpu": []}, {},
        snapshot(), snapshot(), None,
    )
    assert report["capture_report_status"] == "partial"
    assert set(report["gates"]) == {
        "producer_integrity", "userspace_pipeline",
        "producer_to_pipeline_record_count", "producer_pipeline_boot_identity",
        "syscall_reduction", "handler_reduction", "syscall_handler_join",
        "capture_lifecycle",
    }



def test_unified_report_cross_checks_producer_receive_count_and_boot_identity():
    producer_snapshot = {"per_cpu": [{
        "producer_id": "clock-step", "boot_id": "boot-a", "cpu": 0,
        "sequence_allocated": 0, "ringbuf_reserve_failures": 0,
        "ringbuf_submit_successes": 0,
    }]}
    drain_snapshot = {
        "stop_requested": True, "producer_quiesced": True,
        "ring_buffer_drained": True, "consumer_queue_drained": True,
        "writer_flushed": True, "writer_closed": True,
        "final_snapshot_after_drain": True, "queue_depth": 0,
        "inflight_callbacks": 0, "pending_syscalls": 0, "pending_handlers": 0,
    }
    report = build_clock_adjustment_capture_report(
        [], [], [], producer_snapshot, producer_snapshot, drain_snapshot,
        snapshot(), snapshot(
            boot_id="boot-a", consumer_records_read=1,
            decode_failures=1,
        ), None,
    )
    assert report["gates"]["producer_integrity"] is True
    assert report["gates"]["userspace_pipeline"] is True
    assert report["gates"]["producer_to_pipeline_record_count"] is False
    assert report["gates"]["producer_pipeline_boot_identity"] is True
    assert report["capture_report_status"] == "partial"
    assert "producer_received_vs_pipeline_read_mismatch" in [
        item["code"] for item in report["issues"]
    ]



def test_unified_report_requires_lifecycle_gate():
    report = build_clock_adjustment_capture_report(
        [], [], [], {"per_cpu": []}, {"per_cpu": []}, {},
        snapshot(), snapshot(), None,
    )
    assert report["gates"]["capture_lifecycle"] is False
    assert report["capture_lifecycle"]["lifecycle_status"] == "partial"
    assert report["capture_report_status"] == "partial"


def invocation_event(kind, event_type, event_id, invocation_id, timestamp,
                     sequence, parent_id=None):
    payload = {"event_type": event_type, "pid_tgid": 4294967297}
    if kind == "syscall":
        payload["syscall"] = "clock_adjtime"
        if event_type == "syscall_enter":
            payload["modes_read_status"] = "not_requested"
        else:
            payload["return_value"] = 0
    else:
        payload["handler"] = "do_clock_adjtime"
        if event_type == "kernel_handler_enter":
            payload.update({"raw_modes": 0, "raw_clockid": 0})
        else:
            payload["return_value"] = 0
    record = {
        "schema_version": 2,
        "capture_session_id": "capture-1",
        "boot_id": "boot-a",
        "producer_id": "producer",
        "stream_id": "syscall-stream" if kind == "syscall" else "handler-stream",
        "stream_kind": kind,
        "cpu": 0,
        "sequence": sequence,
        "monotonic_ns": timestamp,
        "task_instance_id": "task-1",
        "event_id": event_id,
        "invocation_id": invocation_id,
        "payload": payload,
    }
    if kind == "kernel_handler":
        record["parent_context_status"] = "matched"
        record["parent_syscall_invocation_id"] = parent_id
    return record


def test_unified_report_integrates_schema_v2_invocation_and_parent_join_gates():
    syscalls = [
        invocation_event("syscall", "syscall_enter", "s-enter", "sys-1", 100, 1),
        invocation_event("syscall", "syscall_exit", "s-exit", "sys-1", 400, 2),
    ]
    handlers = [
        invocation_event("kernel_handler", "kernel_handler_enter", "h-enter",
                         "handler-1", 150, 1, "sys-1"),
        invocation_event("kernel_handler", "kernel_handler_exit", "h-exit",
                         "handler-1", 350, 2, "sys-1"),
    ]
    report = build_clock_adjustment_capture_report(
        [], [], [], {"per_cpu": []}, {"per_cpu": []}, {},
        snapshot(), snapshot(), None,
        syscall_envelopes=syscalls, handler_envelopes=handlers,
    )
    assert report["invocation_identity_mode"] == "schema_v2_required"
    assert report["gates"]["syscall_invocation_reduction"] is True
    assert report["gates"]["handler_invocation_reduction"] is True
    assert report["gates"]["invocation_parent_join"] is True
    assert report["invocation_parent_join"]["counts"]["joined_pairs"] == 1
    assert report["capture_report_status"] == "partial"


def test_unified_report_invocation_gate_fails_on_parent_mismatch():
    syscalls = [
        invocation_event("syscall", "syscall_enter", "s-enter", "sys-1", 100, 1),
        invocation_event("syscall", "syscall_exit", "s-exit", "sys-1", 400, 2),
    ]
    handlers = [
        invocation_event("kernel_handler", "kernel_handler_enter", "h-enter",
                         "handler-1", 150, 1, "wrong-parent"),
        invocation_event("kernel_handler", "kernel_handler_exit", "h-exit",
                         "handler-1", 350, 2, "wrong-parent"),
    ]
    report = build_clock_adjustment_capture_report(
        [], [], [], {"per_cpu": []}, {"per_cpu": []}, {},
        snapshot(), snapshot(), None,
        syscall_envelopes=syscalls, handler_envelopes=handlers,
    )
    assert report["gates"]["syscall_invocation_reduction"] is True
    assert report["gates"]["handler_invocation_reduction"] is True
    assert report["gates"]["invocation_parent_join"] is False
    assert report["capture_report_status"] == "partial"


def test_unified_report_rejects_cross_stream_event_id_collision():
    syscalls = [
        invocation_event("syscall", "syscall_enter", "shared", "sys-1", 100, 1),
        invocation_event("syscall", "syscall_exit", "s-exit", "sys-1", 400, 2),
    ]
    handlers = [
        invocation_event("kernel_handler", "kernel_handler_enter", "shared",
                         "handler-1", 150, 1, "sys-1"),
        invocation_event("kernel_handler", "kernel_handler_exit", "h-exit",
                         "handler-1", 350, 2, "sys-1"),
    ]
    report = build_clock_adjustment_capture_report(
        [], [], [], {"per_cpu": []}, {"per_cpu": []}, {},
        snapshot(), snapshot(), None,
        syscall_envelopes=syscalls, handler_envelopes=handlers,
    )
    assert report["gates"]["syscall_invocation_reduction"] is True
    assert report["gates"]["handler_invocation_reduction"] is True
    assert report["gates"]["invocation_parent_join"] is True
    assert report["gates"]["event_id_uniqueness"] is False
    assert report["capture_report_status"] == "partial"


def producer_global_snapshot(**overrides):
    counters = {
        "event_id_allocation_attempts": 0, "event_id_allocation_successes": 0,
        "event_id_allocation_failures": 0,
        "task_instance_id_allocation_attempts": 0,
        "task_instance_id_allocation_successes": 0,
        "task_instance_id_allocation_failures": 0,
        "task_context_admission_attempts": 0, "task_context_admitted": 0,
        "task_context_capacity_rejected": 0,
        "task_context_create_attempts": 0, "task_context_create_successes": 0,
        "task_context_create_failures": 0,
        "task_context_init_attempts": 0, "task_context_init_successes": 0,
        "task_context_init_failures": 0,
        "syscall_invocation_allocation_attempts": 0,
        "syscall_invocation_allocation_successes": 0,
        "syscall_invocation_allocation_failures": 0,
        "handler_invocation_allocation_attempts": 0,
        "handler_invocation_allocation_successes": 0,
        "handler_invocation_allocation_failures": 0,
        "cpu_sequence_allocation_failures": 0,
        "syscall_context_stack_overflow": 0, "syscall_context_pop_mismatch": 0,
        "handler_context_stack_overflow": 0, "handler_context_pop_mismatch": 0,
        "handler_parent_context_unavailable": 0, "task_exit_observed": 0,
        "task_exit_with_pending_syscalls": 0, "task_exit_with_pending_handlers": 0,
    }
    value = {
        "capture_session_id": "capture-1", "boot_id": "boot-a",
        "snapshot_after_producer_detach": True,
        "counters": counters,
        "gauges": {
            "active_syscall_invocations": 0,
            "active_handler_invocations": 0,
            "inflight_bpf_callbacks": 0,
            "task_context_reservations": 2,
        },
    }
    value.update(overrides)
    return value


def test_unified_report_exposes_global_producer_runtime_counter_gate():
    cpu = {"producer_id": "syscall-producer", "boot_id": "boot-a", "cpu": 0,
           "sequence_allocated": 0, "ringbuf_reserve_failures": 0,
           "ringbuf_submit_successes": 0}
    initial = {"per_cpu": [cpu], "global": producer_global_snapshot()}
    final = {"per_cpu": [cpu], "global": producer_global_snapshot()}
    drain = {
        "stop_requested": True, "producer_quiesced": True,
        "ring_buffer_drained": True, "consumer_queue_drained": True,
        "writer_flushed": True, "writer_closed": True,
        "final_snapshot_after_drain": True, "queue_depth": 0,
        "inflight_callbacks": 0, "pending_syscalls": 0, "pending_handlers": 0,
    }
    report = build_clock_adjustment_capture_report(
        [], [], [], initial, final, drain, snapshot(), snapshot(), None,
    )
    assert report["gates"]["producer_runtime_counters"] is True
    assert report["producer_runtime_counters"]["runtime_status"] == "consistent"
