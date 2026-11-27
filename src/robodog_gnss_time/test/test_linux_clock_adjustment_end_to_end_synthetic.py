from robodog_gnss_time.linux_clock_adjustment_pipeline import (
    build_clock_adjustment_capture_report,
)
from robodog_gnss_time.linux_clock_adjustment_producer_state import (
    ClockAdjustmentProducerSessionAllocator,
    ClockAdjustmentProducerState,
)
from robodog_gnss_time.linux_clock_adjustment_session import (
    validate_clock_adjustment_event_envelope,
)


def _pipeline_snapshot(**overrides):
    value = {
        "capture_session_id": "capture-01",
        "boot_id": "boot-01",
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


def _lifecycle():
    names = (
        "created", "producers_attached", "running", "stop_requested",
        "producers_quiesced", "ring_buffer_drained", "consumer_queue_drained",
        "writer_flushed", "writer_closed", "finalized",
    )
    return [
        {"state": name, "capture_session_id": "capture-01",
         "boot_id": "boot-01", "monotonic_ns": index + 1}
        for index, name in enumerate(names)
    ]


def _drain():
    return {
        "stop_requested": True,
        "producer_quiesced": True,
        "ring_buffer_drained": True,
        "consumer_queue_drained": True,
        "writer_flushed": True,
        "writer_closed": True,
        "final_snapshot_after_drain": True,
        "queue_depth": 0,
        "inflight_callbacks": 0,
        "pending_syscalls": 0,
        "pending_handlers": 0,
    }


def test_end_to_end_synthetic_capture_all_report_gates_consistent():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=2)
    initial_global = session.global_snapshot()
    task_a = session.create_task_context()
    task_b = session.create_task_context()

    state_a = ClockAdjustmentProducerState(
        capture_session_id="capture-01", boot_id="boot-01",
        task_instance_seq=int(task_a, 16),
    )
    state_b = ClockAdjustmentProducerState(
        capture_session_id="capture-01", boot_id="boot-01",
        task_instance_seq=int(task_b, 16),
    )

    state_a.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    state_a.handler_enter("do_clock_adjtime", 0, 110, raw_modes=1, raw_clockid=0)
    state_a.handler_exit("do_clock_adjtime", 0, 120, 0)
    state_a.syscall_exit("clock_adjtime", 0, 130, 0)

    state_b.syscall_enter("adjtimex", 1, 105)
    state_b.syscall_exit("adjtimex", 1, 125, 0)

    # Merge the independent task-state traces into one session-wide event stream.
    records = sorted(state_a.emitted + state_b.emitted,
                     key=lambda item: item["monotonic_ns"])
    invocation_ids = {}
    per_cpu_sequence = {}
    for record in records:
        task_id = record["task_instance_id"]
        kind = record["stream_kind"]
        event_type = record["payload"]["event_type"]
        old_invocation_id = record["invocation_id"]
        invocation_key = (task_id, kind, old_invocation_id)
        if invocation_key not in invocation_ids:
            assert event_type.endswith("_enter")
            allocator_kind = "syscall" if kind == "syscall" else "handler"
            invocation_ids[invocation_key] = session.allocate_invocation_id(
                task_id, kind=allocator_kind
            )
        record["invocation_id"] = invocation_ids[invocation_key]

        if kind == "kernel_handler" and record.get("parent_syscall_invocation_id"):
            parent_key = (task_id, "syscall",
                          record["parent_syscall_invocation_id"])
            record["parent_syscall_invocation_id"] = invocation_ids[parent_key]

        record["event_id"] = session.allocate_event_id()
        record["payload"]["pid_tgid"] = (
            4294967297 if task_id == task_a else 8589934594
        )

        sequence_key = (record["producer_id"], record["cpu"])
        per_cpu_sequence[sequence_key] = per_cpu_sequence.get(sequence_key, 0) + 1
        record["sequence"] = per_cpu_sequence[sequence_key]

        if event_type.endswith("_enter"):
            session.begin_invocation(
                task_id, "syscall" if kind == "syscall" else "handler"
            )
        else:
            session.end_invocation(
                task_id, "syscall" if kind == "syscall" else "handler"
            )

    session.release_task_context(task_a)
    session.release_task_context(task_b)
    session.detach()
    final_global = session.global_snapshot()

    per_cpu_final = []
    per_cpu_initial = []
    for (producer_id, cpu), count in sorted(per_cpu_sequence.items()):
        base = {
            "producer_id": producer_id, "boot_id": "boot-01", "cpu": cpu,
        }
        per_cpu_initial.append({
            **base, "sequence_allocated": 0,
            "ringbuf_reserve_failures": 0, "ringbuf_submit_successes": 0,
        })
        per_cpu_final.append({
            **base, "sequence_allocated": count,
            "ringbuf_reserve_failures": 0, "ringbuf_submit_successes": count,
        })

    producer_initial = {
        "per_cpu": per_cpu_initial, "global": initial_global,
    }
    producer_final = {
        "per_cpu": per_cpu_final, "global": final_global,
    }

    syscall_envelopes = [x for x in records if x["stream_kind"] == "syscall"]
    handler_envelopes = [x for x in records if x["stream_kind"] == "kernel_handler"]
    syscall_events = [
        validate_clock_adjustment_event_envelope(x)["event"]
        for x in syscall_envelopes
    ]
    handler_events = [
        validate_clock_adjustment_event_envelope(x)["event"]
        for x in handler_envelopes
    ]
    producer_events = [
        {"producer_id": x["producer_id"], "boot_id": x["boot_id"],
         "cpu": x["cpu"], "sequence": x["sequence"]}
        for x in records
    ]
    pipeline_initial = _pipeline_snapshot()
    pipeline_final = _pipeline_snapshot(
        consumer_records_read=len(records),
        decode_successes=len(records),
        queue_enqueued=len(records),
        writer_records_written=len(records),
    )
    lifecycle_final = {
        "capture_session_id": "capture-01", "boot_id": "boot-01",
        "snapshot_after_writer_close": True, "queue_depth": 0,
        "inflight_callbacks": 0, "pending_syscalls": 0,
        "pending_handlers": 0, "inflight_writer_records": 0,
    }

    report = build_clock_adjustment_capture_report(
        syscall_events, handler_events, producer_events,
        producer_initial, producer_final, _drain(),
        pipeline_initial, pipeline_final, None,
        lifecycle_transitions=_lifecycle(),
        lifecycle_final_snapshot=lifecycle_final,
        syscall_envelopes=syscall_envelopes,
        handler_envelopes=handler_envelopes,
    )

    assert report["capture_report_status"] == "consistent"
    assert all(report["gates"].values())
    assert report["producer_runtime_counters"]["runtime_status"] == "consistent"
    assert report["producer_integrity"]["integrity_status"] == "complete"
    assert report["producer_integrity"]["counts"]["received_records"] == len(records)
    assert report["gates"]["producer_loss_free"] is True
    assert report["invocation_parent_join"]["counts"]["joined_pairs"] == 1
    assert report["event_id_uniqueness"]["audit_status"] == "consistent"
