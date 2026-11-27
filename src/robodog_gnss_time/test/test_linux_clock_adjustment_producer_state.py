from robodog_gnss_time.linux_clock_adjustment_capture_integrity import (
    audit_clock_adjustment_producer_runtime_counters,
)
from robodog_gnss_time.linux_clock_adjustment_pipeline import (
    build_clock_adjustment_capture_report,
)
from robodog_gnss_time.linux_clock_adjustment_invocation_reducer import (
    audit_clock_adjustment_event_id_uniqueness,
    reduce_clock_adjustment_invocation_events,
)
from robodog_gnss_time.linux_clock_adjustment_joiner import (
    correlate_clock_adjustment_invocation_observations,
)
from robodog_gnss_time.linux_clock_adjustment_producer_state import (
    ClockAdjustmentProducerState,
    ClockAdjustmentProducerSessionAllocator,
)


def test_nested_syscall_state_machine_pairs_inner_then_outer_by_id():
    state = ClockAdjustmentProducerState(max_syscall_depth=2)
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    state.syscall_enter("adjtimex", 1, 110)
    state.syscall_exit("adjtimex", 1, 120, 0)
    state.syscall_exit("clock_adjtime", 0, 130, 0)
    report = reduce_clock_adjustment_invocation_events(state.emitted, "syscall")
    assert report["reduction_status"] == "consistent"
    assert report["counts"]["pairs"] == 2
    assert state.active_syscall_invocations == 0
    assert state.per_cpu_snapshot()["per_cpu"][0]["sequence_allocated"] >= 1


def test_syscall_overflow_does_not_overwrite_or_pop_outer_frame():
    state = ClockAdjustmentProducerState(max_syscall_depth=1)
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    assert state.syscall_enter("adjtimex", 0, 110) is None
    assert state.syscall_overflow_depth == 1
    assert state.syscall_exit("adjtimex", 0, 120, 0) is None
    state.syscall_exit("clock_adjtime", 0, 130, 0)
    report = reduce_clock_adjustment_invocation_events(state.emitted, "syscall")
    assert report["reduction_status"] == "consistent"
    assert report["counts"]["pairs"] == 1
    assert state.counters["syscall_context_stack_overflow"] == 1
    assert state.active_syscall_invocations == 0


def test_failed_syscall_entry_reservation_suppresses_exit_and_keeps_sequence_gap():
    state = ClockAdjustmentProducerState()
    assert state.syscall_enter(
        "clock_adjtime", 0, 100, raw_clockid=0, reserve_ok=False
    ) is None
    assert state.syscall_exit("clock_adjtime", 0, 120, 0) is None
    assert state.emitted == []
    counters = state.per_cpu_snapshot()["per_cpu"][0]
    assert counters["sequence_allocated"] == 1
    assert counters["ringbuf_reserve_failures"] == 1
    assert counters["ringbuf_submit_successes"] == 0
    assert state.active_syscall_invocations == 0


def test_event_id_allocation_failure_suppresses_both_entry_and_exit():
    state = ClockAdjustmentProducerState()
    assert state.syscall_enter(
        "clock_adjtime", 0, 100, raw_clockid=0, event_id_ok=False
    ) is None
    assert state.syscall_exit("clock_adjtime", 0, 120, 0) is None
    assert state.emitted == []
    assert state.counters["event_id_allocation_attempts"] == 1
    assert state.counters["event_id_allocation_failures"] == 1
    assert state.per_cpu_snapshot()["per_cpu"] == []
    assert state.active_syscall_invocations == 0


def test_handler_copies_exact_parent_syscall_invocation_id():
    state = ClockAdjustmentProducerState()
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    state.handler_enter("do_clock_adjtime", 0, 110, raw_modes=1, raw_clockid=0)
    state.handler_exit("do_clock_adjtime", 1, 120, 0)
    state.syscall_exit("clock_adjtime", 0, 130, 0)
    syscalls = reduce_clock_adjustment_invocation_events(
        [x for x in state.emitted if x["stream_kind"] == "syscall"], "syscall"
    )
    handlers = reduce_clock_adjustment_invocation_events(
        [x for x in state.emitted if x["stream_kind"] == "kernel_handler"],
        "kernel_handler",
    )
    joined = correlate_clock_adjustment_invocation_observations(syscalls, handlers)
    assert syscalls["reduction_status"] == "consistent"
    assert handlers["reduction_status"] == "consistent"
    assert joined["reduction_status"] == "consistent"
    assert joined["counts"]["joined_pairs"] == 1
    assert joined["joins"][0]["syscall_invocation_id"] == state.emitted[0]["invocation_id"]


def test_nested_handler_overflow_does_not_consume_outer_handler_frame():
    state = ClockAdjustmentProducerState(max_handler_depth=1)
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    state.handler_enter("do_clock_adjtime", 0, 110, raw_modes=1, raw_clockid=0)
    assert state.handler_enter("do_adjtimex", 0, 115, raw_modes=2) is None
    assert state.handler_exit("do_adjtimex", 0, 120, 0) is None
    state.handler_exit("do_clock_adjtime", 0, 125, 0)
    state.syscall_exit("clock_adjtime", 0, 130, 0)
    handlers = reduce_clock_adjustment_invocation_events(
        [x for x in state.emitted if x["stream_kind"] == "kernel_handler"],
        "kernel_handler",
    )
    assert handlers["reduction_status"] == "consistent"
    assert handlers["counts"]["pairs"] == 1
    assert state.counters["handler_context_stack_overflow"] == 1
    assert state.active_handler_invocations == 0


def test_handler_without_parent_keeps_explicit_no_active_context():
    state = ClockAdjustmentProducerState()
    state.handler_enter("do_adjtimex", 0, 100, raw_modes=1)
    state.handler_exit("do_adjtimex", 0, 110, 0)
    handlers = reduce_clock_adjustment_invocation_events(state.emitted, "kernel_handler")
    assert handlers["reduction_status"] == "consistent"
    assert handlers["pairs"][0]["parent_context_status"] == "no_active_context"
    assert handlers["pairs"][0]["parent_syscall_invocation_id"] is None
    assert state.counters["handler_parent_context_unavailable"] == 1


def test_task_exit_accounts_abandoned_frames_and_releases_context_reservation():
    state = ClockAdjustmentProducerState()
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    result = state.task_exit()
    assert result["pending_syscalls"] == 1
    assert state.counters["task_exit_with_pending_syscalls"] == 1
    assert state.active_syscall_invocations == 0
    assert state.task_context_reservations == 0
    state.detach()
    assert state.global_snapshot()["snapshot_after_producer_detach"] is True
    assert state.global_snapshot()["gauges"]["inflight_bpf_callbacks"] == 0


def test_event_ids_are_unique_across_syscall_and_handler_streams():
    state = ClockAdjustmentProducerState()
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    state.handler_enter("do_clock_adjtime", 0, 110, raw_modes=1, raw_clockid=0)
    state.handler_exit("do_clock_adjtime", 0, 120, 0)
    state.syscall_exit("clock_adjtime", 0, 130, 0)
    audit = audit_clock_adjustment_event_id_uniqueness([
        [x for x in state.emitted if x["stream_kind"] == "syscall"],
        [x for x in state.emitted if x["stream_kind"] == "kernel_handler"],
    ])
    assert audit["audit_status"] == "consistent"
    ids = [x["event_id"] for x in state.emitted]
    assert len(ids) == len(set(ids))


def test_invalid_state_configuration_and_double_exit_are_rejected():
    import pytest
    with pytest.raises(ValueError):
        ClockAdjustmentProducerState(max_syscall_depth=0)
    state = ClockAdjustmentProducerState()
    state.task_exit()
    with pytest.raises(RuntimeError):
        state.task_exit()


def test_detach_is_blocked_while_invocation_is_active():
    import pytest
    state = ClockAdjustmentProducerState()
    state.syscall_enter("clock_adjtime", 0, 100, raw_clockid=0)
    with pytest.raises(RuntimeError):
        state.detach()
    state.syscall_exit("clock_adjtime", 0, 120, 0)
    state.detach()
    assert state.global_snapshot()["snapshot_after_producer_detach"] is True


def test_session_allocator_keeps_event_ids_global_and_invocation_ids_task_scoped():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=2)
    task_a = session.create_task_context()
    task_b = session.create_task_context()
    assert task_a != task_b
    syscall_id = session.allocate_invocation_id(task_a, kind="syscall")
    handler_id = session.allocate_invocation_id(task_b, kind="handler")
    assert syscall_id == handler_id == "0000000000000001"
    event_ids = [
        session.allocate_event_id(),
        session.allocate_event_id(),
        session.allocate_event_id(),
    ]
    assert len(event_ids) == len(set(event_ids))
    session.release_task_context(task_a)
    session.release_task_context(task_b)
    session.detach()
    snapshot = session.global_snapshot()
    assert snapshot["snapshot_after_producer_detach"] is True
    assert snapshot["gauges"]["task_context_reservations"] == 0
    assert snapshot["counters"]["event_id_allocation_successes"] == 3
    assert snapshot["counters"]["task_instance_id_allocation_successes"] == 2


def test_session_allocator_enforces_capacity_without_reusing_task_ids():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=1)
    task_a = session.create_task_context()
    assert session.create_task_context() is None
    assert session.counters["task_context_capacity_rejected"] == 1
    session.release_task_context(task_a)
    task_b = session.create_task_context()
    assert task_b != task_a
    session.release_task_context(task_b)
    assert session.task_context_reservations == 0


def test_session_allocator_reconciles_create_and_task_id_failures():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=2)
    assert session.create_task_context(create_ok=False) is None
    assert session.create_task_context(task_id_ok=False) is None
    assert session.task_context_reservations == 0
    assert session.counters["task_context_create_failures"] == 1
    assert session.counters["task_instance_id_allocation_failures"] == 1
    assert session.counters["task_context_init_failures"] == 1


def test_session_allocator_cas_contention_and_exhaustion_fail_closed():
    session = ClockAdjustmentProducerSessionAllocator(max_cas_retries=2)
    assert session.allocate_event_id(cas_attempts=3) is None
    assert session.allocate_event_id(cas_attempts=2) == "0000000000000001"
    assert session.counters["event_id_allocation_failures"] == 1
    exhausted = ClockAdjustmentProducerSessionAllocator(
        start_event_id=(1 << 64) - 1
    )
    assert exhausted.allocate_event_id() is None
    assert exhausted.counters["event_id_allocation_failures"] == 1


def test_session_allocator_task_exit_with_pending_frames_releases_gauges():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=1)
    task = session.create_task_context()
    session.begin_invocation(task, "syscall")
    session.begin_invocation(task, "handler")
    session.release_task_context(task)
    session.detach()
    snapshot = session.global_snapshot()
    assert snapshot["gauges"]["active_syscall_invocations"] == 0
    assert snapshot["gauges"]["active_handler_invocations"] == 0
    assert snapshot["counters"]["task_exit_with_pending_syscalls"] == 1
    assert snapshot["counters"]["task_exit_with_pending_handlers"] == 1


def test_session_allocator_allows_idle_live_task_storage_at_detach():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=1)
    session.create_task_context()
    session.detach()
    snapshot = session.global_snapshot()
    assert snapshot["gauges"]["task_context_reservations"] == 1
    assert snapshot["snapshot_after_producer_detach"] is True


def test_session_allocator_rejects_operations_after_detach():
    import pytest
    session = ClockAdjustmentProducerSessionAllocator()
    task = session.create_task_context()
    session.detach()
    with pytest.raises(RuntimeError):
        session.allocate_event_id()
    with pytest.raises(RuntimeError):
        session.allocate_invocation_id(task)
    with pytest.raises(RuntimeError):
        session.create_task_context()


def test_session_allocator_global_snapshot_passes_runtime_audit_for_live_idle_task():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=2)
    initial = session.global_snapshot()
    task_a = session.create_task_context()
    task_b = session.create_task_context()
    session.allocate_invocation_id(task_a, kind="syscall")
    session.allocate_invocation_id(task_b, kind="handler")
    session.allocate_event_id()
    session.allocate_event_id()
    session.release_task_context(task_a)
    session.detach()
    final = session.global_snapshot()
    audit = audit_clock_adjustment_producer_runtime_counters(initial, final)
    assert audit["runtime_status"] == "consistent"
    assert audit["final_gauges"]["task_context_reservations"] == 1


def test_session_allocator_capacity_rejection_is_visible_to_runtime_audit():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=1)
    initial = session.global_snapshot()
    task = session.create_task_context()
    assert session.create_task_context() is None
    session.release_task_context(task)
    session.detach()
    final = session.global_snapshot()
    audit = audit_clock_adjustment_producer_runtime_counters(initial, final)
    assert audit["runtime_status"] == "partial"
    assert any(
        issue.get("counter") == "task_context_capacity_rejected"
        for issue in audit["issues"]
    )


def test_session_allocator_task_id_and_invocation_id_exhaustion_do_not_wrap():
    session = ClockAdjustmentProducerSessionAllocator(
        start_task_instance_seq=(1 << 64) - 1
    )
    assert session.create_task_context() is None
    assert session.task_context_reservations == 0
    assert session.counters["task_instance_id_allocation_failures"] == 1

    session2 = ClockAdjustmentProducerSessionAllocator()
    task = session2.create_task_context()
    session2.next_invocation_seq[task] = (1 << 64) - 1
    assert session2.allocate_invocation_id(task, kind="handler") is None
    assert session2.counters["handler_invocation_allocation_failures"] == 1


def test_session_allocator_blocks_gauge_mutation_after_detach():
    import pytest
    session = ClockAdjustmentProducerSessionAllocator()
    task = session.create_task_context()
    session.detach()
    with pytest.raises(RuntimeError):
        session.end_invocation(task, "syscall")
    with pytest.raises(RuntimeError):
        session.release_task_context(task)


def test_two_task_interleaving_exit_cleanup_preserves_other_tasks_active_gauge():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=2)
    task_a = session.create_task_context()
    task_b = session.create_task_context()
    session.begin_invocation(task_a, "syscall")
    session.begin_invocation(task_b, "syscall")
    session.release_task_context(task_a)
    assert session.active_syscall_invocations == 1
    session.end_invocation(task_b, "syscall")
    session.release_task_context(task_b)
    assert session.active_syscall_invocations == 0
    assert session.task_context_reservations == 0
    session.detach()
    snapshot = session.global_snapshot()
    assert snapshot["gauges"]["active_syscall_invocations"] == 0
    assert snapshot["counters"]["task_exit_with_pending_syscalls"] == 1


def test_end_to_end_multi_task_fixture_reconciles_global_and_per_cpu_gates():
    session = ClockAdjustmentProducerSessionAllocator(max_tracked_tasks=4)
    initial_global = session.global_snapshot()
    task_ids = [session.create_task_context(), session.create_task_context()]
    assert all(task_ids) and task_ids[0] != task_ids[1]

    states = [
        ClockAdjustmentProducerState(
            capture_session_id="capture-01", boot_id="boot-01",
            task_instance_seq=int(task_id, 16),
        )
        for task_id in task_ids
    ]
    for index, state in enumerate(states):
        cpu = index
        state.syscall_enter("clock_adjtime", cpu, 100 + index * 100,
                            raw_clockid=0)
        state.handler_enter("do_clock_adjtime", cpu, 110 + index * 100,
                            raw_modes=1, raw_clockid=0)
        state.handler_exit("do_clock_adjtime", cpu, 120 + index * 100, 0)
        state.syscall_exit("clock_adjtime", cpu, 130 + index * 100, 0)

    task_for_state = {
        id(state): task_id for state, task_id in zip(states, task_ids)
    }
    records = [
        (record["monotonic_ns"], state, record)
        for state in states for record in state.emitted
    ]
    records.sort(key=lambda item: item[0])
    task_invocation_ids = {task_id: {} for task_id in task_ids}
    syscall_envelopes = []
    handler_envelopes = []

    for _, state, record in records:
        task_id = task_for_state[id(state)]
        event_type = record["payload"]["event_type"]
        kind = record["stream_kind"]
        is_entry = event_type.endswith("_enter")
        if is_entry:
            allocator_kind = "syscall" if kind == "syscall" else "handler"
            invocation_id = session.allocate_invocation_id(task_id, allocator_kind)
            assert invocation_id is not None
            task_invocation_ids[task_id][record["invocation_id"]] = invocation_id
            session.begin_invocation(task_id, kind)
        record["task_instance_id"] = task_id
        record["invocation_id"] = task_invocation_ids[task_id][record["invocation_id"]]
        event_id = session.allocate_event_id()
        assert event_id is not None
        record["event_id"] = event_id
        if kind == "kernel_handler" and record.get("parent_syscall_invocation_id"):
            record["parent_syscall_invocation_id"] = task_invocation_ids[task_id][
                record["parent_syscall_invocation_id"]
            ]
        (syscall_envelopes if kind == "syscall" else handler_envelopes).append(record)
        if not is_entry:
            session.end_invocation(task_id, kind)

    for task_id in task_ids:
        session.release_task_context(task_id)
    session.detach()
    final_global = session.global_snapshot()

    per_cpu = {}
    for state in states:
        for item in state.per_cpu_snapshot()["per_cpu"]:
            key = (item["producer_id"], item["boot_id"], item["cpu"])
            per_cpu[key] = item
    initial_per_cpu = [
        {
            "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
            "sequence_allocated": 0, "ringbuf_reserve_failures": 0,
            "ringbuf_submit_successes": 0,
        }
        for key in sorted(per_cpu)
    ]
    final_per_cpu = [per_cpu[key] for key in sorted(per_cpu)]
    producer_initial = {"per_cpu": initial_per_cpu, "global": initial_global}
    producer_final = {"per_cpu": final_per_cpu, "global": final_global}
    producer_events = [record for _, _, record in records]
    count = len(producer_events)

    pipeline_initial = {
        "capture_session_id": "capture-01", "boot_id": "boot-01",
        "consumer_records_read": 0, "decode_successes": 0, "decode_failures": 0,
        "queue_enqueued": 0, "queue_admission_drops": 0,
        "writer_records_written": 0, "writer_write_failures": 0,
        "snapshot_after_writer_close": True, "final_queue_depth": 0,
        "final_inflight_writer_records": 0,
    }
    pipeline_final = dict(pipeline_initial)
    pipeline_final.update({
        "consumer_records_read": count, "decode_successes": count,
        "queue_enqueued": count, "writer_records_written": count,
    })
    drain = {
        "stop_requested": True, "producer_quiesced": True,
        "ring_buffer_drained": True, "consumer_queue_drained": True,
        "writer_flushed": True, "writer_closed": True,
        "final_snapshot_after_drain": True, "queue_depth": 0,
        "inflight_callbacks": 0, "pending_syscalls": 0, "pending_handlers": 0,
    }
    states_in_order = (
        "created", "producers_attached", "running", "stop_requested",
        "producers_quiesced", "ring_buffer_drained", "consumer_queue_drained",
        "writer_flushed", "writer_closed", "finalized",
    )
    lifecycle = [
        {"state": value, "capture_session_id": "capture-01",
         "boot_id": "boot-01", "monotonic_ns": index}
        for index, value in enumerate(states_in_order)
    ]
    lifecycle_final = {
        "capture_session_id": "capture-01", "boot_id": "boot-01",
        "snapshot_after_writer_close": True, "queue_depth": 0,
        "inflight_callbacks": 0, "pending_syscalls": 0,
        "pending_handlers": 0, "inflight_writer_records": 0,
    }

    report = build_clock_adjustment_capture_report(
        [], [], producer_events, producer_initial, producer_final, drain,
        pipeline_initial, pipeline_final, None, lifecycle, lifecycle_final,
        syscall_envelopes=syscall_envelopes,
        handler_envelopes=handler_envelopes,
    )
    assert count == 8
    assert all(report["gates"][name] for name in (
        "producer_integrity", "producer_loss_free", "producer_runtime_counters",
        "userspace_pipeline", "producer_to_pipeline_record_count",
        "producer_pipeline_boot_identity", "capture_lifecycle",
        "syscall_invocation_reduction", "handler_invocation_reduction",
        "invocation_parent_join", "event_id_uniqueness",
    ))
    assert report["capture_report_status"] == "consistent"
    assert report["producer_integrity"]["counts"]["received_records"] == count
    assert report["producer_runtime_counters"]["final_gauges"][
        "task_context_reservations"
    ] == 0
    assert report["invocation_parent_join"]["counts"]["joined_pairs"] == 2
