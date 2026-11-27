from robodog_gnss_time.linux_clock_adjustment_invocation_reducer import (
    audit_clock_adjustment_event_id_uniqueness,
    reduce_clock_adjustment_invocation_events,
)
from robodog_gnss_time.linux_clock_adjustment_joiner import (
    correlate_clock_adjustment_invocation_observations,
)
from robodog_gnss_time.linux_clock_adjustment_producer_state import (
    ClockAdjustmentProducerState,
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
