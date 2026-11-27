import pytest
from robodog_gnss_time.linux_clock_adjustment_capture_integrity import audit_clock_adjustment_capture_integrity

def snap(seq, failed, submitted):
    return {"per_cpu":[{"producer_id":"clock-step","boot_id":"boot-a","cpu":0,
        "sequence_allocated":seq,"ringbuf_reserve_failures":failed,
        "ringbuf_submit_successes":submitted}]}

def drain(**overrides):
    value={"stop_requested":True,"producer_quiesced":True,"ring_buffer_drained":True,
      "consumer_queue_drained":True,"writer_flushed":True,"writer_closed":True,
      "final_snapshot_after_drain":True,"queue_depth":0,"inflight_callbacks":0,
      "pending_syscalls":0,"pending_handlers":0}
    value.update(overrides)
    return value

def event(seq):
    return {"producer_id":"clock-step","boot_id":"boot-a","cpu":0,"sequence":seq}

def audit(events=None, initial=None, final=None, shutdown=None):
    return audit_clock_adjustment_capture_integrity(
      events if events is not None else [event(1),event(2)],
      initial if initial is not None else snap(0,0,0),
      final if final is not None else snap(2,0,2),
      shutdown if shutdown is not None else drain())

def test_clean_stream_and_complete_drain():
    result=audit()
    assert result["integrity_status"]=="complete"
    assert result["counts"]["received_records"]==2

def test_reserve_failure_is_accounted_by_sequence_gap():
    result=audit([event(1),event(3)],snap(0,0,0),snap(3,1,2))
    assert result["integrity_status"]=="complete"
    assert result["per_cpu"][0]["missing_sequence_ranges"]==[{"first":2,"last":2}]
    assert result["per_cpu"][0]["missing_sequence_count"]==1

def test_unexplained_sequence_gap_is_partial():
    result=audit([event(1),event(3)],snap(0,0,0),snap(3,0,3))
    assert result["integrity_status"]=="partial"
    assert "sequence_gaps_vs_reserve_failures_mismatch" in [x["code"] for x in result["issues"]]

def test_submitted_record_missing_from_consumer_is_partial():
    result=audit([event(1)],snap(0,0,0),snap(2,0,2))
    assert "submitted_vs_received_count_mismatch" in [x["code"] for x in result["issues"]]

def test_counter_equation_must_reconcile():
    result=audit([event(1)],snap(0,0,0),snap(2,0,1))
    assert "producer_counter_equation_mismatch" in [x["code"] for x in result["issues"]]

def test_sequence_outside_snapshot_window_is_partial():
    result=audit([event(1),event(9)],snap(0,0,0),snap(2,0,2))
    assert "event_sequence_outside_snapshot_window" in [x["code"] for x in result["issues"]]

def test_duplicate_sequence_is_partial():
    result=audit([event(1),event(1)],snap(0,0,0),snap(2,0,2))
    assert "duplicate_event_sequence" in [x["code"] for x in result["issues"]]

def test_counter_regression_is_partial():
    result=audit([event(1)],snap(2,0,1),snap(1,0,1))
    assert "counter_regression" in [x["code"] for x in result["issues"]]

def test_each_cpu_has_independent_sequence_domain():
    initial={"per_cpu":[
      {"producer_id":"clock-step","boot_id":"boot-a","cpu":0,"sequence_allocated":0,"ringbuf_reserve_failures":0,"ringbuf_submit_successes":0},
      {"producer_id":"clock-step","boot_id":"boot-a","cpu":1,"sequence_allocated":0,"ringbuf_reserve_failures":0,"ringbuf_submit_successes":0}]}
    final={"per_cpu":[
      {"producer_id":"clock-step","boot_id":"boot-a","cpu":0,"sequence_allocated":1,"ringbuf_reserve_failures":0,"ringbuf_submit_successes":1},
      {"producer_id":"clock-step","boot_id":"boot-a","cpu":1,"sequence_allocated":1,"ringbuf_reserve_failures":0,"ringbuf_submit_successes":1}]}
    result=audit([event(1),{**event(1),"cpu":1}],initial,final)
    assert result["integrity_status"]=="complete"
    assert result["counts"]["cpu_streams_audited"]==2

@pytest.mark.parametrize("field",["stop_requested","producer_quiesced","ring_buffer_drained",
 "consumer_queue_drained","writer_flushed","writer_closed","final_snapshot_after_drain"])
def test_missing_shutdown_phase_fails_closed(field):
    result=audit(shutdown=drain(**{field:False}))
    assert result["integrity_status"]=="partial"
    assert "shutdown_drain_incomplete" in [x["code"] for x in result["issues"]]

@pytest.mark.parametrize("field",["queue_depth","inflight_callbacks","pending_syscalls","pending_handlers"])
def test_nonzero_work_at_shutdown_fails_closed(field):
    result=audit(shutdown=drain(**{field:1}))
    assert result["integrity_status"]=="partial"
    assert "shutdown_work_remaining" in [x["code"] for x in result["issues"]]

def test_malformed_envelope_is_not_echoed():
    result=audit([{"producer_id":object(),"secret_pointer":"0xdead"}],snap(0,0,0),snap(0,0,0))
    assert result["integrity_status"]=="partial"
    assert "secret_pointer" not in str(result)

def test_invalid_input_shapes_rejected():
    with pytest.raises(TypeError):
        audit_clock_adjustment_capture_integrity("not-list",snap(0,0,0),snap(0,0,0),drain())
    with pytest.raises(TypeError):
        audit_clock_adjustment_capture_integrity([],snap(0,0,0),snap(0,0,0),None)

def test_missing_cpu_snapshot_is_partial():
    result=audit([event(1)],snap(0,0,0),{"per_cpu":[]})
    assert "missing_counter_snapshot" in [x["code"] for x in result["issues"]]


def test_huge_sequence_gap_is_reported_compactly_without_expanding_range():
    end=(1<<63)
    result=audit([event(1)],snap(0,0,0),snap(end, end-1, 1))
    assert result["integrity_status"]=="complete"
    assert result["per_cpu"][0]["missing_sequence_count"]==end-1
    assert len(result["per_cpu"][0]["missing_sequence_ranges"])==1


def runtime_counters(**overrides):
    names = [
        "event_id_allocation_attempts", "event_id_allocation_successes",
        "event_id_allocation_failures",
        "task_instance_id_allocation_attempts", "task_instance_id_allocation_successes",
        "task_instance_id_allocation_failures",
        "task_context_admission_attempts", "task_context_admitted",
        "task_context_capacity_rejected",
        "task_context_create_attempts", "task_context_create_successes",
        "task_context_create_failures",
        "task_context_init_attempts", "task_context_init_successes",
        "task_context_init_failures",
        "syscall_invocation_allocation_attempts",
        "syscall_invocation_allocation_successes",
        "syscall_invocation_allocation_failures",
        "handler_invocation_allocation_attempts",
        "handler_invocation_allocation_successes",
        "handler_invocation_allocation_failures",
        "cpu_sequence_allocation_failures",
        "syscall_context_stack_overflow", "syscall_context_pop_mismatch",
        "handler_context_stack_overflow", "handler_context_pop_mismatch",
        "handler_parent_context_unavailable", "task_exit_observed",
        "task_exit_with_pending_syscalls", "task_exit_with_pending_handlers",
    ]
    result = {name: 0 for name in names}
    result.update(overrides)
    return result


def runtime_snapshot(**overrides):
    value = {
        "capture_session_id": "capture-01",
        "boot_id": "boot-a",
        "snapshot_after_producer_detach": True,
        "counters": runtime_counters(),
        "gauges": {
            "active_syscall_invocations": 0,
            "active_handler_invocations": 0,
            "inflight_bpf_callbacks": 0,
            "task_context_reservations": 0,
        },
    }
    value.update(overrides)
    return value


def test_global_runtime_counters_reconcile_and_idle_task_storage_may_remain():
    initial = runtime_snapshot()
    final_counters = runtime_counters(
        task_context_admission_attempts=3, task_context_admitted=3,
        task_context_create_attempts=3, task_context_create_successes=3,
        task_context_init_attempts=3, task_context_init_successes=3,
        task_instance_id_allocation_attempts=3,
        task_instance_id_allocation_successes=3,
    )
    final = runtime_snapshot(counters=final_counters, gauges={
        "active_syscall_invocations": 0,
        "active_handler_invocations": 0,
        "inflight_bpf_callbacks": 0,
        "task_context_reservations": 3,
    })
    result = audit_clock_adjustment_capture_integrity(
        [event(1), event(2)],
        {"per_cpu": snap(0, 0, 0)["per_cpu"], "global": initial},
        {"per_cpu": snap(2, 0, 2)["per_cpu"], "global": final},
        drain(),
    )
    assert result["integrity_status"] == "complete"
    assert result["global_runtime"]["runtime_status"] == "consistent"
    assert result["global_runtime"]["task_context_reservations_zero_required"] is False


def test_global_runtime_counter_equation_and_failure_counters_fail_closed():
    initial = runtime_snapshot()
    counters = runtime_counters(
        event_id_allocation_attempts=2,
        event_id_allocation_successes=1,
        event_id_allocation_failures=1,
    )
    final = runtime_snapshot(counters=counters)
    result = audit_clock_adjustment_capture_integrity(
        [event(1), event(2)],
        {"per_cpu": snap(0, 0, 0)["per_cpu"], "global": initial},
        {"per_cpu": snap(2, 0, 2)["per_cpu"], "global": final},
        drain(),
    )
    assert result["integrity_status"] == "partial"
    codes = [x["code"] for x in result["global_runtime"]["issues"]]
    assert "producer_runtime_failure_observed" in codes
    assert any(x.get("counter") == "event_id_allocation_failures"
               for x in result["global_runtime"]["issues"])


def test_global_runtime_requires_zero_active_invocations_but_not_zero_task_contexts():
    initial = runtime_snapshot()
    final = runtime_snapshot(gauges={
        "active_syscall_invocations": 1,
        "active_handler_invocations": 0,
        "inflight_bpf_callbacks": 0,
        "task_context_reservations": 1,
    })
    result = audit_clock_adjustment_capture_integrity(
        [event(1), event(2)],
        {"per_cpu": snap(0, 0, 0)["per_cpu"], "global": initial},
        {"per_cpu": snap(2, 0, 2)["per_cpu"], "global": final},
        drain(),
    )
    assert result["integrity_status"] == "partial"
    assert "producer_runtime_work_remaining" in [
        x["code"] for x in result["global_runtime"]["issues"]
    ]


def test_global_runtime_snapshot_identity_and_detach_are_required():
    initial = runtime_snapshot()
    final = runtime_snapshot(boot_id="other", snapshot_after_producer_detach=False)
    result = audit_clock_adjustment_capture_integrity(
        [event(1), event(2)],
        {"per_cpu": snap(0, 0, 0)["per_cpu"], "global": initial},
        {"per_cpu": snap(2, 0, 2)["per_cpu"], "global": final},
        drain(),
    )
    assert result["integrity_status"] == "partial"
    codes = [x["code"] for x in result["global_runtime"]["issues"]]
    assert "global_runtime_identity_mismatch" in codes
    assert "global_snapshot_not_after_producer_detach" in codes


def test_global_runtime_snapshot_pair_is_required_when_either_is_present():
    initial = {"per_cpu": snap(0, 0, 0)["per_cpu"], "global": runtime_snapshot()}
    final = {"per_cpu": snap(2, 0, 2)["per_cpu"]}
    result = audit_clock_adjustment_capture_integrity(
        [event(1), event(2)], initial, final, drain()
    )
    assert result["integrity_status"] == "partial"
    assert "missing_global_runtime_snapshot" in [
        x["code"] for x in result["global_runtime"]["issues"]
    ]
