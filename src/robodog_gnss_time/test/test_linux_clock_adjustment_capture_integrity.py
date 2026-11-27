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
