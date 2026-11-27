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
        "producer_integrity", "userspace_pipeline", "syscall_reduction",
        "handler_reduction", "syscall_handler_join",
    }
