import pytest

from robodog_gnss_time.linux_clock_adjustment_session import (
    audit_clock_adjustment_capture_lifecycle,
    validate_clock_adjustment_event_envelope,
)


def envelope(**overrides):
    value = {
        "schema_version": 1,
        "capture_session_id": "capture-01",
        "boot_id": "boot-01",
        "producer_id": "syscall-tracer",
        "stream_id": "syscall-events",
        "stream_kind": "syscall",
        "cpu": 2,
        "sequence": 1,
        "monotonic_ns": 100,
        "payload": {
            "event_type": "syscall_enter",
            "pid_tgid": 4294967297,
            "syscall": "clock_adjtime",
            "raw_clockid": 0,
            "modes_read_status": "not_requested",
        },
    }
    value.update(overrides)
    return value


def test_valid_envelope_projects_legacy_reducer_event():
    result = validate_clock_adjustment_event_envelope(envelope())
    assert result["status"] == "valid"
    assert result["identity"]["capture_session_id"] == "capture-01"
    assert result["event"]["boot_id"] == "boot-01"
    assert result["event"]["sequence"] == 1
    assert result["event"]["monotonic_ns"] == 100


@pytest.mark.parametrize("field,value", [
    ("schema_version", 3),
    ("schema_version", []),
    ("capture_session_id", ""),
    ("boot_id", None),
    ("producer_id", "x" * 129),
    ("stream_id", ""),
    ("stream_kind", "unknown"),
    ("cpu", True),
    ("sequence", 0),
    ("monotonic_ns", -1),
    ("payload", []),
    ("stream_kind", {}),
])
def test_invalid_envelope_fields_fail_closed(field, value):
    result = validate_clock_adjustment_event_envelope(envelope(**{field: value}))
    assert result["status"] == "invalid"
    assert result["issues"]


def test_payload_cannot_override_envelope_identity_or_ordering():
    record = envelope(payload={
        "event_type": "syscall_enter",
        "boot_id": "spoofed",
        "sequence": 999,
    })
    result = validate_clock_adjustment_event_envelope(record)
    assert result["status"] == "invalid"
    assert result["issues"][0]["code"] == "payload_overrides_envelope_field"


@pytest.mark.parametrize("kind,event_type", [
    ("syscall", "kernel_handler_enter"),
    ("kernel_handler", "syscall_enter"),
    ("kernel_handler", {}),
])
def test_stream_kind_must_match_event_type(kind, event_type):
    record = envelope(stream_kind=kind, payload={"event_type": event_type})
    assert validate_clock_adjustment_event_envelope(record)["status"] == "invalid"


def test_non_mapping_envelope_does_not_raise():
    assert validate_clock_adjustment_event_envelope([])["status"] == "invalid"



def envelope_v2(**overrides):
    value = envelope()
    value.update({
        "schema_version": 2,
        "task_instance_id": "task-life-01",
        "event_id": "event-0001",
        "invocation_id": "syscall-inv-01",
    })
    value.update(overrides)
    return value


def test_schema_v2_valid_syscall_envelope_projects_invocation_identity():
    result = validate_clock_adjustment_event_envelope(envelope_v2())
    assert result["status"] == "valid"
    assert result["identity"]["schema_version"] == 2
    assert result["identity"]["task_instance_id"] == "task-life-01"
    assert result["event"]["event_id"] == "event-0001"
    assert result["event"]["invocation_id"] == "syscall-inv-01"


def test_schema_v2_valid_handler_parent_context_is_preserved():
    record = envelope_v2(
        stream_kind="kernel_handler",
        payload={
            "event_type": "kernel_handler_enter",
            "pid_tgid": 4294967297,
            "handler": "do_adjtimex",
        },
        invocation_id="handler-inv-01",
        parent_syscall_invocation_id="syscall-inv-01",
        parent_context_status="matched",
    )
    result = validate_clock_adjustment_event_envelope(record)
    assert result["status"] == "valid"
    assert result["event"]["parent_syscall_invocation_id"] == "syscall-inv-01"
    assert result["event"]["parent_context_status"] == "matched"


@pytest.mark.parametrize("status,parent,code", [
    ("matched", None, "matched_parent_context_missing_id"),
    ("unavailable", "syscall-inv-01", "unavailable_parent_context_has_id"),
    ("stack_overflow", "syscall-inv-01", "unavailable_parent_context_has_id"),
    ("no_active_context", "syscall-inv-01", "unavailable_parent_context_has_id"),
    ({}, None, "invalid_parent_context_status"),
])
def test_schema_v2_handler_parent_context_fails_closed(status, parent, code):
    record = envelope_v2(
        stream_kind="kernel_handler",
        payload={"event_type": "kernel_handler_enter"},
        invocation_id="handler-inv-01",
        parent_context_status=status,
    )
    if parent is not None:
        record["parent_syscall_invocation_id"] = parent
    result = validate_clock_adjustment_event_envelope(record)
    assert result["status"] == "invalid"
    assert code in [item["code"] for item in result["issues"]]


def test_schema_v2_syscall_rejects_handler_parent_fields():
    record = envelope_v2(parent_context_status="matched")
    result = validate_clock_adjustment_event_envelope(record)
    assert result["status"] == "invalid"
    assert "syscall_envelope_has_handler_parent_fields" in [
        item["code"] for item in result["issues"]
    ]


@pytest.mark.parametrize("field,value", [
    ("task_instance_id", ""),
    ("event_id", None),
    ("invocation_id", 1),
])
def test_schema_v2_requires_opaque_identity_strings(field, value):
    result = validate_clock_adjustment_event_envelope(envelope_v2(**{field: value}))
    assert result["status"] == "invalid"
    assert "invalid_invocation_identity" in [item["code"] for item in result["issues"]]


def test_schema_v2_payload_cannot_override_new_envelope_identity():
    record = envelope_v2(payload={
        "event_type": "syscall_enter",
        "invocation_id": "spoofed",
    })
    result = validate_clock_adjustment_event_envelope(record)
    assert result["status"] == "invalid"
    assert result["issues"][0]["code"] == "payload_overrides_envelope_field"


def test_schema_v1_remains_accepted_as_legacy_without_fabricated_ids():
    result = validate_clock_adjustment_event_envelope(envelope())
    assert result["status"] == "valid"
    assert "invocation_id" not in result["event"]


def lifecycle():
    states = [
        "created", "producers_attached", "running", "stop_requested",
        "producers_quiesced", "ring_buffer_drained", "consumer_queue_drained",
        "writer_flushed", "writer_closed", "finalized",
    ]
    return [
        {"state": state, "capture_session_id": "capture-01",
         "boot_id": "boot-01", "monotonic_ns": index * 10}
        for index, state in enumerate(states)
    ]


def final_snapshot(**overrides):
    value = {
        "capture_session_id": "capture-01",
        "boot_id": "boot-01",
        "snapshot_after_writer_close": True,
        "queue_depth": 0,
        "inflight_callbacks": 0,
        "pending_syscalls": 0,
        "pending_handlers": 0,
        "inflight_writer_records": 0,
    }
    value.update(overrides)
    return value


def test_complete_lifecycle_requires_exact_order_and_quiescent_snapshot():
    result = audit_clock_adjustment_capture_lifecycle(lifecycle(), final_snapshot())
    assert result["lifecycle_status"] == "complete"


@pytest.mark.parametrize("mutator,code", [
    (lambda xs: xs.pop(), "lifecycle_order_or_completeness_mismatch"),
    (lambda xs: xs.reverse(), "lifecycle_order_or_completeness_mismatch"),
    (lambda xs: xs.__setitem__(3, {**xs[3], "state": "running"}),
     "lifecycle_order_or_completeness_mismatch"),
    (lambda xs: xs.__setitem__(4, {**xs[4], "capture_session_id": "other"}),
     "lifecycle_identity_changed"),
    (lambda xs: xs.__setitem__(5, {**xs[5], "monotonic_ns": 1}),
     "lifecycle_monotonic_time_regression"),
])
def test_lifecycle_transition_faults_fail_closed(mutator, code):
    transitions = lifecycle()
    mutator(transitions)
    result = audit_clock_adjustment_capture_lifecycle(transitions, final_snapshot())
    assert result["lifecycle_status"] == "partial"
    assert code in [item["code"] for item in result["issues"]]


@pytest.mark.parametrize("field,value", [
    ("queue_depth", 1),
    ("inflight_callbacks", 1),
    ("pending_syscalls", 1),
    ("pending_handlers", 1),
    ("inflight_writer_records", 1),
    ("snapshot_after_writer_close", False),
    ("boot_id", "other"),
    ("capture_session_id", "other"),
])
def test_final_snapshot_must_match_and_be_quiescent(field, value):
    result = audit_clock_adjustment_capture_lifecycle(
        lifecycle(), final_snapshot(**{field: value})
    )
    assert result["lifecycle_status"] == "partial"


def test_lifecycle_wrong_container_type_rejected():
    with pytest.raises(TypeError):
        audit_clock_adjustment_capture_lifecycle({}, final_snapshot())
