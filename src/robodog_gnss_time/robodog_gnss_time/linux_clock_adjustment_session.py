"""Versioned event-envelope and capture-session lifecycle contracts.

Pure validation/reduction only; this module does not start or own a live collector.
"""
from typing import Any

_SCHEMA_VERSION = 1
_STREAM_KINDS = {"syscall", "kernel_handler"}
_LIFECYCLE = (
    "created", "producers_attached", "running", "stop_requested",
    "producers_quiesced", "ring_buffer_drained", "consumer_queue_drained",
    "writer_flushed", "writer_closed", "finalized",
)
_UINT64_MAX = (1 << 64) - 1
_TEXT_LIMIT = 128


def _text(value: Any) -> bool:
    return isinstance(value, str) and 0 < len(value.strip()) <= _TEXT_LIMIT


def _uint(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= _UINT64_MAX


def validate_clock_adjustment_event_envelope(record):
    """Validate envelope identity and return a sanitized legacy reducer event.

    Payload fields cannot override envelope-owned fields. Unknown envelope
    versions and stream kinds fail closed. The payload is copied, not echoed
    wholesale in the result.
    """
    issues = []
    if not isinstance(record, dict):
        return {"status": "invalid", "issues": [{"code": "record_not_mapping"}]}
    required_text = ("capture_session_id", "boot_id", "producer_id", "stream_id")
    for name in required_text:
        if not _text(record.get(name)):
            issues.append({"code": "invalid_envelope_identity", "field": name})
    if record.get("schema_version") != _SCHEMA_VERSION or isinstance(record.get("schema_version"), bool):
        issues.append({"code": "unsupported_envelope_schema_version"})
    kind = record.get("stream_kind")
    if not isinstance(kind, str) or kind not in _STREAM_KINDS:
        issues.append({"code": "unsupported_stream_kind"})
    for name in ("cpu", "sequence", "monotonic_ns"):
        value = record.get(name)
        if not _uint(value):
            issues.append({"code": "invalid_envelope_integer", "field": name})
    if _uint(record.get("sequence")) and record["sequence"] == 0:
        issues.append({"code": "zero_event_sequence"})
    payload = record.get("payload")
    if not isinstance(payload, dict):
        issues.append({"code": "invalid_event_payload"})
    if issues:
        return {"status": "invalid", "issues": issues}

    reserved = set(required_text) | {
        "schema_version", "stream_kind", "cpu", "sequence", "monotonic_ns", "payload"
    }
    if reserved.intersection(payload):
        return {"status": "invalid", "issues": [{"code": "payload_overrides_envelope_field"}]}

    event = dict(payload)
    event.update({
        "boot_id": record["boot_id"],
        "cpu": record["cpu"],
        "sequence": record["sequence"],
        "monotonic_ns": record["monotonic_ns"],
    })
    expected_types = (
        {"syscall_enter", "syscall_exit"} if kind == "syscall"
        else {"kernel_handler_enter", "kernel_handler_exit"}
    )
    if not isinstance(event.get("event_type"), str) or event["event_type"] not in expected_types:
        return {"status": "invalid", "issues": [{"code": "event_type_stream_kind_mismatch"}]}
    return {
        "status": "valid",
        "identity": {
            "capture_session_id": record["capture_session_id"],
            "boot_id": record["boot_id"],
            "producer_id": record["producer_id"],
            "stream_id": record["stream_id"],
            "stream_kind": kind,
            "cpu": record["cpu"],
            "sequence": record["sequence"],
        },
        "event": event,
        "issues": [],
    }


def audit_clock_adjustment_capture_lifecycle(transitions, final_snapshot):
    """Require the exact ordered lifecycle through writer close and finalization."""
    issues = []
    if not isinstance(transitions, list):
        raise TypeError("transitions must be a list")
    accepted = []
    previous_time = -1
    session_id = boot_id = None
    for index, item in enumerate(transitions):
        if not isinstance(item, dict):
            issues.append({"code": "lifecycle_transition_not_mapping", "index": index})
            continue
        state = item.get("state")
        sid, bid, timestamp = item.get("capture_session_id"), item.get("boot_id"), item.get("monotonic_ns")
        if state not in _LIFECYCLE:
            issues.append({"code": "unknown_lifecycle_state", "index": index})
            continue
        if not _text(sid) or not _text(bid) or not _uint(timestamp):
            issues.append({"code": "invalid_lifecycle_identity_or_time", "index": index})
            continue
        if session_id is None:
            session_id, boot_id = sid, bid
        elif sid != session_id or bid != boot_id:
            issues.append({"code": "lifecycle_identity_changed", "index": index})
        if timestamp < previous_time:
            issues.append({"code": "lifecycle_monotonic_time_regression", "index": index})
        previous_time = max(previous_time, timestamp)
        accepted.append(state)

    if tuple(accepted) != _LIFECYCLE:
        issues.append({
            "code": "lifecycle_order_or_completeness_mismatch",
            "expected": list(_LIFECYCLE), "observed": accepted,
        })
    if not isinstance(final_snapshot, dict):
        raise TypeError("final_snapshot must be a mapping")
    if final_snapshot.get("capture_session_id") != session_id:
        issues.append({"code": "final_snapshot_session_mismatch"})
    if final_snapshot.get("boot_id") != boot_id:
        issues.append({"code": "final_snapshot_boot_mismatch"})
    if final_snapshot.get("snapshot_after_writer_close") is not True:
        issues.append({"code": "final_snapshot_not_after_writer_close"})
    for field in ("queue_depth", "inflight_callbacks", "pending_syscalls",
                  "pending_handlers", "inflight_writer_records"):
        value = final_snapshot.get(field)
        if not isinstance(value, int) or isinstance(value, bool) or value != 0:
            issues.append({"code": "final_snapshot_not_quiescent", "field": field})
    return {
        "schema_version": 1,
        "lifecycle_status": "complete" if not issues else "partial",
        "identity": {"capture_session_id": session_id, "boot_id": boot_id},
        "observed_transitions": accepted,
        "issues": issues,
        "interpretation": (
            "Complete means the supplied ordered lifecycle and final quiescence "
            "snapshot satisfy this contract; it does not prove producer coverage "
            "or truthful runtime counters."
        ),
    }


__all__ = [
    "validate_clock_adjustment_event_envelope",
    "audit_clock_adjustment_capture_lifecycle",
]
