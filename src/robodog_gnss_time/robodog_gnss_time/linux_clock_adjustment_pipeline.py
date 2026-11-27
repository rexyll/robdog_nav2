"""Userspace pipeline accounting and unified offline clock-capture report.

This module audits explicit stage counters; it is not a live collector.
"""
from typing import Any

from .linux_clock_adjustment_capture_integrity import audit_clock_adjustment_capture_integrity
from .linux_clock_adjustment_events import reduce_clock_adjustment_events
from .linux_clock_adjustment_kernel_observations import reduce_kernel_clock_adjustment_observations
from .linux_clock_adjustment_joiner import correlate_clock_adjustment_observations
from .linux_clock_adjustment_session import audit_clock_adjustment_capture_lifecycle

_COUNTERS = (
    "consumer_records_read", "decode_successes", "decode_failures",
    "queue_enqueued", "queue_admission_drops", "writer_records_written",
    "writer_write_failures",
)
_IDENTITY = ("capture_session_id", "boot_id")


def _counter(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def audit_clock_adjustment_userspace_pipeline(initial_snapshot, final_snapshot):
    """Reconcile read -> decode -> queue -> writer counters between snapshots.

    Queue admission drops and writer failures are distinct terminal outcomes.
    The final snapshot must be after queue drain and writer close.
    """
    issues = []
    if not isinstance(initial_snapshot, dict) or not isinstance(final_snapshot, dict):
        raise TypeError("pipeline snapshots must be mappings")

    for name in _IDENTITY:
        a, b = initial_snapshot.get(name), final_snapshot.get(name)
        if not isinstance(a, str) or not a.strip() or a != b:
            issues.append({"code": "pipeline_identity_mismatch", "field": name})

    if final_snapshot.get("snapshot_after_writer_close") is not True:
        issues.append({"code": "final_snapshot_not_after_writer_close"})

    deltas = {}
    for name in _COUNTERS:
        a, b = initial_snapshot.get(name), final_snapshot.get(name)
        if not _counter(a) or not _counter(b):
            issues.append({"code": "invalid_pipeline_counter", "field": name})
            continue
        if b < a:
            issues.append({"code": "pipeline_counter_regression", "field": name})
            continue
        deltas[name] = b - a

    for name in ("final_queue_depth", "final_inflight_writer_records"):
        value = final_snapshot.get(name)
        if not _counter(value):
            issues.append({"code": "invalid_final_pipeline_gauge", "field": name})
        elif value != 0:
            issues.append({"code": "pipeline_work_remaining", "field": name, "value": value})

    if len(deltas) == len(_COUNTERS):
        equations = (
            ("read_decode", deltas["consumer_records_read"],
             deltas["decode_successes"] + deltas["decode_failures"]),
            ("decode_queue", deltas["decode_successes"],
             deltas["queue_enqueued"] + deltas["queue_admission_drops"]),
            ("queue_writer", deltas["queue_enqueued"],
             deltas["writer_records_written"] + deltas["writer_write_failures"]),
        )
        for name, left, right in equations:
            if left != right:
                issues.append({"code": "pipeline_stage_equation_mismatch",
                               "stage": name, "input_count": left, "accounted_count": right})

    return {
        "schema_version": 1,
        "pipeline_status": "consistent" if not issues else "partial",
        "identity": {name: final_snapshot.get(name) for name in _IDENTITY},
        "deltas": deltas,
        "final_gauges": {
            "queue_depth": final_snapshot.get("final_queue_depth"),
            "inflight_writer_records": final_snapshot.get("final_inflight_writer_records"),
        },
        "issues": issues,
        "interpretation": (
            "Consistent means stage counters reconcile for the supplied snapshots. "
            "It does not prove counters are truthful, records are semantically valid, "
            "or the input stream is complete unless separately established."
        ),
    }


def build_clock_adjustment_capture_report(
    syscall_events, handler_events, producer_events,
    producer_initial_snapshot, producer_final_snapshot, drain_snapshot,
    pipeline_initial_snapshot, pipeline_final_snapshot,
    wrapper_contract_id=None, lifecycle_transitions=None, lifecycle_final_snapshot=None,
):
    """Compose all offline reducers and preserve their independent status gates."""
    producer = audit_clock_adjustment_capture_integrity(
        producer_events, producer_initial_snapshot, producer_final_snapshot, drain_snapshot
    )
    pipeline = audit_clock_adjustment_userspace_pipeline(
        pipeline_initial_snapshot, pipeline_final_snapshot
    )
    syscall = reduce_clock_adjustment_events(syscall_events)
    handler = reduce_kernel_clock_adjustment_observations(handler_events)
    joined = correlate_clock_adjustment_observations(syscall, handler, wrapper_contract_id)
    lifecycle = audit_clock_adjustment_capture_lifecycle(
        lifecycle_transitions if lifecycle_transitions is not None else [],
        lifecycle_final_snapshot if lifecycle_final_snapshot is not None else {},
    )

    issues = []
    producer_received = producer.get("counts", {}).get("received_records")
    pipeline_read = pipeline.get("deltas", {}).get("consumer_records_read")
    count_match = (
        _counter(producer_received) and _counter(pipeline_read)
        and producer_received == pipeline_read
    )
    pipeline_boot = pipeline.get("identity", {}).get("boot_id")
    producer_boots = {
        item.get("boot_id") for item in producer.get("per_cpu", [])
        if isinstance(item, dict) and isinstance(item.get("boot_id"), str)
    }
    boot_match = bool(pipeline_boot) and producer_boots == {pipeline_boot}
    if not count_match:
        issues.append({"code": "producer_received_vs_pipeline_read_mismatch",
                       "producer_received_records": producer_received,
                       "pipeline_records_read": pipeline_read})
    if not boot_match:
        issues.append({"code": "producer_pipeline_boot_id_mismatch",
                       "producer_boot_ids": sorted(producer_boots),
                       "pipeline_boot_id": pipeline_boot})
    gates = {
        "producer_integrity": producer.get("integrity_status") == "complete",
        "userspace_pipeline": pipeline.get("pipeline_status") == "consistent",
        "producer_to_pipeline_record_count": count_match,
        "producer_pipeline_boot_identity": boot_match,
        "syscall_reduction": syscall.get("reduction_status") == "consistent",
        "handler_reduction": handler.get("reduction_status") == "consistent",
        "syscall_handler_join": joined.get("reduction_status") == "consistent",
        "capture_lifecycle": lifecycle.get("lifecycle_status") == "complete",
    }
    for name, passed in gates.items():
        if not passed:
            issues.append({"code": "capture_report_gate_not_passed", "gate": name})

    return {
        "schema_version": 1,
        "capture_report_status": "consistent" if all(gates.values()) else "partial",
        "gates": gates,
        "producer_integrity": producer,
        "userspace_pipeline": pipeline,
        "syscall_reduction": syscall,
        "handler_reduction": handler,
        "syscall_handler_join": joined,
        "capture_lifecycle": lifecycle,
        "issues": issues,
        "interpretation": (
            "A consistent report means all supplied offline reducer gates passed. "
            "It does not prove instrumentation coverage, kernel counter honesty, "
            "syscall-to-handler call-stack ancestry, or a physical clock effect."
        ),
    }


__all__ = [
    "audit_clock_adjustment_userspace_pipeline",
    "build_clock_adjustment_capture_report",
]
