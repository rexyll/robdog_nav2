"""Audit end-to-end completeness and shutdown drain for clock-step event streams.

This is a pure reducer over producer counters, received records and a final
shutdown snapshot. It does not collect kernel events or prove counter honesty.
"""
_COUNTERS = ("sequence_allocated", "ringbuf_reserve_failures", "ringbuf_submit_successes")
_DRAIN_FLAGS = (
    "stop_requested", "producer_quiesced", "ring_buffer_drained",
    "consumer_queue_drained", "writer_flushed", "writer_closed",
    "final_snapshot_after_drain",
)
_DRAIN_ZERO_FIELDS = ("queue_depth", "inflight_callbacks", "pending_syscalls", "pending_handlers")


def _uint(value):
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _cpu_key(item):
    return (item["producer_id"], item["boot_id"], item["cpu"])


def _validate_snapshot(snapshot, label, issues):
    if not isinstance(snapshot, dict) or not isinstance(snapshot.get("per_cpu"), list):
        issues.append({"code": "invalid_" + label + "_snapshot"})
        return {}
    result = {}
    for index, item in enumerate(snapshot["per_cpu"]):
        if not isinstance(item, dict):
            issues.append({"code": "invalid_" + label + "_cpu_counter", "index": index})
            continue
        producer, boot, cpu = item.get("producer_id"), item.get("boot_id"), item.get("cpu")
        if not isinstance(producer, str) or not producer or len(producer) > 128:
            issues.append({"code": "invalid_producer_id", "snapshot": label, "index": index})
            continue
        if not isinstance(boot, str) or not boot or len(boot) > 128:
            issues.append({"code": "invalid_boot_id", "snapshot": label, "index": index})
            continue
        if not _uint(cpu):
            issues.append({"code": "invalid_cpu", "snapshot": label, "index": index})
            continue
        key = (producer, boot, cpu)
        if key in result:
            issues.append({"code": "duplicate_cpu_counter_snapshot", "snapshot": label, "index": index})
            continue
        values = {}
        valid = True
        for name in _COUNTERS:
            value = item.get(name)
            if not _uint(value):
                issues.append({"code": "invalid_counter", "snapshot": label, "counter": name, "index": index})
                valid = False
            else:
                values[name] = value
        if valid:
            result[key] = values
    return result


_GLOBAL_COUNTERS = (
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
)
_GLOBAL_GAUGES = (
    "active_syscall_invocations", "active_handler_invocations",
    "inflight_bpf_callbacks", "task_context_reservations",
)


def audit_clock_adjustment_producer_runtime_counters(initial, final):
    """Audit global ID/task-context counters and producer quiescence gauges."""
    issues = []
    if not isinstance(initial, dict) or not isinstance(final, dict):
        raise TypeError("global runtime snapshots must be mappings")
    for field in ("capture_session_id", "boot_id"):
        a, b = initial.get(field), final.get(field)
        if not isinstance(a, str) or not a.strip() or a != b:
            issues.append({"code": "global_runtime_identity_mismatch", "field": field})
    if final.get("snapshot_after_producer_detach") is not True:
        issues.append({"code": "global_snapshot_not_after_producer_detach"})

    before, after = initial.get("counters"), final.get("counters")
    if not isinstance(before, dict) or not isinstance(after, dict):
        issues.append({"code": "invalid_global_runtime_counters"})
        before, after = {}, {}
    deltas = {}
    for name in _GLOBAL_COUNTERS:
        a, b = before.get(name), after.get(name)
        if not _uint(a) or not _uint(b):
            issues.append({"code": "invalid_global_runtime_counter", "counter": name})
            continue
        if b < a:
            issues.append({"code": "global_runtime_counter_regression", "counter": name})
            continue
        deltas[name] = b - a

    equations = (
        ("event_id_allocation", "event_id_allocation_attempts",
         ("event_id_allocation_successes", "event_id_allocation_failures")),
        ("task_instance_id_allocation", "task_instance_id_allocation_attempts",
         ("task_instance_id_allocation_successes", "task_instance_id_allocation_failures")),
        ("task_context_admission", "task_context_admission_attempts",
         ("task_context_admitted", "task_context_capacity_rejected")),
        ("task_context_create", "task_context_create_attempts",
         ("task_context_create_successes", "task_context_create_failures")),
        ("task_context_init", "task_context_init_attempts",
         ("task_context_init_successes", "task_context_init_failures")),
        ("syscall_invocation_allocation", "syscall_invocation_allocation_attempts",
         ("syscall_invocation_allocation_successes", "syscall_invocation_allocation_failures")),
        ("handler_invocation_allocation", "handler_invocation_allocation_attempts",
         ("handler_invocation_allocation_successes", "handler_invocation_allocation_failures")),
    )
    if len(deltas) == len(_GLOBAL_COUNTERS):
        for label, total_name, parts in equations:
            accounted = sum(deltas[name] for name in parts)
            if deltas[total_name] != accounted:
                issues.append({"code": "global_runtime_counter_equation_mismatch",
                               "equation": label, "attempts": deltas[total_name],
                               "accounted": accounted})

    failure_counters = (
        "event_id_allocation_failures", "task_instance_id_allocation_failures",
        "task_context_capacity_rejected", "task_context_create_failures",
        "task_context_init_failures", "syscall_invocation_allocation_failures",
        "handler_invocation_allocation_failures", "cpu_sequence_allocation_failures",
        "syscall_context_stack_overflow", "syscall_context_pop_mismatch",
        "handler_context_stack_overflow", "handler_context_pop_mismatch",
        "handler_parent_context_unavailable", "task_exit_with_pending_syscalls",
        "task_exit_with_pending_handlers",
    )
    for name in failure_counters:
        if deltas.get(name, 0) > 0:
            issues.append({"code": "producer_runtime_failure_observed",
                           "counter": name, "delta": deltas[name]})

    gauges_before, gauges_after = initial.get("gauges"), final.get("gauges")
    if not isinstance(gauges_before, dict) or not isinstance(gauges_after, dict):
        issues.append({"code": "invalid_global_runtime_gauges"})
        gauges_after = {}
    final_gauges = {}
    for name in _GLOBAL_GAUGES:
        value = gauges_after.get(name)
        if not _uint(value):
            issues.append({"code": "invalid_global_runtime_gauge", "gauge": name})
            continue
        final_gauges[name] = value
        if name != "task_context_reservations" and value != 0:
            issues.append({"code": "producer_runtime_work_remaining",
                           "gauge": name, "value": value})

    return {
        "schema_version": 1,
        "runtime_status": "consistent" if not issues else "partial",
        "counter_deltas": deltas,
        "final_gauges": final_gauges,
        "task_context_reservations_zero_required": False,
        "issues": issues,
        "interpretation": (
            "Consistent means the supplied global allocator/task-context counters "
            "reconcile and active invocation/callback gauges are zero after producer "
            "detach. A nonzero task_context_reservations gauge is allowed because "
            "live tasks may own idle storage; the session map is destroyed after drain."
        ),
    }



def audit_clock_adjustment_capture_integrity(events, initial_snapshot, final_snapshot, drain_snapshot):
    """Reconcile per-CPU allocated sequence, reserve failures and received records.

    Producer contract: allocate/increment sequence before ringbuf reserve; every
    attempted record increments exactly one of reserve_failures or submit_successes.
    A dedicated, unfiltered consumer stream receives every submitted record.
    """
    issues = []
    if not isinstance(events, list):
        raise TypeError("events must be a list")
    initial = _validate_snapshot(initial_snapshot, "initial", issues)
    final = _validate_snapshot(final_snapshot, "final", issues)

    observed = {}
    malformed = 0
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            malformed += 1
            issues.append({"code": "event_not_mapping", "event_index": index})
            continue
        producer, boot, cpu, seq = (event.get("producer_id"), event.get("boot_id"),
                                    event.get("cpu"), event.get("sequence"))
        if (not isinstance(producer, str) or not producer or len(producer) > 128
            or not isinstance(boot, str) or not boot or len(boot) > 128
            or not _uint(cpu) or not _uint(seq) or seq == 0):
            malformed += 1
            issues.append({"code": "malformed_event_envelope", "event_index": index})
            continue
        key = (producer, boot, cpu)
        seen = observed.setdefault(key, set())
        if seq in seen:
            issues.append({"code": "duplicate_event_sequence", "producer_id": producer,
                           "boot_id": boot, "cpu": cpu, "sequence": seq})
        seen.add(seq)

    all_keys = set(initial) | set(final) | set(observed)
    per_cpu = []
    for key in sorted(all_keys):
        before, after = initial.get(key), final.get(key)
        if before is None or after is None:
            issues.append({"code": "missing_counter_snapshot", "producer_id": key[0],
                           "boot_id": key[1], "cpu": key[2]})
            continue
        deltas = {name: after[name] - before[name] for name in _COUNTERS}
        if any(value < 0 for value in deltas.values()):
            issues.append({"code": "counter_regression", "producer_id": key[0],
                           "boot_id": key[1], "cpu": key[2]})
            continue
        seq_start, seq_end = before["sequence_allocated"], after["sequence_allocated"]
        if deltas["sequence_allocated"] != (
            deltas["ringbuf_reserve_failures"] + deltas["ringbuf_submit_successes"]
        ):
            issues.append({"code": "producer_counter_equation_mismatch", "producer_id": key[0],
                           "boot_id": key[1], "cpu": key[2]})
        seqs = observed.get(key, set())
        out_of_range = sorted(seq for seq in seqs if not seq_start < seq <= seq_end)
        if out_of_range:
            issues.append({"code": "event_sequence_outside_snapshot_window",
                           "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
                           "sequences": out_of_range})
        expected_submitted = deltas["ringbuf_submit_successes"]
        if len(seqs) != expected_submitted:
            issues.append({"code": "submitted_vs_received_count_mismatch",
                           "producer_id": key[0], "boot_id": key[1],
                           "cpu": key[2], "submitted": expected_submitted,
                           "received": len(seqs)})
        # Compact gap ranges avoid allocating a uint64-sized Python set.
        missing_ranges = []
        cursor = seq_start + 1
        for seq in sorted(value for value in seqs if seq_start < value <= seq_end):
            if seq > cursor:
                missing_ranges.append({"first": cursor, "last": seq - 1})
            cursor = max(cursor, seq + 1)
        if cursor <= seq_end:
            missing_ranges.append({"first": cursor, "last": seq_end})
        missing_count = sum(item["last"] - item["first"] + 1 for item in missing_ranges)
        expected_reserve_failures = deltas["ringbuf_reserve_failures"]
        if missing_count != expected_reserve_failures:
            issues.append({"code": "sequence_gaps_vs_reserve_failures_mismatch",
                           "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
                           "sequence_gaps": missing_count,
                           "reserve_failures": expected_reserve_failures})
        per_cpu.append({
            "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
            "sequence_allocated_delta": deltas["sequence_allocated"],
            "ringbuf_reserve_failures_delta": expected_reserve_failures,
            "ringbuf_submit_successes_delta": expected_submitted,
            "received_records": len(seqs), "missing_sequence_ranges": missing_ranges,
            "missing_sequence_count": missing_count,
            "status": "consistent" if not out_of_range and len(seqs) == expected_submitted
                      and missing_count == expected_reserve_failures
                      and deltas["sequence_allocated"] == expected_reserve_failures + expected_submitted
                      else "partial",
        })

    if not isinstance(drain_snapshot, dict):
        raise TypeError("drain_snapshot must be a mapping")
    drain = {}
    for name in _DRAIN_FLAGS:
        value = drain_snapshot.get(name)
        if not isinstance(value, bool):
            issues.append({"code": "invalid_drain_flag", "field": name})
        else:
            drain[name] = value
            if not value:
                issues.append({"code": "shutdown_drain_incomplete", "field": name})
    for name in _DRAIN_ZERO_FIELDS:
        value = drain_snapshot.get(name)
        if not _uint(value):
            issues.append({"code": "invalid_drain_counter", "field": name})
        else:
            drain[name] = value
            if value != 0:
                issues.append({"code": "shutdown_work_remaining", "field": name, "value": value})

    if malformed:
        issues.append({"code": "malformed_event_count", "count": malformed})
    global_initial = (initial_snapshot.get("global")
                      if isinstance(initial_snapshot, dict) else None)
    global_final = (final_snapshot.get("global")
                    if isinstance(final_snapshot, dict) else None)
    global_runtime = None
    if global_initial is not None or global_final is not None:
        if global_initial is None or global_final is None:
            global_runtime = {
                "runtime_status": "partial",
                "issues": [{"code": "missing_global_runtime_snapshot"}],
            }
        else:
            global_runtime = audit_clock_adjustment_producer_runtime_counters(
                global_initial, global_final
            )
        issues.extend(global_runtime.get("issues", []))

    consistent = (not issues and all(item["status"] == "consistent" for item in per_cpu)
                  and (global_runtime is None
                       or global_runtime.get("runtime_status") == "consistent"))
    return {
        "schema_version": 1,
        "integrity_status": "complete" if consistent else "partial",
        "producer_contract": "sequence_increment_before_reserve_one_terminal_counter_per_attempt",
        "consumer_contract": "dedicated_unfiltered_stream",
        "counts": {
            "events_input": len(events), "events_malformed": malformed,
            "cpu_streams_audited": len(per_cpu),
            "sequence_allocated_delta": sum(x["sequence_allocated_delta"] for x in per_cpu),
            "reserve_failures": sum(x["ringbuf_reserve_failures_delta"] for x in per_cpu),
            "submitted_records": sum(x["ringbuf_submit_successes_delta"] for x in per_cpu),
            "received_records": sum(x["received_records"] for x in per_cpu),
        },
        "per_cpu": per_cpu,
        "global_runtime": global_runtime,
        "drain": drain,
        "issues": issues,
        "interpretation": (
            "Complete means the supplied per-CPU counters reconcile with the dedicated "
            "unfiltered received-record stream and the declared stop/drain snapshot is "
            "quiescent. It does not prove the kernel producer or counters are truthful, "
            "that instrumentation covered every relevant call path, or that a clock changed."
        ),
    }


__all__ = [\n    "audit_clock_adjustment_capture_integrity",\n    "audit_clock_adjustment_producer_runtime_counters",\n]
