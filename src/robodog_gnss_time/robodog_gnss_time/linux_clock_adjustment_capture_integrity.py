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
        missing = sorted(set(range(seq_start + 1, seq_end + 1)) - seqs)
        expected_reserve_failures = deltas["ringbuf_reserve_failures"]
        if len(missing) != expected_reserve_failures:
            issues.append({"code": "sequence_gaps_vs_reserve_failures_mismatch",
                           "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
                           "sequence_gaps": len(missing),
                           "reserve_failures": expected_reserve_failures})
        per_cpu.append({
            "producer_id": key[0], "boot_id": key[1], "cpu": key[2],
            "sequence_allocated_delta": deltas["sequence_allocated"],
            "ringbuf_reserve_failures_delta": expected_reserve_failures,
            "ringbuf_submit_successes_delta": expected_submitted,
            "received_records": len(seqs), "missing_sequences": missing,
            "status": "consistent" if not out_of_range and len(seqs) == expected_submitted
                      and len(missing) == expected_reserve_failures
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
    consistent = not issues and all(item["status"] == "consistent" for item in per_cpu)
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
        "drain": drain,
        "issues": issues,
        "interpretation": (
            "Complete means the supplied per-CPU counters reconcile with the dedicated "
            "unfiltered received-record stream and the declared stop/drain snapshot is "
            "quiescent. It does not prove the kernel producer or counters are truthful, "
            "that instrumentation covered every relevant call path, or that a clock changed."
        ),
    }


__all__ = ["audit_clock_adjustment_capture_integrity"]
