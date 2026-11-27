"""Reducer for post-copy Linux clock-adjustment kernel-handler observations.

These records represent normalized kernel-owned timex requests observed at
candidate internal handler entry points. They are distinct from syscall-entry
user-memory snapshots and do not prove physical clock effects.
"""
from typing import Any, Iterable

from .linux_timex_modes import classify_timex_modes

_HANDLERS = {"do_clock_adjtime", "do_adjtimex"}
_EVENT_TYPES = {"kernel_handler_enter", "kernel_handler_exit"}
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_UINT64_MAX = (1 << 64) - 1
_CLOCKID_MIN = -(1 << 31)
_CLOCKID_MAX = (1 << 31) - 1


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _validate(event: Any, index: int):
    issues = []
    if not isinstance(event, dict):
        return None, [{"code": "event_not_mapping", "event_index": index}]
    event_type = event.get("event_type")
    if not isinstance(event_type, str) or event_type not in _EVENT_TYPES:
        return None, [{"code": "unknown_event_type", "event_index": index}]
    handler = event.get("handler")
    if not isinstance(handler, str) or handler not in _HANDLERS:
        issues.append({"code": "unsupported_handler", "event_index": index})
    boot_id = event.get("boot_id")
    if not isinstance(boot_id, str) or not boot_id.strip() or len(boot_id) > 128:
        issues.append({"code": "invalid_boot_id", "event_index": index})
    pid_tgid = event.get("pid_tgid")
    if not _is_int(pid_tgid) or pid_tgid <= 0 or pid_tgid > _UINT64_MAX:
        issues.append({"code": "invalid_pid_tgid", "event_index": index})
    monotonic_ns = event.get("monotonic_ns")
    if not _is_int(monotonic_ns) or monotonic_ns < 0:
        issues.append({"code": "invalid_monotonic_timestamp", "event_index": index})
    cpu = event.get("cpu")
    if not _is_int(cpu) or cpu < 0:
        issues.append({"code": "invalid_cpu", "event_index": index})
    sequence = event.get("sequence")
    if not _is_int(sequence) or sequence < 0 or sequence > _UINT64_MAX:
        issues.append({"code": "invalid_sequence", "event_index": index})

    if event_type == "kernel_handler_enter":
        raw_modes = event.get("raw_modes")
        if not _is_int(raw_modes) or not 0 <= raw_modes <= 0xFFFFFFFF:
            issues.append({"code": "invalid_kernel_raw_modes", "event_index": index})
        raw_clockid = event.get("raw_clockid")
        if handler == "do_clock_adjtime":
            if not _is_int(raw_clockid) or not _CLOCKID_MIN <= raw_clockid <= _CLOCKID_MAX:
                issues.append({"code": "invalid_kernel_raw_clockid", "event_index": index})
        elif handler == "do_adjtimex" and raw_clockid is not None:
            issues.append({"code": "unexpected_kernel_raw_clockid", "event_index": index})
    else:
        return_value = event.get("return_value")
        if not _is_int(return_value) or not _INT64_MIN <= return_value <= _INT64_MAX:
            issues.append({"code": "invalid_handler_return_value", "event_index": index})

    if issues:
        return None, issues

    projected = {
        "event_type": event_type,
        "handler": handler,
        "boot_id": boot_id,
        "pid_tgid": pid_tgid,
        "monotonic_ns": monotonic_ns,
        "cpu": cpu,
        "sequence": sequence,
    }
    if event_type == "kernel_handler_enter":
        projected["raw_modes"] = raw_modes
        projected["mode_decode"] = classify_timex_modes(raw_modes)
        if handler == "do_clock_adjtime":
            projected["raw_clockid"] = raw_clockid
    else:
        projected["return_value"] = return_value
    return projected, issues


def _interpret_handler_return(value: int) -> str:
    if value >= 0:
        return "nonnegative_handler_return_not_effect_proof"
    if value == -14:
        return "negative_efault_handler_result_not_copyback_proof"
    return "negative_errno_handler_effect_unresolved"


def reduce_kernel_clock_adjustment_observations(
    events: Iterable[dict[str, Any]], max_pending: int = 4096
) -> dict[str, Any]:
    """Pair internal handler records using a per-thread, per-handler LIFO stack.

    The caller must supply this handler-observation stream in event order.
    Per-CPU sequence numbers are scoped to this stream. Without a producer-side
    invocation cookie, LIFO is a conservative nesting assumption, not call-stack
    proof. This reducer does not correlate records to outer syscall events.
    """
    if isinstance(max_pending, bool) or not isinstance(max_pending, int):
        raise TypeError("max_pending must be an integer, not bool")
    if max_pending < 1:
        raise ValueError("max_pending must be at least 1")
    if not isinstance(events, Iterable) or isinstance(events, (str, bytes, dict)):
        raise TypeError("events must be an iterable of event mappings")

    pending = {}
    last_sequence = {}
    pairs = []
    unpaired = []
    orphan_exits = []
    issues = []
    counts = {
        "input_events": 0,
        "accepted_events": 0,
        "malformed_events": 0,
        "handler_entries_seen": 0,
        "handler_returns_seen": 0,
        "handler_pairs_completed": 0,
        "orphan_handler_returns": 0,
        "duplicate_handler_entries": 0,
        "nested_handler_entries": 0,
        "pending_capacity_drops": 0,
    }

    for index, raw in enumerate(events):
        counts["input_events"] += 1
        event, event_issues = _validate(raw, index)
        if event_issues:
            issues.extend(event_issues)
        if event is None:
            counts["malformed_events"] += 1
            continue
        counts["accepted_events"] += 1

        seq_key = (event["boot_id"], event["cpu"])
        previous = last_sequence.get(seq_key)
        if previous is not None:
            if event["sequence"] > previous + 1:
                gap = event["sequence"] - previous - 1
                issues.append({
                    "code": "per_cpu_sequence_gap",
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "previous_sequence": previous,
                    "current_sequence": event["sequence"],
                    "missing_count": gap,
                })
            elif event["sequence"] == previous:
                issues.append({
                    "code": "duplicate_per_cpu_sequence",
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "sequence": event["sequence"],
                })
            elif event["sequence"] < previous:
                issues.append({
                    "code": "per_cpu_sequence_regression",
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "previous_sequence": previous,
                    "current_sequence": event["sequence"],
                })
        if previous is None or event["sequence"] > previous:
            last_sequence[seq_key] = event["sequence"]

        key = (event["boot_id"], event["pid_tgid"], event["handler"])
        if event["event_type"] == "kernel_handler_enter":
            counts["handler_entries_seen"] += 1
            active_count = sum(len(stack) for stack in pending.values())
            stack = pending.setdefault(key, [])
            if stack:
                counts["nested_handler_entries"] += 1
            if active_count >= max_pending:
                counts["pending_capacity_drops"] += 1
                unpaired.append({**event, "unpaired_reason": "pending_capacity_exceeded"})
                issues.append({"code": "pending_capacity_exceeded", "pid_tgid": event["pid_tgid"], "handler": event["handler"]})
                continue
            stack.append(event)
            continue

        counts["handler_returns_seen"] += 1
        stack = pending.get(key)
        entry = stack.pop() if stack else None
        if stack == []:
            pending.pop(key, None)
        if entry is None:
            counts["orphan_handler_returns"] += 1
            orphan_exits.append(event)
            issues.append({"code": "orphan_handler_return", "pid_tgid": event["pid_tgid"], "handler": event["handler"]})
            continue
        if event["monotonic_ns"] < entry["monotonic_ns"]:
            unpaired.append({**entry, "unpaired_reason": "handler_return_precedes_entry"})
            orphan_exits.append(event)
            issues.append({"code": "handler_return_precedes_entry", "pid_tgid": event["pid_tgid"], "handler": event["handler"]})
            continue

        pair = {
            "boot_id": event["boot_id"],
            "pid_tgid": event["pid_tgid"],
            "handler": event["handler"],
            "request_observation_stage": "post_copy_kernel_request",
            "entry_monotonic_ns": entry["monotonic_ns"],
            "return_monotonic_ns": event["monotonic_ns"],
            "duration_ns": event["monotonic_ns"] - entry["monotonic_ns"],
            "entry_cpu": entry["cpu"],
            "return_cpu": event["cpu"],
            "entry_sequence": entry["sequence"],
            "return_sequence": event["sequence"],
            "raw_modes": entry["raw_modes"],
            "mode_decode": entry["mode_decode"],
            "raw_clockid": entry.get("raw_clockid"),
            "raw_handler_return": event["return_value"],
            "handler_return_interpretation": _interpret_handler_return(event["return_value"]),
            "pairing_method": "lifo_same_thread_handler_stack",
            "outer_syscall_correlation": "not_joined",
            "clock_effect": "not_measured",
        }
        pairs.append(pair)
        counts["handler_pairs_completed"] += 1

    for stack in pending.values():
        for entry in stack:
            unpaired.append({**entry, "unpaired_reason": "no_matching_handler_return"})
            issues.append({
                "code": "handler_entry_without_return",
                "boot_id": entry["boot_id"],
                "pid_tgid": entry["pid_tgid"],
                "handler": entry["handler"],
            })

    return {
        "schema_version": 1,
        "reduction_status": "consistent" if not issues else "partial",
        "evidence_stage": "post_copy_kernel_request",
        "capture_coverage_status": "not_assessed",
        "clock_effect_status": "not_measured",
        "max_pending": max_pending,
        "pairing_policy": "lifo_same_thread_handler_stack_without_invocation_cookie",
        "counts": counts,
        "pairs": pairs,
        "unpaired_entries": unpaired,
        "orphan_returns": orphan_exits,
        "issues": issues,
        "interpretation": (
            "A paired internal handler observation represents the normalized "
            "kernel-owned request at the candidate hook. It is not joined to "
            "an outer syscall, does not establish hook coverage, and does not "
            "prove that a physical clock changed."
        ),
    }
