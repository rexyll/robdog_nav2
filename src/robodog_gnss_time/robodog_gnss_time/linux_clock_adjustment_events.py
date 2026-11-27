"""Pure userspace reducer for synthetic Linux clock-adjustment syscall events.

The reducer pairs observations and accounts for visible sequence gaps. It is not
a kernel capture agent and cannot prove that the input event stream is complete.
"""

from typing import Any, Iterable

from .linux_timex_modes import classify_timex_modes


_EVENT_TYPES = {"syscall_enter", "syscall_exit"}
_INT64_MIN = -(1 << 63)
_INT64_MAX = (1 << 63) - 1
_UINT64_MAX = (1 << 64) - 1


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _validate_common_event(event: dict[str, Any], index: int) -> tuple[dict[str, Any] | None, list[dict[str, Any]]]:
    issues: list[dict[str, Any]] = []
    event_type = event.get("event_type")
    if event_type not in _EVENT_TYPES:
        issues.append({"code": "unknown_event_type", "event_index": index})
        return None, issues

    boot_id = event.get("boot_id")
    if not isinstance(boot_id, str) or not boot_id.strip() or len(boot_id) > 128:
        issues.append({"code": "invalid_boot_id", "event_index": index})
    monotonic_ns = event.get("monotonic_ns")
    if not _is_int(monotonic_ns) or monotonic_ns < 0:
        issues.append({"code": "invalid_monotonic_timestamp", "event_index": index})
    pid_tgid = event.get("pid_tgid")
    if not _is_int(pid_tgid) or pid_tgid <= 0 or pid_tgid > _UINT64_MAX:
        issues.append({"code": "invalid_pid_tgid", "event_index": index})
    syscall = event.get("syscall")
    if not isinstance(syscall, str) or not syscall.strip() or len(syscall) > 128:
        issues.append({"code": "invalid_syscall_name", "event_index": index})
    cpu = event.get("cpu")
    if not _is_int(cpu) or cpu < 0:
        issues.append({"code": "invalid_cpu", "event_index": index})
    sequence = event.get("sequence")
    if not _is_int(sequence) or sequence < 0 or sequence > _UINT64_MAX:
        issues.append({"code": "invalid_sequence", "event_index": index})

    if event_type == "syscall_exit":
        return_value = event.get("return_value")
        if not _is_int(return_value) or not _INT64_MIN <= return_value <= _INT64_MAX:
            issues.append({"code": "invalid_return_value", "event_index": index})

    if issues:
        return None, issues

    projected = {
        "event_type": event_type,
        "boot_id": boot_id,
        "monotonic_ns": monotonic_ns,
        "pid_tgid": pid_tgid,
        "syscall": syscall,
        "cpu": cpu,
        "sequence": sequence,
    }
    if event_type == "syscall_enter":
        projected["raw_clockid"] = event.get("raw_clockid")
        modes_status = event.get("modes_read_status", "not_requested")
        projected["modes_read_status"] = modes_status
        if modes_status == "ok":
            raw_modes = event.get("raw_modes")
            if _is_int(raw_modes) and 0 <= raw_modes <= 0xFFFFFFFF:
                projected["raw_modes"] = raw_modes
                projected["mode_decode"] = classify_timex_modes(raw_modes)
            else:
                projected["modes_read_status"] = "invalid_value"
                issues.append({"code": "invalid_raw_modes", "event_index": index})
        else:
            projected["mode_decode"] = None
    else:
        projected["return_value"] = return_value
    return projected, issues


def _return_interpretation(return_value: int) -> str:
    if return_value >= 0:
        return "nonnegative_handler_return_not_effect_proof"
    if return_value == -14:  # Linux EFAULT
        return "negative_efault_copy_stage_ambiguous"
    return "negative_errno_effect_unresolved"


def reduce_clock_adjustment_events(
    events: Iterable[dict[str, Any]],
    max_pending: int = 4096,
) -> dict[str, Any]:
    """Pair syscall entry/exit observations and report visible evidence gaps.

    Pair identity includes boot ID, pid_tgid (TGID+TID), and syscall name.
    Per-CPU sequence tracking is isolated by boot ID and CPU. No wall-clock
    timestamps, raw user pointers, or unrecognized event payloads are echoed.
    """
    if isinstance(max_pending, bool) or not isinstance(max_pending, int):
        raise TypeError("max_pending must be an integer, not bool")
    if max_pending < 1:
        raise ValueError("max_pending must be at least 1")

    pending: dict[tuple[str, int, str], dict[str, Any]] = {}
    last_sequence: dict[tuple[str, int], int] = {}
    pairs: list[dict[str, Any]] = []
    unpaired_entries: list[dict[str, Any]] = []
    orphan_exits: list[dict[str, Any]] = []
    sequence_gaps: list[dict[str, Any]] = []
    issues: list[dict[str, Any]] = []

    counts = {
        "input_events": 0,
        "accepted_events": 0,
        "malformed_events": 0,
        "syscall_entry_seen": 0,
        "syscall_exit_seen": 0,
        "entry_pair_completed": 0,
        "orphan_exit": 0,
        "inflight_overwrite": 0,
        "inflight_capacity_drop": 0,
    }

    if not isinstance(events, Iterable) or isinstance(events, (str, bytes, dict)):
        raise TypeError("events must be an iterable of event mappings")

    for index, raw_event in enumerate(events):
        counts["input_events"] += 1
        if not isinstance(raw_event, dict):
            counts["malformed_events"] += 1
            issues.append({"code": "event_not_mapping", "event_index": index})
            continue

        event, event_issues = _validate_common_event(raw_event, index)
        if event_issues:
            issues.extend(event_issues)
        if event is None:
            counts["malformed_events"] += 1
            continue
        counts["accepted_events"] += 1

        seq_key = (event["boot_id"], event["cpu"])
        previous_sequence = last_sequence.get(seq_key)
        if previous_sequence is not None:
            if event["sequence"] > previous_sequence + 1:
                gap = {
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "previous_sequence": previous_sequence,
                    "current_sequence": event["sequence"],
                    "missing_count": event["sequence"] - previous_sequence - 1,
                }
                sequence_gaps.append(gap)
                issues.append({"code": "per_cpu_sequence_gap", **gap})
            elif event["sequence"] == previous_sequence:
                issues.append({
                    "code": "duplicate_per_cpu_sequence",
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "sequence": event["sequence"],
                })
            elif event["sequence"] < previous_sequence:
                issues.append({
                    "code": "per_cpu_sequence_regression",
                    "boot_id": event["boot_id"],
                    "cpu": event["cpu"],
                    "previous_sequence": previous_sequence,
                    "current_sequence": event["sequence"],
                })
        if previous_sequence is None or event["sequence"] > previous_sequence:
            last_sequence[seq_key] = event["sequence"]

        key = (event["boot_id"], event["pid_tgid"], event["syscall"])
        if event["event_type"] == "syscall_enter":
            counts["syscall_entry_seen"] += 1
            if key in pending:
                previous_entry = pending.pop(key)
                unpaired_entries.append({
                    **previous_entry,
                    "unpaired_reason": "superseded_by_duplicate_entry",
                })
                counts["inflight_overwrite"] += 1
                issues.append({"code": "duplicate_entry_overwrite", "pid_tgid": event["pid_tgid"], "syscall": event["syscall"]})
            if len(pending) >= max_pending:
                counts["inflight_capacity_drop"] += 1
                unpaired_entries.append({
                    **event,
                    "unpaired_reason": "inflight_capacity_exceeded",
                })
                issues.append({"code": "inflight_capacity_exceeded", "pid_tgid": event["pid_tgid"], "syscall": event["syscall"]})
                continue
            pending[key] = event
            continue

        counts["syscall_exit_seen"] += 1
        entry = pending.pop(key, None)
        if entry is None:
            counts["orphan_exit"] += 1
            orphan_exits.append(event)
            issues.append({
                "code": "orphan_exit",
                "boot_id": event["boot_id"],
                "pid_tgid": event["pid_tgid"],
                "syscall": event["syscall"],
            })
            continue

        if event["monotonic_ns"] < entry["monotonic_ns"]:
            unpaired_entries.append({
                **entry,
                "unpaired_reason": "exit_timestamp_precedes_entry",
            })
            orphan_exits.append(event)
            issues.append({
                "code": "exit_timestamp_precedes_entry",
                "pid_tgid": event["pid_tgid"],
                "syscall": event["syscall"],
            })
            continue

        return_value = event["return_value"]
        pair = {
            "boot_id": event["boot_id"],
            "pid_tgid": event["pid_tgid"],
            "syscall": event["syscall"],
            "entry_monotonic_ns": entry["monotonic_ns"],
            "exit_monotonic_ns": event["monotonic_ns"],
            "duration_ns": event["monotonic_ns"] - entry["monotonic_ns"],
            "entry_cpu": entry["cpu"],
            "exit_cpu": event["cpu"],
            "entry_sequence": entry["sequence"],
            "exit_sequence": event["sequence"],
            "raw_clockid": entry.get("raw_clockid"),
            "modes_read_status": entry.get("modes_read_status", "not_requested"),
            "raw_modes": entry.get("raw_modes"),
            "mode_decode": entry.get("mode_decode"),
            "raw_return_value": return_value,
            "return_interpretation": _return_interpretation(return_value),
            "clock_effect": "not_measured",
            "clock_target_resolution": "not_inferred",
        }
        pairs.append(pair)
        counts["entry_pair_completed"] += 1

    for entry in pending.values():
        unpaired_entries.append({
            **entry,
            "unpaired_reason": "no_matching_exit_in_input",
        })
        issues.append({
            "code": "entry_without_exit",
            "boot_id": entry["boot_id"],
            "pid_tgid": entry["pid_tgid"],
            "syscall": entry["syscall"],
        })

    reduction_status = "consistent" if not issues else "partial"
    return {
        "schema_version": 1,
        "reduction_status": reduction_status,
        "capture_coverage_status": "not_assessed",
        "clock_effect_status": "not_measured",
        "max_pending": max_pending,
        "counts": counts,
        "pairs": pairs,
        "unpaired_entries": unpaired_entries,
        "orphan_exits": orphan_exits,
        "sequence_gaps": sequence_gaps,
        "issues": issues,
        "interpretation": (
            "This reducer only validates and correlates the supplied records. "
            "A consistent result does not prove that the kernel capture stream "
            "was complete, that no events were lost, or that a requested clock "
            "operation changed a physical clock."
        ),
    }
