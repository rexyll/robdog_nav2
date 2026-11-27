"""Pair schema-v2 syscall/handler events by producer-assigned invocation identity.

This is an offline reducer. It does not collect events or infer missing IDs.
"""
from .linux_clock_adjustment_session import validate_clock_adjustment_event_envelope
from .linux_timex_modes import classify_timex_modes


def reduce_clock_adjustment_invocation_events(records, stream_kind):
    """Pair entry/exit events by exact invocation ID, independent of CPU/arrival order.

    Schema-v1 records are deliberately not upgraded: they have no producer-
    assigned invocation identity and must continue through the legacy reducers.
    """
    if not isinstance(records, list):
        raise TypeError("records must be a list")
    if stream_kind not in ("syscall", "kernel_handler"):
        raise ValueError("unsupported stream_kind")

    issues = []
    event_ids = set()
    invocations = {}
    valid_count = 0

    for index, record in enumerate(records):
        result = validate_clock_adjustment_event_envelope(record)
        if result.get("status") != "valid":
            issues.append({"code": "invalid_event_envelope", "record_index": index,
                           "details": result.get("issues", [])})
            continue
        identity = result["identity"]
        if identity.get("schema_version") != 2:
            issues.append({"code": "legacy_schema_without_invocation_identity",
                           "record_index": index})
            continue
        if identity.get("stream_kind") != stream_kind:
            issues.append({"code": "stream_kind_mismatch", "record_index": index})
            continue

        event_id_key = (identity["capture_session_id"], identity["boot_id"],
                        identity["event_id"])
        if event_id_key in event_ids:
            issues.append({"code": "duplicate_event_id", "record_index": index,
                           "event_id": identity["event_id"]})
            continue
        event_ids.add(event_id_key)

        event = result["event"]
        event_type = event.get("event_type")
        entry_type, exit_type = (
            ("syscall_enter", "syscall_exit") if stream_kind == "syscall"
            else ("kernel_handler_enter", "kernel_handler_exit")
        )
        if event_type not in (entry_type, exit_type):
            issues.append({"code": "invalid_invocation_event_type", "record_index": index})
            continue

        name_field = "syscall" if stream_kind == "syscall" else "handler"
        name = event.get(name_field)
        pid_tgid = event.get("pid_tgid")
        if (not isinstance(name, str) or not name.strip()
                or not isinstance(pid_tgid, int) or isinstance(pid_tgid, bool)
                or pid_tgid <= 0):
            issues.append({"code": "invalid_invocation_subject", "record_index": index})
            continue

        key = (
            identity["capture_session_id"], identity["boot_id"],
            identity["producer_id"], identity["stream_id"],
            identity["task_instance_id"], identity["invocation_id"],
        )
        slot = invocations.setdefault(key, {
            "name": name, "pid_tgid": pid_tgid, "entry": None, "exit": None,
            "parent_syscall_invocation_id": event.get("parent_syscall_invocation_id"),
            "parent_context_status": event.get("parent_context_status"),
            "identity": identity,
        })
        if slot["name"] != name or slot["pid_tgid"] != pid_tgid:
            issues.append({"code": "invocation_subject_mismatch", "record_index": index,
                           "invocation_id": identity["invocation_id"]})
            continue
        if stream_kind == "kernel_handler" and (
            slot["parent_syscall_invocation_id"] != event.get("parent_syscall_invocation_id")
            or slot["parent_context_status"] != event.get("parent_context_status")
        ):
            issues.append({"code": "invocation_parent_context_mismatch",
                           "record_index": index,
                           "invocation_id": identity["invocation_id"]})
            continue

        side = "entry" if event_type == entry_type else "exit"
        if side == "entry":
            if stream_kind == "syscall":
                raw_clockid = event.get("raw_clockid")
                modes_status = event.get("modes_read_status", "not_requested")
                if raw_clockid is not None and (
                    not isinstance(raw_clockid, int) or isinstance(raw_clockid, bool)
                    or not -(1 << 31) <= raw_clockid <= (1 << 31) - 1
                ):
                    issues.append({"code": "invalid_invocation_raw_clockid",
                                   "record_index": index})
                    continue
                if not isinstance(modes_status, str) or not modes_status or len(modes_status) > 64:
                    issues.append({"code": "invalid_invocation_modes_status",
                                   "record_index": index})
                    continue
                raw_modes = event.get("raw_modes")
                if modes_status == "ok" and (
                    not isinstance(raw_modes, int) or isinstance(raw_modes, bool)
                    or not 0 <= raw_modes <= 0xFFFFFFFF
                ):
                    issues.append({"code": "invalid_invocation_raw_modes",
                                   "record_index": index})
                    continue
                entry_payload = {
                    "raw_clockid": raw_clockid,
                    "modes_read_status": modes_status,
                    "raw_modes": raw_modes if modes_status == "ok" else None,
                    "mode_decode": classify_timex_modes(raw_modes) if modes_status == "ok" else None,
                }
            else:
                raw_modes = event.get("raw_modes")
                raw_clockid = event.get("raw_clockid")
                if (not isinstance(raw_modes, int) or isinstance(raw_modes, bool)
                        or not 0 <= raw_modes <= 0xFFFFFFFF):
                    issues.append({"code": "invalid_invocation_handler_modes",
                                   "record_index": index})
                    continue
                if name == "do_clock_adjtime":
                    if (not isinstance(raw_clockid, int) or isinstance(raw_clockid, bool)
                            or not -(1 << 31) <= raw_clockid <= (1 << 31) - 1):
                        issues.append({"code": "invalid_invocation_handler_clockid",
                                       "record_index": index})
                        continue
                elif raw_clockid is not None:
                    issues.append({"code": "unexpected_invocation_handler_clockid",
                                   "record_index": index})
                    continue
                entry_payload = {
                    "raw_modes": raw_modes,
                    "mode_decode": classify_timex_modes(raw_modes),
                    "raw_clockid": raw_clockid,
                }
        else:
            return_value = event.get("return_value")
            if (not isinstance(return_value, int) or isinstance(return_value, bool)
                    or not -(1 << 63) <= return_value <= (1 << 63) - 1):
                issues.append({"code": "invalid_invocation_return_value",
                               "record_index": index})
                continue
            exit_payload = {"return_value": return_value}

        if slot[side] is not None:
            issues.append({"code": "duplicate_invocation_side", "record_index": index,
                           "side": side, "invocation_id": identity["invocation_id"]})
            continue
        slot[side] = {
            "monotonic_ns": event["monotonic_ns"],
            "cpu": event["cpu"],
            "sequence": event["sequence"],
            "event_id": event["event_id"],
            **(entry_payload if side == "entry" else exit_payload),
        }
        valid_count += 1

    pairs, unmatched = [], []
    for key, slot in invocations.items():
        if slot["entry"] is None or slot["exit"] is None:
            unmatched.append({
                "invocation_id": key[-1],
                "missing": "entry" if slot["entry"] is None else "exit",
                "name": slot["name"],
            })
            issues.append({"code": "unmatched_invocation_side",
                           "invocation_id": key[-1],
                           "missing": "entry" if slot["entry"] is None else "exit"})
            continue
        if slot["entry"]["monotonic_ns"] > slot["exit"]["monotonic_ns"]:
            issues.append({"code": "invocation_time_order_mismatch",
                           "invocation_id": key[-1]})
            continue
        pairs.append({
            "capture_session_id": key[0],
            "boot_id": key[1],
            "producer_id": key[2],
            "stream_id": key[3],
            "task_instance_id": key[4],
            "invocation_id": key[5],
            "pid_tgid": slot["pid_tgid"],
            "name": slot["name"],
            "entry_monotonic_ns": slot["entry"]["monotonic_ns"],
            "exit_monotonic_ns": slot["exit"]["monotonic_ns"],
            "entry_cpu": slot["entry"]["cpu"],
            "exit_cpu": slot["exit"]["cpu"],
            "entry_event_id": slot["entry"]["event_id"],
            "exit_event_id": slot["exit"]["event_id"],
            "parent_syscall_invocation_id": slot["parent_syscall_invocation_id"],
            "parent_context_status": slot["parent_context_status"],
            "raw_clockid": slot["entry"].get("raw_clockid"),
            "modes_read_status": slot["entry"].get("modes_read_status"),
            "raw_modes": slot["entry"].get("raw_modes"),
            "mode_decode": slot["entry"].get("mode_decode"),
            "raw_return_value": slot["exit"]["return_value"],
            "raw_handler_return": (slot["exit"]["return_value"]
                                   if stream_kind == "kernel_handler" else None),
            "return_interpretation": (
                "nonnegative_return_not_effect_proof"
                if slot["exit"]["return_value"] >= 0
                else ("negative_efault_copy_stage_ambiguous"
                      if stream_kind == "syscall" and slot["exit"]["return_value"] == -14
                      else ("negative_efault_handler_result_not_copyback_proof"
                            if stream_kind == "kernel_handler"
                            and slot["exit"]["return_value"] == -14
                            else "negative_errno_effect_unresolved"))
            ),
            "duration_ns": slot["exit"]["monotonic_ns"] - slot["entry"]["monotonic_ns"],
            "clock_effect": "not_measured",
        })

    return {
        "schema_version": 1,
        "stream_kind": stream_kind,
        "reduction_status": "consistent" if not issues else "partial",
        "counts": {
            "records_input": len(records),
            "records_valid_v2": valid_count,
            "invocations_observed": len(invocations),
            "pairs": len(pairs),
            "unmatched_invocations": len(unmatched),
        },
        "pairs": pairs,
        "unmatched": unmatched,
        "issues": issues,
        "pairing_policy": "exact_producer_assigned_invocation_id",
        "interpretation": (
            "Pairs are matched by producer-assigned invocation identity, not CPU "
            "sequence or userspace arrival order. This does not prove producer "
            "coverage, truthful identity allocation, or physical clock effect."
        ),
    }


__all__ = ["reduce_clock_adjustment_invocation_events"]
