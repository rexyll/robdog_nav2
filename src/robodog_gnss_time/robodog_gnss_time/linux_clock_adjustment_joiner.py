"""Conservative joiner for reduced syscall and post-copy handler intervals."""
_HANDLERS_BY_SYSCALL = {
    "clock_adjtime": {"do_clock_adjtime", "do_adjtimex"},
    "clock_adjtime32": {"do_clock_adjtime", "do_adjtimex"},
    "adjtimex": {"do_adjtimex"},
    "adjtimex_time32": {"do_adjtimex"},
}
_TOP_LEVEL_HANDLER_BY_SYSCALL = {
    "clock_adjtime": "do_clock_adjtime",
    "clock_adjtime32": "do_clock_adjtime",
    "adjtimex": "do_adjtimex",
    "adjtimex_time32": "do_adjtimex",
}
_CONTRACT = "linux_v6.12_clock_adjtime_adjtimex_wrapper_order"


def _interval(item, start, end):
    return (isinstance(item, dict) and isinstance(item.get("boot_id"), str)
            and bool(item["boot_id"].strip())
            and isinstance(item.get("pid_tgid"), int) and not isinstance(item.get("pid_tgid"), bool)
            and item["pid_tgid"] > 0
            and isinstance(item.get(start), int) and not isinstance(item.get(start), bool)
            and isinstance(item.get(end), int) and not isinstance(item.get(end), bool)
            and 0 <= item[start] <= item[end])


def correlate_clock_adjustment_observations(syscall_report, handler_report, wrapper_contract_id=None):
    """Join same-thread intervals; ambiguous matches fail closed.

    Interval containment is not call-stack proof. Copy-back inference is opt-in
    and only available for the exact Linux v6.12 source-contract identifier.
    """
    if not isinstance(syscall_report, dict) or not isinstance(handler_report, dict):
        raise TypeError("reports must be mappings")
    if wrapper_contract_id not in (None, _CONTRACT):
        raise ValueError("unknown wrapper_contract_id")
    syscalls, handlers = syscall_report.get("pairs"), handler_report.get("pairs")
    if not isinstance(syscalls, list) or not isinstance(handlers, list):
        raise ValueError("both reports must contain pair lists")

    issues, ss, hs = [], [], []
    for i, item in enumerate(syscalls):
        if not _interval(item, "entry_monotonic_ns", "exit_monotonic_ns"):
            issues.append({"code":"invalid_syscall_pair","pair_index":i})
        elif item.get("syscall") not in _HANDLERS_BY_SYSCALL:
            issues.append({"code":"unsupported_syscall_for_join","pair_index":i})
        else:
            ss.append(item)
    for i, item in enumerate(handlers):
        if not _interval(item, "entry_monotonic_ns", "return_monotonic_ns"):
            issues.append({"code":"invalid_handler_pair","pair_index":i})
        elif item.get("handler") not in ("do_clock_adjtime","do_adjtimex"):
            issues.append({"code":"unsupported_handler_for_join","pair_index":i})
        elif not isinstance(item.get("raw_handler_return"),int) or isinstance(item.get("raw_handler_return"),bool):
            issues.append({"code":"invalid_handler_return_in_pair","pair_index":i})
        else:
            hs.append(item)

    choices=[]
    for hi,h in enumerate(hs):
        candidates=[]
        for si,s in enumerate(ss):
            if s["boot_id"] != h["boot_id"] or s["pid_tgid"] != h["pid_tgid"]:
                continue
            if h["handler"] not in _HANDLERS_BY_SYSCALL[s["syscall"]]:
                continue
            if s["entry_monotonic_ns"] <= h["entry_monotonic_ns"] and h["return_monotonic_ns"] <= s["exit_monotonic_ns"]:
                candidates.append((s["exit_monotonic_ns"]-s["entry_monotonic_ns"],si))
        if not candidates:
            choices.append(None)
            issues.append({"code":"handler_without_containing_syscall","handler_pair_index":hi})
            continue
        candidates.sort()
        width=candidates[0][0]
        best=[si for span,si in candidates if span==width]
        if len(best)!=1:
            choices.append(None)
            issues.append({"code":"ambiguous_nested_syscall_candidates","handler_pair_index":hi})
        else:
            choices.append(best[0])

    joins, unjoined, consumed = [], [], set()
    for hi, si in enumerate(choices):
        if si is None:
            unjoined.append(hi)
            continue
        s, h = ss[si], hs[hi]
        consumed.add(si)
        sr = s.get("raw_return_value")
        hr = h["raw_handler_return"]
        top_level_handler = _TOP_LEVEL_HANDLER_BY_SYSCALL[s["syscall"]]
        is_top_level = h["handler"] == top_level_handler
        copyback = (
            wrapper_contract_id == _CONTRACT
            and is_top_level
            and isinstance(sr, int) and not isinstance(sr, bool)
            and hr >= 0 and sr == -14
        )
        if h["handler"] == "do_clock_adjtime":
            relation = "syscall_dispatch_handler"
        elif s["syscall"] in ("clock_adjtime", "clock_adjtime32"):
            relation = "nested_clock_callback_handler"
        else:
            relation = "direct_syscall_handler"
        joins.append({
            "boot_id": s["boot_id"], "pid_tgid": s["pid_tgid"],
            "syscall": s["syscall"], "handler": h["handler"],
            "relation": relation,
            "syscall_entry_monotonic_ns": s["entry_monotonic_ns"],
            "syscall_exit_monotonic_ns": s["exit_monotonic_ns"],
            "handler_entry_monotonic_ns": h["entry_monotonic_ns"],
            "handler_return_monotonic_ns": h["return_monotonic_ns"],
            "raw_handler_return": hr, "raw_syscall_return": sr,
            "copyback_failure_inferred_from_pinned_wrapper": copyback,
            "wrapper_contract_id": wrapper_contract_id,
            "clock_effect": "not_measured",
        })
    unmatched=[i for i in range(len(ss)) if i not in consumed]
    for i in unmatched:
        issues.append({"code":"syscall_without_joined_handler","syscall_pair_index":i})
    upstream=(syscall_report.get("reduction_status")!="consistent"
              or handler_report.get("reduction_status")!="consistent")
    return {
        "schema_version":2,"reduction_status":"consistent" if not issues and not upstream else "partial",
        "wrapper_contract_id":wrapper_contract_id,"copyback_inference_enabled":wrapper_contract_id==_CONTRACT,
        "counts":{"syscall_pairs_input":len(syscalls),"handler_pairs_input":len(handlers),
                  "joined_pairs":len(joins),"unjoined_handler_pairs":len(set(unjoined)),
                  "unmatched_syscall_pairs":len(unmatched)},
        "joins":joins,"unjoined_handler_pair_indices":sorted(set(unjoined)),
        "unmatched_syscall_pair_indices":unmatched,"issues":issues,
        "interpretation":("A join establishes same-boot/thread interval containment under the "
          "syscall-to-handler mapping only; one syscall may legitimately contain multiple handler "
          "observations; it does not prove call-stack ancestry, capture "
          "coverage, or physical clock effect. Copy-back inference requires the explicit "
          "source-pinned wrapper contract.")}


def correlate_clock_adjustment_invocation_observations(syscall_report, handler_report):
    """Join schema-v2 pairs by exact producer-supplied parent invocation identity."""
    if not isinstance(syscall_report, dict) or not isinstance(handler_report, dict):
        raise TypeError("reports must be mappings")
    syscalls = syscall_report.get("pairs")
    handlers = handler_report.get("pairs")
    if not isinstance(syscalls, list) or not isinstance(handlers, list):
        raise ValueError("both reports must contain pair lists")

    issues, joins, unjoined = [], [], []
    syscall_index = {}
    for index, item in enumerate(syscalls):
        if not isinstance(item, dict):
            issues.append({"code": "invalid_syscall_invocation_pair", "pair_index": index})
            continue
        required = ("capture_session_id", "boot_id", "task_instance_id",
                    "invocation_id", "name")
        if any(not isinstance(item.get(key), str) or not item[key].strip()
               for key in required):
            issues.append({"code": "invalid_syscall_invocation_identity",
                           "pair_index": index})
            continue
        if item["name"] not in _HANDLERS_BY_SYSCALL:
            issues.append({"code": "unsupported_syscall_for_invocation_join",
                           "pair_index": index})
            continue
        key = (item["capture_session_id"], item["boot_id"],
               item["task_instance_id"], item["invocation_id"])
        syscall_index.setdefault(key, []).append((index, item))

    for handler_index, handler in enumerate(handlers):
        if not isinstance(handler, dict):
            issues.append({"code": "invalid_handler_invocation_pair",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        required = ("capture_session_id", "boot_id", "task_instance_id",
                    "invocation_id", "name")
        if any(not isinstance(handler.get(key), str) or not handler[key].strip()
               for key in required):
            issues.append({"code": "invalid_handler_invocation_identity",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        if handler["name"] not in ("do_clock_adjtime", "do_adjtimex"):
            issues.append({"code": "unsupported_handler_for_invocation_join",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        if handler.get("parent_context_status") != "matched":
            issues.append({"code": "handler_parent_context_unavailable",
                           "pair_index": handler_index,
                           "status": handler.get("parent_context_status")})
            unjoined.append(handler_index)
            continue
        parent_id = handler.get("parent_syscall_invocation_id")
        if not isinstance(parent_id, str) or not parent_id.strip():
            issues.append({"code": "handler_parent_invocation_id_missing",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        key = (handler["capture_session_id"], handler["boot_id"],
               handler["task_instance_id"], parent_id)
        candidates = syscall_index.get(key, [])
        compatible = [
            (index, syscall) for index, syscall in candidates
            if handler["name"] in _HANDLERS_BY_SYSCALL[syscall["name"]]
        ]
        if len(compatible) != 1:
            code = ("handler_parent_invocation_not_found" if not compatible
                    else "ambiguous_handler_parent_invocation")
            issues.append({"code": code, "pair_index": handler_index,
                           "parent_syscall_invocation_id": parent_id})
            unjoined.append(handler_index)
            continue
        syscall_pair_index, syscall = compatible[0]
        if handler.get("pid_tgid") != syscall.get("pid_tgid"):
            issues.append({"code": "handler_parent_pid_mismatch",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        if not (syscall["entry_monotonic_ns"] <= handler["entry_monotonic_ns"]
                <= handler["exit_monotonic_ns"] <= syscall["exit_monotonic_ns"]):
            issues.append({"code": "handler_outside_parent_syscall_interval",
                           "pair_index": handler_index})
            unjoined.append(handler_index)
            continue
        joins.append({
            "capture_session_id": syscall["capture_session_id"],
            "boot_id": syscall["boot_id"],
            "task_instance_id": syscall["task_instance_id"],
            "syscall_invocation_id": syscall["invocation_id"],
            "handler_invocation_id": handler["invocation_id"],
            "syscall": syscall["name"],
            "handler": handler["name"],
            "relation": ("syscall_dispatch_handler"
                         if handler["name"] == _TOP_LEVEL_HANDLER_BY_SYSCALL[syscall["name"]]
                         else "nested_clock_callback_handler"),
            "identity_join": "exact_parent_syscall_invocation_id",
            "clock_effect": "not_measured",
        })

    upstream = (syscall_report.get("reduction_status") != "consistent"
                or handler_report.get("reduction_status") != "consistent")
    if upstream:
        issues.append({"code": "upstream_invocation_reduction_not_consistent"})
    return {
        "schema_version": 1,
        "reduction_status": "consistent" if not issues and not upstream else "partial",
        "counts": {
            "syscall_pairs_input": len(syscalls),
            "handler_pairs_input": len(handlers),
            "joined_pairs": len(joins),
            "unjoined_handler_pairs": len(set(unjoined)),
        },
        "joins": joins,
        "unjoined_handler_pair_indices": sorted(set(unjoined)),
        "issues": issues,
        "identity_join": "exact_parent_syscall_invocation_id",
        "interpretation": (
            "The join uses producer-supplied parent invocation identity and verifies "
            "same task, PID and interval containment. It supports one syscall to "
            "multiple handler observations. It does not independently prove that "
            "the producer assigned truthful parent identities or measure clock effect."
        ),
    }



__all__=["correlate_clock_adjustment_observations", "correlate_clock_adjustment_invocation_observations"]
