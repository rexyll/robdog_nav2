"""Conservative joiner for reduced syscall and post-copy handler intervals."""
_EXPECTED = {"clock_adjtime":"do_clock_adjtime","clock_adjtime32":"do_clock_adjtime",
             "adjtimex":"do_adjtimex","adjtimex_time32":"do_adjtimex"}
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
        elif item.get("syscall") not in _EXPECTED:
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
            if _EXPECTED[s["syscall"]] != h["handler"]:
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

    owners={}
    for hi,si in enumerate(choices):
        if si is not None:
            owners.setdefault(si,[]).append(hi)
    joins, unjoined, consumed=[],[],set()
    for si,his in owners.items():
        if len(his)!=1:
            issues.append({"code":"multiple_handlers_for_one_syscall","syscall_pair_index":si})
            unjoined.extend(his)
            continue
        hi=his[0]; s=ss[si]; h=hs[hi]
        consumed.add(si)
        sr=s.get("raw_return_value"); hr=h["raw_handler_return"]
        copyback=(wrapper_contract_id==_CONTRACT and isinstance(sr,int)
                  and not isinstance(sr,bool) and hr>=0 and sr==-14)
        joins.append({
            "boot_id":s["boot_id"],"pid_tgid":s["pid_tgid"],"syscall":s["syscall"],
            "handler":h["handler"],"syscall_entry_monotonic_ns":s["entry_monotonic_ns"],
            "syscall_exit_monotonic_ns":s["exit_monotonic_ns"],
            "handler_entry_monotonic_ns":h["entry_monotonic_ns"],
            "handler_return_monotonic_ns":h["return_monotonic_ns"],
            "raw_handler_return":hr,"raw_syscall_return":sr,
            "copyback_failure_inferred_from_pinned_wrapper":copyback,
            "wrapper_contract_id":wrapper_contract_id,"clock_effect":"not_measured"})
    for hi,si in enumerate(choices):
        if si is None or (si in owners and len(owners[si])!=1):
            if hi not in unjoined: unjoined.append(hi)
    unmatched=[i for i in range(len(ss)) if i not in consumed]
    for i in unmatched:
        issues.append({"code":"syscall_without_joined_handler","syscall_pair_index":i})
    upstream=(syscall_report.get("reduction_status")!="consistent"
              or handler_report.get("reduction_status")!="consistent")
    return {
        "schema_version":1,"reduction_status":"consistent" if not issues and not upstream else "partial",
        "wrapper_contract_id":wrapper_contract_id,"copyback_inference_enabled":wrapper_contract_id==_CONTRACT,
        "counts":{"syscall_pairs_input":len(syscalls),"handler_pairs_input":len(handlers),
                  "joined_pairs":len(joins),"unjoined_handler_pairs":len(set(unjoined)),
                  "unmatched_syscall_pairs":len(unmatched)},
        "joins":joins,"unjoined_handler_pair_indices":sorted(set(unjoined)),
        "unmatched_syscall_pair_indices":unmatched,"issues":issues,
        "interpretation":("A join establishes same-boot/thread interval containment under the "
          "syscall-to-handler mapping only; it does not prove call-stack ancestry, capture "
          "coverage, or physical clock effect. Copy-back inference requires the explicit "
          "source-pinned wrapper contract.")}


__all__=["correlate_clock_adjustment_observations"]
