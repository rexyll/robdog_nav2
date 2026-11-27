import pytest
from robodog_gnss_time.linux_clock_adjustment_joiner import correlate_clock_adjustment_observations

CONTRACT="linux_v6.12_clock_adjtime_adjtimex_wrapper_order"

def sc(name="clock_adjtime",start=100,end=300,ret=0,boot="b",tid=11):
    return {"boot_id":boot,"pid_tgid":tid,"syscall":name,"entry_monotonic_ns":start,
            "exit_monotonic_ns":end,"raw_return_value":ret}

def kh(name="do_clock_adjtime",start=150,end=250,ret=0,boot="b",tid=11):
    return {"boot_id":boot,"pid_tgid":tid,"handler":name,"entry_monotonic_ns":start,
            "return_monotonic_ns":end,"raw_handler_return":ret}

def reports(s=None,h=None,ss="consistent",hs="consistent"):
    return ({"reduction_status":ss,"pairs":s or []},{"reduction_status":hs,"pairs":h or []})

def test_join_same_thread_and_boot():
    r=correlate_clock_adjustment_observations(*reports([sc()],[kh()]))
    assert r["reduction_status"]=="consistent" and r["counts"]["joined_pairs"]==1
    assert r["joins"][0]["clock_effect"]=="not_measured"

@pytest.mark.parametrize("name,handler",[("clock_adjtime32","do_clock_adjtime"),("adjtimex","do_adjtimex"),("adjtimex_time32","do_adjtimex")])
def test_native_compat_mapping(name,handler):
    assert correlate_clock_adjustment_observations(*reports([sc(name)],[kh(handler)]))["counts"]["joined_pairs"]==1

def test_cross_thread_or_boot_never_joins():
    r=correlate_clock_adjustment_observations(*reports([sc(tid=1),sc(boot="other")],[kh(tid=2)]))
    assert r["counts"]["joined_pairs"]==0

def test_requires_full_interval_containment():
    r=correlate_clock_adjustment_observations(*reports([sc(end=200)],[kh(end=250)]))
    assert r["counts"]["joined_pairs"]==0

def test_narrowest_nested_interval_wins():
    r=correlate_clock_adjustment_observations(*reports([sc(start=50,end=400),sc()],[kh()]))
    assert r["joins"][0]["syscall_entry_monotonic_ns"]==100

def test_equal_width_candidates_are_ambiguous():
    r=correlate_clock_adjustment_observations(*reports([sc(),sc()],[kh()]))
    assert "ambiguous_nested_syscall_candidates" in [x["code"] for x in r["issues"]]

def test_one_syscall_can_have_multiple_handler_observations():
    r = correlate_clock_adjustment_observations(
        *reports(
            [sc(start=100, end=400)],
            [kh("do_clock_adjtime", start=120, end=380),
             kh("do_adjtimex", start=180, end=300)],
        )
    )
    assert r["reduction_status"] == "consistent"
    assert r["counts"]["joined_pairs"] == 2

def test_copyback_inference_off_by_default():
    r=correlate_clock_adjustment_observations(*reports([sc(ret=-14)],[kh()]))
    assert not r["joins"][0]["copyback_failure_inferred_from_pinned_wrapper"]

def test_copyback_inference_requires_pinned_contract_and_nonnegative_handler():
    a=correlate_clock_adjustment_observations(*reports([sc(ret=-14)],[kh()]),wrapper_contract_id=CONTRACT)
    b=correlate_clock_adjustment_observations(*reports([sc(ret=-14)],[kh(ret=-14)]),wrapper_contract_id=CONTRACT)
    assert a["joins"][0]["copyback_failure_inferred_from_pinned_wrapper"]
    assert not b["joins"][0]["copyback_failure_inferred_from_pinned_wrapper"]

def test_upstream_partial_propagates():
    r=correlate_clock_adjustment_observations(*reports([sc()],[kh()],hs="partial"))
    assert r["reduction_status"]=="partial"

def test_invalid_wrapper_id_and_report_shape_rejected():
    with pytest.raises(ValueError): correlate_clock_adjustment_observations(*reports(),wrapper_contract_id="other")
    with pytest.raises(ValueError): correlate_clock_adjustment_observations({"pairs":[]},{"pairs":None})

def test_bool_timestamp_rejected():
    r=correlate_clock_adjustment_observations(*reports([sc(start=True)],[kh()]))
    assert "invalid_syscall_pair" in [x["code"] for x in r["issues"]]


def test_clock_adjtime_can_own_dispatch_and_nested_adjtimex_handlers():
    syscalls = [sc("clock_adjtime", start=100, end=400, ret=-14)]
    handlers = [
        kh("do_clock_adjtime", start=120, end=380, ret=0),
        kh("do_adjtimex", start=180, end=300, ret=0),
    ]
    r = correlate_clock_adjustment_observations(
        *reports(syscalls, handlers), wrapper_contract_id=CONTRACT
    )
    assert r["schema_version"] == 2
    assert r["reduction_status"] == "consistent"
    assert r["counts"]["joined_pairs"] == 2
    by_handler = {item["handler"]: item for item in r["joins"]}
    assert by_handler["do_clock_adjtime"]["relation"] == "syscall_dispatch_handler"
    assert by_handler["do_clock_adjtime"]["copyback_failure_inferred_from_pinned_wrapper"]
    assert by_handler["do_adjtimex"]["relation"] == "nested_clock_callback_handler"
    assert not by_handler["do_adjtimex"]["copyback_failure_inferred_from_pinned_wrapper"]


def test_adjtimex_directly_owns_do_adjtimex_handler():
    r = correlate_clock_adjustment_observations(
        *reports([sc("adjtimex")], [kh("do_adjtimex")])
    )
    assert r["counts"]["joined_pairs"] == 1
    assert r["joins"][0]["relation"] == "direct_syscall_handler"


def test_clock_adjtime_does_not_require_nested_adjtimex_handler():
    r = correlate_clock_adjustment_observations(
        *reports([sc("clock_adjtime")], [kh("do_clock_adjtime")])
    )
    assert r["counts"]["joined_pairs"] == 1
    assert r["reduction_status"] == "consistent"


def invocation_sc(name, invocation_id, start, end):
    return {
        "capture_session_id": "c", "boot_id": "b", "task_instance_id": "t",
        "invocation_id": invocation_id, "pid_tgid": 11, "name": name,
        "entry_monotonic_ns": start, "exit_monotonic_ns": end,
    }


def invocation_kh(name, invocation_id, parent_id, start, end, status="matched"):
    return {
        "capture_session_id": "c", "boot_id": "b", "task_instance_id": "t",
        "invocation_id": invocation_id, "parent_syscall_invocation_id": parent_id,
        "parent_context_status": status, "pid_tgid": 11, "name": name,
        "entry_monotonic_ns": start, "exit_monotonic_ns": end,
    }


def test_invocation_join_uses_exact_parent_id_and_allows_one_to_many():
    from robodog_gnss_time.linux_clock_adjustment_joiner import (
        correlate_clock_adjustment_invocation_observations,
    )
    syscall = {"reduction_status": "consistent",
               "pairs": [invocation_sc("clock_adjtime", "sys-1", 100, 400)]}
    handlers = {"reduction_status": "consistent", "pairs": [
        invocation_kh("do_clock_adjtime", "h-1", "sys-1", 120, 380),
        invocation_kh("do_adjtimex", "h-2", "sys-1", 180, 300),
    ]}
    result = correlate_clock_adjustment_invocation_observations(syscall, handlers)
    assert result["reduction_status"] == "consistent"
    assert result["counts"]["joined_pairs"] == 2
    assert {x["identity_join"] for x in result["joins"]} == {
        "exact_parent_syscall_invocation_id"
    }


def test_invocation_join_rejects_wrong_parent_id_and_pid():
    from robodog_gnss_time.linux_clock_adjustment_joiner import (
        correlate_clock_adjustment_invocation_observations,
    )
    syscall = {"reduction_status": "consistent",
               "pairs": [invocation_sc("clock_adjtime", "sys-1", 100, 400)]}
    wrong_parent = invocation_kh("do_clock_adjtime", "h-1", "sys-other", 120, 380)
    result = correlate_clock_adjustment_invocation_observations(
        syscall, {"reduction_status": "consistent", "pairs": [wrong_parent]}
    )
    assert result["reduction_status"] == "partial"
    assert "handler_parent_invocation_not_found" in [x["code"] for x in result["issues"]]

    wrong_pid = invocation_kh("do_clock_adjtime", "h-2", "sys-1", 120, 380)
    wrong_pid["pid_tgid"] = 99
    result = correlate_clock_adjustment_invocation_observations(
        syscall, {"reduction_status": "consistent", "pairs": [wrong_pid]}
    )
    assert "handler_parent_pid_mismatch" in [x["code"] for x in result["issues"]]


def test_invocation_join_rejects_unavailable_parent_context():
    from robodog_gnss_time.linux_clock_adjustment_joiner import (
        correlate_clock_adjustment_invocation_observations,
    )
    syscall = {"reduction_status": "consistent",
               "pairs": [invocation_sc("clock_adjtime", "sys-1", 100, 400)]}
    handler = invocation_kh("do_clock_adjtime", "h-1", None, 120, 380,
                            status="stack_overflow")
    result = correlate_clock_adjustment_invocation_observations(
        syscall, {"reduction_status": "consistent", "pairs": [handler]}
    )
    assert result["reduction_status"] == "partial"
    assert "handler_parent_context_unavailable" in [x["code"] for x in result["issues"]]


@pytest.mark.parametrize("syscall_name,handler_name,expected_relation", [
    ("clock_adjtime", "do_clock_adjtime", "syscall_dispatch_handler"),
    ("clock_adjtime32", "do_adjtimex", "nested_clock_callback_handler"),
    ("adjtimex", "do_adjtimex", "direct_syscall_handler"),
    ("adjtimex_time32", "do_adjtimex", "direct_syscall_handler"),
])
def test_invocation_join_relation_labels_follow_native_and_compat_mapping(
    syscall_name, handler_name, expected_relation
):
    from robodog_gnss_time.linux_clock_adjustment_joiner import (
        correlate_clock_adjustment_invocation_observations,
    )
    syscall = {"reduction_status": "consistent", "pairs": [
        invocation_sc(syscall_name, "sys-1", 100, 400)
    ]}
    handler = {"reduction_status": "consistent", "pairs": [
        invocation_kh(handler_name, "h-1", "sys-1", 120, 380)
    ]}
    result = correlate_clock_adjustment_invocation_observations(syscall, handler)
    assert result["reduction_status"] == "consistent"
    assert result["joins"][0]["relation"] == expected_relation
