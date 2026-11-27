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

def test_multiple_handlers_cannot_claim_one_syscall():
    r=correlate_clock_adjustment_observations(*reports([sc()],[kh(),kh(start=160,end=260)]))
    assert "multiple_handlers_for_one_syscall" in [x["code"] for x in r["issues"]]
    assert r["counts"]["joined_pairs"]==0

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
