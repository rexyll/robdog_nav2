import subprocess
from pathlib import Path

import pytest

from robodog_gnss_time.linuxptp_host_evidence import (
    LinuxPtpHostEvidenceConfig,
    collect_linuxptp_host_evidence,
    parse_linuxptp_journal_evidence,
)


def _pmc_output(domain_number=5):
    return f"""sending: GET TIME_STATUS_NP
080027.fffe.7f327b-0 seq 10 RESPONSE MANAGEMENT TIME_STATUS_NP
    master_offset -20
    gmPresent true
    gmIdentity 080027.fffe.d9e453
sending: GET CURRENT_DATA_SET
080027.fffe.7f327b-0 seq 11 RESPONSE MANAGEMENT CURRENT_DATA_SET
    stepsRemoved 1
    offsetFromMaster 20.0
    meanPathDelay 300.0
sending: GET PORT_DATA_SET
080027.fffe.7f327b-1 seq 12 RESPONSE MANAGEMENT PORT_DATA_SET
    portIdentity 080027.fffe.7f327b-1
    portState SLAVE
sending: GET PARENT_DATA_SET
080027.fffe.7f327b-0 seq 13 RESPONSE MANAGEMENT PARENT_DATA_SET
    parentPortIdentity 080027.fffe.1db84e-2
    parentStats 0
    observedParentOffsetScaledLogVariance 0xffff
    observedParentClockPhaseChangeRate 0x7fffffff
    grandmasterPriority1 128
    gm.ClockClass 6
    gm.ClockAccuracy 0x21
    gm.OffsetScaledLogVariance 0x0000
    grandmasterPriority2 128
    grandmasterIdentity 080027.fffe.d9e453
sending: GET TIME_PROPERTIES_DATA_SET
080027.fffe.7f327b-0 seq 14 RESPONSE MANAGEMENT TIME_PROPERTIES_DATA_SET
    currentUtcOffset 37
    leap61 0
    leap59 0
    currentUtcOffsetValid 1
    ptpTimescale 1
    timeTraceable 1
    frequencyTraceable 1
    timeSource 0x20
sending: GET DEFAULT_DATA_SET
080027.fffe.7f327b-0 seq 15 RESPONSE MANAGEMENT DEFAULT_DATA_SET
    twoStepFlag 1
    slaveOnly 0
    numberPorts 1
    priority1 128
    clockClass 248
    clockAccuracy 0xfe
    offsetScaledLogVariance 0xffff
    priority2 128
    clockIdentity 080027.fffe.7f327b
    domainNumber {domain_number}
"""


def _config(**overrides):
    values = {
        "interface": "enp2s0",
        "domain_number": 5,
        "pmc_socket": "/run/ptp4l",
        "transport_specific": 0,
        "target_port_identity": "",
        "ptp4l_service": "ptp4l.service",
        "phc2sys_service": "phc2sys.service",
        "chrony_service": "chrony.service",
    }
    values.update(overrides)
    return LinuxPtpHostEvidenceConfig(**values)



def _journal_output(boot_id=None, message="port 1 (eth0): LISTENING to SLAVE on ANNOUNCE_RECEIPT_TIMEOUT"):
    if boot_id is None:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    import json
    return json.dumps({
        "_BOOT_ID": boot_id,
        "__MONOTONIC_TIMESTAMP": "1000000",
        "__REALTIME_TIMESTAMP": "1795700000000000",
        "__CURSOR": "s=cursor",
        "_SYSTEMD_UNIT": "ptp4l.service",
        "MESSAGE": message,
    }) + "\n"




def _journald_suppression_output(boot_id=None):
    if boot_id is None:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    import json
    return json.dumps({
        "_BOOT_ID": boot_id,
        "__MONOTONIC_TIMESTAMP": "1000001",
        "__REALTIME_TIMESTAMP": "1795700000000001",
        "__CURSOR": "s=suppression",
        "_SYSTEMD_UNIT": "systemd-journald.service",
        "MESSAGE_ID": "a596d6fe7bfa4994828e72309e95d61e",
        "MESSAGE": "Suppressed 17 messages from /system.slice/ptp4l.service",
    }) + "\n"



def test_config_requires_explicit_interface_domain_and_socket():
    with pytest.raises(ValueError):
        LinuxPtpHostEvidenceConfig("", 0, "/run/ptp4l", 0)
    with pytest.raises(ValueError):
        LinuxPtpHostEvidenceConfig("eth0", True, "/run/ptp4l", 0)
    with pytest.raises(ValueError):
        LinuxPtpHostEvidenceConfig("eth0", 256, "/run/ptp4l", 0)
    with pytest.raises(ValueError):
        LinuxPtpHostEvidenceConfig("eth0", 0, "ptp4l.sock", 0)


def test_config_rejects_invalid_service_unit_and_timeout():
    with pytest.raises(ValueError):
        _config(ptp4l_service="--set-clock")
    with pytest.raises(ValueError):
        _config(timeout_s=0)
    with pytest.raises(ValueError):
        _config(timeout_s=float("inf"))


def test_collector_uses_get_only_argv_and_preserves_raw_snapshot_evidence():
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    pmc_call = next(call for call in calls if call[0][0] == "pmc")
    pmc_argv, pmc_kwargs = pmc_call
    commands = pmc_argv[pmc_argv.index("-b") + 2:]
    assert pmc_argv[pmc_argv.index("-t") + 1] == "0x0"
    assert commands == [
        "GET TIME_STATUS_NP",
        "GET CURRENT_DATA_SET",
        "GET PORT_DATA_SET",
        "GET PARENT_DATA_SET",
        "GET TIME_PROPERTIES_DATA_SET",
        "GET DEFAULT_DATA_SET",
    ]
    assert all(command.startswith("GET ") for command in commands)
    assert pmc_kwargs["shell"] is False
    assert report["pmc_snapshot_validation"]["status"] == "valid"
    assert report["schema_version"] == 3
    assert report["pmc_snapshot_validation"]["clock_identity"] == "080027.fffe.7f327b"
    assert report["expected_transport_specific"] == 0
    assert report["expected_target_port_identity"] is None
    assert report["host_time_sync_claim"] == "not_evaluated"
    assert report["evidence_kind"] == "read_only_linuxptp_host_inventory"
    assert all(call[1]["shell"] is False for call in calls)
    assert any(call[0][0] == "systemctl" for call in calls)
    show_call = next(call[0] for call in calls if call[0][:2] == ["systemctl", "show"])
    assert "--property=Id,LoadState,ActiveState,SubState,ExecStart,FragmentPath,DropInPaths" in show_call
    journal_call = next(call[0] for call in calls if call[0][0] == "journalctl")
    assert "--output=json" in journal_call
    assert "--all" in journal_call
    assert report["journal_event_evidence"]["ptp4l.service"]["coverage_status"] == "unknown"
    assert report["journal_event_evidence"]["ptp4l.service"]["event_count"] == 1


def test_collector_rejects_snapshot_domain_mismatch_without_claiming_sync():
    def runner(argv, **kwargs):
        stdout = _pmc_output(domain_number=4) if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["pmc_snapshot_validation"]["status"] == "invalid"
    assert "domain mismatch" in report["pmc_snapshot_validation"]["error"]
    assert report["host_time_sync_claim"] == "not_evaluated"


def test_collector_records_unavailable_and_timeout_without_shell_fallback():
    def runner(argv, **kwargs):
        if argv[0] == "ethtool":
            raise FileNotFoundError("ethtool missing")
        if argv[0] == "timedatectl":
            raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    by_name = {item["name"]: item for item in report["commands"]}
    assert by_name["ethtool_timestamping"]["status"] == "unavailable"
    assert by_name["timedatectl"]["status"] == "timeout"
    assert by_name["pmc_get_snapshot"]["status"] == "ok"


def test_collector_does_not_guess_service_names_when_not_configured():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    config = LinuxPtpHostEvidenceConfig("eth0", 5, "/run/custom-ptp.sock", 0)
    report = collect_linuxptp_host_evidence(config, runner=runner)
    assert not any(argv[0] in {"systemctl", "journalctl"} for argv in calls)
    assert report["interface"] == "eth0"
    assert report["pmc_socket"] == "/run/custom-ptp.sock"


@pytest.mark.parametrize("transport_specific", [True, -1, 16, "0"])
def test_config_rejects_invalid_transport_specific(transport_specific):
    with pytest.raises(ValueError):
        _config(transport_specific=transport_specific)

def test_collector_targets_only_explicit_port_before_fixed_get_requests():
    target = "080027.fffe.7f327b-1"
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(
        _config(target_port_identity=target),
        runner=runner,
    )
    pmc_argv = next(argv for argv in calls if argv[0] == "pmc")
    command_start = pmc_argv.index(f"TARGET {target}")
    assert pmc_argv[command_start + 1:] == [
        "GET TIME_STATUS_NP",
        "GET CURRENT_DATA_SET",
        "GET PORT_DATA_SET",
        "GET PARENT_DATA_SET",
        "GET TIME_PROPERTIES_DATA_SET",
        "GET DEFAULT_DATA_SET",
    ]
    assert report["expected_target_port_identity"] == target
    assert report["host_time_sync_claim"] == "not_evaluated"


@pytest.mark.parametrize("target", [None, 1, True])
def test_config_rejects_non_string_target_port_identity(target):
    with pytest.raises(ValueError, match="target_port_identity must be a string"):
        _config(target_port_identity=target)


@pytest.mark.parametrize("target", [
    "080027.fffe.7f327b",
    "080027.fffe.7f327b-0",
    "080027.fffe.7f327b-65536",
    "not-a-port-identity",
])
def test_config_rejects_invalid_target_port_identity(target):
    with pytest.raises(ValueError, match="target_port_identity"):
        _config(target_port_identity=target)


def test_collector_preserves_multi_port_snapshot_as_invalid_without_selecting_one():
    output = _pmc_output().replace(
        "sending: GET PARENT_DATA_SET",
        "080027.fffe.7f327b-2 seq 16 RESPONSE MANAGEMENT PORT_DATA_SET\n"
        "    portIdentity 080027.fffe.7f327b-2\n"
        "    portState LISTENING\n"
        "sending: GET PARENT_DATA_SET",
    )

    def runner(argv, **kwargs):
        stdout = output if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["commands"][-1]["stdout"] == output
    assert report["pmc_snapshot_validation"]["status"] == "invalid"
    assert "exactly six response blocks" in report["pmc_snapshot_validation"]["error"]
    assert report["host_time_sync_claim"] == "not_evaluated"

def test_collector_rejects_response_not_matching_explicit_target_port():
    configured_target = "080027.fffe.7f327b-2"

    def runner(argv, **kwargs):
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(
        _config(target_port_identity=configured_target),
        runner=runner,
    )
    assert report["pmc_snapshot_validation"]["status"] == "invalid"
    assert "does not match configured TARGET" in report["pmc_snapshot_validation"]["error"]
    assert report["expected_target_port_identity"] == configured_target
    assert report["host_time_sync_claim"] == "not_evaluated"



def test_collector_marks_inventory_complete_only_when_all_evidence_is_present():
    def runner(argv, **kwargs):
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["collection_status"] == "complete"
    assert report["collection_issues"] == []
    assert report["pmc_snapshot_validation"]["status"] == "valid"


def test_collector_marks_partial_when_any_requested_host_command_fails():
    def runner(argv, **kwargs):
        if argv[0] == "chronyc":
            return subprocess.CompletedProcess(argv, 2, stdout="", stderr="chronyd unavailable")
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["pmc_snapshot_validation"]["status"] == "valid"
    assert report["collection_status"] == "partial"
    assert "command_not_ok:chrony_tracking:exit_error" in report["collection_issues"]
    assert "command_not_ok:chrony_sources:exit_error" in report["collection_issues"]


def test_collector_marks_partial_when_pmc_command_or_snapshot_is_invalid():
    def runner(argv, **kwargs):
        if argv[0] == "pmc":
            return subprocess.CompletedProcess(argv, 1, stdout="", stderr="socket unavailable")
        return subprocess.CompletedProcess(argv, 0, stdout=(_journal_output() if argv[0] == "journalctl" else "read-only sample"), stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["pmc_snapshot_validation"]["status"] == "not_run"
    assert report["collection_status"] == "partial"
    assert "command_not_ok:pmc_get_snapshot:exit_error" in report["collection_issues"]
    assert "pmc_snapshot_not_valid:not_run" in report["collection_issues"]


def test_collector_marks_partial_when_pmc_output_is_malformed():
    def runner(argv, **kwargs):
        stdout = "not a LinuxPTP response" if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["commands"][-1]["status"] == "ok"
    assert report["pmc_snapshot_validation"]["status"] == "invalid"
    assert report["collection_status"] == "partial"
    assert "pmc_snapshot_not_valid:invalid" in report["collection_issues"]



def test_collector_exposes_clock_elapsed_difference_without_sync_verdict():
    def runner(argv, **kwargs):
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    consistency = report["collection_clock_consistency"]
    assert consistency["status"] == "measured_no_threshold"
    assert consistency["monotonic_elapsed_ns"] >= 0
    assert consistency["realtime_elapsed_ns"] >= 0
    assert consistency["realtime_minus_monotonic_elapsed_ns"] == (
        consistency["realtime_elapsed_ns"] - consistency["monotonic_elapsed_ns"]
    )
    assert report["host_time_sync_claim"] == "not_evaluated"


def test_collector_marks_boot_identity_change_during_collection_partial(monkeypatch):
    boot_ids = iter(["boot-before", "boot-after"])

    def fake_read_text(self, encoding=None):
        return next(boot_ids)

    monkeypatch.setattr(Path, "read_text", fake_read_text)

    def runner(argv, **kwargs):
        if argv[0] == "journalctl":
            stdout = _journal_output("boot-before")
        else:
            stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["host_boot_id"] == "boot-before"
    assert report["host_boot_id_after_collection"] == "boot-after"
    assert report["collection_status"] == "partial"
    assert "host_boot_id_changed_during_collection" in report["collection_issues"]


def test_collector_declares_pmc_dataset_snapshot_non_atomic():
    def runner(argv, **kwargs):
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    observation = report["pmc_snapshot_observation"]
    assert observation["atomicity"] == "not_guaranteed"
    assert observation["observation_model"] == "sequential_management_responses"
    assert observation["simultaneity"] == "not_proven"
    assert observation["per_dataset_sample_timestamps_available"] is False
    pmc_command = report["commands"][-1]
    assert observation["monotonic_start_ns"] == pmc_command["monotonic_start_ns"]
    assert observation["monotonic_end_ns"] == pmc_command["monotonic_end_ns"]


def test_collector_compares_ptp_identity_at_collection_boundaries():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        stdout = _pmc_output() if argv[0] == "pmc" else (_journal_output() if argv[0] == "journalctl" else "read-only sample")
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    pmc_calls = [argv for argv in calls if argv[0] == "pmc"]
    assert len(pmc_calls) == 2
    assert calls.index(pmc_calls[0]) < calls.index(["timedatectl", "show"])
    assert report["pmc_identity_boundary_before_validation"]["status"] == "valid"
    comparison = report["pmc_identity_boundary_comparison"]
    assert comparison["status"] == "unchanged"
    assert comparison["changed_fields"] == []
    assert report["collection_status"] == "complete"
    assert "transient" in comparison["interpretation"]


def test_collector_marks_changed_gm_identity_between_boundaries_partial():
    pmc_count = 0

    def runner(argv, **kwargs):
        nonlocal pmc_count
        if argv[0] == "pmc":
            pmc_count += 1
            stdout = _pmc_output()
            if pmc_count == 2:
                stdout = stdout.replace(
                    "080027.fffe.d9e453", "080027.fffe.d9e454"
                )
        elif argv[0] == "journalctl":
            stdout = _journal_output()
        else:
            stdout = "read-only sample"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    comparison = report["pmc_identity_boundary_comparison"]
    assert comparison["status"] == "changed"
    assert "gm_identity" in comparison["changed_fields"]
    assert "parent_grandmaster_identity" in comparison["changed_fields"]
    assert report["collection_status"] == "partial"
    assert "ptp_identity_changed_between_collection_boundaries" in (
        report["collection_issues"]
    )


def test_collector_marks_unavailable_ptp_identity_baseline_partial():
    def runner(argv, **kwargs):
        if argv[0] == "pmc":
            return subprocess.CompletedProcess(argv, 1, stdout="", stderr="socket unavailable")
        return subprocess.CompletedProcess(argv, 0, stdout=(_journal_output() if argv[0] == "journalctl" else "read-only sample"), stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["pmc_identity_boundary_before_validation"]["status"] == "not_run"
    assert report["pmc_identity_boundary_comparison"]["status"] == "unavailable"
    assert report["collection_status"] == "partial"


def test_journal_parser_extracts_ptp4l_gm_port_and_announce_events():
    boot = "boot-before"
    messages = [
        "eth0: new foreign master 080027.fffe.d9e453-1",
        "selected best master clock 080027.fffe.d9e453",
        "port 1 (eth0): LISTENING to SLAVE on ANNOUNCE_RECEIPT_TIMEOUT",
        "eth0: master tx announce timeout",
    ]
    import json
    lines = []
    for index, message in enumerate(messages):
        lines.append(json.dumps({
            "_BOOT_ID": boot,
            "__MONOTONIC_TIMESTAMP": str(1000000 + index),
            "__REALTIME_TIMESTAMP": str(1795700000000000 + index),
            "MESSAGE": message,
        }))
    evidence = parse_linuxptp_journal_evidence("\n".join(lines), boot)
    assert evidence["status"] == "parsed"
    assert [event["type"] for event in evidence["events"]] == [
        "gm_candidate_observed", "gm_selected", "port_state_transition",
        "master_announce_transmit_timeout",
    ]
    assert evidence["events"][2]["port"] == "port 1 (eth0)"
    assert evidence["coverage_status"] == "unknown"
    assert "does not prove" in evidence["absence_interpretation"]
    assert evidence["events"][-1]["interpretation"].startswith("ptp4l master announce")


def test_journal_parser_fails_closed_on_wrong_boot_or_missing_monotonic():
    import json
    wrong_boot = json.dumps({
        "_BOOT_ID": "other-boot", "__MONOTONIC_TIMESTAMP": "1",
        "__REALTIME_TIMESTAMP": "2", "MESSAGE": "selected best master clock 080027.fffe.d9e453",
    })
    missing_mono = json.dumps({
        "_BOOT_ID": "boot-before", "__REALTIME_TIMESTAMP": "2", "MESSAGE": "benign",
    })
    evidence = parse_linuxptp_journal_evidence(
        wrong_boot + "\n" + missing_mono, "boot-before"
    )
    assert evidence["status"] == "partial"
    assert evidence["entry_count"] == 0
    assert "journal_boot_id_mismatch:1" in evidence["issues"]
    assert "journal_monotonic_timestamp_invalid:2" in evidence["issues"]


def test_journal_parser_does_not_claim_stability_when_no_event_is_present():
    boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    evidence = parse_linuxptp_journal_evidence(
        _journal_output(message="ptp4l: port 1: periodic status"), boot
    )
    assert evidence["status"] == "parsed"
    assert evidence["event_count"] == 0
    assert evidence["coverage_status"] == "unknown"
    assert "does not prove" in evidence["absence_interpretation"]


def test_journal_parser_rejects_malformed_json():
    evidence = parse_linuxptp_journal_evidence("{not-json}", "boot-before")
    assert evidence["status"] == "partial"
    assert "journal_json_invalid_line:1" in evidence["issues"]


def test_journal_parser_does_not_infer_clock_step_from_unverified_message():
    boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    evidence = parse_linuxptp_journal_evidence(
        _journal_output(message="ptp4l: servo jump; clock stepped"), boot
    )
    assert evidence["status"] == "parsed"
    assert evidence["event_count"] == 0
    assert evidence["coverage_status"] == "unknown"

def test_journal_parser_extracts_systemd_rate_limit_and_kernel_loss_events():
    import json
    boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    records = [
        {
            "_BOOT_ID": boot, "__MONOTONIC_TIMESTAMP": "1000000",
            "__REALTIME_TIMESTAMP": "1795700000000000",
            "MESSAGE_ID": "a596d6fe7bfa4994828e72309e95d61e",
            "MESSAGE": "Suppressed 17 messages from /system.slice/ptp4l.service",
        },
        {
            "_BOOT_ID": boot, "__MONOTONIC_TIMESTAMP": "1000001",
            "__REALTIME_TIMESTAMP": "1795700000000001",
            "MESSAGE_ID": "e9bf28d037fff184e6ae669a370e694725",
            "N_DROPPED": "9",
            "MESSAGE": "Missed 9 kernel messages",
        },
    ]
    evidence = parse_linuxptp_journal_evidence(
        "\n".join(json.dumps(record) for record in records), boot
    )
    assert evidence["status"] == "parsed"
    assert [event["type"] for event in evidence["events"]] == [
        "journal_rate_limit_suppression", "journal_kernel_message_loss",
    ]
    assert evidence["events"][0]["suppressed_count"] == 17
    assert evidence["events"][0]["affected_unit"] == "/system.slice/ptp4l.service"
    assert evidence["events"][1]["dropped_count"] == "9"


def test_collector_records_journal_storage_context_without_claiming_coverage():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        if argv[0] == "pmc":
            stdout = _pmc_output()
        elif argv[0] == "journalctl" and "-u" in argv and "systemd-journald.service" in argv:
            stdout = _journald_suppression_output()
        elif argv[0] == "journalctl" and "-u" in argv:
            stdout = _journal_output()
        else:
            stdout = "read-only context sample"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    context = report["journal_storage_context"]
    assert report["schema_version"] == 3
    assert context["status"] == "observed"
    assert context["coverage_status"] == "unknown"
    assert context["suppression_evidence"]["events"][0]["type"] == "journal_rate_limit_suppression"
    names = {item["name"] for item in context["commands"]}
    assert {"journal_config", "journal_disk_usage", "journal_boot_inventory",
            "journal_suppression_events", "unit_logging_config:ptp4l.service"} <= names
    event_evidence = report["journal_event_evidence"]["ptp4l.service"]
    assert event_evidence["query_window"]["since_argument"] == "15 min ago"
    assert event_evidence["query_window"]["until_argument"] is None
    assert event_evidence["query_window"]["human_since_expression_normalized"] is False
    assert "journal_command_execution_relation" in event_evidence["entries"][0]
    assert "collection_window_relation" not in event_evidence["entries"][0]
    assert report["collection_status"] == "complete"


def test_optional_journal_context_failure_does_not_claim_or_break_core_inventory():
    def runner(argv, **kwargs):
        if argv[0] == "pmc":
            stdout = _pmc_output()
        elif argv[0] == "journalctl" and "-u" in argv:
            stdout = _journal_output()
        elif argv[0] == "systemd-analyze":
            raise FileNotFoundError("systemd-analyze missing")
        else:
            stdout = "read-only context sample"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    assert report["journal_storage_context"]["status"] == "partial"
    assert report["journal_storage_context"]["coverage_status"] == "unknown"
    assert report["collection_status"] == "complete"


def test_collector_marks_non_default_journal_namespace_not_queried_partial():
    def runner(argv, **kwargs):
        if argv[0] == "pmc":
            stdout = _pmc_output()
        elif argv[0] == "journalctl" and "-u" in argv:
            stdout = _journal_output()
        elif argv[0] == "systemctl" and any("LogNamespace" in arg for arg in argv):
            stdout = "LogNamespace=ptp-logs\n"
        else:
            stdout = "read-only context sample"
        return subprocess.CompletedProcess(argv, 0, stdout=stdout, stderr="")

    report = collect_linuxptp_host_evidence(_config(), runner=runner)
    evidence = report["journal_event_evidence"]["ptp4l.service"]
    assert evidence["namespace_observation"]["status"] == "non_default_not_queried"
    assert evidence["namespace_observation"]["namespace"] == "ptp-logs"
    assert "journal_namespace_not_queried:ptp-logs" in evidence["issues"]
    assert report["collection_status"] == "partial"
    assert "journal_evidence_not_parsed:ptp4l.service:partial" in report["collection_issues"]


