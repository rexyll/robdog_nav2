import subprocess

import pytest

from robodog_gnss_time.linuxptp_host_evidence import (
    LinuxPtpHostEvidenceConfig,
    collect_linuxptp_host_evidence,
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
        "ptp4l_service": "ptp4l.service",
        "phc2sys_service": "phc2sys.service",
        "chrony_service": "chrony.service",
    }
    values.update(overrides)
    return LinuxPtpHostEvidenceConfig(**values)


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
        stdout = _pmc_output() if argv[0] == "pmc" else "read-only sample"
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
    assert report["pmc_snapshot_validation"]["clock_identity"] == "080027.fffe.7f327b"
    assert report["expected_transport_specific"] == 0
    assert report["host_time_sync_claim"] == "not_evaluated"
    assert report["evidence_kind"] == "read_only_linuxptp_host_inventory"
    assert all(call[1]["shell"] is False for call in calls)
    assert any(call[0][0] == "systemctl" for call in calls)
    assert any(call[0][0] == "journalctl" for call in calls)


def test_collector_rejects_snapshot_domain_mismatch_without_claiming_sync():
    def runner(argv, **kwargs):
        stdout = _pmc_output(domain_number=4) if argv[0] == "pmc" else ""
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
        stdout = _pmc_output() if argv[0] == "pmc" else ""
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
        stdout = _pmc_output() if argv[0] == "pmc" else ""
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
