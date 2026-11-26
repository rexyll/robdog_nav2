from types import SimpleNamespace
import subprocess

import pytest

from robodog_gnss_time.linuxptp_pmc_collector import (
    LinuxPtpPmcCollector,
    PmcCollectorConfig,
)


def uds_config(**overrides):
    values = dict(
        executable="/usr/sbin/pmc",
        transport="uds",
        domain_number=24,
        boundary_hops=1,
        transport_specific=0x1,
        socket_path="/run/ptp4l-eth0",
        command_timeout_s=1.5,
    )
    values.update(overrides)
    return PmcCollectorConfig(**values)


def test_builds_explicit_uds_domain_profile_and_get_only_argv():
    collector = LinuxPtpPmcCollector(uds_config())
    argv = collector.build_argv()
    assert argv[:10] == (
        "/usr/sbin/pmc", "-d", "24", "-b", "1", "-t", "0x1",
        "-u", "-s", "/run/ptp4l-eth0",
    )
    commands = argv[10:]
    assert commands == (
        "GET TIME_STATUS_NP",
        "GET CURRENT_DATA_SET",
        "GET PORT_DATA_SET",
        "GET PARENT_DATA_SET",
        "GET TIME_PROPERTIES_DATA_SET",
        "GET DEFAULT_DATA_SET",
    )
    assert all(command.startswith("GET ") for command in commands)
    assert not any("SET " in arg or "CMD " in arg for arg in argv)


def test_builds_explicit_network_transport_and_interface():
    collector = LinuxPtpPmcCollector(uds_config(
        transport="udp6", socket_path="", interface="enp2s0",
    ))
    argv = collector.build_argv()
    assert argv[:10] == (
        "/usr/sbin/pmc", "-d", "24", "-b", "1", "-t", "0x1",
        "-6", "-i", "enp2s0",
    )


@pytest.mark.parametrize("changes", [
    {"executable": "pmc"},
    {"transport": "automatic"},
    {"domain_number": True},
    {"domain_number": 256},
    {"boundary_hops": -1},
    {"transport_specific": 256},
    {"socket_path": "relative.sock"},
    {"command_timeout_s": 0},
])
def test_config_rejects_missing_or_invalid_deployment_parameters(changes):
    with pytest.raises(ValueError):
        uds_config(**changes)


def test_network_transport_requires_explicit_interface():
    with pytest.raises(ValueError, match="interface"):
        uds_config(transport="udp4", socket_path="", interface="")


def test_uds_transport_rejects_interface_to_avoid_ambiguous_binding():
    with pytest.raises(ValueError, match="interface must be empty"):
        uds_config(interface="eth0")


def test_collect_captures_raw_output_boot_id_and_monotonic_window(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test\n", encoding="ascii")
    calls = []

    def runner(args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(returncode=0, stdout="raw pmc response\n", stderr="")

    collector = LinuxPtpPmcCollector(
        uds_config(),
        boot_id_path=boot_file,
        runner=runner,
        monotonic_ns=iter([100, 250]).__next__,
    )
    result = collector.collect()
    assert result.succeeded
    assert result.host_boot_id == "boot-test"
    assert (result.monotonic_start_ns, result.monotonic_end_ns) == (100, 250)
    assert result.stdout == "raw pmc response\n"
    assert result.returncode == 0
    assert calls[0][1]["shell"] is False
    assert calls[0][1]["check"] is False
    assert calls[0][1]["timeout"] == 1.5
    assert calls[0][0] == list(result.argv)


def test_nonzero_exit_is_preserved_as_failed_collection(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test", encoding="ascii")
    collector = LinuxPtpPmcCollector(
        uds_config(),
        boot_id_path=boot_file,
        runner=lambda *args, **kwargs: SimpleNamespace(
            returncode=2, stdout="partial", stderr="socket error"
        ),
        monotonic_ns=iter([1, 2]).__next__,
    )
    result = collector.collect()
    assert not result.succeeded
    assert result.returncode == 2
    assert result.stdout == "partial"
    assert result.stderr == "socket error"
    assert "status 2" in result.fault


def test_timeout_preserves_partial_output_and_fails_closed(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test", encoding="ascii")

    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 1.5, output=b"partial", stderr=b"late")

    collector = LinuxPtpPmcCollector(
        uds_config(),
        boot_id_path=boot_file,
        runner=timeout,
        monotonic_ns=iter([1, 2]).__next__,
    )
    result = collector.collect()
    assert result.timed_out
    assert not result.succeeded
    assert result.stdout == "partial"
    assert result.stderr == "late"
    assert result.returncode is None


def test_missing_boot_id_prevents_command_execution(tmp_path):
    collector = LinuxPtpPmcCollector(
        uds_config(),
        boot_id_path=tmp_path / "missing",
        runner=lambda *args, **kwargs: pytest.fail("pmc must not run without boot ID"),
        monotonic_ns=iter([1, 2]).__next__,
    )
    result = collector.collect()
    assert not result.succeeded
    assert not result.host_boot_id
    assert "FileNotFoundError" in result.fault


def test_monotonic_regression_marks_collection_failed(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test", encoding="ascii")
    collector = LinuxPtpPmcCollector(
        uds_config(),
        boot_id_path=boot_file,
        runner=lambda *args, **kwargs: SimpleNamespace(
            returncode=0, stdout="raw", stderr=""
        ),
        monotonic_ns=iter([10, 9]).__next__,
    )
    result = collector.collect()
    assert not result.succeeded
    assert "moved backwards" in result.fault
