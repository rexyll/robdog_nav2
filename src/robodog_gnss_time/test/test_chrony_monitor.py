from pathlib import Path
from types import SimpleNamespace

import pytest

from robodog_gnss_time.chrony_monitor import (
    ChronyHostTimeMonitor,
    ChronyMonitorConfig,
    parse_chrony_tracking,
)


TRACKING = """Reference ID    : 203.0.113.10 (ntp.example)
Stratum         : 3
Ref time (UTC)  : Thu Nov 26 00:00:00 2026
System time     : 0.000125000 seconds fast of NTP time
Last offset     : +0.000010000 seconds
RMS offset      : 0.000020000 seconds
Frequency       : 1.000 ppm fast
Residual freq   : +0.100 ppm
Skew            : 0.200 ppm
Root delay      : 0.010000000 seconds
Root dispersion : 0.000250000 seconds
Update interval : 64.0 seconds
Leap status     : Normal
"""


def test_parse_tracking_and_offset_sign_convention():
    parsed = parse_chrony_tracking(TRACKING)
    assert parsed["utc_offset_ns"] == 125_000
    assert parsed["uncertainty_ns"] == 250_000
    assert parsed["stratum"] == 3
    assert parsed["reference_id"] == "203.0.113.10 (ntp.example)"
    assert parsed["leap_status"] == "Normal"


def test_parse_slow_system_clock_as_negative_offset():
    parsed = parse_chrony_tracking(TRACKING.replace("fast of", "slow of"))
    assert parsed["utc_offset_ns"] == -125_000


@pytest.mark.parametrize("mutated", [
    TRACKING.replace("Stratum         : 3", "Stratum         : 16"),
    TRACKING.replace("Leap status     : Normal", "Leap status     : Not synchronised"),
    TRACKING.replace("Root dispersion : 0.000250000 seconds", "Root dispersion : NaN seconds"),
    TRACKING.replace("System time     : 0.000125000 seconds fast of NTP time",
                     "System time     : 0.000125000 seconds ahead of NTP time"),
])
def test_parser_rejects_or_monitor_marks_bad_tracking(mutated):
    if "ahead of" in mutated or "NaN" in mutated:
        with pytest.raises(ValueError):
            parse_chrony_tracking(mutated)
    else:
        parsed = parse_chrony_tracking(mutated)
        assert parsed["leap_status"] != "Normal" or parsed["stratum"] == 16


def test_monitor_emits_synchronized_sample_with_boot_and_monotonic(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test\n", encoding="ascii")
    calls = []

    def runner(args, **kwargs):
        calls.append((args, kwargs))
        return SimpleNamespace(returncode=0, stdout=TRACKING, stderr="")

    monitor = ChronyHostTimeMonitor(
        ChronyMonitorConfig("chrony:tracking", 1_000_000),
        boot_id_path=boot_file, runner=runner, monotonic_ns=lambda: 987654321,
    )
    sample = monitor.sample()
    assert sample["state"] == "synchronized"
    assert sample["host_boot_id"] == "boot-test"
    assert sample["sample_monotonic_ns"] == 987654321
    assert sample["sample_sequence"] == 1
    assert sample["utc_offset_ns"] == 125_000
    assert calls[0][0] == ["chronyc", "-n", "tracking"]
    assert calls[0][1]["check"] is False


def test_monitor_fails_closed_when_offset_budget_exceeded(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test", encoding="ascii")
    monitor = ChronyHostTimeMonitor(
        ChronyMonitorConfig("chrony:tracking", 100_000),
        boot_id_path=boot_file,
        runner=lambda *args, **kwargs: SimpleNamespace(returncode=0, stdout=TRACKING, stderr=""),
        monotonic_ns=lambda: 10,
    )
    assert monitor.sample()["state"] == "unsynchronized"


def test_monitor_faults_closed_when_chronyc_unavailable(tmp_path):
    boot_file = tmp_path / "boot_id"
    boot_file.write_text("boot-test", encoding="ascii")

    def fail(*args, **kwargs):
        raise FileNotFoundError("chronyc missing")

    monitor = ChronyHostTimeMonitor(
        ChronyMonitorConfig("chrony:tracking", 1_000_000),
        boot_id_path=boot_file, runner=fail, monotonic_ns=lambda: 10,
    )
    sample = monitor.sample()
    assert sample["state"] == "fault"
    assert "chronyc missing" in sample["fault"]
    assert sample["host_boot_id"] == ""


def test_monitor_requires_explicit_error_budget():
    with pytest.raises(ValueError):
        ChronyMonitorConfig("chrony:tracking", 0)
