from pathlib import Path
import gzip

from robodog_gnss_time.linux_kernel_trace_capabilities import (
    collect_linux_kernel_trace_capabilities,
)


def _write(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


def test_capability_audit_reads_config_btf_and_only_listed_tracepoints(tmp_path):
    release = "6.1.99-tegra"
    config = "\n".join([
        "CONFIG_FTRACE=y",
        "CONFIG_TRACEPOINTS=y",
        "CONFIG_FTRACE_SYSCALLS=y",
        "# CONFIG_HAVE_SYSCALL_TRACEPOINTS is not set",
        "CONFIG_BPF_SYSCALL=y",
        "# CONFIG_DEBUG_INFO_BTF is not set",
        "CONFIG_IKCONFIG_PROC=y",
    ]) + "\n"
    proc_config = tmp_path / "proc/config.gz"
    proc_config.parent.mkdir(parents=True)
    with gzip.open(proc_config, "wt", encoding="utf-8") as stream:
        stream.write(config)
    _write(tmp_path / f"boot/config-{release}", "CONFIG_PERF_EVENTS=y\n")
    _write(
        tmp_path / "proc/self/mountinfo",
        "36 25 0:32 / /sys/kernel/tracing rw,nosuid,nodev,noexec,relatime - tracefs tracefs rw\n"
        "40 25 0:33 / /sys/kernel/debug rw,nosuid,nodev,noexec,relatime - debugfs debugfs rw\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/available_events",
        "syscalls:sys_enter_clock_adjtime\nsyscalls:sys_exit_clock_adjtime\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_enter_clock_adjtime/format",
        "name: sys_enter_clock_adjtime\nID: 900\nformat:\n"
        "\tfield:unsigned short common_type; offset:0; size:2;\n"
        "\tfield:int __syscall_nr; offset:8; size:4;\n"
        "\tfield:clockid_t which_clock; offset:16; size:8;\n"
        "\tfield:struct timex * tx; offset:24; size:8;\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_exit_clock_adjtime/format",
        "name: sys_exit_clock_adjtime\nformat:\n"
        "\tfield:long ret; offset:16; size:8;\n",
    )
    _write(tmp_path / "sys/kernel/debug/tracing/available_events", "")
    _write(tmp_path / "sys/kernel/btf/vmlinux", "btf")

    report = collect_linux_kernel_trace_capabilities(
        root=tmp_path, kernel_release=release
    )

    assert report["status"] == "observed"
    assert report["kernel"]["release"] == release
    assert report["kernel_config_sources"]["proc_config_gz"]["settings"]["CONFIG_FTRACE"] == "y"
    assert report["kernel_config_sources"]["proc_config_gz"]["settings"]["CONFIG_HAVE_SYSCALL_TRACEPOINTS"] == "n"
    assert report["kernel_config_sources"]["boot_config"]["settings"]["CONFIG_PERF_EVENTS"] == "y"
    assert report["btf"]["status"] == "present_readable"
    tracefs = report["tracefs"][0]
    assert tracefs["mounts"][0]["filesystem_type"] == "tracefs"
    assert report["tracefs"][1]["mounts"][0]["filesystem_type"] == "debugfs"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["format_fields"] == [
        "common_type", "__syscall_nr", "which_clock", "tx",
    ]
    assert tracefs["syscall_events"]["sys_exit_clock_adjtime"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_enter_adjtimex"]["status"] == "not_listed"
    assert "No event was enabled" in tracefs["interpretation"]


def test_capability_audit_fails_closed_when_kernel_artifacts_are_missing(tmp_path):
    report = collect_linux_kernel_trace_capabilities(
        root=tmp_path, kernel_release="missing-kernel"
    )

    assert report["status"] == "unavailable"
    assert report["kernel_config_sources"]["proc_config_gz"]["status"] == "not_found"
    assert report["kernel_config_sources"]["boot_config"]["status"] == "not_found"
    assert report["btf"]["status"] == "not_found"
    assert all(
        item["available_events_status"] == "not_found"
        for item in report["tracefs"]
    )
    assert report["candidate_syscall_events"]
    assert "No tracefs mount" in report["interpretation"]
