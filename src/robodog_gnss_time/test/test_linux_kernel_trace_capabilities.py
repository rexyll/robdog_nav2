from pathlib import Path
import gzip

from robodog_gnss_time import linux_kernel_trace_capabilities as capability_module
from robodog_gnss_time.linux_kernel_trace_capabilities import (
    collect_linux_kernel_trace_capabilities,
    main,
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
        "25 1 0:20 / /sys rw,nosuid,nodev,noexec,relatime - sysfs sysfs rw\n"
        "36 25 0:32 / /sys/kernel/tracing rw,nosuid,nodev,noexec,relatime - tracefs tracefs rw\n"
        "40 25 0:33 / /sys/kernel/debug rw,nosuid,nodev,noexec,relatime - debugfs debugfs rw\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/available_events",
        "syscalls:sys_enter_clock_adjtime\nsyscalls:sys_exit_clock_adjtime\n"
        "syscalls:sys_enter_clock_adjtime32\nsyscalls:sys_exit_clock_adjtime32\n"
        "syscalls:sys_enter_adjtimex_time32\nsyscalls:sys_exit_adjtimex_time32\n",
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
    for event_name in (
        "sys_enter_clock_adjtime32",
        "sys_exit_clock_adjtime32",
        "sys_enter_adjtimex_time32",
        "sys_exit_adjtimex_time32",
    ):
        _write(
            tmp_path / "sys/kernel/tracing/events/syscalls" / event_name / "format",
            f"name: {event_name}\\nformat:\\n\\tfield:long ret; offset:16; size:8;\\n",
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
    assert tracefs["mounts"] == [{
        "mountpoint": str(tmp_path / "sys/kernel/tracing"),
        "filesystem_type": "tracefs",
        "mount_relation": "exact",
        "mount_options": "rw,nosuid,nodev,noexec,relatime",
    }]
    assert report["tracefs"][1]["mounts"] == [{
        "mountpoint": str(tmp_path / "sys/kernel/debug"),
        "filesystem_type": "debugfs",
        "mount_relation": "ancestor",
        "mount_options": "rw,nosuid,nodev,noexec,relatime",
    }]
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["format_fields"] == [
        "common_type", "__syscall_nr", "which_clock", "tx",
    ]
    assert tracefs["syscall_events"]["sys_exit_clock_adjtime"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_enter_adjtimex"]["status"] == "not_listed"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime32"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_exit_clock_adjtime32"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_enter_adjtimex_time32"]["status"] == "available"
    assert tracefs["syscall_events"]["sys_exit_adjtimex_time32"]["status"] == "available"
    assert tracefs["syscall_event_pairs"]["clock_adjtime"]["pair_status"] == "complete"
    assert tracefs["syscall_event_pairs"]["clock_adjtime32"]["pair_status"] == "complete"
    assert tracefs["syscall_event_pairs"]["adjtimex"]["pair_status"] == "not_listed"
    assert tracefs["syscall_event_pairs"]["adjtimex_time32"]["pair_status"] == "complete"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["enable_observation"] == {
        "status": "not_found", "value": None,
    }
    assert "read only" in tracefs["interpretation"]
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



def test_standalone_cli_writes_json_report_without_ptp_arguments(tmp_path, capsys, monkeypatch):
    expected = {"status": "observed", "candidate_syscall_events": ["sys_enter_clock_adjtime"]}
    monkeypatch.setattr(
        capability_module,
        "collect_linux_kernel_trace_capabilities",
        lambda root="/", kernel_release=None: expected,
    )
    output = tmp_path / "kernel-capabilities.json"

    assert main(["--root", str(tmp_path), "--kernel-release", "6.1-test", "--output", str(output)]) == 0
    assert __import__("json").loads(output.read_text(encoding="utf-8")) == expected

    assert main(["--root", str(tmp_path)]) == 0
    printed = capsys.readouterr().out
    assert __import__("json").loads(printed) == expected



def test_tracepoint_pair_reports_partial_and_enable_states_without_writing(tmp_path):
    _write(
        tmp_path / "proc/self/mountinfo",
        "36 25 0:32 / /sys/kernel/tracing rw,nosuid,nodev,noexec,relatime - tracefs tracefs rw\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/available_events",
        "syscalls:sys_enter_clock_adjtime\nsyscalls:sys_exit_adjtimex\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_enter_clock_adjtime/format",
        "name: sys_enter_clock_adjtime\nformat:\n\tfield:struct timex * tx; offset:8; size:8;\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_exit_adjtimex/format",
        "name: sys_exit_adjtimex\nformat:\n\tfield:long ret; offset:8; size:8;\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_enter_clock_adjtime/enable",
        "1\n",
    )
    _write(
        tmp_path / "sys/kernel/tracing/events/syscalls/sys_exit_adjtimex/enable",
        "X\n",
    )

    report = collect_linux_kernel_trace_capabilities(root=tmp_path)
    tracefs = report["tracefs"][0]
    assert tracefs["syscall_event_pairs"]["clock_adjtime"]["pair_status"] == "partial"
    assert tracefs["syscall_event_pairs"]["adjtimex"]["pair_status"] == "partial"
    assert tracefs["syscall_events"]["sys_enter_clock_adjtime"]["enable_observation"] == {
        "status": "observed",
        "value": "1",
        "meaning": "all_affected_events_enabled",
    }
    assert tracefs["syscall_events"]["sys_exit_adjtimex"]["enable_observation"] == {
        "status": "observed",
        "value": "X",
        "meaning": "mixed_enabled_and_disabled",
    }
    assert not (tmp_path / "sys/kernel/tracing/set_event").exists()



def test_mount_inventory_ignores_unrelated_ancestor_filesystems(tmp_path):
    mountinfo = {
        "status": "ok",
        "content": (
            "20 1 0:10 / / rw,relatime - ext4 /dev/root rw\n"
            "21 20 0:11 / /sys rw,nosuid,nodev,noexec,relatime - sysfs sysfs rw\n"
            "22 21 0:12 / /sys/kernel/debug rw,nosuid,nodev,noexec,relatime - debugfs debugfs rw\n"
        ),
    }
    mounts = capability_module._mount_inventory(
        tmp_path / "sys/kernel/debug/tracing",
        mountinfo,
        root=tmp_path,
    )
    assert mounts == [{
        "mountpoint": str(tmp_path / "sys/kernel/debug"),
        "filesystem_type": "debugfs",
        "mount_relation": "ancestor",
        "mount_options": "rw,nosuid,nodev,noexec,relatime",
    }]



def test_handler_function_inventory_reports_candidates_without_claiming_attachability(tmp_path):
    _write(
        tmp_path / "sys/kernel/tracing/available_filter_functions",
        "do_clock_adjtime\nother_function\ndo_adjtimex [vmlinux]\n",
    )
    report = collect_linux_kernel_trace_capabilities(root=tmp_path)
    symbols = report["handler_function_symbols"][0]["symbols"]
    assert symbols["do_clock_adjtime"]["status"] == "listed"
    assert symbols["do_adjtimex"]["status"] == "listed"
    assert "does not prove BTF FUNC availability" in report["handler_function_symbols"][0]["interpretation"]
    assert "No tracefs mount" in report["interpretation"]


def test_handler_function_inventory_distinguishes_unavailable_from_absent(tmp_path):
    report = collect_linux_kernel_trace_capabilities(root=tmp_path)
    symbols = report["handler_function_symbols"][0]
    assert symbols["status"] == "not_found"
    assert all(item["status"] == "unknown" for item in symbols["symbols"].values())



def test_runtime_policy_inventory_reads_security_controls_without_mutation(tmp_path):
    _write(tmp_path / "proc/sys/kernel/unprivileged_bpf_disabled", "2\n")
    _write(tmp_path / "proc/sys/kernel/perf_event_paranoid", "3\n")
    _write(tmp_path / "proc/sys/kernel/kptr_restrict", "2\n")
    _write(tmp_path / "sys/kernel/security/lockdown", "none [integrity] confidentiality\n")
    _write(
        tmp_path / "proc/self/status",
        "Name:\tpytest\nUid:\t1000\t1000\t1000\t1000\n"
        "Gid:\t1000\t1000\t1000\t1000\nCapEff:\t0000000000000000\n"
        "CapBnd:\t0000000000000000\nNoNewPrivs:\t1\nSeccomp:\t2\nSeccomp_filters:\t1\n",
    )

    report = collect_linux_kernel_trace_capabilities(root=tmp_path)
    policy = report["runtime_policy"]

    assert policy["sysctls"]["unprivileged_bpf_disabled"] == {
        "status": "ok", "value": "2",
    }
    assert policy["sysctls"]["perf_event_paranoid"] == {
        "status": "ok", "value": "3",
    }
    assert policy["lockdown"]["active_mode"] == "integrity"
    assert policy["process_security"]["fields"]["Uid"].startswith("1000")
    assert policy["process_security"]["fields"]["CapEff"] == "0000000000000000"
    assert policy["process_security"]["fields"]["Seccomp"] == "2"
    assert "no BPF syscall was attempted" in policy["interpretation"]
    assert not (tmp_path / "proc/sys/kernel/unprivileged_bpf_disabled").read_text().startswith("0")


def test_runtime_policy_inventory_fails_closed_when_policy_files_are_unavailable(tmp_path):
    report = collect_linux_kernel_trace_capabilities(root=tmp_path)
    policy = report["runtime_policy"]

    assert policy["sysctls"]["unprivileged_bpf_disabled"] == {
        "status": "not_found", "value": None,
    }
    assert policy["lockdown"]["status"] == "not_found"
    assert policy["process_security"]["status"] == "not_found"
    assert policy["process_security"]["fields"] == {}



def _bpftool_json_fixture():
    return {
        "types": [
            {"id": 1, "kind": "STRUCT", "name": "timex", "size": 208, "vlen": 0},
            {"id": 2, "kind": "PTR", "name": "(anon)", "type_id": 1},
            {"id": 3, "kind": "INT", "name": "long", "size": 8},
            {"id": 4, "kind": "TYPEDEF", "name": "clockid_t", "type_id": 5},
            {"id": 5, "kind": "INT", "name": "int", "size": 4},
            {
                "id": 6, "kind": "FUNC_PROTO", "name": "(anon)",
                "ret_type_id": 3, "vlen": 2,
                "params": [
                    {"name": "which_clock", "type_id": 4},
                    {"name": "tx", "type_id": 2},
                ],
            },
            {"id": 7, "kind": "FUNC", "name": "do_clock_adjtime", "type_id": 6, "linkage": "global"},
            {
                "id": 8, "kind": "FUNC_PROTO", "name": "(anon)",
                "ret_type_id": 3, "vlen": 1,
                "params": [{"name": "tx", "type_id": 2}],
            },
            {"id": 9, "kind": "FUNC", "name": "do_adjtimex", "type_id": 8, "linkage": "global"},
        ]
    }


def test_btf_handler_inventory_resolves_func_proto_and_parameter_type_chains(tmp_path, monkeypatch):
    import json

    btf = _write(tmp_path / "sys/kernel/btf/vmlinux", "fixture")
    monkeypatch.setattr(capability_module.shutil, "which", lambda name: "/usr/sbin/bpftool")
    monkeypatch.setattr(
        capability_module.subprocess,
        "run",
        lambda *args, **kwargs: type(
            "Completed", (), {
                "returncode": 0,
                "stdout": json.dumps(_bpftool_json_fixture()),
                "stderr": "",
            }
        )(),
    )

    result = capability_module._btf_handler_function_inventory(btf)
    assert result["status"] == "observed"
    assert result["symbols"]["do_clock_adjtime"]["status"] == "prototype_resolved"
    clock_adj = result["symbols"]["do_clock_adjtime"]
    assert clock_adj["return_type"]["name"] == "long"
    assert clock_adj["parameter_count"] == 2
    assert clock_adj["parameters"][0]["name"] == "which_clock"
    assert clock_adj["parameters"][0]["type"]["kind"] == "TYPEDEF"
    assert clock_adj["parameters"][0]["type"]["target"]["name"] == "int"
    assert clock_adj["parameters"][1]["type"]["target"]["kind"] == "STRUCT"
    assert result["symbols"]["do_adjtimex"]["status"] == "prototype_resolved"
    assert "-j" in result["command"]
    assert "does not prove fentry attachability" in result["interpretation"]


def test_btf_handler_inventory_fails_closed_on_missing_proto_or_type_reference():
    document = _bpftool_json_fixture()
    document["types"] = [
        item for item in document["types"] if item["id"] != 6
    ]
    result = capability_module._parse_btf_handler_functions(document)
    assert result["symbols"]["do_clock_adjtime"]["status"] == "invalid_func_proto_reference"

    document = _bpftool_json_fixture()
    document["types"][5]["params"][1]["type_id"] = 999
    result = capability_module._parse_btf_handler_functions(document)
    assert result["symbols"]["do_clock_adjtime"]["status"] == "prototype_incomplete"
    assert result["symbols"]["do_clock_adjtime"]["parameters"][1]["type"]["status"] == "missing_type"


def test_btf_handler_inventory_reports_unknown_when_bpftool_is_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(capability_module.shutil, "which", lambda name: None)
    result = capability_module._btf_handler_function_inventory(
        tmp_path / "sys/kernel/btf/vmlinux"
    )
    assert result["status"] == "tool_unavailable"
    assert all(item["status"] == "unknown" for item in result["symbols"].values())


def test_btf_handler_inventory_reports_command_failure_without_inference(tmp_path, monkeypatch):
    btf = _write(tmp_path / "sys/kernel/btf/vmlinux", "fixture")
    monkeypatch.setattr(capability_module.shutil, "which", lambda name: "/usr/sbin/bpftool")
    monkeypatch.setattr(
        capability_module.subprocess,
        "run",
        lambda *args, **kwargs: type(
            "Completed", (), {"returncode": 1, "stdout": "", "stderr": "invalid BTF"}
        )(),
    )
    result = capability_module._btf_handler_function_inventory(btf)
    assert result["status"] == "command_failed"
    assert all(item["status"] == "unknown" for item in result["symbols"].values())


def test_btf_handler_inventory_reports_invalid_json(tmp_path, monkeypatch):
    btf = _write(tmp_path / "sys/kernel/btf/vmlinux", "fixture")
    monkeypatch.setattr(capability_module.shutil, "which", lambda name: "/usr/sbin/bpftool")
    monkeypatch.setattr(
        capability_module.subprocess,
        "run",
        lambda *args, **kwargs: type(
            "Completed", (), {"returncode": 0, "stdout": "{", "stderr": ""}
        )(),
    )
    result = capability_module._btf_handler_function_inventory(btf)
    assert result["status"] == "invalid_json"
    assert all(item["status"] == "unknown" for item in result["symbols"].values())



def test_capability_report_distinguishes_rootfs_fixture_from_collector_host(tmp_path, monkeypatch):
    _write(tmp_path / "proc/sys/kernel/random/boot_id", "fixture-boot-id\n")
    monkeypatch.setattr(capability_module.shutil, "which", lambda name: None)

    report = collect_linux_kernel_trace_capabilities(root=tmp_path)

    assert report["schema_version"] == 2
    assert report["collection_mode"] == "rootfs_fixture"
    assert report["kernel"]["release"] is None
    assert report["kernel"]["release_source"] == "unknown_for_rootfs_fixture"
    assert report["kernel"]["machine"] is None
    assert report["kernel"]["boot_id"] == "fixture-boot-id"
    assert report["collector_host"]["release"] == capability_module.platform.uname().release
    assert "not attributed" in report["collector_host"]["note"]
    assert report["kernel_config_sources"]["boot_config"]["status"] == "not_probed"


def test_btf_observation_records_sha256_identity(tmp_path):
    import hashlib

    btf = _write(tmp_path / "sys/kernel/btf/vmlinux", "synthetic-btf-bytes")
    result = capability_module._btf_observation(btf)
    assert result["status"] == "present_readable"
    assert result["sha256"] == hashlib.sha256(b"synthetic-btf-bytes").hexdigest()
    assert result["hash_algorithm"] == "sha256"


def test_tracefs_probe_reports_permission_denied_directory_without_crashing(tmp_path, monkeypatch):
    tracefs_path = tmp_path / "sys/kernel/debug/tracing"
    original_stat = Path.stat

    def permission_denied_for_tracefs(path, *args, **kwargs):
        if path == tracefs_path:
            raise PermissionError(13, "Permission denied", str(path))
        return original_stat(path, *args, **kwargs)

    monkeypatch.setattr(Path, "stat", permission_denied_for_tracefs)

    result = capability_module._tracefs_observation(
        tracefs_path,
        {"status": "not_found", "content": ""},
        root=tmp_path,
    )

    assert result["directory_status"] == "permission_denied"
    assert result["available_events_status"] == "not_found"
    assert all(
        event["status"] == "unknown"
        for event in result["syscall_events"].values()
    )
    assert result["syscall_event_pairs"]["clock_adjtime"]["pair_status"] == "unknown"
