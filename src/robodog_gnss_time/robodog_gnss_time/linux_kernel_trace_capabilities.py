"""Read-only kernel tracing capability inventory for clock-step evidence.

This module inspects existing procfs/sysfs files only. It never mounts tracefs,
enables events, writes trace controls, loads BPF, or changes host configuration.
"""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import time


_CONFIG_KEYS = (
    "CONFIG_FTRACE",
    "CONFIG_TRACEPOINTS",
    "CONFIG_HAVE_SYSCALL_TRACEPOINTS",
    "CONFIG_FTRACE_SYSCALLS",
    "CONFIG_BPF",
    "CONFIG_BPF_SYSCALL",
    "CONFIG_BPF_EVENTS",
    "CONFIG_DEBUG_INFO_BTF",
    "CONFIG_DEBUG_INFO_BTF_MODULES",
    "CONFIG_IKCONFIG",
    "CONFIG_IKCONFIG_PROC",
    "CONFIG_PERF_EVENTS",
    "CONFIG_KPROBE_EVENTS",
)
_HANDLER_SYMBOLS = ("do_clock_adjtime", "do_adjtimex")
_SYSCALL_EVENTS = (
    "sys_enter_clock_adjtime",
    "sys_exit_clock_adjtime",
    "sys_enter_adjtimex",
    "sys_exit_adjtimex",
    "sys_enter_clock_settime",
    "sys_exit_clock_settime",
    "sys_enter_settimeofday",
    "sys_exit_settimeofday",
)
_FIELD_RE = re.compile(r"^\s*field:[^;]+\s+([A-Za-z_][A-Za-z0-9_]*)(?:\[[^]]+\])?;", re.MULTILINE)
_MOUNT_ESCAPE_RE = re.compile(r"\\([0-7]{3})")


def _decode_mount_path(value):
    return _MOUNT_ESCAPE_RE.sub(lambda match: chr(int(match.group(1), 8)), value)


def _read_text(path):
    try:
        with open(path, "r", encoding="utf-8") as stream:
            return {"status": "ok", "content": stream.read()}
    except FileNotFoundError:
        return {"status": "not_found", "content": ""}
    except PermissionError as exc:
        return {"status": "permission_denied", "content": "", "error": str(exc)}
    except (OSError, UnicodeError) as exc:
        return {"status": "read_error", "content": "", "error": str(exc)}


def _parse_kernel_config(content):
    parsed = {}
    for line in content.splitlines():
        if line.startswith("# ") and line.endswith(" is not set"):
            key = line[2:-len(" is not set")]
            if key.startswith("CONFIG_"):
                parsed[key] = "n"
        elif line.startswith("CONFIG_") and "=" in line:
            key, value = line.split("=", 1)
            parsed[key] = value.strip()
    return {key: parsed.get(key) for key in _CONFIG_KEYS}


def _read_config_source(path, compressed=False):
    try:
        if compressed:
            with gzip.open(path, "rt", encoding="utf-8", errors="replace") as stream:
                content = stream.read()
        else:
            with open(path, "r", encoding="utf-8") as stream:
                content = stream.read()
        return {
            "status": "observed",
            "path": str(path),
            "settings": _parse_kernel_config(content),
            "interpretation": "Configuration file contents only; not a runtime feature probe.",
        }
    except FileNotFoundError:
        return {"status": "not_found", "path": str(path), "settings": {}}
    except PermissionError as exc:
        return {"status": "permission_denied", "path": str(path), "settings": {}, "error": str(exc)}
    except (OSError, UnicodeError) as exc:
        return {"status": "read_error", "path": str(path), "settings": {}, "error": str(exc)}


def _btf_observation(path):
    try:
        stat = Path(path).stat()
        digest = hashlib.sha256()
        with open(path, "rb") as stream:
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
        return {
            "status": "present_readable",
            "path": str(path),
            "size_bytes": stat.st_size,
            "sha256": digest.hexdigest(),
            "hash_algorithm": "sha256",
            "interpretation": "BTF file presence/readability and byte identity only; hash does not validate BTF semantics or BPF load permission.",
        }
    except FileNotFoundError:
        return {"status": "not_found", "path": str(path)}
    except PermissionError as exc:
        return {"status": "permission_denied", "path": str(path), "error": str(exc)}
    except OSError as exc:
        return {"status": "read_error", "path": str(path), "error": str(exc)}


def _mount_inventory(path, mountinfo, root="/"):
    target = os.path.normpath(str(path))
    mounts = []
    if mountinfo["status"] == "ok":
        for line in mountinfo["content"].splitlines():
            fields = line.split()
            if len(fields) < 10 or "-" not in fields:
                continue
            separator = fields.index("-")
            if separator + 1 >= len(fields):
                continue
            mountpoint_value = _decode_mount_path(fields[4])
            mountpoint = os.path.normpath(
                str(Path(root) / mountpoint_value.lstrip("/"))
            )
            filesystem_type = fields[separator + 1]
            is_target_or_parent = mountpoint == target or (
                mountpoint != os.sep
                and target.startswith(mountpoint.rstrip(os.sep) + os.sep)
            )
            if is_target_or_parent and filesystem_type in {"tracefs", "debugfs"}:
                mounts.append({
                    "mountpoint": mountpoint,
                    "filesystem_type": filesystem_type,
                    "mount_relation": "exact" if mountpoint == target else "ancestor",
                    "mount_options": fields[5],
                })
    return mounts


def _tracefs_observation(path, mountinfo, root="/"):
    path = Path(path)
    available_path = path / "available_events"
    available = _read_text(available_path)
    mounts = _mount_inventory(path, mountinfo, root=root)
    event_results = {}
    available_names = set(available["content"].splitlines()) if available["status"] == "ok" else set()
    for event_name in _SYSCALL_EVENTS:
        qualified = "syscalls:" + event_name
        if available["status"] != "ok":
            event_results[event_name] = {"status": "unknown"}
            continue
        if qualified not in available_names:
            event_results[event_name] = {"status": "not_listed"}
            continue
        format_path = path / "events" / "syscalls" / event_name / "format"
        format_result = _read_text(format_path)
        fields = (
            _FIELD_RE.findall(format_result["content"])
            if format_result["status"] == "ok" else []
        )
        enable_result = _read_text(
            path / "events" / "syscalls" / event_name / "enable"
        )
        enable_value = enable_result["content"].strip()
        if enable_result["status"] == "ok" and enable_value in {"0", "1", "X", "?"}:
            enable_observation = {
                "status": "observed",
                "value": enable_value,
                "meaning": {
                    "0": "all_affected_events_disabled",
                    "1": "all_affected_events_enabled",
                    "X": "mixed_enabled_and_disabled",
                    "?": "does_not_affect_any_event",
                }[enable_value],
            }
        else:
            enable_observation = {
                "status": enable_result["status"] if enable_result["status"] != "ok" else "unknown_value",
                "value": None,
            }
        event_results[event_name] = {
            "status": "available" if format_result["status"] == "ok" else "format_unreadable",
            "format_status": format_result["status"],
            "format_fields": fields,
            "format": format_result["content"] if format_result["status"] == "ok" else None,
            "enable_observation": enable_observation,
        }
    pairs = {}
    for syscall_name in ("clock_adjtime", "adjtimex", "clock_settime", "settimeofday"):
        entry = event_results["sys_enter_" + syscall_name]["status"]
        exit_status = event_results["sys_exit_" + syscall_name]["status"]
        if entry == "available" and exit_status == "available":
            pair_status = "complete"
        elif entry == "available" or exit_status == "available":
            pair_status = "partial"
        elif entry == "not_listed" and exit_status == "not_listed":
            pair_status = "not_listed"
        else:
            pair_status = "unknown"
        pairs[syscall_name] = {
            "entry_status": entry,
            "exit_status": exit_status,
            "pair_status": pair_status,
        }
    return {
        "path": str(path),
        "directory_status": "present" if path.is_dir() else "not_found",
        "mounts": mounts,
        "available_events_status": available["status"],
        "syscall_events": event_results,
        "syscall_event_pairs": pairs,
        "interpretation": (
            "Event availability is taken from this kernel's available_events and each event's "
            "format file. Event enable files were read only; their state may belong to another "
            "process. No event was enabled and no trace buffer was read."
        ),
    }


def _handler_symbol_observation(path):
    """Read available_filter_functions; presence is not proof of attachability."""
    result = _read_text(Path(path) / "available_filter_functions")
    if result["status"] != "ok":
        return {
            "status": result["status"],
            "symbols": {
                name: {"status": "unknown"} for name in _HANDLER_SYMBOLS
            },
            "interpretation": (
                "Function tracing symbol inventory unavailable; no attach attempt was made."
            ),
        }
    available = set()
    for line in result["content"].splitlines():
        fields = line.split()
        if fields:
            available.add(fields[0])
    return {
        "status": "observed",
        "symbols": {
            name: {"status": "listed" if name in available else "not_listed"}
            for name in _HANDLER_SYMBOLS
        },
        "interpretation": (
            "available_filter_functions membership is only a function-tracing visibility "
            "observation. It does not prove BTF FUNC availability, fentry/kprobe verifier "
            "acceptance, symbol stability, or permission to attach."
        ),
    }


def _resolve_btf_type(type_id, types_by_id, seen=None, depth=0):
    """Resolve a BTF type reference conservatively, preserving modifier layers."""
    if type_id == 0:
        return {"status": "resolved", "kind": "VOID", "name": "void"}
    if isinstance(type_id, bool) or not isinstance(type_id, int):
        return {"status": "invalid_type_id"}
    if depth >= 8:
        return {"status": "depth_limit", "type_id": type_id}
    seen = set() if seen is None else set(seen)
    if type_id in seen:
        return {"status": "cycle", "type_id": type_id}
    item = types_by_id.get(type_id)
    if not isinstance(item, dict):
        return {"status": "missing_type", "type_id": type_id}
    kind = item.get("kind")
    result = {
        "status": "resolved",
        "type_id": type_id,
        "kind": kind,
        "name": item.get("name"),
    }
    if kind in {"TYPEDEF", "CONST", "VOLATILE", "RESTRICT", "TYPE_TAG", "PTR"}:
        seen.add(type_id)
        target = item.get("type_id")
        result["target"] = _resolve_btf_type(
            target, types_by_id, seen=seen, depth=depth + 1
        )
        if result["target"].get("status") != "resolved":
            result["status"] = "incomplete"
    return result


def _parse_btf_handler_functions(document):
    """Parse bpftool JSON and validate FUNC -> FUNC_PROTO -> parameter references."""
    if not isinstance(document, dict) or not isinstance(document.get("types"), list):
        return {
            "status": "invalid_document",
            "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
        }
    types_by_id = {}
    for item in document["types"]:
        if not isinstance(item, dict):
            return {
                "status": "invalid_document",
                "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
            }
        type_id = item.get("id")
        if isinstance(type_id, bool) or not isinstance(type_id, int) or type_id <= 0:
            return {
                "status": "invalid_document",
                "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
            }
        if type_id in types_by_id:
            return {
                "status": "duplicate_type_id",
                "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
            }
        types_by_id[type_id] = item

    found = {}
    for item in types_by_id.values():
        if item.get("kind") == "FUNC" and item.get("name") in _HANDLER_SYMBOLS:
            found.setdefault(item["name"], []).append(item)

    symbols = {}
    for name in _HANDLER_SYMBOLS:
        records = found.get(name, [])
        if not records:
            symbols[name] = {"status": "func_record_absent"}
            continue
        if len(records) != 1:
            symbols[name] = {"status": "ambiguous_func_records", "record_count": len(records)}
            continue
        func = records[0]
        proto_id = func.get("type_id")
        proto = types_by_id.get(proto_id) if isinstance(proto_id, int) and not isinstance(proto_id, bool) else None
        if not isinstance(proto, dict) or proto.get("kind") != "FUNC_PROTO":
            symbols[name] = {
                "status": "invalid_func_proto_reference",
                "func_type_id": proto_id,
            }
            continue
        params = proto.get("params")
        vlen = proto.get("vlen")
        if (
            not isinstance(params, list)
            or isinstance(vlen, bool)
            or not isinstance(vlen, int)
            or vlen != len(params)
        ):
            symbols[name] = {
                "status": "invalid_func_proto",
                "func_type_id": proto_id,
                "declared_parameter_count": vlen,
                "observed_parameter_count": len(params) if isinstance(params, list) else None,
            }
            continue
        return_type = _resolve_btf_type(proto.get("ret_type_id"), types_by_id)
        parsed_params = []
        refs_valid = return_type.get("status") == "resolved"
        for index, param in enumerate(params):
            if not isinstance(param, dict):
                refs_valid = False
                parsed_params.append({"index": index, "status": "invalid_parameter"})
                continue
            resolved = _resolve_btf_type(param.get("type_id"), types_by_id)
            if resolved.get("status") != "resolved":
                refs_valid = False
            parsed_params.append({
                "index": index,
                "name": param.get("name"),
                "type": resolved,
            })
        symbols[name] = {
            "status": "prototype_resolved" if refs_valid else "prototype_incomplete",
            "func_type_id": func.get("id"),
            "func_proto_type_id": proto_id,
            "linkage": func.get("linkage"),
            "return_type": return_type,
            "parameter_count": vlen,
            "parameters": parsed_params,
        }
    return {"status": "observed", "symbols": symbols}


def _btf_handler_function_inventory(path):
    """Optionally inspect vmlinux BTF FUNC and FUNC_PROTO records via bpftool JSON."""
    tool = shutil.which("bpftool")
    if not tool:
        return {
            "status": "tool_unavailable",
            "tool": None,
            "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
            "interpretation": "bpftool is unavailable; BTF function prototypes were not inspected.",
        }
    if not Path(path).is_file():
        return {
            "status": "btf_unavailable",
            "tool": tool,
            "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
            "interpretation": "The vmlinux BTF file is unavailable; no BTF dump was attempted.",
        }
    version = None
    version_status = "unknown"
    try:
        version_result = subprocess.run(
            [tool, "version"], check=False, capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=5,
        )
        if version_result.returncode == 0:
            version = (version_result.stdout or version_result.stderr).strip()[:1000]
            version_status = "observed"
        else:
            version_status = "command_failed"
    except subprocess.TimeoutExpired:
        version_status = "timeout"
    except OSError:
        version_status = "execution_error"
    command = [tool, "-j", "btf", "dump", "file", str(path)]
    try:
        completed = subprocess.run(
            command, check=False, capture_output=True, text=True,
            encoding="utf-8", errors="replace", timeout=20,
        )
    except subprocess.TimeoutExpired:
        status = "timeout"
        detail = "bpftool BTF JSON dump exceeded 20 seconds."
    except OSError as exc:
        status = "execution_error"
        detail = str(exc)
    else:
        if completed.returncode != 0:
            status = "command_failed"
            detail = completed.stderr.strip()[:2000]
        else:
            try:
                document = json.loads(completed.stdout)
            except (TypeError, json.JSONDecodeError) as exc:
                status = "invalid_json"
                detail = str(exc)
            else:
                parsed = _parse_btf_handler_functions(document)
                return {
                    "status": parsed["status"],
                    "tool": tool,
                    "tool_version": {"status": version_status, "value": version},
                    "command": command,
                    "symbols": parsed["symbols"],
                    "interpretation": (
                        "FUNC records are linked to FUNC_PROTO records and referenced return/"
                        "parameter type IDs are checked against this BTF dump. Resolved type "
                        "chains are descriptive only; this does not prove fentry attachability, "
                        "verifier acceptance, symbol stability, or permission to attach."
                    ),
                }
    return {
        "status": status,
        "tool": tool,
        "tool_version": {"status": version_status, "value": version},
        "command": command,
        "detail": detail,
        "symbols": {name: {"status": "unknown"} for name in _HANDLER_SYMBOLS},
        "interpretation": "BTF function prototype inventory could not be completed; no attach was attempted.",
    }

def _runtime_policy_observation(rooted):
    """Read BPF/perf policy and this process security context without changing it."""
    policy_paths = {
        "unprivileged_bpf_disabled": "/proc/sys/kernel/unprivileged_bpf_disabled",
        "perf_event_paranoid": "/proc/sys/kernel/perf_event_paranoid",
        "kptr_restrict": "/proc/sys/kernel/kptr_restrict",
        "lockdown": "/sys/kernel/security/lockdown",
        "process_status": "/proc/self/status",
    }
    observations = {name: _read_text(rooted(path)) for name, path in policy_paths.items()}
    status_fields = {}
    for line in observations["process_status"]["content"].splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            if key in {"Uid", "Gid", "CapEff", "CapBnd", "NoNewPrivs", "Seccomp", "Seccomp_filters"}:
                status_fields[key] = value.strip()
    lockdown_text = observations["lockdown"]["content"].strip()
    lockdown_match = re.search(r"\[([^]]+)\]", lockdown_text)
    return {
        "sysctls": {
            name: {
                "status": item["status"],
                "value": item["content"].strip() if item["status"] == "ok" else None,
            }
            for name, item in observations.items()
            if name not in {"process_status", "lockdown"}
        },
        "lockdown": {
            "status": observations["lockdown"]["status"],
            "active_mode": lockdown_match.group(1) if lockdown_match else None,
            "raw": lockdown_text if observations["lockdown"]["status"] == "ok" else None,
        },
        "process_security": {
            "status": observations["process_status"]["status"],
            "fields": status_fields,
            "interpretation": "Effective capabilities and seccomp are observations for this process only; they do not prove BPF program load/attach permission.",
        },
        "interpretation": "These policy values are read-only preflight evidence. Kernel policy, LSM, namespaces, capabilities, BPF command type, and program/map specifics can still deny an operation; no BPF syscall was attempted.",
    }


def collect_linux_kernel_trace_capabilities(root="/", kernel_release=None):
    """Inventory kernel tracing capabilities with explicit host/fixture provenance."""
    start_mono = time.monotonic_ns()
    start_real = time.time_ns()
    root = Path(root)
    try:
        root_resolved = root.resolve()
        is_live_host = root_resolved == Path("/").resolve()
    except OSError:
        root_resolved = root
        is_live_host = False

    def rooted(path):
        return root / str(path).lstrip("/")

    uname = platform.uname()
    if kernel_release:
        release = kernel_release
        release_source = "explicit_override"
    elif is_live_host:
        release = uname.release
        release_source = "collector_host_uname"
    else:
        release = None
        release_source = "unknown_for_rootfs_fixture"

    mountinfo = _read_text(rooted("/proc/self/mountinfo"))
    boot_id = _read_text(rooted("/proc/sys/kernel/random/boot_id"))
    config_sources = {
        "proc_config_gz": _read_config_source(rooted("/proc/config.gz"), compressed=True),
        "boot_config": (
            _read_config_source(rooted(f"/boot/config-{release}"))
            if release else {"status": "not_probed", "path": None, "settings": {}}
        ),
    }
    tracefs = [
        _tracefs_observation(rooted("/sys/kernel/tracing"), mountinfo, root=root),
        _tracefs_observation(rooted("/sys/kernel/debug/tracing"), mountinfo, root=root),
    ]
    btf_path = rooted("/sys/kernel/btf/vmlinux")
    btf = _btf_observation(btf_path)
    handler_symbols = [
        _handler_symbol_observation(rooted("/sys/kernel/tracing")),
        _handler_symbol_observation(rooted("/sys/kernel/debug/tracing")),
    ]
    readable_tracefs = [
        item for item in tracefs if item["available_events_status"] == "ok"
    ]
    status = (
        "observed" if readable_tracefs
        else "partial" if any(item["directory_status"] == "present" for item in tracefs)
        else "unavailable"
    )
    end_mono = time.monotonic_ns()
    end_real = time.time_ns()
    return {
        "schema_version": 2,
        "status": status,
        "collection_mode": "live_host" if is_live_host else "rootfs_fixture",
        "collection_root": str(root_resolved),
        "kernel": {
            "release": release,
            "release_source": release_source,
            "machine": uname.machine if is_live_host else None,
            "system": uname.system if is_live_host else None,
            "boot_id": boot_id["content"].strip() if boot_id["status"] == "ok" else None,
            "boot_id_status": boot_id["status"],
        },
        "collector_host": {
            "release": uname.release,
            "machine": uname.machine,
            "system": uname.system,
            "note": (
                "Same process/kernel as target observations."
                if is_live_host else
                "Collector host metadata only; not attributed to the rootfs fixture target."
            ),
        },
        "collection_clock": {
            "clock_ids": {"monotonic": "CLOCK_MONOTONIC", "realtime": "CLOCK_REALTIME"},
            "monotonic_start_ns": start_mono,
            "realtime_start_ns": start_real,
            "monotonic_end_ns": end_mono,
            "realtime_end_ns": end_real,
        },
        "kernel_config_sources": config_sources,
        "btf": btf,
        "btf_handler_functions": _btf_handler_function_inventory(btf_path),
        "handler_function_symbols": handler_symbols,
        "runtime_policy": _runtime_policy_observation(rooted),
        "tracefs": tracefs,
        "candidate_syscall_events": list(_SYSCALL_EVENTS),
        "interpretation": (
            "Read-only capability inventory only. collection_mode distinguishes live host from "
            "rootfs fixture; collector_host metadata is not target metadata for fixtures. A "
            "missing event/config/BTF observation is not proof that no alternative instrumentation "
            "exists. Presence does not prove that a tracing program can be loaded, that handler "
            "symbols are BTF attachable, that user-memory reads work, or that events can be "
            "captured without loss. Handler symbol visibility is not proof of fentry/kprobe "
            "attachability. No tracefs mount, event enablement, audit rule, perf session, eBPF "
            "load, or host configuration change was attempted."
        ),
    }


def main(argv=None):
    """Emit a standalone, read-only kernel tracing capability report."""
    parser = argparse.ArgumentParser(
        description="Inventory Linux kernel clock-step tracing capabilities without enabling tracing."
    )
    parser.add_argument(
        "--root",
        default="/",
        help="Root directory for procfs/sysfs fixture inspection (default: live host root).",
    )
    parser.add_argument(
        "--kernel-release",
        default=None,
        help="Kernel release used to locate /boot/config-<release>; defaults to running kernel.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional JSON output path; otherwise print the report to stdout.",
    )
    args = parser.parse_args(argv)
    report = collect_linux_kernel_trace_capabilities(
        root=args.root,
        kernel_release=args.kernel_release,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
