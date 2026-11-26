"""Read-only kernel tracing capability inventory for clock-step evidence.

This module inspects existing procfs/sysfs files only. It never mounts tracefs,
enables events, writes trace controls, loads BPF, or changes host configuration.
"""
import gzip
import os
from pathlib import Path
import platform
import re
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
        with open(path, "rb") as stream:
            stream.read(1)
        return {
            "status": "present_readable",
            "path": str(path),
            "size_bytes": stat.st_size,
            "interpretation": "BTF file presence/readability only; BTF validity and BPF load permission are not tested.",
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
            if mountpoint == target or (
                mountpoint != os.sep
                and target.startswith(mountpoint.rstrip(os.sep) + os.sep)
            ):
                mounts.append({
                    "mountpoint": mountpoint,
                    "filesystem_type": fields[separator + 1],
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
        event_results[event_name] = {
            "status": "available" if format_result["status"] == "ok" else "format_unreadable",
            "format_status": format_result["status"],
            "format_fields": fields,
            "format": format_result["content"] if format_result["status"] == "ok" else None,
        }
    return {
        "path": str(path),
        "directory_status": "present" if path.is_dir() else "not_found",
        "mounts": mounts,
        "available_events_status": available["status"],
        "syscall_events": event_results,
        "interpretation": (
            "Event availability is taken from this kernel's available_events and each event's "
            "format file. No event was enabled and no trace buffer was read."
        ),
    }


def collect_linux_kernel_trace_capabilities(root="/", kernel_release=None):
    """Inventory config, BTF and syscall tracepoint visibility without changing state."""
    start_mono = time.monotonic_ns()
    start_real = time.time_ns()
    root = Path(root)

    def rooted(path):
        return root / str(path).lstrip("/")

    release = kernel_release or platform.release()
    uname = platform.uname()
    mountinfo = _read_text(rooted("/proc/self/mountinfo"))
    config_sources = {
        "proc_config_gz": _read_config_source(rooted("/proc/config.gz"), compressed=True),
        "boot_config": _read_config_source(rooted(f"/boot/config-{release}")),
    }
    tracefs = [
        _tracefs_observation(rooted("/sys/kernel/tracing"), mountinfo, root=root),
        _tracefs_observation(rooted("/sys/kernel/debug/tracing"), mountinfo, root=root),
    ]
    btf = _btf_observation(rooted("/sys/kernel/btf/vmlinux"))
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
        "status": status,
        "kernel": {
            "release": release,
            "machine": uname.machine,
            "system": uname.system,
        },
        "collection_clock": {
            "monotonic_start_ns": start_mono,
            "realtime_start_ns": start_real,
            "monotonic_end_ns": end_mono,
            "realtime_end_ns": end_real,
        },
        "kernel_config_sources": config_sources,
        "btf": btf,
        "tracefs": tracefs,
        "candidate_syscall_events": list(_SYSCALL_EVENTS),
        "interpretation": (
            "Read-only capability inventory only. A missing event/config/BTF observation is not "
            "proof that no alternative instrumentation exists. Presence does not prove that "
            "a tracing program can be loaded, that user-memory reads work, or that events can "
            "be captured without loss. No tracefs mount, event enablement, audit rule, perf "
            "session, eBPF load, or host configuration change was attempted."
        ),
    }
