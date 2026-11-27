"""Read-only dpkg provenance inventory for the running Linux kernel."""
import argparse
import json
import os
from pathlib import Path
import platform
import subprocess


def _command(argv, timeout=5):
    try:
        result = subprocess.run(
            argv, check=False, capture_output=True, text=True, timeout=timeout
        )
    except FileNotFoundError as exc:
        return {"status": "command_not_found", "stdout": "", "stderr": str(exc)}
    except PermissionError as exc:
        return {"status": "permission_denied", "stdout": "", "stderr": str(exc)}
    except subprocess.TimeoutExpired as exc:
        return {"status": "timeout", "stdout": exc.stdout or "",
                "stderr": exc.stderr or ""}
    except OSError as exc:
        return {"status": "command_error", "stdout": "", "stderr": str(exc)}
    return {
        "status": "ok" if result.returncode == 0 else "command_failed",
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def _package_row(package_name):
    fmt = (
        "${binary:Package}\t${Version}\t${source:Package}\t"
        "${source:Version}\t${db:Status-Abbrev}\n"
    )
    result = _command(["dpkg-query", "-W", "-f=" + fmt, package_name])
    if result["status"] != "ok":
        return {"package_query": package_name, **result}
    fields = result["stdout"].rstrip("\n").split("\t")
    if len(fields) != 5:
        return {
            "package_query": package_name,
            "status": "parse_error",
            "stdout": result["stdout"],
        }
    binary, version, source, source_version, status = fields
    installed = status.startswith("ii")
    return {
        "package_query": package_name,
        "status": "installed" if installed else "not_installed",
        "binary_package": binary,
        "binary_version": version,
        "source_package": source or None,
        "source_version": source_version or None,
        "dpkg_status": status,
    }


def _file_owner(path):
    result = _command(["dpkg-query", "-S", path])
    if result["status"] != "ok":
        return {"path": path, **result}
    owners = []
    for line in result["stdout"].splitlines():
        if ": " in line:
            package, owned_path = line.split(": ", 1)
            owners.append({"package": package, "path": owned_path})
    return {"path": path, "status": "owned" if owners else "no_owner", "owners": owners}


def collect_linux_kernel_package_provenance(kernel_release=None):
    release = kernel_release or os.uname().release
    architecture = platform.machine()
    candidate_packages = (
        f"linux-image-{release}",
        f"linux-image-unsigned-{release}",
        f"linux-modules-{release}",
        f"linux-modules-extra-{release}",
        f"linux-headers-{release}",
        f"linux-headers-{release}-generic",
        f"linux-buildinfo-{release}",
    )
    package_rows = [_package_row(name) for name in candidate_packages]
    file_owners = [
        _file_owner(f"/boot/vmlinuz-{release}"),
        _file_owner(f"/boot/config-{release}"),
        _file_owner(f"/usr/lib/modules/{release}/vmlinuz"),
    ]
    installed = [row for row in package_rows if row.get("status") == "installed"]
    source_identities = sorted({
        (row.get("source_package"), row.get("source_version"))
        for row in installed
        if row.get("source_package") and row.get("source_version")
    })
    owner_packages = sorted({
        owner["package"]
        for item in file_owners
        for owner in item.get("owners", [])
    })
    status = "observed" if installed and owner_packages else "partial"
    if len(source_identities) > 1:
        status = "partial"
    return {
        "schema_version": 1,
        "collection_mode": "live_host_read_only",
        "status": status,
        "kernel": {
            "release": release,
            "architecture": architecture,
            "proc_version": _command(["uname", "-v"]).get("stdout", "").strip(),
        },
        "packages": package_rows,
        "kernel_file_owners": file_owners,
        "source_package_identities": [
            {"source_package": name, "source_version": version}
            for name, version in source_identities
        ],
        "interpretation": (
            "dpkg ownership and installed control metadata only. This identifies "
            "the installed binary/source package relationship; it does not fetch "
            "or verify the source tarball, Ubuntu patch series, build configuration, "
            "or equivalence to an upstream kernel tag."
        ),
        "side_effects": {
            "dpkg_database_modified": False,
            "apt_indexes_modified": False,
            "packages_installed_or_removed": False,
            "kernel_or_bpf_configuration_modified": False,
            "bpf_loaded_or_attached": False,
            "tracepoints_enabled": False,
            "clock_changed": False,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Read-only dpkg provenance inventory for the running kernel"
    )
    parser.add_argument("--output", help="write JSON report to this path")
    parser.add_argument("--kernel-release", help="override release for fixture/testing")
    args = parser.parse_args(argv)
    report = collect_linux_kernel_package_provenance(args.kernel_release)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0 if report["status"] == "observed" else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["collect_linux_kernel_package_provenance", "main"]
