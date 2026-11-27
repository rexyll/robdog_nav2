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


def _package_row(package_name, role):
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
        "binary_package_role": role,
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
        (f"linux-image-{release}", "signed_image"),
        (f"linux-image-unsigned-{release}", "kernel_image_unsigned"),
        (f"linux-modules-{release}", "kernel_modules"),
        (f"linux-modules-extra-{release}", "kernel_modules_extra"),
        (f"linux-headers-{release}", "kernel_headers"),
        (f"linux-buildinfo-{release}", "kernel_buildinfo"),
    )
    package_rows = [_package_row(name, role) for name, role in candidate_packages]
    file_owners = [
        _file_owner(f"/boot/vmlinuz-{release}"),
        _file_owner(f"/boot/config-{release}"),
        _file_owner(f"/usr/lib/modules/{release}/vmlinuz"),
    ]
    installed = [row for row in package_rows if row.get("status") == "installed"]
    installed_names = {row.get("binary_package") for row in installed}
    def identities_for(roles):
        return sorted({
            (row.get("source_package"), row.get("source_version"))
            for row in installed
            if row.get("binary_package_role") in roles
            and row.get("source_package") and row.get("source_version")
        })

    source_identities = sorted({
        (row.get("source_package"), row.get("source_version"))
        for row in installed
        if row.get("source_package") and row.get("source_version")
    })
    kernel_code_roles = {
        "kernel_image_unsigned", "kernel_modules",
        "kernel_modules_extra", "kernel_headers", "kernel_buildinfo",
    }
    kernel_code_packages = [
        row for row in installed
        if row.get("binary_package_role") in kernel_code_roles
    ]
    kernel_source_identities = identities_for(kernel_code_roles)
    signed_image_source_identities = identities_for({"signed_image"})
    # The running image must itself be owned by an installed image package.
    # Ownership of config/modules files is not a substitute for /boot/vmlinuz.
    image_path_owners = {
        owner["package"].split(":", 1)[0]
        for owner in file_owners[0].get("owners", [])
    }
    installed_image_packages = {
        row.get("binary_package", "").split(":", 1)[0] for row in installed
        if row.get("binary_package_role") in {"signed_image", "kernel_image_unsigned"}
    }
    image_owner_observed = bool(installed_image_packages.intersection(image_path_owners))
    issues = []
    if not installed:
        issues.append({"code": "no_installed_kernel_binary_packages"})
    if not image_owner_observed:
        issues.append({"code": "installed_kernel_image_owner_not_observed"})
    if not kernel_source_identities:
        issues.append({"code": "kernel_source_package_identity_unavailable"})
    elif len(kernel_source_identities) > 1:
        issues.append({"code": "multiple_kernel_source_package_identities"})
    if any(
        not row.get("source_package") or not row.get("source_version")
        for row in kernel_code_packages
    ):
        issues.append({"code": "installed_kernel_code_package_source_metadata_incomplete"})
    if len(signed_image_source_identities) > 1:
        issues.append({"code": "multiple_signed_image_source_package_identities"})
    status = "observed" if not issues else "partial"
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
        "kernel_source_package_identity": (
            {"source_package": kernel_source_identities[0][0],
             "source_version": kernel_source_identities[0][1]}
            if len(kernel_source_identities) == 1 else None
        ),
        "signed_image_source_package_identities": [
            {"source_package": name, "source_version": version}
            for name, version in signed_image_source_identities
        ],
        "installed_binary_packages": sorted(installed_names),
        "issues": issues,
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
