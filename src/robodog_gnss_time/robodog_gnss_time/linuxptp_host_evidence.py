"""Read-only LinuxPTP host evidence collection; never adjusts clocks or PHCs."""
from dataclasses import dataclass
import glob
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time

from .linuxptp_pmc_parser import (
    parse_pmc_snapshot,
    validate_pmc_snapshot_domain,
)


_PMC_GET_COMMANDS = (
    "GET TIME_STATUS_NP",
    "GET CURRENT_DATA_SET",
    "GET PORT_DATA_SET",
    "GET PARENT_DATA_SET",
    "GET TIME_PROPERTIES_DATA_SET",
    "GET DEFAULT_DATA_SET",
)
_SERVICE_NAME = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.@-]*$")


@dataclass(frozen=True)
class LinuxPtpHostEvidenceConfig:
    interface: str
    domain_number: int
    pmc_socket: str
    transport_specific: int
    target_port_identity: str = ""
    timeout_s: float = 10.0
    ptp4l_service: str = ""
    phc2sys_service: str = ""
    chrony_service: str = ""
    journal_since: str = "15 min ago"

    def __post_init__(self):
        if not isinstance(self.interface, str) or not re.fullmatch(
            r"[A-Za-z0-9_][A-Za-z0-9_.:-]{0,63}", self.interface
        ):
            raise ValueError("interface must be an explicit Linux interface name")
        if isinstance(self.domain_number, bool) or not isinstance(self.domain_number, int):
            raise ValueError("domain_number must be an integer")
        if not 0 <= self.domain_number <= 255:
            raise ValueError("domain_number must fit uint8")
        if isinstance(self.transport_specific, bool) or not isinstance(self.transport_specific, int):
            raise ValueError("transport_specific must be an integer")
        if not 0 <= self.transport_specific <= 15:
            raise ValueError("transport_specific must fit the PTP four-bit field")
        if not isinstance(self.target_port_identity, str):
            raise ValueError("target_port_identity must be a string")
        if self.target_port_identity:
            match = re.fullmatch(
                r"([0-9A-Fa-f]{6}\.[0-9A-Fa-f]{4}\.[0-9A-Fa-f]{6})-([0-9]+)",
                self.target_port_identity,
            )
            if not match or not 1 <= int(match.group(2), 10) <= 65535:
                raise ValueError("target_port_identity must be an explicit LinuxPTP clockIdentity-portNumber")
        if not isinstance(self.pmc_socket, str) or not os.path.isabs(self.pmc_socket):
            raise ValueError("pmc_socket must be an explicit absolute Unix socket path")
        if len(os.fsencode(self.pmc_socket)) >= 108:
            raise ValueError("pmc_socket path is too long for Linux sockaddr_un")
        if isinstance(self.timeout_s, bool) or not isinstance(self.timeout_s, (int, float)):
            raise ValueError("timeout_s must be numeric")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        for name in ("ptp4l_service", "phc2sys_service", "chrony_service"):
            value = getattr(self, name)
            if value and not _SERVICE_NAME.fullmatch(value):
                raise ValueError(f"{name} must be a valid explicit service unit name")
        if not isinstance(self.journal_since, str) or not self.journal_since.strip():
            raise ValueError("journal_since must be non-empty")


def _text(value):
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _run_readonly_command(name, argv, timeout_s, runner=None):
    """Run one argv-only read command and preserve timing/error evidence."""
    if runner is None:
        runner = subprocess.run
    mono_start = time.monotonic_ns()
    realtime_start = time.time_ns()
    try:
        completed = runner(
            list(argv),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
            shell=False,
        )
        status = "ok" if completed.returncode == 0 else "exit_error"
        result = {
            "name": name,
            "argv": list(argv),
            "status": status,
            "returncode": completed.returncode,
            "stdout": _text(completed.stdout),
            "stderr": _text(completed.stderr),
        }
    except FileNotFoundError as exc:
        result = {
            "name": name, "argv": list(argv), "status": "unavailable",
            "returncode": None, "stdout": "", "stderr": str(exc),
        }
    except subprocess.TimeoutExpired as exc:
        result = {
            "name": name, "argv": list(argv), "status": "timeout",
            "returncode": None, "stdout": _text(exc.stdout), "stderr": _text(exc.stderr),
        }
    except OSError as exc:
        result = {
            "name": name, "argv": list(argv), "status": "os_error",
            "returncode": None, "stdout": "", "stderr": str(exc),
        }
    result["monotonic_start_ns"] = mono_start
    result["realtime_start_ns"] = realtime_start
    result["monotonic_end_ns"] = time.monotonic_ns()
    result["realtime_end_ns"] = time.time_ns()
    return result


def _ptp_boundary_identity(snapshot):
    """Return only PTP identity/state fields suitable for boundary comparison."""
    responses = snapshot.responses
    time_status = responses["TIME_STATUS_NP"].values
    port = responses["PORT_DATA_SET"]
    parent = responses["PARENT_DATA_SET"].values
    default = responses["DEFAULT_DATA_SET"].values
    return {
        "clock_identity": snapshot.clock_identity.lower(),
        "port_identity": str(port.values["portIdentity"]).lower(),
        "port_state": str(port.values["portState"]),
        "domain_number": default["domainNumber"],
        "gm_present": time_status["gmPresent"],
        "gm_identity": str(time_status["gmIdentity"]).lower(),
        "parent_grandmaster_identity": str(parent["grandmasterIdentity"]).lower(),
    }


def collect_linuxptp_host_evidence(config: LinuxPtpHostEvidenceConfig, runner=None):
    """Collect raw read-only host/PTP evidence; never claim host UTC synchronized."""
    if not isinstance(config, LinuxPtpHostEvidenceConfig):
        raise TypeError("config must be LinuxPtpHostEvidenceConfig")
    collection_start = time.monotonic_ns()
    realtime_start = time.time_ns()
    try:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
        boot_id_error = None
    except OSError as exc:
        boot_id = None
        boot_id_error = str(exc)

    pmc_argv = [
        "pmc", "-u", "-s", config.pmc_socket, "-d", str(config.domain_number),
        "-t", f"0x{config.transport_specific:x}", "-b", "0",
    ]
    if config.target_port_identity:
        pmc_argv.append(f"TARGET {config.target_port_identity}")
    pmc_argv.extend(_PMC_GET_COMMANDS)

    # Capture a read-only PTP identity baseline before the host inventory.
    pmc_boundary_before = _run_readonly_command(
        "pmc_identity_boundary_before", pmc_argv, config.timeout_s, runner=runner,
    )
    pmc_boundary_before_validation = {"status": "not_run"}
    if pmc_boundary_before["status"] == "ok":
        try:
            before_snapshot = parse_pmc_snapshot(pmc_boundary_before["stdout"])
            validate_pmc_snapshot_domain(before_snapshot, config.domain_number)
            if config.target_port_identity and (
                str(before_snapshot.responses["PORT_DATA_SET"].values["portIdentity"]).lower()
                != config.target_port_identity.lower()
            ):
                raise ValueError("boundary PORT_DATA_SET identity does not match configured TARGET")
            pmc_boundary_before_validation = {
                "status": "valid",
                "identity": _ptp_boundary_identity(before_snapshot),
            }
        except (TypeError, ValueError) as exc:
            pmc_boundary_before_validation = {"status": "invalid", "error": str(exc)}

    commands = [
        ("timedatectl", ["timedatectl", "show"]),
        ("ethtool_timestamping", ["ethtool", "-T", config.interface]),
        ("chrony_tracking", ["chronyc", "-n", "tracking"]),
        ("chrony_sources", ["chronyc", "-n", "sources", "-v"]),
    ]
    for service in (config.ptp4l_service, config.phc2sys_service, config.chrony_service):
        if service:
            commands.append((f"systemctl_status:{service}", [
                "systemctl", "status", "--no-pager", service,
            ]))
            commands.append((f"systemctl_properties:{service}", [
                "systemctl", "show", "--no-pager",
                "--property=Id,LoadState,ActiveState,SubState,ExecStart,FragmentPath,DropInPaths",
                service,
            ]))
            commands.append((f"journal:{service}", [
                "journalctl", "-u", service, "--since", config.journal_since, "--no-pager",
            ]))

    results = [
        _run_readonly_command(name, argv, config.timeout_s, runner=runner)
        for name, argv in commands
    ]
    # TARGET, when configured, changes only pmc client-side destination
    # selection; it is not a remote SET/CMD management operation.
    pmc_result = _run_readonly_command(
        "pmc_get_snapshot", pmc_argv, config.timeout_s, runner=runner,
    )
    pmc_validation = {"status": "not_run"}
    if pmc_result["status"] == "ok":
        try:
            snapshot = parse_pmc_snapshot(pmc_result["stdout"])
            validate_pmc_snapshot_domain(snapshot, config.domain_number)
            if config.target_port_identity and (
                str(snapshot.responses["PORT_DATA_SET"].values["portIdentity"]).lower()
                != config.target_port_identity.lower()
            ):
                raise ValueError("PORT_DATA_SET identity does not match configured TARGET portIdentity")
            pmc_validation = {
                "status": "valid",
                "clock_identity": snapshot.clock_identity,
                "responses": {
                    dataset: {
                        "port_number": response.port_number,
                        "sequence_id": response.sequence_id,
                        "values": {
                            key: str(value) if hasattr(value, "as_tuple") else value
                            for key, value in response.values.items()
                        },
                    }
                    for dataset, response in snapshot.responses.items()
                },
            }
        except (TypeError, ValueError) as exc:
            pmc_validation = {"status": "invalid", "error": str(exc)}

    # A syntactically valid PTP snapshot is only one part of the host
    # inventory. The six GET responses are sequential observations returned
    # by one pmc process; the management protocol output carries no common
    # sample instant, so do not describe them as an atomic snapshot.
    all_command_results = [pmc_boundary_before] + results + [pmc_result]
    ptp_device_paths = sorted(glob.glob("/dev/ptp*"))
    sysfs_ptp_paths = sorted(glob.glob(
        f"/sys/class/net/{config.interface}/device/ptp/ptp*"
    ))
    try:
        boot_id_after = Path("/proc/sys/kernel/random/boot_id").read_text(
            encoding="ascii"
        ).strip()
        boot_id_after_error = None
    except OSError as exc:
        boot_id_after = None
        boot_id_after_error = str(exc)
    collection_end = time.monotonic_ns()
    realtime_end = time.time_ns()

    monotonic_elapsed_ns = collection_end - collection_start
    realtime_elapsed_ns = realtime_end - realtime_start
    realtime_minus_monotonic_elapsed_ns = (
        realtime_elapsed_ns - monotonic_elapsed_ns
    )

    pmc_boundary_comparison = {"status": "unavailable"}
    if (
        pmc_boundary_before_validation["status"] == "valid"
        and pmc_validation["status"] == "valid"
    ):
        before_identity = pmc_boundary_before_validation["identity"]
        after_identity = _ptp_boundary_identity(snapshot)
        changed_fields = [
            key for key in before_identity
            if before_identity[key] != after_identity[key]
        ]
        pmc_boundary_comparison = {
            "status": "unchanged" if not changed_fields else "changed",
            "changed_fields": changed_fields,
            "before": before_identity,
            "after": after_identity,
            "interpretation": (
                "Boundary comparison only; matching endpoints do not prove "
                "that no transient GM or port-state change occurred inside "
                "the collection window."
            ),
        }

    collection_issues = []
    if pmc_boundary_before_validation["status"] != "valid":
        collection_issues.append(
            f"pmc_identity_boundary_before_not_valid:{pmc_boundary_before_validation['status']}"
        )
    if pmc_boundary_comparison["status"] == "changed":
        collection_issues.append("ptp_identity_changed_between_collection_boundaries")
    if not boot_id:
        collection_issues.append("host_boot_id_unavailable")
    if not boot_id_after:
        collection_issues.append("host_boot_id_unavailable_after_collection")
    elif boot_id and boot_id_after != boot_id:
        collection_issues.append("host_boot_id_changed_during_collection")
    for command_result in all_command_results:
        if command_result["status"] != "ok":
            collection_issues.append(
                f"command_not_ok:{command_result['name']}:{command_result['status']}"
            )
    if pmc_validation["status"] != "valid":
        collection_issues.append(
            f"pmc_snapshot_not_valid:{pmc_validation['status']}"
        )
    collection_status = "complete" if not collection_issues else "partial"

    return {
        "schema_version": 1,
        "evidence_kind": "read_only_linuxptp_host_inventory",
        "host_boot_id": boot_id,
        "host_boot_id_error": boot_id_error,
        "host_boot_id_after_collection": boot_id_after,
        "host_boot_id_after_collection_error": boot_id_after_error,
        "interface": config.interface,
        "expected_ptp_domain_number": config.domain_number,
        "expected_transport_specific": config.transport_specific,
        "expected_target_port_identity": config.target_port_identity or None,
        "pmc_socket": config.pmc_socket,
        "collection_monotonic_start_ns": collection_start,
        "collection_realtime_start_ns": realtime_start,
        "collection_monotonic_end_ns": collection_end,
        "collection_realtime_end_ns": realtime_end,
        "collection_clock_consistency": {
            "status": "measured_no_threshold",
            "monotonic_elapsed_ns": monotonic_elapsed_ns,
            "realtime_elapsed_ns": realtime_elapsed_ns,
            "realtime_minus_monotonic_elapsed_ns": (
                realtime_minus_monotonic_elapsed_ns
            ),
            "interpretation": (
                "Raw elapsed-time difference only; includes realtime clock "
                "slew/step and sampling-call overhead. No threshold or "
                "synchronization verdict is applied."
            ),
        },
        "pmc_snapshot_observation": {
            "atomicity": "not_guaranteed",
            "observation_model": "sequential_management_responses",
            "simultaneity": "not_proven",
            "monotonic_start_ns": pmc_result["monotonic_start_ns"],
            "monotonic_end_ns": pmc_result["monotonic_end_ns"],
            "per_dataset_sample_timestamps_available": False,
        },
        "ptp_device_paths": ptp_device_paths,
        "interface_sysfs_ptp_paths": [os.path.realpath(path) for path in sysfs_ptp_paths],
        "commands": all_command_results,
        "pmc_snapshot_validation": pmc_validation,
        "pmc_identity_boundary_before_validation": pmc_boundary_before_validation,
        "pmc_identity_boundary_comparison": pmc_boundary_comparison,
        "collection_status": collection_status,
        "collection_issues": collection_issues,
        "host_time_sync_claim": "not_evaluated",
        "safety_note": (
            "This report is raw host/PTP evidence only. It does not prove "
            "PHC-to-CLOCK_REALTIME discipline or UTC traceability."
        ),
    }


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Collect read-only LinuxPTP host evidence; no clock adjustments."
    )
    parser.add_argument("--interface", required=True)
    parser.add_argument("--domain", required=True, type=int)
    parser.add_argument("--pmc-socket", required=True)
    parser.add_argument("--transport-specific", required=True, type=lambda value: int(value, 0))
    parser.add_argument("--target-port-identity", default="")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--ptp4l-service", default="")
    parser.add_argument("--phc2sys-service", default="")
    parser.add_argument("--chrony-service", default="")
    parser.add_argument("--journal-since", default="15 min ago")
    parser.add_argument("--output", default="")
    args = parser.parse_args(argv)
    config = LinuxPtpHostEvidenceConfig(
        interface=args.interface,
        domain_number=args.domain,
        pmc_socket=args.pmc_socket,
        transport_specific=args.transport_specific,
        target_port_identity=args.target_port_identity,
        timeout_s=args.timeout,
        ptp4l_service=args.ptp4l_service,
        phc2sys_service=args.phc2sys_service,
        chrony_service=args.chrony_service,
        journal_since=args.journal_since,
    )
    report = collect_linuxptp_host_evidence(config)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    else:
        print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
