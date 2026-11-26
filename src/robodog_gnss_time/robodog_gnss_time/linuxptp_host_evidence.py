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
_SERVICE_NAME = re.compile(r"^[A-Za-z0-9_.@-]+$")


@dataclass(frozen=True)
class LinuxPtpHostEvidenceConfig:
    interface: str
    domain_number: int
    pmc_socket: str
    timeout_s: float = 3.0
    ptp4l_service: str = ""
    phc2sys_service: str = ""
    chrony_service: str = ""
    journal_since: str = "15 min ago"

    def __post_init__(self):
        if not isinstance(self.interface, str) or not re.fullmatch(
            r"[A-Za-z0-9_.:-]{1,64}", self.interface
        ):
            raise ValueError("interface must be an explicit Linux interface name")
        if isinstance(self.domain_number, bool) or not isinstance(self.domain_number, int):
            raise ValueError("domain_number must be an integer")
        if not 0 <= self.domain_number <= 255:
            raise ValueError("domain_number must fit uint8")
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
            commands.append((f"journal:{service}", [
                "journalctl", "-u", service, "--since", config.journal_since, "--no-pager",
            ]))

    results = [
        _run_readonly_command(name, argv, config.timeout_s, runner=runner)
        for name, argv in commands
    ]
    pmc_argv = [
        "pmc", "-u", "-s", config.pmc_socket, "-d", str(config.domain_number),
        "-b", "0", *_PMC_GET_COMMANDS,
    ]
    pmc_result = _run_readonly_command(
        "pmc_get_snapshot", pmc_argv, config.timeout_s, runner=runner,
    )
    pmc_validation = {"status": "not_run"}
    if pmc_result["status"] == "ok":
        try:
            snapshot = parse_pmc_snapshot(pmc_result["stdout"])
            validate_pmc_snapshot_domain(snapshot, config.domain_number)
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

    ptp_device_paths = sorted(glob.glob("/dev/ptp*"))
    sysfs_ptp_paths = sorted(glob.glob(
        f"/sys/class/net/{config.interface}/device/ptp/ptp*"
    ))
    collection_end = time.monotonic_ns()
    realtime_end = time.time_ns()
    return {
        "schema_version": 1,
        "evidence_kind": "read_only_linuxptp_host_inventory",
        "host_boot_id": boot_id,
        "host_boot_id_error": boot_id_error,
        "interface": config.interface,
        "expected_ptp_domain_number": config.domain_number,
        "pmc_socket": config.pmc_socket,
        "collection_monotonic_start_ns": collection_start,
        "collection_realtime_start_ns": realtime_start,
        "collection_monotonic_end_ns": collection_end,
        "collection_realtime_end_ns": realtime_end,
        "ptp_device_paths": ptp_device_paths,
        "interface_sysfs_ptp_paths": [os.path.realpath(path) for path in sysfs_ptp_paths],
        "commands": results + [pmc_result],
        "pmc_snapshot_validation": pmc_validation,
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
    parser.add_argument("--timeout", type=float, default=3.0)
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
