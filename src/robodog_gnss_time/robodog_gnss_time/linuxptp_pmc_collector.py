"""Read-only LinuxPTP pmc evidence collection; never qualifies host UTC.

The collector invokes a fixed set of GET management requests with explicit
transport/domain/profile parameters. It captures raw evidence and host monotonic
bracketing but deliberately does not parse, infer freshness, or alter clocks.
"""
from dataclasses import dataclass
from pathlib import Path
import subprocess
import time
from typing import Callable


_DATASETS = (
    "TIME_STATUS_NP",
    "CURRENT_DATA_SET",
    "PORT_DATA_SET",
    "PARENT_DATA_SET",
    "TIME_PROPERTIES_DATA_SET",
    "DEFAULT_DATA_SET",
)
_TRANSPORT_FLAGS = {
    "uds": "-u",
    "udp4": "-4",
    "udp6": "-6",
    "l2": "-2",
}


def _decode_output(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


@dataclass(frozen=True)
class PmcCollectorConfig:
    """All deployment-specific pmc settings are mandatory and explicit."""

    executable: str
    transport: str
    domain_number: int
    boundary_hops: int
    transport_specific: int
    socket_path: str = ""
    interface: str = ""
    command_timeout_s: float = 2.0

    def __post_init__(self):
        if not isinstance(self.executable, str) or not Path(self.executable).is_absolute():
            raise ValueError("pmc executable must be an absolute path")
        if self.transport not in _TRANSPORT_FLAGS:
            raise ValueError("transport must be one of uds, udp4, udp6, or l2")
        for name, value in (
            ("domain_number", self.domain_number),
            ("boundary_hops", self.boundary_hops),
            ("transport_specific", self.transport_specific),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{name} must be an integer")
            if not 0 <= value <= 255:
                raise ValueError(f"{name} must fit uint8")
        if self.transport == "uds":
            if not isinstance(self.socket_path, str) or not Path(self.socket_path).is_absolute():
                raise ValueError("UDS transport requires an absolute server socket_path")
            if self.interface:
                raise ValueError("interface must be empty for UDS transport")
        else:
            if not isinstance(self.interface, str) or not self.interface.strip():
                raise ValueError("network transport requires an explicit interface")
            if self.socket_path:
                raise ValueError("socket_path is only valid for UDS transport")
        if (isinstance(self.command_timeout_s, bool)
                or not isinstance(self.command_timeout_s, (int, float))
                or self.command_timeout_s <= 0):
            raise ValueError("command_timeout_s must be positive")


@dataclass(frozen=True)
class PmcCollection:
    """Raw collection result; returncode None means the process did not complete."""

    argv: tuple[str, ...]
    host_boot_id: str
    monotonic_start_ns: int
    monotonic_end_ns: int
    returncode: int | None
    stdout: str
    stderr: str
    timed_out: bool
    fault: str

    @property
    def succeeded(self) -> bool:
        return not self.fault and not self.timed_out and self.returncode == 0


class LinuxPtpPmcCollector:
    """Run only fixed pmc GET requests and preserve their raw evidence."""

    def __init__(
        self,
        config: PmcCollectorConfig,
        *,
        boot_id_path="/proc/sys/kernel/random/boot_id",
        runner: Callable = subprocess.run,
        monotonic_ns: Callable = time.monotonic_ns,
    ):
        self.config = config
        self._boot_id_path = Path(boot_id_path)
        self._runner = runner
        self._monotonic_ns = monotonic_ns

    def build_argv(self) -> tuple[str, ...]:
        cfg = self.config
        args = [
            cfg.executable,
            "-d", str(cfg.domain_number),
            "-b", str(cfg.boundary_hops),
            "-t", hex(cfg.transport_specific),
            _TRANSPORT_FLAGS[cfg.transport],
        ]
        if cfg.transport == "uds":
            args.extend(["-s", cfg.socket_path])
        else:
            args.extend(["-i", cfg.interface])
        # Keep each complete pmc command as one argv item. No shell, stdin,
        # user-provided command, config file, SET, CMD, TARGET or clock action.
        args.extend(f"GET {dataset}" for dataset in _DATASETS)
        return tuple(args)

    def collect(self) -> PmcCollection:
        argv = self.build_argv()
        start_ns = self._monotonic_ns()
        boot_id = ""
        returncode = None
        stdout = ""
        stderr = ""
        timed_out = False
        fault = ""
        try:
            boot_id = self._boot_id_path.read_text(encoding="ascii").strip()
            if not boot_id:
                raise ValueError("Linux boot ID is empty")
            completed = self._runner(
                list(argv),
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=self.config.command_timeout_s,
                check=False,
                shell=False,
            )
            returncode = completed.returncode
            stdout = _decode_output(completed.stdout)
            stderr = _decode_output(completed.stderr)
            if returncode != 0:
                fault = f"pmc exited with status {returncode}"
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            stdout = _decode_output(exc.stdout)
            stderr = _decode_output(exc.stderr)
            fault = "pmc command timed out"
        except Exception as exc:
            fault = f"{type(exc).__name__}: {exc}"
        end_ns = self._monotonic_ns()
        if end_ns < start_ns:
            fault = fault or "CLOCK_MONOTONIC moved backwards during collection"
        return PmcCollection(
            argv=argv,
            host_boot_id=boot_id,
            monotonic_start_ns=start_ns,
            monotonic_end_ns=end_ns,
            returncode=returncode,
            stdout=stdout,
            stderr=stderr,
            timed_out=timed_out,
            fault=fault,
        )
