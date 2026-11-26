"""Read-only Chrony tracking evidence collector.

The collector never changes clock state. It treats missing, malformed, unsynchronised,
or over-budget Chrony evidence as non-synchronised. Thresholds are mandatory deployment
inputs and must come from the system timing/integrity budget.
"""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
import re
import subprocess
import time
from pathlib import Path

_NSEC = Decimal(1_000_000_000)
_REFERENCE_UNKNOWN = {"00000000", "0000000000000000", "0.0.0.0", "::"}


def _seconds_to_ns(value: str) -> int:
    try:
        scaled = Decimal(value) * _NSEC
    except InvalidOperation as exc:
        raise ValueError("invalid Chrony seconds value") from exc
    if not scaled.is_finite() or scaled < 0:
        raise ValueError("Chrony seconds value must be finite and non-negative")
    integral = scaled.to_integral_value()
    if scaled != integral:
        raise ValueError("Chrony value exceeds nanosecond precision")
    result = int(integral)
    if result > (1 << 64) - 1:
        raise ValueError("Chrony nanosecond value exceeds uint64")
    return result


def _field(lines, label):
    matches = [line.split(":", 1)[1].strip() for line in lines
               if line.partition(":")[0].strip() == label]
    if len(matches) != 1 or not matches[0]:
        raise ValueError(f"Chrony tracking must contain exactly one {label!r} field")
    return matches[0]


def parse_chrony_tracking(output: str):
    """Parse chronyc -n tracking output without locale-dependent guessing."""
    if not isinstance(output, str) or not output.strip():
        raise ValueError("Chrony tracking output is empty")
    lines = output.splitlines()
    reference_id = _field(lines, "Reference ID")
    stratum_text = _field(lines, "Stratum")
    system_time = _field(lines, "System time")
    root_dispersion = _field(lines, "Root dispersion")
    leap_status = _field(lines, "Leap status")
    try:
        stratum = int(stratum_text, 10)
    except ValueError as exc:
        raise ValueError("Chrony Stratum must be an integer") from exc
    if not 0 <= stratum <= 255:
        raise ValueError("Chrony Stratum outside uint8 range")
    match = re.fullmatch(r"([+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)) seconds (fast|slow) of NTP time", system_time)
    if not match:
        raise ValueError("unsupported Chrony System time format")
    offset_magnitude = _seconds_to_ns(match.group(1).lstrip("+").lstrip("-"))
    offset_ns = offset_magnitude if match.group(2) == "fast" else -offset_magnitude
    dispersion_match = re.fullmatch(r"([+]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)) seconds", root_dispersion)
    if not dispersion_match:
        raise ValueError("unsupported Chrony Root dispersion format")
    uncertainty_ns = _seconds_to_ns(dispersion_match.group(1).lstrip("+"))
    return {
        "reference_id": reference_id,
        "stratum": stratum,
        "utc_offset_ns": offset_ns,
        "uncertainty_ns": uncertainty_ns,
        "leap_status": leap_status,
    }


@dataclass(frozen=True)
class ChronyMonitorConfig:
    source_id: str
    max_host_utc_error_ns: int
    command_timeout_s: float = 1.0

    def __post_init__(self):
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must be a non-empty configured identifier")
        if isinstance(self.max_host_utc_error_ns, bool) or not isinstance(self.max_host_utc_error_ns, int) or self.max_host_utc_error_ns <= 0:
            raise ValueError("max_host_utc_error_ns must be a positive integer")
        if isinstance(self.command_timeout_s, bool) or not isinstance(self.command_timeout_s, (int, float)) or self.command_timeout_s <= 0:
            raise ValueError("command_timeout_s must be positive")


class ChronyHostTimeMonitor:
    """Collect one status sample using chronyc; boot and monotonic clocks are local."""

    def __init__(self, config: ChronyMonitorConfig, *, boot_id_path="/proc/sys/kernel/random/boot_id",
                 runner=subprocess.run, monotonic_ns=time.monotonic_ns):
        self.config = config
        self._boot_id_path = Path(boot_id_path)
        self._runner = runner
        self._monotonic_ns = monotonic_ns
        self._sequence = 0

    def sample(self):
        try:
            boot_id = self._boot_id_path.read_text(encoding="ascii").strip()
            if not boot_id:
                raise ValueError("Linux boot ID is empty")
            completed = self._runner(
                ["chronyc", "-n", "tracking"], capture_output=True, text=True,
                timeout=self.config.command_timeout_s, check=False,
            )
            if completed.returncode != 0:
                raise RuntimeError(f"chronyc exited with status {completed.returncode}")
            evidence = parse_chrony_tracking(completed.stdout)
            reference = evidence["reference_id"].strip()
            healthy = (
                evidence["leap_status"].lower() == "normal"
                and 1 <= evidence["stratum"] <= 15
                and reference not in _REFERENCE_UNKNOWN
                and abs(evidence["utc_offset_ns"]) + evidence["uncertainty_ns"]
                    <= self.config.max_host_utc_error_ns
            )
            state = "synchronized" if healthy else "unsynchronized"
            fault = ""
        except Exception as exc:
            evidence = {
                "reference_id": "unavailable", "stratum": 0,
                "utc_offset_ns": 0, "uncertainty_ns": 0,
                "leap_status": "unknown",
            }
            state = "fault"
            fault = f"{type(exc).__name__}: {exc}"
        self._sequence += 1
        return {
            "state": state,
            "source_id": self.config.source_id,
            "host_boot_id": boot_id,
            **evidence,
            "sample_monotonic_ns": self._monotonic_ns(),
            "sample_sequence": self._sequence,
            "fault": fault,
        }
