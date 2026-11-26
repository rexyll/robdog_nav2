"""Strict adapter for host UTC synchronization evidence ROS messages."""

from dataclasses import dataclass

from robodog_gnss_interfaces.msg import HostTimeSyncStatus


UINT64_MAX = (1 << 64) - 1
INT64_MIN = -(1 << 63)
INT64_MAX = (1 << 63) - 1


@dataclass(frozen=True)
class HostTimeSyncSample:
    state: str
    source_id: str
    host_boot_id: str
    utc_offset_ns: int
    uncertainty_ns: int
    sample_monotonic_ns: int
    sample_sequence: int
    reference_id: str
    stratum: int
    leap_status: str

    def as_policy_kwargs(self):
        return {
            "host_sync_state": self.state,
            "host_sync_source": self.source_id,
            "host_utc_offset_ns": self.utc_offset_ns,
            "host_utc_uncertainty_ns": self.uncertainty_ns,
            "host_sync_sample_monotonic_ns": self.sample_monotonic_ns,
        }


class HostTimeSyncStatusAdapter:
    """Reject replayed, reordered, cross-boot or malformed clock evidence."""

    _STATE_MAP = {
        HostTimeSyncStatus.STATE_UNKNOWN: "unknown",
        HostTimeSyncStatus.STATE_SYNCHRONIZED: "synchronized",
        HostTimeSyncStatus.STATE_UNSYNCHRONIZED: "unsynchronized",
        HostTimeSyncStatus.STATE_FAULT: "fault",
    }

    def __init__(self, *, expected_host_boot_id):
        if not isinstance(expected_host_boot_id, str) or not expected_host_boot_id.strip():
            raise ValueError("expected_host_boot_id must be a non-empty string")
        self.expected_host_boot_id = expected_host_boot_id
        self._last_sequence = None
        self._last_sample_monotonic_ns = None

    @staticmethod
    def _uint(name, value):
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= UINT64_MAX:
            raise ValueError(f"{name} must be uint64")
        return value

    def decode(self, message):
        if not isinstance(message, HostTimeSyncStatus):
            raise TypeError("message must be HostTimeSyncStatus")
        if not isinstance(message.source_id, str) or not message.source_id.strip():
            raise ValueError("source_id must be a non-empty string")
        if message.host_boot_id != self.expected_host_boot_id:
            raise ValueError("host_boot_id does not match the active host boot")
        if isinstance(message.state, bool) or message.state not in self._STATE_MAP:
            raise ValueError("unknown host synchronization state")
        if (
            isinstance(message.utc_offset_ns, bool)
            or not isinstance(message.utc_offset_ns, int)
            or not INT64_MIN <= message.utc_offset_ns <= INT64_MAX
        ):
            raise ValueError("utc_offset_ns must be int64")
        uncertainty = self._uint("uncertainty_ns", message.uncertainty_ns)
        sample_ns = self._uint("sample_monotonic_ns", message.sample_monotonic_ns)
        sequence = self._uint("sample_sequence", message.sample_sequence)
        if self._last_sequence is not None and sequence <= self._last_sequence:
            raise ValueError("sample_sequence must strictly increase")
        if (
            self._last_sample_monotonic_ns is not None
            and sample_ns <= self._last_sample_monotonic_ns
        ):
            raise ValueError("sample_monotonic_ns must strictly increase")
        if not isinstance(message.reference_id, str) or not message.reference_id.strip():
            raise ValueError("reference_id must be a non-empty string")
        if not isinstance(message.leap_status, str) or not message.leap_status.strip():
            raise ValueError("leap_status must be a non-empty string")
        if isinstance(message.stratum, bool) or not isinstance(message.stratum, int) or not 0 <= message.stratum <= 255:
            raise ValueError("stratum must be uint8")
        if not isinstance(message.leap_status, str) or not message.leap_status.strip():
            raise ValueError("leap_status must be a non-empty string")
        sample = HostTimeSyncSample(
            state=self._STATE_MAP[message.state],
            source_id=message.source_id,
            host_boot_id=message.host_boot_id,
            utc_offset_ns=message.utc_offset_ns,
            uncertainty_ns=uncertainty,
            sample_monotonic_ns=sample_ns,
            sample_sequence=sequence,
            reference_id=message.reference_id,
            stratum=message.stratum,
            leap_status=message.leap_status,
        )
        self._last_sequence = sequence
        self._last_sample_monotonic_ns = sample_ns
        return sample
