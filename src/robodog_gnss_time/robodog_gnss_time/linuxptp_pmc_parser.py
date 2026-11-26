"""Strict read-only parser for selected LinuxPTP pmc GET responses.

This module parses PTP clock/GM evidence only. It deliberately does not convert
PTP master offset into host SystemTime-to-UTC offset or publish synchronized
HostTimeSyncStatus.
"""
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from types import MappingProxyType
from typing import Mapping
import re


_HEADER = re.compile(
    r"^\s*(?P<clock_identity>[0-9A-Fa-f:.]+)-(?P<port_number>[0-9]+)\s+"
    r"seq\s+(?P<sequence_id>[0-9]+)\s+RESPONSE MANAGEMENT "
    r"(?P<dataset>[A-Z0-9_]+)\s*$"
)
_DATASET_FIELDS = {
    "TIME_STATUS_NP": {
        "master_offset", "ingress_time", "cumulativeScaledRateOffset",
        "scaledLastGmPhaseChange", "gmTimeBaseIndicator",
        "lastGmPhaseChange", "gmPresent", "gmIdentity",
    },
    "CURRENT_DATA_SET": {"stepsRemoved", "offsetFromMaster", "meanPathDelay"},
    "PORT_DATA_SET": {
        "portIdentity", "portState", "logMinDelayReqInterval",
        "peerMeanPathDelay", "logAnnounceInterval", "announceReceiptTimeout",
        "logSyncInterval", "delayMechanism", "logMinPdelayReqInterval",
        "versionNumber",
    },
    "TIME_PROPERTIES_DATA_SET": {
        "currentUtcOffset", "leap61", "leap59", "currentUtcOffsetValid",
        "ptpTimescale", "timeTraceable", "frequencyTraceable", "timeSource",
    },
}
_SNAPSHOT_DATASETS = (
    "TIME_STATUS_NP",
    "CURRENT_DATA_SET",
    "PORT_DATA_SET",
    "TIME_PROPERTIES_DATA_SET",
)
_BOOL_FIELDS = {
    "gmPresent", "leap61", "leap59", "currentUtcOffsetValid",
    "ptpTimescale", "timeTraceable", "frequencyTraceable",
}
_INT_FIELDS = {
    "master_offset", "ingress_time", "gmTimeBaseIndicator", "stepsRemoved",
    "currentUtcOffset", "leap61", "leap59", "currentUtcOffsetValid",
    "ptpTimescale", "timeTraceable", "frequencyTraceable",
    "announceReceiptTimeout", "versionNumber",
}
_DECIMAL_FIELDS = {"offsetFromMaster", "meanPathDelay", "peerMeanPathDelay"}


@dataclass(frozen=True)
class PmcResponse:
    """One pmc RESPONSE block, retaining responder and sequence identity."""

    dataset: str
    clock_identity: str
    port_number: int
    sequence_id: int
    values: Mapping[str, object]


@dataclass(frozen=True)
class PmcSnapshot:
    """Four-dataset observation from one pmc command output.

    The snapshot is not assigned a sample timestamp and does not establish
    that the responses were simultaneous or fresh. The collector must record
    a host monotonic interval around the command and apply a freshness policy.
    """

    clock_identity: str
    responses: Mapping[str, PmcResponse]


def _parse_values(lines, dataset):
    values = {}
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("sending:"):
            continue
        parts = stripped.split(None, 1)
        if len(parts) != 2:
            raise ValueError(f"malformed pmc dataset line: {stripped!r}")
        key, value = parts
        if key not in _DATASET_FIELDS[dataset]:
            raise ValueError(f"unknown {dataset} field: {key}")
        if key in values:
            raise ValueError(f"duplicate {dataset} field: {key}")
        values[key] = value.strip()
    required = {
        "TIME_STATUS_NP": {"master_offset", "gmPresent", "gmIdentity"},
        "CURRENT_DATA_SET": {"stepsRemoved", "offsetFromMaster", "meanPathDelay"},
        "PORT_DATA_SET": {"portState"},
        "TIME_PROPERTIES_DATA_SET": {
            "currentUtcOffset", "leap61", "leap59", "currentUtcOffsetValid",
            "ptpTimescale", "timeTraceable", "frequencyTraceable", "timeSource",
        },
    }[dataset]
    missing = required - values.keys()
    if missing:
        raise ValueError(f"{dataset} missing required fields: {sorted(missing)}")
    parsed = {}
    for key, raw in values.items():
        if key in _BOOL_FIELDS:
            if raw not in {"0", "1", "true", "false"}:
                raise ValueError(f"{dataset}.{key} must be a boolean flag")
            parsed[key] = raw in {"1", "true"}
        elif key in _INT_FIELDS:
            if not re.fullmatch(r"[+-]?\d+", raw):
                raise ValueError(f"{dataset}.{key} must be an integer")
            parsed[key] = int(raw, 10)
        elif key in _DECIMAL_FIELDS:
            try:
                number = Decimal(raw)
            except InvalidOperation as exc:
                raise ValueError(f"{dataset}.{key} must be numeric") from exc
            if not number.is_finite():
                raise ValueError(f"{dataset}.{key} must be finite")
            parsed[key] = number
        else:
            if not raw:
                raise ValueError(f"{dataset}.{key} must be non-empty")
            parsed[key] = raw
    if dataset == "TIME_PROPERTIES_DATA_SET" and parsed["leap61"] and parsed["leap59"]:
        raise ValueError("PTP leap61 and leap59 cannot both be asserted")
    return parsed


def parse_pmc_response_envelope(output: str, dataset: str) -> PmcResponse:
    """Parse one response block and retain responder clock/port/sequence identity."""
    if dataset not in _DATASET_FIELDS:
        raise ValueError(f"unsupported LinuxPTP dataset: {dataset}")
    if not isinstance(output, str) or not output.strip():
        raise ValueError("pmc output is empty")
    lines = output.splitlines()
    headers = []
    for index, line in enumerate(lines):
        match = _HEADER.fullmatch(line)
        if match:
            headers.append((index, match))
    matching = [(index, match) for index, match in headers
                if match.group("dataset") == dataset]
    if len(matching) != 1:
        raise ValueError(f"expected exactly one RESPONSE MANAGEMENT {dataset} header")
    index, match = matching[0]
    clock_identity = match.group("clock_identity")
    port_number = int(match.group("port_number"), 10)
    sequence_id = int(match.group("sequence_id"), 10)
    if port_number > 65535 or sequence_id > 65535:
        raise ValueError("pmc response port number and sequence ID must fit uint16")
    end = next((header_index for header_index, _ in headers if header_index > index), len(lines))
    values = _parse_values(lines[index + 1:end], dataset)
    if dataset == "PORT_DATA_SET":
        expected_port_identity = f"{clock_identity}-{port_number}"
        if values["portIdentity"].lower() != expected_port_identity.lower():
            raise ValueError("PORT_DATA_SET portIdentity does not match response identity")
    return PmcResponse(
        dataset=dataset,
        clock_identity=clock_identity,
        port_number=port_number,
        sequence_id=sequence_id,
        values=MappingProxyType(values),
    )


def parse_pmc_response(output: str, dataset: str):
    """Backward-compatible dataset-only parser; response identity is discarded."""
    return dict(parse_pmc_response_envelope(output, dataset).values)


def parse_pmc_snapshot(output: str) -> PmcSnapshot:
    """Parse exactly one response for each supported dataset from one pmc output.

    All four responses must come from the same PTP clock identity. Port numbers
    and sequence IDs are preserved per response rather than assumed equal.
    """
    if not isinstance(output, str) or not output.strip():
        raise ValueError("pmc output is empty")
    headers = [match for line in output.splitlines()
               if (match := _HEADER.fullmatch(line))]
    actual_datasets = [match.group("dataset") for match in headers]
    if len(actual_datasets) != len(_SNAPSHOT_DATASETS):
        raise ValueError("pmc snapshot must contain exactly four response blocks")
    if set(actual_datasets) != set(_SNAPSHOT_DATASETS):
        raise ValueError("pmc snapshot contains missing, duplicate, or unsupported datasets")
    responses = {
        dataset: parse_pmc_response_envelope(output, dataset)
        for dataset in _SNAPSHOT_DATASETS
    }
    clock_identities = {response.clock_identity.lower() for response in responses.values()}
    if len(clock_identities) != 1:
        raise ValueError("pmc snapshot responses came from different PTP clock identities")
    sequence_ids = [response.sequence_id for response in responses.values()]
    if len(set(sequence_ids)) != len(sequence_ids):
        raise ValueError("pmc snapshot response sequence IDs are not unique")
    clock_identity = next(iter(responses.values())).clock_identity
    return PmcSnapshot(
        clock_identity=clock_identity,
        responses=MappingProxyType(responses),
    )
