"""Strict read-only parser for selected LinuxPTP pmc GET responses.

This module parses PTP clock/GM evidence only. It deliberately does not convert
PTP master offset into host SystemTime-to-UTC offset or publish synchronized
HostTimeSyncStatus.
"""
from decimal import Decimal, InvalidOperation
import re


_HEADER = re.compile(
    r"^\\s*[0-9A-Fa-f:.]+-[0-9]+\\s+seq\\s+[0-9]+\\s+"
    r"RESPONSE MANAGEMENT ([A-Z0-9_]+)\\s*$"
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


def parse_pmc_response(output: str, dataset: str):
    """Parse one exact RESPONSE MANAGEMENT dataset block from pmc stdout."""
    if dataset not in _DATASET_FIELDS:
        raise ValueError(f"unsupported LinuxPTP dataset: {dataset}")
    if not isinstance(output, str) or not output.strip():
        raise ValueError("pmc output is empty")
    lines = output.splitlines()
    matching_headers = []
    for index, line in enumerate(lines):
        match = _HEADER.fullmatch(line)
        if match and match.group(1) == dataset:
            matching_headers.append(index)
    if len(matching_headers) != 1:
        raise ValueError(f"expected exactly one RESPONSE MANAGEMENT {dataset} header")
    start = matching_headers[0] + 1
    values = {}
    for line in lines[start:]:
        if _HEADER.fullmatch(line):
            break
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
            if not re.fullmatch(r"[+-]?\\d+", raw):
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
