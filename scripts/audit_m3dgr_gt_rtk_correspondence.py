#!/usr/bin/env python3
"""Audit whether Dynamic03 TUM GT corresponds to the recorded RTK NavSatFix stream.

This is a read-only source-association diagnostic. It preserves TUM file order,
does not sort/deduplicate, and never transforms or rewrites either input.
A low rigid-fit residual is only correspondence evidence, not GT certification.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_RTK_TOPIC = "/mavros/global_position/raw/fix"
WGS84_A = 6_378_137.0
WGS84_E2 = 6.69437999014e-3


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stamp_to_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsec", None))
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def parse_tum_rows(path: Path) -> tuple[list[float], np.ndarray]:
    timestamps: list[float] = []
    positions: list[tuple[float, float, float]] = []
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except UnicodeDecodeError as exc:
        raise ValueError(f"TUM file is not valid UTF-8: {exc}") from exc
    for line_number, raw in enumerate(lines, start=1):
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) != 8:
            raise ValueError(
                f"TUM line {line_number}: expected 8 fields, got {len(fields)}"
            )
        try:
            values = [float(field) for field in fields]
        except ValueError as exc:
            raise ValueError(f"TUM line {line_number}: non-numeric field") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"TUM line {line_number}: non-finite value")
        q_norm = math.sqrt(sum(value * value for value in values[4:8]))
        if q_norm <= 1e-12 or abs(q_norm - 1.0) > 1e-3:
            raise ValueError(f"TUM line {line_number}: invalid quaternion norm {q_norm}")
        timestamps.append(values[0])
        positions.append((values[1], values[2], values[3]))
    if len(timestamps) < 2:
        raise ValueError("TUM trajectory must contain at least two valid rows")
    return timestamps, np.asarray(positions, dtype=np.float64)


def timestamp_order_stats(values_ns: list[int]) -> dict[str, Any]:
    duplicates = 0
    backwards = 0
    positive_gaps: list[float] = []
    for previous, current in zip(values_ns, values_ns[1:]):
        delta = current - previous
        if delta == 0:
            duplicates += 1
        elif delta < 0:
            backwards += 1
        else:
            positive_gaps.append(delta / 1e9)
    positive_gaps.sort()

    def percentile(fraction: float) -> float | None:
        if not positive_gaps:
            return None
        index = min(len(positive_gaps) - 1, int((len(positive_gaps) - 1) * fraction))
        return positive_gaps[index]

    return {
        "count": len(values_ns),
        "first_ns": values_ns[0] if values_ns else None,
        "last_ns": values_ns[-1] if values_ns else None,
        "minimum_ns": min(values_ns) if values_ns else None,
        "maximum_ns": max(values_ns) if values_ns else None,
        "duplicate_count": duplicates,
        "backwards_count": backwards,
        "strictly_increasing": len(values_ns) >= 2 and duplicates == 0 and backwards == 0,
        "positive_gap_sec": {
            "minimum": min(positive_gaps) if positive_gaps else None,
            "p50": percentile(0.50),
            "p95": percentile(0.95),
            "maximum": max(positive_gaps) if positive_gaps else None,
        },
    }


def summarize_deltas(delta_ns: list[int]) -> dict[str, Any]:
    if not delta_ns:
        return {"count": 0}
    values_ms = np.asarray(delta_ns, dtype=np.float64) / 1e6
    absolute_ms = np.abs(values_ms)
    return {
        "count": len(delta_ns),
        "signed_ms": {
            "minimum": float(np.min(values_ms)),
            "p05": float(np.percentile(values_ms, 5)),
            "median": float(np.median(values_ms)),
            "p95": float(np.percentile(values_ms, 95)),
            "maximum": float(np.max(values_ms)),
        },
        "absolute_ms": {
            "median": float(np.median(absolute_ms)),
            "p95": float(np.percentile(absolute_ms, 95)),
            "maximum": float(np.max(absolute_ms)),
        },
        "within_abs_1ms": int(np.count_nonzero(absolute_ms <= 1.0)),
        "within_abs_5ms": int(np.count_nonzero(absolute_ms <= 5.0)),
        "within_abs_20ms": int(np.count_nonzero(absolute_ms <= 20.0)),
        "within_abs_100ms": int(np.count_nonzero(absolute_ms <= 100.0)),
    }


def geodetic_to_enu(
    latitude_deg: list[float],
    longitude_deg: list[float],
    altitude_m: list[float],
) -> np.ndarray:
    """Convert WGS84 geodetic samples to a local ENU frame at sample zero."""
    lat = np.radians(np.asarray(latitude_deg, dtype=np.float64))
    lon = np.radians(np.asarray(longitude_deg, dtype=np.float64))
    alt = np.asarray(altitude_m, dtype=np.float64)
    if lat.size < 2 or lon.size != lat.size or alt.size != lat.size:
        raise ValueError("latitude/longitude/altitude arrays must have equal length >= 2")
    if not (np.all(np.isfinite(lat)) and np.all(np.isfinite(lon)) and np.all(np.isfinite(alt))):
        raise ValueError("GNSS coordinates contain non-finite values")
    if np.any(np.abs(lat) > np.pi / 2) or np.any(np.abs(lon) > np.pi):
        raise ValueError("GNSS latitude/longitude outside valid WGS84 range")

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    prime_vertical = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (prime_vertical + alt) * cos_lat * np.cos(lon)
    y = (prime_vertical + alt) * cos_lat * np.sin(lon)
    z = (prime_vertical * (1.0 - WGS84_E2) + alt) * sin_lat
    ecef = np.column_stack((x, y, z))

    lat0 = lat[0]
    lon0 = lon[0]
    rotation = np.asarray(
        [
            [-np.sin(lon0), np.cos(lon0), 0.0],
            [-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)],
            [np.cos(lat0) * np.cos(lon0), np.cos(lat0) * np.sin(lon0), np.sin(lat0)],
        ],
        dtype=np.float64,
    )
    return (ecef - ecef[0]) @ rotation.T


def rigid_trajectory_correspondence(
    source_enu: np.ndarray,
    tum_positions: np.ndarray,
) -> dict[str, Any]:
    """Fit a single SE(3) transform for source-association diagnosis only."""
    source = np.asarray(source_enu, dtype=np.float64)
    target = np.asarray(tum_positions, dtype=np.float64)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("ENU and TUM positions must have the same Nx3 shape")
    if source.shape[0] < 3 or not np.all(np.isfinite(source)) or not np.all(np.isfinite(target)):
        raise ValueError("at least three finite paired positions are required")

    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean
    covariance = source_centered.T @ target_centered
    u, _, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = 1.0 if np.linalg.det(u @ vt) >= 0 else -1.0
    rotation = u @ correction @ vt
    translation = target_mean - source_mean @ rotation
    predicted = source @ rotation + translation
    residual = np.linalg.norm(predicted - target, axis=1)

    translation_only_residual = np.linalg.norm(
        source_centered - target_centered, axis=1
    )
    return {
        "method": "one global rigid 3D transform fitted only for source-association diagnosis",
        "scale_fitted": False,
        "rotation_determinant": float(np.linalg.det(rotation)),
        "rmse_3d_m": float(np.sqrt(np.mean(residual * residual))),
        "median_3d_m": float(np.median(residual)),
        "p95_3d_m": float(np.percentile(residual, 95)),
        "maximum_3d_m": float(np.max(residual)),
        "translation_only_rmse_3d_m": float(
            np.sqrt(np.mean(translation_only_residual * translation_only_residual))
        ),
        "source_enu_bounds_m": {
            axis: [float(np.min(source[:, index])), float(np.max(source[:, index]))]
            for index, axis in enumerate(("east", "north", "up"))
        },
        "tum_position_bounds_m": {
            axis: [float(np.min(target[:, index])), float(np.max(target[:, index]))]
            for index, axis in enumerate(("x", "y", "z"))
        },
        "warning": (
            "This fitted transform is diagnostic only. It must not be used to tune "
            "trajectory evaluation alignment or claim GT validity."
        ),
    }


def audit_gt_rtk_correspondence(
    bag_path: Path,
    tum_path: Path,
    *,
    rtk_topic: str = DEFAULT_RTK_TOPIC,
) -> dict[str, Any]:
    """Compare original TUM rows with the RTK NavSatFix stream in original bag order."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency 'rosbags'. Install requirements-dataset-tools.txt."
        ) from exc

    bag_path = bag_path.expanduser().resolve()
    tum_path = tum_path.expanduser().resolve()
    if not bag_path.is_file() or bag_path.suffix.lower() != ".bag":
        raise ValueError("source must be an existing original ROS1 .bag file")
    if not tum_path.is_file():
        raise FileNotFoundError(f"TUM file not found: {tum_path}")

    gt_timestamps_sec, gt_positions = parse_tum_rows(tum_path)
    fix_bag_times: list[int] = []
    fix_header_times: list[int] = []
    latitudes: list[float] = []
    longitudes: list[float] = []
    altitudes: list[float] = []
    statuses: list[int] = []
    covariance_types: list[int] = []
    types: set[str] = set()

    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic == rtk_topic]
        if not connections:
            raise ValueError(f"RTK topic not found in source bag: {rtk_topic}")
        types = {str(c.msgtype) for c in connections}
        if types != {"sensor_msgs/msg/NavSatFix"}:
            raise ValueError(
                f"Expected only sensor_msgs/msg/NavSatFix on {rtk_topic}; found {sorted(types)}"
            )

        for connection, bag_timestamp_ns, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            header_ns = stamp_to_ns(message)
            if header_ns is None:
                raise ValueError("RTK NavSatFix message has no valid header timestamp")
            fix_bag_times.append(int(bag_timestamp_ns))
            fix_header_times.append(header_ns)
            latitudes.append(float(message.latitude))
            longitudes.append(float(message.longitude))
            altitudes.append(float(message.altitude))
            statuses.append(int(message.status.status))
            covariance_types.append(int(message.position_covariance_type))

    if len(set(len(value) for value in (
        fix_bag_times, fix_header_times, latitudes, longitudes, altitudes, statuses,
        covariance_types,
    ))) != 1:
        raise RuntimeError("internal error: RTK message columns have different lengths")

    gt_ns = [int(round(value * 1_000_000_000)) for value in gt_timestamps_sec]
    count_equal = len(gt_ns) == len(fix_header_times)
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "association_report_complete",
        "source_bag": str(bag_path),
        "source_bag_size_bytes": bag_path.stat().st_size,
        "source_bag_sha256": sha256_file(bag_path),
        "tum_file": str(tum_path),
        "tum_file_size_bytes": tum_path.stat().st_size,
        "tum_sha256": sha256_file(tum_path),
        "rtk_topic": rtk_topic,
        "rtk_message_types": sorted(types),
        "gt_rows": len(gt_ns),
        "rtk_messages": len(fix_header_times),
        "row_count_equal": count_equal,
        "gt_timestamp_order": timestamp_order_stats(gt_ns),
        "rtk_header_timestamp_order": timestamp_order_stats(fix_header_times),
        "rtk_bag_record_timestamp_order": timestamp_order_stats(fix_bag_times),
        "rtk_status_distribution": {
            str(value): statuses.count(value) for value in sorted(set(statuses))
        },
        "rtk_covariance_type_distribution": {
            str(value): covariance_types.count(value) for value in sorted(set(covariance_types))
        },
        "limitations": [
            "This compares GT and NavSatFix records in their stored sequence order; it does not reorder or deduplicate either stream.",
            "A row-count match or timestamp match is source-association evidence, not proof that the RTK solution is ground truth.",
            "GNSS-to-ENU is computed from the first RTK fix using WGS84 and is only a trajectory-shape comparison.",
            "The fitted rigid transform is diagnostic only and must not be reused as an EVO evaluation alignment.",
            "RTK fix status, covariance, receiver configuration, correction age and antenna reference remain to be interpreted.",
        ],
    }

    if count_equal:
        timestamp_deltas = [
            gt_timestamp - fix_timestamp
            for gt_timestamp, fix_timestamp in zip(gt_ns, fix_header_times)
        ]
        bag_timestamp_deltas = [
            gt_timestamp - bag_timestamp
            for gt_timestamp, bag_timestamp in zip(gt_ns, fix_bag_times)
        ]
        report["ordinal_timestamp_comparison"] = {
            "gt_minus_rtk_header": summarize_deltas(timestamp_deltas),
            "gt_minus_rtk_bag_record": summarize_deltas(bag_timestamp_deltas),
            "interpretation": (
                "The arrays have equal row counts and are compared by file/bag order. "
                "Inspect residual distributions; do not infer identity from count equality alone."
            ),
        }

        try:
            enu = geodetic_to_enu(latitudes, longitudes, altitudes)
            report["ordinal_position_comparison"] = rigid_trajectory_correspondence(
                enu, gt_positions
            )
        except ValueError as exc:
            report["ordinal_position_comparison"] = {
                "status": "unavailable",
                "reason": str(exc),
            }
    else:
        report["ordinal_timestamp_comparison"] = {
            "status": "unavailable",
            "reason": "GT row count differs from RTK message count; no positional pairing performed",
        }
        report["ordinal_position_comparison"] = {
            "status": "unavailable",
            "reason": "GT row count differs from RTK message count; no positional pairing performed",
        }

    report["rtk_header_vs_bag_record_delta"] = summarize_deltas(
        [header - bag for header, bag in zip(fix_header_times, fix_bag_times)]
    )
    report["interpretation_gate"] = {
        "status": "requires_human_review",
        "do_not_claim": [
            "that RTK is the authoritative Dynamic03 GT",
            "that a row-count/timestamp/trajectory-shape match proves centimetre accuracy",
            "that TUM timestamps are aligned closely enough for APE/RPE",
        ],
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only ordinal/time/position correspondence audit for Dynamic03 TUM vs RTK."
    )
    parser.add_argument("source_bag", type=Path, help="Original Dynamic03 ROS1 .bag file")
    parser.add_argument("tum_trajectory", type=Path, help="Original, unmodified Dynamic03 TUM file")
    parser.add_argument("--rtk-topic", default=DEFAULT_RTK_TOPIC, help="RTK NavSatFix topic")
    parser.add_argument("--json-out", type=Path, help="Optional JSON report path")
    args = parser.parse_args()

    try:
        report = audit_gt_rtk_correspondence(
            args.source_bag,
            args.tum_trajectory,
            rtk_topic=args.rtk_topic,
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
