#!/usr/bin/env python3
"""Audit an M3DGR-style TUM ground-truth trajectory without modifying it.

Expected row layout: timestamp tx ty tz qx qy qz qw.
The tool validates syntax, finite values, timestamp ordering and quaternion
normalization. It does not establish the dataset identity, coordinate frame,
clock alignment, or correspondence to a particular rosbag.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def audit_tum_trajectory(
    path: Path,
    *,
    quaternion_norm_tolerance: float = 1e-3,
) -> dict[str, Any]:
    """Parse and audit a whitespace-delimited TUM pose trajectory."""
    path = path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"trajectory file not found: {path}")
    if not math.isfinite(quaternion_norm_tolerance) or quaternion_norm_tolerance <= 0:
        raise ValueError("quaternion_norm_tolerance must be finite and greater than zero")

    issues: list[str] = []
    malformed_rows: list[dict[str, Any]] = []
    non_finite_rows: list[dict[str, Any]] = []
    quaternion_issues: list[dict[str, Any]] = []
    timestamps: list[float] = []
    positions: list[tuple[float, float, float]] = []
    quaternion_norms: list[float] = []
    duplicate_timestamps = 0
    backwards_timestamps = 0
    data_rows = 0
    comment_rows = 0
    previous_timestamp: float | None = None
    previous_pose: tuple[float, ...] | None = None
    identical_pose_adjacent_pairs = 0
    duplicate_timestamp_same_pose_pairs = 0
    backwards_timestamp_same_pose_pairs = 0
    timestamp_anomaly_pose_change_pairs = 0

    try:
        text = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError(f"trajectory is not valid UTF-8 text: {exc}") from exc

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            comment_rows += 1
            continue

        data_rows += 1
        fields = stripped.split()
        if len(fields) != 8:
            malformed_rows.append(
                {
                    "line": line_number,
                    "field_count": len(fields),
                    "reason": "expected exactly 8 TUM fields",
                }
            )
            continue

        try:
            values = [float(field) for field in fields]
        except ValueError:
            malformed_rows.append(
                {"line": line_number, "field_count": 8, "reason": "non-numeric field"}
            )
            continue

        if not all(math.isfinite(value) for value in values):
            non_finite_rows.append({"line": line_number})
            continue

        timestamp = values[0]
        position = (values[1], values[2], values[3])
        quaternion = values[4:8]
        norm = math.sqrt(sum(value * value for value in quaternion))

        pose_signature = position + tuple(quaternion)
        if previous_timestamp is not None:
            same_pose = pose_signature == previous_pose
            if same_pose:
                identical_pose_adjacent_pairs += 1
            if timestamp == previous_timestamp:
                duplicate_timestamps += 1
                if same_pose:
                    duplicate_timestamp_same_pose_pairs += 1
                else:
                    timestamp_anomaly_pose_change_pairs += 1
            elif timestamp < previous_timestamp:
                backwards_timestamps += 1
                if same_pose:
                    backwards_timestamp_same_pose_pairs += 1
                else:
                    timestamp_anomaly_pose_change_pairs += 1
        previous_timestamp = timestamp
        previous_pose = pose_signature

        timestamps.append(timestamp)
        positions.append(position)
        quaternion_norms.append(norm)

        if norm <= 1e-12 or abs(norm - 1.0) > quaternion_norm_tolerance:
            quaternion_issues.append(
                {
                    "line": line_number,
                    "norm": norm,
                    "deviation": abs(norm - 1.0),
                }
            )

    if malformed_rows:
        issues.append(f"{len(malformed_rows)} malformed TUM row(s)")
    if non_finite_rows:
        issues.append(f"{len(non_finite_rows)} row(s) contain NaN or infinity")
    if duplicate_timestamps:
        issues.append(f"{duplicate_timestamps} duplicate timestamp(s)")
    if backwards_timestamps:
        issues.append(f"{backwards_timestamps} backwards timestamp(s)")
    if quaternion_issues:
        issues.append(
            f"{len(quaternion_issues)} quaternion(s) are zero or outside norm tolerance "
            f"{quaternion_norm_tolerance}"
        )
    if len(timestamps) < 2:
        issues.append("fewer than two valid pose rows")

    chronological = duplicate_timestamps == 0 and backwards_timestamps == 0
    start_time = min(timestamps) if timestamps else None
    end_time = max(timestamps) if timestamps else None
    ordered_duration = (
        timestamps[-1] - timestamps[0]
        if len(timestamps) >= 2 and chronological
        else None
    )
    average_rate = (
        (len(timestamps) - 1) / ordered_duration
        if ordered_duration is not None and ordered_duration > 0
        else None
    )

    position_bounds: dict[str, list[float]] | None = None
    if positions:
        position_bounds = {
            axis: [
                min(position[index] for position in positions),
                max(position[index] for position in positions),
            ]
            for index, axis in enumerate(("x", "y", "z"))
        }

    path_length = 0.0
    for first, second in zip(positions, positions[1:]):
        path_length += math.sqrt(sum((b - a) ** 2 for a, b in zip(first, second)))

    report = {
        "schema_version": 1,
        "trajectory_format": "TUM",
        "input_file": str(path),
        "file_size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
        "data_rows": data_rows,
        "comment_rows": comment_rows,
        "valid_pose_rows": len(timestamps),
        "malformed_rows": malformed_rows[:100],
        "malformed_row_count": len(malformed_rows),
        "non_finite_rows": non_finite_rows[:100],
        "non_finite_row_count": len(non_finite_rows),
        "timestamp": {
            "first_in_file": timestamps[0] if timestamps else None,
            "last_in_file": timestamps[-1] if timestamps else None,
            "minimum": start_time,
            "maximum": end_time,
            "ordered_duration_sec": ordered_duration,
            "average_rate_hz": average_rate,
            "duplicate_count": duplicate_timestamps,
            "backwards_count": backwards_timestamps,
            "strictly_increasing": chronological and len(timestamps) >= 2,
        },
        "position_bounds": position_bounds,
        "pose_repetition": {
            "adjacent_identical_pose_pairs": identical_pose_adjacent_pairs,
            "duplicate_timestamp_same_pose_pairs": duplicate_timestamp_same_pose_pairs,
            "backwards_timestamp_same_pose_pairs": backwards_timestamp_same_pose_pairs,
            "timestamp_anomaly_pose_change_pairs": timestamp_anomaly_pose_change_pairs,
            "pose_fields": "tx ty tz qx qy qz qw; exact numeric equality",
        },
        "path_length_in_file_order_m": path_length if positions else None,
        "quaternion_norm": {
            "minimum": min(quaternion_norms) if quaternion_norms else None,
            "maximum": max(quaternion_norms) if quaternion_norms else None,
            "tolerance": quaternion_norm_tolerance,
            "invalid_count": len(quaternion_issues),
            "invalid_samples": quaternion_issues[:100],
            "field_order": "qx qy qz qw",
        },
        "issues": issues,
        "status": "valid_tum_trajectory" if not issues else "blocked_review_required",
        "limitations": [
            "Eight numeric fields validate TUM syntax only; they do not prove this is Dynamic03 GT.",
            "Timestamp units are treated as seconds according to TUM convention, but epoch/clock "
            "origin and alignment with rosbag time are not inferred.",
            "Coordinate frame, handedness, geodetic datum and transform convention are not inferred.",
            "A structurally valid trajectory is not proof of ground-truth accuracy.",
        ],
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only structural audit of a TUM-format ground-truth trajectory."
    )
    parser.add_argument("trajectory", type=Path, help="TUM trajectory text file")
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Optional machine-readable JSON report path",
    )
    parser.add_argument(
        "--quaternion-norm-tolerance",
        type=float,
        default=1e-3,
        help="Maximum absolute deviation of quaternion norm from 1 (default: 1e-3)",
    )
    args = parser.parse_args()

    try:
        report = audit_tum_trajectory(
            args.trajectory,
            quaternion_norm_tolerance=args.quaternion_norm_tolerance,
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output + "\n", encoding="utf-8")

    return 0 if report["status"] == "valid_tum_trajectory" else 10


if __name__ == "__main__":
    raise SystemExit(main())
