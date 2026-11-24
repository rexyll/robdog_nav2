#!/usr/bin/env python3
"""Create a non-authoritative, non-destructive TUM candidate from repeated pose runs.

Only consecutive rows with exactly equal pose fields (tx..qw) may be reordered.
Rows are sorted by timestamp inside each such run, and exact timestamp+pose
duplicates are collapsed. If adjacent pose runs have overlapping or reversed
time ranges, generation fails closed. The input file is never modified.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from collections import Counter
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_pose_run_candidate(
    source_path: Path,
    output_dir: Path,
    *,
    quaternion_norm_tolerance: float = 1e-3,
) -> dict[str, Any]:
    """Create a candidate TUM file while preserving each unique pose/time pair."""
    source_path = source_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()

    if not source_path.is_file():
        raise FileNotFoundError(f"source trajectory not found: {source_path}")
    if output_dir.exists():
        raise FileExistsError(f"refusing to overwrite existing output directory: {output_dir}")
    if output_dir == source_path or source_path.is_relative_to(output_dir):
        raise ValueError("output directory must not contain or replace the source trajectory")
    if not math.isfinite(quaternion_norm_tolerance) or quaternion_norm_tolerance <= 0:
        raise ValueError("quaternion_norm_tolerance must be finite and greater than zero")

    source_sha_before = sha256_file(source_path)
    try:
        source_text = source_path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError as exc:
        raise ValueError(f"trajectory is not valid UTF-8 text: {exc}") from exc

    comments: list[str] = []
    records: list[dict[str, Any]] = []
    malformed_rows: list[int] = []
    non_finite_rows: list[int] = []
    invalid_quaternion_rows: list[int] = []

    for line_number, raw_line in enumerate(source_text.splitlines(), start=1):
        stripped = raw_line.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            comments.append(stripped)
            continue
        fields = stripped.split()
        if len(fields) != 8:
            malformed_rows.append(line_number)
            continue
        try:
            values = [float(field) for field in fields]
        except ValueError:
            malformed_rows.append(line_number)
            continue
        if not all(math.isfinite(value) for value in values):
            non_finite_rows.append(line_number)
            continue
        q_norm = math.sqrt(sum(value * value for value in values[4:8]))
        if q_norm <= 1e-12 or abs(q_norm - 1.0) > quaternion_norm_tolerance:
            invalid_quaternion_rows.append(line_number)
            continue
        records.append(
            {
                "line_number": line_number,
                "raw": stripped,
                "timestamp": values[0],
                "pose": tuple(values[1:8]),
            }
        )

    if malformed_rows or non_finite_rows or invalid_quaternion_rows:
        raise ValueError(
            "source is not a structurally valid TUM trajectory: "
            f"malformed={len(malformed_rows)}, non_finite={len(non_finite_rows)}, "
            f"invalid_quaternion={len(invalid_quaternion_rows)}"
        )
    if len(records) < 2:
        raise ValueError("fewer than two valid TUM pose rows")

    # Count timestamp anomalies before any transformation, retaining the
    # distinction between anomalies inside a constant-pose run and at a pose edge.
    input_duplicate_timestamps = 0
    input_backwards_timestamps = 0
    duplicate_same_pose = 0
    backwards_same_pose = 0
    timestamp_anomaly_pose_change = 0
    for previous, current in zip(records, records[1:]):
        delta = current["timestamp"] - previous["timestamp"]
        same_pose = current["pose"] == previous["pose"]
        if delta == 0:
            input_duplicate_timestamps += 1
            if same_pose:
                duplicate_same_pose += 1
            else:
                timestamp_anomaly_pose_change += 1
        elif delta < 0:
            input_backwards_timestamps += 1
            if same_pose:
                backwards_same_pose += 1
            else:
                timestamp_anomaly_pose_change += 1

    # Preserve pose-run boundaries. Never merge equal poses separated by a
    # different pose, because doing so could reorder a genuine trajectory loop.
    runs: list[list[dict[str, Any]]] = []
    for record in records:
        if runs and runs[-1][0]["pose"] == record["pose"]:
            runs[-1].append(record)
        else:
            runs.append([record])

    candidate_records: list[dict[str, Any]] = []
    duplicates_removed = 0
    run_length_counts: Counter[int] = Counter()
    run_boundary_gaps: list[float] = []

    previous_run_max: float | None = None
    for run_index, run in enumerate(runs, start=1):
        run_min = min(item["timestamp"] for item in run)
        run_max = max(item["timestamp"] for item in run)
        if previous_run_max is not None:
            boundary_gap = run_min - previous_run_max
            if boundary_gap <= 0:
                raise ValueError(
                    "pose-run timestamp ranges overlap or go backwards at a pose boundary; "
                    f"run {run_index - 1} max={previous_run_max:.9f}, "
                    f"run {run_index} min={run_min:.9f}. Refusing to reorder across poses."
                )
            run_boundary_gaps.append(boundary_gap)

        sorted_run = sorted(run, key=lambda item: item["timestamp"])
        unique_run: list[dict[str, Any]] = []
        seen_timestamps: set[float] = set()
        for item in sorted_run:
            timestamp = item["timestamp"]
            if timestamp in seen_timestamps:
                # Pose fields are identical within this run, so this is an
                # exact duplicate timestamp+pose observation.
                duplicates_removed += 1
                continue
            seen_timestamps.add(timestamp)
            unique_run.append(item)

        if not unique_run:
            raise ValueError(f"pose run {run_index} produced no unique timestamps")
        candidate_records.extend(unique_run)
        run_length_counts[len(run)] += 1
        previous_run_max = run_max

    candidate_timestamps = [item["timestamp"] for item in candidate_records]
    if any(b <= a for a, b in zip(candidate_timestamps, candidate_timestamps[1:])):
        raise ValueError("internal error: generated candidate is not strictly time ordered")

    candidate_lines = [*comments, *(item["raw"] for item in candidate_records)]
    candidate_text = "\n".join(candidate_lines) + "\n"
    source_sha = sha256_file(source_path)
    if source_sha != source_sha_before:
        raise RuntimeError("source trajectory changed while being audited; refusing candidate generation")

    manifest = {
        "schema_version": 1,
        "status": "ordered_tum_candidate_not_validated_ground_truth",
        "source_file": str(source_path),
        "source_size_bytes": source_path.stat().st_size,
        "source_sha256": source_sha,
        "output_file": "trajectory.tum",
        "output_sha256": None,
        "method": {
            "pose_run_definition": "maximal consecutive run with exact equality of tx ty tz qx qy qz qw",
            "ordering": "sort timestamps only within each constant-pose run; preserve run order",
            "deduplication": "remove repeated timestamps within a constant-pose run; pose fields are identical in that run",
            "cross_pose_policy": "require each run minimum timestamp to be strictly greater than the previous run maximum; otherwise refuse",
            "coordinate_or_pose_transform": "none",
            "time_offset_or_resampling": "none",
        },
        "input": {
            "rows": len(records),
            "duplicate_timestamps": input_duplicate_timestamps,
            "backwards_timestamps": input_backwards_timestamps,
            "duplicate_timestamp_same_pose_pairs": duplicate_same_pose,
            "backwards_timestamp_same_pose_pairs": backwards_same_pose,
            "timestamp_anomaly_pose_change_pairs": timestamp_anomaly_pose_change,
            "consecutive_pose_run_count": len(runs),
            "pose_run_length_distribution": {
                str(size): count for size, count in sorted(run_length_counts.items())
            },
        },
        "output": {
            "rows": len(candidate_records),
            "exact_duplicate_timestamp_pose_rows_removed": duplicates_removed,
            "strictly_increasing_timestamps": True,
            "first_timestamp": candidate_timestamps[0],
            "last_timestamp": candidate_timestamps[-1],
            "duration_sec": candidate_timestamps[-1] - candidate_timestamps[0],
            "minimum_gap_between_pose_runs_sec": (
                min(run_boundary_gaps) if run_boundary_gaps else None
            ),
        },
        "limitations": [
            "This is a derived candidate only, not an assertion that the source is authoritative GT.",
            "The source file is not modified; this output must be kept separate and traceable to its SHA256.",
            "The operation assumes timestamps can be reordered only while pose fields are exactly identical.",
            "No clock-domain, time offset, drift, coordinate frame, geodetic datum, or GT accuracy is established.",
            "Confirm the Dynamic03 GT export provenance and author semantics before using this candidate for evo APE/RPE.",
        ],
    }

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    staging_root = Path(
        tempfile.mkdtemp(prefix=".tum-pose-run-candidate-", dir=output_dir.parent)
    )
    staged_dir = staging_root / "candidate"
    try:
        staged_dir.mkdir()
        staged_trajectory = staged_dir / "trajectory.tum"
        staged_trajectory.write_text(candidate_text, encoding="utf-8")
        candidate_sha = sha256_file(staged_trajectory)
        manifest["output_sha256"] = candidate_sha
        (staged_dir / "manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if output_dir.exists():
            raise FileExistsError(
                f"refusing to overwrite existing output directory: {output_dir}"
            )
        os.replace(staged_dir, output_dir)
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)

    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Create a non-authoritative TUM candidate by ordering timestamps only "
            "inside consecutive identical-pose runs."
        )
    )
    parser.add_argument("trajectory", type=Path, help="Original TUM trajectory; never modified")
    parser.add_argument(
        "output_dir",
        type=Path,
        help="New output directory; contains trajectory.tum and manifest.json",
    )
    parser.add_argument(
        "--quaternion-norm-tolerance",
        type=float,
        default=1e-3,
        help="Maximum absolute deviation of quaternion norm from 1 (default: 1e-3)",
    )
    args = parser.parse_args()

    try:
        report = create_pose_run_candidate(
            args.trajectory,
            args.output_dir,
            quaternion_norm_tolerance=args.quaternion_norm_tolerance,
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
