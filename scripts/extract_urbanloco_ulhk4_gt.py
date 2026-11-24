#!/usr/bin/env python3
"""Extract the fixed UrbanLoco HK-Data20190117 NovAtel INSPVAX GT to TUM."""
from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path
from typing import Any

from urbanloco_gt_helpers import (
    geodetic_to_enu,
    order_and_deduplicate,
    parse_inspvax_position,
)

EXPECTED_TYPE_TOKEN = "INSPVAX"


def _resolve_db3_files(bag_path: Path) -> list[Path]:
    path = bag_path.expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".db3":
            raise ValueError(f"Standalone rosbag input must be a .db3 file: {path}")
        return [path]
    if path.is_dir():
        db3_files = sorted(path.glob("*.db3"))
        if db3_files:
            return db3_files
    raise FileNotFoundError(
        f"No standalone .db3 file or rosbag2 directory containing .db3 files: {path}"
    )


def extract_samples(bag_path: Path, topic: str) -> tuple[list[tuple[int, float, float, float]], str]:
    db3_files = _resolve_db3_files(bag_path)

    samples: list[tuple[int, float, float, float]] = []
    found_type: str | None = None

    for db3_path in db3_files:
        with sqlite3.connect(str(db3_path)) as connection:
            row = connection.execute(
                "SELECT id, type FROM topics WHERE name = ?", (topic,)
            ).fetchone()
            if row is None:
                continue
            topic_id, topic_type = row
            if EXPECTED_TYPE_TOKEN not in str(topic_type).upper():
                raise TypeError(
                    f"Topic {topic} has type {topic_type!r}; expected NovAtel INSPVAX"
                )
            if found_type is not None and found_type != topic_type:
                raise TypeError(
                    f"Topic {topic} has inconsistent types across bag files: "
                    f"{found_type!r} vs {topic_type!r}"
                )
            found_type = str(topic_type)

            cursor = connection.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,),
            )
            for timestamp_ns, serialized in cursor:
                lat, lon, height = parse_inspvax_position(bytes(serialized))
                samples.append((int(timestamp_ns), lat, lon, height))

    if not samples:
        raise ValueError(
            f"No {topic} INSPVAX messages found in {bag_path}; check ros2 bag info"
        )
    if found_type is None:
        raise ValueError(f"Ground-truth topic {topic} not found in bag metadata")
    return samples, found_type


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-path", "--bag-dir", dest="bag_path", type=Path, required=True,
                        help="rosbag2 directory or standalone SQLite3 .db3 file")
    parser.add_argument("--topic", default="/novatel_data/inspvax")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    try:
        raw_samples, topic_type = extract_samples(args.bag_path, args.topic)
        unique_samples, duplicate_count = order_and_deduplicate(raw_samples)
        if len(unique_samples) < 2:
            raise ValueError(
                f"Need at least 2 unique ground-truth samples, got {len(unique_samples)}"
            )

        origin = (unique_samples[0][1], unique_samples[0][2], unique_samples[0][3])
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", encoding="utf-8") as output:
            for timestamp_ns, lat, lon, height in unique_samples:
                east, north, up = geodetic_to_enu(lat, lon, height, origin)
                timestamp_s = timestamp_ns / 1_000_000_000.0
                # Orientation is identity; evo_ape performs rigid Umeyama
                # alignment (-a) because this GT frame is local ENU.
                output.write(
                    f"{timestamp_s:.9f} {east:.6f} {north:.6f} {up:.6f} "
                    "0 0 0 1\n"
                )

        report: dict[str, Any] = {
            "bag_input": str(args.bag_path.resolve()),
            "topic": args.topic,
            "topic_type": topic_type,
            "cdr_layout": "UrbanLoco HK-Data20190117 novatel_oem7_msgs/INSPVAX",
            "cdr_field_offsets_bytes": {
                "latitude_deg": 60,
                "longitude_deg": 68,
                "height_m": 76,
            },
            "timestamp_source": "rosbag2 SQLite messages.timestamp (nanoseconds)",
            "raw_sample_count": len(raw_samples),
            "duplicate_timestamp_count_removed": duplicate_count,
            "unique_sample_count": len(unique_samples),
            "enu_origin_geodetic": {
                "latitude_deg": origin[0],
                "longitude_deg": origin[1],
                "height_m": origin[2],
            },
            "tum_output": str(args.out.resolve()),
            "orientation_note": "identity quaternion; evo_ape must run with Umeyama alignment",
        }
        json_path = args.json_out or args.out.with_suffix(".json")
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"Ground truth poses: {len(unique_samples)}")
        print(f"Duplicate timestamps removed: {duplicate_count}")
        print(f"TUM: {args.out}")
        print(f"Report: {json_path}")
        return 0
    except Exception as exc:
        parser.exit(2, f"ERROR: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
