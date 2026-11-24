#!/usr/bin/env python3
"""Read-only signature and INSPVAX payload audit for UrbanLoco HK-Data20190117.

Accepts either a rosbag2 directory or one standalone SQLite3 .db3 file.
No ROS message packages are required; point cloud and IMU payloads are not
deserialized. The INSPVAX CDR layout is checked using the fixed, documented
HK-Data20190117 offsets before the dataset is admitted to the P2 pipeline.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path
from typing import Any

from urbanloco_gt_helpers import parse_inspvax_position

CORE_TOPICS = {
    "/velodyne_points_0": "sensor_msgs/msg/PointCloud2",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/novatel_data/inspvax": "novatel_oem7_msgs/msg/INSPVAX",
}


def db3_files(input_path: Path) -> list[Path]:
    path = input_path.expanduser().resolve()
    if path.is_file():
        if path.suffix.lower() != ".db3":
            raise ValueError(f"Standalone input must be a .db3 file: {path}")
        return [path]
    if path.is_dir():
        files = sorted(path.glob("*.db3"))
        if files:
            return files
    raise FileNotFoundError(
        f"Expected a standalone .db3 file or a directory containing .db3 files: {path}"
    )


def _timestamp_summary(values: list[int]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min_ns": None, "max_ns": None, "duration_s": None,
                "average_hz": None, "duplicate_count": 0, "backward_count": 0}
    duplicate_count = 0
    backward_count = 0
    for previous, current in zip(values, values[1:]):
        if current == previous:
            duplicate_count += 1
        elif current < previous:
            backward_count += 1
    minimum, maximum = min(values), max(values)
    duration = (maximum - minimum) / 1_000_000_000.0
    rate = (len(values) - 1) / duration if duration > 0.0 and len(values) > 1 else None
    return {
        "count": len(values),
        "min_ns": minimum,
        "max_ns": maximum,
        "duration_s": duration,
        "average_hz": rate,
        "duplicate_count": duplicate_count,
        "backward_count": backward_count,
    }


def audit_bag(input_path: Path) -> dict[str, Any]:
    files = db3_files(input_path)
    topic_types: dict[str, str] = {}
    topic_times: dict[str, list[int]] = {name: [] for name in CORE_TOPICS}
    topic_counts: dict[str, int] = {name: 0 for name in CORE_TOPICS}
    global_min: int | None = None
    global_max: int | None = None
    positions: list[tuple[float, float, float]] = []
    gt_count = 0

    for db3_path in files:
        uri = db3_path.resolve().as_uri() + "?mode=ro"
        with sqlite3.connect(uri, uri=True) as connection:
            tables = {
                row[0]
                for row in connection.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                )
            }
            if not {"topics", "messages"}.issubset(tables):
                raise ValueError(f"{db3_path} is not a rosbag2 SQLite database")

            columns = {
                row[1] for row in connection.execute("PRAGMA table_info(topics)")
            }
            if not {"id", "name", "type"}.issubset(columns):
                raise ValueError(f"Unexpected rosbag2 topics schema in {db3_path}")

            for topic_id, name, type_name in connection.execute(
                "SELECT id, name, type FROM topics"
            ):
                if name in CORE_TOPICS:
                    old_type = topic_types.get(name)
                    if old_type is not None and old_type != type_name:
                        raise TypeError(
                            f"Inconsistent type for {name}: {old_type!r} vs {type_name!r}"
                        )
                    topic_types[name] = str(type_name)
                    rows = connection.execute(
                        "SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY id",
                        (topic_id,),
                    )
                    times = [int(row[0]) for row in rows]
                    topic_times[name].extend(times)
                    topic_counts[name] += len(times)

                    if name == "/novatel_data/inspvax":
                        payload_rows = connection.execute(
                            "SELECT id, data FROM messages WHERE topic_id = ? ORDER BY id",
                            (topic_id,),
                        )
                        for message_id, payload in payload_rows:
                            try:
                                positions.append(
                                    parse_inspvax_position(bytes(payload))
                                )
                            except Exception as exc:
                                raise ValueError(
                                    f"INSPVAX message id {message_id} in {db3_path} "
                                    f"failed the fixed CDR layout/range check: {exc}"
                                ) from exc
                            gt_count += 1

            bounds = connection.execute(
                "SELECT MIN(timestamp), MAX(timestamp) FROM messages"
            ).fetchone()
            if bounds and bounds[0] is not None:
                low, high = int(bounds[0]), int(bounds[1])
                global_min = low if global_min is None else min(global_min, low)
                global_max = high if global_max is None else max(global_max, high)

    for name, expected_type in CORE_TOPICS.items():
        actual_type = topic_types.get(name)
        if actual_type is not None and actual_type != expected_type:
            raise TypeError(
                f"{name} type is {actual_type!r}; expected {expected_type!r}"
            )
    missing = [name for name in CORE_TOPICS if topic_counts[name] == 0]
    if missing:
        raise ValueError(f"Required UrbanLoco core topics are missing/empty: {missing}")
    if gt_count < 2:
        raise ValueError(f"Expected at least two INSPVAX samples; found {gt_count}")

    summaries = {
        name: {
            "type": topic_types.get(name),
            "count": topic_counts[name],
            "timestamps": _timestamp_summary(topic_times[name]),
        }
        for name in CORE_TOPICS
    }
    latitudes = [point[0] for point in positions]
    longitudes = [point[1] for point in positions]
    heights = [point[2] for point in positions]
    duration_s = (
        (global_max - global_min) / 1_000_000_000.0
        if global_min is not None and global_max is not None else None
    )
    is_single_file = input_path.expanduser().resolve().is_file()
    return {
        "input_path": str(input_path.expanduser().resolve()),
        "input_kind": "standalone_db3" if is_single_file else "rosbag2_directory",
        "database_files": [
            {"path": str(path), "size_bytes": path.stat().st_size} for path in files
        ],
        "global_message_time": {
            "min_ns": global_min, "max_ns": global_max, "duration_s": duration_s
        },
        "required_topics": summaries,
        "inspvax_payload_audit": {
            "decoded_count": gt_count,
            "cdr_layout": "UrbanLoco HK-Data20190117 novatel_oem7_msgs/msg/INSPVAX",
            "field_offsets_bytes": {
                "latitude_deg": 60, "longitude_deg": 68, "height_m": 76
            },
            "latitude_range_deg": [min(latitudes), max(latitudes)],
            "longitude_range_deg": [min(longitudes), max(longitudes)],
            "height_range_m": [min(heights), max(heights)],
        },
        "assessment": (
            "core topic/type signature and all INSPVAX payloads match the "
            "project's HK-Data20190117 P2 parser contract; source provenance "
            "still depends on the user's download record/checksum"
        ),
        "limitations": [
            "This audit does not deserialize PointCloud2 or Imu payloads.",
            "Topic signature alone is not cryptographic proof of dataset provenance.",
            "A valid INSPVAX position layout does not validate GNSS timing or accuracy.",
            "This is an offline Velodyne dataset audit, not MID-360 or robot hardware validation.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_input", type=Path,
                        help="rosbag2 directory or standalone .db3 file")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    try:
        report = audit_bag(args.bag_input)
        if args.json_out:
            args.json_out.parent.mkdir(parents=True, exist_ok=True)
            args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
        print("PASS: UrbanLoco P2 core topic/type and INSPVAX payload audit")
        return 0
    except Exception as exc:
        parser.exit(2, f"ERROR: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
