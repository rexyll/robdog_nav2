#!/usr/bin/env python3
"""Read-only structural audit for the M3DGR Dynamic03 ROS 1 bag.

This tool reads raw message records and only deserializes a few samples from
the MID-360 LiDAR and IMU topics. It never rewrites or converts the source bag.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any


EXPECTED_LIDAR_TYPES = {
    "livox_ros_driver/msg/CustomMsg",
    "livox_ros_driver/CustomMsg",
    "livox_ros_driver2/msg/CustomMsg",
}
EXPECTED_IMU_TYPES = {
    "sensor_msgs/msg/Imu",
    "sensor_msgs/Imu",
}


def stamp_to_ns(message: Any) -> int | None:
    """Return ROS header stamp as nanoseconds, or None if unavailable."""
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsec", None))
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def summarize_frequency(count: int, start_ns: int | None, end_ns: int | None) -> float | None:
    if count < 2 or start_ns is None or end_ns is None or end_ns <= start_ns:
        return None
    return (count - 1) / ((end_ns - start_ns) / 1_000_000_000.0)


def _connection_details(connection: Any) -> dict[str, Any]:
    msgdef = getattr(connection, "msgdef", None)
    if msgdef is None:
        msgdef_length = None
    elif isinstance(msgdef, str):
        msgdef_length = len(msgdef)
    else:
        msgdef_length = len(str(msgdef))
    return {
        "connection_id": int(getattr(connection, "id", -1)),
        "topic": str(connection.topic),
        "msgtype": str(connection.msgtype),
        "digest": getattr(connection, "digest", None),
        "message_definition_length": msgdef_length,
    }


def audit_ros1_bag(
    bag_path: Path,
    *,
    lidar_topic: str = "/livox/mid360/lidar",
    imu_topic: str = "/livox/mid360/imu",
    sample_limit: int = 3,
) -> dict[str, Any]:
    """Read bag metadata, message timestamps and a few sensor payload samples."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:  # pragma: no cover - exercised by CLI environment
        raise RuntimeError(
            "Missing dependency 'rosbags'. Install with: "
            "python3 -m pip install rosbags==0.11.5"
        ) from exc

    bag_path = bag_path.expanduser().resolve()
    if not bag_path.is_file():
        raise FileNotFoundError(f"ROS 1 bag file not found: {bag_path}")
    if bag_path.suffix.lower() != ".bag":
        raise ValueError(
            f"Expected a ROS 1 .bag file, got {bag_path.name!r}; "
            "do not rename a ROS 2 .db3 file to .bag."
        )
    with bag_path.open("rb") as handle:
        magic = handle.read(13)
    if magic != b"#ROSBAG V2.0\\n":
        raise ValueError(
            "File does not have ROS 1 bag v2.0 magic; do not treat a ROS 2 DB3 as ROS1."
        )

    connection_details: list[dict[str, Any]] = []
    topic_stats: dict[str, dict[str, Any]] = {}
    sample_state: dict[str, dict[str, Any]] = {
        lidar_topic: {"decoded": 0, "samples": [], "errors": []},
        imu_topic: {"decoded": 0, "samples": [], "errors": []},
    }

    with AnyReader([bag_path]) as reader:
        for connection in reader.connections:
            connection_details.append(_connection_details(connection))
            topic = str(connection.topic)
            stats = topic_stats.setdefault(
                topic,
                {
                    "message_count": 0,
                    "message_types": set(),
                    "start_time_ns": None,
                    "end_time_ns": None,
                    "last_seen_time_ns": None,
                    "duplicate_timestamps": 0,
                    "backwards_timestamps": 0,
                    "raw_message_size_min": None,
                    "raw_message_size_max": 0,
                },
            )
            stats["message_types"].add(str(connection.msgtype))

        for connection, timestamp_ns, rawdata in reader.messages():
            topic = str(connection.topic)
            timestamp_ns = int(timestamp_ns)
            stats = topic_stats[topic]
            stats["message_count"] += 1
            stats["start_time_ns"] = (
                timestamp_ns
                if stats["start_time_ns"] is None
                else min(stats["start_time_ns"], timestamp_ns)
            )
            stats["end_time_ns"] = (
                timestamp_ns
                if stats["end_time_ns"] is None
                else max(stats["end_time_ns"], timestamp_ns)
            )
            previous = stats["last_seen_time_ns"]
            if previous is not None:
                if timestamp_ns == previous:
                    stats["duplicate_timestamps"] += 1
                elif timestamp_ns < previous:
                    stats["backwards_timestamps"] += 1
            stats["last_seen_time_ns"] = timestamp_ns

            raw_size = len(rawdata)
            stats["raw_message_size_min"] = (
                raw_size
                if stats["raw_message_size_min"] is None
                else min(stats["raw_message_size_min"], raw_size)
            )
            stats["raw_message_size_max"] = max(stats["raw_message_size_max"], raw_size)

            if topic not in sample_state:
                continue
            state = sample_state[topic]
            if state["decoded"] + len(state["errors"]) >= sample_limit:
                continue
            try:
                message = reader.deserialize(rawdata, connection.msgtype)
                header_stamp_ns = stamp_to_ns(message)
                sample: dict[str, Any] = {
                    "bag_timestamp_ns": timestamp_ns,
                    "header_stamp_ns": header_stamp_ns,
                    "header_frame_id": getattr(
                        getattr(message, "header", None), "frame_id", None
                    ),
                    "raw_message_size_bytes": raw_size,
                    "msgtype": str(connection.msgtype),
                }
                if topic == lidar_topic:
                    points = getattr(message, "points", None)
                    offsets = (
                        [int(getattr(point, "offset_time")) for point in points]
                        if points is not None
                        else []
                    )
                    declared_count = getattr(message, "point_num", None)
                    sample.update(
                        {
                            "timebase_raw": getattr(message, "timebase", None),
                            "point_num_declared": (
                                int(declared_count) if declared_count is not None else None
                            ),
                            "points_decoded": len(points) if points is not None else None,
                            "offset_time_raw_min": min(offsets) if offsets else None,
                            "offset_time_raw_max": max(offsets) if offsets else None,
                        }
                    )
                    if not hasattr(message, "timebase") or points is None:
                        raise ValueError(
                            "Decoded LiDAR message lacks Livox CustomMsg timebase/points fields"
                        )
                state["decoded"] += 1
                state["samples"].append(sample)
            except Exception as exc:  # keep audit report useful on partial decode failure
                state["errors"].append(
                    {
                        "bag_timestamp_ns": timestamp_ns,
                        "msgtype": str(connection.msgtype),
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                )

        bag_start = int(reader.start_time) if reader.message_count else None
        bag_end = int(reader.end_time) if reader.message_count else None
        bag_duration = int(reader.duration) if reader.message_count else 0
        total_messages = int(reader.message_count)

    finalized_topics: dict[str, Any] = {}
    for topic, stats in sorted(topic_stats.items()):
        finalized_topics[topic] = {
            "message_count": stats["message_count"],
            "message_types": sorted(stats["message_types"]),
            "start_time_ns": stats["start_time_ns"],
            "end_time_ns": stats["end_time_ns"],
            "duration_sec": (
                (stats["end_time_ns"] - stats["start_time_ns"]) / 1_000_000_000.0
                if stats["start_time_ns"] is not None and stats["end_time_ns"] is not None
                else None
            ),
            "average_rate_hz": summarize_frequency(
                stats["message_count"], stats["start_time_ns"], stats["end_time_ns"]
            ),
            "duplicate_timestamps": stats["duplicate_timestamps"],
            "backwards_timestamps": stats["backwards_timestamps"],
            "raw_message_size_min_bytes": stats["raw_message_size_min"],
            "raw_message_size_max_bytes": stats["raw_message_size_max"],
        }

    issues: list[str] = []
    lidar_info = finalized_topics.get(lidar_topic)
    imu_info = finalized_topics.get(imu_topic)
    if lidar_info is None:
        issues.append(f"required LiDAR topic missing: {lidar_topic}")
    elif not set(lidar_info["message_types"]).issubset(EXPECTED_LIDAR_TYPES):
        issues.append(
            f"unexpected LiDAR message type(s) on {lidar_topic}: "
            f"{lidar_info['message_types']}"
        )
    if imu_info is None:
        issues.append(f"required MID-360 IMU topic missing: {imu_topic}")
    elif not set(imu_info["message_types"]).issubset(EXPECTED_IMU_TYPES):
        issues.append(
            f"unexpected IMU message type(s) on {imu_topic}: {imu_info['message_types']}"
        )

    for topic, label in ((lidar_topic, "LiDAR"), (imu_topic, "IMU")):
        state = sample_state[topic]
        if not state["samples"]:
            issues.append(f"no {label} sample could be deserialized from {topic}")
        if state["errors"]:
            issues.append(
                f"{label} sample deserialization errors on {topic}: "
                f"{len(state['errors'])}"
            )

    report = {
        "schema_version": 1,
        "dataset_candidate": "M3DGR",
        "sequence_candidate": "Dynamic03",
        "input_format": "ROS1_bag",
        "input_file": str(bag_path),
        "file_size_bytes": bag_path.stat().st_size,
        "bag_message_count": total_messages,
        "bag_start_time_ns": bag_start,
        "bag_end_time_ns": bag_end,
        "bag_duration_sec": bag_duration / 1_000_000_000.0,
        "required_topics": {
            "lidar": lidar_topic,
            "imu": imu_topic,
        },
        "connections": connection_details,
        "topics": finalized_topics,
        "sample_deserialization": sample_state,
        "issues": issues,
        "status": "structural_inputs_present" if not issues else "blocked_review_required",
        "limitations": [
            "This is a read-only structural audit, not an algorithm replay.",
            "Bag timestamps and header/timebase fields are reported separately; "
            "clock synchronization is not inferred.",
            "Raw Livox offset_time values are reported without assigning a unit "
            "until the message definition and driver semantics are verified.",
            "A topic/type match does not verify ground-truth correctness or data license.",
        ],
    }
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only audit of M3DGR Dynamic03 ROS 1 bag structure."
    )
    parser.add_argument("bag", type=Path, help="Path to the original Dynamic03.bag file")
    parser.add_argument(
        "--json-out",
        type=Path,
        help="Optional path for a machine-readable JSON report",
    )
    parser.add_argument(
        "--lidar-topic", default="/livox/mid360/lidar", help="Expected MID-360 LiDAR topic"
    )
    parser.add_argument(
        "--imu-topic", default="/livox/mid360/imu", help="Expected MID-360 built-in IMU topic"
    )
    args = parser.parse_args()

    try:
        report = audit_ros1_bag(
            args.bag,
            lidar_topic=args.lidar_topic,
            imu_topic=args.imu_topic,
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    output = json.dumps(report, ensure_ascii=False, indent=2)
    print(output)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output + "\n", encoding="utf-8")

    return 0 if report["status"] == "structural_inputs_present" else 10


if __name__ == "__main__":
    raise SystemExit(main())
