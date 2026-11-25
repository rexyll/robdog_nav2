#!/usr/bin/env python3
"""Read-only audit of MID-360 Livox CustomMsg and IMU timestamps in a ROS 2 bag.

Run after sourcing ROS 2 Jazzy and installing rosbag2_py plus livox_ros_driver2:
    python3 scripts/audit_livox_rosbag_timing.py /path/to/bag

The tool reports raw time semantics; it does not claim that PTP is enabled or
that the physical LiDAR/IMU clocks are synchronized.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any, Sequence


NSEC_PER_SEC = 1_000_000_000
NSEC_PER_MSEC = 1_000_000


from livox_time_audit_helpers import (
    nearest_neighbor_deltas_ns,
    percentile,
    stamp_to_ns,
    summarize_time_series,
    summarize_values,
    summarize_timestamp_pairing,
    validate_cli_thresholds,
)


def summarize_offsets(
    samples_ns: Sequence[int],
    total_count: int,
    order_violation_count: int,
    negative_count: int,
) -> dict[str, Any]:
    sampled = summarize_values(samples_ns)
    if sampled["min"] is not None:
        sampled["p50_ms"] = sampled["p50"] / NSEC_PER_MSEC
        sampled["p95_ms"] = sampled["p95"] / NSEC_PER_MSEC
        sampled["p99_ms"] = sampled["p99"] / NSEC_PER_MSEC
        sampled["max_ms"] = sampled["max"] / NSEC_PER_MSEC
    return {
        "total_point_count": total_count,
        "sampled_point_count": len(samples_ns),
        "offset_ns_sample": sampled,
        "negative_offset_count": negative_count,
        "within_frame_order_violation_count": order_violation_count,
    }


def audit_rosbag(
    bag_uri: str | Path,
    storage_id: str = "sqlite3",
    lidar_topic: str = "/livox/lidar",
    imu_topic: str = "/livox/imu",
    offset_sample_cap: int = 100_000,
) -> dict[str, Any]:
    """Read Livox CustomMsg and sensor_msgs/Imu records and summarize timing."""
    if offset_sample_cap < 1:
        raise ValueError("offset_sample_cap must be at least 1")

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS bag audit dependencies are missing. Source ROS 2 Jazzy and "
            "install rosbag2_py, rosidl_runtime_py, and livox_ros_driver2."
        ) from exc

    uri = str(Path(bag_uri).expanduser().resolve())
    if not Path(uri).exists():
        raise FileNotFoundError(f"Bag path does not exist: {uri}")

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = [topic for topic in (lidar_topic, imu_topic) if topic not in topic_types]
    if missing:
        raise ValueError(
            f"Required topics missing from bag: {missing}; found {sorted(topic_types)}"
        )

    lidar_type_name = topic_types[lidar_topic]
    imu_type_name = topic_types[imu_topic]
    expected_lidar_type = "livox_ros_driver2/msg/CustomMsg"
    expected_imu_type = "sensor_msgs/msg/Imu"
    if lidar_type_name != expected_lidar_type:
        raise TypeError(
            f"{lidar_topic} type is {lidar_type_name}, expected {expected_lidar_type}"
        )
    if imu_type_name != expected_imu_type:
        raise TypeError(
            f"{imu_topic} type is {imu_type_name}, expected {expected_imu_type}"
        )

    lidar_message_class = get_message(lidar_type_name)
    imu_message_class = get_message(imu_type_name)

    lidar_timebase_ns: list[int] = []
    lidar_bag_receive_ns: list[int] = []
    imu_header_ns: list[int] = []
    imu_bag_receive_ns: list[int] = []
    header_minus_timebase_ns: list[int] = []
    offset_samples_ns: list[int] = []
    total_points = 0
    negative_offsets = 0
    offset_order_violations = 0
    lidar_messages = 0
    imu_messages = 0

    while reader.has_next():
        topic, serialized, bag_receive_timestamp_ns = reader.read_next()
        bag_receive_timestamp_ns = int(bag_receive_timestamp_ns)
        if topic == lidar_topic:
            message = deserialize_message(serialized, lidar_message_class)
            lidar_messages += 1
            timebase_ns = int(message.timebase)
            header_ns = stamp_to_ns(message.header.stamp)
            lidar_timebase_ns.append(timebase_ns)
            lidar_bag_receive_ns.append(bag_receive_timestamp_ns)
            header_minus_timebase_ns.append(header_ns - timebase_ns)

            previous_offset: int | None = None
            points = message.points
            total_points += len(points)
            # Sample evenly within each frame, capped globally to bound memory.
            stride = max(1, math.ceil(len(points) / 1000))
            for point_index, point in enumerate(points):
                offset_ns = int(point.offset_time)
                if offset_ns < 0:
                    negative_offsets += 1
                if previous_offset is not None and offset_ns < previous_offset:
                    offset_order_violations += 1
                previous_offset = offset_ns
                if point_index % stride == 0 and len(offset_samples_ns) < offset_sample_cap:
                    offset_samples_ns.append(offset_ns)
        elif topic == imu_topic:
            message = deserialize_message(serialized, imu_message_class)
            imu_messages += 1
            imu_header_ns.append(stamp_to_ns(message.header.stamp))
            imu_bag_receive_ns.append(bag_receive_timestamp_ns)

    if lidar_messages == 0 or imu_messages == 0:
        raise ValueError(
            f"Bag contains no data for required topics: lidar={lidar_messages}, "
            f"imu={imu_messages}"
        )

    nearest_delta_ns = nearest_neighbor_deltas_ns(lidar_timebase_ns, imu_header_ns)
    header_delta_abs_ns = [abs(value) for value in header_minus_timebase_ns]
    return {
        "bag_uri": uri,
        "storage_id": storage_id,
        "topics": {
            lidar_topic: lidar_type_name,
            imu_topic: imu_type_name,
        },
        "timestamp_contract": {
            "lidar_timebase_and_header_stamp_unit": "nanoseconds",
            "point_offset_time_unit": "nanoseconds",
            "point_lio_custom_msg_conversion": "offset_time_ns / 1e6 = milliseconds",
            "imu_header_stamp_unit": "nanoseconds",
            "ptp_or_physical_clock_sync_claimed": False,
        },
        "message_counts": {
            "lidar_custom_msg": lidar_messages,
            "imu": imu_messages,
        },
        "lidar_timebase_ns": summarize_time_series(lidar_timebase_ns),
        "imu_header_stamp_ns": summarize_time_series(imu_header_ns),
        "rosbag_receive_timestamp": {
            "semantics": "SequentialReader.read_next timestamp; rosbag2 message receive timestamp",
            "clock_domain": "not inferable from bag payload alone; inspect recorder use_sim_time and /clock configuration",
            "lidar_ns": summarize_time_series(lidar_bag_receive_ns),
            "imu_ns": summarize_time_series(imu_bag_receive_ns),
            "lidar_receive_minus_custommsg_timebase": summarize_timestamp_pairing(
                lidar_timebase_ns, lidar_bag_receive_ns
            ),
            "imu_receive_minus_header_stamp": summarize_timestamp_pairing(
                imu_header_ns, imu_bag_receive_ns
            ),
            "warning": "receive-minus-sensor deltas are descriptive only; they are not transport latency or clock offset without proven clock-domain mapping",
        },
        "lidar_header_stamp_minus_timebase_ns": summarize_values(
            header_minus_timebase_ns
        ),
        "absolute_lidar_imu_nearest_sample_delta_ns": summarize_values(
            nearest_delta_ns
        ),
        "point_offsets": summarize_offsets(
            offset_samples_ns,
            total_points,
            offset_order_violations,
            negative_offsets,
        ),
        "diagnostics": {
            "header_timebase_mismatch_count": sum(
                value != 0 for value in header_minus_timebase_ns
            ),
            "lidar_timebase_non_monotonic_count": (
                summarize_time_series(lidar_timebase_ns)["duplicate_timestamp_count"]
                + summarize_time_series(lidar_timebase_ns)["backward_timestamp_count"]
            ),
            "imu_timestamp_non_monotonic_count": (
                summarize_time_series(imu_header_ns)["duplicate_timestamp_count"]
                + summarize_time_series(imu_header_ns)["backward_timestamp_count"]
            ),
            "point_offset_order_violation_count": offset_order_violations,
            "lidar_bag_receive_timestamp_non_monotonic_count": (
                summarize_time_series(lidar_bag_receive_ns)["duplicate_timestamp_count"]
                + summarize_time_series(lidar_bag_receive_ns)["backward_timestamp_count"]
            ),
            "imu_bag_receive_timestamp_non_monotonic_count": (
                summarize_time_series(imu_bag_receive_ns)["duplicate_timestamp_count"]
                + summarize_time_series(imu_bag_receive_ns)["backward_timestamp_count"]
            ),
            "nearest_lidar_imu_delta_is_not_a_clock_sync_proof": True,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit MID-360 Livox CustomMsg and IMU timestamps in a ROS 2 bag."
    )
    parser.add_argument("bag_uri", help="rosbag2 directory")
    parser.add_argument("--storage-id", default="sqlite3", choices=("sqlite3", "mcap"))
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--offset-sample-cap", type=int, default=100_000)
    parser.add_argument("--max-header-timebase-delta-ns", type=int, default=0)
    parser.add_argument(
        "--max-nearest-lidar-imu-p95-ms",
        type=float,
        default=None,
        help="Optional project-defined threshold; no default safety threshold is assumed.",
    )
    parser.add_argument("--strict-monotonic", action="store_true")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()

    try:
        validate_cli_thresholds(
            args.max_header_timebase_delta_ns,
            args.max_nearest_lidar_imu_p95_ms,
        )
    except ValueError as exc:
        parser.error(str(exc))

    try:
        report = audit_rosbag(
            args.bag_uri,
            storage_id=args.storage_id,
            lidar_topic=args.lidar_topic,
            imu_topic=args.imu_topic,
            offset_sample_cap=args.offset_sample_cap,
        )
    except Exception as exc:  # CLI boundary: provide a concise actionable error.
        parser.exit(2, f"ERROR: {exc}\n")

    header_delta = report["lidar_header_stamp_minus_timebase_ns"]
    max_header_delta = max(abs(header_delta["min"]), abs(header_delta["max"]))
    failures: list[str] = []
    if max_header_delta > args.max_header_timebase_delta_ns:
        failures.append(
            "LiDAR header stamp differs from CustomMsg.timebase by "
            f"{max_header_delta} ns (limit {args.max_header_timebase_delta_ns} ns)"
        )

    nearest = report["absolute_lidar_imu_nearest_sample_delta_ns"]
    nearest_p95_ms = (
        nearest["p95"] / NSEC_PER_MSEC if nearest["p95"] is not None else None
    )
    if (
        args.max_nearest_lidar_imu_p95_ms is not None
        and nearest_p95_ms is not None
        and nearest_p95_ms > args.max_nearest_lidar_imu_p95_ms
    ):
        failures.append(
            "P95 nearest LiDAR/IMU sample delta is "
            f"{nearest_p95_ms:.3f} ms (limit {args.max_nearest_lidar_imu_p95_ms:.3f} ms)"
        )

    if args.strict_monotonic:
        if report["diagnostics"]["lidar_timebase_non_monotonic_count"]:
            failures.append("LiDAR timebase contains duplicate/backward timestamps")
        if report["diagnostics"]["imu_timestamp_non_monotonic_count"]:
            failures.append("IMU timestamps contain duplicate/backward timestamps")

    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(json.dumps(report, indent=2))
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("PASS: bag timestamp contract checks completed.")
    print(
        "NOTE: nearest LiDAR/IMU sample delta is descriptive only; it does not "
        "prove hardware PTP synchronization or calibrated sensor clock offset."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
