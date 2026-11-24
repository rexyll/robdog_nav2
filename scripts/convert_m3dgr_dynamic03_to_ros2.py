#!/usr/bin/env python3
"""Convert only the M3DGR MID-360 LiDAR + IMU streams to the pinned Jazzy types.

This is deliberately not a general rosbag converter. It requires:
1. an original ROS1 bag matching the upstream M3DGR Dynamic03 SHA256;
2. a successful read-only audit JSON for that exact bag;
3. the expected M3DGR legacy Livox ROS1 message type.

The source file is never modified. The output is created transactionally in a
new rosbag2 directory and contains only the two LIO input topics.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tempfile

import numpy as np
from importlib.metadata import version as package_version
from pathlib import Path
from typing import Any


LIDAR_TOPIC = "/livox/mid360/lidar"
IMU_TOPIC = "/livox/mid360/imu"
SOURCE_LIDAR_TYPES = {
    "livox_ros_driver/msg/CustomMsg",
    "livox_ros_driver/CustomMsg",
}
SOURCE_IMU_TYPES = {
    "sensor_msgs/msg/Imu",
    "sensor_msgs/Imu",
}
TARGET_LIDAR_TYPE = "livox_ros_driver2/msg/CustomMsg"
TARGET_IMU_TYPE = "sensor_msgs/msg/Imu"

# Exact field layout from the M3DGR ROS1 driver and pinned Livox ROS Driver2.
CUSTOM_POINT_MSG = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
CUSTOM_MSG_MSG = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
"""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def expected_sha256(manifest: Path, artifact_key: str) -> str:
    matches: list[str] = []
    for line in manifest.read_text(encoding="utf-8-sig").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        fields = stripped.split()
        if len(fields) == 2 and fields[0] == artifact_key:
            matches.append(fields[1].lower())
    if len(matches) != 1:
        raise ValueError(
            f"Expected exactly one SHA256 entry for {artifact_key!r} in {manifest}; "
            f"found {len(matches)}."
        )
    value = matches[0]
    if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise ValueError(f"Invalid SHA256 value for {artifact_key!r}.")
    return value


def validate_preflight(
    source_bag: Path,
    audit_report_path: Path,
    checksum_manifest: Path,
) -> tuple[dict[str, Any], str]:
    source_bag = source_bag.expanduser().resolve()
    audit_report_path = audit_report_path.expanduser().resolve()
    checksum_manifest = checksum_manifest.expanduser().resolve()

    if not source_bag.is_file() or source_bag.suffix.lower() != ".bag":
        raise ValueError("Input must be the original ROS1 .bag file; DB3 is not accepted.")
    if not audit_report_path.is_file():
        raise FileNotFoundError(f"Read-only bag audit JSON not found: {audit_report_path}")
    if not checksum_manifest.is_file():
        raise FileNotFoundError(f"Checksum manifest not found: {checksum_manifest}")

    actual_sha = sha256_file(source_bag)
    expected = expected_sha256(checksum_manifest, "m3dgr/Dynamic03.bag")
    if actual_sha != expected:
        raise ValueError(
            "Input SHA256 does not match the pinned M3DGR Dynamic03 source. "
            f"Expected {expected}, got {actual_sha}."
        )

    report = json.loads(audit_report_path.read_text(encoding="utf-8"))
    if report.get("status") != "structural_inputs_present":
        raise ValueError(
            "Bag audit did not pass structural_inputs_present; refusing conversion. "
            f"status={report.get('status')!r}, issues={report.get('issues')!r}"
        )
    if Path(str(report.get("input_file", ""))).expanduser().resolve() != source_bag:
        raise ValueError("Audit JSON input_file does not match the supplied source bag.")
    if int(report.get("file_size_bytes", -1)) != source_bag.stat().st_size:
        raise ValueError("Audit JSON file_size_bytes does not match the source bag.")

    topics = report.get("topics", {})
    lidar = topics.get(LIDAR_TOPIC)
    imu = topics.get(IMU_TOPIC)
    if not isinstance(lidar, dict) or not isinstance(imu, dict):
        raise ValueError("Audit JSON does not contain both required MID-360 topics.")
    lidar_types = set(lidar.get("message_types", []))
    imu_types = set(imu.get("message_types", []))
    if len(lidar_types) != 1 or not lidar_types.issubset(SOURCE_LIDAR_TYPES):
        raise ValueError(f"Unsupported ROS1 LiDAR source type(s): {sorted(lidar_types)}")
    if len(imu_types) != 1 or not imu_types.issubset(SOURCE_IMU_TYPES):
        raise ValueError(f"Unsupported ROS1 IMU source type(s): {sorted(imu_types)}")

    samples = report.get("sample_deserialization", {})
    lidar_samples = samples.get(LIDAR_TOPIC, {})
    imu_samples = samples.get(IMU_TOPIC, {})
    if int(lidar_samples.get("decoded", 0)) < 1 or lidar_samples.get("errors"):
        raise ValueError("LiDAR sample deserialization did not pass in the audit report.")
    if int(imu_samples.get("decoded", 0)) < 1 or imu_samples.get("errors"):
        raise ValueError("IMU sample deserialization did not pass in the audit report.")
    for sample in lidar_samples.get("samples", []):
        if sample.get("point_num_matches_decoded_count") is False:
            raise ValueError("Audit report contains a Livox point_num/points length mismatch.")
        if sample.get("header_timebase_delta_ns") is not None and abs(
            int(sample["header_timebase_delta_ns"])
        ) > 2:
            raise ValueError("Audit report contains a header.stamp/timebase mismatch > 2 ns.")

    return report, actual_sha


def _make_target_header(source_header: Any, typestore: Any) -> Any:
    stamp = source_header.stamp
    sec = int(getattr(stamp, "sec"))
    nanosec = int(getattr(stamp, "nanosec", getattr(stamp, "nsec", 0)))
    time_type = typestore.types["builtin_interfaces/msg/Time"]
    header_type = typestore.types["std_msgs/msg/Header"]
    return header_type(
        stamp=time_type(sec=sec, nanosec=nanosec),
        frame_id=str(source_header.frame_id),
    )


def convert_custom_msg(source_message: Any, typestore: Any) -> Any:
    """Map old livox_ros_driver/CustomMsg to livox_ros_driver2/CustomMsg."""
    target_point_type = typestore.types["livox_ros_driver2/msg/CustomPoint"]
    target_msg_type = typestore.types[TARGET_LIDAR_TYPE]

    points = [
        target_point_type(
            offset_time=int(point.offset_time),
            x=float(point.x),
            y=float(point.y),
            z=float(point.z),
            reflectivity=int(point.reflectivity),
            tag=int(point.tag),
            line=int(point.line),
        )
        for point in source_message.points
    ]
    if int(source_message.point_num) != len(points):
        raise ValueError(
            f"Livox point_num={source_message.point_num} but decoded points={len(points)}."
        )

    rsvd = np.asarray([int(value) for value in source_message.rsvd], dtype=np.uint8)
    if len(rsvd) != 3:
        raise ValueError(f"Livox rsvd field must contain 3 bytes, got {len(rsvd)}.")

    return target_msg_type(
        header=_make_target_header(source_message.header, typestore),
        timebase=int(source_message.timebase),
        point_num=int(source_message.point_num),
        lidar_id=int(source_message.lidar_id),
        rsvd=rsvd,
        points=points,
    )


def convert_imu(source_message: Any, typestore: Any) -> Any:
    """Rebuild sensor_msgs/Imu using the ROS2 Jazzy typestore."""
    quaternion_type = typestore.types["geometry_msgs/msg/Quaternion"]
    vector3_type = typestore.types["geometry_msgs/msg/Vector3"]
    imu_type = typestore.types[TARGET_IMU_TYPE]

    orientation = source_message.orientation
    angular_velocity = source_message.angular_velocity
    linear_acceleration = source_message.linear_acceleration

    return imu_type(
        header=_make_target_header(source_message.header, typestore),
        orientation=quaternion_type(
            x=float(orientation.x),
            y=float(orientation.y),
            z=float(orientation.z),
            w=float(orientation.w),
        ),
        orientation_covariance=np.asarray(source_message.orientation_covariance, dtype=np.float64),
        angular_velocity=vector3_type(
            x=float(angular_velocity.x),
            y=float(angular_velocity.y),
            z=float(angular_velocity.z),
        ),
        angular_velocity_covariance=np.asarray(
            [float(value) for value in source_message.angular_velocity_covariance],
            dtype=np.float64,
        ),
        linear_acceleration=vector3_type(
            x=float(linear_acceleration.x),
            y=float(linear_acceleration.y),
            z=float(linear_acceleration.z),
        ),
        linear_acceleration_covariance=np.asarray(
            [float(value) for value in source_message.linear_acceleration_covariance],
            dtype=np.float64,
        ),
    )


def convert_m3dgr_lio_streams(
    source_bag: Path,
    output_bag: Path,
    audit_report_path: Path,
    checksum_manifest: Path,
) -> dict[str, Any]:
    """Convert the two LIO topics; preserve original bag and all source timestamps."""
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    source_bag = source_bag.expanduser().resolve()
    output_bag = output_bag.expanduser().resolve()
    audit_report_path = audit_report_path.expanduser().resolve()
    checksum_manifest = checksum_manifest.expanduser().resolve()

    if output_bag.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_bag}")
    if output_bag == source_bag or source_bag in output_bag.parents:
        raise ValueError("Output path must not be the source bag or a child of its path.")

    audit_report, source_sha = validate_preflight(
        source_bag, audit_report_path, checksum_manifest
    )

    destination_store = get_typestore(Stores.ROS2_JAZZY)
    destination_store.register(
        get_types_from_msg(CUSTOM_POINT_MSG, "livox_ros_driver2/msg/CustomPoint")
    )
    destination_store.register(
        get_types_from_msg(CUSTOM_MSG_MSG, TARGET_LIDAR_TYPE)
    )

    output_bag.parent.mkdir(parents=True, exist_ok=True)
    staging_root = Path(
        tempfile.mkdtemp(prefix=".m3dgr-dynamic03-convert-", dir=output_bag.parent)
    )
    staged_bag = staging_root / "ros2_lio_input"

    counts = {LIDAR_TOPIC: 0, IMU_TOPIC: 0}
    first_output_ns: int | None = None
    last_output_ns: int | None = None
    try:
        with AnyReader([source_bag]) as reader, Writer(staged_bag, version=8) as writer:
            source_connections = [
                connection
                for connection in reader.connections
                if connection.topic in {LIDAR_TOPIC, IMU_TOPIC}
            ]
            topic_types: dict[str, set[str]] = {}
            for connection in source_connections:
                topic_types.setdefault(connection.topic, set()).add(connection.msgtype)
            if topic_types.get(LIDAR_TOPIC, set()).issubset(SOURCE_LIDAR_TYPES) is False:
                raise ValueError("Source LiDAR connection type changed after preflight.")
            if topic_types.get(IMU_TOPIC, set()).issubset(SOURCE_IMU_TYPES) is False:
                raise ValueError("Source IMU connection type changed after preflight.")

            lidar_connection = writer.add_connection(
                LIDAR_TOPIC,
                TARGET_LIDAR_TYPE,
                typestore=destination_store,
            )
            imu_connection = writer.add_connection(
                IMU_TOPIC,
                TARGET_IMU_TYPE,
                typestore=destination_store,
            )

            for connection, timestamp_ns, rawdata in reader.messages(connections=source_connections):
                if connection.topic == LIDAR_TOPIC:
                    source_message = reader.deserialize(rawdata, connection.msgtype)
                    target_message = convert_custom_msg(source_message, destination_store)
                    serialized = destination_store.serialize_cdr(
                        target_message, TARGET_LIDAR_TYPE
                    )
                    writer.write(lidar_connection, int(timestamp_ns), serialized)
                    counts[LIDAR_TOPIC] += 1
                elif connection.topic == IMU_TOPIC:
                    source_message = reader.deserialize(rawdata, connection.msgtype)
                    target_message = convert_imu(source_message, destination_store)
                    serialized = destination_store.serialize_cdr(
                        target_message, TARGET_IMU_TYPE
                    )
                    writer.write(imu_connection, int(timestamp_ns), serialized)
                    counts[IMU_TOPIC] += 1
                else:
                    continue

                first_output_ns = (
                    int(timestamp_ns)
                    if first_output_ns is None
                    else min(first_output_ns, int(timestamp_ns))
                )
                last_output_ns = (
                    int(timestamp_ns)
                    if last_output_ns is None
                    else max(last_output_ns, int(timestamp_ns))
                )

        if counts[LIDAR_TOPIC] == 0 or counts[IMU_TOPIC] == 0:
            raise ValueError(f"Converted output is missing a required stream: {counts}")

        manifest = {
            "schema_version": 1,
            "status": "converted_lio_topics_only",
            "source_file": str(source_bag),
            "source_size_bytes": source_bag.stat().st_size,
            "source_sha256": source_sha,
            "source_audit_report": str(audit_report_path),
            "source_audit_status": audit_report["status"],
            "source_dataset": "M3DGR Dynamic03",
            "source_format": "ROS1",
            "target_format": "ROS2",
            "rosbags_version": package_version("rosbags"),
            "rosbag2_version": 8,
            "mapping": {
                LIDAR_TOPIC: {
                    "source_type": "livox_ros_driver/msg/CustomMsg",
                    "target_type": TARGET_LIDAR_TYPE,
                    "field_mapping": "header,timebase,point_num,lidar_id,rsvd,points[] copied field-by-field",
                    "timebase_unit": "ns",
                    "point_offset_unit": "ns",
                },
                IMU_TOPIC: {
                    "source_type": "sensor_msgs/msg/Imu",
                    "target_type": TARGET_IMU_TYPE,
                    "field_mapping": "header,orientation,covariances,angular_velocity,linear_acceleration copied",
                },
            },
            "message_counts": counts,
            "first_output_timestamp_ns": first_output_ns,
            "last_output_timestamp_ns": last_output_ns,
            "limitations": [
                "Only MID-360 LiDAR and MID-360 built-in IMU topics were converted.",
                "All other source topics, including GNSS and camera streams, were intentionally omitted.",
                "Message conversion does not prove PTP/PPS clock synchronization or GT time alignment.",
                "The output is a ROS2 bag storage conversion, not Point-LIO algorithm validation.",
            ],
        }
        (staged_bag / "conversion_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        if output_bag.exists():
            raise FileExistsError(f"Refusing to overwrite existing output: {output_bag}")
        os.replace(staged_bag, output_bag)
        return manifest
    finally:
        shutil.rmtree(staging_root, ignore_errors=True)


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Convert verified M3DGR Dynamic03 MID-360 LIO topics to ROS2 Jazzy types."
    )
    parser.add_argument("source_bag", type=Path, help="Original Dynamic03 ROS1 .bag file")
    parser.add_argument("audit_report", type=Path, help="Successful JSON output from bag audit")
    parser.add_argument(
        "output_bag",
        type=Path,
        help="New ROS2 output directory; must not already exist",
    )
    parser.add_argument(
        "--checksum-manifest",
        type=Path,
        default=repo_root / "datasets" / "external_sha256.txt",
        help="Pinned upstream checksum manifest",
    )
    args = parser.parse_args()

    try:
        result = convert_m3dgr_lio_streams(
            args.source_bag,
            args.output_bag,
            args.audit_report,
            args.checksum_manifest,
        )
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
