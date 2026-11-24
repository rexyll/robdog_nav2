#!/usr/bin/env python3
"""Record Point-LIO nav_msgs/Odometry poses to a TUM trajectory file."""
from __future__ import annotations

import argparse
import signal

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class OdometryRecorder(Node):
    def __init__(self, topic: str, output_path: str) -> None:
        super().__init__("robodog_point_lio_tum_recorder")
        self._file = open(output_path, "w", encoding="utf-8")
        self._count = 0
        self._duplicates = 0
        self._backwards = 0
        self._last_stamp_ns: int | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
        )
        self._subscription = self.create_subscription(
            Odometry, topic, self._on_odometry, qos
        )

    def _on_odometry(self, msg: Odometry) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if self._last_stamp_ns is not None:
            if stamp_ns == self._last_stamp_ns:
                self._duplicates += 1
            elif stamp_ns < self._last_stamp_ns:
                self._backwards += 1
        self._last_stamp_ns = stamp_ns

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._file.write(
            f"{stamp_ns / 1e9:.9f} "
            f"{position.x:.6f} {position.y:.6f} {position.z:.6f} "
            f"{orientation.x:.9f} {orientation.y:.9f} "
            f"{orientation.z:.9f} {orientation.w:.9f}\n"
        )
        self._count += 1
        if self._count % 200 == 0:
            self._file.flush()

    @property
    def count(self) -> int:
        return self._count

    def close(self) -> None:
        self._file.flush()
        self._file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/aft_mapped_to_init")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = OdometryRecorder(args.topic, args.out)

    def stop(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"poses={node.count}, duplicate_stamps={node._duplicates}, "
            f"backwards_stamps={node._backwards}"
        )
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
