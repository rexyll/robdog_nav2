"""Read-only freshness monitor for the public Lite3 ROS 2 state topics."""

import time
from typing import Dict, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from robodog_lite3_bridge.health_state import StreamState, classify_stream


class TopicContractMonitor(Node):
    """Reports whether configured Lite3 state streams are arriving.

    This node does not connect to the vendor SDK and does not publish motion
    commands. It only observes standard ROS 2 topics and publishes diagnostics.
    """

    def __init__(self) -> None:
        super().__init__("lite3_topic_contract_monitor")

        self.declare_parameter("leg_odom_topic", "/leg_odom2")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("diagnostics_topic", "/diagnostics/robodog_lite3")
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("publish_period_sec", 0.2)

        self._timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        publish_period = float(self.get_parameter("publish_period_sec").value)
        if self._timeout_sec <= 0.0:
            raise ValueError("stale_timeout_sec must be greater than zero")
        if publish_period <= 0.0:
            raise ValueError("publish_period_sec must be greater than zero")

        self._streams: Dict[str, Dict[str, Optional[float]]] = {
            "leg_odom": {
                "topic": str(self.get_parameter("leg_odom_topic").value),
                "last_rx": None,
            },
            "imu": {
                "topic": str(self.get_parameter("imu_topic").value),
                "last_rx": None,
            },
            "joint_states": {
                "topic": str(self.get_parameter("joint_states_topic").value),
                "last_rx": None,
            },
        }

        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

        self._subscriptions = [
            self.create_subscription(
                Odometry,
                self._streams["leg_odom"]["topic"],
                lambda _msg: self._record_receive("leg_odom"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Imu,
                self._streams["imu"]["topic"],
                lambda _msg: self._record_receive("imu"),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                JointState,
                self._streams["joint_states"]["topic"],
                lambda _msg: self._record_receive("joint_states"),
                qos_profile_sensor_data,
            ),
        ]

        self._timer = self.create_timer(publish_period, self._publish_diagnostics)
        self.get_logger().info(
            "Monitoring Lite3 state topics in read-only mode; no cmd_vel publisher is created."
        )

    def _record_receive(self, stream_name: str) -> None:
        self._streams[stream_name]["last_rx"] = time.monotonic()

    def _publish_diagnostics(self) -> None:
        now_monotonic = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        all_fresh = True
        for stream_name, stream in self._streams.items():
            last_rx = stream["last_rx"]
            state = classify_stream(last_rx, now_monotonic, self._timeout_sec)

            status = DiagnosticStatus()
            status.name = "robodog_lite3/" + stream_name
            status.hardware_id = "Lite3 ROS 2 state interface"
            status.values = [
                KeyValue(key="topic", value=str(stream["topic"])),
                KeyValue(key="stream_state", value=state.value),
                KeyValue(key="freshness_timeout_sec", value=str(self._timeout_sec)),
            ]

            if state == StreamState.NOT_RECEIVED:
                status.level = DiagnosticStatus.WARN
                status.message = "No message received yet"
                all_fresh = False
            elif state == StreamState.STALE:
                status.level = DiagnosticStatus.ERROR
                age = now_monotonic - float(last_rx)
                status.message = "Stream is stale"
                status.values.append(KeyValue(key="age_sec", value=f"{age:.3f}"))
                all_fresh = False
            else:
                age = now_monotonic - float(last_rx)
                status.level = DiagnosticStatus.OK
                status.message = "Stream is fresh"
                status.values.append(KeyValue(key="age_sec", value=f"{age:.3f}"))

            msg.status.append(status)

        summary = DiagnosticStatus()
        summary.name = "robodog_lite3/read_only_contract"
        summary.hardware_id = "Lite3 ROS 2 state interface"
        if all_fresh:
            summary.level = DiagnosticStatus.OK
            summary.message = "All configured state topics are fresh"
        else:
            summary.level = DiagnosticStatus.WARN
            summary.message = "One or more configured state topics are missing or stale"
        msg.status.append(summary)
        self._diagnostics_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicContractMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
