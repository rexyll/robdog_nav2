"""ROS 2 publisher for read-only host Chrony synchronization evidence."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robodog_gnss_interfaces.msg import HostTimeSyncStatus

from robodog_gnss_time.chrony_monitor import ChronyHostTimeMonitor, ChronyMonitorConfig


class HostTimeSyncNode(Node):
    def __init__(self):
        super().__init__("host_time_sync_monitor")
        source_id = self.declare_parameter("source_id", "").value
        max_error_ns = self.declare_parameter("max_host_utc_error_ns", 0).value
        poll_period_s = self.declare_parameter("poll_period_s", 1.0).value
        command_timeout_s = self.declare_parameter("command_timeout_s", 1.0).value
        if isinstance(poll_period_s, bool) or not isinstance(poll_period_s, (int, float)) or poll_period_s <= 0:
            raise ValueError("poll_period_s must be positive")
        if command_timeout_s >= poll_period_s:
            raise ValueError("command_timeout_s must be shorter than poll_period_s")
        self._monitor = ChronyHostTimeMonitor(ChronyMonitorConfig(
            source_id=source_id,
            max_host_utc_error_ns=max_error_ns,
            command_timeout_s=command_timeout_s,
        ))
        topic = self.declare_parameter("status_topic", "/gnss/host_time_sync").value
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError("status_topic must be a non-empty string")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=4,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(HostTimeSyncStatus, topic, qos)
        self._timer = self.create_timer(float(poll_period_s), self._publish_sample)
        self._publish_sample()

    def _publish_sample(self):
        sample = self._monitor.sample()
        if not sample["host_boot_id"]:
            self.get_logger().error(
                "Cannot publish host clock evidence: Linux boot ID unavailable; "
                "downstream policy remains fail-closed"
            )
            return
        message = HostTimeSyncStatus()
        message.state = {
            "unknown": HostTimeSyncStatus.STATE_UNKNOWN,
            "synchronized": HostTimeSyncStatus.STATE_SYNCHRONIZED,
            "unsynchronized": HostTimeSyncStatus.STATE_UNSYNCHRONIZED,
            "fault": HostTimeSyncStatus.STATE_FAULT,
        }[sample["state"]]
        message.source_id = sample["source_id"]
        message.host_boot_id = sample["host_boot_id"]
        message.utc_offset_ns = sample["utc_offset_ns"]
        message.uncertainty_ns = sample["uncertainty_ns"]
        message.sample_monotonic_ns = sample["sample_monotonic_ns"]
        message.sample_sequence = sample["sample_sequence"]
        message.reference_id = sample["reference_id"]
        message.stratum = sample["stratum"]
        message.leap_status = sample["leap_status"]
        self._publisher.publish(message)
        if sample["state"] != "synchronized":
            self.get_logger().warning(
                f"Host clock evidence is {sample['state']}: "
                f"{sample['fault'] or sample['leap_status']}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = HostTimeSyncNode()
        rclpy.spin(node)
    except Exception as exc:
        rclpy.logging.get_logger("host_time_sync_monitor").fatal(
            f"Host time monitor failed: {type(exc).__name__}: {exc}"
        )
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
