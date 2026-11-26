"""Single-threaded ROS 2 node joining four UBX navigation streams to receiver time."""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robodog_gnss_interfaces.msg import (
    GnssNavCov,
    GnssNavHpPosLlh,
    GnssNavPvt,
    GnssNavStatus,
    GnssObservation,
    GnssReceiverTime,
)

from .observation_message_adapter import populate_gnss_observation_message
from .ubx_ros_epoch_runtime import UbxRosEpochRuntime


class GnssObservationNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("gnss_observation", parameter_overrides=parameter_overrides)

        topic_defaults = {
            "nav_pvt_topic": "/gnss/nav_pvt",
            "nav_hp_pos_llh_topic": "/gnss/nav_hp_pos_llh",
            "nav_cov_topic": "/gnss/nav_cov",
            "nav_status_topic": "/gnss/nav_status",
            "receiver_time_topic": "/gnss/receiver_time",
            "observation_topic": "/gnss/observation",
        }
        topics = {
            name: self.declare_parameter(name, default).value
            for name, default in topic_defaults.items()
        }
        for name, value in topics.items():
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")

        qos_depth = self.declare_parameter("qos_depth", 256).value
        max_pending_epochs = self.declare_parameter("max_pending_epochs", 64).value
        max_pair_receive_skew_ns = self.declare_parameter(
            "max_pair_receive_skew_ns", 250_000_000
        ).value
        max_pending_age_ns = self.declare_parameter(
            "max_pending_age_ns", 1_000_000_000
        ).value
        for name, value, minimum, maximum in (
            ("qos_depth", qos_depth, 1, 65536),
            ("max_pending_epochs", max_pending_epochs, 1, 4096),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns, 1, (1 << 63) - 1),
            ("max_pending_age_ns", max_pending_age_ns, 1, (1 << 63) - 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            GnssObservation, topics["observation_topic"], qos
        )
        self._fault = None
        self._runtime = UbxRosEpochRuntime(
            max_pending_epochs=max_pending_epochs,
            max_pair_receive_skew_ns=max_pair_receive_skew_ns,
            max_pending_age_ns=max_pending_age_ns,
            publish_observation=self._publish_observation,
        )
        self._subscriptions = [
            self.create_subscription(GnssNavPvt, topics["nav_pvt_topic"], self._on_pvt, qos),
            self.create_subscription(
                GnssNavHpPosLlh, topics["nav_hp_pos_llh_topic"], self._on_position, qos
            ),
            self.create_subscription(GnssNavCov, topics["nav_cov_topic"], self._on_covariance, qos),
            self.create_subscription(
                GnssNavStatus, topics["nav_status_topic"], self._on_status, qos
            ),
            self.create_subscription(
                GnssReceiverTime, topics["receiver_time_topic"], self._on_receiver_time, qos
            ),
        ]
        self._expiry_timer = self.create_timer(0.1, self._expire_stale)
        self.get_logger().info(
            "GNSS observation joiner ready: "
            + " ".join(f"{key}={value}" for key, value in topics.items())
        )

    @property
    def faulted(self):
        return self._fault is not None

    def _publish_observation(self, result):
        message = populate_gnss_observation_message(result, GnssObservation())
        self._publisher.publish(message)

    def _dispatch(self, callback, message):
        if self._fault is not None:
            return
        try:
            callback(message)
        except Exception as exc:
            self._fault = exc
            self.get_logger().fatal(
                "GNSS observation input rejected; runtime latched fail-closed: "
                f"{type(exc).__name__}: {exc}"
            )
            if rclpy.ok():
                rclpy.shutdown()

    def _on_pvt(self, message):
        self._dispatch(self._runtime.handle_nav_pvt, message)

    def _on_position(self, message):
        self._dispatch(self._runtime.handle_nav_hp_pos_llh, message)

    def _on_covariance(self, message):
        self._dispatch(self._runtime.handle_nav_cov, message)

    def _on_status(self, message):
        self._dispatch(self._runtime.handle_nav_status, message)

    def _on_receiver_time(self, message):
        self._dispatch(self._runtime.handle_receiver_time, message)

    def _expire_stale(self):
        if self._fault is not None:
            return
        try:
            self._runtime.expire_stale()
        except Exception as exc:
            self._fault = exc
            self.get_logger().fatal(
                f"GNSS epoch expiry failed; runtime latched fail-closed: {type(exc).__name__}: {exc}"
            )
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    result = 0
    try:
        node = GnssObservationNode()
        executor.add_node(node)
        executor.spin()
        if node.faulted:
            result = 2
    except Exception as exc:
        if node is not None and node.faulted:
            result = 2
        get_logger("gnss_observation").fatal(
            f"GNSS observation node failed: {type(exc).__name__}: {exc}"
        )
        result = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
