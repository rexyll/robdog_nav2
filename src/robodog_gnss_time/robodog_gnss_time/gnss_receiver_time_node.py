"""Single-threaded ROS 2 subscriber for ordered raw GNSS ingress events."""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.logging import get_logger
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robodog_gnss_interfaces.msg import GnssIngressEvent, GnssObservation, GnssReceiverTime
from robodog_gnss_quality.ubx_ingress_event_processor import GnssIngressEventProcessor
from robodog_gnss_quality.ubx_time_mapper import UbxReceiverTimeMapper
from robodog_gnss_quality.ubx_raw_nav_epoch_assembler import UbxRawNavEpochAssembler

from .ingress_consumer import GnssIngressConsumer
from .receiver_time_message_adapter import populate_receiver_time_message
from .observation_message_adapter import populate_gnss_observation_message


class GnssReceiverTimeNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("gnss_receiver_time", parameter_overrides=parameter_overrides)
        ingress_topic = self.declare_parameter("ingress_topic", "/gnss/ingress").value
        mapping_topic = self.declare_parameter("mapping_topic", "/gnss/receiver_time").value
        observation_topic = self.declare_parameter("observation_topic", "/gnss/observation").value
        qos_depth = self.declare_parameter("qos_depth", 256).value
        max_pending_epochs = self.declare_parameter("max_pending_epochs", 64).value
        max_pair_receive_skew_ns = self.declare_parameter(
            "max_pair_receive_skew_ns", 250_000_000
        ).value
        max_pending_age_ns = self.declare_parameter(
            "max_pending_age_ns", 1_000_000_000
        ).value
        trusted_sources = self.declare_parameter(
            "trusted_leap_sources", [],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY),
        ).value
        if not isinstance(ingress_topic, str) or not ingress_topic.strip():
            raise ValueError("ingress_topic must be a non-empty string")
        if not isinstance(mapping_topic, str) or not mapping_topic.strip():
            raise ValueError("mapping_topic must be a non-empty string")
        if not isinstance(observation_topic, str) or not observation_topic.strip():
            raise ValueError("observation_topic must be a non-empty string")
        if isinstance(qos_depth, bool) or not isinstance(qos_depth, int) or not 1 <= qos_depth <= 65536:
            raise ValueError("qos_depth must be in [1, 65536]")
        for name, value, minimum, maximum in (
            ("max_pending_epochs", max_pending_epochs, 1, 4096),
            ("max_pair_receive_skew_ns", max_pair_receive_skew_ns, 1, (1 << 63) - 1),
            ("max_pending_age_ns", max_pending_age_ns, 1, (1 << 63) - 1),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
                raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
        if not isinstance(trusted_sources, (list, tuple)):
            raise ValueError("trusted_leap_sources must be an integer array")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(GnssReceiverTime, mapping_topic, qos)
        self._observation_publisher = self.create_publisher(
            GnssObservation, observation_topic, qos
        )
        mapper = UbxReceiverTimeMapper(
            trusted_leap_sources=set(trusted_sources),
            require_ingress_sequence=True,
        )
        processor = GnssIngressEventProcessor(mapper)
        navigation_assembler = UbxRawNavEpochAssembler(
            max_pending_epochs=max_pending_epochs,
            max_pair_receive_skew_ns=max_pair_receive_skew_ns,
            max_pending_age_ns=max_pending_age_ns,
        )
        self._consumer = GnssIngressConsumer(
            processor,
            self._publish_mapping,
            navigation_assembler=navigation_assembler,
            publish_observation=self._publish_observation,
        )
        self._subscription = self.create_subscription(
            GnssIngressEvent, ingress_topic, self._on_ingress, qos
        )
        self.get_logger().info(
            f"GNSS ingress consumer ready: ingress={ingress_topic} mapping={mapping_topic} "
            f"observation={observation_topic} trusted_leap_sources={sorted(set(trusted_sources))} "
            f"max_pending_epochs={max_pending_epochs} "
            f"max_pair_receive_skew_ns={max_pair_receive_skew_ns} "
            f"max_pending_age_ns={max_pending_age_ns}"
        )

    def _publish_mapping(self, mapping):
        message = populate_receiver_time_message(mapping, GnssReceiverTime())
        self._publisher.publish(message)

    def _publish_observation(self, result):
        message = populate_gnss_observation_message(result, GnssObservation())
        self._observation_publisher.publish(message)

    def _on_ingress(self, message):
        if self._consumer.faulted:
            return
        try:
            self._consumer.handle_ros_message(message)
        except Exception as exc:
            self.get_logger().fatal(
                f"GNSS ingress stream rejected; consumer latched fail-closed: {type(exc).__name__}: {exc}"
            )
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    result = 0
    try:
        node = GnssReceiverTimeNode()
        executor.add_node(node)
        executor.spin()
        if node._consumer.faulted:
            result = 2
    except Exception as exc:
        if node is not None and node._consumer.faulted:
            result = 2
        get_logger("gnss_receiver_time").fatal(
            f"GNSS receiver-time node failed: {type(exc).__name__}: {exc}"
        )
        result = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
