"""ROS 2 adapter for the transport-independent RTK quality gate."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robodog_gnss_interfaces.msg import GnssObservation, GnssQualityStatus

from robodog_gnss_quality.quality_gate import (
    CarrierSolution,
    CovarianceType,
    GnssObservation as CoreObservation,
    GateConfig,
    QualityDecision,
    RejectionReason,
    RtkQualityGate,
)


def _integer(name, value, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")
    return value


def observation_from_ros(message):
    """Convert the ROS contract without inferring missing values or clock domains."""
    if not isinstance(message.source_clock_id, str) or not message.source_clock_id.strip():
        raise ValueError("source_clock_id must be a non-empty string")
    if not isinstance(message.process_session_id, str) or not message.process_session_id.strip():
        raise ValueError("process_session_id must be a non-empty string")
    if not isinstance(message.gnss_fix_ok, bool):
        raise ValueError("gnss_fix_ok must be bool")
    carrier_code = _integer("carrier_solution", message.carrier_solution, 0, 3)
    covariance_code = _integer("covariance_type", message.covariance_type, 0, 3)
    covariance = tuple(message.position_covariance)
    if len(covariance) != 9:
        raise ValueError("position_covariance must contain exactly 9 values")
    return CoreObservation(
        source_timestamp_ns=_integer("source_timestamp_ns", message.source_timestamp_ns, 0, (1 << 64) - 1),
        source_clock_id=message.source_clock_id,
        receive_monotonic_ns=_integer("receive_monotonic_ns", message.receive_monotonic_ns, 0, (1 << 64) - 1),
        latitude_deg=message.latitude_deg,
        longitude_deg=message.longitude_deg,
        altitude_m=message.altitude_m,
        navsat_status=_integer("navsat_status", message.navsat_status, -1, 2),
        gnss_fix_ok=message.gnss_fix_ok,
        carrier_solution={
            0: CarrierSolution.NO_SOLUTION,
            1: CarrierSolution.FLOAT,
            2: CarrierSolution.FIXED,
            3: CarrierSolution.UNKNOWN,
        }[carrier_code],
        position_covariance=covariance,
        covariance_type=CovarianceType(covariance_code),
    ), (
        message.process_session_id,
        _integer("connection_generation", message.connection_generation, 0, (1 << 64) - 1),
        _integer("gps_week", message.gps_week, 0, 32767),
        _integer("i_tow_ms", message.i_tow_ms, 0, 604799999),
    )


class GnssRtkQualityNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__("gnss_rtk_quality", parameter_overrides=parameter_overrides)
        observation_topic = self.declare_parameter("observation_topic", "/gnss/observation").value
        status_topic = self.declare_parameter("status_topic", "/gnss/quality").value
        qos_depth = self.declare_parameter("qos_depth", 64).value
        max_fix_age_ns = self.declare_parameter("max_fix_age_ns", 200_000_000).value
        max_horizontal_std_m = self.declare_parameter("max_horizontal_std_m", 0.25).value
        max_vertical_std_m = self.declare_parameter("max_vertical_std_m", 0.5).value
        max_plausible_speed_mps = self.declare_parameter("max_plausible_speed_mps", 5.0).value
        required_consecutive_fixed_samples = self.declare_parameter(
            "required_consecutive_fixed_samples", 3
        ).value
        max_jump_reference_interval_ns = self.declare_parameter(
            "max_jump_reference_interval_ns", 2_000_000_000
        ).value
        allow_approximated_covariance = self.declare_parameter(
            "allow_approximated_covariance", False
        ).value

        for name, value in (("observation_topic", observation_topic), ("status_topic", status_topic)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if isinstance(qos_depth, bool) or not isinstance(qos_depth, int) or not 1 <= qos_depth <= 65536:
            raise ValueError("qos_depth must be an integer in [1, 65536]")
        for name, value in (
            ("max_fix_age_ns", max_fix_age_ns),
            ("max_jump_reference_interval_ns", max_jump_reference_interval_ns),
            ("required_consecutive_fixed_samples", required_consecutive_fixed_samples),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if not isinstance(allow_approximated_covariance, bool):
            raise ValueError("allow_approximated_covariance must be bool")

        self._gate = RtkQualityGate(GateConfig(
            max_fix_age_ns=max_fix_age_ns,
            max_horizontal_std_m=max_horizontal_std_m,
            max_vertical_std_m=max_vertical_std_m,
            max_plausible_speed_mps=max_plausible_speed_mps,
            required_consecutive_fixed_samples=required_consecutive_fixed_samples,
            max_jump_reference_interval_ns=max_jump_reference_interval_ns,
            allow_approximated_covariance=allow_approximated_covariance,
        ))
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(GnssQualityStatus, status_topic, qos)
        self._subscription = self.create_subscription(
            GnssObservation, observation_topic, self._on_observation, qos
        )
        self._active_context = None
        self._last_receive_ns = None
        self._last_observation = None
        self._last_decision_signature = None
        self._fault = None
        self._max_fix_age_ns = max_fix_age_ns
        self._watchdog = self.create_timer(
            max(0.001, min(max_fix_age_ns / 4_000_000_000.0, 0.1)), self._on_watchdog
        )
        initial = GnssQualityStatus()
        initial.qualified = False
        initial.reason = "no_data"
        self._publisher.publish(initial)

    @property
    def faulted(self):
        return self._fault is not None

    def _status(self, observation, context, decision):
        message = GnssQualityStatus()
        message.qualified = decision.qualified
        message.reason = decision.reason.value
        message.consecutive_fixed_samples = decision.consecutive_fixed_samples
        message.horizontal_std_valid = decision.horizontal_std_m is not None
        message.horizontal_std_m = decision.horizontal_std_m or 0.0
        message.vertical_std_valid = decision.vertical_std_m is not None
        message.vertical_std_m = decision.vertical_std_m or 0.0
        message.estimated_speed_valid = decision.estimated_speed_mps is not None
        message.estimated_speed_mps = decision.estimated_speed_mps or 0.0
        if observation is not None:
            message.i_tow_ms = context[3]
            message.gps_week = context[2]
            message.process_session_id = context[0]
            message.connection_generation = context[1]
            message.source_timestamp_ns = observation.source_timestamp_ns
            message.source_clock_id = observation.source_clock_id
            message.receive_monotonic_ns = observation.receive_monotonic_ns
        return message

    def _publish_decision(self, observation, context, decision):
        signature = (
            decision.qualified, decision.reason.value,
            decision.consecutive_fixed_samples,
            decision.horizontal_std_m, decision.vertical_std_m,
            decision.estimated_speed_mps,
        )
        if signature != self._last_decision_signature:
            self._publisher.publish(self._status(observation, context, decision))
            self._last_decision_signature = signature

    def _on_observation(self, message):
        if self._fault is not None:
            return
        try:
            observation, context = observation_from_ros(message)
            receive_ns = observation.receive_monotonic_ns
            if self._last_receive_ns is not None and receive_ns <= self._last_receive_ns:
                decision = QualityDecision(
                    qualified=False,
                    reason=RejectionReason.OUT_OF_ORDER,
                    consecutive_fixed_samples=self._gate.consecutive_fixed_samples,
                )
                self._publish_decision(observation, context, decision)
                return
            expected_clock_id = (
                f"{context[0]}:connection:{context[1]}:gps-week:{context[2]}"
            )
            if observation.source_clock_id != expected_clock_id:
                raise ValueError("source_clock_id does not match observation session/generation/GPS week")
            if self._active_context is not None:
                old_session, old_generation, old_week, _ = self._active_context
                if context[0] == old_session and context[1] < old_generation:
                    decision = QualityDecision(
                        qualified=False,
                        reason=RejectionReason.OUT_OF_ORDER,
                        consecutive_fixed_samples=self._gate.consecutive_fixed_samples,
                    )
                    self._publish_decision(observation, context, decision)
                    return
                if context[0] == old_session and context[1] == old_generation and context[2] < old_week:
                    decision = QualityDecision(
                        qualified=False,
                        reason=RejectionReason.OUT_OF_ORDER,
                        consecutive_fixed_samples=self._gate.consecutive_fixed_samples,
                    )
                    self._publish_decision(observation, context, decision)
                    return
            if context[:3] != (self._active_context[:3] if self._active_context else None):
                self._gate.reset()
                self._active_context = context
            self._last_receive_ns = receive_ns
            self._last_observation = (observation, context)
            decision = self._gate.evaluate(observation, time.monotonic_ns())
            self._publish_decision(observation, context, decision)
        except Exception as exc:
            self._fault = exc
            self.get_logger().fatal(
                f"GNSS quality input rejected; gate latched fail-closed: {type(exc).__name__}: {exc}"
            )
            if rclpy.ok():
                rclpy.shutdown()

    def _on_watchdog(self):
        if self._fault is not None or self._last_observation is None:
            return
        observation, context = self._last_observation
        now_ns = time.monotonic_ns()
        if now_ns - observation.receive_monotonic_ns <= self._max_fix_age_ns:
            return
        decision = self._gate.evaluate(observation, now_ns)
        self._publish_decision(observation, context, decision)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    result = 0
    try:
        node = GnssRtkQualityNode()
        executor.add_node(node)
        executor.spin()
        if node.faulted:
            result = 2
    except Exception as exc:
        get_logger("gnss_rtk_quality").fatal(
            f"GNSS RTK quality node failed: {type(exc).__name__}: {exc}"
        )
        result = 1
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
