"""Synthetic Lite3 state topics for integration tests only."""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


class MockLite3StatePublisher(Node):
    """Publishes neutral synthetic state messages; never commands robot motion."""

    def __init__(self) -> None:
        super().__init__("mock_lite3_state_publisher")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("leg_odom_topic", "/leg_odom2")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_states_topic", "/joint_states")

        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("leg_odom_topic").value), 10
        )
        self._imu_pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), 10
        )
        self._joint_pub = self.create_publisher(
            JointState, str(self.get_parameter("joint_states_topic").value), 10
        )
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warning(
            "Publishing SYNTHETIC neutral Lite3 state. Do not use this node for live robot control."
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.orientation.w = 1.0
        # Large covariance marks synthetic pose/twist as unusable for real
        # localization. This publisher only tests topic integration.
        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = 1.0e6
            odom.twist.covariance[index] = 1.0e6
        self._odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation_covariance[0] = -1.0
        for index in (0, 4, 8):
            imu.angular_velocity_covariance[index] = 1.0e6
            imu.linear_acceleration_covariance[index] = 1.0e6
        self._imu_pub.publish(imu)

        # Empty arrays intentionally verify topic availability/type only;
        # no guessed Lite3 joint names or positions are generated.
        joint_states = JointState()
        joint_states.header.stamp = stamp
        self._joint_pub.publish(joint_states)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLite3StatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
