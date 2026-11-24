"""Launch synthetic state topics and the read-only contract monitor."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robodog_lite3_bridge",
                executable="lite3_state_mock",
                name="mock_lite3_state_publisher",
                output="screen",
            ),
            Node(
                package="robodog_lite3_bridge",
                executable="lite3_topic_contract_monitor",
                name="lite3_topic_contract_monitor",
                output="screen",
                parameters=[
                    {
                        "leg_odom_topic": "/leg_odom2",
                        "imu_topic": "/imu/data",
                        "joint_states_topic": "/joint_states",
                        "diagnostics_topic": "/diagnostics/robodog_lite3",
                        "stale_timeout_sec": 1.0,
                    }
                ],
            ),
        ]
    )
