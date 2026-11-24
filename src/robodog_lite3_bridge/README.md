# robodog_lite3_bridge (read-only scaffold)

This initial package intentionally does not connect to the Lite3 vendor SDK and does not publish a motion command.

## Included nodes

- lite3_state_mock: publishes neutral synthetic /mock/lite3/leg_odom2, /mock/lite3/imu/data, and /mock/lite3/joint_states messages. Odometry carries very large covariances, IMU orientation is marked unavailable, and JointState arrays are empty. All synthetic topics use the /mock/lite3 namespace and mock_* frame IDs so they cannot shadow real robot topics. This is only for topic/type and launch integration tests.
- lite3_topic_contract_monitor: subscribes to those three mock state topics and publishes diagnostic_msgs/DiagnosticArray on /diagnostics/robodog_lite3. Freshness uses host monotonic receive time and a configurable timeout.
- telemetry_contract.py: transport-agnostic validation for a proposed versioned gateway envelope, session/sequence tracking, and finite vector values. It does not implement a wire protocol or network I/O.

## Build

    source /opt/ros/humble/setup.bash  # or jazzy
    cd <workspace-root>
    colcon build --packages-select robodog_lite3_bridge
    source install/setup.bash

## Mock launch

    ros2 launch robodog_lite3_bridge mock_state.launch.py
    ros2 topic echo /diagnostics/robodog_lite3

## Unit tests

The pytest suite contains 25 tests and covers freshness boundaries, invalid timeout values, envelope version/metadata validation, source timestamp/clock pairing, session resets, packet gaps/duplicates/reordering, NaN/Inf rejection, and vector lengths. CI run 45328657069 passed on both ROS 2 Humble and Jazzy (25/25 each).

## Important safety boundary

Do not connect this mock publisher to robot localization or motion control. Do not treat the synthetic topic stream as a real robot state. The monitor is read-only and creates no motion publisher. Real SDK/UDP integration, command gating, physical E-stop, and hardware watchdog are separate tasks that require the target Lite3 firmware and vendor documentation.
