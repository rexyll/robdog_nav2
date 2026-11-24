# Lite3 本体 ROS 2 接口契约与安全命令桥

- 状态：基于公开源码完成集成路线核验；真实固件和最终 motion-host 拓扑待云深处书面确认。
- 进度台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 厂商路径评估：[Lite3 vendor integration assessment](lite3_vendor_sdk_integration_assessment.md)
- 硬件 ADR：[ADR-0002](../decisions/ADR-0002-hardware-platform.md)
- 集成 ADR：[ADR-0003](../decisions/ADR-0003-lite3-ros-integration.md)

## 1. 重要修正：厂商有两套不同 ROS 路径

不能将 Legacy Lite3_ROS 与 Current sdk_deploy 当成同一个 ROS driver。旧版 transfer 发布标准 ROS 消息；新 SDK deploy 使用 DRDDS 自定义消息和独立 SDK state machine。详细证据见 vendor integration assessment。

## 2. Legacy Lite3_ROS（Foxy）接口

公开分支 ros2-foxy 的 transfer/Jetson2Motion 示例发布：

| Topic | 消息类型 | 说明 |
|---|---|---|
| /leg_odom | geometry_msgs/msg/PoseWithCovarianceStamped | 旧版腿式位姿输出 |
| /leg_odom2 | nav_msgs/msg/Odometry | 旧版位姿/速度输出 |
| /imu/data | sensor_msgs/msg/Imu | IMU |
| /joint_states | sensor_msgs/msg/JointState | 关节状态 |
| /cmd_vel | geometry_msgs/msg/Twist | 旧版命令入口 |

限制：源码使用 RCL_STEADY_TIME 生成 header.stamp；不得未经映射直接作为 ROS system/GNSS/source time。公开 /leg_odom2 片段未见 child_frame_id 与 covariance 显式赋值。必须逐项验证 frame、单位、符号、协方差、时间戳和 IMU/odom 数据相关性。

## 3. Current sdk_deploy 接口

新 SDK 的 lite3_transfer 对外接口为：

| Topic/Service | 类型 | 说明 |
|---|---|---|
| /IMU_DATA | drdds/msg/ImuData | DRDDS IMU 数据 |
| /JOINTS_DATA | drdds/msg/JointsData | DRDDS 关节数据 |
| /JOINTS_CMD | drdds/msg/JointsDataCmd | 低层关节命令；不是 Nav2 cmd_vel |
| /SDK_MODE | drdds/srv/StdSrvInt32 | SDK mode / 发布频率管理 |
| /EMERGENCY_STOP | drdds/srv/StdSrvInt32 | 厂商 SDK 急停调用链 |
| /EMERGENCY_STOP_SIGNAL | drdds/msg/StdMsgInt32 | 通知状态机的停止信号 |

DRDDS ImuData 包含 roll/pitch/yaw、角速度和线加速度；JointsData 包含 16 个 JointData 项。RobotStatus 公开定义为 10Hz，含 JointState、base_rpy、base_angular_velocity、command state、timeout 计数和 telemetry_fresh。但这些 topic 并不等价于 nav_msgs/Odometry。

如果没有可信腿式里程计/机身位置速度通道，不能只用关节状态或 IMU 积分伪造连续 odom。必须从厂商确认完整的状态估计输出，或设计经真值验证的状态估计。

## 4. rl_deploy 速度接口

新版高层策略部署示例提供 /rl_deploy/cmd_vel、/rl_deploy/command_state、/rl_deploy/effective_cmd_vel 和 /rl_deploy/status。command_state 包括 stand、rl_control、lie_down、damping；/rl_deploy/cmd_vel 只有在 rl_control 状态下有效。

厂商文档中 0.7m/s 前进、0.5m/s 侧移、0.7rad/s yaw 和约 0.25s 命令超时是示例默认行为，不是项目批准的导航速度或独立安全 watchdog。

**不得直接将 Nav2 cmd_vel remap 到 /rl_deploy/cmd_vel。** 必须先确认策略状态切换、命令限制、E-stop、watchdog 和停止路径；错误切换可能导致机器人突然移动。

## 5. Foxy / Jazzy 跨发行版约束

sdk_deploy cross-host 文档指出 Lite3 motion host 当前流程是 Ubuntu 20.04 / ROS 2 Foxy，并明确警告 Foxy 主机与非 Foxy host 的 ROS 2 通信可能不正常；还记录 Fast DDS 发现参与者混杂可能引起 CDR 反序列化错误。

本项目不假定 Foxy 与 Jazzy DDS 原生互通，不将其放在同一默认 DDS domain。当前候选架构是 vendor-compatible process/host 与 Nav2 Jazzy 隔离，通过经验证的非 DDS UDP/TCP gateway 输出标准 ROS 消息。此拓扑仍需云深处书面确认和实物验证。

## 6. 项目标准输出契约（仅在通过验证后发布）

| 项目 Topic | 类型 | 发布准入 |
|---|---|---|
| /robot/imu/data | sensor_msgs/msg/Imu | 单位、轴向、frame、source timestamp、covariance 核验 |
| /robot/joint_states | sensor_msgs/msg/JointState | joint names、顺序、单位、正方向与 URDF 核验 |
| /robot/odometry | nav_msgs/msg/Odometry | 取得可信腿式里程计/机身速度来源，或完成经验证的估计 |
| /robot/status | DiagnosticArray / 项目状态 | SDK mode、E-stop、telemetry freshness、command timeout |
| /cmd_vel_vendor | 厂商批准的私有接口 | command state、限幅、watchdog、E-stop 验证之后才启用 |

不为填满接口而发布虚假的 odometry、零 covariance 或将 host receive time 冒充 source timestamp。

## 7. 当前仓库已经实现

robodog_lite3_bridge 当前只有 mock 和只读监测：
- mock topic 位于 /mock/lite3/*，frame 使用 mock_*，不覆盖真实机器人 topic/TF；
- 发布 synthetic odom/IMU/joint 类型供接口测试；
- 只读 freshness monitor 发布 DiagnosticArray；
- 不连接 vendor SDK/DRDDS/UDP，不发布 /cmd_vel；
- GitHub Actions 在 Humble/Jammy 和 Jazzy/Noble 均已构建通过，每侧 6/6 pytest。

这些测试只验证项目自有 mock/诊断代码，不代表真实 Lite3 SDK、传感器或运动控制已经验证。

## 8. 安全和许可证约束

- 在厂商确认 SDK mode/command_state、E-stop、底层 watchdog 和安全手册前，禁止开放 Nav2 自动运动。
- vendor E-stop service/策略超时不等于独立物理急停；实机测试必须具备硬件急停和现场监护。
- sdk_deploy 外层声明 BSD-3-Clause，嵌套 MotionSDK 等依赖需要逐项审计。未完成 SBOM/license/redistribution review 前，不复制 SDK .so 或第三方源代码到公共仓库。

## 9. 实施 Gate

- [ ] 厂商确认 Lite3 Venture 当前固件、SDK、motion host 和 AGX host 拓扑；
- [ ] 厂商确认正式腿式 odometry/body velocity 数据源及其频率、时间戳、frame、covariance；
- [ ] 证明 Foxy/Jazzy 之间的网关不依赖 DDS 跨版本互通；
- [ ] 完成 DRDDS IMU/joint/status 到标准 ROS 消息的映射审计；
- [ ] 完成 SDK mode、command_state、E-stop、watchdog 的安全审查与台架测试；
- [ ] 完成逐项 third-party license/SBOM 审计。