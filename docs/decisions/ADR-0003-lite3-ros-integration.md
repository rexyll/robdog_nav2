# ADR-0003：Lite3 厂商 ROS 集成路径与跨发行版边界

- 状态：**Proposed — 官方路径核验完成，最终部署拓扑待云深处确认**
- 日期：2026-11-24
- 关联：[TaskPlan.md](../../TaskPlan.md)、[Lite3 vendor integration assessment](../implementation/lite3_vendor_sdk_integration_assessment.md)

## 背景

Lite3 公开仓库包含旧版 Lite3_ROS 与新 sdk_deploy 两套不同 ROS 路径。旧版直接发布标准 ROS 2 消息，分支为 Foxy；新 SDK deploy 面向 Lite3 Venture，使用 drdds 自定义消息、lite3_transfer、lite3_sdk_service 和 rl_deploy，并说明 Lite3 motion host 当前使用 Foxy，跨主机不同 ROS distro 的 DDS 通信不可靠。

本项目的长期 Nav2 目标为 ROS 2 Jazzy / Ubuntu 24.04。不能将 Foxy 与 Jazzy 的 DDS 通信视为已支持。

## 决策

1. 把 sdk_deploy 作为 Lite3 Venture 当前厂商集成路径的首选核验对象。
2. 把 Lite3_ROS Foxy transfer 作为旧版标准 topic 映射参考，不默认作为 production driver。
3. 不在本项目公共仓库直接复制完整厂商 SDK、.so 或 third-party tree；先固定上游 commit，完成 SBOM/license/notice/redistribution 审查。
4. robodog_lite3_bridge 的责任是把经过验证的厂商数据转换成项目标准消息，而不是在其中直接调用关节 SDK。
5. Nav2 Jazzy 与 vendor Foxy side 不共享默认 DDS domain。首选隔离兼容层 + 经验证的非 DDS transport gateway；若厂商书面确认可原生在 Jazzy 运行，再减少桥接层。
6. 首先只读接入 IMU/joint/status；只有获得可信本体 odom 后才接 robot_localization。
7. 在厂商确认 SDK mode/command_state/E-stop/watchdog，并验证 stop 路径之前，禁用从 Nav2 到机器狗的运动命令路径。

## 原因

- 新版 SDK transfer 输出 DRDDS custom messages，不等同于标准 sensor_msgs/Imu 或 nav_msgs/Odometry；
- 旧版 /leg_odom2 代码使用 steady clock 作为消息时间戳，且 frame/covariance 需审计；
- 新 SDK 的 /rl_deploy/cmd_vel 只在 rl_control policy state 下有效，另有 stand/lie_down/damping state transitions；
- 厂商 README 明确提示 Foxy 与不同 ROS distro 跨主机 DDS 通信问题；
- SDK 源码嵌套 third-party packages，需单独许可检查。

## 后果

- 必须实现标准消息边界和时钟/TF/covariance 映射；
- 可能需要非 DDS UDP/TCP 网关或 vendor-supported sidecar；
- 不得未经设备和供应商验证直接将 /cmd_vel remap 到 /rl_deploy/cmd_vel；
- 需要专门验证状态机、命令超时、E-stop、底层 telemetry stale 行为；
- 真实硬件接入前，mock package CI 仍只验证项目自有逻辑。

## 接受此 ADR 的条件

- 云深处书面确认 Lite3 Venture 当前 firmware、SDK 和 motion/compute host 角色；
- 通过实际硬件验证 topic/message、状态频率、时间戳、坐标、covariance；
- 经台架/安全程序验证 command_state、E-stop、watchdog 和停止距离；
- 跨 ROS distro 数据路径无共享 DDS 假设，且故障时 fail-safe；
- 许可证和 SBOM 审计通过。
