# Lite3 厂商 ROS 2 集成路径评估

- 日期：2026-11-24
- 状态：**架构核验完成；最终集成路径待云深处按目标 SKU/固件书面确认**
- 唯一进度台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 关联决策：[ADR-0003 — Lite3 ROS 集成路径](../decisions/ADR-0003-lite3-ros-integration.md)

## 1. 结论先行

目前公开的 Lite3 ROS 2 代码存在两套不同路线，不能把 topic、消息类型、时间戳或控制入口视为等价：

1. **Legacy Lite3_ROS（ROS 2 Foxy）**：transfer 包通过 UDP 接收运动主机数据，并直接转换为标准 ROS 消息 /leg_odom、/leg_odom2、/imu/data、/joint_states；示例同时订阅 /cmd_vel。它易于理解、标准消息友好，但 README 与分支明确指向 Foxy，代码中发现 steady-clock 时间戳以及 odometry frame/covariance 需进一步核对，不能直接作为 Jazzy 生产接口。
2. **Current sdk_deploy（官方新 SDK 部署路径）**：面向 Lite3 Venture Edition，包含 lite3_transfer、drdds 自定义消息、lite3_sdk_service 和 lite3_sdk_deploy policy/controller。lite3_transfer 面向硬件 UDP bridge，发布 /IMU_DATA、/JOINTS_DATA 等 DRDDS 消息；lite3_sdk_service 管理 SDK 模式、transfer 启停、频率和紧急停止；rl_deploy 使用 /rl_deploy/cmd_vel 等高层策略接口。这不是一套直接输出 Nav2 所需标准 Odometry/Imu/JointState 的简单 ROS bridge。

**推荐方向：以 sdk_deploy 作为 Lite3 Venture 厂商支持路径的优先核验对象；在 Nav2 和 Lite3 SDK 之间建立独立适配边界，不让 Nav2 直接依赖 DRDDS 或厂商内部控制状态机。** 在获得供应商对目标固件、计算机位置、ROS distro 和跨主机通信的书面确认前，不在项目仓库复制/编译/启动真实 SDK。

## 2. 两条路线的差异

| 项目 | Legacy Lite3_ROS | Current sdk_deploy |
|---|---|---|
| 仓库 | [DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS) | [DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy) |
| 当前公开分支 | ros2-foxy | main |
| 核验 commit | e4ab5dc1c7f3ff46274162a1ef302253936578aa | 20279073bc43522aa06b0abf611bb2daaa53d87d |
| 仓库声明许可 | MIT | BSD-3-Clause（外层仓库；嵌套依赖仍须逐项检查） |
| ROS distro | Foxy | Lite3 motion host 当前工作流说明为 Ubuntu 20.04 / Foxy；交叉主机示例有 Foxy 跨版本限制警告 |
| 状态消息 | 标准 PoseWithCovarianceStamped、Odometry、Imu、JointState | drdds/msg/ImuData、drdds/msg/JointsData、drdds/msg/RobotStatus 等 |
| 速度入口 | 旧示例 /cmd_vel | /rl_deploy/cmd_vel，且只在策略进入 rl_control 状态时有效 |
| 管理/保护 | 示例未提供与新 SDK service 同等的完整服务信息 | /SDK_MODE、/EMERGENCY_STOP、/EMERGENCY_STOP_SIGNAL 以及独立 UDP service |
| 适用性 | 适合作为理解标准 ROS topic 映射和历史协议的参考 | 更适合作为当前 Lite3 Venture 官方 SDK 部署路径的首选核验对象 |
| 风险 | Foxy 已过 EOL；时间戳/frame/covariance 质量需修复 | DRDDS 绑定、Foxy DDS 互通、SDK mode/安全流程、控制状态机更复杂 |

## 3. Legacy Lite3_ROS 细节与限制

公开 ros2-foxy 分支的 src/transfer/src/Jetson2Motion.cpp：
- 在 UDP 接收后发布 /leg_odom、/leg_odom2、/joint_states、/imu/data。
- /leg_odom2 消息类型是 nav_msgs/msg/Odometry。
- /cmd_vel 被订阅并转成 UDP 命令，示例语义为 linear.x 前后、linear.y 左右、angular.z yaw。
- 若代码以 rclcpp::Clock(RCL_STEADY_TIME).now() 赋给 header.stamp，该 stamp 是本机 steady time，不可未经时钟转换直接视为 ROS system time / GNSS time。
- 公开片段中 /leg_odom2 的 child_frame_id 未见赋值；姿态/速度 covariance 也未在片段中显式填入，可能保留默认零值。零 covariance 在 robot_localization 中会被误读为高置信度，必须核对/修复。
- 其 joint names、方向符号和 IMU frame 也必须与真实 URDF 和设备坐标系逐项核对。

因此，旧 bridge 的标准消息形式有价值，但不能仅因为 topic 名称相同就直接进入 local EKF。

## 4. Current sdk_deploy 的 ROS 接口和控制状态机

### 4.1 lite3_transfer

公开源码 src/lite3_transfer/interface/robot/udp_interface.hpp 和包说明表明：
- 发布 /IMU_DATA，类型 drdds/msg/ImuData；
- 发布 /JOINTS_DATA，类型 drdds/msg/JointsData；
- 订阅 /JOINTS_CMD，类型 drdds/msg/JointsDataCmd；
- 发布 /EMERGENCY_STOP_SIGNAL；
- 提供 /SDK_MODE 和 /EMERGENCY_STOP 服务；
- 使用 UDP 接收/发送运动主机数据；
- 源码默认发布频率为 200 Hz，可通过 SDK_MODE 服务调整；这不是项目承诺的有效频率，需在目标固件上测试。
- CMake 根据 aarch64/x86 选择 libdeeprobotics_legged_sdk 对应共享库；自带第三方 MotionSDK/DRDDS 依赖。

### 4.2 DRDDS 原始消息语义

已核验的消息定义：
- ImuData.msg：MetaType header + ImuDataValue data；
- ImuDataValue.msg：roll/pitch/yaw、omega_x/y/z、acc_x/y/z；
- JointsData.msg：MetaType header + JointsDataValue data；
- JointsDataValue.msg：16 个 JointData 项数组；
- RobotStatus.msg：10Hz 状态消息，包含 command/state、安全模式、command timeout/rejected count、joint/imu stamp、telemetry_fresh、JointState、base_rpy 和 base_angular_velocity。

注意：
- /JOINTS_DATA 是低层关节数据，不等于机身 odometry。
- /IMU_DATA 有姿态和角速度/加速度字段，但坐标轴、单位、协方差、stamp 及时间域仍需经厂商手册/代码和实测确认。
- RobotStatus 的 base_rpy/角速度和 telemetry_fresh 是状态信息，不等于可直接输入 robot_localization 的完整 nav_msgs/Odometry。
- 如果当前 SDK 路径没有公开可靠的腿式 odometry/机身速度状态，必须请厂商明确提供；不能仅靠关节角度或 IMU 积分假造绝对里程计。

### 4.3 rl_deploy 与速度命令

新 SDK 的高层部署 README 给出：
- /rl_deploy/cmd_vel：策略接口的 geometry_msgs/msg/Twist；
- /rl_deploy/command_state：字符串状态命令，包括 stand、rl_control、lie_down、damping；
- /rl_deploy/effective_cmd_vel：实际生效速度；
- /rl_deploy/status：10Hz 的 drdds/msg/RobotStatus。
- 文档默认速度上限示例：前进 0.7m/s、侧移 0.5m/s、yaw 0.7rad/s；这是示例默认值，不是本项目导航速度目标。
- 文档说明约 0.25s 没有速度输入后 effective command 归零；这属于该策略接口行为，不能替代独立 Collision Monitor、command adapter watchdog 或物理急停。
- /rl_deploy/cmd_vel 仅在策略控制状态（rl_control）下有效，不能将它当作任意状态下的通用速度入口。

**控制安全警告：** 新 SDK 文档要求仅在 idle/safe 状态切换 SDK Mode，状态切换后等待约 10s，并检查关节自由度/阻尼状态；错误模式切换可能导致机器突然站起或冲出。仓库尚未有真实设备，禁止照着网络示例直接执行 stand/rl_control/cmd_vel。

## 5. Foxy 与 Jazzy 跨主机问题

sdk_deploy 的 cross-host README 说明：
- Lite3 motion host 当前工作流使用 Ubuntu 20.04 / ROS 2 Foxy；
- Jetson AGX Orin + JetPack 6.1 / CUDA 12.6 的 cross-host 计算配置有公开验证案例；
- 文档明确警告：Lite3 主机 Foxy 与 host computer 非 Foxy 版本时，ROS 2 通信可能无法正常工作；文档称使用临时 shadow subscriber workaround。
- 文档还记录不同 ROS 参与者/发现流量可能造成 Fast DDS CDR deserialization std::bad_alloc，建议隔离不兼容参与者或 DDS domain。

**项目决定：**
1. 不让 Nav2 Jazzy 与 Lite3 Foxy 节点默认共享一个 ROS 2 DDS domain。
2. 不把跨 distro 的 DDS 通信视为已支持或可依赖。
3. 将厂商兼容层和 Nav2 标准接口分离；跨 distro 的候选边界为显式 UDP/TCP/其他经过验证的非 DDS 消息网关。
4. 网关消息需携带采集时间、源时钟、frame、序列号、状态健康和协方差/质量；丢包、超时和重放必须可诊断。
5. 先让 vendor stack 只读输出状态；在验证 command_state、SDK 模式、estop API 和底层 watchdog 前不接 Nav2 运动指令。

待云深处书面确认的架构问题：
- 目标 Lite3 Venture SKU/固件是否支持在机载 AGX 上原生运行 Jazzy？
- 目前推荐的 motion-host/AGX host 分工是否要求 Foxy 在哪一台设备运行？
- Foxy 主机能否只向 Jazzy 主机提供非 DDS UDP 状态数据？厂商是否支持此拓扑？
- 公开 DRDDS 消息是否足够重建本体 odometry，还是必须使用额外的 legged odom 状态通道？
- 官方 E-stop 服务、命令 timeout、SDK Mode 的具体状态机和现场安全手册是什么？

## 6. 许可证和供应链管理

- sdk_deploy 外层仓库声明 BSD-3-Clause；
- Lite3_ROS repository 声明 MIT；
- src/lite3_transfer/third_party/Lite3_MotionSDK/LICENSE 声明 MIT；
- drdds package.xml 声明 BSD 3-Clause；
- sdk_deploy 含多个嵌套第三方依赖，必须逐目录读取许可证/notice，不能只看顶层 README 或仓库主许可证。
- 不将整份 sdk_deploy 或预编译 .so 复制进本项目公开仓库，直到完成 SBOM、license/notice、二进制分发权、固件/SDK 授权和维护责任审查。
- 更安全的第一步是记录上游仓库、固定 commit、依赖版本和安装校验脚本；真正 submodule/vendor copy 必须在明确批准后进行。

## 7. 本项目推荐的集成层次

    Lite3 motion host / official SDK runtime
              |
              | vendor-supported Foxy / UDP / DRDDS boundary
              v
    Vendor compatibility adapter (isolated)
              |
              | explicit non-DDS gateway, with timestamp/frame/status
              v
    robodog_lite3_bridge (project-owned; Jazzy/Humble standard messages)
              |
              +--> normalized odometry / IMU / joint state / robot status
              |
              +--> localization + diagnostics
              |
    Nav2 Jazzy
              |
    safe command adapter (disabled until vendor-approved)
              |
    validated vendor command gateway / SDK state machine

当前仓库中 robodog_lite3_bridge 只有 mock 发布器和 topic freshness monitor，尚无任何真实 SDK、DRDDS、UDP 或 cmd_vel 集成。

### 建议适配输出契约（最终字段由源码/实测确认）

| 项目输出 | 候选类型 | 来源/限制 |
|---|---|---|
| /robot/imu/data | sensor_msgs/msg/Imu | DRDDS ImuData 经轴向/单位/时间戳转换，填充真实 covariance |
| /robot/joint_states | sensor_msgs/msg/JointState | DRDDS JointsData/RobotStatus.joints，需核对 12/16 joint 映射和名称 |
| /robot/odometry | nav_msgs/msg/Odometry | 只在取得可信本体位置/速度来源后发布；不能凭 IMU 积分或关节角度虚构 |
| /robot/status | 项目内部诊断状态或标准 DiagnosticArray | command state、SDK mode、estop、telemetry freshness |
| /cmd_vel_vendor | 厂商验证的命令协议 | 只有明确进入允许运行状态、E-stop 和 watchdog 验证后开放 |

## 8. 当前决策与阶段 Gate

**决定：** 暂不复制或集成厂商 SDK 源码；优先以官方 sdk_deploy 为厂商参考路径，但先做只读状态映射和跨 distro 网关 POC；Legacy Lite3_ROS 用作标准消息协议参考，不作为默认 production path。

准入条件：
- [ ] 云深处确认目标 SKU、当前固件、SDK deploy 版本和推荐 ROS distro；
- [ ] 选定 motion host/AGX host 的分工与网络拓扑；
- [ ] 确认 Foxy/Jazzy 分离方式，不依赖跨 distro DDS；
- [ ] 证明 DRDDS/厂商状态数据能提供可信 IMU、joint state、机身 odometry 或明确说明缺项；
- [ ] 时间戳、轴向、单位、frame、协方差和状态机完成实测/审计；
- [ ] E-stop 与 watchdog 的操作流程获得厂商说明，并在独立测试环境验证；
- [ ] 完成第三方许可证/SBOM 和源码/二进制使用范围审查。

## 9. 一手资料与固定版本

- [sdk_deploy main @ 20279073bc43522aa06b0abf611bb2daaa53d87d](https://github.com/DeepRoboticsLab/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d87d)
- [Lite3_ROS ros2-foxy @ e4ab5dc1c7f3ff46274162a1ef302253936578aa](https://github.com/DeepRoboticsLab/Lite3_ROS/tree/e4ab5dc1c7f3ff46274162a1ef302253936578aa)
- [sdk_deploy Lite3 SDK deployment guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/src/Lite3_sdk_deploy/README.md)
- [sdk_deploy lite3_sdk_service guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/src/lite3_sdk_service/README.md)
- [sdk_deploy lite3_transfer](https://github.com/DeepRoboticsLab/sdk_deploy/tree/main/src/lite3_transfer)
- [legacy Lite3_ROS Jetson2Motion source](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/ros2-foxy/src/transfer/src/Jetson2Motion.cpp)
- [sdk_deploy Lite3 MotionSDK license](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/src/lite3_transfer/third_party/Lite3_MotionSDK/LICENSE)


## 10. 2026-11-24 公开 SDK 字段与安全审计补充

已单独核验公开 Motion SDK / RL SDK / RobotStatus 数据字段、单位差异、transfer 启动控制权和软件急停边界。公开 RobotStatus 只有 base_rpy/base_angular_velocity，没有 base position/linear velocity/covariance；不能单独转换为完整 nav_msgs/Odometry。公开 Motion SDK 可读关节、IMU 与足端接触力，但仍须对目标固件核对具体单位和时间源。

另一个重要限制是 lite3_transfer 启动及 SDK_MODE 可能获取 SDK 控制权，故实机 bring-up 前必须获得只读启动 SOP、遥控器接管方式和物理急停说明。详见 [Lite3 public SDK field audit](lite3_public_sdk_field_audit.md) 与 [供应商确认矩阵 V-28–V-32](../interfaces/lite3_vendor_confirmation_matrix.md)。
