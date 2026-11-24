# Lite3 本体 ROS 2 接口契约与安全命令桥

- 状态：接口设计草案，基于公开 Lite3 ROS 2 transfer/SDK 文档；具体固件和本机行为待验证。
- 进度真源：[根目录 TaskPlan.md](../../TaskPlan.md)
- 相关 ADR：[ADR-0002 硬件平台选型](../decisions/ADR-0002-hardware-platform.md)

## 1. 目的

为 Nav2 提供标准 ROS 2 里程计、IMU 和速度接口，同时把厂商 SDK/UDP 协议限制在一个隔离的 bridge 层。Nav2 不直接访问 UDP、SDK 数据结构或关节控制接口。

## 2. 已公开的 Lite3 ROS 2 topic 契约

云深处公开的 `Lite3_ROS` transfer 示例列出以下话题：

| Topic | 消息类型 | 方向 | 公开语义 |
|---|---|---|---|
| `/leg_odom` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Lite3 → ROS | 腿式里程计位姿 |
| `/leg_odom2` | `nav_msgs/msg/Odometry` | Lite3 → ROS | 腿式里程计位姿和速度 |
| `/imu/data` | `sensor_msgs/msg/Imu` | Lite3 → ROS | IMU |
| `/joint_states` | `sensor_msgs/msg/JointState` | Lite3 → ROS | 关节状态 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Lite3 | 速度命令 |

公开示例中的 Twist 分量：
- `linear.x`：纵向速度，正值前进；
- `linear.y`：横向速度，正值向左；
- `angular.z`：偏航角速度，正值左转；
- `linear.z`、`angular.x`、`angular.y`：示例注明无效。

SDK deploy 公开说明提到状态消息可在约 10 Hz 更新、有效控制命令可达约 50 Hz。**这些是公开示例的说明，不可作为任意固件/模式下的保证频率。** 本项目必须在目标 Lite3 版本上检查 topic、频率、timestamp、frame_id、covariance 和控制语义。

## 3. 项目内部接口命名（建议）

不要直接将厂商原始 topic 暴露为 Nav2 的永久接口。通过 remap/namespace 隔离，内部统一成：

| 项目内部 topic | ROS 类型 | 数据来源 |
|---|---|---|
| `/robot/legged/odom` | `nav_msgs/msg/Odometry` | Lite3 `/leg_odom2` |
| `/robot/imu/data` | `sensor_msgs/msg/Imu` | Lite3 `/imu/data` |
| `/robot/joint_states` | `sensor_msgs/msg/JointState` | Lite3 `/joint_states` |
| `/robot/status` | 厂商状态或项目状态消息 | SDK/transfer |
| `/cmd_vel_nav2` | 发行版配置决定 Twist / TwistStamped | Nav2 输出 |
| `/cmd_vel_lite3` | `geometry_msgs/msg/Twist` | 安全 adapter 的最终输出 |

topic 重命名和 frame_id 最终在真实驱动 launch 中固定，不允许只为了方便而伪造 frame_id。

## 4. 当前已实现的软件骨架（2026-11-24）

仓库中已建立 robodog_lite3_bridge ament_python package：
- lite3_state_mock 发布 /mock/lite3/leg_odom2、/mock/lite3/imu/data、/mock/lite3/joint_states；使用 mock_odom/mock_base_link/mock_imu_link，不会覆盖真实硬件 topic/TF。
- lite3_topic_contract_monitor 只读订阅配置的话题，发布 DiagnosticArray 新鲜度诊断。
- mock 消息具有大协方差/未知 IMU orientation/空 JointState arrays，只用于 topic/type/launch 测试。
- 当前 package 不连接 Lite3 SDK/UDP，不发布 /cmd_vel，不能用于真实定位或运动控制。
- GitHub Actions 计划对 Humble/Ubuntu 22.04 与 Jazzy/Ubuntu 24.04 构建测试；结果以根 TaskPlan 记录的 workflow run 为准。

## 5. 软件包职责边界

建议拆成两个层：

### 4.1 `robodog_lite3_bridge`

- 调用官方 SDK/transfer 或官方 UDP 协议；
- 接收 Lite3 机身状态、IMU、关节状态、leg odometry；
- 做数据类型映射、单位检查、frame/time 标注和诊断；
- 不做 Nav2 规划，不做目标检测，不直接发关节动作；
- 保留厂商原始状态用于故障追查；
- 固定官方代码 URL、license、commit/tag 和 SDK/固件版本。

### 4.2 `robodog_cmd_adapter`

- 订阅 Nav2 输出（经过 Velocity Smoother 和 Collision Monitor）；
- 检查机器人运动模式/自动控制使能、健康状态、急停状态、命令时间戳和数据年龄；
- 按真实底层可执行运动模型对 x/y/yaw 限幅；
- 增加线速度/角速度变化率限制；
- 对命令超时、网络断开、状态异常、保护触发输出停止命令并锁存故障；
- 记录 Nav2 期望速度和机器人实际速度，诊断执行误差；
- 不允许任何旁路节点直接向厂商 `/cmd_vel` 发布运动命令。

## 6. 速度命令安全链

```text
Nav2 Controller
      |
      v
/cmd_vel_nav2
      |
Velocity Smoother
      |
Collision Monitor
      |
robodog_cmd_adapter
      |  - mode/enable check
      |  - speed/accel limit
      |  - freshness/watchdog
      |  - e-stop/status gate
      v
/cmd_vel_lite3
      |
Lite3 SDK / UDP transfer
      |
Motion host / gait controller
```

### 5.1 必须实现的保护

- 命令只从一个受控入口进入；
- 过期命令不能继续维持上一次非零速度；
- 未进入允许的运动模式时，非零速度被拒绝；
- 无法读取机器人状态或通信链路断开时，adapter 进入故障状态；
- 触发物理急停、远程停止或底层保护时，adapter 不得自动恢复行走；
- 清除故障/重新使能必须是显式动作，不应由 Nav2 自动 recovery 隐式完成；
- 命令超时阈值、速度/加速度限制由实测、底层 SDK 限制和安全风险评估确定，当前不写入未经验证的固定数值。

## 7. 状态估计输入的验证事项

收到 `/leg_odom2` 和 `/imu/data` 后先记录原始数据，不应立即认为两个数据统计独立。

必须核实：
1. `/leg_odom2` 是否已融合 IMU；
2. pose/twist 所属坐标系、frame_id 和 child_frame_id；
3. 时间戳来自运动主机采样时刻还是 ROS 接收时刻；
4. covariance 是否真实有效，是否只是零矩阵/占位符；
5. 关节/足端接触状态是否可用于检测滑移；
6. 运动模式切换时速度、姿态和里程计是否连续；
7. 原地转向、横移和足滑时里程计表现。

只有完成上述检查，才选择 EKF 的融合变量。若厂商提供的腿式 odometry 已经是 IMU 融合状态，不将同一惯性信息重复当成独立传感器融合。

## 8. 当前 ROS 2 发行版策略

- Lite3 Venture 官方感知扩展示例使用 Ubuntu 22.04 + ROS 2 Humble + JetPack 6.1。
- Lite3_ROS 公共仓库的旧版 ROS 2 示例有 Foxy 版本信息；SDK deploy 新仓库提供 Lite3 ROS 2 transfer。
- 首轮以 Humble 复现官方硬件示例；同时评估 JetPack 7.2.2 + Ubuntu 24.04 + Jazzy 在 AGX Orin 上的实际构建/运行情况。
- JetPack 7.2.2 当前面向 Jetson Orin Family，系统基线为 Ubuntu 24.04；ROS 2 Lyrical 需要 Ubuntu 26.04，当前不作为 Orin 默认基线。
- 自研 bridge/adapters 需在 CI 检查 Humble/Jazzy；若 Orin BSP 支持 Ubuntu 26.04 且外设驱动成熟，再评估 Lyrical。实际兼容性以锁定 commit 的构建结果为准。
- 不同 ROS distro 不应默认在同一 DDS domain 直接互通。需要跨 distro 时通过 UDP/串口/标准消息网关传输。

## 9. 实施验收

1. **只读模式：** 启动 bridge，确认 odom/IMU/joint/status 连续，记录频率/时间戳/丢包。
2. **坐标模式：** 检查所有 frame，生成 TF 图，确认唯一发布者。
3. **消息语义：** 比较静止、前进、横移、原地旋转时 topic 值和机器人真实方向。
4. **状态异常：** 断开 Ethernet/SDK、停止 topic、发送 NaN/超限命令，确认 adapter 拒绝非零运动并报告诊断。
5. **Watchdog：** 只有在现场急停、人工监护和封闭空间条件就绪时，逐级测试命令超时到停止的实际延迟。
6. **低速执行：** 台架/抬起状态只用于通信与步态状态确认；任何地面移动从极低速度开始，逐级记录停车距离。
7. **Nav2 闭环：** 在 Collision Monitor 正常、数据新鲜、机器人处于允许模式时才开放自动运动。

## 10. 参考源码与文档

- [DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS)
- [DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
- [Lite3 官方感知算法开发示例](https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/)
