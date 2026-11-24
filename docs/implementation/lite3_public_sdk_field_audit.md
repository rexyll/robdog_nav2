# Lite3 Venture 当前公开 SDK 状态接口与安全边界审计

- 核验日期：2026-11-24
- 状态：公开接口源码核验完成；目标 Venture 固件/SDK 的完整能力仍需云深处书面确认
- 唯一进度台账：[TaskPlan.md](../../TaskPlan.md)
- 相关方案：[Lite3 SDK 集成评估](lite3_vendor_sdk_integration_assessment.md)、[供应商确认矩阵](../interfaces/lite3_vendor_confirmation_matrix.md)

## 1. 审计范围与结论

本审计仅对当前公开的 DeepRobotics sdk_deploy、Lite3 Motion SDK / RL SDK 接口与其文档作判断，不声称覆盖私有固件接口、未公开 SDK、所有 Venture 出厂版本或经销商专用分支。

**结论：当前已审阅的公开接口可以提供关节、IMU、接触力和控制状态，但尚不能证明其提供 Nav2 所需的完整机身位置与线速度 odometry。** 项目不得把 base_rpy、base_angular_velocity、关节角度或 IMU 积分拼成伪造的 nav_msgs/Odometry。

当前推荐的项目数据映射如下：

| 公开源 | 可确认数据 | 项目处理 |
|---|---|---|
| Motion SDK RobotData | IMU、12 个关节状态、足端接触力、设备 tick/timestamp | 只读规范化；单位/轴向/时钟与实际固件核验后再发布 |
| RL SDK / DRDDS /IMU_DATA | IMU 姿态、角速度、加速度数据 | 转换为 /robot/imu/data 前需核对字段单位、frame、时间戳和协方差 |
| /JOINTS_DATA / RobotStatus.joints | 关节状态 | 映射至 URDF joint names，形成 /robot/joint_states |
| RobotStatus | command/source/state、安全模式、timeout/拒绝计数、joint/IMU 时间戳、telemetry freshness、base RPY 和角速度 | 只进入健康诊断和状态监控；不视作完整 base odometry |
| 接触力 | 4 足接触力数据（具体消息字段/单位依版本核实） | 保留独立原始数据；后续可用于滑移/接触状态研究，不直接当作位置观测 |

## 2. RobotStatus.msg 字段审计

在公开仓库 DeepRoboticsLab/sdk_deploy 当前核验的 RobotStatus.msg（固定参考 commit 20279073bc43522aa06b0abf611bb2daaa53d87d）中，定义字段包括：

- header、robot；
- command_source、current_state、current_state_id、requested_state、requested_state_id；
- safety_mode、command；
- command_timed_out、rejected_commands、command_updates；
- joint_stamp、imu_stamp、telemetry_fresh；
- joints、base_rpy、base_angular_velocity。

该消息注释标明发布频率 10Hz，并注明关节位置和 base_rpy 为 radians，关节速度及 base_angular_velocity 为 rad/s（SDK/body frame）。该消息未定义 base xyz position、base linear velocity 或 pose/twist covariance。因此它本身不能构成 nav_msgs/Odometry。

这项审计只说明当前公开消息定义没有这些字段；厂商可能存在未公开的 estimator API、私有传输包或指定固件功能。V-06/V-07 仍必须向厂商确认，不能把“公开仓库没找到”扩大为“硬件绝对不存在”。

## 3. 单位、坐标系与时间戳风险

公开 Motion SDK 数据说明中出现以下单位语义：

- 关节位置：rad；速度：rad/s；力矩：N·m；温度：°C；
- IMU RPY：degree；角速度：degree/s；线加速度：m/s²；
- IMU 数据含 SDK timestamp（公开说明以 ms 表示）。

RL SDK/DRDDS 消息与 Motion SDK C++ API 的数据结构并不完全相同。实现 adapter 前必须以目标固件所用 msg、头文件、样例包和现场 rosbag 为准，逐项确认：
1. 姿态角是 degree 还是 rad，四元数是否可用；
2. angular velocity 是 degree/s 还是 rad/s；
3. acceleration 是 m/s² 还是其他单位；
4. IMU、关节、RobotStatus 的 timestamp 是采样时刻还是发送/接收时刻；
5. timestamp 所属设备/主机时钟域、启动时 epoch、回绕与重启行为；
6. 所有数据各自的轴向、frame、右手系定义和安装外参；
7. covariance/质量指标由谁计算；若没有真实协方差，不得填零矩阵假装高置信度。

在这些项目核验之前，数据只能进入原始诊断/离线审计，不进入 robot_localization 的正式融合配置。

## 4. 重要安全发现：transfer 启动并非纯被动读取

当前 sdk_deploy 的 lite3_transfer 启动流程包含 SDK mode/control acquisition 逻辑；SDK_MODE service 可启动/停止 transfer 与 SDK 控制状态。不能把“只启动 telemetry bridge”自然等同于“机器人完全不受控制状态变化影响”。

因此：
- 没有真实机器狗时，只允许审阅源码、mock 测试和离线样例；
- 真实机器狗第一次 bring-up 前，必须由云深处确认安全启动顺序、SDK Mode 默认态、站立/趴下/阻尼要求和遥控器接管方式；
- 未得到书面操作步骤，不运行会切换 SDK mode 的 Start() 或 /SDK_MODE 请求；
- 不发布 /JOINTS_CMD、/rl_deploy/cmd_vel，不切换 stand / rl_control；
- 仅在厂商批准的试验场地、有人员保护和可即时切断运动的条件下做逐步验证。

## 5. 软件安全功能不是安全认证急停

公开 RL state-machine 代码实现软件侧安全监测，文档/代码中可见的保护项目包括驱动器异常、关节 NaN、IMU 数据异常/超时、关节过温、电池阈值和用户命令 heartbeat 超时等；状态异常可进入阻尼/安全控制状态。公开接口另有 /EMERGENCY_STOP service 与 /EMERGENCY_STOP_SIGNAL。

这些是 SDK 控制软件功能，不足以证明其具备安全等级认证、硬件隔离断电、冗余安全回路或满足机器人系统风险评估要求。采购和实机测试前仍需厂商提供：
- 物理急停按钮/无线遥控急停的硬件链路图；
- 急停是否切断驱动使能，断电后机器人如何保持/失稳；
- 软件 E-stop 与物理 E-stop 的差异、复位步骤和权限；
- 通信断开、Jetson 掉电、SDK crash、host 重启时的底层行为；
- 安全模式是否有独立于普通 SDK 的硬件监督器。

本项目 Nav2 Collision Monitor 与 watchdog 仍是非安全认证软件保护层，不能替代物理急停。

## 6. Foxy/Jazzy 跨主机边界

当前官方 cross-host 文档描述 Lite3 motion host 使用 Ubuntu 20.04 / ROS 2 Foxy，并警告非 Foxy 的 host 与其 ROS 2 通信可能异常；文档提及 shadow subscriber workaround。该 workaround 不作为本项目生产集成基线。

项目暂时保持：
- vendor SDK / DRDDS 与 Nav2 Jazzy 使用不同通信域；
- 不共享 DDS domain，不传输 ROS 2 CDR bytes 作为稳定跨发行版协议；
- 由厂商确认 motion-host / AGX-host 角色后，再决定 UDP/TCP gateway；
- gateway 必须保留 source timestamp、clock domain、sequence、frame、unit、quality、session 和丢包统计；
- 运动命令通道与 telemetry 数据通道分离，且默认关闭。

## 7. 本轮审计后的适配输出契约

| 目标 topic | 消息 | 允许条件 |
|---|---|---|
| /robot/imu/data | sensor_msgs/Imu | 单位、轴向、时间戳与 covariance 经核验 |
| /robot/joint_states | sensor_msgs/JointState | 完成 12 关节名称、方向、单位与索引映射 |
| /robot/contact_state | 自定义只读消息/诊断消息 | 原始字段/单位/frame 已核验；无位置伪造 |
| /robot/status | diagnostic_msgs/DiagnosticArray 或项目状态消息 | 明确 current state、SDK mode、timeout、E-stop 信号与 freshness |
| /robot/odometry | nav_msgs/Odometry | 仅厂商给出并验证完整 legged odometry，或 Point-LIO 等算法通过验收后发布 |
| Nav2 速度命令 | 禁用 | Vendor state machine、watchdog、E-stop、速度边界、安全验收全部通过前不开放 |

## 8. 必须由云深处答复的补充问题

除原供应商矩阵 V-01 至 V-27 外，补充：

- V-28：Lite3 Venture 当前交付固件是否提供官方 base/legged odometry（position + linear velocity）？若有，提供 message/API/UDP schema、频率、时间戳、frame、covariance/quality 示例。
- V-29：Motion SDK/RL SDK 的 contact force 字段定义、单位、坐标系、零点标定和采样时刻是什么？是否存在 contact confidence/slip indicator？
- V-30：启动 lite3_transfer 或调用 /SDK_MODE 是否会获取电机/SDK 控制权？在不发送关节/速度命令的状态下，如何保证只读采集？
- V-31：/EMERGENCY_STOP 和 JointDamping 是否仅为软件层功能？物理急停、无线遥控急停、驱动器禁能/断电路径及安全认证信息是什么？
- V-32：目标 Venture SKU/固件的 IMU、关节、接触力、RobotStatus 的正式 timestamp 单位与 clock domain 分别是什么？能否提供带真实源时间的 rosbag/数据样例？

## 9. 一手资料与代码定位

- [DeepRobotics Lite3 SDK overview](https://developer.deeprobotics.cn/docs/lite3/overview/)
- [Lite3 Motion SDK](https://developer.deeprobotics.cn/docs/lite3/sdk-dev/motion-sdk/)
- [Lite3 RL SDK](https://developer.deeprobotics.cn/docs/lite3/sdk-dev/rl-sdk/)
- [DeepRobotics sdk_deploy fixed commit](https://github.com/DeepRoboticsLab/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d87d)
- [RobotStatus.msg at fixed commit](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/drdds/msg/RobotStatus.msg)
- [lite3_transfer source](https://github.com/DeepRoboticsLab/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_transfer)
- [Lite3 SDK service guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_sdk_service/README.md)
- [Lite3 cross-host communication guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/cross_machine/README.md)
