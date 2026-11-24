# Lite3 Venture 供应商技术确认矩阵

- 用途：采购与真实 SDK 集成前的供应商技术问询清单。
- 项目状态台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 相关决策：[ADR-0002](../decisions/ADR-0002-hardware-platform.md)、[ADR-0003](../decisions/ADR-0003-lite3-ros-integration.md)
- 相关源码评估：[Lite3 vendor SDK integration assessment](../implementation/lite3_vendor_sdk_integration_assessment.md)

## 1. 机型、固件和软件支持

| ID | 需要厂商确认的问题 | 需要的证据/输出 | 对项目的影响 |
|---|---|---|---|
| V-01 | 当前 Lite3 Venture 可采购 SKU、硬件 revision、固件版本是什么？ | SKU/配置表、固件 release notes | 固定目标设备和驱动版本 |
| V-02 | 当前 Venture 推荐使用 Legacy Lite3_ROS 还是 sdk_deploy？ | 官方推荐仓库、branch/tag/commit | 确定 vendor integration path |
| V-03 | motion host 与 AGX Orin host 的正式部署拓扑是什么？哪个进程运行在哪台设备？ | 官方部署图、启动说明、网络接口 | 决定 ROS distro 隔离和 gateway |
| V-04 | 当前支持 ROS 2 Foxy、Humble、Jazzy 中哪些版本？是否有 Ubuntu 24.04 / JetPack 7.x 正式支持计划？ | 支持矩阵、编译说明、测试设备清单 | 决定长期软件基线 |
| V-05 | Foxy motion host 与 Jazzy AGX host 之间，官方支持哪种跨主机通信方式？是否禁止共享 DDS？ | 官方支持的 UDP/TCP/bridge 方案、配置示例 | 避免不兼容 DDS 数据损坏 |

## 2. 定位、IMU 和里程计（采购前关键问题）

| ID | 需要厂商确认的问题 | 需要的证据/输出 | 对项目的影响 |
|---|---|---|---|
| V-06 | 当前 sdk_deploy 是否提供正式的 base/legged odometry（位置、姿态、线速度、角速度）？具体 topic/message 是什么？ | 消息定义、发布频率、字段说明和示例 rosbag | 决定 local EKF 是否可直接建立 |
| V-07 | 若没有 nav_msgs/Odometry，是否有内部腿式状态估计原始数据/UDP packet 或 SDK API？ | 数据结构、单位、坐标系、SDK 头文件 | 决定是否开发状态估计 adapter |
| V-08 | DRDDS RobotStatus 的 base_rpy、base_angular_velocity 和 JointState 字段定义是什么？是否包含 base position/linear velocity？ | 完整 RobotStatus.msg 和字段文档 | 避免把状态信息误当 odom |
| V-09 | IMU 输出频率、测量单位、坐标轴、frame、温漂/偏置、时间戳来源是什么？ | IMU 手册、topic 录制样例、轴向图 | 决定 IMU 标定及 EKF |
| V-10 | 厂商腿式 odom 是否已经融合 IMU？协方差如何计算？ | 状态估计文档、covariance 说明 | 避免 EKF 重复融合相关测量 |
| V-11 | 是否可获取足端接触、足端速度、滑移状态或质量标志？ | topic/message/API 说明 | 改善足滑和动态状态估计 |
| V-12 | 所有数据的采样时刻、host receive time、时钟域、frame_id 和 covariance 的语义是什么？ | 消息时间戳说明、实际 rosbag | 决定时间同步、TF 和信任模型 |

## 2.1 2026-11-24 公开接口核验补充

- 已审阅的公开 sdk_deploy RobotStatus.msg 发布频率为 10Hz，字段有 joints、base_rpy、base_angular_velocity、状态、timeout 计数和 telemetry_fresh；该消息没有 base xyz position、base linear velocity 或 pose/twist covariance。
- Motion SDK 公开接口说明可读取 IMU、12 个关节及足端接触力；当前公开材料没有证明其提供完整机身位置/线速度估计。
- lite3_transfer 的启动/SDK_MODE 流程可能获取 SDK 控制权，不能默认视作无副作用的纯只读采集。实机 bring-up 前必须取得安全操作流程。
- /EMERGENCY_STOP 与阻尼状态属于公开 SDK 的软件功能；是否有独立硬件急停、驱动器禁能回路及安全认证仍需书面确认。
- 详细字段审计见 [Lite3 public SDK field audit](../implementation/lite3_public_sdk_field_audit.md)。

## 3. 高层运动控制与安全

| ID | 需要厂商确认的问题 | 需要的证据/输出 | 对项目的影响 |
|---|---|---|---|
| V-13 | Nav2 应连接旧 /cmd_vel 还是 /rl_deploy/cmd_vel？后者需要哪个 command_state？ | 当前 SDK 控制状态机图和控制接口手册 | 避免错误状态下命令无效或突发运动 |
| V-14 | stand、rl_control、lie_down、damping 等状态切换的正式前置条件和操作顺序是什么？ | 厂商安全 SOP 和状态转移图 | 控制接入的强制安全 Gate |
| V-15 | 命令 watchdog、网络断连、SDK 进程退出、状态数据过期时，机器人分别采取什么动作？ | timeout 值、底层行为说明、实测日志 | 建立 fail-safe 命令层 |
| V-16 | /EMERGENCY_STOP service 是否为安全认证急停？是否仍需物理急停和独立遥控停止？ | 安全手册、风险声明、E-stop 接线图 | 防止把软件 service 当硬件安全设备 |
| V-17 | 官方推荐的速度/加速度/jerk、横移限制、旋转速度和模式限制是多少？ | 具体固件/SDK 版本参数表 | 设置 Nav2 controller/velocity smoother |
| V-18 | 进入自动运动、退出自动运动、急停复位和重新使能由谁授权？ | 操作流程和状态反馈 | 避免 Nav2 自动 recovery 误复位 |

## 4. 外部电源、载荷和安装

| ID | 需要厂商确认的问题 | 需要的证据/输出 | 对项目的影响 |
|---|---|---|---|
| V-19 | 24V、12V、5V 辅助电源每路的连续电流、峰值电流、总功率和保护策略是什么？ | 电气规格表、连接器 pinout | 决定 Orin/雷达/外设供电 |
| V-20 | 24V XT30 是否允许同时给 MID-360 和外部 24V→USB-C PD DC/DC 供电？ | 供电路径图、最大负载说明 | 决定是否需要独立电池 |
| V-21 | 4.5kg payload 的重心、安装面积、惯量和动态步态限制是什么？ | 载荷条件、允许 CoM 图、机械图 | 决定 3.6kg 项目内部预算是否可行 |
| V-22 | 是否有 Venture + AGX Orin 官方安装板/孔位/减振结构 CAD？ | STEP/STP、孔位图、固定件清单 | 决定上装结构和视场布置 |
| V-23 | 外部计算机/传感器装载后续航如何变化？是否有负载曲线？ | 不同上装质量下续航数据 | 决定任务时长和电池预算 |

## 5. 网络、时间同步和服务接口

| ID | 需要厂商确认的问题 | 需要的证据/输出 | 对项目的影响 |
|---|---|---|---|
| V-24 | motion host 与 AGX Orin 之间的 Ethernet IP、端口、UDP/TCP 协议、MTU、带宽要求是什么？ | network.toml 样例、port list、网络拓扑 | 设计不依赖 DDS 的 gateway |
| V-25 | lite3_sdk_service 的 UDP 端口、on/off/rate/estop 命令及安全限制是什么？ | service API 文档、请求/响应示例 | 建立只读启动与健康监测 |
| V-26 | SDK 数据包是否包含源采样时间、序列号和丢包计数？ | 协议说明、PCAP/rosbag 样例 | 时间映射与数据完整性 |
| V-27 | 本体 IMU、motion host clock、AGX system clock 是否支持 PTP/PPS 或其他时间校正？ | 时间同步架构和误差预算 | 与相机/MID-360/RTK 对齐 |
| V-28 | 当前 Venture 固件是否提供官方 base/legged odometry（position + linear velocity）？若有，其 message/API/UDP schema、频率、时间戳、frame、covariance/quality 是什么？ | 正式接口文档、消息定义、带真值 rosbag | 决定 local odom 主要来源 |
| V-29 | Motion SDK/RL SDK 的 contact force 字段定义、单位、坐标系、零点标定和采样时刻是什么？是否存在 contact confidence/slip indicator？ | 接口定义、标定说明、样例数据 | 决定是否能用于滑移/接触状态估计 |
| V-30 | 启动 lite3_transfer 或调用 /SDK_MODE 是否会获取电机/SDK 控制权？如何确保只读 telemetry bring-up？ | 启动状态图、只读步骤、遥控器接管 SOP | 防止启动 telemetry 时意外切换机器人控制权 |
| V-31 | /EMERGENCY_STOP 和 JointDamping 是否仅为软件层功能？物理急停、无线遥控急停、驱动器禁能/断电路径及安全认证信息是什么？ | 安全手册、电气图、测试流程和认证声明 | 关闭实机测试安全 Gate |
| V-32 | 目标 Venture SKU/固件的 IMU、关节、接触力、RobotStatus 时间戳单位与 clock domain 分别是什么？能否提供源采样时间 rosbag？ | timestamp 说明、clock-domain 图、原始 rosbag | 决定跨主机时间映射和 EKF 可用性 |

## 6. 项目采购和集成准入

采购前必须拿到或实测以下资料：
- [ ] 目标 Lite3 Venture SKU/固件/SDK 和推荐部署拓扑已确认；
- [ ] 正式 legged odometry/body velocity 输出已确认；若无，则接受项目改用 LiDAR odometry/SLAM 或 stereo VIO 的方案；
- [ ] Foxy/Jazzy 跨主机通信采用正式支持的隔离方案，不依赖默认 DDS 互通；
- [ ] SDK mode、command_state、timeout、E-stop、watchdog 操作流程已获得书面说明；
- [ ] 每路电源连续/峰值电流、总功率、载荷/CoM 和安装 CAD 已收到；
- [ ] 网络端口/UDP 协议/时间戳语义已收到；
- [ ] SDK 和 third-party 依赖的 license/SBOM 审核完成；
- [ ] 完成只读 bring-up 前，未向机器狗发出任何关节或速度运动命令。