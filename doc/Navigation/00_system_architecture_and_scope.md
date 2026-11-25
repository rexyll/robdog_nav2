# 00 — 完整导航模块总体架构与系统边界

## 1. 定义

本项目所称导航模块是端到端闭环系统：将任务目标和环境/机器人状态转化为满足运动能力与安全约束的运动意图，并通过四足平台执行反馈形成闭环。它包含多个可独立部署的软件子系统、硬件设备和安全机制。

Nav2 是优先评估的 ROS 2 导航框架，承担任务行为编排、规划/控制插件托管、Costmap 和恢复行为等职责；它不等于定位、感知、SLAM、四足地形分析、底层步态或完整安全系统。

## 2. 分层、责任与边界

| 层 | 主要生产者/组件 | 主要消费者 | 输出/契约 | 典型失效 |
|---|---|---|---|---|
| 任务应用 | Mission Manager、UI、巡检调度 | 导航协调层 | Goal/Waypoint/区域/取消/暂停 | 目标无效、任务冲突 |
| 任务协调 | BT/状态机、Nav2 BT Navigator 或自研调度器 | 定位、地图、规划、控制 | 黑板、Action 状态、恢复策略 | 超时、死循环、恢复耗尽 |
| 设备接入 | Livox、相机、GNSS、IMU、关节/足端驱动 | 时间同步、感知、定位 | 原始观测与设备状态 | 断流、丢包、时钟漂移 |
| 时间与标定 | PTP/NTP/PPS/trigger、标定管理 | 所有多传感器消费者 | 同步时间戳、内外参、时钟域 | 时间跳变、标定过期 |
| 状态估计 | 腿式 odom、LIO、VIO、EKF、GNSS gate | 地图、Costmap、规划、控制 | map→odom、odom→base、pose/twist/covariance | 漂移、跳变、失锁 |
| SLAM/地图 | slam_toolbox/LIO-SLAM/map server | 定位、规划、Costmap | 2D/3D地图、地图元数据 | 地图不一致、回环跳变 |
| 环境感知 | 点云/深度处理、检测/跟踪、地面/负障碍检测 | 环境表达、规划 | obstacle/track/ground/semantic | 漏检、误检、延迟 |
| 环境表达 | Costmap、voxel、elevation、semantic/topological map | Planner、Controller、traversability | 环境代价/高度/语义/拓扑 | stale layer、清除失败 |
| 可通行性 | 坡度/粗糙度/台阶/落足区域评估 | 全局/局部规划、步态接口 | traversability/risk/footstep constraints | 错判可通行区域 |
| 规划与避障 | Route/Global Planner/Local Planner/Controller | 命令仲裁与执行 | Path/Trajectory/velocity intent | 无解、振荡、碰撞风险 |
| Nav2 框架 | BT Navigator、Planner/Controller/Behavior、Costmap | 任务与运动命令层 | Action/Path/cmd_vel | 插件失败、Lifecycle失效 |
| 命令安全 | arbiter、限幅、Collision Monitor、watchdog | 平台 adapter | 受约束命令或 stop | 保护未触发、命令竞争 |
| 平台适配 | Lite3 adapter/SDK transport | 厂商运动控制器 | 厂商定义的速度/姿态/步态命令 | SDK失联、模式错误 |
| 底层执行 | 厂商步态控制、电机驱动 | 机身运动 | 实际运动与反馈 | 跌倒、打滑、过温 |
| 观测与运维 | diagnostics、rosbag、metrics、日志 | 工程师/安全监控 | 运行证据和告警 | 诊断盲区、数据缺失 |

## 3. 两条必须闭合的链路

### 3.1 感知到决策链
传感器物理观测 → 驱动/解码 → 时间同步/坐标标定 → 预处理/运动畸变补偿 → 定位/检测/跟踪/地面分割 → 地图/Costmap/elevation/traversability → 全局/局部规划 → 轨迹/运动意图。

### 3.2 命令到执行反馈链
任务/控制器 → 速度/轨迹意图 → 命令仲裁 → 安全门控/限幅/碰撞监控/watchdog → 平台协议适配 → 厂商步态/运动控制器 → 关节/足端/机身实际响应 → 状态估计/执行反馈 → 控制器闭环。

“已发布 cmd_vel”不是“机器人已执行”；必须有命令接受状态、实际速度/姿态反馈、停止响应和故障状态。

## 4. 坐标与时间系统

初始 TF 目标为 map → odom → base_link → sensor frames，但需由实际定位架构确定唯一发布 authority。任何模块不得重复发布同一 TF edge。记录：
- frame 名称、父子关系、轴向、单位和 REP-103/REP-105 约定。
- 时间戳来源（sensor hardware / host receive / ROS time）、时钟域、同步方法、偏移和漂移。
- transform lookup 的目标时间、缓存、外推限制和超时行为。
- 机身姿态变化、足式振动和 LiDAR 扫描期间运动对 deskew 的影响。
- GNSS antenna lever arm、IMU/LiDAR/camera extrinsics、双目左右相机标定和触发偏移。

## 5. 质量门控和失效传播

每条输入必须有 validity、freshness、confidence/covariance、frame/time validity 和 health 状态。不得只检查 topic 是否存在。至少定义：
- stale data、时钟回拨/跳变、序号丢失、TF 缺失、协方差无效。
- 定位跳变、GNSS Fixed→Float→Single、LIO tracking lost、地图不连续。
- 点云空帧/稀疏/遮挡、深度无效、动态目标遮挡和误检。
- Costmap observation timeout、ghost obstacle、unknown space。
- 规划无解、控制器 oscillation、command timeout、adapter/SDK 断连。
- CPU/GPU/内存/温度/电源/网络异常。

各故障必须有检测阈值来源、状态机、降级行为、恢复条件和可审计事件。默认 fail-safe，不允许以陈旧数据继续输出运动命令。

## 6. Nav2 与非 Nav2 组件

Nav2 管理导航行为和标准化规划/控制流程；下列能力通常由外围组件提供或需要平台专用实现：
- 驱动与硬件时间同步；
- LIO/VIO/leg odometry、GNSS quality gate、SLAM/重定位；
- 语义检测、动态目标跟踪、负障碍识别；
- elevation/traversability/footstep planning；
- Lite3 步态/运动接口、SDK 控制权和反馈；
- 硬件急停、驱动禁能、独立 watchdog 和安全认证。

这些组件与 Nav2 的接口必须逐项研究，不能仅凭存在 ROS topic 就认为集成成立。

## 7. 候选实现和决策原则

- 定位：优先评估经验证的厂商腿式 odometry；否则比较 Point-LIO/FAST-LIO2/VIO；GNSS 作为全局约束且严格质量门控。
- 感知：MID-360 主几何源，双目提供补充深度/语义；独立观测源保留来源标记。
- 环境表达：Nav2 2D Costmap 作为平面导航层；另行评估 elevation/grid_map/traversability 表达四足地形风险。
- 规划：Nav2 Planner/Controller 作为 baseline；将足式运动学、姿态、步态、落足约束通过可验证接口纳入，不假设差速模型适用。
- 执行：标准 Twist 只作为高层意图候选，必须验证 Lite3 速度接口语义；禁止直接映射为关节命令。
- 安全：软件碰撞监控不能代替物理急停和厂商安全链。

## 8. 验收边界

研究文档、源码追踪、静态测试、ROS 集成、仿真、真实 bag 回放、硬件只读台架、HIL、封闭场地运动是不同证据等级。每份结论标注证据层级，不跨级推断。项目当前硬件未采购，本文为系统架构研究基线。
