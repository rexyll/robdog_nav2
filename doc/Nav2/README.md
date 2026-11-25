# Nav2 专项研究与工程设计目录

研究基线：ROS 2 Jazzy / Nav2 Jazzy；兼顾 Humble 兼容性。项目候选平台为 Lite3 Venture + MID-360 + 双 RGB 被动双目 + ZED-F9P + Jetson AGX Orin。设备尚未采购，接口和参数须以供应商资料及实测为准。

## 目标与范围

本目录梳理 Nav2 的架构、生命周期、任务执行、数据生产者/消费者、Topic/Action/Service/TF、Costmap、算法插件、硬件接口、四足适配、安全、调试和验证。

Nav2 不是完整机器人系统。它主要消费地图、定位/里程计、TF、障碍物观测和运动学约束，执行任务编排、全局规划、局部控制与恢复。传感器驱动、SLAM/定位、视觉/LiDAR 感知、底层步态和硬件安全由周边模块负责。

## 文档地图

| 文档 | 主要问题 |
|---|---|
| 00 总体架构 | 层次、模块边界、节点/服务器/插件 |
| 01 生命周期与任务流程 | bringup、Lifecycle、NavigateToPose 时序 |
| 02 数据生产者与消费者 | Topic、Action、Service、TF、消息与数据质量 |
| 03 感知与 Costmap | 预处理、Obstacle/Voxel/Static/Inflation、marking/clearing |
| 04 算法与插件选型 | Planner、Controller、Smoother、Behavior、Localization |
| 05 硬件与软件模块 | 传感器、计算平台、驱动、状态估计、依赖 |
| 06 四足集成与安全 | 运动学、cmd_vel adapter、步态、碰撞监控、降级 |
| 07 验证、调试与路线 | 分层验证、工具、指标、实施顺序 |

## 核心闭环

    上层任务 / 用户
       │ NavigateToPose Action
       ▼
    BT Navigator ── BT XML / Blackboard / Recovery
       ├── Planner Server → Global Planner → Path
       ├── Smoother Server → Smoothed Path（可选）
       ├── Controller Server + Local Costmap → cmd_vel
       └── Behavior Server → Spin / BackUp / Wait 等
                                  │
                                  ▼
    Velocity Smoother → Collision Monitor → 四足命令适配器
                                  │
                                  ▼
                         厂商步态/运动控制器
                                  │
                                  ▼
                               机器狗
                                  │
           IMU / leg odom / LiDAR / camera / GNSS
                                  │
                                  ▼
                 State Estimation + Perception + TF
                                  │
                       Costmaps / map / odom
                                  └────────→ Nav2

## 阅读顺序

先读 00、01 建立整体模型；再读 02、03 理解数据如何进入 Costmap 和控制器；读 04 选择算法；05、06 落到四足平台；最后按 07 验证。

## 适用范围与禁止推断

- 文档说明 Nav2 通用机制和项目建议架构；实际节点、话题、频率、QoS、插件和参数以最终 launch/YAML 与运行图为准。
- Jazzy 官方文档是主要基线。Humble/Jazzy 参数、默认 topic、消息类型和行为可能不同。
- Lite3 正式腿式 Odometry、速度命令接口、横移能力、最大速度/加速度、制动距离和 SDK host 分工仍待供应商确认。
- Collision Monitor 是 CPU 软件监控，不是安全认证急停。
- Nav2 2D Costmap 不等于四足落足规划或完整三维可通行性规划。

## 官方资料

- Nav2 Jazzy：https://docs.nav2.org/jazzy/
- Navigation Concepts：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/
- Navigation Servers：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/
- Behavior Trees：https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/
- Configuration Guide：https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/
- State Estimation：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/
- Environmental Representation：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/environmental_representation/
- TF setup：https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/transformation/setup_transforms/
- REP-105：https://www.ros.org/reps/rep-0105.html
- REP-103：https://www.ros.org/reps/rep-0103.html
- Source：https://github.com/ros-navigation/navigation2

本目录首轮内容属于架构研究/设计基线，不表示已完成 ROS 启动、仿真或实机验收。进度以仓库根目录 TaskPlan.md 为准。
