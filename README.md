# RobDog Nav2 — 四足机器狗导航与实时避障

> 使用 **LiDAR + RTK GNSS + 两台普通 RGB 2D 相机（被动双目）**，基于 ROS 2 / Nav2 构建四足机器狗的定位、导航、动态障碍物检测和实时避障系统。

本仓库是一个新项目，采用文档先行、接口先行、分阶段验证的方式推进。当前先建立系统设计和可验证的实施计划；在机器狗型号、传感器型号、底层运动控制接口和实际 ROS 2 发行版确认之前，不把假设写成已验证的硬件事实，也不伪造可直接上机的标定值或参数。

## 项目目标

开发面向四足机器狗的完整导航闭环：

1. 获取 LiDAR、RTK、左右 RGB 相机、IMU 与机器狗底层里程计数据。
2. 形成连续、可信的本体状态估计与统一 TF 坐标树。
3. 室外使用 RTK 提供全局地理定位；本地运动估计保持连续，不因 GNSS 跳变而抖动。
4. 利用 LiDAR 和双目视觉深度识别静态、动态及部分低矮/高处障碍物。
5. 通过 Nav2 全局规划、局部控制、行为树和恢复行为执行路径导航。
6. 通过 Collision Monitor 做独立于 costmap/controller 的快速减速/停止逻辑。
7. 将 Nav2 的速度指令安全地转换为机器狗底层步态/运动控制接口。
8. 通过仿真、台架、封闭场地和受控户外测试逐层验证。

## 已确定的初始路线

- **主要导航框架：** ROS 2 Nav2。
- **主要几何避障传感器：** LiDAR。具体是 2D 还是 3D、扫描平面/垂直视场、点云或 LaserScan 输出，待硬件确认。
- **全局定位：** RTK GNSS。需结合 IMU 航向和连续本体里程计，RTK 单天线位置本身不能可靠提供静止时航向。
- **视觉：** 两台普通 RGB 2D 相机组成被动双目系统，提供视差/深度点云以及目标语义识别；它不是天然等同于工业 RGB-D 相机。
- **定位融合：** 以 `robot_localization` 双 EKF + `navsat_transform_node` 作为首选参考架构，具体状态变量、协方差和数据源须基于底层实际输出标定。
- **障碍物融合：** LiDAR 与双目点云保留独立观测源，分别做时间戳、TF、范围、高度、置信度和自体过滤，再接入 Nav2 costmap；不在未经验证的情况下简单拼接成一个点云。
- **局部避障：** Nav2 local costmap + controller + Collision Monitor；机器狗底层控制器必须提供限速、急停/停止、命令超时保护。
- **地形可通行性：** Nav2 2D costmap 不足以描述所有坡度、台阶、沟坎、悬崖和落足风险。此类能力需要地形分析/可通行性层及必要的自定义 costmap 插件或底层落足规划配合。

## 当前 ROS 2 基线建议

建议先以 **Ubuntu 24.04 + ROS 2 Jazzy** 建立软件基线：Jazzy 是 LTS，官方发布资料说明支持至 2029 年 5 月，并将 Ubuntu 24.04 amd64/arm64 列为 Tier 1 平台。若机器狗厂商 SDK、驱动或计算机系统要求 Humble、其他发行版或定制镜像，则先做兼容性评估，再锁定版本。发行版尚未由真实硬件验证，当前属于项目基线建议而非最终锁定。

## 系统数据流（概念图）

```mermaid
flowchart TB
  subgraph Sensors["传感器与底层状态"]
    L["LiDAR → /scan 或 /points"]
    C1["左 RGB 相机"]
    C2["右 RGB 相机"]
    GNSS["RTK GNSS → NavSatFix"]
    IMU["IMU → Imu"]
    LEG["腿式里程计 / 机身状态估计"]
  end

  subgraph Perception["传感器处理"]
    STEREO["双目标定、同步、校正、视差"]
    CLOUD["双目深度 / PointCloud2"]
    LPROC["LiDAR 时间补偿 / 过滤 / 自体过滤"]
    VPROC["视觉点云置信度、距离与高度过滤"]
    SEM["目标检测 / 跟踪（语义辅助）"]
  end

  subgraph Localization["状态估计与定位"]
    EKFL["Local EKF：连续 odom → base_link"]
    NAVSAT["navsat_transform：GNSS → 局部笛卡尔坐标"]
    EKFG["Global EKF：GNSS + local odom → map → odom"]
  end

  subgraph Nav["Nav2 导航"]
    GLOBAL["Global Costmap + Planner"]
    LOCAL["Local Costmap：LiDAR + Stereo + Inflation"]
    CTRL["Controller / Behavior Tree"]
    CM["Collision Monitor"]
  end

  BASE["cmd_vel adapter → 四足步态/运动控制器"]
  L --> LPROC --> LOCAL
  C1 --> STEREO
  C2 --> STEREO
  STEREO --> CLOUD --> VPROC --> LOCAL
  STEREO --> SEM
  GNSS --> NAVSAT
  IMU --> EKFL
  IMU --> NAVSAT
  LEG --> EKFL
  EKFL --> NAVSAT
  EKFL --> EKFG
  NAVSAT --> EKFG
  EKFG --> GLOBAL
  EKFG --> LOCAL
  GLOBAL --> CTRL
  LOCAL --> CTRL
  SEM -.语义层/可选插件.-> LOCAL
  CTRL --> CM --> BASE
```

## 传感器分工

| 设备 | 核心职责 | 不应承担的职责 |
|---|---|---|
| LiDAR | 稳定几何测距、局部占据/障碍物标记与清除 | 单独完成全部语义识别；单线 2D LiDAR 不能看见扫描平面以外所有障碍物 |
| RTK GNSS | 室外绝对位置、地理航点与全局漂移约束 | 近场避障；静止单天线 RTK 不提供可靠航向；非固定解不能当作厘米级真值 |
| 双目 RGB | 语义检测、视觉深度、补充 LiDAR 视场/高度盲区 | 在透明/反光/无纹理/弱光场景中保证深度；单独承担安全急停 |
| IMU + 腿式里程计 | 连续姿态、速度与本体运动估计 | 未验证的足端滑移、腾空/落地状态不能被当成无误差里程计 |

## 关键架构约束

- **定位 TF：** `map → odom → base_link → {lidar_link, left_camera_optical_frame, right_camera_optical_frame, imu_link, gnss_link}`。每条 TF 只能有一个权威发布者。
- **平滑性：** local EKF 优先维持连续的 `odom → base_link`；GNSS 修正放在 global EKF 中形成 `map → odom`，避免 RTK 更新直接造成局部速度/姿态突跳。
- **相机：** 两相机必须刚性安装、视场重叠、标定准确并尽可能硬件同步；发布正确的 CameraInfo 和 optical frame。
- **四足机身运动：** 机身俯仰/横滚、步态周期、冲击和相机/LiDAR 扫描运动都必须纳入时间同步、运动补偿和 TF 设计。
- **导航运动模型：** 先确认机器狗是差速式、全向式还是具有受限横移/旋转能力。Nav2 controller 的 holonomic、速度/加速度约束必须与底层真实能力一致。
- **命令链路：** `Nav2 → velocity smoother → collision monitor → cmd_vel adapter → robot controller`。最终拓扑与 remap 在接口确认后锁定；底层必须有命令 watchdog 和硬件级停止手段。
- **失效策略：** RTK 降级、传感器超时、TF 缺失、深度无效、定位协方差超限、控制器离线时，必须触发限速、停车或导航降级，不能沿用过期数据继续盲行。
- **安全：** Nav2 Collision Monitor 是软件级 CPU 监测，不是安全认证设备。人机协作/公共环境的安全完整性必须另行评估硬件急停、独立安全扫描器和执行机构制动能力。

## 文档导航

- [系统架构与数据流](docs/architecture/system_architecture.md)
- [传感器接入与融合](docs/architecture/sensor_fusion.md)
- [Nav2 导航、Costmap 与实时避障](docs/navigation/nav2_navigation.md)
- [坐标系、定位与 TF 设计](docs/localization/frames_and_localization.md)
- [双目 RGB 深度与障碍物识别](docs/perception/stereo_obstacle_perception.md)
- [验证、安全与失效降级](docs/validation/safety_and_validation.md)
- [硬件/软件接口待确认清单](docs/interfaces/hardware_interface_checklist.md)
- [ROS 2 Workspace 与 package 规划](docs/implementation/ros2_workspace_and_packages.md)
- [导航与感知验收测试矩阵](docs/testing/acceptance_test_matrix.md)
- [实施任务计划 TaskPlan](TaskPlan.md)
- [决策记录 ADR-0001](docs/decisions/ADR-0001-sensor-roles-and-localization.md)
- [前序讨论归档](docs/discussion/2026-11-24-stereo-camera-discussion.md)
- [参考资料](docs/references.md)
- [源代码目录说明](src/README.md)
- [配置目录说明](config/README.md)
- [Launch 目录说明](launch/README.md)

## 当前阶段状态

<badge>阶段 A：需求与硬件基线核实</badge> 尚未完成。当前已建立架构与任务计划文档；尚未完成具体传感器驱动、标定、融合参数、Nav2 YAML、控制适配器和真实机器狗测试。详见 [TaskPlan.md](TaskPlan.md)。

## 重要外部参考

- [Nav2 Jazzy：Mapping and Localization](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization.html)
- [Nav2 Jazzy：Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/collision_monitor/configuring-collision-monitor-node.html)
- [Nav2：GPS Navigation Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)
- [robot_localization：navsat_transform_node](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/navsat_transform_node.rst)
- [Nav2：TF Setup](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)
- [ROS 2 Jazzy release](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released)
