# 00 — Nav2 总体架构

## 1. 职责边界

Nav2 是 ROS 2 上的模块化移动机器人导航框架。它接收任务目标、位姿/速度、地图和障碍物表示，组织路径规划、路径跟踪、行为恢复，并输出运动速度命令。它不自动提供传感器驱动、可靠定位、障碍检测、机器人动力学模型、步态控制或安全认证。

系统分层：

1. Robot/Hardware：执行器、四足运动控制器、急停、驱动器。
2. Device Drivers：LiDAR、相机、GNSS、IMU、关节/足端状态驱动。
3. Robot Description：URDF/Xacro、robot_state_publisher、joint_states、TF。
4. Perception & State Estimation：点云/深度预处理、障碍物提取、SLAM/LIO/VIO、robot_localization、GNSS quality gate。
5. Navigation Environment：Map Server/SLAM、Global/Local Costmap、footprint。
6. Nav2 Task & Algorithm：BT Navigator、Planner、Controller、Smoother、Behavior、Waypoint/Route。
7. Command & Safety：Velocity Smoother、Collision Monitor、平台 command adapter、watchdog。
8. Application：目标点、巡检路线、跟随、任务状态和人机交互。

## 2. 核心服务器

| 组件 | 输入 | 输出/效果 | 职责 |
|---|---|---|---|
| BT Navigator | NavigateToPose / NavigateThroughPoses goal | 调度其他 action、反馈/结果 | 行为树执行引擎 |
| Planner Server | 起点、终点、Global Costmap | nav_msgs/Path | 托管全局规划插件 |
| Controller Server | Path、位姿/速度、Local Costmap | Twist/TwistStamped | 托管控制器、Goal/Progress Checker，并持有 local costmap |
| Smoother Server | Path | 平滑 Path | 路径后处理 |
| Behavior Server | Spin/BackUp/Wait 等请求、Costmap/TF | 行为结果/运动命令 | 恢复和原子行为 |
| Costmap 2D | Map、Scan、PointCloud2、TF、footprint | 代价网格 | 多层环境表示 |
| Map Server | 地图 YAML/image | /map | 静态地图加载/保存 |
| Lifecycle Manager | lifecycle node 列表 | configure/activate/deactivate/shutdown | 生命周期编排 |
| Velocity Smoother | 上游速度、可选 odom | 平滑速度 | 限速、加速度、deadband、timeout |
| Collision Monitor | 传感器观测、速度命令 | 限制/停止后的命令 | 绕过 Costmap/Planner 的额外 CPU 级监控 |
| Waypoint Follower | Pose 序列 | 顺序执行 waypoint | waypoint task plugin |
| Route Server（可选） | 路网和目标 | Route | 预定义拓扑路网 |

服务器管理 Action、生命周期、参数和资源；插件通过 pluginlib 加载算法。BT 或应用可按场景选择插件 ID。

## 3. 系统逻辑图

    应用层 → BT Navigator
                 ├→ Planner Server → Planner Plugin → Path
                 ├→ Smoother Server → Smoother Plugin → Path
                 ├→ Controller Server → Controller Plugin → cmd_vel
                 └→ Behavior Server → Recovery/Primitive Behavior
    cmd_vel → Velocity Smoother → Collision Monitor → 四足 Adapter → Locomotion
    Sensors → Drivers → Perception / State Estimation → TF / Costmaps → Planner & Controller

## 4. 两类循环

**任务循环**：应用发起导航 Action → BT 检查条件 → 规划/重规划 → 控制任务 → 监测成功/失败 → 恢复/重试/取消 → 返回结果。由 BT tick 和 action 状态驱动。

**运动循环**：Controller 周期读取 Path、当前状态和 Costmap → 计算控制量 → Velocity Smoother 限幅/插值 → Collision Monitor 检查 → adapter 转换底层命令 → 控制器执行。底层步态闭环频率通常高于 Nav2，Nav2 不应直接闭环控制每个关节。

## 5. 关键数据对象

- Pose/Goal：目标位置和朝向，常用 PoseStamped。
- Path：nav_msgs/Path，由 Planner 生成，Controller 消费。
- Costmap：网格代价场，表达未知、空闲、障碍和膨胀代价。
- Odometry：nav_msgs/Odometry，含 pose/twist 与协方差；frame_id/child_frame_id 必须正确。
- TF：时变坐标变换，连接地图、里程计、机身和传感器 frame。
- Observation：LaserScan / PointCloud2 / Range 等。
- Velocity command：geometry_msgs/Twist 或 TwistStamped；上下游消息类型必须一致。

## 6. 与本项目对应

MID-360 为主几何源，CustomMsg 需经转换/处理成为 Costmap 可消费的 PointCloud2。双目通过 image_pipeline/stereo_image_proc 形成深度点云，作为独立补充源。GNSS 经 quality gate、时间/坐标处理和 robot_localization/navsat_transform 进入全局定位。本体 IMU 与正式腿式 odometry（若验证通过）提供 local state；否则继续评估 Point-LIO 等路线。Nav2 速度命令必须经过平台 adapter。

## 7. 架构验收前提

TF 完整且唯一发布者；odom 连续、时间戳可信、协方差有来源；Costmap 可解释且观测不过期；Planner/Controller 与真实运动学匹配；命令链有 watchdog 和独立停止机制；每阶段有可复现测试和日志。

参考：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/ 、https://docs.nav2.org/jazzy/getting_started/navigation_concepts/
