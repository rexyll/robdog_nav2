# 02 — 导航子系统清单、上游基线与证据登记

更新日期：2026-11-25。本文记录 N0 阶段首轮公开仓库检查和源码基线快照。分支 HEAD 是可变引用；以下 commit SHA 才是本次研究复核锚点。

## 1. 证据等级

| 等级 | 定义 | 可支持的结论 |
|---|---|---|
| E0 | 名称/搜索线索 | 仅表示候选存在，不能支持技术结论 |
| E1 | 官方仓库、文档或源码已读取 | 可说明源码/文档明确写出的事实 |
| E2 | 固定版本源码 + 接口/依赖审计 | 可说明特定版本的模块职责、接口和依赖 |
| E3 | 可复现静态/单元/构建测试 | 仅支持测试覆盖范围内的软件结论 |
| E4 | ROS 集成/运行图/真实数据回放 | 支持对应环境和数据下的运行结论 |
| E5 | 目标硬件只读台架 | 支持硬件接入、时间/带宽/数据质量结论 |
| E6 | HIL/受控实机 | 支持实际闭环场景覆盖范围内的结论 |

禁止跨等级推断。研究文档本身不等于 E3 以上证据。

## 2. 术语基线

- **Navigation Module**：从任务目标、环境和机器人状态到安全运动执行反馈的完整闭环。
- **Mission / Task Management**：任务优先级、目标/航点/区域、暂停/取消、状态机和任务恢复。
- **Localization**：机器人在给定参考坐标系中的位姿估计；需区分 local odometry、global localization 和 map alignment。
- **Odometry**：连续局部运动估计，常以 odom→base_link 表达；不自动等同全局定位。
- **SLAM**：同时估计轨迹与地图；需区分建图、定位、回环和重定位模式。
- **Perception**：从传感器数据提取障碍、地面、动态目标、语义和几何属性。
- **Costmap**：面向导航规划/控制的代价表达；Nav2 常用 2D 网格，不能代表完整 3D 地形。
- **Elevation Map**：空间网格中的高度及其不确定度/统计信息表达，具体字段依实现。
- **Traversability**：结合地形几何、机器人能力和风险得到的可通行程度，不等同于障碍物二值分类。
- **Global Planner / Local Planner / Controller**：分别处理长距离路径、局部轨迹或实时运动控制；不同框架术语可能不同。
- **Behavior Tree (BT)**：任务级行为编排机制；不是定位算法，也不是运动控制器。
- **Command Adapter**：将导航层运动意图转换为平台明确支持的命令，并执行模式、范围、超时和反馈校验。
- **Safety Supervisor**：独立监控命令、状态和安全条件的组件；不能与 Nav2 的普通恢复行为混为一谈。
- **Producer / Consumer**：生产数据的节点/设备与消费该数据的节点/算法；每条边都要定义接口契约和失效策略。

## 3. 首轮上游仓库与固定源码锚点

以下 commit 来自 2026-11-25 对公开 GitHub ref 的读取。版本 pin 仅表示本次研究检查点，不表示已经完成构建、许可证合规或目标硬件验证。

| 子系统 | 上游仓库 | 分支快照 | Commit SHA | 首轮核验事实 | 当前判断/下一步 |
|---|---|---|---|---|---|
| ROS 2 Navigation Framework | https://github.com/ros-navigation/navigation2 | jazzy | c7046550547d382a5186d106b0895eda02485d7c | 已读取 nav2_bringup/package.xml；包版本 1.3.14、Apache-2.0；依赖包含 navigation2、slam_toolbox、ros_gz_sim/bridge 等；已读取 BtNavigator 源文件，节点继承 LifecycleNode 并声明 global_frame、robot_base_frame、odom_topic 等参数 | E2 源码入口；继续追踪 action server、BT navigator plugins、Planner/Controller Server 和消息接口 |
| State Estimation / Sensor Fusion | https://github.com/cra-ros-pkg/robot_localization | jazzy-devel | fea7fc3dcad81c533121e4aa2fb1087a0565d8cc | 已读取 package.xml；版本 3.8.4，包含 nav_msgs、sensor_msgs、geometry_msgs、tf2、geographic_msgs 等依赖 | E1/E2 初始；需追踪 EKF/UKF、navsat_transform、输入融合掩码、时间延迟与 TF 发布 |
| 2D SLAM | https://github.com/SteveMacenski/slam_toolbox | jazzy | f84dad92a6579aeff4fe0c2590bbc74d3348e5af | 已读取 package.xml；版本 2.8.5、LGPL；依赖含 pluginlib、message_filters、tf2、Ceres、SuiteSparse、TBB、Lifecycle 相关包 | E1/E2 初始；核验 async/sync mapping、localization、serialization、scan matching 和 ARM64依赖 |
| Camera Processing | https://github.com/ros-perception/image_pipeline | jazzy | 6c3df3099bc7b1ec92215719215c8eefd0d3aa69 | README 将其定位为 camera driver 原始图像与上层视觉处理之间的处理管线；提及 Jetson 可评估 Isaac ROS image pipeline | E1；逐包审计 image_proc、stereo_image_proc、depth_image_proc、camera_info_manager 和硬件加速替代路线 |
| Multi-layer Grid Map | https://github.com/ANYbotics/grid_map | ros2 | 0ece1b53ca03199527d0d42913cf2567d0d6a913 | 已读取 grid_map/package.xml；meta-package 版本 2.5.0、BSD，依赖 grid_map_core/ros/msgs/pcl/cv/costmap 等多个包 | E1/E2 初始；检查 Jazzy 构建状态、数据结构、filters、转换和 Costmap 接口 |
| Elevation Mapping | https://github.com/ANYbotics/elevation_mapping | master（仓库元数据快照） | 未固定 | GitHub 仓库元数据标记 archived=true；尝试读取根 package.xml 返回 404，不能据此确认 ROS 2 包结构 | 风险/待核验；必须调查归档原因、最后可用 ROS 2 分支、fork/替代实现，不作为当前依赖承诺 |
| Traversability | 候选名称曾指向 ANYbotics traversability_estimation | 未确认 | 未固定 | 本次仓库 API 返回 404；不将该仓库视为当前可用上游 | 待寻找可验证的替代项目/论文/实现；明确算法输入输出和维护状态 |
| 四足底层控制 | Lite3 厂商 SDK/接口 | 厂商版本待确认 | 未固定 | 项目已有 SDK 字段审计指出公开 RobotStatus 不含可信 base xyz/linear velocity/covariance；SDK_MODE 可能取得控制权 | 硬件/授权 Gate；禁止根据公开状态包推定 leg odometry 或运动接口 |
| LiDAR Driver | https://github.com/Livox-SDK/livox_ros_driver2 | 项目已锁定版本 | 13eb05e4e6dd7a765b934d0c5fd6236676a57b49 | 根 TaskPlan 已记录 driver v1.2.6 与 SDK2 v1.3.1 的既有 pin 和 x86_64 build 证据 | 复用现有审计；继续确认 Jazzy/ARM64/Orin、CustomMsg 字段、设备时钟和真实 MID-360 |
| Control Framework | https://github.com/ros-controls/ros2_control | jazzy | c3f7a299788dace3d3f17046358e0439be4f54ef | 已读取仓库 ref，确认 jazzy 分支存在并固定 SHA；本轮尚未读取 controller/hardware interface 源码 | E1；研究它是否适合本体状态/设备管理边界，不能默认用它替代厂商步态控制器 |

## 4. 本项目已有硬件/软件证据应如何复用

- Lite3：复用 TaskPlan 中 sdk_deploy/RobotStatus 字段审计、供应商问题矩阵和安全 Gate；所有未证实字段保持未确认。
- MID-360：复用 livox_ros_driver2 / Livox-SDK2 固定版本、CustomMsg/timebase/offset_time 审计、时间审计工具和 Dynamic03 只读数据记录。
- Stereo：复用 See3CAM 型号、镜头候选、硬件 trigger、双路 USB3 带宽和标定待办；不得把普通 RGB 相机等同于已经验证的深度传感器。
- GNSS：复用 ZED-F9P driver/UBX 四路历元配对与 RTK quality gate；普通 STATUS_FIX 不等于 RTK Fixed。
- 定位数据：Dynamic03 GT 与 raw-fix 高度同源且无 RTK Fixed 证据，禁止用作独立 APE/RPE ground truth。
- 计算平台：区分 x86_64 CI、通用 ARM64 build、Jetson Orin 原生 build、Jetson 实时性能测量。

## 5. 首轮源码证据记录

### Nav2
- 文件：nav2_bringup/package.xml
- 固定版本：ros-navigation/navigation2 @ c7046550547d382a5186d106b0895eda02485d7c
- 观察：Nav2 bringup 包的执行依赖中包含 slam_toolbox 与 Gazebo 相关包；这是 bringup/示例依赖，不应推断 Nav2 本体包含 SLAM 或 Gazebo。
- 文件：nav2_bt_navigator/src/bt_navigator.cpp
- 观察：BtNavigator 是 LifecycleNode；默认 global_frame=map、robot_base_frame=base_link、odom_topic=odom，且配置 Navigator plugin loader。参数默认值是源码默认，不代表本项目最终配置。

### robot_localization
- 文件：package.xml
- 固定版本：cra-ros-pkg/robot_localization @ fea7fc3dcad81c533121e4aa2fb1087a0565d8cc
- 观察：包依赖 nav_msgs、sensor_msgs、geometry_msgs、tf2 和 geographic_msgs，表明其定位融合与地理坐标转换会涉及这些 ROS 接口；具体融合数学与 TF authority 仍需读取实现源码。

### slam_toolbox
- 文件：package.xml
- 固定版本：SteveMacenski/slam_toolbox @ f84dad92a6579aeff4fe0c2590bbc74d3348e5af
- 观察：除 ROS 消息和 TF 外，还依赖 Ceres、SuiteSparse、TBB 等优化/数值依赖；部署成本和 ARM64 构建必须纳入评估。

### image_pipeline
- 文件：README.md
- 固定版本：ros-perception/image_pipeline @ 6c3df3099bc7b1ec92215719215c8eefd0d3aa69
- 观察：项目将 image_pipeline 定位为相机原始图像与高层视觉处理之间的中间处理管线，并提示 Jetson 用户评估 Isaac ROS 加速路径。需逐包验证 Jazzy API、实际相机驱动和 GPU/CPU 资源。

### grid_map
- 文件：grid_map/package.xml
- 固定版本：ANYbotics/grid_map @ 0ece1b53ca03199527d0d42913cf2567d0d6a913
- 观察：meta-package 组合多个 grid_map 子包；后续要分别核验核心数据结构、ROS 接口、PCL/CV 转换、Costmap 适配和维护状态。

## 6. 本阶段尚未完成的源代码审计

1. Nav2 action→BT→Planner→Controller→command 的完整调用链和失败路径。
2. robot_localization EKF/UKF/navsat_transform 的消息输入、融合掩码、时间处理、TF 发布与 reset 行为。
3. slam_toolbox mapping/localization 模式、scan matcher、loop closure、pose graph、serialization。
4. image_pipeline stereo/depth 处理链、CameraInfo/同步策略、QoS 和 Jetson 资源。
5. grid_map 数据布局、filters、elevation 输入输出和 Costmap 转换。
6. elevation_mapping 的维护/归档原因及可用替代；traversability 可靠上游的定位。
7. Point-LIO/FAST-LIO2、Livox driver、ZED-F9P driver 的 ROS 2 Jazzy/ARM64/Jetson 适配证据。
8. Lite3 SDK 的只读遥测、leg odometry、运动接口、SDK 权限、E-stop 和反馈接口。
9. 每个节点的 executor/callback group、线程模型、实时性、QoS 和数据龄期。

## 7. 参考链接

- Nav2 package manifest: https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bringup/package.xml
- Nav2 BT Navigator source: https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bt_navigator/src/bt_navigator.cpp
- robot_localization manifest: https://github.com/cra-ros-pkg/robot_localization/blob/fea7fc3dcad81c533121e4aa2fb1087a0565d8cc/package.xml
- slam_toolbox manifest: https://github.com/SteveMacenski/slam_toolbox/blob/f84dad92a6579aeff4fe0c2590bbc74d3348e5af/package.xml
- image_pipeline README: https://github.com/ros-perception/image_pipeline/blob/6c3df3099bc7b1ec92215719215c8eefd0d3aa69/README.md
- grid_map manifest: https://github.com/ANYbotics/grid_map/blob/0ece1b53ca03199527d0d42913cf2567d0d6a913/grid_map/package.xml

本文为首轮上游基线，不代表已经完成完整依赖、许可证、安全、构建和实机适配审计。
