# 10 — Point-LIO 去 MRS 化与四足状态估计接口审计

- 审计日期：2026-11-25
- 专项分支：`nav`
- 证据等级：固定上游源码 E2；官方 ROS 包索引/文档 E1；本项目未构建、未运行、未实测
- 目的：判断 CTU-MRS Point-LIO 中哪些依赖属于算法核心、哪些只是 MRS UAV 工程封装；审计其 Odometry/TF 输出能否作为四足机器人融合定位输入；评估 GNSS/RTK 与本体里程计融合架构。
- 非目标：本轮不修改上游 Point-LIO，不直接引入 robot_localization，不冻结最终状态估计拓扑。

## 1. 固定上游基线

| 组件 | 固定版本 | 本轮审计内容 |
|---|---|---|
| CTU-MRS Point-LIO | `e7ff034328b263ab92b06252d91acba1dd969b6d` | `src/laser_mapping.cpp`、`include/estimator.h`、`CMakeLists.txt`、MID-360 preset |
| MRS Point-LIO Estimator Plugin | `b43af57300b43a29ab6bfa1c68a9442fb869cea1` | plugin source、package.xml、CMake、plugins.xml |
| MRS UAV State Estimators | `1ccd7283f964f78c558d09b78d3a4cc6c5e3bc42`（`ros2` branch head） | `StateGeneric` interface/implementation |
| MRS UAV Managers | `75d897af40543b9769911a4ebf8db3aae20f7586`（`ros2` branch head） | `StateEstimator` base interface |
| robot_localization | `fea7fc3dcad81c533121e4aa2fb1087a0565d8cc`（`jazzy-devel` branch head） | Jazzy EKF/UKF/GNSS fusion candidate |

上游入口：
- https://github.com/ctu-mrs/point-lio/tree/e7ff034328b263ab92b06252d91acba1dd969b6d
- https://github.com/ctu-mrs/mrs_point_lio_estimator_plugin/tree/b43af57300b43a29ab6bfa1c68a9442fb869cea1
- https://github.com/ctu-mrs/mrs_uav_state_estimators/tree/1ccd7283f964f78c558d09b78d3a4cc6c5e3bc42
- https://github.com/ctu-mrs/mrs_uav_managers/tree/75d897af40543b9769911a4ebf8db3aae20f7586
- https://github.com/cra-ros-pkg/robot_localization/tree/fea7fc3dcad81c533121e4aa2fb1087a0565d8cc

说明：MRS 两个 branch head 是本轮解析到的固定 SHA，不代表上游长期稳定 release。项目依赖必须以 commit 锁定，不得继续只写 `ros2` 浮动分支。

## 2. Point-LIO 中 MRS 依赖的实际边界

### 2.1 算法主体没有继承 MRS UAV estimator

`point_lio::PointLio` 直接继承 `mrs_lib::Node`，算法状态、ES-EKF、IMU 处理、点云预处理、ikd-Tree 和地图更新均在 Point-LIO 自己的源码中。其头文件引入的 MRS 设施主要是：

- `mrs_lib::Node`：节点基类与 node handle 访问；
- `mrs_lib::ParamLoader`：YAML/参数加载；
- `mrs_lib::PublisherHandler` / `SubscriberHandler`：ROS pub/sub 封装；
- `mrs_lib::ThreadTimer`：主处理循环定时器；
- `mrs_lib::TransformBroadcaster`、`AttitudeConverter`：TF 发布和姿态转换。

Point-LIO 的 CMake 还依赖 `backward_ros`（错误栈/诊断设施）及 ROS/PCL/Livox 等常规依赖。以上 MRS 包装层并非 LIO 数学模型或 ikd-Tree 算法的必要组成；原则上可用原生 `rclcpp::Node`、参数接口/YAML parser、publisher/subscription、callback group、timer/thread、`tf2_ros::TransformBroadcaster` 替换。

**结论：算法层去 MRS 化在架构上可行，但不是“删掉一个依赖”即可完成。** 需要保留线程安全、队列锁、回调组、SensorData QoS、transient-local 地图云 QoS、线程退出/join、参数默认值/必填校验、clock 类型和 TF 行为，并对替换前后做 rosbag 输出等价性测试。

### 2.2 MRS UAV plugin 是另一层可选集成，不是 Point-LIO 算法核心

`mrs_point_lio_estimator_plugin` 的两个 plugin 类只继承 `rclcpp::Node` 与 `mrs_uav_state_estimators::StateGeneric`，并通过 pluginlib 注册为 `mrs_uav_managers::StateEstimator`。其源文件没有订阅 Point-LIO 的 `nav_msgs/Odometry`，也没有把 Point-LIO 输出转换成四足状态。

固定版本的 `StateGeneric` 实现会：
- 订阅 MRS Hardware API 的 QuaternionStamped orientation 与 Vector3Stamped angular velocity；
- 加载 lateral / altitude / heading 子估计器；
- 发布 MRS UAV State、Odometry、attitude 等接口；
- 依赖 `mrs_msgs`、MRS managers、MRS libraries 和 UAV-specific state model。

因此，该 plugin 是 MRS UAV System 的插件适配壳/状态估计管理器扩展，不是一个通用 Point-LIO→Odometry bridge，也不是 Lite3/四足导航的必要组件。除非项目明确决定运行 MRS UAV System，否则不应把 `mrs_uav_managers`、`mrs_uav_state_estimators` 或该 plugin 加入本项目核心依赖。

## 3. Point-LIO Odometry / TF 输出契约审计

### 3.1 当前输出字段

源码 `PointLio::publish_odometry()` 设置：
- `header.frame_id = uav_name + "/point_lio_world"`；
- `child_frame_id = uav_name + "/point_lio_body"`；
- stamp 默认取 `lidar_end_time`；当 `publish_odometry_without_downsample=true` 时取当前点处理时间 `time_current`；
- pose 来自内部 EKF 状态的 position / orientation；
- linear twist 从世界系速度经姿态逆变换到 child/body frame；
- 可选发布同一 parent→child 的 TF，但默认配置 `publish/tf/fcu: false`。

`point_lio_body` 是 Point-LIO 内部状态所对应的 body/IMU 参考，不应未经外参核对就重命名或等同于机器人 `base_link`。MID-360 preset 中有 LiDAR→IMU 外参，但还必须另行确认 IMU→base_link 与机身安装变换。

### 3.2 明确发现的输出缺陷/风险

1. **Pose covariance 与 twist covariance 未赋值。** 在锁定的 `laser_mapping.cpp` 中未发现对 `odomAftMapped.pose.covariance` 或 `odomAftMapped.twist.covariance` 的写入。消息默认零值不是“零不确定度”的有效估计。不得将该 Odometry 直接交给 EKF 并把零 covariance 当作可信观测。
2. **默认 `use_imu_as_input=true` 时 angular twist 未写入。** `set_odomtwist()` 仅在 `use_imu_as_input == false` 分支赋值 `out.angular`；默认 YAML 为 true，因此 angular velocity 字段没有由当前分支更新。由于消息对象被复用，不能把该字段视为有效角速度。必须修复或在 adapter 中明确禁用该字段。
3. **输出 frame 不是通用机器人 frame。** parent/child 名称由 `uav_name` 拼接固定字符串生成；未证明 child 是 Lite3 base_link。pose 的参考点与安装外参必须经源码变换链和实测确认。
4. **TF 默认关闭。** Odometry 发布不等于 TF 已发布。若启用 `publish/tf/fcu`，Point-LIO 会发布 `point_lio_world → point_lio_body`；项目必须保证 TF 树只有一个权威发布者，避免与融合器/平台 odometry 重复广播同一 child。
5. **时间戳是扫描参考时间，不是消息发布时间。** 默认使用 `lidar_end_time`，符合 scan-end 输出意图；仍需与 IMU/LiDAR 同一硬件时间域、ROS clock 和下游 TF buffer 做 bag 验证。
6. **reset/relocalization 状态合同不完整。** 当前审计看到 bag loop 的 reset 路径会重置 IMU processor，但尚未证明完整 map/state/TF 连续性、跟踪失效检测、协方差膨胀和重定位语义。不可把“节点仍发布 Odometry”作为 tracking healthy 的证据。
7. **初始化协方差与 ROS 消息协方差是两回事。** 源码确实为内部 ESEKF 初始化矩阵 `P_init`，但发布函数没有把其状态子块映射到 ROS Odometry covariance；不能据此推断输出协方差已存在。

### 3.3 最低输出合同（项目 adapter 必须实现）

| 字段 | 最低要求 |
|---|---|
| `header.frame_id` | 明确的连续局部世界系，建议 `odom`；不把任意 LIO map 直接当全局 map |
| `child_frame_id` | 明确状态参考点；通过静态外参链连接到 `base_link` |
| `header.stamp` | scan-end 或明确估计时刻；同一 clock domain；不得用 wall-clock publish time 覆盖 |
| pose/twist | 明确 world/body 表达；单位 SI；角速度来源有效 |
| covariance | 由 estimator 内部协方差正确投影/变换，或经 bag/GT 标定得到保守值；禁止全零伪装确定 |
| TF | 由单一 authority 发布；清楚区分 `map→odom` 与 `odom→base_link` |
| health | tracking state、last lidar/imu stamp、queue age、drop count、innovation/residual、reset counter |
| failure | stale / no IMU coverage / tracking lost / clock jump / reset 时置无效并触发融合层降级 |

## 4. 去 MRS 化建议架构

建议采用分层 package，而不是把 MRS manager 整体搬入四足项目：

1. `robodog_lio_core`：保留 Point-LIO 算法、预处理、ESEKF、ikd-Tree；仅暴露纯 C++ estimator state / covariance / health API，不依赖 MRS UAV types。
2. `robodog_point_lio_ros`：原生 rclcpp component/node，订阅 Livox CustomMsg 与 IMU，发布 deskewed cloud、内部状态、诊断和原始 LIO Odometry；保留明确的时间合同与 QoS。
3. `robodog_state_estimation`：融合层/协调层，接收 LIO、腿式 odometry、IMU、RTK；负责 frame registry、quality gate、reset/relocalization 和 map/odom TF ownership。
4. `robodog_lite3_state_adapter`：仅在厂商确认数据语义后，将 SDK 的 base pose/velocity/joint/contact/IMU 变成标准 ROS 消息；不把 RobotStatus 或 IMU-only 积分伪装成完整 odometry。
5. `robodog_gnss_adapter`：将 ZED-F9P fix、RTK status、heading/基站状态、时间和 covariance 变成标准 GNSS 接口；保留 FIX/FLOAT/SINGLE 等质量状态，不仅凭 NavSatFix 存在就融合。

第一阶段优先做 ROS middleware 去 MRS 化，保持算法源码最小改动；第二阶段再评估拆出可单测的 LIO core。所有改动先在 `nav` 分支，且必须保留原始上游 commit、许可证审计和差异补丁。

## 5. robot_localization / RTK 融合评估

### 5.1 Jazzy 适配证据

ROS Index 当前将 `robot_localization` 列为 Jazzy 已发布包（Apache-2.0、维护中）；本轮固定审计其 `jazzy-devel` head：`fea7fc3dcad81c533121e4aa2fb1087a0565d8cc`。上游 `navsat_transform_node` 的接口是：
- 输入：`nav_msgs/Odometry`、带地理参考 yaw 的 `sensor_msgs/Imu`（或明确启用 use_odometry_yaw）和 `sensor_msgs/NavSatFix`；
- 输出：转换到机器人 world frame 的 GNSS `nav_msgs/Odometry`，可供 EKF/UKF 融合；
- 支持 UTM 或 local ENU、datum、可选 cartesian/world transform；
- GNSS 输出与 EKF 融合时对应输入的 `*_differential` 应为 false。

### 5.2 适合的职责与边界

`robot_localization` 可作为候选通用融合框架，但不是四足状态估计的全部：
- 适合融合有明确 frame/stamp/covariance 的 LIO Odometry、腿式里程计、独立 IMU、GNSS 转换 Odometry；
- 不直接解决点云 LIO、动态障碍、地形可通行性、足端接触估计或四足运动学约束；
- RTK 必须基于 NavSatFix status、position covariance、RTK FIX/FLOAT 状态和差分龄期建立质量门控；坏 fix 不能继续作为高置信观测；
- 单天线 GNSS 静止时不能提供机身 yaw。navsat_transform 需要地理参考 heading；不能把 LIO 启动时任意初始化的 yaw 当作全球航向；
- 若 Point-LIO 已使用 MID-360 内置 IMU，再把同一 IMU 原始数据作为独立测量送入普通 EKF，可能重复计算相关信息。除非建模相关性或明确分离估计量，否则不得简单“把所有传感器都加进去”。

### 5.3 当前推荐的候选拓扑（待数据验证）

- **局部连续状态：** 若 Lite3 厂商提供并验证腿式 odometry，优先比较它与 LIO 的漂移、延迟、frame 和 covariance；可用其中一个作主要 local odom，另一个作为有相关性意识的观测/诊断。若无可信腿式 odom，再以 LIO 为局部连续定位候选。
- **全局约束：** RTK 经 GNSS adapter + quality gate + navsat_transform 提供 map/global 位置约束；heading 由经标定的双天线 GNSS、磁航向/可靠绝对航向或 datum/已知方向初始化提供，不能假定 ZED-F9P 单天线本身给出 yaw。
- **融合器：** robot_localization EKF/UKF 是第一候选基线，不预先认定最终方案。先用 rosbag 评估时延、协方差、相关性、GNSS 跳变、失锁、LIO reset 和 frame 连续性；若需接触/滑移/足端约束或非高斯状态，后续评估专用 legged-state estimator / factor graph。
- **TF ownership：** 推荐融合器唯一发布 `odom→base_link`；全局融合/定位层唯一发布 `map→odom`；LIO 原始输出仅发布自身隔离 frame 或 Odometry，不与最终 TF 冲突。

## 6. 风险、Gate 与待验证项

- [ ] 读取 Point-LIO 实际 MID-360+IMU bag，核验状态参考点、姿态/速度坐标表达、stamp 与时钟域。
- [ ] 修复/验证默认 `use_imu_as_input=true` 时 angular twist 未赋值的问题；补 covariance 输出或 adapter 标定。
- [ ] 为内部 ESEKF covariance 到 ROS 6×6 pose/twist covariance 建立数学映射和单测，特别是状态索引、姿态扰动参数化和 body/world 速度变换。
- [ ] 明确 LIO tracking lost、reset、relocalization、map jump、clock jump 的状态机和下游融合处理。
- [ ] 固定 MRS 子模块递归依赖树与许可证/SBOM；不把 MRS UAV plugin 当成 LIO 算法依赖。
- [ ] 建立原始 Point-LIO 与原生 rclcpp wrapper 的 bag replay A/B，比较 odom、cloud、TF、频率、延迟和 CPU/RAM。
- [ ] 在 Jazzy x86_64 与 Orin ARM64 构建；Orin 做温度/功耗/内存/长时 soak。
- [ ] 获取 Lite3 厂商正式 legged odometry / body velocity / contact / timestamp / covariance 语义，决定腿式 odom 与 LIO 的主从关系。
- [ ] 对 robot_localization 做独立 GNSS quality gate、双 EKF 或其他拓扑试验；验证 GNSS heading、datum、RTK FIX→FLOAT→SINGLE 降级。
- [ ] 对相同 IMU 被 LIO 与 EKF 重复融合的相关性风险作定量处理。

## 7. 本轮结论

1. Point-LIO 算法层可以在保留 ESEKF/点云算法的前提下，用原生 rclcpp 替换 MRS middleware wrappers；需系统性保持 QoS、线程、时钟和参数语义。
2. MRS Estimator Plugin 是 MRS UAV System 插件，不是 Point-LIO 输出适配器；四足项目无须默认依赖该 plugin 或 MRS UAV managers。
3. 当前 Point-LIO Odometry 缺少 ROS covariance 输出；默认 `use_imu_as_input=true` 时 angular twist 也没有由发布函数赋值。它只能作为待修复/适配的 LIO 原始观测，不能直接当作最终 base_link 状态。
4. robot_localization 在 Jazzy 有维护中的发布包和 GNSS 转换链，是合理的第一融合基线，但必须先补齐 covariance、frame、heading、质量门控和相关性处理。
5. 本轮均为 E1/E2 上游证据；未在本项目编译或运行，未获取真实 MID-360/IMU bag，未完成 Lite3 接口验证、Orin 测试或实机验收。
