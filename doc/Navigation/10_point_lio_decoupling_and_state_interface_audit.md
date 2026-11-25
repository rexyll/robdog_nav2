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


## 8. Point-LIO ESEKF 协方差：已确认可访问，但尚未发布

本轮进一步固定检查 Point-LIO `include/estimator.h` 与其 vendored IKFoM ESEKF：

- `PointLio` 同时持有 `kf_input`（24 维）和 `kf_output`（30 维）滤波器对象。
- IKFoM ESEKF 暴露 `get_P() const`，返回内部 covariance；并非算法本身没有协方差。
- 默认 `config/default.yaml` 明确 `use_imu_as_input: true`，因此正常 MID-360 preset 继承默认输入模型时，实际状态与 covariance 来源是 `kf_input`，不是 `kf_output`。
- `kf_input` 的状态字段声明顺序为 position、orientation、LiDAR-IMU rotation、LiDAR-IMU translation、velocity、gyro bias、accel bias、gravity；状态 covariance 初始化为 24×24，且源码显式对 6–11、15–20、21–23 等块设置初始值。按该声明顺序，位置/姿态位于前 6 个误差状态维度，速度位于 12–14；进一步固定 IKFoM MTK manifold：SO(3) 的 `boxplus/oplus` 均为右乘 `R ← R·Exp(δθ)`，`boxminus` 使用 `Log(R_otherᵀ·R)`，因此旋转误差是右侧局部切空间表达；S2 gravity 的 `DOF=2`、`DIM=3`，而该 ESEKF 使用 24 维 ambient covariance。故 P 的 gravity 末尾 3×3 块不是可直接当作普通 3D 向量 covariance 的独立自由度，导出时必须按 S2 约束和当前 gravity 方向投影到切平面。状态字段顺序与 `P_init` 块设置共同支持 position 0–2、SO(3) 3–5、外参旋转 6–8、外参平移 9–11、velocity 12–14、gyro bias 15–17、accel bias 18–20、gravity ambient block 21–23；但仍须通过编译期/运行期索引断言和单测锁定该分支的 P 坐标映射。
- `publish_odometry()` / `set_posestamp()` / `set_odomtwist()` 未读取 `kf_input.get_P()`，也没有将任何内部 covariance 写入 ROS Odometry 的 pose/twist covariance。因此应修正此前可能产生的误解：**Point-LIO 内部确实维护 covariance，缺陷在于该 ROS2 输出路径没有导出它。**

### 8.1 不允许直接截取 6×6 对角块作为最终 covariance

建议 adapter 从内部 ESEKF 暴露经过版本锁定的状态快照 API，至少返回 state、完整 P、时间戳、状态模型类型（input/output）、外参估计状态与 tracking health。再根据已验证的误差状态定义构造选择/变换 Jacobian：

- IMU/LIO 参考点 pose covariance：从 P 中提取 position 与 SO(3) error 子块，并保留二者之间的 cross-covariance。
- 速度 covariance：提取 velocity 子块，并明确它在 world 还是 body 表达。
- 将 LIO/IMU 参考点转换到 `base_link`：若存在固定 lever arm，pose covariance 需要用 SE(3) Jacobian/Adjoint 传播；不能仅复制原 6×6 块。若角速度参与从 IMU 原点换算到 base_link 原点的线速度，还需要 `v_base = v_imu + ω × r_imu_to_base` 及其协方差/交叉项。
- SO(3) 的局部误差坐标与 ROS Odometry 采用的姿态参数化不应想当然地视为同一坐标。必须确认 IKFoM 的 left/right perturbation、误差注入和重置 Jacobian，再将姿态误差协方差转换到 ROS 约定。
- 发布时必须验证对称性、有限值、半正定性、单位、frame、stamp 和误差状态顺序；不满足时置健康状态 invalid，不得用全零或任意常数掩盖。

## 9. robot_localization Jazzy 源码行为与 GNSS 输入安全 Gate

固定版本：`cra-ros-pkg/robot_localization jazzy-devel` SHA `fea7fc3dcad81c533121e4aa2fb1087a0565d8cc`（package.xml version 3.8.4，Apache-2.0）。

### 9.1 EKF covariance 行为

- EKF prediction 使用 `P = F P Fᵀ + Δt Q`；measurement correction 使用 Joseph form 更新 covariance。
- measurement update 根据每个输入的 update vector 只融合选定状态维度；NaN/Inf 测量会被排除。
- 对 measurement covariance 的负对角元素取绝对值；小于 `1e-9` 的对角元素会被提升到 `1e-9` 以保持数值稳定。
- **因此全零 Odometry covariance 不会被理解为“未知/无效”，而可能变成极小观测噪声，令 EKF 过度信任该输入。** 这使 Point-LIO covariance 缺失成为融合阻断项，而非可忽略的消息字段问题。
- `sensor_timeout` 只使滤波器在没有新测量时执行 prediction，不会替项目判断某一路 LiDAR/GNSS 是否仍有有效观测，也不是 RTK FIX/FLOAT 质量门控。
- `smooth_lagged_data` 可在收到时间戳落后于最新滤波时刻的测量时回滚到历史状态并重放后续测量；启用时 `history_length` 必须覆盖最大传感器/计算延迟。Point-LIO 输出应使用测量参考时间，而不是回调到达或发布时间；真实延迟分布要由 bag 统计后配置。
- `two_d_mode` 会强制将 Z、roll、pitch 及相关速度/加速度置零并修改 covariance；四足机器人在斜坡、台阶、崎岖地形运行，不能将其作为默认模式。

### 9.2 navsat_transform_node 的 RTK 质量与 covariance 边界

固定 Jazzy 源码的 `gpsFixCallback()`：
- 接受条件只要求 `status != STATUS_NO_FIX` 且经纬高不是 NaN；不会在该回调内区分接收机厂商定义的 RTK FIX、RTK FLOAT、DGNSS、SINGLE、差分龄期或基站状态。
- 位置 covariance 数值被直接复制；本轮未见根据 `position_covariance_type` 做 unknown/approximated/known 的可靠性分支。
- `prepareGpsOdometry()` 会把该 covariance 旋转后写入 GPS Odometry pose covariance。故输入 NavSatFix 若 covariance type 为 UNKNOWN、协方差全零或供应商填值不可信，不能指望 navsat_transform 自动修正其语义。
- GNSS 质量门控必须放在 navsat_transform 之前：检查接收机原始 RTK fix type、差分龄期/基站状态、卫星数/几何质量（若驱动提供）、NavSatFix status、covariance type 和 covariance 合理性；不合格时停止发布融合输入或显式标 invalid。恢复时要做跳变门控与重新收敛策略。
- navsat_transform 只建立地理坐标到机器人 world frame 的转换，并不提供接收机 RTK 解状态机；它需要 Odometry、GNSS 和有地理参考的 heading。单天线 ZED-F9P 静止时不能仅凭位置解得到可靠机身 yaw；`use_odometry_yaw` 只能使用已经可靠对齐到地理方向的绝对 yaw，不能把任意 LIO 启动 yaw 当作地理航向。

### 9.3 推荐融合拓扑更新

先保留候选双滤波结构，而不是立即把所有数据喂入单个 EKF：

1. **Local continuous filter（world=`odom`）：** 融合经过验证的腿式里程计/速度与一份明确的惯性输入，输出连续 `odom→base_link`。LIO 先作为独立观测/诊断基线；当其 covariance 和参考点合同修复后，再评估融合 LIO pose/velocity。
2. **Global correction filter（world=`map`）：** 使用 local filter 的连续状态作为基础，接收经过 RTK quality gate + navsat_transform 的 GNSS 位置约束，输出 `map→odom`。GPS 输入保持 absolute（`differential=false`），并按上游说明配置。
3. **IMU 相关性：** 若 Point-LIO 默认 `use_imu_as_input=true` 已将 MID-360 IMU 作为过程输入，不可未经相关性分析又把同一原始 IMU 的 orientation/angular velocity/acceleration 当作独立、统计独立观测全部加入 EKF。优先让 local filter 使用独立腿式状态估计提供的速度/姿态约束，或采用有明确信息分工的融合架构。
4. **TF 唯一权威：** local filter 唯一发布 `odom→base_link`；global filter 唯一发布 `map→odom`；Point-LIO raw odom 不直接广播与融合器相同的 child TF。
5. **四足三维状态：** 保持 3D roll/pitch/z 与地形高度能力；任何二维简化只能作为特定平地模式，不可影响完整导航系统的状态估计合同。

## 10. 本轮新增 Gate 与未决项

- [ ] 从 IKFoM state/manifold 源码和单元测试锁定 24 维 `kf_input` 的全部误差状态索引、SO(3) 误差坐标约定、gravity tangent 表达及 P 的 reset/boxplus 变换。
- [ ] 在 Point-LIO adapter 中导出 `kf_input.get_P()` 的只读快照；验证实时读写线程同步，不允许无锁读取 EKF 状态/P。
- [ ] 为 pose/velocity covariance 建立数学映射与有限差分/Monte Carlo 单测；包含 LIO IMU 原点到 `base_link` 的 lever-arm 与 cross-covariance。
- [ ] 修正默认 IMU-input 分支 angular twist 未赋值的问题；优先明确发布 bias-corrected angular velocity 的 frame/stamp/covariance，或将该维度从 adapter measurement mask 中禁用。
- [ ] 设计 RTK adapter 的质量状态机，验证 FIX→FLOAT→SINGLE→NO_FIX、差分龄期过期、covariance unknown/zero、跳变和恢复。
- [ ] 通过真实 rosbag 统计 LIO output latency，决定 `smooth_lagged_data` 与 `history_length`，并测试乱序、重复 stamp、clock jump 和 bag loop。
- [ ] 验证 `robot_localization` 的 3D EKF 与双滤波 TF 拓扑，特别是 LIO/GNSS reset 时 map→odom 是否连续、local odom 是否保持连续。
- [ ] 获取 Lite3 厂商状态估计接口，确认腿式 odometry 是否已经融合机载 IMU；未确认前禁止重复融合同一 IMU。

## 11. 本轮结论补充

1. Point-LIO 内部 ESEKF covariance 可通过 IKFoM `get_P()` 读取；默认 MID-360 配置走 24 维 `kf_input`。当前 ROS Odometry publisher 没有导出内部 P，故应开发 adapter/state snapshot，而非虚构 covariance。
2. Point-LIO 默认 IMU-input 分支不写 angular twist，消息字段会保持默认零值，不代表测得角速度。
3. robot_localization EKF 会将小于 `1e-9` 的测量 covariance 对角元素抬到 `1e-9`；零 covariance 输入可能导致过度信任，Point-LIO 输出 covariance 缺失必须作为融合阻断 Gate。
4. navsat_transform 对 NavSatFix 只排除 NO_FIX/NaN，不负责 RTK 解状态和差分龄期质量门控；且会直接使用 position covariance。RTK adapter 必须先验证质量状态和 covariance type。
5. 双 EKF + navsat_transform 仍是候选拓扑，不是最终选型；四足系统必须保留 3D 状态，并处理同一 IMU 的信息相关性。


## 12. Point-LIO tracking health、输入中断与 reset 生命周期审计

固定源码：CTU-MRS Point-LIO `e7ff034328b263ab92b06252d91acba1dd969b6d`，重点文件：
- `src/laser_mapping.cpp`（blob `30fe3a5b3650899d3d46e35d2ce03dcc1261f065`）
- `include/imu_processing.hpp`（blob `61007da2d8edac33d175c31a91d8a3f0947c8523`）
- `src/estimator.cpp`（blob `8555f72748e63060e057c30b2d564171d052f2b4`）

### 12.1 初始化标志不是 tracking health

- `is_initialized_` 在 `initialize()` 完成时置为 true；随后 LiDAR/IMU callbacks 和主处理 timer 只检查这个一次性启动标志。
- IMU 初始化另有 `imu_need_init_` 和 `init_iter_num`；`ImuProcess::Process()` 在初始化样本不足时返回，输入 IMU 为空时也直接返回。
- 在本次固定文件范围内，没有发现 tracking-valid/degraded/lost 状态机、持续残差/创新门限、退化检测结果向外发布、covariance health gate 或 DiagnosticArray/diagnostic_updater 输出。
- 因此 `is_initialized_ == true` 仅表示节点完成参数/对象初始化；不证明当前 LiDAR/IMU 连续、地图匹配有效或估计可用于导航。Odometry publisher 没有健康状态字段/独立有效性 topic。

### 12.2 时间回退与 bag loop 的实际行为

- `callbackLivox()` / `callbackStandardPC()` 收到比 `last_timestamp_lidar` 更早的 header stamp 时，只记录 `loop back, clear buffer` 并 return；在这两个 callback 中未看到清空 lidar/time queues、重置 last timestamp 或发出 reset 状态。
- `callbackIMU()` 先用 `header.stamp - time_lag_imu_to_lidar` 与 `last_timestamp_imu` 比较；回退时记录 `loop back, clear deque` 后 return，但该路径也没有实际清空 deque。
- `flg_reset` 在类成员中初始化为 false，主处理 timer 存在 if 分支调用 `p_imu->Reset()`；在固定 `laser_mapping.cpp` 全文件中只找到声明/读取，未找到任何置 true 的赋值。因此该 reset 分支在当前固定源码中不可由已审计代码触发。
- 即使该分支被触发，`ImuProcess::Reset()` 只重置均值加速度/角速度、`imu_need_init_` 和初始化计数；它本身不重置 Point-LIO ESEKF 状态与 covariance、ikd-Tree 地图、LIO 时间戳/队列、输出 TF 连续性或 map epoch。不能把它当成完整 estimator reset/relocalization。
- 结论：bag loop、ROS clock rewind、设备重启/时间域切换时，当前固定实现没有形成可证明的全链路 reset contract。实际运行可能停留在旧时间戳/旧队列状态；必须在项目 adapter/生命周期管理层检测并显式隔离新旧 epoch。

### 12.3 缺少输入 freshness 与失效传播

- `sync_packages()` 需要 LiDAR 与 IMU 队列均有数据，并等待 `last_timestamp_imu >= lidar_end_time`；未满足时返回 false，主处理不消费当前扫描。
- 这只是同步可用性条件，不是输入 freshness、最大队列龄期、IMU 覆盖率、点云丢帧率或估计退化检测。
- 已发现的 `time_lag_imu_to_lidar` 仍只修正 IMU 回退 gate 时间，队列保存原始消息，后续 scan coverage/integration 读取原始 header stamp；必须先统一 IMU 样本时间轴，再定义 tracking health。
- 不得将持续发布（或最后一条有效 Odometry 仍留在 topic/TF buffer）视为健康；下游必须依据最后有效测量时间和 health 状态拒绝 stale pose。

### 12.4 四足导航要求的 health 状态合同

项目侧新增 `robodog_lio_health`（名称暂定）状态输出，不修改上游 fork 的本轮审计范围。最低字段：
- `header.stamp`：健康状态评估时刻；另带 last accepted LiDAR/IMU measurement stamp，全部注明 clock domain。
- `state`：`INITIALIZING` / `TRACKING` / `DEGRADED` / `LOST` / `RESETTING`。
- `tracking_valid`、`pose_valid`、`covariance_valid`；不可只用一个 bool 表达所有质量。
- LiDAR/IMU age、scan/IMU queue depth、丢弃/乱序计数、scan-end IMU coverage、处理延迟、最近 reset/clock epoch ID。
- 可用时包含有效匹配点比例、残差/innovation、迭代收敛信息、局部几何退化指标；若算法未暴露必须标 unavailable，不造数。
- `TRACKING` 需同时满足输入新鲜、时间单调、scan/IMU 覆盖、估计数值有限、匹配质量和 covariance 合格；短时阈值待真实 bag 标定。
- `DEGRADED`：不确定度或观测质量接近门限，限制速度/禁用高风险地形，不能继续按 nominal covariance 融合。
- `LOST/RESETTING`：停止将 LIO pose 作为融合测量；保持 local odom 的连续性策略由融合层决定，必要时触发安全停车/降级；完成重初始化或 relocalization 后必须经过连续合格样本滞回才能恢复。
- Reset/relocalization 必须增加单调递增 `estimator_epoch`，并明确 map reset、局部 odom continuity、TF authority、历史测量清空、下游 costmap/map 重新注册行为。

这些是项目侧接口/安全要求，不是当前 Point-LIO 已具备能力。

## 13. robot_localization Jazzy 延迟测量、重复时间戳与时钟跳变审计

固定源码：`cra-ros-pkg/robot_localization @ fea7fc3dcad81c533121e4aa2fb1087a0565d8cc`，重点文件 `src/ros_filter.cpp`、`src/filter_base.cpp`、`src/ekf.cpp`、`src/navsat_transform.cpp`。

### 13.1 每 topic 时间顺序与跨 topic lag 是两种情况

- pose/twist/IMU/odometry callbacks 维护每 topic 的 `last_message_times_`。同一 topic 的较旧 stamp 被拒绝；相等 stamp 通过 `<=` 条件并入队，因此 duplicate stamp 并非自动拒绝。
- `integrateMeasurements()` 用 priority queue 按 measurement stamp 排序；当队首时间早于 filter 最新 measurement time 且 `smooth_lagged_data=true` 时，尝试 `revertTo(first_stamp - 1ns)`，再重放历史测量。
- 因为同一 topic 的倒序消息已在 callback 被拒绝，lag smoothing 的主要适用情形是某个 topic 的测量相对其他 topic/当前 filter 状态较晚到达，而不是修复同一 topic 任意乱序流。
- 若 history 不足，`revertTo()` 返回 false；当前 `integrateMeasurements()` 仍会继续处理队列中的测量，而不会将其标记 invalid 或自动 reset。不能把开启 smooth_lagged_data 理解为所有迟到测量都安全可恢复。
- `history_length` 应覆盖实测最大端到端迟到时间，并额外留出调度/抖动余量；超出窗口时项目 adapter 应丢弃或隔离该测量并发出诊断，而不是依赖 filter 自动处理。

### 13.2 sensor_timeout 是预测触发，不是健康门控

- 没有新 measurement 时，达到 `sensor_timeout` 后，`integrateMeasurements()` 会令 filter predict 到当前时间；它不会自动把输出标记为 stale/invalid，也不等同于传感器健康检查。
- 默认 `predict_to_current_time=false`；当 sensor timeout 触发时会启用本轮预测。预测输出在消息层仍可能是有限、可发布的状态，但其可信度取决于过程噪声和失联时长。
- 导航侧必须另行检查各传感器 last-valid stamp、预测时长、covariance 上界和融合输入质量；超过可接受 coast 时间后降级/停车。

### 13.3 reset_on_time_jump 参数与显式 reset

- Jazzy 源码声明 `reset_on_time_jump` 参数，但 pose/twist callback 中与之相关的时间跳变处理分支目前是注释代码；在本次固定 `ros_filter.cpp` 全文件中未发现该参数的有效执行路径。因此不能依赖设置该参数自动处理 ROS clock rewind。
- 显式 reset service 会清理 measurement/history、TF buffer、各 topic last-message timestamps，并把 filter 置为 uninitialized；这是明确的状态重置路径，但调用方仍需协调所有传感器 epoch、bag loop 与 map/odom TF 连续性。
- set_pose 路径记录 `last_set_pose_time_`，丢弃 stamp 不晚于 reset 的输入；reset 和 set_pose 语义不同，不应混用。

### 13.4 双滤波 TF 连续性不是自动保证

- Jazzy 源码按 REP-105 设计：world_frame=odom 的滤波器发布 `odom→base_link`；world_frame=map 的滤波器在可获得外部 `odom→base_link` 时计算并发布 `map→odom`。
- 该结构允许 global correction 在 map frame 中跳变而 local odom 连续，但前提是 local filter 不被同一次 reset/时间跳变同步清零、输入 frame 与时间一致、TF buffer/history 被正确协调。
- 项目必须用 bag/HIL 测试 local filter reset、global filter reset、GNSS FIX↔FLOAT、LIO epoch 变化、ROS time rewind、暂停/恢复和重复 stamp；量化 `odom→base_link` 连续性、`map→odom` 跳变量、TF lookup 成功率及恢复时间。

## 14. 现有 robodog_gnss_quality 核心与 ROS adapter 边界

本轮核对仓库已有 `src/robodog_gnss_quality`，不重复设计第二套 RTK quality core：

- `quality_gate.py` 已有 fail-closed RTK Fixed gate、NavSatStatus 合法值、host freshness/source monotonicity、协方差类型/有限值/对称/PSD/退化与不确定度门限、3D jump gate、连续 Fixed 恢复滞回。
- `ubx_epoch_joiner.py` 已实现 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS 按精确 iTOW 四路 join、有界缓存、接收单调时钟 watermark、stale/late/conflicting epoch tombstone、显式 receiver generation reset；纯算法层不依赖 ROS。
- 该 package 仍没有实际 ROS subscriptions、receiver GPS-week/time mapper、ROS diagnostics publisher 或 NavSatFix/Odometry publisher。不能把纯算法单测通过描述成 RTK 驱动集成完成。
- 推荐接口边界：u-blox 原始 UBX ROS topics → ROS adapter 解码/精确 iTOW join + validated GPS-week/boot generation mapper + host steady-clock freshness → 现有 `UbxFourMessageEpochJoiner` → 现有 `RtkQualityGate` → 仅对 ACCEPTED epoch 发布带正确 ENU covariance 与 measurement stamp 的 NavSatFix/quality status → `navsat_transform_node`。
- adapter 必须处理 receiver reboot/week rollover、source clock epoch、topic callback 到达偏序、bounded cache、diagnostic counters、ROS time 与 steady-clock 分离；quality gate 的 `reset()` 与 joiner 的 `begin_generation()` 必须在同一受控 reset transaction 中协调。
- NavSatFix 本身无法完整表达 RTK carrier state 与 receiver age，必须并行发布明确 quality/status topic；禁止仅靠 NavSatStatus=FIX 推断 RTK Fixed。

## 15. 本轮新增验证 Gate

- [ ] Point-LIO 输入中断、LiDAR/IMU 时间回退、设备重启、bag loop、ROS clock rewind、空/迟到 IMU、scan-end coverage 不足的单元/集成测试；确认 reset transaction 真正清理状态、队列、地图 epoch 与输出 validity。
- [ ] 为 Point-LIO 建立 health state API/topic；对输入 freshness、queue age、处理延迟、匹配退化、数值有限性、covariance 合法性和连续恢复滞回建立测试。
- [ ] 修正 `time_lag_imu_to_lidar` 让 gate、队列、同步与积分使用同一 corrected timestamp；加入正/负 lag 与乱序/重复 stamp 单测。
- [ ] robot_localization 验证 same-topic duplicate stamp、same-topic out-of-order、cross-topic delayed sample、history 足够/不足、sensor timeout coast、explicit reset、set_pose、ROS clock rewind。
- [ ] 验证 `reset_on_time_jump` 参数行为；当前固定源码只见声明和注释掉的 callback 分支，若上游无实现则项目不得依赖该参数。
- [ ] 实施并测试双 EKF reset ownership：local odom 连续、global map correction 可控；LIO/GNSS receiver epoch reset 时清理对应 measurement history，禁止跨 epoch 重放。
- [ ] 在现有 `robodog_gnss_quality` 纯算法 core 上新增 ROS 2 adapter/diagnostics 与 NavSatFix publisher；先完成消息/时钟/重启集成测试，再接 navsat_transform。
- [ ] 用真实 MID-360/IMU + ZED-F9P rosbag 测量最大延迟、乱序、掉包、时钟回退与恢复时间；确定 health/timeout/history 参数。真实数据尚未具备前，不冻结数值阈值。


## 16. robot_localization 双 EKF 的 TF 生成、重置和输出时效

固定源码仍为 `cra-ros-pkg/robot_localization @ fea7fc3dcad81c533121e4aa2fb1087a0565d8cc`，`src/ros_filter.cpp`。

### 16.1 map→odom 的计算不是第二个独立里程计 TF

- 当滤波器的 `world_frame == odom_frame` 时，节点直接广播自身 `odom→base_link` 变换。
- 当 `world_frame == map_frame` 时，节点先取得 global filter 的 `map→base_link` 状态，再在同一测量时间查询 TF buffer 中的 `odom→base_link`，计算并广播 `map→odom`。
- 这意味着典型双 EKF 拓扑必须明确分工：local EKF 是 `odom→base_link` 的唯一权威；global EKF 的 `world_frame=map` 仅负责 `map→odom`。global EKF 必须能在对应时间查询 local odom TF，否则不会生成 map→odom，并输出错误日志。
- global filter 的输入不能形成反馈环：不得把其自身的 map-frame corrected pose 又当作 local EKF 独立测量；输入配置必须逐维审计，避免把同一 local odom 的 pose/twist 和其上游原始数据当成统计独立观测重复融合。
- map→odom 是两个估计结果的相对变换；GNSS/全局修正可以使 map-frame 位置发生离散变化，而 local odom 应保持连续。这个性质依赖 local EKF 不随 global reset 一起重置，且 TF 时间查询成功。

### 16.2 显式 reset 的真实清理范围

固定 Jazzy `reset()` 实现会清理：
- initial/previous measurements、measurement queue、filter state history、measurement history；
- angular acceleration 状态、last-state twist rotation；
- last set-pose/diagnostic/control/published timestamps；
- TF buffer 与 per-topic `last_message_times_`；
- filter 内部状态，并重新装载 sensor timeout、process noise covariance 和 initial estimate covariance。

这比 Point-LIO 的 `ImuProcess::Reset()` 完整，但它仍然只重置单个 robot_localization 实例，不会协调另一个 EKF、Point-LIO map、GNSS receiver generation、Nav2 costmap 或外部 TF broadcaster。项目必须有 reset coordinator/epoch policy，不能只调用一个 filter 的 reset service 就宣称全系统复位完成。

### 16.3 reset_on_time_jump 不能作为 bag loop 保障

- 参数 `reset_on_time_jump` 在参数加载中被声明。
- pose/acceleration callback 检测同 topic 旧时间戳的分支里，调用 `reset()` 的代码仍被注释；当前固定 SHA 未实现自动 rewind reset。
- 同 topic 时间戳相等满足 `last_message_time <= msg_stamp`，因此相等 stamp 会被接受；严格更旧的 stamp 会发诊断并丢弃。
- bag loop 或 ROS time rewind 后，旧数据可能被按 topic 丢弃；即使手动 reset，也必须确认两个 EKF、TF、LIO/GNSS epoch 和 Nav2 下游状态按一致事务重新开始。

### 16.4 发布时效与 NaN 输出不是 fail-closed

- periodic update 在 filter 已初始化时持续调用 `getFilteredOdometryMessage()`；输出 header stamp 来自 filter 的 last measurement time，而 sensor timeout 可以推动 prediction 时间前进。因此输出时间戳不等于“刚刚收到新传感器观测”的证明。
- `validateFilterOutput()` 检测到 NaN 时会记录 Critical Error，但当前周期代码没有因该返回值而立即阻止后续 Odometry 发布；不可把日志告警当作下游 fail-closed。
- global EKF 生成 map→odom 时若查不到同时间的 odom→base_link，则跳过 TF 发布；这可能导致 global odometry topic 与 map→odom TF 更新不同步。必须用 diagnostics 和 TF-age gate 暴露该状态。
- 项目融合输出 adapter/health supervisor 应独立检查 measurement age、prediction/coast duration、协方差界、finite/PSD、TF lookup age 和 epoch 一致性；失效时撤销 `pose_valid`，阻止规划器继续消费 stale pose，并触发速度限制或安全停车。

## 17. GNSS 纯算法包的 ROS 化前置工程 Gate

再次核对本仓库 `src/robodog_gnss_quality/package.xml` 与 `setup.py`：
- 当前包是 `ament_python`，只依赖 `setuptools` 与测试工具，没有 `rclpy`、`sensor_msgs`、`diagnostic_msgs`、u-blox message package 或 ROS entry point；README 也明确没有 ROS node/receiver socket/UBX decoder/NavSatFix publisher。
- package metadata 当前为 `license=UNSPECIFIED`，maintainer email 是 `maintainers@example.invalid` 占位值。纯算法 CI 通过不能视为可发布 ROS 包或可集成依赖；在新增 ROS adapter 前需先确定真实项目维护者信息与可核验 SPDX license，不能擅自猜测/填入。
- 建议保持核心 transport-independent；另建独立 ROS 2 adapter package（例如 `robodog_gnss_ros`，命名待架构确认），依赖实际选定的 u-blox ROS message package，并通过 entry points 安装节点。adapter 将 ROS message 转成现有 immutable epoch records，调用 joiner 和 quality gate；只对 ACCEPTED 观测发布 NavSatFix，同时发布独立 quality/status 与 diagnostics。
- receiver GPS week/boot generation mapper 必须独立测试，且不可用 ROS header stamp 或 callback arrival order 冒充 receiver measurement time。ROS steady clock 用于 host freshness；ROS time 用于消息 stamp/TF；receiver time 用于源测量 epoch，三者需要显式映射与 epoch ID。
- 在确定真实驱动版本、消息类型、topic、receiver clock mapping 和许可之前，不新增未经验证的 ROS 依赖或伪造 NavSatFix 生产者。

## 18. 本轮新增验证 Gate

- [ ] 双 EKF 集成：local EKF 唯一发布 `odom→base_link`；global EKF 只发布 `map→odom`；验证 global EKF 查询同时间 local TF 的行为、TF timeout、map correction jump 与 local odom continuity。
- [ ] 分别 reset local/global EKF，验证 measurement/history/TF buffer 清理范围、另一个 filter 的连续性、map→odom 恢复时间；再加入 LIO epoch 与 GNSS receiver generation 变化。
- [ ] bag loop/ROS time rewind：测试 `reset_on_time_jump=true/false` 在固定 SHA 上均不能作为有效自动复位机制；项目 reset coordinator 应检测 clock epoch 并显式 reset/flush。
- [ ] 测试重复 stamp、同 topic 乱序、跨 topic 延迟、history 不足、sensor timeout coast；监控发布 Odometry stamp、实际最新观测 stamp、prediction duration 和 output validity。
- [ ] 注入 NaN/Inf、负/非 PSD covariance、TF 缺失/过期；验证 health supervisor 确实阻止无效 Odometry/TF 进入 Nav2，而不只是输出日志。
- [ ] GNSS ROS adapter 实现前，确认包 SPDX license、真实 maintainer metadata、u-blox ROS driver/message package 固定 SHA、消息定义与依赖许可。
- [ ] ROS adapter 完成后，在真实 ZED-F9P bag 验证四路 UBX 同历元 join、GPS week rollover、receiver reboot、时间映射、steady-clock stale、quality recovery 和 NavSatFix covariance frame；未验证前不得接入 global EKF。


## 19. 定位有效性到 Nav2 Costmap / Planner / Controller 的故障传播审计

固定 Navigation2 Jazzy 源码：`c7046550547d382a5186d106b0895eda02485d7c`。重点文件：
- `nav2_costmap_2d/src/costmap_2d_ros.cpp`（blob `fe8cd94284e7b3b266c516c8fe40c1fb50053195`）
- `nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d_ros.hpp`
- `nav2_planner/src/planner_server.cpp`（blob `15152434a3430e12dad6ac841d3b732501bb6df9`）
- `nav2_controller/src/controller_server.cpp`（blob `a39db25a1aa5c17be431694971bed72c07cd8f4e`）
- `nav2_util/src/robot_utils.cpp`（blob `a9f4d137fa21229404c7c9726c0d4f70bf4a11b1`）

### 19.1 Costmap 的 robot pose 失败路径

- `Costmap2DROS::getRobotPose()` 直接调用 `nav2_util::getCurrentPose()`，从 TF buffer 查询 `global_frame → robot_base_frame`；查询/变换失败时返回 false。
- `Costmap2DROS::updateMap()` 每周期先取 robot pose；只有成功时才调用 `layered_costmap_->updateMap(x,y,yaw)`。pose 获取失败时本周期直接返回，不更新 layered costmap，也不发布本周期 footprint。
- 这个失败路径没有在 `updateMap()` 中显式把 LayeredCostmap 的 current 标志设置为 false。LayeredCostmap current 是启用 layers/filters 的聚合状态；因此不能假设 TF 丢失必然令 `isCurrent()` 变 false。若各 layer/filter 仍报告 current，Planner/Controller 的 Costmap current wait gate 可能仍然放行。
- 结果是两个独立健康维度必须分开监控：**Costmap observation/plugin current** 与 **robot pose/TF validity**。不能以 `isCurrent()==true` 代替定位 TF 有效。

### 19.2 Planner 的 pose 失效传播

- Planner 的 `waitForCostmap()` 只等待 global Costmap `isCurrent()`。
- 随后 `getStartPose()` 在 goal 未提供显式 start pose 时调用 `getRobotPose()`；失败则抛出 `PlannerTFError("Unable to get start pose")`。
- `computePlan()` / `computePlanThroughPoses()` 将该错误映射为 Planner Action 的 TF error 并终止当前规划 goal；Planner 不发布运动命令。
- 因此 Planner 有 TF 失败终止路径，但没有读取项目侧 `pose_valid`、LIO tracking health、EKF measurement age、covariance health 或 estimator epoch 的原生接口。即使 TF 可查询，过期但仍留在 buffer 中的变换也不能由这一布尔查询证明为新鲜。

### 19.3 Controller 的 pose 失效传播

- Controller 每轮先等待 local Costmap `isCurrent()`；然后 `computeAndPublishVelocity()` 调用 `getRobotPose()`。
- pose 查询失败时抛出 `ControllerTFError`；catch 路径调用 `publishZeroVelocity()`，FollowPath Action 以 `TF_ERROR` 终止。
- 这是一条软件层零速度请求路径，不证明命令已穿过速度平滑器/Collision Monitor/平台 adapter，也不证明 Lite3 接受命令或已制动。
- 若 TF 可查询但来自失效/过期 estimator，Nav2 原生代码没有项目级 health/epoch/freshness Gate；Controller 可能继续按该 pose 计算速度。因此必须在 Nav2 前的定位输出/TF 发布边界，或独立 command safety supervisor 设置明确的 pose-validity Gate。

### 19.4 故障传播矩阵（项目设计合同，尚未实现）

| 故障/状态 | 定位/融合输出 | Costmap 行为 | Planner | Controller / 安全层 |
|---|---|---|---|---|
| LIO `DEGRADED`，pose 仍经融合层验证有效 | 保留 pose_valid；标记降级与不确定度上升 | 只允许在传感器与 TF freshness 均合格时更新 | 允许受限重规划；禁止高风险地形路线 | 限速、扩大安全裕量；由 terrain/safety policy 决定是否禁行 |
| LIO `LOST` 或 covariance/数值无效 | 撤销 LIO measurement validity；融合层按策略保持短时 local continuity 或标 invalid | 不得仅因旧 TF 仍存在就继续作为健康地图输入 | 阻止新规划/取消依赖当前 pose 的规划 goal | 立即请求安全停止；平台 watchdog 监督执行反馈 |
| local EKF stale/coast 超限 | pose_valid=false；停止发布可被误认作新鲜的有效定位输出，或另发显式 invalid 状态 | pose 获取失败时 updateMap 本周期跳过；current 可能仍为 true | TF error/显式 supervisor gate 终止规划 | TF_ERROR 路径请求零速度；独立 watchdog 验证停车 |
| global EKF / GNSS 失效，local odom 仍健康 | 保留 odom 连续性；撤销 global/map pose validity，冻结或标记 map correction | global Costmap 若依赖 map TF，可能停止更新；local Costmap 需独立评估 | 禁止依赖失效 map frame 的全局规划；可否 local safe-stop 由任务策略决定 | 仅允许经验证的局部受限行为；不得把旧 map→odom 当作当前全局定位 |
| TF 缺失/时间外推失败 | pose lookup invalid | Costmap updateMap 跳过本周期，但 plugin current 不必然变 false | getStartPose 失败时 TF_ERROR | Controller 抛 ControllerTFError，发布零速度并终止 FollowPath |
| 传感器停流但 layer current 未及时变 false | estimator/source age invalid | Costmap 可能仍报告 current（取决于 ObservationBuffer 配置） | Planner current gate 可能放行，必须由外部 health gate 拦截 | Controller current gate 可能放行，必须由 pose/sensor supervisor 拦截 |
| receiver generation / estimator epoch 变化 | 隔离旧 epoch measurement/history；reset coordinator 完成后再恢复 valid | 清理/重建受影响地图层或标记其数据 epoch | 暂停依赖旧地图/定位的规划并在新 epoch 后重规划 | 进入 RESETTING/stop；恢复需连续合格样本滞回及平台状态确认 |

### 19.5 建议的统一 validity contract 与动作状态机

项目侧建议建立独立 `robodog_navigation_health` 聚合状态（名称暂定），不把多个子系统的 bool 简单 OR/AND 后丢失原因。至少维护：
- `estimator_epoch`、`gnss_receiver_generation`、`map_epoch`；
- LIO tracking state、pose/covariance validity、last accepted LiDAR/IMU measurement stamp；
- local/global EKF last measurement stamp、prediction/coast duration、reset generation；
- `odom→base_link` 与 `map→odom` 的 lookup success、transform stamp/age、authority；
- global/local Costmap plugin current、最后成功 robot-pose update 时刻、update duration；
- Planner/Controller Action 状态、最后有效命令时间、平台命令接收/执行反馈与实测速度。

状态/动作建议：
1. `INITIALIZING/RESETTING`：禁止新导航 goal 和运动命令；完成 epoch 对齐、TF/地图注册后进入恢复观察。
2. `NOMINAL`：所有定位、TF、地图和平台反馈 Gate 合格，允许正常规划/控制。
3. `DEGRADED`：保留的定位仍通过质量门限，但不确定度/数据龄期接近阈值；限速、缩短重规划周期、禁用高风险地形。
4. `POSE_INVALID` 或 `MAP_INVALID`：分别阻止依赖该 frame 的 Planner/Controller；local odom 健康不代表 global map 健康，反之亦然。
5. `STOP_REQUESTED`：发布零速度只是请求；必须等待平台反馈速度降至安全阈值或驱动器确认停止。超时则由独立 watchdog/物理急停路径升级处理。
6. `RECOVERY`：只有同一 epoch 内连续合格观测、TF 查询成功、covariance/age 合格、Costmap 成功更新且平台反馈正常，才解除降级；具体样本数/时间阈值必须由真实 bag/HIL 标定，本阶段不预设数值。

### 19.6 本轮结论与证据边界

1. Nav2 Jazzy 的 Costmap pose lookup 失败会跳过本周期地图更新，但没有同步强制 `LayeredCostmap::isCurrent=false`；所以 Costmap current 与定位 TF validity 必须作为两个独立 Gate。
2. Planner 在默认从 TF 获取起点时，pose 失败映射为 TF_ERROR 并终止规划；Controller pose 失败映射为 TF_ERROR、发布零速度并终止 FollowPath。
3. Planner/Controller 不原生理解 Point-LIO health、robot_localization measurement age、covariance validity、GNSS receiver generation 或 estimator epoch；这些必须由项目侧 supervisor/adapter 提供。
4. Controller 的零速度是 ROS topic 输出，不是底层停止确认；必须贯通平台反馈与独立 watchdog。
5. 本节为固定上游源码 E2 + 项目现有 GNSS package metadata 静态核验；没有新增 ROS 构建、仿真、真实 bag、HIL 或机器狗实测。


## 20. 现有 GNSS quality core 与测试覆盖矩阵

固定本项目审计对象（`nav` 分支当前版本）：
- `src/robodog_gnss_quality/robodog_gnss_quality/quality_gate.py`（blob `1083b8d1cc261d4833928d3cfaf38bf522acf3c4`）
- `src/robodog_gnss_quality/robodog_gnss_quality/ubx_epoch_joiner.py`（blob `9a9db8fdce9e035c3997f1a101165bef82b99817`）
- `src/robodog_gnss_quality/robodog_gnss_quality/ubx_nav_pvt.py`
- `test/test_quality_gate.py`（30 个测试）
- `test/test_ubx_epoch_joiner.py`（17 个测试）

### 20.1 已有纯算法能力（源码事实）

**RtkQualityGate**
- 输入是 immutable `GnssObservation`，包含 receiver source timestamp/clock ID、host monotonic receive time、坐标、NavSatStatus、receiver `gnss_fix_ok`、carrier solution、ENU 3×3 covariance 和 covariance type。
- fail-closed 接受策略要求 NavSatStatus 属于合法值、receiver fix flag=true、carrier solution 明确 FIXED；FLOAT/NO_SOLUTION/UNKNOWN 不合格。
- freshness 使用 host monotonic receive timestamp；source timestamp 仅在同一 source clock ID 内比较，拒绝非递增 epoch。
- 对坐标 finite/range、covariance type、finite/symmetry/PSD/degenerate、水平/垂直标准差做门控；默认拒绝 APPROXIMATED 与 UNKNOWN covariance。
- 使用 haversine + altitude delta 估算 3D 位移速度，拒绝超过配置的 plausible speed；跨越配置 continuity interval 的数据 gap 会重新开始 Fixed 连续样本计数。
- 连续 Fixed 样本满足配置数量后才返回 ACCEPTED；无效输入通常清空 recovery/reference 状态。仅对 out-of-order 样本使用 `reset=False`，不让旧数据破坏当前连续计数。

**UbxFourMessageEpochJoiner + UBX record builder**
- 按精确 iTOW 聚合 NAV-PVT、NAV-HPPOSLLH、NAV-COV、NAV-STATUS 四路消息；四路 iTOW 必须完全一致，且同一 iTOW 的 source-time mapping 必须一致。
- 校验 NAV-PVT 与 NAV-STATUS 的 fix flag/carrier solution 一致，FIXED 时要求 STATUS carrier_solution_valid；要求 HPPOSLLH 所有位置分量有效、NAV-COV pos_cor_valid。
- 使用有界 pending LRU、host monotonic receive skew、显式 stale expiry 和 retired tombstone；拒绝 conflicting duplicate、late message、跨 generation 混入、重复完成 epoch 和非递增输出 epoch。
- receiver week/boot generation 变化必须由调用方先验证，再显式调用 `begin_generation()`；`reset()` 后禁止隐式重新开始。
- NAV-COV NED covariance 转 ENU 时对 Down 相关 cross terms 正确变号；不能只交换对角元素。HPPOSLLH 高精度坐标按 base + HP 分量换算。
- 核心明确不包含串口/UBX 解码、ROS subscriptions、GPS-week mapper、NavSatFix publisher 或 diagnostics。

### 20.2 已有测试覆盖

| 范围 | 已覆盖场景 | 证据 |
|---|---|---|
| Gate 配置 | 非正 age、非有限阈值、恢复样本数非法 | 4 tests |
| GNSS 基础质量 | FLOAT/NO_SOLUTION、NavSatStatus invalid、gnssFixOK=false、坐标越界/NaN | 7+ tests |
| Freshness/时间 | stale、future receive timestamp、duplicate/out-of-order、长时间数据 gap、receiver clock epoch change | tests |
| Covariance | UNKNOWN/APPROXIMATED、零/NaN、非对称、非 PSD、水平/垂直阈值、DIAGONAL_KNOWN 非零 off-diagonal | tests |
| 动态跳变/恢复 | implausible position jump、合理小运动、反经线跨越、连续 Fixed hysteresis | tests |
| UBX pairing | 四路任意到达顺序、跨 iTOW 不串配、同 iTOW mapping 冲突、PVT/STATUS 质量一致性、重复/冲突消息 | tests |
| Cache/时钟 | 容量上限、stale expiry、late callback、retired epoch 防复活、GPS week iTOW 边界、显式 generation/reset | tests |

以上是静态检查到的测试用例数量/名称与实现对应关系；未在本轮重新运行测试。此前 CI 的历史通过记录不替代当前 HEAD 的重新运行。

### 20.3 仍缺少的集成契约与测试

1. **GNSS generation transaction 尚未封装。** Joiner 有 `begin_generation/reset`，Gate 有独立 `reset`，但没有单一 coordinator 原子协调两者、GPS-week mapper、NavSatFix 输出 epoch 和 global EKF history。必须定义事务顺序，避免旧 epoch 观测在 reset 后进入 gate 或 EKF。
2. **GPS-week/time mapper 不存在。** `source_timestamp_ns` 由 adapter 调用方提供；核心不会验证它与 GPS week/iTOW 的数学对应。需实现并测试 GPS week rollover、receiver reboot、iTOW wrap、GPS week 来源无效、设备上电/掉电与 ROS bag loop。
3. **GNSS quality 状态未发布。** `QualityDecision` 有 reason/qualified/连续样本数/不确定度/速度，但没有 ROS diagnostics、状态 topic、last accepted epoch、receiver generation 和输出有效期；`UbxFourMessageEpochJoiner` 的内部 counters 也尚无统一 diagnostics contract。
4. **NavSatFix 与质量状态的消费原子性未实现。** Adapter 必须保证仅同一 ACCEPTED epoch 的 NavSatFix 进入 navsat_transform；拒绝/过期/重置时不能保留旧 fix 作为新数据重复发布。质量状态与 NavSatFix 必须能按 epoch 关联。
5. **输出时钟/坐标契约待 adapter 实现。** receiver measurement stamp、ROS time、host steady receive time 三者必须分别保留；NED→ENU covariance、WGS84 geodetic→map ENU 原点/高度基准、GNSS antenna→base_link lever arm 仍需显式映射与测试。
6. **Gate 与 joiner 之间缺少组合级 property/fuzz 测试。** 需覆盖随机 topic arrival permutation、重复/冲突/迟到混合、bounded memory、generation 切换边界、相同 iTOW 不同 source mapping、gate reset 中断和异常恢复。
7. **当前 package metadata 仍阻止正式 ROS 化发布。** `package.xml`/`setup.py` 的 `UNSPECIFIED` license 和 `maintainers@example.invalid` 需由项目维护者确认；本轮不修改这些声明，也不擅自新增 ROS dependencies。
8. **阈值不应在无设备数据时冻结。** max fix age、pair receive skew、pending age、covariance std、plausible speed、continuity interval、recovery sample count 均须由 ZED-F9P 真实输出、host load 和导航速度/风险测试标定。

### 20.4 对 N1/N3 的直接工程结论

- 现有 GNSS 算法核心覆盖了重要的 fail-closed 质量策略与四路 UBX 同历元配对，不应另起炉灶重写。
- 下一阶段应先实现/审计 transport-independent `GnssEpochCoordinator`（名称暂定）接口与 GPS week/receiver generation mapper 的状态转移规格，再决定是否拆分 ROS adapter package。
- GNSS ACCEPTED 只代表该历元满足 GNSS 输入质量 Gate；不自动代表 global EKF/map pose 有效。Global filter 还需检查 local TF 对齐、measurement age、covariance、epoch 与 map→odom 发布连续性。
- 本节为本项目固定分支源码和测试静态审计；没有新增测试运行、ROS 构建、真实 ZED-F9P bag、HIL 或硬件证据。


## 21. GNSS / LIO / EKF / Nav2 跨组件 Epoch 与 Reset Coordinator 契约

本节将 §19 的 validity Gate 与 §20 的 GNSS 纯算法 core 组合成可实现、可单测的系统事务设计。**这是项目侧设计合同，不代表当前仓库已经实现 coordinator，也不代表 robot_localization、Point-LIO 或 Nav2 原生支持跨节点原子 reset。**

### 21.1 先区分四种 epoch，禁止混成一个计数器

| 标识 | 所属对象 | 递增条件 | 不应触发的动作 |
|---|---|---|---|
| `estimator_epoch` | Point-LIO / LIO estimator 生命周期 | LIO map/state 被清空、重初始化、不可连续恢复的时钟回退/设备重启 | 单纯 GNSS FIX↔FLOAT 不应重置 LIO |
| `local_odom_epoch` | local state estimator 的连续里程计坐标系 | local EKF 被 reset/reinitialize，或 odom 原点/状态发生不连续变化 | global GNSS correction 丢失不应重置健康的 local odom |
| `gnss_receiver_generation` | 接收机时钟/启动生命周期 | 接收机重启、GPS week/iTOW 映射失效后重新建立、source clock identity 改变 | 单纯 RTK FIX↔FLOAT 不应递增 generation |
| `map_epoch` | global map / map→odom 解释域 | map 原点、地图内容坐标解释、global localization datum 或地图版本发生不兼容变化 | 单纯 GNSS 暂时降级但地图基准未改变，不应自动清空地图 |

另需为 reset transaction 分配独立单调 `reset_transaction_id`，以及各组件 `component_reset_generation`。所有 ID 在系统进程重启后不得被误认为仍处于旧生命周期；启动时必须创建新的 boot/session identity。跨进程持久化或 ROS 消息中使用复合 identity（boot UUID + uint64 counter），不能只依赖易重置的本地计数器。

### 21.2 组件 reset ownership

| 组件 | 唯一 reset owner | 必须清理/隔离 | 不得越权 |
|---|---|---|---|
| UBX epoch joiner | GNSS epoch coordinator | pending epochs、retired/tombstone 状态按 generation 规则重建；拒绝旧 generation callback | 不自行推断 GPS week 或接收机重启 |
| RTK quality gate | GNSS epoch coordinator | source monotonic reference、recovery/reference history、连续 Fixed 计数 | 不自行变更 receiver generation |
| GPS week / receiver time mapper | GNSS epoch coordinator 管理的 mapper | week anchor、iTOW unwrap、boot identity、source→ROS stamp mapping | 不以 host receive timestamp 伪造 receiver measurement time |
| Point-LIO | LIO lifecycle supervisor（由系统 coordinator 编排） | LIO state/P、局部地图/ikd-tree、输入队列、时间基准、tracking health、TF epoch | GNSS generation 单独变化不得直接 reset LIO |
| local EKF | local state-estimation supervisor | measurement queue/history、filter state、per-topic timestamps、TF buffer及 reset generation | global EKF/GNSS-only reset 不得重置仍健康的 local odom |
| global EKF | global localization coordinator | GNSS measurement queue/history、global filter state、map→odom 关联与有效性 | 不得擅自发布与 local EKF 冲突的 odom→base_link |
| elevation/map layers | map lifecycle coordinator | 与旧 estimator/map epoch 关联的动态/高程观测；保留或清空静态地图必须有明确策略 | 不得把旧 epoch 数据重新标为当前观测 |
| Nav2 Planner/Controller/Costmap | navigation health supervisor | 阻止新 goal、取消受影响 goal、阻止无效 pose 进入控制；按需清理 costmap | Nav2 action 的 TF_ERROR/零速度不能代替 reset coordinator |
| 四足平台 adapter / watchdog | 平台安全控制器 | 停止请求、命令超时、执行反馈与急停状态 | 软件 reset 成功不得推断平台已停止 |

### 21.3 触发事件与最小影响范围

1. **GNSS RTK FIX→FLOAT/SINGLE/NO_FIX**：只撤销 GNSS measurement validity，global localization 进入 DEGRADED/MAP_INVALID；若 local odom 健康则保留。默认不递增 receiver generation，不 reset joiner/gate 的时钟身份，不 reset LIO/local EKF。停止使用旧 GNSS fix 作为新观测。
2. **GNSS receiver reboot / GPS week anchor 丢失 / source clock identity 改变**：递增 `gnss_receiver_generation`，执行 GNSS 子事务：隔离旧 generation 的四路缓存、reset quality gate、重建 time mapper；global EKF 暂停接收 GNSS，检查其旧 measurement history，必要时 reset global filter 或丢弃 GNSS 测量。local EKF/LIO 不随之自动 reset。
3. **Point-LIO 输入时钟回退、设备重启或 LIO map/state 被重置**：递增 `estimator_epoch`。若 LIO 是 local EKF 的唯一/关键状态来源且无法证明状态连续，则 local pose validity 撤销并进入 local reset transaction；否则仅隔离 LIO 输入，保留经独立腿式里程计验证的 local odom。是否递增 `local_odom_epoch` 由 local filter 是否被 reset 决定。
4. **local EKF 自身 reset 或 odom 原点改变**：递增 `local_odom_epoch`，旧 `odom→base_link` 历史、依赖旧 odom 的局部地图/规划全部失效。Nav2 必须暂停，不能将新 odom 原点伪装成连续旧 frame。
5. **global datum/map origin 改变**：递增 `map_epoch`，暂停 global planner，清理/重投影与旧 map frame 绑定的地图和目标。若只是 GNSS 短时中断、global EKF 保持原 datum，则不递增 map epoch。
6. **ROS time rewind / bag loop**：检测 clock epoch 变化。不能只依赖 robot_localization 的 `reset_on_time_jump`；必须停止输出消费、隔离旧 ROS-time measurements/TF、清理相关队列并显式发起 transaction。host steady clock freshness 与 ROS clock epoch 必须分开。
7. **仅 Nav2 Costmap observation stale**：不触发 LIO/RTK/EKF reset；只令受影响 costmap invalid，禁止相关规划/控制，按传感器恢复和 map-layer 策略重建。

### 21.4 Reset transaction 状态机与顺序

Coordinator 必须由单一串行事件处理器（或等价的严格互斥状态机）处理触发事件，不能让多个 ROS callbacks 并发地分别 reset 各组件。推荐阶段：

1. `DETECTED`：记录触发源、旧/新 epoch、时间域、原因码、关联组件和 transaction ID；相同事件去重，重复命令幂等。
2. `QUIESCING`：冻结新导航 goal；取消受影响 Planner/Controller actions；向 command arbiter 发 stop request；平台 adapter/watchdog 确认执行状态。未确认停车时不得声称安全停稳，按平台安全策略升级。
3. `INVALIDATING`：先发布新 transaction 的 invalidation barrier。所有 producer/consumer 检查 boot identity、epoch 和 transaction generation；拒绝迟到的旧 epoch 数据。此时 pose/map validity 均 fail-closed。
4. `DRAINING`：停止相关输入回调进入下游，等待 in-flight callback/worker 退出；清空 ROS message filters、同步器、应用队列、TF buffer 中受影响的历史。不得仅清理算法内部 deque 而保留外部缓存。
5. `RESETTING_COMPONENTS`：按影响范围调用组件唯一 owner 的 reset。GNSS-only transaction 不 reset LIO/local EKF；local odom discontinuity transaction 则按依赖图 reset local EKF、global alignment、局部地图和 Nav2。
6. `REBASING`：建立新 receiver time mapping、local odom identity、map datum/epoch 和 TF authority。确认唯一 TF 发布者，等待同 epoch 的 transform 链完整。
7. `WARMING_UP`：只接收新 epoch 数据，等待 LIO tracking、GNSS gate（若该事务需要 GNSS）、EKF measurement、TF freshness、Costmap 成功更新及平台 feedback 各自通过质量门。每个子系统必须报告自己的 last accepted measurement，不可用 timer tick 代替。
8. `COMMITTING`：Coordinator 原子发布 transaction committed 状态和新 validity snapshot；解除 stop/goal inhibit 前，重新验证平台处于可运动状态、急停未触发、命令通路健康。
9. `COMMITTED`：记录 transaction outcome、各组件 generation、耗时与诊断。恢复导航由上层重新发 goal，不自动恢复旧的 FollowPath 控制流。
10. 任一阶段失败进入 `ABORTED_FAIL_CLOSED`：保持运动 inhibit、相关 pose/map invalid，保留可诊断现场；不得回滚成“旧状态仍有效”。人工/上层策略决定重试或人工介入。

事务的“原子性”是**逻辑可见性原子性**：在 COMMIT 前所有受影响 consumer 都看到 invalid/inhibited；COMMIT 后才看到新 epoch 的一致快照。ROS 2 多节点不提供跨节点数据库式原子提交，因此需要 barrier、ack、generation fencing 和超时失败关闭来实现。

### 21.5 跨组件数据接受规则

每条状态/观测/TF/命令至少携带或可关联：
- boot/session identity；
- 相关 `estimator_epoch`、`local_odom_epoch`、`gnss_receiver_generation`、`map_epoch`（不相关字段显式标记 N/A，不能填 0 冒充有效）；
- source measurement timestamp + source clock identity；
- ROS header timestamp + ROS clock epoch；
- host steady receive timestamp；
- producer validity、quality reason、covariance validity；
- 对于 reset 相关消息，`reset_transaction_id` 和 component generation。

消费者规则：
- epoch 不匹配、boot identity 不匹配、已撤销 generation、时间域无法映射、stamp 回退或 freshness 不合格：拒绝，不做“尽力接受”。
- 同一 epoch 的重复 stamp 是否可接受由具体传感器合同定义；GNSS 完成 epoch 不得重复向 navsat_transform 发布。
- 旧消息不能因 ROS queue 延迟、TF buffer 保留或 bag loop 在 reset 后重新进入系统。
- `NavSatFix` 只作为 GNSS observation payload；只有与同一 ACCEPTED decision/epoch 绑定的记录才可送 global EKF。质量状态应并行发布，并由 adapter 保证一致性。
- `map→odom` 只有 global filter 在同一 measurement time 查询到同一 `local_odom_epoch` 的 `odom→base_link` 且 covariance/age 合格时才有效；否则 map pose invalid。
- Command arbiter 只接受当前有效 epoch 的速度命令；旧 epoch 命令必须丢弃，stop/inhibit 命令优先级高于普通导航命令。

### 21.6 Reset ownership 与动作矩阵

| 事件 | GNSS joiner/gate/mapper | LIO | local EKF / odom | global EKF / map | Nav2 / 平台 |
|---|---|---|---|---|---|
| RTK Fixed→Float | gate invalid；generation 不变 | 保持 | 保持 | 停止 GNSS 更新；global 降级 | global goal inhibit；若 local-only safe behavior 未验证则停车 |
| Receiver reboot/week remap | 新 generation、清缓存、reset gate/mapper | 保持 | 保持 | 隔离旧 GNSS queue/history；按需 reset global | 停止 global navigation，确认 map datum 后恢复 |
| LIO lost but independent leg odom healthy | 保持 | LIO reset/重新初始化 | local EKF可按融合策略继续，但标记 LIO source invalid | 取决于 local validity | 降速或停止，禁止旧 LIO TF 冒充有效 |
| LIO reset is local pose authority | 保持 | 新 estimator epoch | local EKF reset；新 local_odom_epoch | global alignment 重建；旧 map→odom invalid | 停止、清局部地图、重新规划 |
| local EKF reset | 不必 reset | 不必 reset（除非依赖闭环） | 新 local_odom_epoch | 重新建立 map→odom；旧历史隔离 | stop + cancel goals + costmap/plan epoch fencing |
| global datum/map change | 视来源决定 | 保持 | 保持 | 新 map_epoch，global filter/datum 重建 | 仅 global navigation inhibit，local odom 保持 |
| ROS clock rewind / bag loop | 重新校验 receiver mapping | 检查 LIO 时间/输入 epoch | 显式 reset/flush，不能依赖参数自动处理 | 显式 reset/flush | 所有时间相关 consumer inhibit，平台保持安全态 |

### 21.7 最小可实现接口（先 transport-independent）

建议先实现纯算法层状态机，不先引入 ROS 依赖。接口草案（字段语义合同，不是已提交代码/API）：

- `ResetTrigger`: `trigger_id`、`reason_code`、`source_component`、旧/新 epoch proposals、source/ROS/steady timestamps、受影响 scope。
- `ResetPlan`: transaction ID、需要 inhibit 的 consumer、组件 reset 顺序、需要等待的 acknowledgements、保留/失效的数据域。
- `ComponentResetAck`: component name、boot identity、old/new generation、reset success、queue/history drained、TF authority ready、diagnostic reason。
- `ValiditySnapshot`: 独立的 `pose_valid_local`、`pose_valid_global`、`gnss_measurement_valid`、`lio_tracking_valid`、`local_costmap_valid`、`global_costmap_valid`、`platform_stopped_confirmed` 与 epoch tuple。
- `ResetOutcome`: COMMITTED/ABORTED_FAIL_CLOSED、各组件 ack、失败阶段、原因码和可恢复性。

禁止单一 `system_ok=true` 掩盖 GNSS 无效但 local odom 有效、或 local pose 有效但 global map 无效等差异。

### 21.8 必须先于 ROS adapter 的纯算法测试

1. GNSS-only generation change 不改变 LIO/local_odom epoch；RTK FIX↔FLOAT 不增加 receiver generation。
2. Receiver reboot 递增 generation；旧四路 UBX pending/late/duplicate 数据不能穿过 reset barrier；旧 gate recovery count 不得继承。
3. GPS week rollover（连续时间）不触发 reboot generation；同 iTOW 不同 GPS week 不得误配；week unknown/ambiguous 必须 fail closed。
4. local EKF reset 必须递增 local_odom_epoch；global datum reset 必须递增 map_epoch；仅 GNSS dropout 不清空健康 local odom。
5. reset 事件重复、乱序、并发到达时 transaction ID 幂等且串行；旧 ack/迟到 ack 不得提交新 transaction。
6. drain/ack timeout、组件 reset 失败、TF authority 冲突、平台 stop 未确认时进入 ABORTED_FAIL_CLOSED，保持 inhibit。
7. COMMIT 前旧 epoch measurement/TF/cmd 全拒绝；COMMIT 后只接受新 epoch；跨 boot identity 的计数器碰撞不能误接受。
8. local/global pose validity 独立；global GNSS 失效不误伤 local pose，local odom reset 必须使 global map→odom 失效。
9. ROS clock rewind 与 host steady clock 前进同时出现时，host freshness 不能掩盖 ROS time epoch 变化；bag loop 不得复用旧 TF/measurement。
10. 状态快照序列化/反序列化、崩溃恢复、进程重启、重复命令与故障注入的 property-based tests。

### 21.9 尚未冻结的设计决策

- LIO lost 后 local EKF 是否可依赖 Lite3 腿式 odometry 独立维持：等待厂商正式 odometry 接口、IMU 融合关系与协方差证据。
- GNSS receiver generation 的可靠 reboot 判据：等待 ZED-F9P 原始 UBX/PVT/STATUS、GPS week 来源、boot/reset indicator 和实机日志。
- global EKF 在 GNSS generation 改变后是保留 map datum 还是 reset global filter：按 map datum continuity 与历史 measurement queue 审查，不能一概而论。
- map epoch 变化时静态地图、elevation map、动态障碍层分别保留/重投影/清空的策略：需有地图坐标基准和观测来源合同。
- stop confirmation 的速度阈值、持续时间、ack timeout、warm-up 样本数：必须由 Lite3 驱动语义、实机日志、bag/HIL 与安全风险分析标定；当前不填猜测数值。

### 21.10 证据与实施边界

- §19：Navigation2 Jazzy 固定 SHA 的 Costmap/Planner/Controller 静态源码证据。
- §20：本项目 GNSS core 与测试静态代码证据；本轮未重新运行测试。
- 本节：基于上述证据形成的 E3 项目设计合同/待验证方案；不是实现完成或硬件安全认证。
- 未新增 ROS package、未修改现有算法核心、未改 license/maintainer metadata，未运行 ROS build、bag、仿真、HIL 或机器狗实测。

## 22. Reset API 逐事件 Transition Table 与现有 GNSS Core 适配约束

本节将 §21 的系统设计转成逐事件执行表，并核对当前 GNSS core 的真实 reset API。状态机和 coordinator 仍是设计规格，不代表已实现。

### 22.1 当前可调用的 GNSS reset API（源码核验）

| API | 实际行为 | 不会做的事 / 注意事项 |
|---|---|---|
| RtkQualityGate.reset() | 将 consecutive Fixed samples 置 0，清除 last accepted jump reference | 不改变配置；不管理 receiver generation；不通知 global EKF/NavSatFix publisher |
| UbxFourMessageEpochJoiner.reset() | source_clock_id 置空；requires-explicit-generation 置 true；清空 pending、retired、latest receive time、last emitted source timestamp | 不清空 evicted/rejected 累计诊断计数；reset 后 add_* 必须先 begin_generation() |
| UbxFourMessageEpochJoiner.begin_generation(id) | 设置新的 source_clock_id；解除 explicit-generation barrier；清空 pending、retired 和 epoch 时间状态 | 不验证 GPS week/reboot 判据；不重置累计诊断计数；调用者必须先完成 generation 判定 |
| UbxFourMessageEpochJoiner.expire_stale(now) | 按 host monotonic time 淘汰超龄 pending epoch 并写入 retired tombstone | 不是 receiver reset；不清理 gate 或 global EKF history |

关键差异：Gate.reset() 清除的是质量恢复/跳变参考状态；Joiner.reset() 还会进入“禁止接收直到显式 begin_generation”的封锁状态。Joiner 的诊断计数是跨 generation 累计量，若需要按 generation 统计，应在 coordinator 外层记录 counter baseline 或增加分代 metrics，而不能假设 reset 会清零。

另外，Gate.evaluate() 检测到 source_clock_id 与其上一条 reference 不同，会自行 reset 后继续处理；但 Joiner 对 source_clock_id 变化会抛出 EpochPairingError，要求显式 begin_generation()。因此 adapter 必须以 Joiner/coordinator 为 generation 权威，不能绕过 joiner 直接把新 clock ID 的 observation 喂给 Gate，否则两个组件可能对 generation 切换作出不同处理。

### 22.2 逐事件 Transition Table

| 事件 | 前置条件 / 判定 | Epoch 变化 | Reset / 隔离顺序 | 必需 ACK 与恢复条件 | 失败处理 |
|---|---|---|---|---|---|
| RTK FIXED→FLOAT/SINGLE/NO_FIX | 同一 receiver clock generation；接收机状态消息与测量历元已配对 | 不递增 receiver generation、estimator、local odom 或 map epoch | 先撤销 GNSS measurement valid；停止发布后续 NavSatFix；保留 joiner 配对状态与 mapper | GNSS gate 输出拒绝原因；global localization 明确报告 GNSS degraded；local odom 独立有效性仍通过 | Global pose 不可信时禁止 global goal；不得把最后一条 FIX 重复当新观测；local-only 运动须另有验证策略，否则停车 |
| Receiver reboot / source clock identity 改变 | 有接收机 reboot 证据，或原 source clock mapping 已不可继续；仅 iTOW 变小不足以单独证明 reboot，必须结合 GPS week/boot evidence | receiver_generation +1；其他 epoch 默认不变 | 1) GNSS invalidation barrier；2) 停止 NavSatFix；3) joiner.reset()；4) gate.reset()；5) mapper 结束旧 generation 并建立新映射；6) mapper 验证新 generation 后 joiner.begin_generation(new_id)；7) 新 epoch 数据重新配对并经 gate | mapper ACK 新 clock identity/week anchor；joiner ACK active generation；gate recovery 从 0 重新开始；global EKF 确认旧 GNSS queue/history 已隔离或 reset | 任一 mapper/joiner/gate ACK 缺失则保持 GNSS invalid；global EKF 未隔离旧数据不得恢复 GNSS 输入 |
| GPS week rollover（连续运行） | 有可信 week 证据，iTOW rollover 与 source time 连续；无 reboot 证据 | receiver_generation 不变；source clock mapping 更新其 week anchor/映射段 ID | 不调用 joiner.reset()/begin_generation()，除非实际 source clock identity 按接口设计需变化；必须保证连续 source_timestamp_ns 单调且相同 iTOW 不跨周混配 | mapper ACK 连续时间映射；joiner 输出 source timestamp 严格递增；gate 仅接受递增 source time | week 不确定、时间不连续或同 iTOW 跨周歧义时停止 GNSS 输出，转入 receiver-time recovery，不猜测 generation |
| GPS week unknown / ambiguous | mapper 无法唯一确定 GPS week 或 boot identity | 暂不递增任何 generation，标记 time mapping invalid；确认重启后再递增 | 立即 GNSS invalidation；停止构造 GnssObservation；不允许以 host receive time 替代 source time | mapper 重新取得可信 week/boot anchor；随后按 rollover 或 reboot 两条路径恢复 | 保持 GNSS invalid；不能通过 reset joiner 后随意 begin_generation 绕过不确定性 |
| Point-LIO tracking lost，但独立腿式 odom 健康 | 厂商腿式 odom 已证明独立、连续、时戳/covariance/TF 合格；目前该硬件证据尚缺 | 视 LIO 是否重置递增 estimator_epoch；local_odom_epoch 不变 | 隔离 LIO 输出与旧 epoch cloud/TF；如重启 LIO，由 LIO owner reset state/P/map/queues；local EKF 保留腿式 odom 路径 | LIO 新 epoch tracking ACK；local EKF 持续收到独立腿式 odom；local TF age/covariance 正常 | 没有独立 odom 证据时不得假设 local pose 可继续；停止或进入经验证的降级模式 |
| Point-LIO 是 local pose authority 且 reset/lost 造成不连续 | 没有其他已验证 source 能保持 local odom continuity，或 LIO reset 改变 odom 原点 | estimator_epoch +1；local_odom_epoch +1；map_epoch 仅在 global datum/地图坐标解释也改变时递增 | stop/inhibit → LIO invalidation/reset → local EKF history/queue/TF reset → local odom 新 epoch → global EKF 重新建立 map→odom → 旧 local elevation/dynamic layers 按策略清理 | LIO tracking 与 local EKF measurement ACK；新 odom→base_link TF 连续于新 epoch；global map→odom 在同一时刻查询成功；Costmap 新观测更新 | 任一 TF/pose/covariance/Costmap gate 未通过则不 commit，保持停车/goal inhibit |
| local EKF reset / odom origin jump | local filter reset 被显式请求或检测到输出不连续；须先确认不是单条 outlier | local_odom_epoch +1；global map_epoch 通常不递增，但旧 map→odom validity 撤销直至重建 | 冻结控制 → local pose invalid → drain local filter history/queues/TF → reset local EKF → 新 odom identity → global EKF 在新 local TF 上重建 map→odom → 重置依赖旧 odom 的局部地图/规划 | local EKF ACK 新 generation；唯一 odom→base_link authority；global EKF 同 measurement-time TF lookup 成功；costmap 以新 odom epoch 重建 | 新 local odom 未稳定前保持停止；不能把新原点沿用旧 frame/epoch 标签 |
| global datum/map origin change | 明确执行 datum/地图基准更换，而非普通 GNSS dropout | map_epoch +1；local_odom_epoch 不变 | inhibit global goals → global pose invalid → global EKF reset/re-datum → 清理或重投影旧 map-frame layers/goals → 等待 map→odom 新 epoch | global EKF datum ACK；map→odom TF 唯一且同 local epoch；global costmap 和 global planner 输入新 map epoch | global localization 不可用；local odom 可保留，但不能运行依赖旧 map 的全局导航 |
| ROS clock rewind / bag loop | ROS clock epoch 发生回退；host steady clock 与 ROS clock 分开监测 | 新建 ros_clock_epoch；是否递增其他 epoch 由组件 reset 影响决定 | 先 inhibit 所有时间相关 consumer → 清空 ROS-time message queues/message filters/TF buffers → LIO/EKF 各自按已验证 reset contract 处理 → GNSS mapper 检查 receiver clock 与 ROS clock 映射 → 新 clock epoch 后再恢复 | 所有受影响 producer/consumer ACK 清空旧 ROS-time history；新 TF stamp 与 clock epoch 一致；measurement age 合格 | 任何组件未清旧时间缓存则保持 inhibit；不能依赖 robot_localization reset_on_time_jump 自动完成事务 |
| 仅 local/global Costmap observation stale | 定位 pose/TF 仍有效；仅特定 observation source age 超限或 source current=false | 所有定位/map epoch 不变；对应 costmap observation generation 可单独标记 invalid/recovered | 禁止受影响 costmap 的规划/控制消费；隔离 stale observation；恢复后等待新原始观测更新 bounds/costs | 对应 layer/source 报告新观测 stamp、TF transform 和 update 成功；Costmap current 恢复 | 不 reset LIO/GNSS/EKF；若 obstacle clearance 不可证明，则 stop rather than assume free space |

### 22.3 GNSS 子事务的确定调用顺序

针对 receiver reboot / source clock identity change，按当前 API 的最小安全顺序：

1. coordinator 先将 GNSS validity 置 false，并在 adapter 发布路径上关闭 NavSatFix gate；这是跨组件 invalidation barrier。
2. 停止新 UBX callbacks 进入 joiner，并等待正在执行的 add_* 调用退出；旧 callback 携带旧 generation token，之后即使迟到也必须拒绝。
3. 调用 joiner.reset()。该调用清除 pending/retired/epoch ordering 状态并要求显式 begin_generation()；旧 generation 不可继续进入。
4. 调用 gate.reset()，清除旧 generation 的 recovery counter 与 jump reference。
5. 重建 receiver time mapper。先判定是真正 reboot、新 GPS week rollover，还是 clock identity 不明；不允许用 reset 操作替代判定。
6. 仅在 mapper 给出可信新 generation ID 与单调 source timestamp mapping 后，调用 joiner.begin_generation(new_source_clock_id)。
7. 接收新四路消息并完成 exact-iTOW pairing；每个完整 observation 只调用 gate.evaluate() 一次。
8. 只有 Gate 返回 ACCEPTED 且 epoch、source clock、host freshness 均匹配当前 transaction 时，adapter 才发布 NavSatFix，并同步发布带相同 accepted epoch 的 quality status。
9. global EKF consumer 需确认旧 generation measurement/history 已隔离，且新的 map→odom 与 local TF 对齐；否则 GNSS measurement 可以被质量 Gate 接受，但不得恢复 global pose validity。

注意：Joiner 的累计诊断计数跨 generation 保留，推荐 diagnostics 同时输出 lifetime counters 与 generation-local counters（后者由 coordinator 在切换时保存 baseline 计算）。

### 22.4 Coordinator 最小纯算法接口（进一步收敛）

在不引入 ROS 依赖的前提下，建议最小接口分成三层：

- GnssGenerationController：输入 ReceiverClockEvidence；输出 ROLLOVER_CONTINUOUS、REBOOT_NEW_GENERATION、CLOCK_AMBIGUOUS；只有前两种有对应可执行路径。
- ResetCoordinator：输入 ResetTrigger；输出 immutable ResetPlan 和当前 transaction phase；按单线程事件序列消费 ComponentResetAck；禁止回调直接修改 epoch counters。
- EpochFence：对每条 GNSS observation、Odometry、TF metadata、Costmap observation 和 cmd intent 校验 boot identity + relevant epoch tuple + transaction generation；拒绝旧数据。

不得让 GNSS generation controller 单凭 iTOW 倒退判定 reboot；GPS week rollover 时 iTOW 自然回绕，必须结合 GPS week 与 source time continuity。

### 22.5 对应测试与通过条件

| 测试组 | 关键输入 | 通过条件 |
|---|---|---|
| Gate/Joiner reset semantics | reset 前后读取 counter、pending、retired、last timestamp、generation 状态 | Gate recovery/reference 清零；Joiner 要求显式 begin_generation；累计 counters 保留且有明确定义 |
| GNSS generation switch | 旧 generation 延迟 callback 与新 generation 消息交错 | 旧 callback 全拒绝；新 generation pending 从空状态开始；旧 Gate reference 不影响新恢复计数 |
| Week rollover | iTOW 从周末到周初，GPS week +1，source time 连续 | generation 不变；完成 epoch 严格递增；无跨周误配 |
| Reboot vs ambiguous clock | iTOW 回退但无 week/boot 证据；随后补充可信 reboot evidence | ambiguous 阶段不发 NavSatFix；证据确认后才开始新 generation |
| Gate exactly-once | 同一 completed epoch 重复回调或 adapter 重试 | Gate 只评估一次；NavSatFix 只发布一次；重复不会增加 recovery sample count |
| EKF history fence | global EKF queue 中混入旧 receiver generation 测量 | 旧测量在进入 filter 前被隔离；若无法确认则 global pose invalid |
| Fail-closed ACK | mapper/joiner/gate/global EKF 任一 reset/ACK 失败或超时 | transaction ABORTED_FAIL_CLOSED；GNSS invalid，global goal inhibit；local odom 按独立 validity 决定是否保留 |

阈值（pair skew、stale age、ACK timeout、恢复样本数）仍须由真实 ZED-F9P 数据、host load 和安全分析标定，本节不填入猜测数值。

### 22.6 证据边界

- Joiner/Gate 的方法行为来自本项目当前 nav 分支源码静态核验；本轮没有重新运行单元测试。
- Transition table 是 E3 项目设计规格；未实现 coordinator、mapper、EpochFence 或 ROS adapter。
- Lite3 腿式 odometry 独立连续性、ZED-F9P reboot/week 证据、global EKF history 清理接口和平台 stop ACK 均仍是外部 Gate。
- 未运行 build、bag、仿真、HIL 或实机测试。
## 23. GNSS core 包工程化前置条件与 coordinator 实施边界

本轮静态核对当前 nav 分支的 package.xml、setup.py、setup.cfg 与两个测试模块，目的是确定 transport-independent coordinator 能否安全地落入现有包。

### 23.1 当前包形态

| 项目 | 当前事实 | 工程影响 |
|---|---|---|
| ROS build type | ament_python | 现有 core 是 ROS 2 workspace 可识别的 Python 包，但其算法代码不依赖 ROS transport |
| package runtime dependencies | 仅 setuptools；未声明 rclpy、sensor_msgs、nav_msgs、diagnostic_msgs 或 u-blox ROS messages | 不应把 ROS adapter 直接塞入 core 的默认 import/runtime path |
| Python packaging | setup.py + setup.cfg；未发现 pyproject.toml 或 requirements.txt | 依赖与构建声明较轻，但 Python 最低版本/运行时矩阵尚未明确 |
| test framework | 测试模块继承 unittest.TestCase；setup.cfg 配置 pytest testpaths=test、python_files=test_*.py；package.xml 声明 python3-pytest | 可继续沿用 unittest 风格并由 pytest/colcon test 驱动；需在实际 workspace 验证 runner 行为 |
| license | package.xml 与 setup.py 均为 UNSPECIFIED | 发布、复用、拆分 adapter 前的治理阻断；不得自行猜测 SPDX 标识 |
| maintainer | email 为 maintainers@example.invalid | ROS 包元数据未达到正式发布条件；须由用户/项目维护者提供真实责任主体 |

### 23.2 推荐的代码边界（不改变现有包的 transport independence）

建议后续将纯算法 coordinator 放在现有 robodog_gnss_quality 包中，前提是用户/项目维护者先确认该包的 SPDX license 和 maintainer metadata。推荐模块职责：

- generation.py：接收经过验证的 ReceiverClockEvidence，判定 CONTINUOUS_WEEK_ROLLOVER、NEW_RECEIVER_GENERATION 或 AMBIGUOUS；不解析串口字节，不从 host receive time 推造 receiver time。
- coordinator.py：串行消费 ResetTrigger，产生 immutable ResetPlan/phase/ack outcome；负责 joiner.reset()、gate.reset() 与显式 begin_generation() 的顺序和 generation fence。
- epoch_fence.py：提供纯函数校验 boot/session identity、receiver generation、相关 epoch tuple 和 reset transaction generation；不依赖 ROS message 类型。
- diagnostics.py：定义 lifetime counters 与 generation-local counters 的数据模型；具体 ROS diagnostics publisher 留给独立 adapter。
- ROS adapter：后续独立 package（名称待定），依赖真实选定的 u-blox ROS driver/message package；负责 ROS clock/steady clock、消息解码、NavSatFix 与 quality status 发布及 navsat_transform 对接。

不得在现有 core 中导入 rclpy 或具体 ROS message class；不得让 core 直接创建 NavSatFix；不得将 receiver GPS week/iTOW mapper 假设为已存在。

### 23.3 当前可先做与必须等待的工作

| 工作 | 是否可先做 | 前置条件 |
|---|---|---|
| 为现有 Gate/Joiner reset 语义增加回归测试 | 可以 | 不改变 license/maintainer 声明；在当前测试风格下增加 unittest |
| 纯算法 generation evidence model 与判定状态表 | 设计可先做；代码前需治理确认 | GPS week/boot evidence 字段先定义，不虚构 ZED-F9P 实际消息映射 |
| ResetCoordinator/ EpochFence 纯算法实现 | 暂缓提交代码 | 先确认包 SPDX license、maintainer；明确 API 后再实现 |
| GPS week/iTOW mapper 的真实时间算法 | 部分可做 | 可先定义数学/边界测试；最终 boot/week 判据必须由接收机协议与真实日志证实 |
| u-blox ROS adapter | 不可冻结依赖 | 先审查实际选定驱动、Jazzy compatibility、消息定义、source timestamp 与 license |
| NavSatFix/global EKF integration | 不可宣称完成 | 需 adapter、epoch fence、global filter history 隔离与真实 bag 测试 |

### 23.4 当前审计结论

现有包结构足以容纳 transport-independent Python 状态机，但 license/maintainer 元数据尚未达到可发布治理要求。由于本轮没有获得真实 SPDX 许可和维护者身份，本轮只记录实现边界，不创建代码、不修改元数据、不引入 ROS dependencies。静态文件检查未运行 colcon build、pytest 或 unittest；不得将包结构审计视为构建通过。

