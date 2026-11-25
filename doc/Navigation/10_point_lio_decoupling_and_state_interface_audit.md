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
