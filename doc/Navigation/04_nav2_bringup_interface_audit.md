# 04 — Nav2 bringup 到运动命令链：Jazzy 接口审计（N1 子任务）

更新日期：2026-11-25。研究对象是完整导航模块中的 Nav2 子系统边界；本文只记录固定源码快照可证实的 bringup、节点、命令 topic 和运行参数，不代表本项目已经启动或完成 ROS 集成。

## 1. 证据基线

- 上游仓库：https://github.com/ros-navigation/navigation2
- 分支：jazzy
- 固定 commit：`c7046550547d382a5186d106b0895eda02485d7c`
- 关键源码：
  - [navigation_launch.py](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bringup/launch/navigation_launch.py)
  - [bringup_launch.py](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bringup/launch/bringup_launch.py)
  - [nav2_params.yaml](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bringup/params/nav2_params.yaml)
  - [controller_server.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_controller/src/controller_server.cpp)
  - [velocity_smoother.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_velocity_smoother/src/velocity_smoother.cpp)
  - [collision_monitor_node.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_collision_monitor/src/collision_monitor_node.cpp)

## 2. Bringup 结构与 Lifecycle

`navigation_launch.py` 在非 composition 模式下分别启动 controller_server、smoother_server、planner_server、route_server、behavior_server、bt_navigator、waypoint_follower、velocity_smoother、collision_monitor 和 docking_server；随后由 `nav2_lifecycle_manager/lifecycle_manager` 管理 lifecycle_nodes 列表。composition 模式通过 LoadComposableNodes 将对应组件装入指定 container。该 launch 文件对 `/tf` 和 `/tf_static` 做相对命名空间 remap，并使用同一份 RewrittenYaml 参数文件。

`bringup_launch.py` 将 localization/SLAM launch 与 navigation launch 组合：`slam` 与 `use_localization` 参数决定是否引入 slam_launch 或 localization_launch；它不意味着 Nav2 本身提供完整传感器驱动或 SLAM 算法。

**架构结论：** Nav2 bringup 本身是由多个 Lifecycle 节点、Lifecycle Manager、插件和外围定位/地图启动组合而成，不是一个单体导航节点。部署审计必须同时检查 launch 条件、composition、namespace、remap、参数重写和 Lifecycle 状态。

## 3. 命令链与 topic remap（源码事实）

基于 `navigation_launch.py`：

| 阶段 | 节点/组件 | 源码默认接口/重映射 | 说明 |
|---|---|---|---|
| 局部控制输出 | controller_server | 节点内 `cmd_vel` → launch remap `cmd_vel_nav` | Controller Server 通过 FollowPath action 接收路径并发布速度意图 |
| 行为输出 | behavior_server | `cmd_vel` → `cmd_vel_nav` | Spin/BackUp/DriveOnHeading 等行为输出汇入导航命令通道 |
| 速度平滑输入 | velocity_smoother | launch 将输入 `cmd_vel` remap 为 `cmd_vel_nav` | 与 controller/behavior 共用输入通道 |
| 速度平滑输出 | velocity_smoother | `cmd_vel_smoothed` | 受速度、加速度、减速度和超时参数约束 |
| 碰撞监控输入 | collision_monitor | 默认 `cmd_vel_smoothed` | 读取传感器 observation sources 与 TF，评估 stop/slowdown/limit/approach |
| 碰撞监控输出 | collision_monitor | 默认 `cmd_vel` | 输出最终软件监控后的 Twist 命令；下游平台适配器并非 Nav2 自动提供 |

注意：这里的 topic 名是该固定 bringup 与默认参数的接口，不应未经检查就认定为本项目最终名称。自定义 namespace、remap、参数文件和 stamped/unstamped 选择会改变实际接口。

## 4. 节点内部接口和失效行为

### 4.1 Controller Server

- 源码通过 `nav2_util::TwistPublisher` 发布 `cmd_vel`，通过 `nav_2d_utils::OdomSubscriber` 订阅 odometry。
- 建立 `follow_path` action server；读取 FollowPath goal 中的 path、controller_id、goal_checker_id、progress_checker_id。
- 使用 pluginlib 装载 Controller、GoalChecker、ProgressChecker；控制循环默认参数来自 bringup YAML，示例值 controller_frequency=20 Hz、costmap_update_timeout=0.30 s。
- 每轮先检查 Costmap 是否 current，再获取 robot pose、progress checker、odom twist 并调用 controller plugin 计算速度。
- 对 TF 错误、无有效控制、进度失败、路径无效、控制超时等异常，源码路径会发布零速度并以对应 error_code 结束 action；取消成功时也发布零速度。
- deactivate 路径会发布零速度，但这只是 ROS 层命令，不证明物理机器狗已经停止。

### 4.2 Velocity Smoother

- 通过 `nav2_util::TwistSubscriber` 订阅 `cmd_vel`，通过 `nav2_util::TwistPublisher` 发布 `cmd_vel_smoothed`；subscriber 回调支持 Twist 与 TwistStamped 两种回调入口。
- 对 NaN/Inf 命令拒绝输入；读取 max/min velocity、max_accel、max_decel、feedback、odom_topic、velocity_timeout 等参数。
- OPEN_LOOP 不以里程计反馈闭环估计当前速度；CLOSED_LOOP 才创建 OdomSmoother 读取 odom。
- 默认样例配置为 20 Hz、OPEN_LOOP、速度超时 1.0 s；这些是样例配置，不是 Lite3 的安全限值。
- 本项目应以 Lite3 供应商确认的运动边界、实测制动能力、反馈新鲜度和四足步态限制重新确定参数，不能直接复制默认值。

### 4.3 Collision Monitor

- 默认输入 `cmd_vel_smoothed`、输出 `cmd_vel`，并通过 `nav2_util::TwistSubscriber` 接受 stamped/unstamped Twist。
- 数据源支持 scan、pointcloud、range、polygon；来源数据过期且 source_timeout 非零时，处理逻辑将当前动作切换为 STOP 并将请求速度置零。
- 多边形可配置 STOP、SLOWDOWN、LIMIT、APPROACH 等动作；具体行为由 polygon 类型和配置决定。
- 默认示例使用 scan 源和 FootprintApproach polygon；这不适用于未经修改的 MID-360 PointCloud2 + 双目深度输入。
- 输出零速度或软件碰撞监控动作不等于独立安全认证、驱动禁能或物理急停。Nav2 之外仍须有平台侧 watchdog、命令仲裁和物理 E-stop。

## 5. 默认参数的四足适配风险

固定 commit 的示例 `nav2_params.yaml` 包含如下默认/示例假设：

- controller 插件示例为 `nav2_mppi_controller::MPPIController`，运动模型为 `DiffDrive`；该配置不能被解释为已经建模 Lite3 的横移、原地旋转、步态切换、机身姿态或足端落点约束。
- 速度、加速度、控制频率、costmap 尺寸/分辨率、robot_radius、障碍源 topic 和 Collision Monitor polygon 均为示例参数；不可作为实机安全限值。
- local costmap 示例 observation source 为 LaserScan `/scan`，而项目主几何源是 MID-360 PointCloud2；必须重新设计 source、height filtering、voxel/elevation/traversability 表达及时间戳策略。
- 默认 Collision Monitor 示例采用 LaserScan source；切换为 PointCloud2 时需核验 pointcloud source 的字段、frame、TF、时间戳和 source timeout。
- 2D Costmap/Collision Monitor 不提供四足落足规划、负障碍完整检测或认证级急停。

## 6. 本项目建议的逻辑命令链（尚待集成）

`Controller/Behavior` → `cmd_vel_nav` → `Velocity Smoother` → `cmd_vel_smoothed` → `Collision Monitor` → `cmd_vel_safe`（建议项目自定义明确命名）→ **独立 Safety Supervisor / command arbiter** → **Lite3 command adapter** → 厂商 locomotion SDK → 机器人；实际机身速度/姿态/足端/关节状态回传 State Estimation 与 supervisor。

说明：
1. 当前上游默认输出名是 `cmd_vel`，建议项目层通过显式 remap 将安全输出命名为 `cmd_vel_safe`，避免与未经安全仲裁的通道混淆；这是设计建议，不是 Nav2 固定接口。
2. Safety Supervisor 必须独立监控定位健康、传感器 freshness、命令来源/模式、速度边界、通信超时、平台状态和 E-stop。
3. 平台 adapter 不能把 Twist 直接映射为关节命令；必须根据厂商确认的速度/机身/步态接口转换并检查 ACK、拒绝原因和执行反馈。
4. Collision Monitor 与独立安全监督应分层，不能把软件监控当成物理急停。

## 7. N1 接口矩阵增量

| Producer | Consumer | 接口 | 证据状态 | 尚待核验 |
|---|---|---|---|---|
| BT Navigator / task BT | Planner Server | ComputePathToPose action（需进一步源码追踪） | 上游固定源码已确认相关服务器启动 | action 名/goal/result/error、preemption/cancel 调用链 |
| BT Navigator / task BT | Controller Server | FollowPath action | action 定义已在 03 文档核验；Server 建立位置已核验 | 完整 BT 节点到 action client 的调用链 |
| Controller Server | Velocity Smoother | `cmd_vel_nav`（由 launch remap） | launch + server publisher 源码 | 实际运行 graph、QoS、stamped/unstamped 参数 |
| Behavior Server | Velocity Smoother | `cmd_vel_nav`（由 launch remap） | launch remap 源码 | 行为并发/仲裁与 action 时序 |
| Velocity Smoother | Collision Monitor | `cmd_vel_smoothed` | 参数文件 + smoother publisher | 实际消息类型、时钟、timeout 与频率 |
| Collision Monitor | Platform Safety/Adapter | 默认 `cmd_vel` | 参数文件 + Collision Monitor publisher | 项目安全仲裁、adapter、SDK ACK/反馈均未实现/未确认 |
| LiDAR/Camera | Collision Monitor | scan/pointcloud source | Collision Monitor 支持的 source 类型已核验 | 本项目 PointCloud2 source 配置、真实 topic/QoS/TF/时延 |
| Estimator | Controller/Costmap/Collision Monitor | odom/TF | Controller 依赖 OdomSubscriber 与 TF；Collision Monitor 依赖 TF | 唯一 TF authority、真实 Lite3 odom 和 freshness |

## 8. 研究结论与后续动作

1. Nav2 速度输出链存在明确的多级通道，必须按 bringup remap 还原；不能把 Controller Server 的 `cmd_vel` 直接视为最终机器人命令。
2. Collision Monitor 的 source timeout 可触发软件 STOP，但若 Collision Monitor 自身进程、DDS、主机或下游 adapter 失效，不能保证物理停止；需要独立安全链。
3. 默认参数明显是通用示例，不应直接应用于 Lite3；尤其 DiffDrive、LaserScan、速度/加速度限制、footprint 和 collision zones 均需重做。
4. 当前审计为 E1/E2 源码级证据，未达到 ROS 集成 E4、硬件 E5 或 HIL/E6。
5. 下一步继续 N1：追踪 Planner Server / BT Navigator action client、local/global Costmap 数据生产者与传感器观测链；并将传感器、定位、地形、规划、执行、安全链逐边填入接口矩阵。


## 8. BT Navigator → Planner/Controller Action 链（N1 追加核验）

新增固定源码入口：
- [bt_navigator.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bt_navigator/src/bt_navigator.cpp)
- [navigate_to_pose.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp)
- [planner_server.cpp](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_planner/src/planner_server.cpp)
- [ComputePathToPose BT node](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp)
- [FollowPath BT node](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_behavior_tree/plugins/action/follow_path_action.cpp)

源码核验结果：

1. `BtNavigator::on_configure()` 创建 TF Buffer/Listener，读取 global_frame、robot_base_frame、transform_tolerance、odom_topic；加载内置 BT plugin 列表和用户插件，并通过 pluginlib 加载 NavigateToPose / NavigateThroughPoses navigator plugins。它还创建 OdomSmoother 读取 odom，用于导航反馈速度估计。
2. `NavigateToPoseNavigator::configure()` 创建对自身 action name 的 action client，并订阅 `goal_pose`（PoseStamped，SystemDefaultsQoS）作为 RViz/外部目标输入；配置默认 blackboard key `goal` 与 `path`。Goal 中的 `behavior_tree` 非空时用于加载指定 BT XML；未指定时默认使用 `navigate_to_pose_w_replanning_and_recovery.xml`。
3. 默认 BT 由 BT action nodes 作为 action clients：`ComputePathToPose` 连接 `compute_path_to_pose`，从 blackboard 读取 goal/planner_id，成功时把 result path 写回 blackboard；失败时写入 error_code 并返回 FAILURE。 `FollowPath` 连接 `follow_path`，从 blackboard 读取 path/controller_id/goal_checker_id/progress_checker_id；运行中若 blackboard 的 path 或插件 ID 改变，会更新 action goal，形成重规划/路径更新接口。
4. `PlannerServer::on_configure()` 创建 global Costmap2DROS 并启动 NodeThread，pluginlib 加载 GlobalPlanner；创建 `compute_path_to_pose` 与 `compute_path_through_poses` 两个 action server，并发布 `plan`（nav_msgs/Path）。示例参数默认 GridBased→NavFn，但这不是项目必须采用的算法。
5. 因此源码层的核心任务链可明确为：外部 NavigateToPose goal → BtNavigator 的 navigator plugin/BT → ComputePathToPose action client → PlannerServer action server → Global Costmap + GlobalPlanner plugin → nav_msgs/Path/blackboard → FollowPath action client → ControllerServer action server → Controller plugin + Local Costmap + odom/TF → velocity command。

### 8.1 已确认与未确认边界

- 已确认 action 名称和主要 goal 输入：`compute_path_to_pose`、`follow_path`；Planner Server 同时提供 `compute_path_through_poses`。
- 已确认 BT blackboard 在路径计算与路径跟踪间传递 Path；FollowPath BT 节点支持在运行中检测 path/plugin ID 变化并更新 action goal。
- 已确认 Planner/Controller 分别持有 Global/Local Costmap2DROS，且启动独立 NodeThread；Costmap 不是一个只由 planner/controller 回调同步计算的普通函数。
- 尚未完整追踪 BT XML 的所有节点、recovery 分支、goal preemption/cancel 的每个调用路径；尚未确认本项目实际 BT XML 和自定义插件。
- 源码固定的 action/节点/接口不等于实际 ROS graph；topic 名、QoS、线程调度和运行频率仍需 launch 实测。

### 8.2 N1 端到端链路状态更新

| 链路 | 当前证据 | 状态 |
|---|---|---|
| NavigateToPose → BT Navigator | Navigator plugin、BT XML 选择、goal/path blackboard key | E2 源码核验 |
| BT → ComputePathToPose → Planner Server | BT action client、action 名、goal/path/error_code、Global Costmap + plugin | E2 源码核验 |
| BT → FollowPath → Controller Server | BT action client、path/controller/checker IDs、运行中更新 goal | E2 源码核验 |
| Planner/Controller → Costmap | Global/Local Costmap2DROS 与 NodeThread | E2 源码核验 |
| Controller → Smoother → Collision Monitor | launch remap 与节点 publisher/subscriber | E2 源码核验 |
| Collision Monitor → Lite3 实际运动与反馈 | 仅存在 Nav2 输出边界；项目 adapter/SDK/feedback 未接通 | 未完成 / vendor Gate |
