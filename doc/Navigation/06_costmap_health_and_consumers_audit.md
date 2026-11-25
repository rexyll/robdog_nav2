# Costmap 健康状态与 Planner/Controller 消费端审计

- 专项分支：`nav`
- 审计日期：2026-11-25
- 上游固定版本：Navigation2 Jazzy `c7046550547d382a5186d106b0895eda02485d7c`
- 证据等级：E2（固定上游源码审计）；未执行本项目 ROS 节点、仿真或实机验证

## 1. 目的与结论

本审计将上一份 Costmap 生产端审计（`05_sensor_to_costmap_observation_audit.md`）与 Planner Server、Controller Server 的消费端连接起来，回答 Costmap 的 `isCurrent()` 是否真正影响规划和控制。

**结论：Planner 与 Controller 都显式等待 Costmap 变为 current；等待超时会终止当前 Action。Controller 超时路径明确发布零速度。** 但这不是持续的传感器健康安全监控：它检查的是 Costmap layer/filter 的 current 聚合状态，且只在等待/控制循环的特定位置检查；它不自动保证每个 observation 的数据龄期、几何质量、动态障碍跟踪质量或机器人实际停止。

## 2. Planner Server 消费端

源码：`nav2_planner/src/planner_server.cpp`

### 2.1 初始化与参数

- `PlannerServer` 默认声明 `costmap_update_timeout=1.0` 秒，成员初值为 1s。
- configure 时创建 `global_costmap` 的 `Costmap2DROS`，取得 Costmap，并启动独立 `NodeThread`。
- configure 时读取参数并转为 `rclcpp::Duration`，用于后续等待。
- Planner action 包括 `compute_path_to_pose` 和 `compute_path_through_poses`。

### 2.2 等待条件与失败路径

`waitForCostmap()` 以约 100 Hz 循环检查 `costmap_ros_->isCurrent()`。只要返回 false，就继续等待；当 `now() - waiting_start > costmap_update_timeout_` 时抛出 `nav2_core::PlannerTimedOut`。

两个规划 Action 的计算入口都在开始正式取起点/调用 planner 前调用该等待函数。因此这不是单纯的可观测状态，而是规划前置 Gate。

超时被捕获后：
- `compute_path_to_pose` 返回对应的 `TIMEOUT` error code，并 `terminate_current(result)`；
- `compute_path_through_poses` 同样返回 `TIMEOUT` 并终止当前 goal；
- 该超时路径没有在 Planner Server 中发布运动命令（规划器本身只产出路径）。

### 2.3 边界

- Planner 等待的是 Costmap 聚合 current 状态，不直接逐条检查 LaserScan/PointCloud2 时间戳或点云质量。
- 规划成功只表示 planner 基于当前可用 Costmap 生成了路径；不等于局部控制器能跟踪，也不等于路径在执行时仍安全。
- `costmap_update_timeout` 是规划入口等待预算，不是全局定位、传感器或导航任务的端到端超时。

## 3. Controller Server 消费端

源码：`nav2_controller/src/controller_server.cpp`

### 3.1 参数与循环位置

- 默认 `costmap_update_timeout=0.30` 秒（成员初值 300ms）。
- 默认 `controller_frequency=20.0` Hz。
- `computeControl()` 收到 FollowPath goal 后进入控制循环；每轮控制循环在更新全局路径、判断目标是否到达、计算并发布速度之前，检查 `costmap_ros_->isCurrent()`。
- Costmap 非 current 时以约 100 Hz 等待；超过 timeout 抛出 `ControllerTimedOut`。

### 3.2 超时与停车动作

`ControllerTimedOut` 被显式捕获。异常路径按顺序：
1. 记录错误；
2. 调用 `publishZeroVelocity()`；
3. 将 FollowPath result 的 error code 设为 `CONTROLLER_TIMED_OUT`；
4. `terminate_current(result)` 终止当前控制 Action。

其他 Controller 异常（如 TF error、NoValidControl、InvalidPath、未知异常）也有相应零速度处理路径，但错误分类和恢复语义不同，不能统称为 Costmap stale。

### 3.3 检查并不覆盖整个执行周期

- Controller 在每个控制循环的计算前检查 current；如果某次 Costmap 在检查后、速度命令发布前变 stale，单凭该 Gate 不会提供原子性保证。
- 当前状态可能在下一轮循环才被观察到；控制循环频率、Costmap 更新频率、线程调度和超时共同影响响应时间。
- 该机制不等价于 Collision Monitor、独立硬件 watchdog 或底层急停。零速度消息也不证明四足底层已接受并完成制动。

## 4. 与 LayeredCostmap current 聚合的关系

结合 `nav2_costmap_2d/src/layered_costmap.cpp` 的 `isCurrent()` 实现：
- 聚合逻辑遍历启用的 layers 与 filters；
- 只要其中一个启用组件报告非 current，整体就非 current；
- 所有启用组件均 current，整体才为 true。

因此一个 layer 的 stale/未初始化状态可能同时阻断 global planner 或 local controller，具体影响取决于它们各自 Costmap 配置。Planner 使用 global_costmap；Controller 使用 local_costmap，两者不能假设共享同一 current 状态。

需要特别注意：上一份审计已确认 ObservationBuffer 的 `expected_update_rate=0` 分支不做 stale 时间检查。若具体 layer 的 current 语义在该配置下始终为 true，则 Planner/Controller 的 current Gate 无法弥补该 observation freshness 缺口。项目配置必须显式设定合理的更新期望，并另行监控数据龄期。

## 5. 时间语义与暂停仿真风险

两处等待逻辑均以节点 `now()` 计算 elapsed timeout；循环休眠使用 `rclcpp::Rate`。源码审计确认了计时源与循环结构，但未在本项目验证 use_sim_time、/clock 暂停/跳变时的实际行为。

因此设计与测试时必须覆盖：
- ROS time 正常推进；
- 仿真 /clock 暂停；
- ROS time 回拨/跳变；
- 传感器停流但 Costmap layer 仍报告 current；
- TF 暂时不可用；
- 单个 layer/filter 失去 current；
- Costmap 更新频率低于控制频率。

不要只依赖单个 ROS-time timeout 作为物理安全 watchdog。独立安全层应使用可验证的时钟/硬件机制，并能在底层执行器确认停车。

## 6. 项目级建议（设计要求，尚未实现）

1. **分层健康状态**：分别暴露 sensor transport、message age、TF availability、layer current、Costmap update duration、planner readiness、controller readiness。
2. **显式 freshness**：每个 PointCloud2/LaserScan source 配置合理的 expected update rate；额外检查 header stamp、接收时间、时钟域、丢包和连续超时。
3. **区分全局与局部地图健康**：global_costmap 与 local_costmap 独立监控，禁止用一方 current 推断另一方 current。
4. **控制器 stale 策略**：Costmap 不健康时停止产生新速度命令并请求安全停止；停止是否成功必须通过平台反馈确认。
5. **故障传播**：将 planner timeout、controller timeout、TF error、sensor stale、localization degraded 映射为不同任务状态和恢复策略。
6. **独立 watchdog**：不得将 Nav2 的零速度 Twist 视为急停；平台适配层应有命令超时、限速、失联停车与硬件急停接口。
7. **遥测记录**：记录各 source header stamp、接收时间、QoS、layer current、Costmap update duration、FollowPath feedback/result、最终平台速度/姿态反馈。

## 7. 后续验证矩阵

| 场景 | 注入方式 | 预期观察 | 当前证据 |
|---|---|---|---|
| global layer stale | 停止 global source 或令 layer stale | Planner 等待并在 timeout 后返回 TIMEOUT | E2 源码；未运行 |
| local layer stale | 停止 local source 或令 layer stale | Controller 等待，发布零速度并返回 CONTROLLER_TIMED_OUT | E2 源码；未运行 |
| 单个 plugin stale | 测试 layer/filter current=false | 对应 Costmap 整体非 current | E2 源码；未运行 |
| observation rate 未配置 | expected_update_rate=0 | 验证是否存在 stale 检测缺口 | E2 源码；需集成验证 |
| TF 丢失 | 阻断必要 transform | 区分 Costmap pose/update 与 Controller TF error | 设计场景；未运行 |
| /clock 暂停/跳变 | 仿真时钟注入 | 验证 timeout 与安全停止的真实时钟行为 | 未验证 |
| 零速度未执行 | mock adapter 丢弃命令 | watchdog 检测无执行反馈并进入安全状态 | 项目级待实现 |

## 8. 关键源码定位

固定上游版本：`https://github.com/ros-navigation/navigation2/tree/c7046550547d382a5186d106b0895eda02485d7c`

- `nav2_planner/src/planner_server.cpp`
  - constructor：`costmap_update_timeout` 默认 1.0s
  - `PlannerServer::waitForCostmap()`
  - `PlannerServer::computePlan()`
  - `PlannerServer::computePlanThroughPoses()`
  - `PlannerTimedOut` catch
- `nav2_controller/src/controller_server.cpp`
  - constructor：`costmap_update_timeout` 默认 0.30s
  - `ControllerServer::computeControl()`
  - 控制循环内 `isCurrent()` 等待
  - `ControllerTimedOut` catch、`publishZeroVelocity()`
- `nav2_costmap_2d/src/layered_costmap.cpp`
  - `LayeredCostmap::isCurrent()` 聚合 layers/filters 状态

## 9. 证据边界

本文件只证明固定上游源码中存在上述逻辑（E2）。未证明本项目参数覆盖默认值、Costmap layer 在真实传感器停流时会正确置 stale、timeout 的实测时长、零速度到达 Lite3 adapter、底层执行器制动或真实机器狗安全停止。以上均需后续配置审计、ROS 集成测试、故障注入和硬件 Gate。
