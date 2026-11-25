# 01 — 生命周期与导航任务完整流程

## 1. Bringup：启动不等于可导航

推荐依赖顺序：

1. 硬件/驱动：启动传感器和底层状态只读接口；启动运动 SDK 前确认厂商 SOP 与安全状态。
2. Robot Description：加载 URDF/Xacro，启动 robot_state_publisher；固定外参进入 TF。
3. 状态估计：启动本体 odom/LIO/VIO、IMU 融合、GNSS quality gate/global localization；验证 map→odom→base_link。
4. 地图/感知：Map Server 或 SLAM；LiDAR/双目预处理；检查时间戳、frame、QoS、频率。
5. Nav2 Lifecycle：Lifecycle Manager 对受管节点依次 configure、activate。
6. 命令安全链：Velocity Smoother、Collision Monitor、平台 adapter 和底层 watchdog；先在仿真/无运动模式验证。
7. 应用层：所有 readiness gate 通过后才允许发送导航 Action。

Lifecycle 常见状态：unconfigured → inactive → active；异常/关闭还涉及 error/finalized。configure 创建资源、加载插件和参数；activate 开始处理；deactivate 停止活动但保留配置；cleanup 释放资源。Lifecycle Manager 管理节点列表、启动顺序、超时和 bond 心跳；受管节点以 launch/YAML 为准。

## 2. NavigateToPose 概念时序

    Application --NavigateToPose(goal)--> BT Navigator
    BT Navigator --ComputePathToPose(start, goal)--> Planner Server
    Planner Server --Path/error--> BT Navigator
    [可选] BT Navigator --SmoothPath(path)--> Smoother Server
    BT Navigator --FollowPath(path)--> Controller Server
    Controller Server --cmd_vel--> Velocity Smoother
    Velocity Smoother --smoothed cmd_vel--> Collision Monitor
    Collision Monitor --safe cmd_vel--> Base Adapter
    Base Adapter --> Locomotion Controller --> Robot
    Robot --> odom/IMU/joints --> State Estimator --> TF/Costmap --> Nav2
    BT Navigator 持续监控结果；需要时重规划或调用 Behavior Server 恢复
    BT Navigator --result/feedback--> Application

该图是概念交互；实际 BT XML 决定 tick、重规划条件、重试、恢复和是否调用 Smoother。

## 3. 任务阶段

### 接收目标
应用调用 NavigateToPose 或 NavigateThroughPoses。BT Navigator 选择 BT XML，初始化 blackboard（goal、path、error code、retry counter 等）。目标 frame 必须能通过 TF 转换到规划全局 frame。

### 规划
Planner Server 从起点到目标生成全局 Path。输入包括 global costmap、起点/目标和 planner ID。失败原因可能是起终点位于 lethal/unknown cell、地图不连通、TF 失败、Costmap 未更新或运动模型不匹配。

### 平滑（可选）
Smoother Server 对 Path 做几何/运动学平滑。平滑必须保持避障、可行性和端点约束；不能仅凭视觉更顺滑就认定安全。

### 路径跟踪
Controller Server 托管 Controller、Goal Checker、Progress Checker，并持有 local costmap。Controller 周期读取 Path、当前 pose/twist 和环境，输出速度命令。路径不可跟踪、Costmap 超时、目标达成或进度失败会改变 Action 状态。

### 速度整形与碰撞监控
Velocity Smoother 按速度、加速度、减速度和 deadband 限制命令；可 OPEN_LOOP 使用上次命令，也可 CLOSED_LOOP 使用 odom。Collision Monitor 根据传感器观测检查 stop/slowdown/limit/approach 区域。随后 adapter 再做运动模式、步态和底层限制映射。

### 成功、失败与恢复
Goal Checker 判断是否到达；Progress Checker 判断是否在时间窗口内有足够进展。BT 可调用 Wait、Spin、BackUp、DriveOnHeading、清理 Costmap、重新规划或最终 abort。恢复行为必须经过同一安全命令链。

## 4. Action / Service / Topic / Parameter

- Action：长时任务、有反馈、可取消，如 NavigateToPose、ComputePathToPose、FollowPath、Spin。
- Service：短时请求，如清理 Costmap、加载地图、Lifecycle state change（具体接口以节点版本为准）。
- Topic：持续流，如 /tf、/odom、/scan、/points、/map、cmd_vel。
- Parameter：配置，如 controller_frequency、plugin list、frame IDs、QoS、timeout。

导航 Action 不是底盘控制接口。

## 5. 故障传播

| 故障 | 影响 | 推荐处理 |
|---|---|---|
| TF 失败/过期 | 观测不能投影，Costmap/Controller 报错 | 停止消费过期数据，禁止继续自主运动 |
| odom stale/跳变 | 速度估计和局部控制不可信 | watchdog 停车，重新初始化定位 |
| sensor timeout | Costmap/Collision Monitor 盲区 | 标记 source stale，限制或停止 |
| GNSS Fixed 降级 | 全局精度下降 | quality gate 降权/拒绝，必要时暂停地理任务 |
| Planner failure | 无路径 | 有界重试/恢复/退出 |
| Controller failure | 无控制量 | 零速度或底层 stop |
| BT/server crash | 任务中断 | supervisor 检测，底层 timeout stop |
| Adapter/base link loss | 命令不可达 | 底层独立 watchdog 停止 |

## 6. Readiness Gate

开放导航 Action 前检查：必要 lifecycle nodes active；时钟策略正确；TF 连通；odom freshness/covariance 合理；传感器数据年龄合格；Costmap 持续更新；命令链输入输出类型一致；Collision Monitor 有有效观测；底层控制器处于明确可控/停止状态。

参考：https://docs.nav2.org/jazzy/tutorials/general_tutorials/adding_a_nav2_task_server/ 、https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/
