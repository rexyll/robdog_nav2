# 04 — Nav2 算法与插件体系

## 1. 插件机制

Nav2 将算法实现与服务器生命周期/请求管理解耦。Server 接受 Action、管理 Costmap/TF/参数，按 pluginlib 注册名加载一个或多个插件。BT 或应用通过 planner/controller ID 选择策略。插件名、参数和默认值必须核对 Jazzy 安装版本，不能直接复制 Rolling 配置。

## 2. 全局 Planner

| 类别 | 方法 | 适合情况 | 风险/限制 |
|---|---|---|---|
| NavFn | 栅格 Dijkstra/A* 类搜索 | 规则二维地图 | 运动学表达有限 |
| Smac 2D | 2D A* 系列 | 栅格路径、成本权重 | 二维环境 |
| Smac Hybrid-A* | 状态含姿态/运动学约束 | 非完整约束、汽车式模型 | 参数/算力敏感 |
| Smac State Lattice | 离散运动原语/状态格 | 有明确运动原语的底盘 | primitives 必须匹配真实能力 |
| Theta* | 任意角/视线 shortcut | 减少栅格折线 | 仍需检查碰撞和曲率 |

不要直接把 Ackermann Hybrid-A* 套给四足。先确认是否横移、原地旋转、倒退及对应速度边界；第一版平地可用 2D planner 建 baseline。

## 3. 局部 Controller

| 类别 | 核心思路 | 优点 | 需验证 |
|---|---|---|---|
| DWB | Dynamic Window 轨迹采样 + critics 评分 | 可解释、参数丰富 | 运动模型和速度空间 |
| Regulated Pure Pursuit | 前视点跟踪，按曲率/障碍调速 | 简洁、轻量 | 动态避障能力和曲率 |
| MPPI | 采样控制序列、代价函数评价 | 可扩展 critic、模型预测 | Orin 算力、模型/时域/噪声 |
| Graceful | 连续平滑路径跟随 | 轨迹平滑 | 运动学和场景适配 |
| Rotation Shim | 先对齐路径方向再交给主 controller | 处理初始航向误差 | 原地转向能力和空间 |

建议对照：DWB 作为易诊断 baseline；RPP 作为低复杂度跟踪对照；MPPI 在模型和资源基线建立后评估。不能预先断言某个插件最佳。

## 4. Goal / Progress Checker

Goal Checker 判断到达位置/朝向容差；StoppedGoalChecker 还检查是否停止。Progress Checker 判断时间窗口内是否取得足够进展。四足步态和地面打滑会影响 odom，阈值需基于可信状态估计，避免把机身摆动误判为进展。

## 5. Smoother

Simple Smoother：轻量迭代；Savitzky-Golay：局部多项式拟合；Constrained Smoother：约束运动学/曲率优化。Smoother 只处理 Path，不替代 Controller 实时避障。平滑后需重新检查 Costmap 碰撞、曲率和端点。

## 6. Behavior / Recovery

常见 Spin、BackUp、Wait、DriveOnHeading、AssistedTeleop 等是 BT 可调用的原子行为。Recovery 是任务恢复，不是硬件急停。四足的 Spin/BackUp/DriveOnHeading 必须经 adapter，并确认步态、支撑相、倒退/横移能力和安全速度；未知能力时禁用。

## 7. Behavior Tree

BT 是任务编排，不是局部轨迹控制算法。节点类别包括 Action（调用规划/跟踪/恢复服务器）、Condition（定位/目标/路径/故障条件）、Control（Sequence、Fallback、RecoveryNode、PipelineSequence）、Decorator（重试、距离/时间条件、速率控制、超时）。

BT 可周期性重规划：ComputePathToPose 生成 Path，FollowPath 执行；条件触发时更新 Path。失败后可清理 Costmap、Wait、重试或返回失败。行为由 XML 和 blackboard 决定。

## 8. Nav2 外部定位/感知算法

| 任务 | 模块/算法候选 | 输入 | 输出 |
|---|---|---|---|
| 2D SLAM | slam_toolbox | LaserScan/odom/TF | map、map→odom |
| 地图定位 | AMCL | map、Scan、odom/TF | map→odom |
| 状态融合 | robot_localization EKF/UKF | IMU、odom、GNSS 投影 | state/odom |
| GNSS 投影 | navsat_transform_node | NavSatFix、heading、odom | GPS odometry |
| LiDAR odom | Point-LIO / FAST-LIO2 | 3D LiDAR + IMU | local odom/map |
| Stereo VIO | VIO/视觉 SLAM | 左右图像、IMU | pose/odom/map |
| 目标跟踪 | detector + tracker | RGB/depth/cloud | 类别、轨迹、速度 |
| 地形估计 | elevation/traversability | 3D cloud、姿态、接触 | elevation/cost/risk |

以上多数不是 Nav2 内置必选算法；须先验证时间、TF、协方差和失效模式。

## 9. 选型门

确定平面运动学、环境、地图模式；建立可解释 baseline；单变量对照；统一记录成功率、路径长度、最小障碍间距、跟踪误差、重规划、CPU/GPU、内存和延迟。报告需记录版本、参数、数据、场景和失败样本。

参考：https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/ 、https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/
