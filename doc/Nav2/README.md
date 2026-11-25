# Nav2 子系统研究目录（完整导航模块的一个组成部分）

本目录不是整个导航模块的定义。Nav2 负责 ROS 2 导航框架中的行为编排、规划/控制插件托管、Costmap 和恢复行为；完整导航系统还需要独立研究传感器驱动、时间同步、定位/SLAM、感知、地图/地形/可通行性、四足步态接口、安全、任务管理和验证部署。

完整导航模块总目录见 [doc/Navigation/README.md](../Navigation/README.md)。专项进度唯一真源为根目录 [TaskPlan.md](../../TaskPlan.md)。所有研究仅在 nav 分支进行。

## 本目录现有文档

| 文档 | 主题 |
|---|---|
| [00 总体架构](00_architecture_overview.md) | Nav2 内部职责、服务器和数据对象 |
| [01 生命周期与任务流](01_lifecycle_and_navigation_flow.md) | Bringup、Lifecycle、NavigateToPose |
| [02 数据生产者与消费者](02_dataflow_producer_consumer_matrix.md) | Nav2 相关接口和数据流 |
| [03 感知与 Costmap](03_perception_and_costmaps.md) | 观测层、Costmap layer |
| [04 算法与插件](04_algorithms_and_plugins.md) | Planner/Controller/Smoother/Behavior |
| [05 硬件与软件模块](05_hardware_and_software_modules.md) | Nav2 周边依赖和硬件接口 |
| [06 四足集成与安全](06_quadruped_integration_and_safety.md) | cmd_vel adapter、步态、安全边界 |
| [07 验证、调试与路线](07_validation_debugging_and_roadmap.md) | 分层验证和实施路线 |

## 当前研究基线

ROS 2 Jazzy / Nav2 Jazzy 为源码研究候选基线，Humble 兼容性作为对照。源码级结论前必须固定上游 tag/commit，并记录 package、文件路径、类/函数、调用关系、executor/callback、接口、参数、错误路径和证据日期。

已有内容属于架构研究/设计基线，不代表 Nav2 节点集成、Gazebo 仿真、真实 rosbag 算法回放、Jetson 构建或真实机器狗验收。
