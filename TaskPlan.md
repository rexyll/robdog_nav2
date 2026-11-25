# RobDog 完整导航模块项目总任务计划（唯一进度台账）

- 文档版本：v0.3.112
- 更新日期：2026-11-25
- 默认分支：`main`
- 仓库：https://github.com/rexyll/robdog_nav2
- 项目目标：以四足机器人为载体，研究、设计并最终集成完整的 ROS 2 导航模块，覆盖任务管理、传感器接入与时间同步、定位/状态估计/SLAM、环境感知、地图与地形表达、可通行性、全局/局部规划、动态避障、Nav2（仅为其中一个子系统）、四足运动接口、独立安全、运行时部署、监控和分层验证。
- **本体技术预选：DEEP Robotics Lite3 Venture**（未采购，待供应商确认）。
- **传感器预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）；ArduSimple simpleRTK2B Budget / u-blox ZED-F9P。**
- **机载计算预选：NVIDIA Jetson AGX Orin Developer Kit。**
- 当前专项阶段：**N1 — 端到端数据流与接口审计（仅 nav 分支）**；N0 范围/架构/术语/证据基线已冻结；硬件准备的阶段 A 仍保持原状态并行受 Gate 约束。
- 当前状态：**N0 已完成；N1 已建立系统级数据流清单，并完成 Nav2 主链、Costmap 生产端与 Planner/Controller 消费端的首轮固定源码闭环；已对 MID-360 逐点时间数据、高程地图/四足地形层，以及 Point-LIO/FAST-LIO2 的算法时间补偿、驱动时间戳合同、ROS 2 集成依赖、ESEKF covariance 导出和 robot_localization/GNSS 融合接口开展上游审计；本轮进一步将定位 validity/epoch/TF age 与 Nav2 Costmap、Planner、Controller 的失效传播串成接口矩阵；新增 ZED-F9P UTC→GPS mapper 的分层补丁计划、NAV-TIMELS parser/lifecycle fence 设计、deterministic synthetic test vectors，并补齐 source-policy/精确历元 join/fail-closed 状态机/12 组纯算法 property-style 测试合同（doc/Navigation/14 §4.12，设计未实现）；本轮补充 NAV-PVT 92-byte 与 NAV-TIMEUTC 20-byte 原始 payload 布局、validity/confirmation masks、signed nano 和长度/日历拒绝向量。GNSS 包 metadata 待确认项已完成分支来源核验：`package.xml` 与 `setup.py` 在 `main`、`nav` 及共同基点 `07a740711783c219f3c091a5c8e9f2fecc71cbc3` 内容完全一致，确认为 main 基线遗留而非 nav 新增；不修改许可证/维护者字段，仍须项目治理确认后才能解除实现 Gate。N1 的其余接口/运行证据仍在补齐；LIO 许可冲突、本项目自身 Jazzy/ARM64 构建、Orin 性能与实机验证均未通过/未完成；上游固定 Point-LIO 与 Estimator Plugin 已有 Jazzy AMD64/ARM64 CI 成功证据。**

---

## 0. TaskPlan 管理规则（必须遵守）

1. **根目录 `TaskPlan.md` 是项目唯一进度真源。** 所有任务状态、阶段进度、完成证据、阻塞项、决策变更、测试结果和下一步动作必须写入本文件。
2. 其他文档只保存详细设计、接口说明、ADR、测试报告和稳定参考资料；不得只更新其他文档或对话而不回写本文件。
3. 每次推进必须更新本文件中的：当前阶段、变更任务状态、完成证据、Git commit SHA、测试/验证方式、风险/未决项、下一步动作。
4. 文档/设计完成不等于代码实现；仿真通过不等于真实机器狗通过；只有证据足够才标记任务完成。
5. **导航模块专项分支纪律（用户明确要求）：完整导航模块的研究、文档、代码、测试、CI 和相关决策只在 `nav` 分支进行。专项完成并通过 N9 验收前，不得将专项中间成果合并/复制到 `main`，不得在 `main` 上继续本专项。完成后先汇总差异、审查风险、形成回归证据和合并方案，再回到 `main`；是否合并以专项验收结论和用户既定主线治理规则执行。** 其他非导航专项仍按各自约定执行。
6. 状态标记：`[ ]` 未开始；`[~]` 实施中/部分完成；`[x]` 已完成且有证据；`[!]` 阻塞或风险。
7. 本文件超过约 30 KB 时按主题拆成 `TaskPlan_Part-*.md`。**拆分后仍必须把所有进度状态保留在本根文件**；分册只放详细设计，不放需要维护的重复进度清单。
8. 未知传感器参数、供电电流、质量、时延、速度、协方差和准确度一律标“待确认/待测”，不猜测、不照抄示例参数。

## 1. 项目目标、硬件基线与系统决策

### 1.1 完整导航模块目标
使用四足机器狗 + 3D LiDAR + 两台普通 RGB 相机（被动双目）+ RTK GNSS，结合机器人 IMU、经验证的本体状态估计和必要的外部定位/感知组件，构建从任务目标到安全运动执行反馈的完整导航闭环。Nav2 是候选导航框架/子系统，不是完整导航模块本身。

完整范围至少包括：任务/航点/区域管理；设备驱动、时间同步和标定；腿式 odometry/LIO/VIO/GNSS/SLAM 与融合定位；动态环境感知和目标跟踪；2D/3D/语义/拓扑/高程地图；地形可通行性与四足约束；全局路线、全局路径、局部轨迹和动态避障；Nav2 任务/规划/控制/Costmap/恢复；四足步态和厂商 SDK 适配；命令仲裁、watchdog、Collision Monitor 与独立急停；ROS 2 runtime、Jetson 部署、可观测性、数据记录和分层验收。

导航模块必须同时闭合“感知/定位→环境表达→规划决策”和“命令→安全仲裁→平台执行→真实反馈”两条链路。不得以 Nav2 action 成功、cmd_vel 已发布或仿真通过替代真实执行反馈与安全验收。

### 1.2 技术预选 BOM（未采购、待供应商确认）
- **本体：** DEEP Robotics Lite3 Venture；公开标称载荷 4.5 kg，具备外部 Ethernet 和 24V/12V/5V 扩展供电，但各路持续/峰值电流、安装 CAD、质心限制均待书面确认。
- **LiDAR：** Livox MID-360，3D、Ethernet/PointCloud2，主要几何避障传感器。
- **双目：** 2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1 Color（无镜头），配两只同型号匹配广角 C-mount 镜头和共同硬件 trigger。
- **RTK：** ArduSimple simpleRTK2B Budget / u-blox ZED-F9P + ANN-MB 天线；需要 NTRIP 或自建基站。
- **计算：** NVIDIA Jetson AGX Orin Developer Kit（64GB 开发配置）；另需工业 Ethernet switch、相机触发分配、电源/DC-DC/保险和定制安装板。
- **载荷预算：** 内部目标外设总质量 ≤3.6 kg（不是厂商认证上装值）。已核实设备小计约 1.330 kg；若采用两只 CIL530 镜头，候选小计约 1.664 kg，仍未包含相机本体与全部附件。已知设备端功率估算约 22.8–69.4 W，转换损耗/启动峰值待测；禁止未经厂商确认直接用 Lite3 24V 给 AGX 供电。
- 详细规格/风险/供电图见 [TaskPlan Part 1](TaskPlan_Part-01_ArchitectureAndHardware.md)、[ADR-0002](docs/decisions/ADR-0002-hardware-platform.md) 和 [power/mass budget](docs/implementation/power_mass_budget.md)。

### 1.3 软件版本策略
- 首轮按云深处公开方案复现 Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble。
- Orin 长期目标为 Ubuntu 24.04 + ROS 2 Jazzy；AGX Orin Developer Kit 使用 JetPack 7.2.1 base ISO 后按 NVIDIA 指引升级至 7.2.2。
- ROS 2 Lyrical（Ubuntu 26.04）待 Orin BSP/驱动支持成熟后再评估。
- 自研包保持 Humble/Jazzy 可移植；Lite3 厂商 SDK 与 Nav2 标准消息层隔离，不默认让 Foxy 与 Jazzy DDS 节点共享 domain。

### 1.4 系统分工与定位边界
- **定位：** 厂商可信腿式 odometry 若存在并通过时间/坐标/covariance 验证，优先作为 local odom；否则第一备用评估 Point-LIO + MID-360 内置 IMU。FAST-LIO2 为对照，Stereo VIO/FAST-LIVO2 后续评估。禁止用 IMU-only 积分伪造位置。
- **全局：** RTK 经 navsat_transform 和 quality gate 提供 map 位置约束；单天线 GNSS 静止时不能提供机身 yaw。
- **障碍物：** MID-360 为主几何源；双目深度云为补充观测；RGB 语义负责类别/跟踪，不替代几何测距。两路 observation source 保持独立。
- **Nav2：** Global/Local Costmap、Planner、Controller、BT、Velocity Smoother、Collision Monitor，再经 Lite3 安全 command adapter 和厂商步态控制器。
- **四足限制：** 2D costmap 不能替代落足点规划、坡度/台阶/沟坎/悬崖的 traversability。Collision Monitor 是软件防护，不是安全认证急停。
- **重要 Lite3 Gate：** 新版 sdk_deploy 的 DRDDS /IMU_DATA、/JOINTS_DATA 和 RobotStatus 不等于 nav_msgs/Odometry。正式 legged odometry、motion-host/AGX-host 分工及跨 Foxy/Jazzy 数据路径仍待厂商确认。见 [ADR-0003](docs/decisions/ADR-0003-lite3-ros-integration.md)、[vendor SDK assessment](docs/implementation/lite3_vendor_sdk_integration_assessment.md) 和 [supplier matrix](docs/interfaces/lite3_vendor_confirmation_matrix.md)。

---

## 2. 阶段任务清单（所有状态只在本节维护）

详细技术方案和实施说明见：
- [TaskPlan_Part-01：系统架构与硬件设计](TaskPlan_Part-01_ArchitectureAndHardware.md)
- [TaskPlan_Part-02：测试、风险与参考资料](TaskPlan_Part-02_TestingAndGovernance.md)
- [ADR-0002：硬件平台选型](docs/decisions/ADR-0002-hardware-platform.md)
- [ADR-0003：Lite3 ROS 集成路径](docs/decisions/ADR-0003-lite3-ros-integration.md)
- [ADR-0004：本体 odometry 缺失时的 LIO/VIO 备用方案](docs/decisions/ADR-0004-local-odometry-fallback.md)
- [Odometry fallback strategy](docs/implementation/odometry_fallback_strategy.md)
- [Lite3 厂商 SDK 集成评估](docs/implementation/lite3_vendor_sdk_integration_assessment.md)
- [硬件接口确认清单](docs/interfaces/hardware_interface_checklist.md)
- [Lite3 Venture 供应商技术确认矩阵](docs/interfaces/lite3_vendor_confirmation_matrix.md)

### 导航模块专项深研计划 N0–N9（仅在 nav 分支执行）

> 本专项覆盖完整导航模块；Nav2 对应 N6，不再作为全部研究对象。详细目录和专题交付物见 [doc/Navigation/README.md](doc/Navigation/README.md)。

- [x] N0 研究基线与范围冻结：已创建 nav 分支；研究目标已纠正为完整导航模块；已建立总体目录、系统架构与数据流框架、术语基线、证据等级和首轮上游仓库/commit 锚点。完成证据见 doc/Navigation/README.md、00_system_architecture_and_scope.md、01_end_to_end_dataflow_and_interfaces.md、02_subsystem_inventory_and_evidence_register.md。此状态仅表示研究范围/治理基线冻结，不代表各子系统源码研究完成。
- [~] N1 端到端数据流与接口审计：已建立系统级 producer/consumer 清单与 ROS 2 Jazzy 消息/action 契约；完成 Nav2 bringup→BT→Planner/Controller→Smoother→Collision Monitor 主链源码审计。Costmap 输入链详见 `doc/Navigation/05_sensor_to_costmap_observation_audit.md`；Costmap current 到 Planner/Controller 的等待/超时消费闭环详见 `doc/Navigation/06_costmap_health_and_consumers_audit.md`（Planner 默认 1.0s，Controller 默认 0.30s；超时路径已源码核验）。本轮增加 MID-360→deskew→elevation/grid_map→traversability 的跨子系统接口审计，详见 `doc/Navigation/07_mid360_elevation_traversability_audit.md`：固定 Livox Driver 2、elevation_mapping_cupy ROS2 branch、grid_map Jazzy 与传统 traversability_estimation 参考版本；识别出 elevation_mapping_cupy 默认 XYZ 输入会丢弃 Livox per-point timestamp/tag/line，且整帧按 header stamp 查询一次 TF，因此必须先完成逐点时间补偿再输入；Jetson Orin + MID-360 FPS 下降 issue #137 仍 open，v2.2.0 RTX 4090 性能数据不能外推到 Orin。仍未完成项目实际 adapter、驱动/SDK/ARM64 构建、运行 QoS/频率/延迟、LIO/deskew、Lite3 执行反馈与实机验证；当前证据为 E1/E2 上游资料/源码，不是项目运行证据。 本轮新增 `doc/Navigation/08_point_lio_fast_lio2_audit.md`：固定审计 CTU Point-LIO ROS2 fork 与 HKU FAST-LIO2 官方实现。Point-LIO ROS2 fork 已接入 Livox CustomMsg/逐点 offset_time，并按点时间推进状态、将点投影到 world；但 body cloud 不是 deskewed，world cloud 输出丢失逐点时间；`time_lag_imu_to_lidar` 疑似仅用于回退检查而未作用于同步消费端，且 parser 未显式读取 CustomMsg.timebase。FAST-LIO2 官方代码有 IMU 前向传播+反向 scan-end deskew，但官方仓库仍为 ROS1/catkin，不能直接用于 Jazzy。两者 LICENSE 文件均 GPL-2.0、package.xml 均写 BSD，许可证声明冲突，已列为冻结依赖前阻断 Gate。CTU fork 还依赖 MRS libs，仓库 CI 未证明 Jazzy/ARM64。仍未完成项目 adapter、构建、bag A/B、Orin soak、Lite3 执行反馈与实机验证；当前证据为 E1/E2 上游资料/源码，不是项目运行证据。 本轮新增 `doc/Navigation/09_mid360_timestamp_and_point_lio_integration_audit.md`：固定 Livox Driver 2 commit `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，确认 ROS2 Driver 生成 CustomMsg 时 `header.stamp` 与 `timebase` 均来自 `pkg.base_time`，每点 `offset_time` 是相对首点的纳秒偏移，因此 `header.stamp + offset_time` 可重建点时间；PointXYZRTLT PointCloud2 的 timestamp 则是每点绝对时间，格式间不可混用。进一步定位 Point-LIO `time_lag_imu_to_lidar`：修正时间用于 last_timestamp_imu gate，但队列保存原始 IMU 消息，sync/integration 仍消费原始 header stamp，形成 gate 与积分时间轴不一致。MRS Point-LIO Core `ef9de2a55b43e492aec6e20bcab113973b9390a2` 是 ament metapackage，但 gitman 对 Point-LIO、Estimator Plugin、Livox Driver 使用浮动 ros2 branch；workflow 委托外部 reusable workflow；已追踪到 `ctumrs/ros_jazzy:latest` 容器中的 AMD64/ARM64 jobs，Point-LIO 固定 commit `e7ff034...` 与 Estimator Plugin `b43af573...` 的两架构 Actions build 均成功，MRS core 较早 commit `0f3e329...` 的两架构 build 也成功；当前 core HEAD `ef9de2a...` 尚无同 SHA CI 证据，ARM64 CI 不等于 Jetson Orin 硬件验证。明确 Driver `xfer_format=1` 才对应 Point-LIO CustomMsg 输入。仍未取得实际 MID-360/IMU bag，未运行时间同步单测、Jazzy/ARM64 构建、Orin 测试或实机验证；当前为 E1/E2 上游证据。 本轮追加 CI 核验：Point-LIO run `24843451862`、Estimator Plugin run `21216131865` 均在 AMD64/ARM64 jobs 成功；MRS core run `33400579110` 的 AMD64/ARM64 成功对应较早 SHA `0f3e329...`，不是当前 core HEAD。已将“Jazzy/ARM64 matrix 未证实”修正为“上游有明确 Jazzy AMD64/ARM64 成功证据，但当前 core HEAD 和 Orin 实机仍待验证”。详见 09 §6.1。
- [~] N1 本轮追加 `doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md`：固定审计 Point-LIO `e7ff034...`、MRS Estimator Plugin `b43af573...`、MRS UAV State Estimators `1ccd728...`、MRS UAV Managers `75d897...` 与 robot_localization Jazzy `fea7fc3...`。源码显示 Point-LIO 算法核心只通过 `mrs_lib` 使用 Node/ParamLoader/pub-sub/ThreadTimer/TF wrappers，原则上可用原生 rclcpp/tf2_ros 去 MRS 化；MRS estimator plugin 是 UAV StateGeneric plugin，订阅 MRS Hardware API 姿态/角速度并加载 lateral/altitude/heading 子估计器，未见其把 Point-LIO Odometry 接入状态输出，因此不是四足项目必需依赖。Point-LIO `publish_odometry()` 未写 pose/twist covariance；进一步确认默认 `use_imu_as_input=true` 时内部使用 24 维 `kf_input`，IKFoM 提供 `get_P()` 但 publisher 未导出，不能误判为算法没有 covariance；状态字段顺序显示位置/姿态在前 6 个误差维度、速度候选索引为 12–14，精确 SO(3) tangent/误差 reset 映射待单测。默认 IMU-input 分支 `set_odomtwist()` 不赋值 angular twist；输出 frame 为 `uav_name/point_lio_world` 与 `uav_name/point_lio_body`，TF 默认关闭，child frame 不能直接视为 Lite3 `base_link`。robot_localization Jazzy EKF 对小于 `1e-9` 的 measurement covariance 对角元素作下限处理，因此全零 covariance 可能造成过度信任；`sensor_timeout` 不是传感器健康/RTK质量门控，lagged data 可通过 `smooth_lagged_data/history_length` 回滚重放。navsat_transform 仅排除 `STATUS_NO_FIX` 与 NaN，直接复制 NavSatFix covariance，不能区分接收机 RTK FIX/FLOAT 或 covariance_type unknown；须由现有 `robodog_gnss_quality` 核心与后续 ROS adapter 在 navsat_transform 前执行质量/协方差 Gate。双 EKF `odom`/`map` 分工保留为候选，必须避免同一 MID-360 IMU 被 LIO 与 EKF 当作独立信息重复融合，并确保 TF 唯一权威。以上仅 E1/E2 上游源码证据，未在本项目构建、bag 回放或硬件运行。
- [x] N1 本轮继续补齐定位健康与时间异常接口审计（追加至 `doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §12–15）：固定 CTU Point-LIO `e7ff034...`，确认 `is_initialized_` 只是一次性初始化标志，没有 tracking health/validity 状态机、持续 freshness/退化诊断或失效协方差门控；LiDAR/IMU 时间回退回调只记录并 return，未实际清空队列；`flg_reset` 在固定源码中仅初始化/读取、未发现置位点，IMU reset 仅重置 IMU 初始化统计，不重置 ESEKF/map/TF/时间 epoch。固定 robot_localization Jazzy `fea7fc3...`，确认每 topic 旧时间戳被拒绝、重复 stamp 可进入队列，跨 topic lag 才走 history revert/replay；history 不足时 `revertTo()` 返回 false 后处理循环仍继续，`sensor_timeout` 只触发预测而不使输出失效；`reset_on_time_jump` 仅声明且 callback 分支被注释，不能依赖其自动处理时钟回退。核对现有 `robodog_gnss_quality`，确认已有 fail-closed gate 与四路 UBX 精确 iTOW bounded joiner，但仍无 ROS adapter/time mapper/diagnostics/NavSatFix publisher；后续在现有 core 上补 adapter，不重复造核心。仅 E1/E2 固定上游源码与本仓库代码静态证据，未做本项目新增构建、bag 或硬件运行。
- [~] N1 本轮新增定位有效性到 Nav2 消费端故障传播审计（`doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §19）：固定 Navigation2 Jazzy `c704655...`，确认 `Costmap2DROS::updateMap()` 在 `getRobotPose()` 失败时仅跳过本周期地图更新，并不显式将 `LayeredCostmap::isCurrent()` 置 false；因此 Costmap plugin current 与定位 TF validity 是两个独立 Gate。Planner 在默认 TF 起点获取失败时以 TF_ERROR 终止规划；Controller 获取 pose 失败时以 TF_ERROR 发布零速度并终止 FollowPath，但零速度不等于 Lite3 已停车。Nav2 原生接口不消费 Point-LIO health、EKF measurement age/covariance、GNSS generation 或 estimator epoch，需项目级 navigation health supervisor/reset coordinator 显式管理。新增故障传播矩阵及 INITIALIZING/RESETTING、NOMINAL、DEGRADED、POSE_INVALID/MAP_INVALID、STOP_REQUESTED、RECOVERY 状态合同；不预设时间阈值。`robodog_gnss_quality` 的 `UNSPECIFIED` license 与示例 maintainer 邮箱已通过 `main`/`nav`/共同基点文件 Blob 对比确认为 main 基线遗留（非 nav 分支引入）；仍须用户/项目治理确认后才可 ROS 化发布，本专项不擅自修改。仅固定上游源码 E2 与本项目静态核验，无新增构建、bag、仿真、HIL 或实机验证。\n- [~] N1 本轮继续审计现有 `robodog_gnss_quality` 核心及测试（`doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §20）：确认 `RtkQualityGate` 已实现 fail-closed RTK Fixed、host monotonic freshness、source epoch 单调性、坐标/covariance/水平垂直不确定度/3D jump gate 和恢复滞回；`UbxFourMessageEpochJoiner` 已实现四路 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS 精确 iTOW 配对、有界缓存、receive-skew/stale/late/conflict/duplicate/显式 generation reset，且 NED covariance→ENU 的 Down cross-term 变号已实现。静态枚举已有 30 个 quality gate 测试与 17 个 joiner 测试；本轮未重新运行。仍缺 GPS-week/receiver-time mapper、joiner+gate+global EKF 的原子 reset coordinator、ROS diagnostics/quality status、NavSatFix 与 ACCEPTED epoch 原子关联、组合级 property/fuzz 与真实接收机时钟测试。未改包 metadata；`UNSPECIFIED` license 和占位 maintainer 仍需确认。仅本项目源码/测试静态证据，无新增测试运行或设备验证。\n- [~] N1 新增跨组件 Epoch/Reset Coordinator 设计合同（`doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §21）：明确区分 `estimator_epoch`、`local_odom_epoch`、`gnss_receiver_generation`、`map_epoch` 和 `reset_transaction_id`，定义各组件唯一 reset owner、GNSS-only/LIO/local EKF/global datum/ROS clock rewind 等触发事件的最小影响范围，以及 DETECTED→QUIESCING→INVALIDATING→DRAINING→RESETTING_COMPONENTS→REBASING→WARMING_UP→COMMITTING→COMMITTED/ABORTED_FAIL_CLOSED 事务流程。定义跨节点 generation fencing、旧数据拒绝、逻辑可见性原子提交、独立 local/global validity 与纯算法测试矩阵。明确当前为 E3 项目设计合同，非实现；未增加 ROS 依赖、未修改算法核心或 metadata，未进行 build/bag/HIL/实机验证。\n- [~] N1 新增逐事件 Epoch/Reset Transition Table 与 GNSS reset API 对照（`doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §22）：源码确认 `RtkQualityGate.reset()` 清除 recovery counter 与 jump reference；`UbxFourMessageEpochJoiner.reset()` 清除 pending/retired/epoch ordering 并要求显式 `begin_generation()`，但保留 lifetime eviction/rejection counters；`begin_generation()` 不验证 GPS week/reboot 证据。发现 Gate 可在 source_clock_id 改变时自行 reset，而 Joiner 对 clock ID 改变会拒绝并要求显式 generation，因此 coordinator/adapter 必须以 Joiner generation fence 为唯一入口，避免两组件 generation 分歧。新增 FIX→FLOAT、receiver reboot、week rollover、clock ambiguous、LIO lost/reset、local EKF reset、global datum change、ROS clock rewind、Costmap stale 的 precondition/epoch/reset order/ACK/failure 表；定义 GNSS 子事务 barrier→停 callback→joiner.reset→gate.reset→time mapper 重建→显式 begin_generation→exactly-once gate→NavSatFix/quality status 同 epoch 发布。仅静态源码核验和 E3 设计，无测试重跑/ROS 实现/硬件验证。\n- [~] N1 新增 GNSS core 工程化前置审计（`doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` §23）：确认现有包为 ament_python、默认仅 setuptools runtime、测试使用 unittest.TestCase 并由 setup.cfg 配置 pytest 收集；未发现 pyproject.toml/requirements.txt，Python 最低版本矩阵未声明。package.xml/setup.py 均为 `UNSPECIFIED` license，maintainer email 为 `maintainers@example.invalid`。现有结构可容纳 transport-independent coordinator，但正式实现/发布前需确认 SPDX license 与维护者元数据；本轮不擅自更改，不引入 ROS runtime dependencies，不创建代码。仅静态文件审计，无 colcon/pytest/unittest 运行。\n- [~] N1 为现有 GNSS core 补充 reset 语义回归测试（`test/test_quality_gate.py`、`test/test_ubx_epoch_joiner.py`）：新增 Gate 显式 reset 后恢复计数归零、旧 source timestamp/远距离位置按新 epoch 首样本处理；新增 Joiner reset 清空 pending/retired 但保留 lifetime eviction counter、显式 begin 新 generation 后拒绝旧 generation callback 且不污染新缓存。现有 Joiner reset 强制 begin_generation 的测试继续保留。本轮仅提交测试代码，尚未运行 pytest/colcon；需由 GNSS quality CI 的 Humble/Jazzy jobs 验证。\n- [ ] N2 传感器驱动、时钟同步与标定：MID-360、双目、IMU、ZED-F9P、Lite3 遥测；驱动源码/协议、PTP/PPS/硬件触发、时钟域转换、外参/内参、数据记录与健康监测。
- [ ] N3 定位、状态估计、SLAM 与地图：腿式 odom、Point-LIO/FAST-LIO2、VIO、GNSS quality gate、robot_localization、SLAM、重定位、地图坐标注册和不确定度管理。
- [ ] N4 环境感知、地形表达与可通行性：点云/深度预处理、地面/障碍/动态目标/负障碍、elevation/grid_map、坡度/粗糙度/台阶/沟坎/落足风险。
- [ ] N5 任务管理、规划与动态避障：任务状态机/BT、拓扑路线、全局规划、局部轨迹优化、动态目标预测、恢复行为、四足运动学/步态约束。
- [ ] N6 Nav2 子系统源码级深研：固定 Jazzy tag/commit；追踪 NavigateToPose Action→BT Navigator→Blackboard/BT tick→Planner Server→Controller Server→速度输出→反馈/结果；研究 Costmap、Lifecycle、插件、Behavior、Smoother、Collision Monitor 和 Nav2 外围接口。
- [ ] N7 四足运动执行、安全和 ROS 2 运行时：Lite3 命令语义/能力边界、adapter、SDK 控制权、反馈、命令仲裁、watchdog、物理急停、DDS/QoS/Executor/Lifecycle、Jetson 资源和部署。
- [ ] N8 系统验证与架构收敛：单元/集成/仿真/真实 bag 回放/硬件只读台架/HIL/封闭场地分层；算法对照、量化指标、故障注入、ADR、部署拓扑与完整集成方案。
- [ ] N9 专项验收与回归：文档覆盖完整、源码/接口证据可追溯、风险和未决项闭环或显式保留、CI/测试可复现；输出 nav→main 差异审查和合并建议。N9 完成前不得将本专项合并回 main。

**N0 首轮已完成的文件基线：**
- [总体导航模块目录](doc/Navigation/README.md)
- [完整导航模块架构与边界](doc/Navigation/00_system_architecture_and_scope.md)
- [端到端数据流与接口审计框架](doc/Navigation/01_end_to_end_dataflow_and_interfaces.md)
- [Nav2 子系统目录](doc/Nav2/README.md)（原有 8 篇 Nav2 文档继续保留）

**N0 尚未完成：** 上游仓库/tag/commit 固定、术语表、组件/接口证据矩阵、源码级调用链、ROS 集成和运行验证。当前文档是研究框架，不得标记为系统深研完成。

### 阶段 A — 硬件基线与接口冻结（既有硬件 Gate；当前导航专项阶段见 N1）

- [x] A1. 完成本体和外置导航传感器技术预选：Lite3 Venture、MID-360、双 See3CAM_50CUG_CXXCC_BX_H01R1、simpleRTK2B/ZED-F9P、AGX Orin。
- [x] A2. 核对云深处 Lite3 Venture + AGX Orin + MID-360 官方扩展示例与初始软件版本。
- [ ] A3. 向云深处确认 Venture 当前 SKU、交付件、固件/SDK、安装 CAD、外设接口持续/峰值电流、载荷与重心限制。
- [~] A4. 已建立初步供电/质量预算：已核实器件 1.330 kg；若采用 CIL530 双镜头，候选小计约 1.664 kg，余 1.936 kg 供相机本体和附件。两台相机本体、交换机、触发板、支架/线束、DC/DC 尚未称重/测功；Lite3 各路电源额定仍待书面确认。
- [~] A5. 已核对 MID-360 公开规格，并锁定驱动候选 livox_ros_driver2 v1.2.6 commit 13eb05e4e6dd7a765b934d0c5fd6236676a57b49、Livox-SDK2 release v1.3.1 commit f5d9375f84efe2b15bc0a052d3e18482ed13adf4（GCC 13 使用 -include cstdint）；实际设备 revision、SDK/driver ARM64 构建、线束与网络参数仍待确认。
- [~] A6. 已锁定 See3CAM 彩色无镜头 SKU；新增 6mm C-Mount 镜头候选 CIL530、120mm 双目基线和 5MP@20fps 初始测试配置。镜头暗角/MTF、GPIO 电气时序、双机曝光同步与 Jetson USB 带宽待样品验证。
- [~] A7. 已核对 simpleRTK2B Budget / ZED-F9P 当前 SKU 与最高 10Hz RTK 输出；板卡不含天线，目标场地及 NTRIP/自建基站尚未确定。
- [~] A8. 已审计 current sdk_deploy 公共 RobotStatus：包含 base_rpy/base_angular_velocity、joint/status/timeout/freshness，但不含 base xyz/linear velocity/covariance；不能据此发布 Odometry。Motion SDK 可读 IMU/12 joints/contact force；lite3_transfer 启动可能获取 SDK 控制权，实机 bring-up 前必须有厂商 SOP。见 [公开 SDK 字段与安全审计](docs/implementation/lite3_public_sdk_field_audit.md)。
- [~] A9. 已形成候选 Topic/TF 与命令安全链设计；真实 frame_id、时间同步误差、运动学和 USB/Ethernet/电源布局待硬件资料确认后冻结。
- [~] A10. 已建立定位、感知、端到端时延、停车距离、Nav2 路径跟踪、RTK 降级、地形和长时运行的指标定义与测量方法；最终数值阈值待 Lite3 实际速度/制动能力、部署场地和风险等级确定。见 [验收指标框架](docs/testing/acceptance_metrics_and_scenarios.md)。
- [~] A11. 已核实 ROS 发行版与 Jetson 组合：首轮复现厂商 Humble 栈，同时评估 AGX Orin DevKit JetPack 7.2.1 ISO base → 7.2.2 apt upgrade / Ubuntu 24.04 / Jazzy；ROS Lyrical（Ubuntu 26.04）暂不纳入当前 Orin 正式基线。完成实际兼容性矩阵后冻结版本；采购前风险评审作为后续 A12。
- [ ] A12. 完成采购前风险评审；无供电/载荷/接口重大风险后批准采购与进入阶段 B。
- [ ] A13. 向云深处确认当前 Venture 固件/SDK 的推荐 ROS 路径、motion host / AGX Orin host 分工、正式 legged odometry/body velocity（含时间戳/frame/covariance）、contact force 语义、SDK_MODE 是否获取控制权及物理急停/驱动禁能方案；问题扩展至供应商矩阵 V-32。

**阶段 A 准入：** 厂商接口、电源、质量、安装和软件兼容均有资料或测量支持；形成 BOM、CAD、网络/电源拓扑和验收阈值。

### 阶段 B — 驱动、时间同步和标定

- [x] B0. 已建立 robodog_lite3_bridge ROS 2 Python 包骨架，包含隔离在 /mock/lite3 namespace 和 mock_* frames 的合成状态发布、只读 topic 新鲜度诊断和 pytest 单元测试；GitHub Actions 在 Humble/Jammy 与 Jazzy/Noble 均成功构建，每个发行版 6/6 测试通过。未接入真实 Lite3 SDK，不发布运动命令。
- [ ] B1. 建立厂商兼容性矩阵：Legacy Lite3_ROS Foxy、Current sdk_deploy/Foxy motion-host、AGX Orin JetPack 6.1/Humble 参考环境，以及 JetPack 7.2.2/Ubuntu 24.04/Jazzy 项目环境；禁止默认使用跨发行版 DDS。
- [ ] B2. 依据 A13 厂商答复选择 SDK 路线；先只读读取厂商 IMU/joint/status 和所有可用 odometry；禁止启动运动状态或发布关节命令。
- [~] B3. 已实现 transport-agnostic envelope/sequence 基础校验，并新增只读 contact_state 消息类型；IMU/contact_state/base_odometry 等空间消息必须带 frame_id；ROS 3x3/6x6 covariance 已增加矩阵对称性与 PSD 结构校验，新增 5 项边界测试，Humble/Jammy 与 Jazzy/Noble CI run 45377526595 均通过。仍未实现编码器、UDP/TCP socket、vendor mapping 或 real gateway。
- [x] B4. 已在 Ubuntu 24.04 + ROS 2 Jazzy x86_64 完成 Livox SDK2 v1.3.1 与 livox_ros_driver2 v1.2.6 的构建/安装；CI run 45337301420 成功。仅为 x86_64 编译证据，Orin aarch64、真实 MID-360 设备发现/数据采集、PTP 和 60 分钟稳定性仍未验收。CustomMsg/timebase/offset_time ns 语义已源码核验。
- [ ] B5. 接入左右 UVC 相机和 GPIO 硬件触发，验证双路 USB3 带宽。
- [~] B6. 已完成 ZED-F9P ROS 2 驱动候选审查并固定 `aussierobots/ublox_dgnss` 0.7.6 commit `9d4f5addbc72d092c57dde99b20712258e96e81e`（Apache-2.0）；`rtcm_msgs` 1.1.6 源码依赖固定 commit `8d27ab5f4d4a581e1bb9c87d572994c25197f6d4`（BSD-3-Clause）。Humble/Jammy 和 Jazzy/Noble 两套 CI [run 45387577856](https://github.com/rexyll/robdog_nav2/actions/runs/45387577856) 均完整 success：7 个包 colcon build/install 完成，UBXNavPVT/HPPosLLH/COV 所需 iTOW 和质量字段检查通过。评估文档记录 header stamp=主机 USB 收包时间、不可直接信任上游 NavSatFix converter 的跨 topic 状态/协方差匹配。**真实 ZED-F9P USB/固件/UBX输出、NTRIP/RTK Fixed/PPS 和 qualified NavSatFix ROS adapter 尚未接入，B6 仍为部分完成。**
- [ ] B7. 建立 URDF/TF，完成相机内外参、双目基线、LiDAR/GNSS 外参。
- [ ] B8. 测量 PTP/GPS/相机触发/UDP 的同步误差、时延和抖动。
- [ ] B9. 录制完整 rosbag2，建立回放和 diagnostics。
- [ ] B10. 所有 topic、frame_id、QoS、频率、数据年龄和 TF 检查通过。
- [~] B11. Point-LIO P0/P1 与 P3a ARM64 build 已通过；UrbanLoco `HK-Data20190117_0.db3` 仅完成 rosbag2/topic/318 条 INSPVAX CDR 诊断，来源不明，不能代替指定 ROS1 原始 bag。UrbanLoco P2 replay/evo 仍被 provenance gate 阻塞（CI [45394849538](https://github.com/rexyll/robdog_nav2/actions/runs/45394849538) success），但不再作为整个项目唯一数据入口；并行替代基准转入 B13。
- [~] B12. MID-360 rosbag 时间审计 CLI 已完成：读取 CustomMsg.timebase/header.stamp/逐点 offset_time 与 IMU header.stamp，统计频率、非单调时间、offset 分布和 LiDAR-IMU 最近样本差；CLI 阈值现在拒绝负数、NaN 与 ±Inf，并在打开 rosbag 前失败退出，0 与未指定阈值保持合法；新增 8 项参数/helper 和 CLI 前置校验测试，Python compile、CLI --help、21/21 项测试 CI [run 45384314282](https://github.com/rexyll/robdog_nav2/actions/runs/45384314282) success。代码提交：b2e44f7d5eb23f886f569e49a506fee22f742522、e3403fd8746c808e4b956f8da0b64256b242af62、afe8207d3f0310326e0736d1cec17e9e46b0072f、4b2e089f7206a639b8dadf7f150200d58bff1a6c。真实 MID-360 rosbag 尚未采集/审计，最近邻消息差不是 PTP 同步精度证明。
- [~] B13. Dynamic03 原始 bag 官方 SHA256、结构 audit、真实 ROS1→ROS2 转换和 ros2 bag info 均通过；输出 59,749 条 CDR 消息，LiDAR 2,846、IMU 56,903，类型/计数正确。原始 GT 4,269 行有 1,675 duplicate + 728 backwards timestamp；pose-run candidate 2,594 行虽通过 TUM parser，但非权威。新线索：原始 GT 行数恰与 /mavros/global_position/raw/fix 4,269 条消息相同；M3DGR Dynamic 场景 GT 类型为 RTK/Mocap、平台 RTK 标称 15Hz，但逐行对应和 RTK/Mocap 类型未证实。来源关联审计待本地执行，evo 阻塞。详情见 [替代数据集策略 §6](docs/datasets/alternative_dataset_strategy.md)。
- [~] B14. Dynamic03 真实 bag 已完成只读审计、ROS1→ROS2 Livox 转换和 ros2 bag info；未执行 Point-LIO replay。GT/raw-fix 4,269 条逐行时间差全部 <1ms，诊断 SE(3) RMSE 0.044mm，强烈支持同源但非独立精度验证。raw-fix status=0 仅普通 STATUS_FIX，covariance_type=1 approximate；receiver_lla 无有效 covariance 元数据。receiver_pvt 全量 2,845 条：valid_fix 仅865（30.4%），diff_soln=false 及 carr_soln=0 均为 100%；h_acc 中位31.381m/P95 224.992m，v_acc 中位32.408m/P95 327.376m，排除其作为 RTK Fixed 真值。新增 raw GNSS topic/观测审计（commit 08051e3），文档记录质量结论（commit 1d1ae96）；CI 尚待验证。下一步审查 range_meas 载波相位/LLI/CN0、ephemeris/PPS/RTCM 及 raw-fix MAVLink 物理来源。GT Gate 仍阻塞，不运行/宣称 APE/RPE 精度验收通过。

### 阶段 C — 状态估计与 RTK 全局定位

- [ ] C1. 核实当前 sdk_deploy 是否提供可信 legged odometry/body velocity；若只提供 IMU/joint/status，启动 Point-LIO + MID-360 内置 IMU 备用 POC；不得用 IMU-only 积分冒充可信位置。
- [ ] C2. 标定 MID-360 LiDAR/内置 IMU 外参、设备时间戳/PTP 和每点 offset_time；验证 Point-LIO 所需点云与 IMU 同步。
- [ ] C3. 比较 vendor legged odom（若有）与 Point-LIO 的位置/航向误差、漂移、更新频率、延迟和资源开销；选定唯一主要 local odom 来源。
- [ ] C4. 将通过验收的 local odom 表达为 `odom→base_link`；仅在有额外独立观测且 covariance 合理时再配置 local EKF，避免重复融合同一 IMU/LIO 信息。
- [ ] C5. 配置 navsat_transform（ENU/UTM、datum、yaw offset、磁偏角、GNSS 天线杆臂）。
- [~] C6. 已实现 `robodog_gnss_quality` RTK Fixed 质量门控、NAV-PVT flags 解码、四路原始 UBX 精确历元组装函数 `build_observation_from_ubx_epochs()` 和 `UbxFourMessageEpochJoiner` 有界缓存器。严格要求 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS 的 iTOW 完全相等、四路 host monotonic receive skew 在配置范围、PVT/STATUS 质量字段一致、HPPOSLLH 无 invalid flags、NAV-COV `pos_cor_valid=true`；完整 NED covariance 转 ENU（包括 Down 轴相关交叉项符号）。缓存器按 iTOW + 单一 source_clock_id 汇合，提供 LRU 容量、pending-age 淘汰、receive-watermark/最大乱序窗口、重复冲突检测和 bounded retired tombstone，禁止被淘汰/过期/迟到历元重新拼接；receiver generation 变化需显式 begin_generation，reset 后不自动接受新时钟域。最新 GNSS CI [run 45389178931](https://github.com/rexyll/robdog_nav2/actions/runs/45389178931) 在 Humble/Jammy、Jazzy/Noble 各 73/73 passed。u-blox 驱动跨发行版 7 包 build/install/接口验证 [run 45388253243](https://github.com/rexyll/robdog_nav2/actions/runs/45388253243) success。**仍缺 ROS subscription adapter 将实际 ROS 消息映射进四类 record、receiver GPS week/boot-generation 检测与 source-time mapper、实际接收机数据、qualified NavSatFix 发布、navsat_transform/global EKF 接线；C6 保持部分完成。**
- [ ] C7. 测试 Fixed/Float/Single、遮挡、多路径、断改正和恢复。
- [ ] C8. 测试静止、直行、转弯、横移（若支持）、步态切换和机身振动。
- [ ] C9. 完成 GNSS 降级、超时、跳变剔除和恢复策略。

### 阶段 D — LiDAR/双目障碍物感知

- [ ] D1. 完成 MID-360 点云 deskew、自体过滤、地面分割和障碍物提取。
- [ ] D2. 完成双目硬同步、rectification、disparity、depth 和 PointCloud2。
- [ ] D3. 完成深度置信度/无效视差/范围/高度过滤。
- [ ] D4. 关联 RGB 语义检测与三维几何，形成动态目标跟踪。
- [ ] D5. 绘制 LiDAR 与相机盲区覆盖图，确定互补范围。
- [ ] D6. 测试墙/箱/人/细杆/玻璃/反光/无纹理/弱光/逆光/运动模糊。
- [ ] D7. 输出深度误差、检测召回率、首次发现距离和端到端延迟报告。

### 阶段 E — Nav2 导航框架、平地导航和实时避障（完整导航模块中的一个子系统）

- [ ] E1. 明确室内 SLAM 地图与室外 RTK 地图/航点模式。
- [ ] E2. 建立 global costmap 与 local rolling costmap。
- [ ] E3. 将 MID-360 与 stereo cloud 作为独立 observation source。
- [ ] E4. 配置 obstacle/voxel、footprint、inflation、clearing、range 和 timeout。
- [ ] E5. 实测选择 Planner/Controller/BT，匹配 Lite3 实际步态与运动能力。
- [ ] E6. 串接 velocity smoother、Collision Monitor、Lite3 adapter 和 watchdog。
- [ ] E7. 台架/静止验证命令链后，在封闭区域进行受控低速测试。
- [ ] E8. 完成突然出现障碍、动态行人、窄通道、转弯、倒退（若支持）和传感器断流测试。
- [ ] E9. 完成停车距离、响应时延、误停/漏停和异常停止回归。

### 阶段 F — 四足地形感知和鲁棒性

- [ ] F1. 建立 3D 地面高程/坡度/粗糙度表示。
- [ ] F2. 检测台阶、路沿、沟槽、负障碍和悬空障碍。
- [ ] F3. 实现 traversability layer 并与 Nav2/底层落足规划协作。
- [ ] F4. 加入动态行人预测和速度约束。
- [ ] F5. 故障注入：RTK 丢失、LiDAR 停止、双目退化、TF 断链、计算机重启、控制器断线。
- [ ] F6. 测试振动、热、续航、线缆和长时间稳定性。

### 阶段 G — Jazzy 迁移、端到端验收与部署

- [ ] G1. 自研包在 Ubuntu 24.04 + JetPack 7.x + ROS 2 Jazzy 构建，并保留厂商 Humble bridge 的兼容性验证。
- [ ] G2. 分别验证 Lite3 bridge、Livox、camera、robot_localization、Nav2 Jazzy 兼容性。
- [ ] G3. 不兼容厂商组件以 UDP/标准消息 bridge 隔离，不默认混用不同 distro DDS。
- [ ] G4. 在 Humble 2027-05 EOL 前退出长期运行基线；持续评估 ROS 2 Lyrical（Ubuntu 26.04 / May 2031 LTS），待 Jetson Orin 官方 BSP 和整套驱动可用时再启动迁移。
- [ ] G5. 完成仿真、rosbag、HIL、封闭场地和受控户外验收。
- [ ] G6. 输出安装、标定、启动、监控、故障恢复和维护手册。
- [ ] G7. 冻结软件/固件/驱动/地图/模型/标定版本并发布可复现版本。

---

## 3. 当前进度（由每轮执行持续更新）

### 已完成
- [x] 新增 u-blox 四路原始消息有界历元缓存 `UbxFourMessageEpochJoiner`：按 iTOW 聚合 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS；加入 host monotonic receive watermark、显式 expire-stale、容量/时效淘汰和 bounded retired tombstone，防止 incomplete/late/conflicting epochs 复活；receiver generation 变化需显式 begin_generation。Humble/Jammy、Jazzy/Noble 各 73/73 passed（run 45389178931）。模块：`src/robodog_gnss_quality/robodog_gnss_quality/ubx_epoch_joiner.py`；测试：`src/robodog_gnss_quality/test/test_ubx_epoch_joiner.py`。此为纯算法缓存核心，不含 ROS subscriptions、GPS week mapper 或实际设备接入。
- [x] 完成 ZED-F9P ROS 2 驱动候选评估与来源固定：`aussierobots/ublox_dgnss` 0.7.6 (`9d4f5add...`, Apache-2.0)；`rtcm_msgs` 1.1.6 (`8d27ab5f...`, BSD-3-Clause)。静态审查记录 header stamp=主机 USB 收包时间、上游 NavSatFix converter 未按 iTOW 配对、NAV-COV 为 NED covariance；严格 join/时钟 epoch/NED→ENU 适配边界见 `docs/implementation/zed_f9p_driver_evaluation.md`。跨 Humble/Jazzy 的 7 个包 build/install 均已成功（run 45386892781），但 CLI message-field check 因 BrokenPipeError 失败；修复后 [run 45387577856](https://github.com/rexyll/robdog_nav2/actions/runs/45387577856) 仍在执行，暂不记录为完整 CI pass。
- [x] 仓库初始化，默认分支 main。
- [x] 补充多传感器时间同步方案：区分双目共触发、LiDAR PTP/GPS、Lite3 SDK 时钟、GNSS PPS 和 host monotonic freshness。
- [x] 建立 robodog_lite3_bridge 只读软件骨架：隔离 mock odom/IMU/joint topics、DiagnosticArray topic freshness monitor、单元测试和双发行版 CI workflow 文件；Humble 与 Jazzy 构建和各 6 项测试均通过。
- [x] 完成系统架构、数据流、RTK/双 EKF、双目、Nav2、Collision Monitor 和四足地形设计。
- [x] 完成 Point-LIO 去 MRS 化可行性与 Odometry/TF 输出源码审计；已识别 covariance 缺失、默认 IMU-input 分支 angular twist 未赋值、非 base_link frame 和 TF 默认关闭等接口 Gate。详见 `doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md`。
- [x] 完成 Point-LIO ESEKF covariance 与 robot_localization Jazzy / navsat_transform 源码行为审计：确认默认 `use_imu_as_input=true` 时 Point-LIO 使用 24 维 `kf_input`，内部 covariance 可由 IKFoM `get_P()` 读取，但 publisher 未导出；位置/姿态误差在前 6 维、速度位于 12–14；MTK SO(3) 使用右乘局部误差，S2 gravity 为 DOF2/DIM3 且 P 使用 24 维 ambient 表达，需将 gravity covariance 投影到当前切平面；全部索引与线程安全仍需项目单测锁定。确认 robot_localization EKF 将小于 `1e-9` 的测量 covariance 对角元素提升到 `1e-9`，全零 covariance 会导致过度信任风险；navsat_transform 只拒绝 `STATUS_NO_FIX`/NaN，并直接使用 NavSatFix covariance，需在其前置 RTK quality adapter 检查解状态、龄期和 covariance type。详见 10 §8–11；仅 E1/E2 源码证据。
- [x] 完成 A–G 阶段拆分和测试/风险基线。
- [x] 完成 Lite3 厂商 ROS 2 路线源码评估与 ADR-0003，明确 Foxy/Jazzy 不共享默认 DDS，且 new sdk_deploy 的 DRDDS status 不等于完整 Odometry。
- [x] 建立 odometry fallback strategy 和 ADR-0004，将 Point-LIO 作为缺失厂商 odometry 时的首选备用 POC，并记录 FAST-LIO2/FAST-LIVO2 许可、per-point time 和 Orin 实机验证边界。
- [x] 设计本体 odometry 缺失时的备用路线：Point-LIO + MID-360 内置 IMU 首选 POC，FAST-LIO2 对照，Stereo VIO/FAST-LIVO2 暂缓；尚未在 Orin/MID-360 上实测。
- [x] 建立 Lite3 SDK/UDP bridge 与 Nav2 速度命令安全适配器接口契约。
- [x] 完成 Lite3 Legacy Lite3_ROS Foxy 与 current sdk_deploy 路线源码核验，发现两者消息/状态接口不同，并明确不能假设跨 Foxy/Jazzy DDS 互通。
- [x] 建立多传感器时间同步和标定方案，区分双目曝光同步、设备时钟对齐、运动补偿和 ROS 时间戳。
- [x] 建立机载供电与载荷预算：已知设备质量小计约 1.330 kg，已知负载约 22.8–69.4 W；所有未知项保留待称重/待测。
- [x] 将 mock 数据限定在 /mock/lite3 topic 与 mock_* TF frame，避免污染真实机器人数据流。
- [x] 完成网关协议 v0.1 语义草案及 envelope/sequence/消息类型/frame/数值/covariance 检查；新增 contact_state 必须携带 frame_id 的验证和 2 个单元测试，Humble/Jazzy 双发行版 37 项测试均通过（run 45332714322、45332706824、45332702335）。
- [x] 完成 Lite3 遥测协方差矩阵结构校验：在原长度/有限值/非负对角检查上增加对称性与 3x3/6x6 正半定（PSD）检查，使用无 NumPy 依赖的 Jacobi 特征值算法；保留 sensor_msgs/Imu covariance[0] == -1 未知估计标记的 allow_unknown 语义。新增 5 项边界测试；ROS 2 package CI run 45377526595 的 Humble/Jammy、Jazzy/Noble 两个 job 均 success。代码提交 df302eb6b767bc8dd521ac7fd121b6cf38fef645。
- [x] Point-LIO P0 来源准备 CI 通过：checkout 上游 gitlink SHA 97b0042...，按顺序应用 Intel 3 份固定补丁，校验 patch commit subjects、lock marker 和 clean worktree（run 45334159646）。只证明源码准备可复现，不代表 build/replay 或硬件验收。
- [x] 建立供电/载荷初步预算，依据厂商公开功率/质量列出已知小计和未确认项目。
- [x] 完成本体和传感器技术预选。
- [x] 核对云深处 Lite3 Venture + AGX Orin + MID-360 的官方参考配置。
- [x] 核对 e-con Systems See3CAM 彩色无镜头 SKU 与官方 Trigger / Global Shutter / USB3 规格。
- [x] 核对 JetPack 7.2.2 对 Jetson Orin 的 Ubuntu 24.04 基线，并据此将 Jazzy 设为 Orin 当前长期目标；记录 Lyrical 的 Ubuntu 26.04 前置条件。
- [x] 创建 ADR-0002 硬件选型记录和更新硬件接口清单。
- [x] TaskPlan 拆分为根目录进度台账 + 两份设计分册。
- [x] 更新 Lite3 bridge 接口契约，区分 Legacy standard ROS topics 与 current DRDDS / rl_deploy interfaces；记录 vendor odometry availability 为待确认。
- [x] 新增 `robodog_gnss_quality` 独立 ament_python 包与 Humble/Jazzy CI：显式 FIXED carrier-solution gate、receiver GNSS-fix flag、NavSatStatus 合法值、源时钟 epoch reset、host freshness、位置范围、NavSatFix covariance 类型/PSD/不确定度、haversine 3D jump gate 与连续 Fixed 恢复滞回；最终 30/30 tests 在 Humble/Jammy 和 Jazzy/Noble 均通过（GNSS CI run 45378893084）。仅完成纯算法核心，不含 ROS driver adapter、epoch pairing、NavSatFix 发布或 EKF 集成。
- [x] 扩展 `robodog_gnss_quality`：新增 UBX-NAV-PVT flags 解码，按 GNSSFixOK 与 carrSoln 0/1/2/3 映射 NO_SOLUTION/FLOAT/FIXED/UNKNOWN；新增 NavSatFix 位置记录与 NAV-PVT 的 iTOW 严格配对和接收时间偏差 Gate，拒绝历元不匹配而不做 nearest-time 猜配；补充 receiver clock epoch/周内 iTOW 边界说明。最终 CI run 45379297789：Humble/Jammy 与 Jazzy/Noble 各 42/42 passed。仍未实现具体 ROS driver adapter 或真实串口/接收机运行。
- [x] 将此前生成的 R3–R5 通用算法原型作为实验参考归档保存至 docs/experimental/assistant_r5_reference/；仅保存选定纯 Python 核心/测试/报告，不并入正式 src/。原型报告的 102/102 是离线单测/配置/源码契约检查，不是 ROS 构建、pluginlib、Nav2 运行或实机验证；集成边界见归档 README。归档提交 73c7996ac8d06e3a71a2288a0e6be50504018303。

- [x] Livox SDK2 v1.3.1 + livox_ros_driver2 v1.2.6 在 Jazzy/Noble x86_64 编译与安装通过：[run 45337301420](https://github.com/rexyll/robdog_nav2/actions/runs/45337301420)。
- [x] 固定 Point-LIO Intel Jazzy port 在 Jazzy/Noble x86_64 编译与安装通过：[run 45337200320](https://github.com/rexyll/robdog_nav2/actions/runs/45337200320)；colcon 2 packages finished，`pointlio_mapping` executable 被找到。
- [x] MID-360 rosbag 时间审计工具已增加 CLI 质量阈值参数防护（负数、NaN、±Inf 拒绝；0/未设置接受），并加入“校验先于 rosbag 打开”的端到端测试；最终 CI [run 45384314282](https://github.com/rexyll/robdog_nav2/actions/runs/45384314282) 的 Python 3.12 编译、CLI help 与 21/21 项测试通过。真实 MID-360 bag/时钟同步仍未验证。
- [x] 完成 ZED-F9P ROS 2 驱动候选评估与固定：`aussierobots/ublox_dgnss` 0.7.6 (`9d4f5add...`, Apache-2.0) 和源码依赖 `rtcm_msgs` 1.1.6 (`8d27ab5f...`, BSD-3-Clause)。Humble/Jammy 与 Jazzy/Noble 两 CI job 均通过：7 packages finished、安装完成并通过 UBXNavPVT/HPPosLLH/COV iTOW/quality field 验证：[run 45387577856](https://github.com/rexyll/robdog_nav2/actions/runs/45387577856)。仅为跨发行版软件构建/接口证据，不代表实机 USB、NTRIP、RTK Fixed 或 PPS 验收。
- [x] 扩展 GNSS 纯算法 core 以适配固定 u-blox 原始 UBX topic：新增 PVT/HPPOSLLH/NAV-COV/NAV-STATUS 四路精确 iTOW join、接收单调时差 gate、质量字段一致性、位置有效标志检查和完整 NED→ENU covariance 矩阵转换；新增 14 项测试，最终 GNSS CI [run 45388146723](https://github.com/rexyll/robdog_nav2/actions/runs/45388146723) 在 Humble/Jammy 与 Jazzy/Noble 各 56/56 passed。该函数仍是纯算法，不含实际 ROS topic subscriber、bounded cache、receiver time mapper 或 NavSatFix publisher。
### 未完成 / 阻塞
- [x] ROS 2 package CI 最新复跑：[run 45328982630](https://github.com/rexyll/robdog_nav2/actions/runs/45328982630)，Humble/Jammy 与 Jazzy/Noble 均 build 成功、35 passed，0 errors/failures/skipped；仅覆盖 mock/契约逻辑，不覆盖真实 SDK/传感器。
- [x] 已在本地 Python 环境对纯 freshness helper 的 6 个单元用例运行 unittest，6/6 通过；这不替代 ROS 2 package CI 或真实硬件验证。
- [ ] 未采购设备，未接收实物。
- [!] UrbanLoco 专项 P2 仍被原始 ROS1 bag/可验证转换来源缺失阻塞；现有 standalone DB3 仅为诊断样本，来源不能推定。该阻塞不再暂停整体 LIO/四足验证：按 B13 使用替代数据集分别建立证据，UrbanLoco 保留为可选复现任务。
- [ ] 质量预算中仍有相机/镜头、网络交换机、触发板、DC/DC、支架和线束未称重；3.6 kg 内部载荷目标尚未验证。
- [ ] 尚未获得 Lite3 Venture 电源额定、载荷/重心和安装 CAD 的书面确认；因此不能批准从机器人 24V 辅助口直接给 AGX Orin 供电。
- [!] 当前公开 RobotStatus 不含 base xyz/linear velocity/covariance；尚不能排除厂商存在未公开 estimator channel，需书面确认正式腿式 odometry/body velocity。Foxy/Jazzy 仍须隔离。
- [ ] 双目精确 SKU 已明确，但 GPIO 触发电压/极性/时序、镜头、双机硬件同步和 Jetson 双路 USB3 带宽尚待供应商确认。
- [ ] RTK NTRIP 改正服务或基站尚未确定；ZED-F9P ROS 驱动候选及跨 Humble/Jazzy 软件构建已通过，但真实板卡 USB/固件配置、UBX topic/频率/丢包/iTOW 匹配、ROS subscription adapter、PPS/timebase 和 Fixed/Float 现场状态仍待实测。
- [ ] 载荷、供电、热设计、质心和续航未实测；质量预算还缺相机本体、交换机、触发板、DC/DC、支架和线束，功率预算还缺转换损耗与峰值测量。
- [ ] 厂商 Lite3 SDK、MID-360 驱动、双目相机驱动和 RTK 驱动尚未在 Jetson 真实硬件上编译；尚无传感器 rosbag 和 Nav2 实机结果。
- [ ] Lite3 mock topic 骨架不是实际 SDK bridge，不可接入 robot_localization 或运动控制；真实 SDK/UDP 数据适配仍待目标固件验证。
- [ ] gateway wire codec、UDP/TCP、vendor source mapping 均未实现；42 项纯逻辑测试通过及双发行版 CI 成功仍不代表实际网络链路/厂商 SDK 验证。Covariance validator 已增加矩阵对称性和 PSD 结构检查，但不证明来源协方差经过统计标定或真实可信；全零矩阵虽为数学 PSD，仍须由来源质量策略决定是否可接受。
- [x] 新增消息类型/frame/covariance 检查后的最终 CI 已通过；失败 run 45328900700 根因为 `status` 未加入 message_type 白名单，修复后 45328937159 与 45328982630 双发行版均通过。
- [ ] 未确认的 legged odometry 是定位架构的实质性阻塞项：如果 vendor SDK 不提供可信本体 odom，需转向经验证的 LiDAR odometry/SLAM 或 stereo VIO。
- [ ] Dynamic03 GNSS Gate：range_meas 1054信号载波相位有效0、PVT全量diff_soln=false/carr_soln=0，当前bag不足以重建RTK Fixed。M3DGR SPP源码默认订阅raw/fix；MAVROS源码确认其来自MAVLink GPS_RAW_INT，covariance_type=1为eph/epv近似协方差分支，但物理接收机映射/RTK改正状态仍未闭环。需作者提供飞控参数/接线/RTCM证据；GT禁用于APE/RPE。详见策略§6.15–6.16。
- [!] lite3_transfer 启动/SDK_MODE 可能获取 SDK 控制权；未获得厂商只读 bring-up SOP、物理急停及遥控接管说明前，不在实机启动 SDK mode、不发送关节/速度命令。
- [ ] 时间同步方案已文档化；双目硬件触发、MID-360 PTP/内置 IMU 时钟、Lite3 UDP 时间戳、ZED-F9P PPS 均未测量。

- 2026-11-25 / v0.3.104：在 `doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md` §4.9–4.10 补齐 NAV-PVT 与 NAV-TIMEUTC byte-level parser fixtures。按 ZED-F9P R07 固定 NAV-PVT 92-byte 布局（UTC civil fields、valid byte offset 11、signed nano offset 16、flags2 offset 22）及 NAV-TIMEUTC 20-byte 布局（signed nano offset 8、UTC civil offsets 12–18、valid byte offset 19）；分别定义 PVT validDate/validTime/fullyResolved 与 confirmedAvail/confirmedDate/confirmedTime 的独立组合、TIMEUTC validTOW/validWKN/validUTC/utcStandard mask、负 nano、sec=60、reserved bits、payload 长度和日历边界测试。补充说明 parser 结构解码与 mapper policy 必须分层，确认元数据缺失只能进入 UNCONFIRMED。静态核对现有包仍为 `UNSPECIFIED` license 与 `maintainers@example.invalid` 占位 maintainer；不擅自修改 metadata、不创建生产代码、不运行测试/CI、不宣称接收机兼容。证据为官方 R07 协议 E1/E2 + 本项目静态结构与 E3 合成测试规格；目标 F9P MON-VER 和真实输出仍待用户提供/设备采集。

- 2026-11-25 / v0.3.111：在 §4.14.3 补充 BIPM 对 TAI−GPS 的关系为 19s + C₀（C₀ 为随时间变化、量级数十纳秒），明确 USNO 整秒 TAI−UTC 历史表减 19s 仅能得到整秒 GPS−UTC leap offset，不能证明纳秒级 GPS/TAI realization。将 §4.13 输出定位为 nominal integer timeline，要求另行保留 receiver tAcc/时间实现不确定度；纳秒级溯源若成为需求，需另引入 BIPM/UTC(k)−GPS C₀ 数据及独立精度合同。仅修订设计边界，无代码、artifact、测试或硬件验证。
- 2026-11-25 / v0.3.110：在 `doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md` §4.14 定义 leap-transition evidence 数据模型与来源策略。区分 IERS Bulletin C 事件公告、USNO tai-utc.dat 历史 TAI−UTC 表、接收机 NAV-TIMELS 事件字段与项目冻结离线 transition artifact；记录 F9 HPG 1.30/R01 的 24-byte NAV-TIMELS 字段、validCurrLs/validTimeToLsEvent 独立性、timeToLsEvent 正负语义及 GPS week/day 锚点，并明确 lsChange=0 不等于无历史事件。定义 canonical transition record（前后 GPS−UTC offset、delta、GPS effective week/TOW、来源版本/URL/hash、发布时间/获取时间、parser version、review status、coverage），规定 TAI−UTC 不能直接当 GPS−UTC、TAI−GPS=19s 关系需单独来源核验；新增 receiver current-offset/event-evidence 双路径、冲突 fail-closed、离线 artifact 更新治理及 14 项测试合同。未生成生产 artifact、未写代码/依赖、未运行测试/硬件验证；目标 F9P MON-VER 与 raw TIMELS 帧仍待采集。
- 2026-11-25 / v0.3.109：在 `doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md` §4.13 固化 UTC civil label→GPS integer timeline 数学合同。定义 Gregorian 名义 UTC 纳秒坐标 + 同历元可信 NAV-TIMELS.currLs 得到 GPS absolute ns，再以整数 divmod 计算 week/TOW；明确 GPS epoch、604800s week、整数/溢出/负周边界。用 2016 正闰秒 23:59:59 / 23:59:60 / 次日 00:00:00 分别使用 L=17/17/18，证明 GPS 时间线连续且不得折叠 sec=60；signed nano 需 leap-aware 借位/进位。补充 iTOW 毫秒与 TOW 纳秒 sub-ms 精度合同和 12 项数学测试向量。审查发现 sec=60 的合法日期/事件边界无法仅由当前 currLs 证明，P1 前需设计 transition evidence API；相关来源/精度阈值保持未冻结。仅更新设计文档，无代码/依赖/测试运行/硬件验证。
- 2026-11-25 / v0.3.108：对 §4.12 source policy/status contract 与文档前文进行交叉一致性审查，修正 §2.1 输出状态枚举与 §4.12.4 不一致的问题；将旧草案 VALID_CONFIRMED/VALID_UNCONFIRMED 统一映射为 VALID/UNCONFIRMED。同步修正 §4.6 VALID-009/010：NAV-TIMEUTC validWKN 或 NAV-STATUS towSet/wknSet 不满足时，不得把仅由 UTC+leap offset 算出的时间当作导航消费的 VALID；缺失/false 归 AMBIGUOUS_INVALID，gpsFixOK=false/no-fix 才归 NO_FIX。进一步限定 DEGRADED 仅用于四类必需输入均完整有效、可选附加核验（如 NAV-TIMEGPS）缺失的诊断状态，不能绕过必需源。仅文档合同修订，无代码/测试运行。
- 2026-11-25 / v0.3.107：在 `doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md` §4.12 完成 UTC→GPS mapper source-policy table 与纯算法 property-style test contract。明确 PVT 主候选、TIMEUTC 同历元核验、TIMELS current offset/source allowlist、STATUS fix/time validity 的职责边界；规定四消息按 exact iTOW 且同 process session/connection generation 配对，禁止 nearest-neighbor/interpolation，并以 monotonic receive time、sequence fence 和 bounded cache 隔离周回绕与旧 generation。补充 signed nano、sec=60、PVT/TIMEUTC 冲突、TIMELS stale/unknown source、STATUS validity 与 fail-closed 状态机（NO_FIX/UNCONFIRMED/VALID/DEGRADED/AMBIGUOUS_INVALID/STALE/RESETTING）；仅 VALID 默认允许进入导航消费通道。定义 12 组属性测试不变量，覆盖 Gregorian/nano、2016 闰秒连续性、GPS week 边界 ±1ns、乱序/重复/错配、source/validity 枚举、generation flush、bounded cache 与状态输出。静态检查现有包仅声明 pytest，测试使用 unittest；未发现 Hypothesis 声明，因此不擅自增加依赖。所有 age/skew/tAcc/tolerance/cache/allowlist/hysteresis 数值保持 unset，未运行测试/CI，未新增生产代码；目标设备 MON-VER/原始帧仍待采集。
- 2026-11-25 / v0.3.106：完成 `robodog_gnss_quality` 包元数据问题的分支来源核验。GitHub compare 显示 `nav` 相对 `main` ahead 93 / behind 0，merge-base 为 `07a740711783c219f3c091a5c8e9f2fecc71cbc3`；本轮核验起始 `main` HEAD 为该 SHA，`nav` HEAD 为 `81eafa858b18778d351c8db4d65378972c6d8340`。在 `main`、`nav` 和共同基点分别读取 `src/robodog_gnss_quality/package.xml` 与 `setup.py`，三处 Blob SHA 完全一致：package.xml `88917287fe25064b6b64b4dda2cc9a18edff87c4`，setup.py `1459b8f56f57b2dbb15ecfd6c43633d5083e87f9`；内容均含 `UNSPECIFIED` license 与 `maintainers@example.invalid`。因此可确认这是分支创建时已存在的 main 基线遗留项，不是 nav 专项引入或修改。未更改两个文件、未填写未经确认的 SPDX/maintainer，不影响其他 GNSS 算法研究；仅在进入正式实现/发布 Gate 时仍需治理确认。证据为 GitHub 分支/compare 元数据与三个 ref 的文件内容/Blob SHA 静态核验，无构建/运行/硬件测试。
- 2026-11-25 / v0.3.105：固定审计 aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e 的 NAV-PVT/TIMEUTC/STATUS payload、ROSIDL 与发布链，在 doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md §4.11 记录字段映射、原始 validity/reserved byte 丢失、STATUS 仅输出 wkn_set/tow_set 而无 week 数值，以及 NAV-TIMELS transport/message 缺失。发现固定 offset 的 payload constructor 与 buf_offset helper 未在已审计路径实施 exact-length guard；将 P0 设为字段读取前 class/id/exact length validation，并要求 malformed payload tests。现有 PVT/TIMEUTC/STATUS 可复用已有 UBX dispatch 与 ROS topics；NAV-TIMELS 仍需 ROSIDL、payload、dispatch、publisher。仅静态审计，无编译/畸形帧注入/实机测试；metadata license/maintainer Gate 未通过，不修改生产代码。

- 2026-11-25 / v0.3.112：在 `doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md` §4.15 固化 USNO TAI−UTC 历史分段解析与离线 artifact 生成规格：定义原始记录 grammar/字段保真、GPS epoch 1980-01-06 clipping（1980-01-01 TAI−UTC=19s，推得 GPS−UTC=0s）、整秒/零 drift 准入、JD 日期交叉校验、严格排序/连续性/±1s transition 约束，以及 2016-12-31 23:59:60 与 2017-01-01 effective midnight 的独立语义；新增 USNO-PARSE-001..008 与 USNO-GEN-009..016 测试合同。IERS 历史表/BIPM 表已交叉确认 2015-07-01 TAI−UTC=36s、2017-01-01=37s，减去 19s 后 GPS−UTC 为 17s/18s。当前未获取 USNO 原始文件字节/hash，未实现 parser/generator、未生成 VERIFIED artifact、未运行测试；来源快照 Gate 保持打开。

### 下一步（按顺序执行）
1. 已核验 reset 测试提交 `33919e40cd2e586ec850379e767e835ad8ea8ff8`（GNSS run `45574153276`）与 `7a001103a8e368f6eed1e8dd225e174e71465277`（GNSS run `45574226464`）的 Humble/Jammy、Jazzy/Noble jobs 均成功；并已静态核对新增测试调用的 Gate/Joiner API 与实现一致。已新增 `doc/Navigation/11_gnss_generation_and_reset_coordinator_design.md`，定义 ReceiverClockEvidence/generation 判定、ResetCoordinator 事务、EpochFence 与测试矩阵。固定 ZED-F9P driver `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e` 已审计：当前接口没有 NAV-TIMEGPS week 数值、持久 receiver boot/session ID 或 ROS connection-generation event；NAV-STATUS `msss` 是 uint32 startup/reset 毫秒计数，不能单独区分自然回绕与 reboot。进一步协议审计已修正时间主链：以 NAV-PVT UTC validity/confirmation + NAV-TIMEUTC 同历元核验 + NAV-TIMELS 当前 leap-second offset + NAV-STATUS tow/week validity 构建 UTC→GPS mapper；NAV-TIMEGPS 仅作为可选交叉校验。固定驱动缺 NAV-TIMELS 与 connection-generation event，下一步评估这两项最小扩展并等待用户提供目标 F9P MON-VER/固件协议版本；已进一步固定 driver 代码插入点：NAV-TIMELS 需新增 ROSIDL msg、payload parser、ubx_nav dispatch 与 node publisher；USB generation 需与 libusb hotplug/UBX ingress queue 共用 sequence fence，并审查不同 callback groups 在 MultiThreadedExecutor 下的并发安全。UTC mapper source-policy table 与 12 组纯算法 property-style 测试合同已写入 doc/Navigation/14 §4.12；本轮一致性复核统一 §2.1 与 §4.12 的状态枚举，修正 VALID-009/010 的 STATUS validity 规则，并明确 NO_FIX 与 AMBIGUOUS_INVALID 的区别、DEGRADED 不得绕过四类必需输入；设计未实现，现有 package 只声明 pytest/unittest，未引入 Hypothesis。已完成 §4.13 UTC civil label→GPS integer timeline 数学定义：明确普通秒公式、GPS epoch/周长/divmod、不使用 POSIX timestamp、2016 闰秒前/中/后分别采用 L=17/17/18、signed nano 的 leap-aware 归一化、iTOW 毫秒与 TOW 纳秒精度边界及 12 项数学测试合同；同时识别出 sec=60 还需要 transition evidence API，§4.12 的 current-offset policy 单独不足。已完成 §4.14 leap-transition evidence 数据模型与来源策略：区分 IERS Bulletin C 公告、USNO TAI−UTC 历史表、接收机 NAV-TIMELS 事件字段和项目冻结离线 artifact；定义 canonical transition record/provenance/hash/review status/coverage、TAI−GPS=19s 转换审计边界、receiver current-offset 与 event-evidence 双路径、来源冲突 fail-closed、运行时无网络更新及 14 项测试合同。已用 BIPM 年报交叉核验 TAI−GPS 关系应写为 19s + C₀（C₀ 为随时间变化、量级数十纳秒），因此 USNO 整秒 TAI−UTC 表减 19s 只能生成整秒 GPS−UTC leap offset，不能声称纳秒级时间实现精度；§4.14.3 已增加 nominal integer timeline 与 receiver uncertainty 的边界。下一步获取并归档 USNO 原始 tai-utc.dat 字节快照与 SHA-256，逐行核验 §4.15 parser grammar/日期/JD/drift/offset；再生成仅 DRAFT 的离线 transition artifact，并用 IERS/BIPM 独立向量审核；driver P0 exact-length hardening、raw validity observability 和 NAV-TIMELS ROSIDL/dispatch/publisher 作为独立补丁 Gate，待 package license/maintainer 获确认后再实施；真实 mapper/硬件验证仍受目标 firmware 和设备日志 Gate 阻塞。正式 coordinator/mapper 代码仍等待用户/项目维护者确认 package SPDX license 与 maintainer，不引入 ROS runtime dependencies。
2. 严格在 `nav` 分支执行 N0→N9：N0 范围/架构/术语/证据基线已冻结；N1 已完成 Nav2 主任务链首轮源码审计（NavigateToPose→BT→ComputePathToPose/Planner→FollowPath/Controller→Smoother→Collision Monitor），继续追踪 Costmap 观测源和非 Nav2 上下游接口；Nav2 的完整内部深研仍归 N6。
3. N1 将系统级 Producer/Consumer 清单细化为源码/消息定义可验证的接口矩阵，优先检查传感器→定位/感知→地图/Costmap→规划→命令仲裁→四足执行反馈，避免只研究孤立软件包。
4. 并行保留原硬件 Gate：推进 ZED-F9P ROS adapter/UBX 时钟与质量 gate、Lite3 正式 odometry/运动接口/急停/SDK 权限确认、双目同步/镜头/USB 带宽、载荷/供电/热设计。
5. 保持 Dynamic03 GT provenance gate：不排序/去重原始 GT，不将 2,594 行候选用于 APE/RPE；在 RTK/GT 来源、时钟、天线杆臂和 frame semantics 证据闭环前，不运行或宣称精度验收。
6. 保持 x86_64、通用 ARM64 build、Jetson Orin 实机 build、仿真、真实 bag 回放、HIL、真实机器狗验证分级；不得混称。
7. N9 专项验收前不回 main、不合并本专项成果；完成后先做差异/风险/CI/回归审查，再形成回 main 的提案。

---

## 4. 进度变更记录
- 2026-11-25 / v0.3.98：在 `doc/Navigation/13_zed_f9p_utc_gps_time_mapping_audit.md` §10 进一步追踪固定 driver 的实际扩展路径：NAV-TIMEUTC 当前采用 header payload parser、ubx_nav.hpp FrameComms/dispatch、ublox_dgnss_node.cpp publisher 三层；新增 NAV-TIMELS 还必须登记显式 ROSIDL message list。确认 libusb hotplug callback 由 10ms handle_usb_events timer 的 event processing 路径调用，attach/detach 会同步调用 node callback；UBX ingress callback 记录 rclcpp::Clock().now() 并向 ubx_queue_ push，UBX consumer 属于另一 callback group，且当前 size/front 读取未持有 queue mutex。指出不同 MutuallyExclusive callback groups 在 MultiThreadedExecutor 下不自动互斥，裸共享状态和 queue 读取需线程安全 Gate；提出 process_session_id、connection_generation、每帧 generation/sequence、同序 lifecycle event queue、detach invalid barrier 和 snapshot+event stream 设计。未确认实际 deployment 使用多线程 executor，也未运行 TSAN/USB stress；风险仅静态条件性证据，不宣称复现 bug。更新 TaskPlan 下一步。
- 2026-11-25 / v0.3.97：进一步对照固定驱动消息定义与 u-blox ZED-F9P Interface Description UBX-18010854 R07 / Integration Manual UBX-18010802 R12，纠正 §12 原先将 NAV-TIMEGPS 作为优先时间映射方案的表述。确认 NAV-PVT 已提供 valid_date/valid_time/fully_resolved、confirmed_avail/date/time 和 nano；NAV-TIMEUTC 提供 valid_tow/valid_wkn/valid_utc/utc_std 但 valid_wkn 只是标志、没有 week 数值；NAV-TIMELS 提供 currLs/validCurrLs/srcOfCurrLs，可作为当前 GPS−UTC offset 候选，但默认来源可能过期且 unknown 不可信；NAV-TIMEGPS 在 u-blox 9 protocol 27.11 定义 week/fTOW/leapS，但 Integration Manual 推荐 UTC 时间作为可靠绝对时间来源。固定驱动已支持 PVT/TIMEUTC/STATUS，但缺 TIMELS 与 connection-generation event。新增 `doc/Navigation/13_zed_f9p_utc_gps_time_mapping_audit.md`，明确 UTC+leap-second→GPS week/TOW 算法边界、精确 iTOW join、负 nano/sec=60 处理、source validity、驱动最小扩展和测试矩阵；并在 §12 增加修正说明、更新 Navigation README 和 TaskPlan 下一步。证据为固定上游源码与公开协议/集成手册 E1/E2，设计为 E3；目标板卡 firmware/protocol、真实设备日志、mapper 实现与测试均未完成。
- 2026-11-25 / v0.3.96：固定审计 ZED-F9P 候选驱动 `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e` 的 NAV-STATUS/NAV-TIMEUTC/NAV-PVT 消息、NAV parser registrations、USB input callback 与 hotplug attach/detach。确认 NAV-STATUS 提供 uint32 `msss`（milliseconds since Startup/Reset）但没有持久 boot/session ID；NAV-TIMEUTC 有 `valid_wkn` 标志和 UTC calendar 字段但没有 GPS week 数值；固定驱动未发现 NAV-TIMEGPS message/parser/publisher；USB detach/attach 只更新内部 readiness/parameter state，没有向 ROS adapter 发布 connection generation event；UBX ROS header.stamp 来自 USB 输入回调的 `rclcpp::Clock().now()`，不是 receiver time 或 steady freshness。新增 `doc/Navigation/12_zed_f9p_receiver_generation_evidence_audit.md`，明确不能用 msss 下降单独判定 reboot，提出驱动扩展 NAV-TIMEGPS + connection lifecycle interface 或验证 UTC/leap-second-aware mapper 两条路线；在真实设备证据前 generation mapper 必须 fail-closed。更新 Navigation README 与 TaskPlan。固定源码 E1/E2、设计建议 E3；未连接真实接收机、未运行 bag/HIL/实机测试。
- 2026-11-25 / v0.3.95：新增 `doc/Navigation/11_gnss_generation_and_reset_coordinator_design.md`，把 GNSS generation 判定与跨组件 reset 设计细化为可实施合同：ReceiverClockEvidence 输入、CONTINUOUS/NEW_GENERATION/AMBIGUOUS 决策、week rollover 与 reboot 区分、generation controller 状态机、ResetCoordinator 串行事务/ACK/idempotency/fail-closed、EpochFence 消费规则及确定性测试矩阵。特别规定连续 GPS week rollover 不应无故重置 Joiner/Gate generation；iTOW 倒退单独不足以判定 reboot；平台物理停车 ACK 与 ROS reset ACK 分离。更新 Navigation README 索引与 TaskPlan 下一步。仅 E3 设计规格，未实现 coordinator/mapper/adapter，未修改 package metadata；真实 ZED-F9P 时间映射与 boot evidence 仍待驱动源码和设备日志验证。
- 2026-11-25 / v0.3.94：完成 v0.3.93 新增 Gate/Joiner reset 测试的源码 API 交叉核对；Gate 的 `reset()`、recovery counter、`RECOVERING` reason 与 observation 字段，Joiner 的 `reset()`、`begin_generation()`、generation fence、pending/retired/lifetime eviction counter 均与实际实现吻合。通过 GitHub Actions API 精确查询 push run，确认 GNSS quality gate CI：commit `33919e40cd2e586ec850379e767e835ad8ea8ff8` run `45574153276`、commit `7a001103a8e368f6eed1e8dd225e174e71465277` run `45574226464`，两次均由 Humble/Jammy 与 Jazzy/Noble 两个 job 成功完成（Build and test GNSS quality package step success）；相应 ROS 2 package CI 两次亦成功。此为 CI 证据，不等于 Jetson Orin、真实 ZED-F9P、bag/HIL 或实机验证。更新下一步为设计 generation controller / ResetCoordinator / EpochFence 的纯算法边界；正式代码仍受 SPDX license 与 maintainer metadata 确认 Gate 约束，未修改 package metadata、未增加 ROS 依赖。
- 2026-11-25 / v0.3.93：为 GNSS core 的 Gate/Joiner reset 行为补充两组 unittest 回归测试。Gate 测试覆盖显式 reset 清除 recovery counter 与 jump reference，并确认新 generation 的低 source timestamp/位置不会被旧 reference 错误拒绝；Joiner 测试覆盖 reset 清空 pending/retired、保留 lifetime eviction counter、显式 begin 新 generation 后拒绝旧 generation callback 且不污染新 cache。提交时测试尚未运行；其后 Humble/Jammy 与 Jazzy/Noble CI 结果见 v0.3.94 记录。未改 package metadata、未实现 coordinator、未增加 ROS dependencies。\n\n- 2026-11-25 / v0.3.92：追加 10 专题文档 §23，静态核对 GNSS core 的 ament_python packaging 与测试工程。确认默认 runtime 仅 setuptools、测试采用 unittest.TestCase 并由 setup.cfg 配置 pytest 收集；无 pyproject.toml/requirements.txt，Python 版本矩阵未声明；package.xml/setup.py license=UNSPECIFIED，maintainer email 为示例占位。确定现有 core 可容纳 transport-independent coordinator，但正式代码/发布前需治理确认；本轮不更改 metadata、不引入 ROS runtime dependencies、不创建代码。无 colcon/pytest/unittest 执行证据。\n\n- 2026-11-25 / v0.3.91：追加 10 专题文档 §22，完成 GNSS/LIO/EKF/Nav2 逐事件 reset transition table 与 GNSS core reset API 对照。源码确认 Gate.reset 清除恢复/跳变参考；Joiner.reset 清空 epoch cache/order 并要求显式 begin_generation，但累计 diagnostics counters 保留；begin_generation 不负责验证 GPS week/reboot。识别 Gate source_clock_id 自动 reset 与 Joiner 显式 generation fence 的行为差异，规定 coordinator 必须统一 generation 权威。定义 FIX→FLOAT、receiver reboot、week rollover、clock ambiguous、LIO/local EKF reset、global datum、ROS clock rewind、Costmap stale 的前置条件、epoch 变化、reset 顺序、ACK 与失败动作，并给出 GNSS 子事务的 exactly-once 发布合同。仅静态源码与 E3 设计证据，未重跑测试、未实现 ROS adapter/coordinator、无硬件验证。\n\n- 2026-11-25 / v0.3.90：新增 10 专题文档 §21，形成跨组件 Epoch/Reset Coordinator 设计合同。区分 estimator_epoch、local_odom_epoch、gnss_receiver_generation、map_epoch 与 reset_transaction_id；明确 UBX joiner/RTK gate/time mapper、Point-LIO、local/global EKF、地图层、Nav2 与平台安全层的 reset ownership；定义 GNSS quality downgrade、receiver reboot/week remap、LIO reset、local odom discontinuity、global datum change、ROS clock rewind 的最小影响范围；设计串行事务状态机、跨节点 generation fencing、logical atomic visibility、ack/drain/warm-up/commit 与 fail-closed abort；给出 transport-independent 数据契约和 10 类纯算法测试。仅 E3 项目设计合同，未实现 coordinator/ROS adapter，未新增构建、bag、HIL 或实机证据。\n\n- 2026-11-25 / v0.3.89：继续静态审计本项目 `robodog_gnss_quality` 的 `quality_gate.py`、`ubx_epoch_joiner.py`、`ubx_nav_pvt.py` 与测试。确认现有 core 已覆盖 RTK Fixed fail-closed、host freshness、source monotonicity、坐标/covariance/uncertainty/3D jump 与恢复滞回；四路 UBX exact-iTOW pairing、bounded cache、stale/late/conflict/duplicate 防复活、显式 receiver generation/reset 和 NED→ENU covariance cross-term 变换。枚举现有 30 个 Gate tests + 17 个 joiner tests（本轮未重新运行）。识别缺口：GPS-week/receiver time mapper、跨 joiner+gate+EKF 的原子 reset coordinator、ROS diagnostics/quality status、NavSatFix 与 ACCEPTED epoch 原子关联、组合级 property/fuzz/设备时钟测试。追加 10 专题文档 §20，未修改 package license/maintainer 占位信息。仅静态源码/测试证据，无新增 CI、设备或 bag 验证。\n\n
- 2026-11-25 / v0.3.88：固定 Navigation2 Jazzy `c704655...` 的 Costmap2DROS、PlannerServer、ControllerServer 与 robot_utils 源码，追踪 pose/TF lookup 失败传播。发现 Costmap `updateMap()` 获取 robot pose 失败时只跳过当前更新周期，不会显式把 LayeredCostmap current 置 false，因此 plugin current 不能代表定位有效；Planner TF 起点失败映射为 TF_ERROR 并终止规划，Controller pose 失败映射为 TF_ERROR、发布零速度并终止 FollowPath，但零速度不构成底层停车确认。Nav2 不原生消费 LIO health、EKF measurement age/covariance、GNSS receiver generation 或 estimator epoch。向 10 专题文档追加 §19，形成故障传播矩阵与项目级 validity/动作状态机；并确认 GNSS core 包 license/maintainer metadata 仍需项目治理确认。仅 E2 固定源码与本仓库静态证据，无新增构建、bag、仿真、HIL 或实机验证。\n\n2026-11-25 / v0.3.87：继续固定 robot_localization Jazzy `fea7fc3...` 源码，追踪双 EKF TF 生成与 reset 实现。确认 `world_frame=odom` 直接发布 `odom→base_link`；`world_frame=map` 需在同一时间查询 local `odom→base_link`，再计算 `map→odom`，global EKF 不应与 local EKF争抢同一 TF。显式 reset 清理本实例 measurement queue/history、TF buffer、per-topic timestamps 和 filter state，但不协调另一个 EKF、LIO/GNSS epoch 或 Nav2 状态。`reset_on_time_jump` 参数虽声明，回调 reset 分支仍注释；重复 stamp 可接受、旧 stamp 丢弃。发现 `validateFilterOutput()` 检出 NaN 后当前周期仍未通过该返回值阻止发布，且 sensor-timeout prediction 的输出 stamp 不能证明有新传感器观测。核对 `robodog_gnss_quality` package metadata：仍是纯 ament_python core，无 ROS dependencies/entry point，license=UNSPECIFIED、maintainer email 为示例占位值；ROS adapter 前必须补齐可核验许可/维护者信息与实际驱动消息契约。追加 10 专题文档 §16–18。仅 E1/E2 固定源码与本仓库静态证据，无新增构建、bag 或硬件验证。

- 2026-11-25 / v0.3.86：固定 Point-LIO 与 robot_localization Jazzy 源码继续审计 tracking health/reset/lagged data。Point-LIO `is_initialized_` 不代表 tracking valid；LiDAR/IMU 时间回退仅 log+return，队列未清理；`flg_reset` 在固定文件中未见置位，`ImuProcess::Reset()` 不重置 ESEKF/map/TF/时间 epoch。robot_localization 对每 topic 旧 stamp 丢弃、duplicate stamp 接受；跨 topic lag 可 history replay，但 history 不足时失败后仍继续处理；`sensor_timeout` 只预测不失效；`reset_on_time_jump` 当前仅声明且处理分支注释。确认项目已有 `robodog_gnss_quality` 纯算法 core 和 UBX 四路精确 iTOW joiner，缺 ROS adapter/time mapper/diagnostics/NavSatFix publisher。追加专题文档 §12–15，定义 health/reset contract 与新增验证 Gate。证据仅固定源码静态审计，未新增项目构建、bag 或硬件验证。
- 2026-11-25 / v0.3.85：继续深入固定 Point-LIO vendored IKFoM MTK manifold，确认 SO(3) `boxplus/oplus` 使用右乘 `R·Exp(δθ)`，`boxminus` 使用 `Log(R_otherᵀ·R)`，旋转 covariance 因而处于右侧局部切空间；gravity S2 为 DOF=2、DIM=3，而 ESEKF covariance 使用 24 维 ambient 表达。结合 state field 顺序与 `P_init` block offsets，进一步固定 `kf_input` 的候选索引：position 0–2、rotation 3–5、LiDAR-IMU extrinsic rotation 6–8、translation 9–11、velocity 12–14、gyro bias 15–17、accel bias 18–20、gravity ambient block 21–23。明确 gravity covariance 必须按 S2 约束投影到当前切平面，不能直接当作普通三维向量 covariance；所有索引仍须以项目 adapter 编译期/运行期断言和单测最终验收。更新 10 专题文档；仅固定上游源码证据，不代表项目已实现或验证。

- 2026-11-25 / v0.3.84：继续 N1/N3 状态估计接口审计，固定检查 Point-LIO `e7ff034...` 的 `estimator.h`、`laser_mapping.cpp`、默认 YAML 与 vendored IKFoM ESEKF，以及 `robot_localization jazzy-devel` SHA `fea7fc3...` 的 EKF/navsat_transform 实现。确认默认 `use_imu_as_input=true` 时 Point-LIO 使用 24 维 `kf_input`；IKFoM 有 `get_P()` covariance accessor，内部状态字段顺序为 pos/rot/extrinsic rot/extrinsic trans/vel/bg/ba/gravity，publisher 未导出 P；默认 IMU-input 分支 angular twist 保持消息默认值。确认 robot_localization EKF 对 measurement covariance 小于 `1e-9` 的对角元素设下限，故零 covariance 会带来过度信任风险；sensor_timeout 只预测、不提供传感器健康门控；smooth_lagged_data 可回滚重放延迟观测。navsat_transform `gpsFixCallback()` 只拒绝 STATUS_NO_FIX 和 NaN，直接复制 position covariance，未执行接收机 RTK FIX/FLOAT、差分龄期或 covariance_type 质量分级，必须由 GNSS adapter 前置门控。补充 covariance 的 SE(3)/SO(3) 映射、lever-arm、IMU 信息相关性、3D EKF 和双滤波 TF ownership Gate。更新 10 专题文档；仅 E1/E2 源码审计，无本项目构建、bag 或硬件验证。

- 2026-11-25 / v0.3.83：继续 N1，固定审计 CTU-MRS Point-LIO、MRS estimator plugin、MRS UAV StateGeneric/StateEstimator 与 robot_localization Jazzy。确认 Point-LIO 算法核心对 MRS 的依赖集中在节点/参数/pub-sub/timer/TF 等 middleware wrapper，原则上可用原生 rclcpp/tf2_ros 替换；MRS plugin 本身是 MRS UAV StateGeneric 插件，订阅硬件 API 姿态/角速度并组合 lateral/altitude/heading 子估计器，不是 Point-LIO Odometry adapter。源码审计发现 Point-LIO Odometry 未写 pose/twist covariance；默认 `use_imu_as_input=true` 时 angular twist 未由 `set_odomtwist()` 赋值；输出 child frame 为 `uav_name/point_lio_body`，不能未经外参验证等同 `base_link`，TF 默认关闭。确认 robot_localization Jazzy 是候选 EKF/UKF + navsat_transform 融合基线，但需处理 RTK quality/heading、协方差、IMU 相关性和 TF ownership。新增 `doc/Navigation/10_point_lio_decoupling_and_state_interface_audit.md` 并更新导航索引。仅 E1/E2 上游证据；未运行项目构建、bag 回放、Orin 或实机测试。

- 2026-11-25 / v0.3.82：补充追踪 CTU MRS reusable CI，确认下游 workflow 使用 `ctumrs/ros_jazzy:latest`，分别运行 AMD64 与 ARM64 build。Point-LIO 固定 commit `e7ff034328b263ab92b06252d91acba1dd969b6d` 的 run `24843451862` 两架构均成功；Estimator Plugin `b43af57300b43a29ab6bfa1c68a9442fb869cea1` 的 run `21216131865` 两架构均成功；MRS core run `33400579110` 两架构均成功，但其测试 SHA `0f3e329...` 早于当前审计头 `ef9de2a...`。修正先前“Jazzy/ARM64 matrix 未证实”的表述：上游已有 Jazzy AMD64/ARM64 CI 成功证据，但不是 Orin 实机，也不能证明当前 MRS core HEAD 的全部依赖树通过。进一步固定审计 Estimator Plugin CMake/package.xml，确认其依赖 `mrs_uav_managers`、`mrs_uav_state_estimators`、`mrs_lib`，证实与 MRS UAV estimator plugin 接口耦合。更新 09 审计文档和 TaskPlan。

- 2026-11-25 / v0.3.81：继续 N1，固定审计 Livox Driver 2 `21445540f0d100dc86a7e6df312dd70bbdb4afdf` 的 `lddc.cpp`，确认 CustomMsg `header.stamp` 与 `timebase` 同源于 `pkg.base_time`，每点 `offset_time` 为相对首点的纳秒偏移；PointXYZRTLT PointCloud2 timestamp 则写入每点绝对时间值，不能混用。进一步追踪 Point-LIO `callbackIMU()` / `sync_packages()`，确认 `time_lag_imu_to_lidar` 修正值用于 last_timestamp_imu gate，但入队和积分消费仍是原始消息时间戳，存在时间轴不一致风险。审计 MRS Point-LIO Core `ef9de2a55b43e492aec6e20bcab113973b9390a2`：ament metapackage 依赖 point_lio 与 estimator plugin；gitman 对 Point-LIO/Plugin/Driver 使用浮动 ros2 branch；CI 委托 reusable workflow，尚未确认 Jazzy/ARM64 matrix。明确 Point-LIO CustomMsg 输入需 Driver `xfer_format=1`。新增 `doc/Navigation/09_mid360_timestamp_and_point_lio_integration_audit.md` 并更新索引。未取得项目实际 bag，未运行构建/回放/Orin/实机验证；证据限 E1/E2。

- 2026-11-25 / v0.3.80：继续 N1，固定审计 `ctu-mrs/point-lio @ e7ff034328b263ab92b06252d91acba1dd969b6d` 与 `hku-mars/FAST_LIO @ 7cc4175de6f8ba2edf34bab02a42195b141027e9`。确认 CTU ROS2 Point-LIO 将 Livox CustomMsg offset_time 转为毫秒，按点时间推进/更新状态并将点变换到 world；其 body cloud 未做 scan-end deskew，world cloud 输出不保留逐点时间；IMU lag 参数疑似未贯穿同步消费端，CustomMsg.timebase 未显式读取。FAST-LIO2 官方实现执行 IMU 前向传播与反向逐点 scan-end deskew，但其官方仓库为 ROS1/catkin，不可直接用于 Jazzy。两项目 LICENSE 文件为 GPL-2.0 而 package.xml 声明 BSD，形成许可冲突；Point-LIO ROS2 fork 另依赖 MRS libs，Jazzy/ARM64 构建未证实。新增 `doc/Navigation/08_point_lio_fast_lio2_audit.md` 并更新专题索引。上述均为 E1/E2 上游证据；本项目未编译、未回放、未在 Orin/实机验证。

- 2026-11-25 / v0.3.79：继续在 nav 分支开展跨子系统数据链审计。固定 Livox ROS Driver 2 commit `21445540f0d100dc86a7e6df312dd70bbdb4afdf`，确认 MID-360 CustomMsg 含 timebase/逐点 offset_time，PointXYZRTLT PointCloud2 含逐点 timestamp；固定 elevation_mapping_cupy ROS2 branch commit `d9e1c9fe42ea3d463519333a443da99f680adde8`，追踪 PointCloud2 parser、整帧 header stamp TF 查询、XYZ-only 融合与 GridMap 发布，确认默认路径不会保留 Livox 逐点时间且本节点不做逐点 deskew；审计 grid_map Jazzy `d9614992e36c66d3fe67d9125ca105a0a47676fe` 的多层 2.5D 容器与转换能力，以及 legacy traversability_estimation `14d24c059e1c43466aadf328280adf6394d78039` 的 slope/roughness/step/normal/footprint 过滤概念。识别 Jetson Orin AGX + MID-360 的 elevation_mapping_cupy FPS 下降 issue #137 仍 open；v2.2.0 性能验证使用 RTX 4090，不能外推为 Orin 通过。新增 `doc/Navigation/07_mid360_elevation_traversability_audit.md` 并更新专题索引。仅 E1/E2 上游证据，未在本项目编译/运行/实机验证。

- 2026-11-25 / v0.3.78：继续 N1，固定 Navigation2 Jazzy commit `c7046550547d382a5186d106b0895eda02485d7c`，完成 Costmap current 状态从 LayeredCostmap 到 PlannerServer/ControllerServer 的消费端追踪。Planner 默认 `costmap_update_timeout=1.0s`，规划 Action 入口等待 global_costmap current，超时返回 TIMEOUT；Controller 默认 0.30s，在每个控制循环计算前等待 local_costmap current，超时发布零速度并以 CONTROLLER_TIMED_OUT 终止 FollowPath。明确 current Gate 不等于逐消息 freshness、动态障碍质量或实际底层停车确认；仿真时钟暂停/跳变仍需运行验证。新增 `doc/Navigation/06_costmap_health_and_consumers_audit.md`。仅 E2 上游源码证据，未运行 ROS/仿真/实机。

- 2026-11-25 / v0.3.77：继续 N1，追踪 Costmap2DROS 与 LayeredCostmap：Costmap2DROS 通过独立 std::thread + WallRate 更新；getRobotPose 失败时本周期不更新；update_frequency 与 publish_frequency 分离；LayeredCostmap 按配置顺序执行 plugin updateBounds/updateCosts，filters 在 plugin 之后处理；isCurrent() 对所有启用 plugins/filters 做 AND 聚合。补充指出 current 聚合存在不等于 Planner/Controller 每次决策都会检查，消费端调用点仍需源码核验。审计追加至 doc/Navigation/05_sensor_to_costmap_observation_audit.md，提交 6421d3d1。仅 E2 源码证据，未运行验证。

- 2026-11-25 / v0.3.76：继续 N1，固定 Navigation2 Jazzy commit c7046550547d382a5186d106b0895eda02485d7c，审计 ObstacleLayer/VoxelLayer/ObservationBuffer：确认 PointCloud2 原生输入、x/y/z 字段读取、TF MessageFilter、sensor-data QoS depth 50、marking/clearing、height/range filtering；确认 expected_update_rate=0 时 ObservationBuffer 不检查 stale；VoxelLayer 有限 z_voxels 最终仍投影为二维 Costmap；默认 YAML 的 local/global Costmap 和 Collision Monitor 仍使用 LaserScan /scan。新增 doc/Navigation/05_sensor_to_costmap_observation_audit.md，提交 834136a2。结论仅为 E1/E2 源码审计，不代表 MID-360/双目接入或运行验收。

- 2026-11-25 / v0.3.75：继续 N1，固定 Navigation2 Jazzy commit c7046550547d382a5186d106b0895eda02485d7c，源码追踪 NavigateToPoseNavigator、goal_pose 订阅、BT XML 选择、ComputePathToPose/FollowPath BT action client、PlannerServer 的 compute_path_to_pose/compute_path_through_poses action server、Global/Local Costmap2DROS 和独立 NodeThread；确认 FollowPath BT 节点可根据 blackboard path/plugin ID 变化更新 action goal。审计追加至 doc/Navigation/04_nav2_bringup_interface_audit.md，提交 f9d42d4a。仍属 E1/E2，不代表运行图或机器人执行验收。

- 2026-11-25 / v0.3.74：在 nav 分支完成 N1 中 Nav2 bringup 与运动命令链首轮源码审计：固定 navigation2 Jazzy commit c7046550547d382a5186d106b0895eda02485d7c；追踪 navigation_launch lifecycle nodes、controller/behavior→cmd_vel_nav→velocity_smoother→cmd_vel_smoothed→collision_monitor→cmd_vel；核验 Controller Server FollowPath server、odom/TF、costmap update timeout 和失败零速度路径；核验 Velocity Smoother 双 Twist 输入、OPEN_LOOP/CLOSED_LOOP 与 velocity timeout；核验 Collision Monitor source timeout 触发 STOP 及 LaserScan 示例源限制。新增 doc/Navigation/04_nav2_bringup_interface_audit.md，提交 4298dd67。只属于 E1/E2 源码证据，未运行 ROS 集成或实机测试。

- 2026-11-25 / v0.3.69：用户纠正研究目标，正式将范围从“Nav2 专项”扩展为“完整机器人导航模块”；Nav2 明确降为 N6 子系统。固定专项纪律：研究/文档/代码/测试仅在 nav 分支开展，N9 验收前不合并 main。新增 doc/Navigation/README.md、00_system_architecture_and_scope.md、01_end_to_end_dataflow_and_interfaces.md；重写 doc/Nav2/README.md 明确其子系统定位。当前属于 N0 研究框架，尚未完成源码级深研、ROS 集成或硬件验证。首轮提交：Nav2 README 042f4000；Navigation README 7234a0ae；架构文档 d4e862df；数据流文档 8db9087f；TaskPlan 目标纠正 a3dffc63、回填 291ee40f；N0 上游基线/术语/证据登记 424317c3；N1 ROS 接口契约 10b0bbd6；Navigation 目录索引更新后续提交。

- v0.1–v0.3.54：见 [TaskPlan-History-2026-11-24.md](docs/history/TaskPlan-History-2026-11-24.md)。
- v0.3.55–v0.3.66 完整快照：见 [TaskPlan-Snapshot-v0.3.66-2026-11-25.md](docs/history/TaskPlan-Snapshot-v0.3.66-2026-11-25.md)。
- 2026-11-25 / v0.3.68：Nav2 专项首轮研究文档已写入 doc/Nav2/（架构、生命周期/任务流、生产者消费者矩阵、Costmap、算法、硬件软件模块、四足安全、验证路线）；属于设计文档，不代表 ROS 集成/仿真/实机验收。提交：archive 5c7bc4b6；TaskPlan 初始保存 080d3931；Nav2 文档 commits 11d97882、98dba2c4、1ce5231a、41bad402、65290550、60342d48、b6034826、a53b5ea3、3c38c112。

---


- 2026-11-25 / v0.3.70–v0.3.72：完成 N0 范围/架构/术语/证据基线并进入 N1。新增上游仓库基线登记（Nav2 Jazzy c7046550、robot_localization jazzy-devel fea7fc3d、slam_toolbox Jazzy f84dad92、image_pipeline Jazzy 6c3df309、grid_map ROS 2 0ece1b53、ros2_control Jazzy c3f7a299）；记录 elevation_mapping 仓库归档状态、traversability_estimation 404 等风险，不将未验证项目列为依赖。固定 common_interfaces Jazzy commit a941f14b，核验 Imu/Odometry/PointCloud2/TwistStamped，以及 Nav2 NavigateToPose/FollowPath Action 字段与语义。新增 doc/Navigation/02_subsystem_inventory_and_evidence_register.md、03_ros_interface_contracts.md；README 索引提交 8a06637e；TaskPlan 本轮更新提交 09d21408。以上是源码/接口研究证据，不代表节点运行、构建或硬件验证。


- [~] N1 本轮新增 ZED-F9P UTC→GPS mapper 工程化设计（`doc/Navigation/14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md`）：将实现拆分为纯算法层（UTC civil/nano、GPS epoch/week、NAV-TIMELS typed model、exact-epoch mapper、clock identity）与后续 ROS driver adapter；明确 NAV-TIMELS 24-byte parser/ROSIDL/dispatch/publisher 插入点、process session/connection generation/queue sequence 有序 lifecycle fence、跨 callback group 线程安全和 fail-closed 条件。定义 GPS epoch/week 边界、负 nano 跨日历借位、2016 正闰秒连续性、NAV-TIMELS validity/source、PVT/TIMEUTC flags、exact iTOW/session join、reconnect/sequence gap/monotonicity 等 synthetic test vectors；二次核对 R07 原始 bitfield 图后确认 NAV-TIMELS bit 0=validCurrLs、bit 1=validTimeToLsEvent，bit 2..7 reserved；补充 flags=0x00/0x01/0x02/0x03 语义及 signed currLs/timeToLsEvent 的 byte-level raw fixtures；锁定 NAV-TIMEUTC valid flags mask 0x01/0x02/0x04、reserved 0x08、utcStandard 高四位；并按现有 frozen dataclass/显式异常风格定义纯算法 API 草案与 parser/normalizer/mapper 职责边界；新增 NAV-TIMEUTC byte 19 的 validTOW/validWKN/validUTC/utcStandard 独立 mask vectors，明确 raw decoder 与 source policy 分层。向量为规格，不是测试运行结果；未修改代码、未运行单测/CI、未读取真实接收机。目标 F9P MON-VER、NAV 输出配置、leap-second source allowlist/staleness 阈值和 package license/maintainer 仍为 Gate。
## 5. 文档索引

- [TaskPlan Part 1 — 架构与硬件完整设计](TaskPlan_Part-01_ArchitectureAndHardware.md)
- [TaskPlan Part 2 — 测试、风险与参考资料](TaskPlan_Part-02_TestingAndGovernance.md)
- [ADR-0001 — 传感器职责与定位架构](docs/decisions/ADR-0001-sensor-roles-and-localization.md)
- [ADR-0002 — 硬件平台选型](docs/decisions/ADR-0002-hardware-platform.md)
- [硬件接口确认清单](docs/interfaces/hardware_interface_checklist.md)
- [ROS 2 package 规划](docs/implementation/ros2_workspace_and_packages.md)
- [Lite3 bridge 接口契约](docs/implementation/lite3_bridge_interface.md)
- [Lite3 跨发行版遥测网关协议提案](docs/implementation/lite3_gateway_protocol.md)
- [R3–R5 实验原型归档与集成边界](docs/experimental/assistant_r5_reference/README.md)
- [Telemetry envelope validator](src/robodog_lite3_bridge/robodog_lite3_bridge/telemetry_contract.py)
- [Telemetry validator tests](src/robodog_lite3_bridge/tests/test_telemetry_contract.py)
- [GNSS RTK quality gate package](src/robodog_gnss_quality/README.md)
- [GNSS RTK quality gate core](src/robodog_gnss_quality/robodog_gnss_quality/quality_gate.py)
- [GNSS RTK quality gate tests](src/robodog_gnss_quality/test/test_quality_gate.py)
- [UBX-NAV-PVT 解码与 GNSS 历元配对](src/robodog_gnss_quality/robodog_gnss_quality/ubx_nav_pvt.py)
- [UBX-NAV-PVT 配对测试](src/robodog_gnss_quality/test/test_ubx_nav_pvt.py)
- [GNSS quality gate CI](.github/workflows/gnss-quality-ci.yml)
- [u-blox 四路 UBX 有界历元缓存器](src/robodog_gnss_quality/robodog_gnss_quality/ubx_epoch_joiner.py)
- [u-blox 四路 UBX 缓存器测试](src/robodog_gnss_quality/test/test_ubx_epoch_joiner.py)
- [ZED-F9P ROS 驱动评估与 iTOW 适配契约](docs/implementation/zed_f9p_driver_evaluation.md)
- [u-blox ZED-F9P driver Humble/Jazzy build CI](.github/workflows/ublox-dgnss-driver-ci.yml)
- [Lite3 厂商 SDK 集成路径评估](docs/implementation/lite3_vendor_sdk_integration_assessment.md)
- [ADR-0003：Lite3 ROS 集成路径](docs/decisions/ADR-0003-lite3-ros-integration.md)
- [机载供电与载荷预算](docs/implementation/power_mass_budget.md)
- [双目光学、同步与 RTK 接入基线](docs/implementation/stereo_optics_and_rtk_baseline.md)
- [Lite3 公开 SDK 字段与安全审计](docs/implementation/lite3_public_sdk_field_audit.md)
- [MID-360 Livox 驱动锁定基线](docs/implementation/livox_mid360_driver_baseline.md)
- [MID-360 rosbag 时间审计 CLI](scripts/audit_livox_rosbag_timing.py)
- [MID-360 rosbag 时间审计 helper/单测](scripts/livox_time_audit_helpers.py)
- [Point-LIO Jazzy POC 来源与验收基线](docs/implementation/point_lio_jazzy_poc_baseline.md)
- [Point-LIO 固定源准备脚本](scripts/prepare_point_lio_intel_ros2.sh)
- [Point-LIO Jazzy 构建脚本](scripts/build_point_lio_jazzy.sh)
- [UrbanLoco P2 数据准备说明](datasets/README.md)
- [外部公开数据集 SHA256 清单](datasets/external_sha256.txt)
- [外部数据集文件 SHA256 验证脚本](scripts/verify_public_dataset_checksum.sh)
- [外部数据集 SHA256 验证测试](scripts/tests/test_verify_public_dataset_checksum.sh)
- [M3DGR Dynamic03 ROS1 bag 只读审计工具](scripts/audit_m3dgr_ros1_bag.py)
- [数据集审计工具依赖](requirements-dataset-tools.txt)
- [M3DGR ROS1 bag 审计测试](scripts/tests/test_m3dgr_ros1_bag.py)
- [M3DGR ROS1→ROS2 显式 Livox 类型转换器](scripts/convert_m3dgr_dynamic03_to_ros2.py)
- [M3DGR ROS1→ROS2 转换集成测试](scripts/tests/test_convert_m3dgr_dynamic03_to_ros2.py)
- [M3DGR TUM GT 只读审计工具](scripts/audit_tum_trajectory.py)
- [M3DGR TUM GT 审计测试](scripts/tests/test_tum_trajectory_audit.py)
- [外部公开数据集替代路线（GrandTour / QuadSLAM / M3DGR）](docs/datasets/alternative_dataset_strategy.md)
- [UrbanLoco GT 提取脚本](scripts/extract_urbanloco_ulhk4_gt.py)
- [UrbanLoco P2 回放脚本](scripts/run_urbanloco_ulhk4.sh)
- [UrbanLoco P2 evo 评测脚本](scripts/evaluate_urbanloco_ulhk4.sh)
- [UrbanLoco P2 测试工作流](.github/workflows/point-lio-urbanloco-p2-tests.yml)
- [导航与避障验收指标框架](docs/testing/acceptance_metrics_and_scenarios.md)
- [Lite3 bridge package 说明](src/robodog_lite3_bridge/README.md)
- [多传感器时间同步与标定](docs/implementation/time_sync_and_calibration.md)
- [验收测试矩阵](docs/testing/acceptance_test_matrix.md)
- [完整导航模块专项研究总目录](doc/Navigation/README.md)
- [完整导航模块总体架构与边界](doc/Navigation/00_system_architecture_and_scope.md)
- [Nav2 bringup 到运动命令链接口审计](doc/Navigation/04_nav2_bringup_interface_audit.md)
- [传感器观测到 Nav2 Costmap 输入链审计](doc/Navigation/05_sensor_to_costmap_observation_audit.md)\n- [Costmap 健康状态与 Planner/Controller 消费端审计](doc/Navigation/06_costmap_health_and_consumers_audit.md)
- [端到端数据流与接口审计框架](doc/Navigation/01_end_to_end_dataflow_and_interfaces.md)
- [导航子系统清单、上游基线与证据登记](doc/Navigation/02_subsystem_inventory_and_evidence_register.md)
- [ROS 2 核心接口契约与数据语义](doc/Navigation/03_ros_interface_contracts.md)
- [Nav2 子系统研究总目录](doc/Nav2/README.md)
- [系统架构](docs/architecture/system_architecture.md)
- [传感器融合](docs/architecture/sensor_fusion.md)
- [Nav2 导航和避障](docs/navigation/nav2_navigation.md)
- [四足地形/安全验证](docs/validation/safety_and_validation.md)
- [双目 RGB 深度感知](docs/perception/stereo_obstacle_perception.md)

- [TaskPlan v0.3.66 基线快照](docs/history/TaskPlan-Snapshot-v0.3.66-2026-11-25.md)
