# RobDog 完整导航模块项目总任务计划（唯一进度台账）

- 文档版本：v0.3.79
- 更新日期：2026-11-25
- 默认分支：`main`
- 仓库：https://github.com/rexyll/robdog_nav2
- 项目目标：以四足机器人为载体，研究、设计并最终集成完整的 ROS 2 导航模块，覆盖任务管理、传感器接入与时间同步、定位/状态估计/SLAM、环境感知、地图与地形表达、可通行性、全局/局部规划、动态避障、Nav2（仅为其中一个子系统）、四足运动接口、独立安全、运行时部署、监控和分层验证。
- **本体技术预选：DEEP Robotics Lite3 Venture**（未采购，待供应商确认）。
- **传感器预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）；ArduSimple simpleRTK2B Budget / u-blox ZED-F9P。**
- **机载计算预选：NVIDIA Jetson AGX Orin Developer Kit。**
- 当前专项阶段：**N1 — 端到端数据流与接口审计（仅 nav 分支）**；N0 范围/架构/术语/证据基线已冻结；硬件准备的阶段 A 仍保持原状态并行受 Gate 约束。
- 当前状态：**N0 已完成；N1 已建立系统级数据流清单，并完成 Nav2 主链、Costmap 生产端与 Planner/Controller 消费端的首轮固定源码闭环；已对 MID-360 逐点时间数据、ROS 2 Jazzy 高程地图候选和四足地形层开展上游审计。N1 的其余接口/运行证据仍在补齐；采购、目标 Orin 联调、实机验证均未完成。**

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
- [~] N1 端到端数据流与接口审计：已建立系统级 producer/consumer 清单与 ROS 2 Jazzy 消息/action 契约；完成 Nav2 bringup→BT→Planner/Controller→Smoother→Collision Monitor 主链源码审计。Costmap 输入链详见 `doc/Navigation/05_sensor_to_costmap_observation_audit.md`；Costmap current 到 Planner/Controller 的等待/超时消费闭环详见 `doc/Navigation/06_costmap_health_and_consumers_audit.md`（Planner 默认 1.0s，Controller 默认 0.30s；超时路径已源码核验）。本轮增加 MID-360→deskew→elevation/grid_map→traversability 的跨子系统接口审计，详见 `doc/Navigation/07_mid360_elevation_traversability_audit.md`：固定 Livox Driver 2、elevation_mapping_cupy ROS2 branch、grid_map Jazzy 与传统 traversability_estimation 参考版本；识别出 elevation_mapping_cupy 默认 XYZ 输入会丢弃 Livox per-point timestamp/tag/line，且整帧按 header stamp 查询一次 TF，因此必须先完成逐点时间补偿再输入；Jetson Orin + MID-360 FPS 下降 issue #137 仍 open，v2.2.0 RTX 4090 性能数据不能外推到 Orin。仍未完成项目实际 adapter、驱动/SDK/ARM64 构建、运行 QoS/频率/延迟、LIO/deskew、Lite3 执行反馈与实机验证；当前证据为 E1/E2 上游资料/源码，不是项目运行证据。
- [ ] N2 传感器驱动、时钟同步与标定：MID-360、双目、IMU、ZED-F9P、Lite3 遥测；驱动源码/协议、PTP/PPS/硬件触发、时钟域转换、外参/内参、数据记录与健康监测。
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

### 下一步（按顺序执行）
1. 严格在 `nav` 分支执行 N0→N9：N0 范围/架构/术语/证据基线已冻结；N1 已完成 Nav2 主任务链首轮源码审计（NavigateToPose→BT→ComputePathToPose/Planner→FollowPath/Controller→Smoother→Collision Monitor），继续追踪 Costmap 观测源和非 Nav2 上下游接口；Nav2 的完整内部深研仍归 N6。
2. N1 将系统级 Producer/Consumer 清单细化为源码/消息定义可验证的接口矩阵，优先检查传感器→定位/感知→地图/Costmap→规划→命令仲裁→四足执行反馈，避免只研究孤立软件包。
3. 并行保留原硬件 Gate：推进 ZED-F9P ROS adapter/UBX 时钟与质量 gate、Lite3 正式 odometry/运动接口/急停/SDK 权限确认、双目同步/镜头/USB 带宽、载荷/供电/热设计。
4. 保持 Dynamic03 GT provenance gate：不排序/去重原始 GT，不将 2,594 行候选用于 APE/RPE；在 RTK/GT 来源、时钟、天线杆臂和 frame semantics 证据闭环前，不运行或宣称精度验收。
5. 保持 x86_64、通用 ARM64 build、Jetson Orin 实机 build、仿真、真实 bag 回放、HIL、真实机器狗验证分级；不得混称。
6. N9 专项验收前不回 main、不合并本专项成果；完成后先做差异/风险/CI/回归审查，再形成回 main 的提案。

---

## 4. 进度变更记录

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
