# RobDog Nav2 项目总任务计划（唯一进度台账）

- 文档版本：v0.3.16
- 更新日期：2026-11-24
- 默认分支：`main`
- 仓库：https://github.com/rexyll/robdog_nav2
- 项目目标：以四足机器人为载体，使用 **LiDAR + 两台普通 RGB 相机（被动双目）+ RTK GNSS**，结合机器狗 IMU 与经验证的本体状态估计（厂商腿式 odometry 输出待确认），构建 ROS 2 / Nav2 全局导航、局部实时避障和四足地形可通行性能力。
- **本体技术预选：DEEP Robotics Lite3 Venture**（未采购，待供应商确认）。
- **传感器预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）；ArduSimple simpleRTK2B Budget / u-blox ZED-F9P。**
- **机载计算预选：NVIDIA Jetson AGX Orin Developer Kit。**
- 当前阶段：**阶段 A — 硬件基线与接口冻结**
- 当前状态：**方案预选完成；采购、实物测量、驱动编译和机器狗实机验证未完成。**

---

## 0. TaskPlan 管理规则（必须遵守）

1. **根目录 `TaskPlan.md` 是项目唯一进度真源。** 所有任务状态、阶段进度、完成证据、阻塞项、决策变更、测试结果和下一步动作必须写入本文件。
2. 其他文档只保存详细设计、接口说明、ADR、测试报告和稳定参考资料；不得只更新其他文档或对话而不回写本文件。
3. 每次推进必须更新本文件中的：当前阶段、变更任务状态、完成证据、Git commit SHA、测试/验证方式、风险/未决项、下一步动作。
4. 文档/设计完成不等于代码实现；仿真通过不等于真实机器狗通过；只有证据足够才标记任务完成。
5. 默认只在 `main` 推进；没有明确要求不创建新分支。
6. 状态标记：`[ ]` 未开始；`[~]` 实施中/部分完成；`[x]` 已完成且有证据；`[!]` 阻塞或风险。
7. 本文件超过约 30 KB 时按主题拆成 `TaskPlan_Part-*.md`。**拆分后仍必须把所有进度状态保留在本根文件**；分册只放详细设计，不放需要维护的重复进度清单。
8. 未知传感器参数、供电电流、质量、时延、速度、协方差和准确度一律标“待确认/待测”，不猜测、不照抄示例参数。

## 1. 项目目标、硬件基线与系统决策

### 1.1 目标
使用四足机器狗 + 3D LiDAR + 两台普通 RGB 相机（被动双目）+ RTK GNSS，结合机器人 IMU/经验证的本体状态估计，基于 ROS 2 / Nav2 实现室内外导航、局部实时避障、动态障碍感知和后续四足地形可通行性。

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

### 阶段 A — 硬件基线与接口冻结（当前阶段）

- [x] A1. 完成本体和外置导航传感器技术预选：Lite3 Venture、MID-360、双 See3CAM_50CUG_CXXCC_BX_H01R1、simpleRTK2B/ZED-F9P、AGX Orin。
- [x] A2. 核对云深处 Lite3 Venture + AGX Orin + MID-360 官方扩展示例与初始软件版本。
- [ ] A3. 向云深处确认 Venture 当前 SKU、交付件、固件/SDK、安装 CAD、外设接口持续/峰值电流、载荷与重心限制。
- [~] A4. 已建立初步供电/质量预算：已核实器件 1.330 kg；若采用 CIL530 双镜头，候选小计约 1.664 kg，余 1.936 kg 供相机本体和附件。两台相机本体、交换机、触发板、支架/线束、DC/DC 尚未称重/测功；Lite3 各路电源额定仍待书面确认。
- [~] A5. 已核对 MID-360 公开规格，并锁定驱动候选 livox_ros_driver2 v1.2.6 commit 13eb05e4e6dd7a765b934d0c5fd6236676a57b49、Livox-SDK2 候选 commit c0796f04c143143899c87a773d9f6b7136453c0b；实际设备 revision、SDK/driver ARM64 构建、线束与网络参数仍待确认。
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
- [~] B3. 已实现 transport-agnostic envelope/sequence 基础校验，并新增只读 contact_state 消息类型；IMU/contact_state/base_odometry 等空间消息必须带 frame_id。仍未实现编码器、UDP/TCP socket、vendor mapping 或 real gateway。
- [~] B4. 已锁定 Livox ROS Driver2 v1.2.6 与 SDK2 候选 commit，并新增 Jazzy x86_64 build workflow（run 45332455019，当前运行中）；Orin aarch64 与真实 MID-360 仍待验证。使用 CustomMsg 保留每点时间供 Point-LIO，禁止先转换为 PointXYZI。详见 [MID-360 驱动基线](docs/implementation/livox_mid360_driver_baseline.md)。
- [ ] B5. 接入左右 UVC 相机和 GPIO 硬件触发，验证双路 USB3 带宽。
- [ ] B6. 接入 ZED-F9P，完成 NavSatFix/UBX 状态和 NTRIP/基站。
- [ ] B7. 建立 URDF/TF，完成相机内外参、双目基线、LiDAR/GNSS 外参。
- [ ] B8. 测量 PTP/GPS/相机触发/UDP 的同步误差、时延和抖动。
- [ ] B9. 录制完整 rosbag2，建立回放和 diagnostics。
- [ ] B10. 所有 topic、frame_id、QoS、频率、数据年龄和 TF 检查通过。
- [ ] B11. 固定 Point-LIO ROS 2 Jazzy port/fork、patch 和依赖 commit；Livox driver 预选 v1.2.6（13eb05e4e6dd7a765b934d0c5fd6236676a57b49），SDK2 预选 c0796f04c143143899c87a773d9f6b7136453c0b。Point-LIO fork 尚未冻结；先审许可证并以公开 bag 离线 build/replay，目标 Orin/MID-360 实测仍未完成。

### 阶段 C — 状态估计与 RTK 全局定位

- [ ] C1. 核实当前 sdk_deploy 是否提供可信 legged odometry/body velocity；若只提供 IMU/joint/status，启动 Point-LIO + MID-360 内置 IMU 备用 POC；不得用 IMU-only 积分冒充可信位置。
- [ ] C2. 标定 MID-360 LiDAR/内置 IMU 外参、设备时间戳/PTP 和每点 offset_time；验证 Point-LIO 所需点云与 IMU 同步。
- [ ] C3. 比较 vendor legged odom（若有）与 Point-LIO 的位置/航向误差、漂移、更新频率、延迟和资源开销；选定唯一主要 local odom 来源。
- [ ] C4. 将通过验收的 local odom 表达为 `odom→base_link`；仅在有额外独立观测且 covariance 合理时再配置 local EKF，避免重复融合同一 IMU/LIO 信息。
- [ ] C5. 配置 navsat_transform（ENU/UTM、datum、yaw offset、磁偏角、GNSS 天线杆臂）。
- [ ] C6. 配置 GNSS quality gate 与 global EKF 输出 `map→odom`。
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

### 阶段 E — Nav2 平地导航和实时避障

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
- [~] 完成网关协议 v0.1 语义草案及 envelope/sequence/消息类型/frame/数值/covariance 检查；新增 contact_state 必须携带 frame_id 的验证和 2 个单元测试，Humble/Jazzy 37 项复跑进行中（run 45332714322）。
- [x] 建立供电/载荷初步预算，依据厂商公开功率/质量列出已知小计和未确认项目。
- [x] 完成本体和传感器技术预选。
- [x] 核对云深处 Lite3 Venture + AGX Orin + MID-360 的官方参考配置。
- [x] 核对 e-con Systems See3CAM 彩色无镜头 SKU 与官方 Trigger / Global Shutter / USB3 规格。
- [x] 核对 JetPack 7.2.2 对 Jetson Orin 的 Ubuntu 24.04 基线，并据此将 Jazzy 设为 Orin 当前长期目标；记录 Lyrical 的 Ubuntu 26.04 前置条件。
- [x] 创建 ADR-0002 硬件选型记录和更新硬件接口清单。
- [x] TaskPlan 拆分为根目录进度台账 + 两份设计分册。
- [x] 更新 Lite3 bridge 接口契约，区分 Legacy standard ROS topics 与 current DRDDS / rl_deploy interfaces；记录 vendor odometry availability 为待确认。

### 未完成 / 阻塞
- [x] ROS 2 package CI 最新复跑：[run 45328982630](https://github.com/rexyll/robdog_nav2/actions/runs/45328982630)，Humble/Jammy 与 Jazzy/Noble 均 build 成功、35 passed，0 errors/failures/skipped；仅覆盖 mock/契约逻辑，不覆盖真实 SDK/传感器。
- [x] 已在本地 Python 环境对纯 freshness helper 的 6 个单元用例运行 unittest，6/6 通过；这不替代 ROS 2 package CI 或真实硬件验证。
- [ ] 未采购设备，未接收实物。
- [ ] Point-LIO/FAST-LIO2 尚未在本项目 Jetson AGX Orin + MID-360 上构建和验证；Intel 文档中的 Jazzy port/Velodyne 数据集测试不能替代目标硬件验收。
- [ ] 质量预算中仍有相机/镜头、网络交换机、触发板、DC/DC、支架和线束未称重；3.6 kg 内部载荷目标尚未验证。
- [ ] 尚未获得 Lite3 Venture 电源额定、载荷/重心和安装 CAD 的书面确认；因此不能批准从机器人 24V 辅助口直接给 AGX Orin 供电。
- [!] 当前公开 RobotStatus 不含 base xyz/linear velocity/covariance；尚不能排除厂商存在未公开 estimator channel，需书面确认正式腿式 odometry/body velocity。Foxy/Jazzy 仍须隔离。
- [ ] 双目精确 SKU 已明确，但 GPIO 触发电压/极性/时序、镜头、双机硬件同步和 Jetson 双路 USB3 带宽尚待供应商确认。
- [ ] RTK NTRIP 改正服务或基站尚未确定。
- [ ] 载荷、供电、热设计、质心和续航未实测；质量预算还缺相机本体、交换机、触发板、DC/DC、支架和线束，功率预算还缺转换损耗与峰值测量。
- [ ] 厂商 Lite3 SDK、MID-360 驱动、双目相机驱动和 RTK 驱动尚未在 Jetson 真实硬件上编译；尚无传感器 rosbag 和 Nav2 实机结果。
- [ ] Lite3 mock topic 骨架不是实际 SDK bridge，不可接入 robot_localization 或运动控制；真实 SDK/UDP 数据适配仍待目标固件验证。
- [ ] gateway wire codec、UDP/TCP、vendor source mapping 均未实现；35 项纯逻辑测试仍不代表实际网络链路/厂商 SDK 验证。Covariance 检查只验证长度、有限值和对角线非负性，不证明完整矩阵 PSD。
- [x] 新增消息类型/frame/covariance 检查后的最终 CI 已通过；失败 run 45328900700 根因为 `status` 未加入 message_type 白名单，修复后 45328937159 与 45328982630 双发行版均通过。
- [ ] 未确认的 legged odometry 是定位架构的实质性阻塞项：如果 vendor SDK 不提供可信本体 odom，需转向经验证的 LiDAR odometry/SLAM 或 stereo VIO。
- [!] lite3_transfer 启动/SDK_MODE 可能获取 SDK 控制权；未获得厂商只读 bring-up SOP、物理急停及遥控接管说明前，不在实机启动 SDK mode、不发送关节/速度命令。
- [ ] 时间同步方案已文档化；双目硬件触发、MID-360 PTP/内置 IMU 时钟、Lite3 UDP 时间戳、ZED-F9P PPS 均未测量。

### 下一步（按顺序执行）
1. 按供应商矩阵 V-06/V-13/V-16/V-19/V-21/V-22/V-28–V-32 向云深处取得正式接口、控制权、物理急停、电源、载荷与 CAD 书面答复。
2. 向相机供应商确认两台 See3CAM_50CUG_CXXCC_BX_H01R1 的 GPIO 触发参数和匹配广角镜头。
3. 确认 RTK 的 NTRIP/基站来源和部署测试区域。
4. 继续完成质量/功耗/散热预算和初步安装 CAD，核实 3.6 kg 项目外设目标；重点获取 Lite3 每路辅助电源的额定电流。预算细节见 power_mass_budget.md。
5. Livox 按锁定 commit 先完成 Jazzy x86_64 build，再完成 Orin aarch64 build；Build pass 前不能标记 B4 完成。
6. 继续完成 A3–A10、A12 供应商确认和质量/电源/时间同步预算；A11 发行版策略已形成文档，实际 JetPack/Humble/Jazzy 构建矩阵待验证。
7. Lite3 公开 SDK 审计已写入 [lite3_public_sdk_field_audit.md](docs/implementation/lite3_public_sdk_field_audit.md)；取得厂商只读启动 SOP 和状态接口/网络契约后，再实现 readonly gateway POC。

---

## 4. 进度变更记录

历史记录 v0.1–v0.3.4 已归档至 [TaskPlan-History-2026-11-24.md](docs/history/TaskPlan-History-2026-11-24.md)。本表只保留近期影响当前基线的变更。

| 日期 | 版本 | 变更 | 提交/证据 |
|---|---|---|---|
| 2026-11-24 | v0.3.12 | 更新 MID-360、双目镜头/同步/USB 带宽、RTK 接入基线；核实 35/35 双发行版 CI 成功 | CI 45328982630；详见 stereo optics / RTK baseline |
| 2026-11-24 | v0.3.13 | 将两只 CIL530 候选镜头合计 334g 计入质量预算，候选总计调整为 1.664kg；其余未称重项和厂商载荷 Gate 保持未完成 | power_mass_budget.md commit 2ec892ebc6b44b789a47d792e7d555a7f7c302d9 |
| 2026-11-24 | v0.3.14 | 审计 Lite3 公共 SDK 状态字段/启动控制权/软件急停边界；补充 V-28–V-32 厂商问题；锁定 MID-360 Jazzy driver 与 SDK2 候选 commit，B4 build 保持待验证 | lite3 audit f9f8cec2; vendor matrix 90d7ab86; Livox baseline f5803ad7 |
| 2026-11-24 | v0.3.15 | 建立导航/避障验收指标和测量方法框架，A10 部分完成；新增 Livox Jazzy x86_64 build workflow，仍需等待结果并完成 Orin ARM64 验证 | acceptance metrics 30a81088; Livox CI run 45332455019 |
| 2026-11-24 | v0.3.16 | 将 Lite3 足端接触力纳入只读 telemetry envelope；contact_state 必须带 frame_id，新增 2 项测试；Humble/Jazzy 37 项 CI 待结果 | contract 3341cc7b; tests fb463763; protocol 440ee4a4; CI 45332714322 |

---

## 5. 文档索引

- [TaskPlan Part 1 — 架构与硬件完整设计](TaskPlan_Part-01_ArchitectureAndHardware.md)
- [TaskPlan Part 2 — 测试、风险与参考资料](TaskPlan_Part-02_TestingAndGovernance.md)
- [ADR-0001 — 传感器职责与定位架构](docs/decisions/ADR-0001-sensor-roles-and-localization.md)
- [ADR-0002 — 硬件平台选型](docs/decisions/ADR-0002-hardware-platform.md)
- [硬件接口确认清单](docs/interfaces/hardware_interface_checklist.md)
- [ROS 2 package 规划](docs/implementation/ros2_workspace_and_packages.md)
- [Lite3 bridge 接口契约](docs/implementation/lite3_bridge_interface.md)
- [Lite3 跨发行版遥测网关协议提案](docs/implementation/lite3_gateway_protocol.md)
- [Telemetry envelope validator](src/robodog_lite3_bridge/robodog_lite3_bridge/telemetry_contract.py)
- [Telemetry validator tests](src/robodog_lite3_bridge/tests/test_telemetry_contract.py)
- [Lite3 厂商 SDK 集成路径评估](docs/implementation/lite3_vendor_sdk_integration_assessment.md)
- [ADR-0003：Lite3 ROS 集成路径](docs/decisions/ADR-0003-lite3-ros-integration.md)
- [机载供电与载荷预算](docs/implementation/power_mass_budget.md)
- [双目光学、同步与 RTK 接入基线](docs/implementation/stereo_optics_and_rtk_baseline.md)
- [Lite3 公开 SDK 字段与安全审计](docs/implementation/lite3_public_sdk_field_audit.md)
- [MID-360 Livox 驱动锁定基线](docs/implementation/livox_mid360_driver_baseline.md)
- [导航与避障验收指标框架](docs/testing/acceptance_metrics_and_scenarios.md)
- [Lite3 bridge package 说明](src/robodog_lite3_bridge/README.md)
- [多传感器时间同步与标定](docs/implementation/time_sync_and_calibration.md)
- [验收测试矩阵](docs/testing/acceptance_test_matrix.md)
- [系统架构](docs/architecture/system_architecture.md)
- [传感器融合](docs/architecture/sensor_fusion.md)
- [Nav2 导航和避障](docs/navigation/nav2_navigation.md)
- [四足地形/安全验证](docs/validation/safety_and_validation.md)
- [双目 RGB 深度感知](docs/perception/stereo_obstacle_perception.md)
