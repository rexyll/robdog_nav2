# RobDog Nav2 项目总任务计划（唯一进度台账）

- 文档版本：v0.3.63
- 更新日期：2026-11-25
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
- [!] lite3_transfer 启动/SDK_MODE 可能获取 SDK 控制权；未获得厂商只读 bring-up SOP、物理急停及遥控接管说明前，不在实机启动 SDK mode、不发送关节/速度命令。
- [ ] 时间同步方案已文档化；双目硬件触发、MID-360 PTP/内置 IMU 时钟、Lite3 UDP 时间戳、ZED-F9P PPS 均未测量。

### 下一步（按顺序执行）
1. 为 ZED-F9P 明确实际 ROS 2 driver/UBX 输出配置；实现 NAV-PVT GNSSFixOK/carrSoln 与 NavSatFix 的同 GNSS epoch 配对及 clock-epoch 处理，再接入 quality gate；仅用实测 covariance、输出周期和时间延迟确定门限。
1. 按供应商矩阵 V-06/V-13/V-16/V-19/V-21/V-22/V-28–V-32 向云深处取得正式接口、控制权、物理急停、电源、载荷与 CAD 书面答复。
2. 向相机供应商确认两台 See3CAM_50CUG_CXXCC_BX_H01R1 的 GPIO 触发参数和匹配广角镜头。
3. 确认 RTK 的 NTRIP/基站来源和部署测试区域。
4. 继续完成质量/功耗/散热预算和初步安装 CAD，核实 3.6 kg 项目外设目标；重点获取 Lite3 每路辅助电源的额定电流。预算细节见 power_mass_budget.md。
5. 保持 Livox/Point-LIO 的 x86_64 构建基线固定；为 Jetson AGX Orin aarch64 建立同版本交叉编译/原生构建验证，不能把 x86_64 pass 当作 Orin pass。
6. 继续完成 A3–A10、A12 供应商确认和质量/电源/时间同步预算；A11 发行版策略已形成文档，实际 JetPack/Humble/Jazzy 构建矩阵待验证。
7. 真实 Dynamic03 转换已通过：ros2 bag info 显示 59,749 条 CDR 消息（LiDAR 2,846、IMU 56,903）。GT↔raw-fix 逐条来源关联已获得强证据（时间戳最大差约 0.000712ms；诊断刚体拟合 RMSE 0.044mm），但 NavSatFix status=0 仅是普通未增强 STATUS_FIX，covariance_type=1 为近似协方差，RTK Fixed/GT 精度仍未认证。下一步 `git pull --ff-only origin main` 后重跑扩展版 `audit_m3dgr_gt_rtk_correspondence.py`，检查 NavSatFix frame/service/covariance 数值以及 `/ublox_driver/receiver_lla`、`receiver_pvt` 的真实类型/字段/首尾样本；不得做 nearest-time 猜配。依据字段确认 UBX fix flags、carrier solution、iTOW、accuracy/covariance、differential correction age 与 topic-device mapping，再确定 RTK 质量 Gate。原始 GT 不排序/去重；2,594 行候选不用于来源判断或 APE；不跑 evo、不做 Point-LIO 轨迹精度验证，直到来源质量、时钟、天线杆臂和 frame semantics 经证据确认。UrbanLoco P2 单独保留 provenance gate.

---

## 4. 进度变更记录

历史记录 v0.1–v0.3.54 已归档至 [TaskPlan-History-2026-11-24.md](docs/history/TaskPlan-History-2026-11-24.md)。本表只保留最近影响当前基线的变更。

| 日期 | 版本 | 变更 | 提交/证据 |
|---|---|---|---|
| 2026-11-24 | v0.3.55 | 新增 TUM GT 只读审计及 7 项测试；固定 M3DGR Livox ROS1 `offset_time`=ns/timebase/header 语义来源；确认 ROS1 `livox_ros_driver/CustomMsg` 与 Jazzy Point-LIO `livox_ros_driver2/msg/CustomMsg` 类型不同，记入显式类型映射/重序列化门禁；bag 5/5、GT 7/7、UrbanLoco 13/13 CI [45416922661](https://github.com/rexyll/robdog_nav2/actions/runs/45416922661) success | TUM audit 0f42bcc5d1a5be5664696957fd187c1f7045b5d3; tests 0627d2afd732b5849c1e3e660a67550fdfc8bdb5 / 59b84dba906e744b1f049ab918537671ee63af94; Livox audit 23c334be6377bf19ead94f39d636a876cd98c819; docs f3ef67656deb932633957c7b13c825ba4ba381cb / cc107912a11d4bb46379555870012ac54f95dee2; CI 45416922661 |
| 2026-11-24 | v0.3.56 | 新增 M3DGR Dynamic03 受控 ROS1→ROS2 转换器：官方 SHA256 + matching audit JSON 双门禁；合成转换/回读 4/4；真实 bag/GT 尚待审计 | converter/test/docs/CI commits ... | 
| 2026-11-24 | v0.3.57 | 真实 Dynamic03 bag audit 通过，发现 MID-360 Driver2 类型；GT 4,269 行 TUM 格式但有 1,675 duplicate/728 backwards，阻断 evo | converter 97e9211ace2c83dce6ca1cc1f357a307a0669f73; tests b36e365a6ccb9c17c3aad7be07f049ba41a6758b; docs 96fabf895242f75a39c3e3393542d6d7bd41e771; CI 45423779787 |
| 2026-11-24 | v0.3.58 | TUM audit 增加位姿/时间戳异常关联统计；CI [45424213139](https://github.com/rexyll/robdog_nav2/actions/runs/45424213139) success | audit 9b67ed712d4618f0551315921313bca6d65154ea; tests e6f0a4fdef2fc3aa4bd53a7a448ad7ff58262771; docs 3a66a48e14b35369f76c0f4a0d382096617685a1
| 2026-11-24 | v0.3.59 | 新增 TUM pose-run non-authoritative candidate generator，6/6 测试；CI 45425579274 success | generator f5d1ef3 / fe362365; tests 199f8229; workflow 34d29c5a; CI 45425579274 |
| 2026-11-24 | v0.3.60 | 真实 Dynamic03 ROS2 转换/ros2 bag info 通过；新增只读逐行 GT↔raw-fix 时间/ENU 关联审计，CI [45427945207](https://github.com/rexyll/robdog_nav2/actions/runs/45427945207) success | audit 96303cb6; tests 4cdf5170; workflow 3c78e0dc; docs 823f4a30; CI 45427945207 |
| 2026-11-25 | v0.3.61 | 真实 GT↔raw-fix 逐条时间/空间来源关联获得强证据；纠正 NavSatStatus=0 为普通 STATUS_FIX；扩展 frame/service/covariance 和两个 u-blox topic 的消息字段快照审计，新增回归测试；RTK Fixed/GT 精度仍未认证 | audit/tool+tests/docs/TaskPlan; CI pending |

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
- [系统架构](docs/architecture/system_architecture.md)
- [传感器融合](docs/architecture/sensor_fusion.md)
- [Nav2 导航和避障](docs/navigation/nav2_navigation.md)
- [四足地形/安全验证](docs/validation/safety_and_validation.md)
- [双目 RGB 深度感知](docs/perception/stereo_obstacle_perception.md)
