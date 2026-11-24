# RobDog Nav2 项目总任务计划（唯一进度台账）

- 文档版本：v0.2.1
- 更新日期：2026-11-24
- 默认分支：`main`
- 仓库：https://github.com/rexyll/robdog_nav2
- 项目目标：以四足机器人为载体，使用 **LiDAR + 两台普通 RGB 相机（被动双目）+ RTK GNSS**，结合机器狗原生 IMU/腿式里程计，构建 ROS 2 / Nav2 全局导航、局部实时避障和四足地形可通行性能力。
- **本体技术预选：DEEP Robotics Lite3 Venture**（未采购，待供应商确认）。
- **传感器预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG Color；ArduSimple simpleRTK2B Budget / u-blox ZED-F9P。**
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

## 1. 项目目标与首选硬件

### 1.1 首选本体

**DEEP Robotics 绝影 Lite3 Venture。**

选择原因：
- 云深处官方 Lite3 Venture 扩展感知示例使用 AGX Jetson Orin + Livox Mid-360，给出了机械安装、24V XT30 供电和 Ethernet 接线方式；
- Venture 公开标称 4.5 kg 载荷，具备 2× Ethernet 与 24V/12V/5V 外部电源接口；
- 有腿式里程计、IMU、关节状态和基于 Twist 的速度控制 SDK 路径；
- 不需要本项目从零开发四足关节驱动和步态底层。

公开参数约 12.2 kg 本体、4.5 kg 载荷、610×370×445 mm；公开空载续航约 1.5–2 小时。所有参数需按目标 SKU 和交付批次复核，外设搭载后的续航必须实测。

### 1.2 首选外置设备 BOM

| 子系统 | 预选型号 | 数量 | 接口/用途 |
|---|---|---:|---|
| 四足本体 | DEEP Robotics Lite3 Venture | 1 | 腿式运动、IMU/里程计、运动控制 |
| 3D LiDAR | Livox MID-360 | 1 | Ethernet / PointCloud2，主几何避障 |
| RGB 左右相机 | e-con Systems See3CAM_50CUG Color | 2 | USB3/UVC、Global Shutter；组成被动双目 |
| 镜头 | 匹配的 C-mount 镜头，初始目标 HFOV 70°–80° | 2 | 同型号/焦距/成像圈；供应商确认 |
| 双目触发 | 同步触发分配器和线束 | 1 套 | 同步两相机曝光；电平/引脚需确认 |
| RTK | ArduSimple simpleRTK2B Budget / ZED-F9P + ANN-MB | 1 套 | NavSatFix/UBX，室外全局定位 |
| 机载计算 | NVIDIA Jetson AGX Orin Developer Kit | 1 | ROS 2、Nav2、立体视觉、点云 |
| Ethernet | 工业级千兆交换机（至少 5 口） | 1 | 连接 Lite3、MID-360、AGX |
| 电源 | 经核算 DC/DC、电源分配与保险 | 1 套 | 给 AGX 和传感器供电 |
| 机械安装 | Venture 安装板、减振和天线支架 | 1 套 | 固定设备、控制质心和线缆 |

**载荷预算：** Venture 标称 4.5 kg。项目内部目标为所有外设、计算机、支架、线束和电源件合计不超过 **3.6 kg**（保留约 20% 余量；这是项目工程目标，不是厂商认证值）。若超过目标，重新选计算平台/载板或布局；不能突破厂商载荷/质心限制。

**电源约束：** Lite3 有外设 24V/12V/5V 接口，但各路持续/峰值电流和总功率待厂商书面确认。不能把 24V 直接接入 AGX Orin 开发套件 DC 输入；需要符合额定电压/功率的 DC/DC 和保护。

### 1.3 软件发行版策略

- 首轮 bring-up 对齐云深处公开配置：Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble + AGX Orin + MID-360。
- ROS 2 Humble 预计 2027 年 5 月结束支持；因此自研包从第一天起保持 Humble/Jazzy 可移植，Lite3 SDK 隔离在 bridge/adapter 内。
- 长期目标 ROS 2 Jazzy；在 Humble EOL 前完成 Ubuntu 24.04 + Jazzy 构建和集成迁移验证。
- Livox `livox_ros_driver2` 有 Jazzy 构建说明，但仍需锁定 commit 后编译和实测。
- 不默认在一个 DDS domain 混用不同 ROS 2 发行版节点；跨发行版时采用 UDP/串口/标准消息网关。

### 1.4 完整系统数据流

```text
Lite3 leg odom + IMU ──> Local EKF ──> odom -> base_link
                                  └──> navsat_transform <── RTK NavSatFix
                                                   |
                                             GNSS quality gate
                                                   |
                                               Global EKF
                                                   └──> map -> odom

MID-360 ──> PointCloud2 ──> filtering/self-filter/deskew ───┐
                                                            ├─> Nav2 local costmap
2 × RGB ──> sync/rectify/disparity/depth/cloud ─────────────┘
     └──> object detection/tracking (semantic enhancement)

Nav2 Global Costmap + Planner + BT
                 |
Nav2 Local Costmap + Controller
                 |
Velocity Smoother
                 |
Collision Monitor
                 |
Lite3 command adapter / watchdog
                 |
Lite3 vendor motion SDK / gait controller
```

### 1.5 关键分工原则

- LiDAR 是主几何避障源，双目 PointCloud2 是补充观测源；两种 source 保留独立配置和诊断。
- RTK 用于室外全局位置与航点，不用于近场碰撞检测；单天线 GNSS 静止时不提供机身 yaw。
- local EKF 维持连续 `odom→base_link`；global EKF 与 navsat_transform 利用 RTK 提供 `map→odom` 修正。
- RGB 目标检测提供类别/跟踪信息，不代替几何深度。
- Nav2 2D costmap 不是四足足端落点规划；复杂地形需独立 traversability/terrain 能力。
- Collision Monitor 是软件层附加保护，不是安全认证急停设备；必须有物理急停、遥控停止和底层 watchdog。

---

## 2. 阶段任务清单（所有状态只在本节维护）

详细技术方案和实施说明见：
- [TaskPlan_Part-01：系统架构与硬件设计](TaskPlan_Part-01_ArchitectureAndHardware.md)
- [TaskPlan_Part-02：测试、风险与参考资料](TaskPlan_Part-02_TestingAndGovernance.md)
- [ADR-0002：硬件平台选型](docs/decisions/ADR-0002-hardware-platform.md)
- [硬件接口确认清单](docs/interfaces/hardware_interface_checklist.md)

### 阶段 A — 硬件基线与接口冻结（当前阶段）

- [x] A1. 完成本体和外置导航传感器技术预选：Lite3 Venture、MID-360、双 See3CAM_50CUG、simpleRTK2B/ZED-F9P、AGX Orin。
- [x] A2. 核对云深处 Lite3 Venture + AGX Orin + MID-360 官方扩展示例与初始软件版本。
- [ ] A3. 向云深处确认 Venture 当前 SKU、交付件、固件/SDK、安装 CAD、外设接口持续/峰值电流、载荷与重心限制。
- [ ] A4. 获取整套设备质量、功耗、热设计和网络拓扑；确认外设总质量是否满足 3.6 kg 项目目标。
- [ ] A5. 获取 MID-360 当前版本、SDK、驱动 commit、供电线束和 Ethernet 配置。
- [ ] A6. 向 e-con Systems 确认 See3CAM_50CUG Color exact SKU、GPIO trigger 电气规格、同步模式、Jetson/Linux UVC 和 70°–80° 镜头。
- [ ] A7. 确认 simpleRTK2B 的天线、USB/UART、目标地区 NTRIP 改正服务或自建基站。
- [ ] A8. 确认 Lite3 IMU/leg odom/joint_states/cmd_vel 当前接口、频率、时间戳、协方差、横移语义、速度限制和 watchdog。
- [ ] A9. 冻结整机 Topic/TF/时间同步/运动学契约和 USB/Ethernet/电源布局。
- [ ] A10. 定义第一阶段量化验收指标：定位、检测、时延、停车距离、路径误差、续航和失效行为。
- [ ] A11. 完成采购前风险评审；无供电/载荷/接口重大风险后批准采购与进入阶段 B。

**阶段 A 准入：** 厂商接口、电源、质量、安装和软件兼容均有资料或测量支持；形成 BOM、CAD、网络/电源拓扑和验收阈值。

### 阶段 B — 驱动、时间同步和标定

- [ ] B1. 构建 Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble 首轮镜像。
- [ ] B2. 编译 Lite3 SDK/ROS 2 transfer，只读读取 odom/IMU/joint/status。
- [ ] B3. 建立 Lite3 bridge/adapter，先只读诊断，再验证限幅和 watchdog，不立即开放自主运动。
- [ ] B4. 编译 MID-360 Livox SDK2 和 `livox_ros_driver2`。
- [ ] B5. 接入左右 UVC 相机和 GPIO 硬件触发，验证双路 USB3 带宽。
- [ ] B6. 接入 ZED-F9P，完成 NavSatFix/UBX 状态和 NTRIP/基站。
- [ ] B7. 建立 URDF/TF，完成相机内外参、双目基线、LiDAR/GNSS 外参。
- [ ] B8. 测量 PTP/GPS/相机触发/UDP 的同步误差、时延和抖动。
- [ ] B9. 录制完整 rosbag2，建立回放和 diagnostics。
- [ ] B10. 所有 topic、frame_id、QoS、频率、数据年龄和 TF 检查通过。

### 阶段 C — 状态估计与 RTK 全局定位

- [ ] C1. 核实厂商腿式 odom 是否已融合 IMU，避免重复融合强相关信息。
- [ ] C2. 配置 local EKF 输出平滑 `odom→base_link`。
- [ ] C3. 配置 navsat_transform（ENU/UTM、datum、yaw offset、磁偏角、天线杆臂）。
- [ ] C4. 配置 GNSS quality gate 与 global EKF 输出 `map→odom`。
- [ ] C5. 测试 Fixed/Float/Single、遮挡、多路径、断改正和恢复。
- [ ] C6. 测试静止、直行、转弯、横移（若支持）、步态切换和机身振动。
- [ ] C7. 完成 GNSS 降级、超时、跳变剔除和恢复策略。

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

- [ ] G1. 自研包在 Ubuntu 24.04 + ROS 2 Jazzy 构建。
- [ ] G2. 分别验证 Lite3 bridge、Livox、camera、robot_localization、Nav2 Jazzy 兼容性。
- [ ] G3. 不兼容厂商组件以 UDP/标准消息 bridge 隔离，不默认混用不同 distro DDS。
- [ ] G4. 在 Humble 2027-05 EOL 前完成 Jazzy 迁移和 CI。
- [ ] G5. 完成仿真、rosbag、HIL、封闭场地和受控户外验收。
- [ ] G6. 输出安装、标定、启动、监控、故障恢复和维护手册。
- [ ] G7. 冻结软件/固件/驱动/地图/模型/标定版本并发布可复现版本。

---

## 3. 当前进度（由每轮执行持续更新）

### 已完成
- [x] 仓库初始化，默认分支 main。
- [x] 完成系统架构、数据流、RTK/双 EKF、双目、Nav2、Collision Monitor 和四足地形设计。
- [x] 完成 A–G 阶段拆分和测试/风险基线。
- [x] 完成本体和传感器技术预选。
- [x] 核对云深处 Lite3 Venture + AGX Orin + MID-360 的官方参考配置。
- [x] 创建 ADR-0002 硬件选型记录和更新硬件接口清单。
- [x] TaskPlan 拆分为根目录进度台账 + 两份设计分册。

### 未完成 / 阻塞
- [ ] 未采购设备，未接收实物。
- [ ] 尚未获得 Lite3 Venture 电源额定、载荷/重心和安装 CAD 的书面确认。
- [ ] 双目 exact SKU/GPIO trigger/镜头/Jetson 双机同步尚待供应商确认。
- [ ] RTK NTRIP 改正服务或基站尚未确定。
- [ ] 载荷、供电、热设计、质心和续航未实测。
- [ ] ROS 驱动未在目标计算机编译；尚无传感器 rosbag 和 Nav2 实机结果。

### 下一步（按顺序执行）
1. 整理 Lite3 Venture + AGX Orin + MID-360 扩展接口/安装/电气确认问题，向厂商取得书面答复。
2. 向相机供应商确认两台 See3CAM_50CUG 的 GPIO 触发参数和匹配广角镜头。
3. 确认 RTK 的 NTRIP/基站来源和部署测试区域。
4. 建立质量/功耗/散热预算和初步安装 CAD，检查 3.6 kg 项目外设目标。
5. 更新本文件 A3–A11 进度；A 阶段准入后再推进 B 阶段。

---

## 4. 进度变更记录

| 日期 | 版本 | 变更 | 提交/证据 |
|---|---|---|---|
| 2026-11-24 | v0.1 | 新仓库初始化、建立系统架构和 A–G 初版任务计划 | 仓库首次初始化提交 |
| 2026-11-24 | v0.2 | 预选 Lite3 Venture、MID-360、双 See3CAM_50CUG、ZED-F9P RTK、AGX Orin；建立硬件 ADR/接口清单 | ADR-0002 和接口清单提交 |
| 2026-11-24 | v0.2.1 | TaskPlan 超过 30 KB，按规则拆为架构硬件分册和测试风险分册；保留本根文件唯一进度状态 | 本次 TaskPlan 维护提交 |

---

## 5. 文档索引

- [TaskPlan Part 1 — 架构与硬件完整设计](TaskPlan_Part-01_ArchitectureAndHardware.md)
- [TaskPlan Part 2 — 测试、风险与参考资料](TaskPlan_Part-02_TestingAndGovernance.md)
- [ADR-0001 — 传感器职责与定位架构](docs/decisions/ADR-0001-sensor-roles-and-localization.md)
- [ADR-0002 — 硬件平台选型](docs/decisions/ADR-0002-hardware-platform.md)
- [硬件接口确认清单](docs/interfaces/hardware_interface_checklist.md)
- [ROS 2 package 规划](docs/implementation/ros2_workspace_and_packages.md)
- [验收测试矩阵](docs/testing/acceptance_test_matrix.md)
- [系统架构](docs/architecture/system_architecture.md)
- [传感器融合](docs/architecture/sensor_fusion.md)
- [Nav2 导航和避障](docs/navigation/nav2_navigation.md)
- [四足地形/安全验证](docs/validation/safety_and_validation.md)
- [双目 RGB 深度感知](docs/perception/stereo_obstacle_perception.md)
