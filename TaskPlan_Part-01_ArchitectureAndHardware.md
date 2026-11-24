# TaskPlan Part 1 — 系统架构与硬件设计细节

- 文档版本：v0.2
- 更新日期：2026-11-24
- 默认分支：`main`
- 项目仓库：https://github.com/rexyll/robdog_nav2
- 项目目标：基于四足机器狗，使用 **LiDAR + 两台普通 RGB 相机组成被动双目 + RTK GNSS**，结合机器狗 IMU/腿式里程计，构建 ROS 2 / Nav2 全局导航、局部实时避障和四足地形可通行性系统。
- **本体预选：DEEP Robotics 绝影 Lite3 Venture（未采购、待厂商确认最终交付和接口）。**
- **传感器预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）；ArduSimple simpleRTK2B Budget（u-blox ZED-F9P + ANN-MB 天线）。**
- **计算平台预选：NVIDIA Jetson AGX Orin Developer Kit（64GB 版本作为首轮开发配置；安装、电源和质量预算待验证）。**
- 当前阶段：**阶段 A — 硬件基线与接口冻结**
- 当前状态：**方案预选已完成；采购、实物测量、驱动编译和机器狗实机验证尚未完成。**

---

## 0. 项目治理规则（必须长期遵守）

### 0.1 TaskPlan 是唯一进度真源

1. **所有任务状态、完成证据、阻塞项、决策变更、测试结论和下一步动作，必须更新到仓库根目录 `TaskPlan.md`。**
2. 不得只在 README、Issue、Pull Request、其他设计文档或对话中记录进度而不回写本文件。
3. 其他文档用于保存详细设计、接口说明、测试报告和 ADR；根目录 TaskPlan 负责汇总当前状态、任务清单、验收准入和变更记录。
4. 每次推进至少更新：
   - 当前阶段与总体状态；
   - 变更任务的复选框；
   - 本次完成了什么、证据是什么；
   - Git commit SHA、测试/验证方式；
   - 新增风险、未决项和下一步动作。
5. 只有有代码/文件、官方证据、构建日志、rosbag、测试报告或可重复操作记录支持时，才允许把任务标为完成。方案设计完成不等于功能实现，文档完成不等于系统通过测试。
6. 默认只在 `main` 上工作；用户未明确要求时，不创建新分支。
7. 如果 TaskPlan 增长到约 **30 KB** 或出现明显难以维护的长度，则按主题/阶段拆分为根目录 `TaskPlan_Part-01.md`、`TaskPlan_Part-02.md` 等；根目录 `TaskPlan.md` 必须保留总体目标、当前状态、关键决策、阶段索引、最近进度和分文件导航。拆分之后进度仍必须先更新根目录索引，再更新对应分册。
8. 不承诺后台执行。每轮实际完成的工作都应即时写入仓库并在回复中报告。

### 0.2 状态标记

- `[ ]`：未开始。
- `[x]`：已完成，必须有证据。
- `[~]`：正在实施或部分完成；需写明剩余工作。
- `[!]`：阻塞/存在风险，需写明原因和解除条件。
- 阶段状态只能按阶段准入标准更新，不按“做了很多工作”主观宣布完成。

### 0.3 证据规则

- 硬件参数优先采用制造商数据手册/开发者文档。
- 软件兼容性需同时检查官方发行版文档、驱动源码和本机编译/运行结果。
- 采购价格、库存、交付周期会变化；只记录查询日期和报价，不作为长期固定参数。
- 任何未知质量、重量、电流、工作温度、时钟同步误差和机器人速度限制都标注“待供应商确认”或“待实测”，不得猜测。
- 真实导航安全不能仅靠算法演示证明；必须测量停止时间、停车距离、误检/漏检与失效行为。

---


> 本分册保存系统/硬件的完整设计细节。任务状态和进度只在根目录 [TaskPlan.md](TaskPlan.md) 更新，不在本分册维护进度。

## 1. 目标场景与系统能力边界

### 1.1 第一阶段目标场景

- 平整室内地面、实验室、办公室、仓储和建筑物周边的室外硬质地面；
- 机器人自主行走到目标点、按预设航点巡航、静态障碍绕行；
- 对行人、箱体、墙壁、桌椅、货架、路沿等目标进行几何检测或视觉语义增强；
- RTK 可用的室外区域使用全球位置作为地图/航点约束；
- GNSS 不可用的室内场景使用本体里程计与局部感知/SLAM，不能依赖 RTK；
- 传感器或定位异常时，系统按预定义策略降速、停止或切换到受限模式。

### 1.2 第一阶段不承诺的能力

- 不承诺仅凭 2D Nav2 costmap 完成可靠的楼梯、复杂台阶、沟壑、悬崖和落足规划；
- 不承诺普通双目对透明玻璃、镜面、反光金属、纯色无纹理、极暗/逆光或细电线稳定测深；
- 不承诺单天线 RTK 在静止时输出可信机身航向；
- 不承诺普通 LiDAR + RGB 视觉达到功能安全认证等级；
- 不把 Nav2 Collision Monitor 当成硬件急停、安全激光扫描器或安全控制器；
- 不在未验证四足控制接口前假设机器狗是差速底盘或完全全向底盘。

### 1.3 最终系统闭环

`传感器 → 时间同步/标定/TF → 状态估计与障碍物感知 → Nav2 全局规划/局部控制 → Collision Monitor → 四足速度适配器 → 厂商步态/运动控制 → 反馈与诊断`

---

## 2. 本体与硬件选型（已预选，待采购确认）

### 2.1 四足本体：DEEP Robotics 绝影 Lite3 Venture

<badge>项目首选本体</badge>

**选择理由：**

1. 云深处官方开发者文档明确给出 Lite3 Venture 的外扩感知开发方案，并示范安装 Jetson AGX Orin 与 Livox Mid-360，降低本项目的机械安装和传感器驱动风险。
2. Lite3 Venture 标称负载能力 4.5 kg，提供 2 个 Ethernet 和 24V/12V/5V 外部电源接口，适合连接额外机载计算机、雷达和双目相机。
3. 官方提供 Lite3 SDK/ROS 2 通信代码，能获得腿式里程计、IMU、关节状态，并通过 `geometry_msgs/msg/Twist` 向机器狗发送平移/转向命令。
4. 本体已有基本运动控制、IMU 和腿式运动状态，不需要自行开发电机驱动、关节控制和步态底层；项目聚焦感知、定位和 Nav2。

**官方公开参数（采购前仍需销售/技术支持书面确认）：**

| 项目 | Lite3 Venture |
|---|---|
| 外形尺寸（站立） | 约 610 × 370 × 445 mm |
| 本体重量 | 约 12.2 kg |
| 标称负载 | 4.5 kg |
| 标称最大坡度 | 40°（不能直接作为本项目安全通行坡度） |
| 标称越障高度 | 18 cm（不能直接理解为自主导航可跨越高度） |
| 空载续航 | 约 1.5–2 h；实际搭载计算机/传感器后的续航待测 |
| 外部通信 | 2 × Ethernet |
| 外部供电 | 24V / 12V / 5V 接口；各路允许持续/峰值电流待供应商确认 |

**为什么不选其他 Lite3 型号：**

- Lite3 Basic：标称负载更高，但产品比较信息中 Venture 有明确外部 I/O 扩展接口，而 Basic 的接口配置不满足本项目优先考虑的机载设备扩展。
- Lite3 Pro：计算/感知能力更强，但标称载荷约 4 kg；本项目仍需加装双相机、RTK 和雷达。
- Lite3 LiDAR 版：标称载荷约 2.5 kg，且自带导航感知配置，不是为本项目自定义传感器组合优化的起点。
- 以上对比仅基于公开参数；若云深处能提供 Venture 之外更适合外部载荷的更新型号，应先补充对比 ADR，不擅自更换项目基线。

### 2.2 传感器和计算平台 BOM

| 子系统 | 预选型号 | 数量 | 项目用途/接口 | 状态 |
|---|---|---:|---|---|
| 四足本体 | DEEP Robotics Lite3 Venture | 1 | 腿式运动、底层控制、IMU/腿式里程计 | 已预选，未采购 |
| 3D LiDAR | Livox MID-360 | 1 | Ethernet，PointCloud2；主要几何障碍源 | 已预选，需确认供电线束和安装 |
| 左/右普通 RGB 相机 | e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）（同型号、同批次优先） | 2 | USB 3.2 Gen 1/UVC，全球快门；组成被动双目 | 已预选，需确认 GPIO 触发和镜头 |
| 双目镜头 | 配套匹配 C-mount 镜头，建议初始水平视场约 70°–80° | 2 | 同焦距、同型号、同光学参数 | 需供应商选型，不使用未经确认的默认窄视场镜头 |
| 双目同步 | 共同时钟/硬件触发分配板及线束 | 1 套 | 将同一触发信号送入左右相机 GPIO | 需根据相机 pinout/电平确认 |
| RTK GNSS | ArduSimple simpleRTK2B Budget Starter Kit，u-blox ZED-F9P + ANN-MB 多频天线 | 1 套 | USB/UART，NavSatFix/UBX；室外全局位置 | 已预选，需 RTK 改正服务/基站 |
| 机载计算机 | NVIDIA Jetson AGX Orin Developer Kit（64GB 开发配置） | 1 | 运行 ROS 2、Nav2、视觉/点云处理；USB3、Ethernet | 参考厂商开发方案，需确认安装、电源和质量预算 |
| 以太网 | 工业级 Gigabit Ethernet Switch（至少 5 口） | 1 | 连接 Lite3、MID-360、AGX 主机和维护网口 | 规格需确认 |
| 电源 | 经核定的 DC/DC 与电源分配/保险 | 1 套 | 从 Lite3 外部电源或独立电源给 AGX 和外设供电 | 不能直接按接口电压接入，需供应商确认电流 |
| 机械件 | Venture 专用安装板/定制减振支架/天线支架 | 1 套 | 固定计算机、LiDAR、双目与 RTK | 需绘制 CAD 和测量质心 |
| 安全件 | 独立急停/遥控停止和线束固定 | 1 套 | 实机调试人员保护 | 必须落实 |

### 2.3 为什么选这些传感器

**LiDAR — Livox MID-360**

- 水平视场 360°、垂直视场约 -7° 至 +52°；
- 典型点云帧率 10 Hz、首回波点率 200,000 点/秒；
- 标称测程在 100 klx 条件下约 40 m（10% 反射率）/70 m（80% 反射率）；
- Ethernet 输出，支持 PTPv2/GPS 时间同步，内置 IMU，IP67；
- 是 Lite3 Venture 官方感知扩展示例列出的雷达型号。
- 主要作用：稳定几何测距、障碍物占据、local costmap 和 Collision Monitor。
- 限制：不同反射率、入射角、雨雾/灰尘、低矮物、负障碍和机身运动会影响结果；不把内置 IMU 自动视为已校准的机身 IMU。

**双目相机 — 2 × See3CAM_50CUG Color**

- 5 MP（2448×2048）、Sony IMX264、Global Shutter；
- USB 3.2 Gen 1，Linux/UVC，最高约 35 fps（5MP）；
- 相机系列具有 GPIO/Trigger/Strobe 功能，彩色型号列出 Trigger 控制；
- 选择同型号同镜头的两台相机，外部共触发；实际触发模式、电平和帧同步方式需从最新手册/供应商确认。
- 不用产品页面随附的窄视场镜头直接作为默认方案；先按机器人前方有效探测距离与视场，选用成像圈匹配的 C-mount 镜头，初始目标约 70°–80° 水平视场，再做标定和畸变评估。
- 起始基线可按约 100–120 mm 作为机械设计试验值，不是已确定最终值；结合近/中/远距离深度误差、遮挡与视场重叠试验定型。
- 两台普通相机组成被动双目，不是 RGB-D 相机；透明、反光、无纹理、弱光、快速运动和遮挡场景必须依赖 LiDAR/其他传感器补偿。

**RTK — ArduSimple simpleRTK2B Budget / ZED-F9P**

- 多频 GNSS，搭配 ANN-MB 多频天线；
- 最高性能配置可达到最高约 10 Hz 位置输出；
- 达到 RTK 级定位必须有 NTRIP 改正数据或自建基站；没有改正源时不能把单点定位精度当成厘米级。
- 本项目先用单天线方案，机身航向由 Lite3 IMU/腿式状态估计提供；低速或静止时不能从单天线 GNSS 位置推断机身 yaw。
- 双天线 GNSS heading 作为后续可选升级，不纳入第一版 BOM。

**机载计算 — NVIDIA Jetson AGX Orin**

- 选择 AGX Orin 是为了尽量遵循云深处 Lite3 Venture 的公开感知扩展参考配置。
- 64GB 开发套件可同时接入两路 USB3 相机、LiDAR Ethernet 和 RTK USB/UART；实际总线带宽、功耗和散热需要测量。
- 该开发套件用于第一版原型，不等于最终工业化机载计算机；后续可在相同 Orin 软件基线下换用工业载板，但必须重新验证供电、接口、散热和 SDK。
- 机载计算机的 15–60W 额定功耗范围意味着要测量高负载功耗和热设计，不能仅按空闲状态设计供电。

### 2.4 载荷、供电和安装约束（采购前必须关闭）

1. Lite3 Venture 标称载荷 4.5 kg。项目内部设计目标为**全部外置设备、支架、线缆和电源分配合计不超过 3.6 kg**（保留约 20% 载荷余量）；这是项目工程预算，不是厂商额定指标。
2. 实际 AGX Orin 开发套件、相机和镜头、GNSS 板/天线、以太网交换机、DC/DC、电缆、支架重量必须逐项称重；若超过目标，优先将计算平台换成紧凑 Orin 工业载板或重新布局，不允许突破本体载荷。
3. 计算机安装于低重心、远离腿部扫掠范围的位置；LiDAR 位置保证 360° 水平视场尽量无遮挡；双目水平前视、两相机基线刚性；RTK 天线置于尽可能开阔且远离电磁干扰的位置。
4. 需测量整机质心、俯仰/横滚惯量影响、上盖结构承载、行走振动和跌倒碰撞风险；不能只按静态重量判断。
5. Lite3 提供 24V/12V/5V 外设接口，但每路最大连续/峰值电流、总功率和保护策略必须获得厂商确认。
6. MID-360 按官方 Venture 参考方案使用 24V XT30 供电和 Ethernet；AGX Orin 使用符合其电压/功率要求的独立 DC/DC 或经认可的电源路径。禁止把机器人 24V 直接接到 AGX 开发套件 DC 输入。
7. 配置保险、反接/浪涌保护、线缆应力释放和电磁兼容；USB 相机线缆要固定，避免步态运动导致接触不良。
8. 搭载外设后的续航必须重新测量；本体标称空载续航不能作为整机续航承诺。

---

## 3. ROS 2 软件平台决策

> 版本选择以 2026-11-24 已发布的软件为准：Humble 用于厂商参考复现，Jazzy 作为当前 Jetson Orin 长期目标，Lyrical 先记录为未来评估版本。

### 3.1 发行版策略

- 云深处官方 Lite3 Venture 感知扩展示例使用 Ubuntu 22.04、ROS 2 Humble、JetPack 6.1、CUDA 12.6，并以 AGX Jetson Orin + MID-360 为参考。
- 阶段 B 首先复现厂商参考栈（Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble），作为 Lite3 驱动兼容性基准，不将它默认定为项目长期部署基线。
- ROS 2 Humble 上游支持至 **2027 年 5 月**；自研包需保持 Humble/Jazzy API 可移植，所有厂商 SDK 隔离在 adapter/bridge 内。
- NVIDIA 当前 JetPack 7.2.2 支持 Jetson Orin Family，并以 Ubuntu 24.04 为 L4T 系统基线。项目同时验证 JetPack 7.2.2 + Ubuntu 24.04 + ROS 2 Jazzy，将 Jazzy 作为当前 Orin 长期部署目标。
- ROS 2 Lyrical Luth 已于 2026 年 5 月发布，LTS 支持至 2031 年 5 月，但其 Tier 1 Ubuntu 平台为 26.04；当前 JetPack 7.2.2 的 Orin 参考系统为 Ubuntu 24.04，因此暂不把 Lyrical 作为 Orin 项目基线。待 NVIDIA Orin BSP 支持 Ubuntu 26.04、且 Lite3/相机/LiDAR/NAV2 驱动可用后重新评估。
- Livox 官方 `livox_ros_driver2` 提供 Ubuntu 24.04 / ROS 2 Jazzy 构建说明；仍需在项目锁定 commit 上实际编译测试。
- 不混用不同 ROS distro 的 DDS 节点作为默认架构；如需隔离厂商旧栈和 Nav2 新栈，应使用 UDP/串口/标准消息网关并限制网络域。

### 3.2 Lite3 本体接口

优先采用云深处官方 `sdk_deploy` / `Lite3_ROS` 源码与厂商 SDK，不直接控制电机和关节驱动。

官方 ROS 2 接口示例包含：
- `/leg_odom`：`geometry_msgs/msg/PoseWithCovarianceStamped`；
- `/leg_odom2`：`nav_msgs/msg/Odometry`（位姿+速度）；
- `/imu/data`：`sensor_msgs/msg/Imu`；
- `/joint_states`：`sensor_msgs/msg/JointState`；
- `/cmd_vel`：`geometry_msgs/msg/Twist`，线速度 x/y 与角速度 z；坐标方向/速度范围/模式要求以具体 SDK 版本手册为准。
- 厂商 SDK 部分状态消息约 10 Hz、控制有效命令可达约 50 Hz；具体由 SDK 版本和机器人固件决定，必须测量。

本项目需实现 `robodog_lite3_bridge` 或 `robodog_cmd_adapter`，负责命令限幅、状态映射、命令超时、连接状态和 Nav2 topic 适配。不得未经厂商许可改写底层安全控制或直接向关节发送 Nav2 命令。

### 3.3 软件层和建议包

| 包/子系统 | 工作内容 |
|---|---|
| `robodog_description` | URDF/Xacro、base/sensor frames、碰撞外形 |
| `robodog_lite3_bridge` | Lite3 SDK/UDP/ROS2 通信、odom/IMU/status 与 cmd_vel adapter |
| `robodog_sensor_bringup` | Mid-360、左右相机、RTK 驱动、时间同步 |
| `robodog_stereo_perception` | 双目同步、校正、视差、深度、点云和置信度 |
| `robodog_obstacle_perception` | LiDAR/点云过滤、地面分割、聚类、障碍物目标 |
| `robodog_localization` | local/global EKF、navsat_transform、GNSS gate |
| `robodog_nav2_bringup` | Nav2、地图、planner/controller/BT、costmaps |
| `robodog_safety` | Collision Monitor、停止/减速区、命令健康监测 |
| `robodog_diagnostics` | 时间戳、频率、温度、网络、GNSS 状态、TF/命令链路健康 |
| `robodog_interfaces` | 仅在标准 ROS 消息不足时创建自定义接口 |

外部驱动优先固定上游 commit 并使用官方驱动；自研包不复制厂商代码。SDK、模型权重和第三方依赖必须进行许可证/分发边界检查。

---

## 4. 坐标系、时间同步与定位融合

### 4.1 TF 树

```text
map
 └── odom
      └── base_link
           ├── imu_link
           ├── lidar_link
           ├── left_camera_link
           │    └── left_camera_optical_frame
           ├── right_camera_link
           │    └── right_camera_optical_frame
           └── gnss_antenna_link
```

- local EKF 唯一发布 `odom → base_link`，提供连续、平滑的本体运动。
- global EKF/全局定位仲裁器唯一发布 `map → odom`。
- robot_state_publisher/URDF 唯一发布机体到传感器的固定外参。
- 不允许多个节点同时发布同一 parent-child TF。
- 统一 ROS 右手系、米、弧度；base 通常 x 前/y 左/z 上；相机 optical frame 按 REP 103。
- GNSS 天线相对 base_link 的杆臂必须测量并写入 URDF/配置。

### 4.2 Local EKF

输入：
- Lite3 `/leg_odom2` 或经验证的腿式 odom；
- `/imu/data`；
- 若可获得，接触状态/足端速度/滑移置信度。

输出：
- `/odometry/filtered/local`；
- `odom → base_link`。

先确认厂商腿式 odometry 是否已在内部融合 IMU，避免把强相关数据错误地当成两个独立传感器重复融合。需要实际检查 covariance、坐标系、时间戳和静止/走动/转弯表现。

### 4.3 RTK 与 Global EKF

数据流：
1. ZED-F9P 输出 `NavSatFix` 和原始 UBX 状态；
2. GNSS quality gate 检查 RTK Fixed/Float/Single、协方差、改正龄期、时间戳和异常跳变；
3. `navsat_transform_node` 接收 GNSS、local odom 和可信 IMU yaw，投影成局部 Cartesian odometry；
4. Global EKF 融合 local odom 与经门控 GNSS，发布 `map → odom`；
5. Nav2 global costmap/航点使用 map，local costmap 通常使用 odom。

单天线 GNSS 不提供静止机身航向；第一阶段采用 Lite3 IMU/腿式状态估计的 yaw。若实测航向不满足要求，再评估双天线 heading。

### 4.4 时间同步

- 双目左右相机必须硬件同步优先；软件同步误差要根据机器速度、相机帧率、基线和深度误差预算量化。
- MID-360 支持 PTPv2/GPS 时间同步；优先将 LiDAR 与主机时钟同步。
- GNSS 提供 PPS/时间信息时，评估将其纳入统一时间基准。
- Lite3 SDK UDP 状态消息要测量网络时延、抖动和时间戳来源；若仅有接收时间，记录其限制。
- 记录各节点数据 age；过期数据不允许继续更新 costmap 或作为定位真值。

### 4.5 RTK 降级

- Fixed 且状态/协方差可信：正常融合；
- Float/Single：根据实际精度降权或停止绝对位置更新；
- 改正中断、超时、跳变或多路径：冻结/剔除异常 GNSS，短期依靠 local odom；
- 无 GNSS 超过允许的漂移时间或定位协方差超限：减速/停止；
- GNSS 恢复后须通过创新量与持续有效性检查，再平滑恢复，不能瞬时跳变。

所有门限由数据集和测试确定，不从示例 YAML 照抄。

---

## 5. LiDAR 与双目视觉障碍物感知

### 5.1 LiDAR 感知流水线

`MID-360 → Livox SDK2 / livox_ros_driver2 → PointCloud2 → 时间/TF 检查 → 自体过滤 → 离群点与地面处理 → obstacle points → Nav2 Costmap`

- 使用厂商 MID-360 配置、正确以太网 IP 和驱动版本；
- 处理机身俯仰、横滚和运动造成的点云畸变；
- 使用 URDF 实际外形做自体过滤；
- 过滤距离、强度、地面点及无效回波，保留原始 topic 便于复核；
- 对传感器安装视场进行遮挡检查；
- LiDAR 的内置 IMU 与机器狗 IMU 坐标/时间不同，不可未经标定直接融合。

### 5.2 双目视觉流水线

`left/right Image + CameraInfo → 硬件同步 → stereo rectify → disparity → depth → confidence mask → PointCloud2 → 过滤/降采样 → costmap`

- 使用 ROS 2 `image_pipeline/stereo_image_proc` 或经过评测的 OpenCV StereoSGBM/深度学习模型。
- 标定左右相机内参、畸变、外参和基线，锁定焦距、分辨率和对焦状态。
- 以目标距离范围决定焦距与基线。先用 70°–80° 水平视场镜头和约 100–120 mm 基线做试验，不把该值当作最终规格。
- 输出视差、深度置信度、有效像素率、深度 PointCloud2 和诊断状态。
- 双目深度公式：`Z = f × B / d`；深度误差随距离上升明显增加，因此必须按距离区间评估误差。
- YOLO 等语义目标检测可以标注类别/动态目标，但类别结果不替代几何测距，不可仅凭框中心生成安全障碍点。

### 5.3 双目不可依赖的场景

玻璃/透明塑料、镜面/高反光、无纹理单色物体、极暗/逆光、左右不同步、快速运动模糊、细电线/小尺寸目标、强遮挡和极近距离边界。此时使用 LiDAR 等独立来源补足；若存在共同盲区，则需要改变传感器布局或增加适用的专用传感器。

### 5.4 两类障碍源进入 Nav2

- LiDAR 和 stereo PointCloud2 保持独立 observation source；
- local costmap 依据实际应用选择 ObstacleLayer 或 VoxelLayer；
- 点云 source 配置 marking/clearing、range、min/max height、source timeout、QoS、TF；
- Stereo 无效视差/空洞不等于空间 free，不能无条件用于 clearing；
- 对每个障碍保留来源和质量信息，便于排查误检和冲突；
- 语义行人跟踪作为动态目标增强层，不让语义层覆盖几何层。

---

## 6. Nav2 全局导航与实时避障

### 6.1 系统节点链

`任务/航点客户端 → BT Navigator → Planner Server + Global Costmap → Controller Server + Local Costmap → Velocity Smoother → Collision Monitor → Lite3 cmd_vel adapter → 厂商步态控制器`

所有速度命令必须经过唯一的安全命令链，禁止存在绕过 Collision Monitor/command gate 的额外发布者。

### 6.2 Global Costmap

- 室内可用已知栅格地图或 SLAM Toolbox；
- 室外 RTK 位置和航点转换到局部 ENU/UTM，再发送 map 坐标目标；
- 地理坐标经纬度不能直接作为 Nav2 map 的 x/y 米；
- global costmap 以静态地图、禁行区和必要的静态障碍背景为主。

### 6.3 Local Costmap

- `global_frame = odom`，使用 rolling window；
- MID-360 PointCloud2 作为主几何观测源；
- 双目 PointCloud2 作为补充观测源；
- 需要高度占据判断时评估 VoxelLayer；单纯二维障碍投影可评估 ObstacleLayer；
- footprint 根据实际机体外形、站立高度、腿部扫掠范围和转向姿态测量；
- inflation 与 stop distance 综合考虑定位误差、感知误差、时延、机器狗速度和真实制动距离；
- 不复制 Nav2 官方示例中的 footprint、速度和 inflation 参数直接上机。

### 6.4 Planner / Controller

候选方案：
- Planner：NavFn / Smac 2D / Smac Hybrid，依据运动约束选型；
- Controller：Regulated Pure Pursuit / MPPI，依据四足实际运动和避障效果实测；
- Lite3 官方 Twist 示例有线速度 x/y 与 yaw 角速度 z，但须在实际固件和 SDK 上确认横移语义、最大速度、加速度、旋转能力和步态约束。
- Nav2 参数必须与底层可执行能力一致；不能把不支持的侧移速度发给机器人。

### 6.5 Collision Monitor 与停止链

- 通过 MID-360 与经过验证的双目障碍源构建 slowdown/stop zone；
- 监控传感器数据是否新鲜、TF 是否可用、Collision Monitor 是否健康；
- command adapter 加速度/速度限幅、模式检查、命令过期停止和底层 watchdog；
- 真实测试必须有物理急停/遥控停止、人工监护和封闭测试区域。
- Collision Monitor 属于软件级附加防护，不是安全认证设备，也不保证硬实时；人员周边操作须单独做风险分析。

---

## 7. 四足地形可通行性扩展

Nav2 二维代价图只解决投影平面内障碍绕行，不保证足端落在安全地面。平地导航完成后，开展：
1. 3D 点云地面分割与高程网格；
2. 坡度、粗糙度、台阶高度、路沿和负障碍识别；
3. 可通行性代价层（traversability layer）；
4. 与 Lite3 底层步态/落足状态接口；
5. 对草地、碎石、松软地面、湿滑地面单独测试；
6. 将不允许跨越的台阶/坑/悬崖编码为禁行区域；
7. 动态行人跟踪与预测，用于速度调节和安全间隙。

任何允许跨越的高度/坡度都必须以机器狗官方能力和实际安全试验为依据，不能直接使用产品最大越障参数。

---

