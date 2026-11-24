# ADR-0002：四足本体与外置导航传感器平台选型

- 状态：**Proposed / 技术预选完成，待供应商与实物验证**
- 日期：2026-11-24
- 关联主计划：[根目录 TaskPlan.md](../../TaskPlan.md)
- 决策范围：四足本体、LiDAR、两台普通 RGB 相机、RTK、机载计算和 ROS 2 基线。

## 1. 决策摘要

项目首选平台：

- 四足本体：DEEP Robotics Lite3 Venture；
- LiDAR：Livox MID-360；
- 双目相机：2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头，GPIO trigger，配匹配 C-mount 镜头）；
- RTK：ArduSimple simpleRTK2B Budget（u-blox ZED-F9P + ANN-MB 多频天线）；
- 首轮机载计算：NVIDIA Jetson AGX Orin Developer Kit；
- 首轮厂商参考栈：Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble；
- 当前 Orin 长期目标：JetPack 7.2.2 + Ubuntu 24.04 + ROS 2 Jazzy；
- 未来评估：ROS 2 Lyrical（Ubuntu 26.04，LTS 至 2031-05），待 Orin 官方 BSP/整套驱动支持 Ubuntu 26.04 后再决定。

该决策是技术选型，不代表已采购、已获得供应商确认或已通过真实硬件验证。

## 2. 为什么选 Lite3 Venture

DEEP Robotics 的公开开发者文档将 Lite3 Venture 用作扩展感知平台，列出 AGX Jetson Orin、Livox Mid-360、Ubuntu 22.04 和 ROS 2 Humble 的完整软件硬件示例。其传感器接线说明也展示了 MID-360 由 Lite3 24V XT30 接口供电并通过 Ethernet 接入 AGX 主机。

产品公开资料列出 Venture 标称 4.5 kg 载荷、2× Ethernet 和 24V/12V/5V 外部供电接口。相较 Basic，Venture 的外部扩展接口更适合项目；相较 LiDAR 版，其标称载荷更大，且可自定义传感器。产品页所列空载续航不能代表安装外设后的续航。

**已知限制：**
- 4.5 kg 是公开载荷参数，不能仅凭此断言任何安装布局都可行。
- 外部电源每路持续/峰值电流公开资料不足，须书面确认。
- 安装了 AGX、MID-360、双相机、RTK、支架和线束之后需重新测量重量、质心、结构承载、振动、热和续航。
- 官方感知扩展示例使用 D435i，而本项目用两台普通 RGB 相机替换它，因此双目驱动、同步、标定和感知质量属于项目自定义研发，不是厂商示例开箱即用功能。

## 3. 硬件选型对照

### 3.1 本体

| 型号 | 公开载荷 | 接口/特点 | 结论 |
|---|---:|---|---|
| Lite3 Basic | 5 kg | 基础功能，公开产品比较中无 Venture 级外部扩展接口信息 | 不作为首选 |
| Lite3 Venture | 4.5 kg | 2× Ethernet、24V/12V/5V 外部供电；官方扩展感知示例 | **首选** |
| Lite3 Pro | 4 kg | 更强的集成计算/感知配置 | 本项目仍需大量自定义外设，载荷余量较小 |
| Lite3 LiDAR | 2.5 kg | 自带导航感知配置 | 载荷余量不足，且项目希望使用自己的传感器配置 |

以上以厂商当期产品比较为准，具体地区、SKU 和选件可能变化。

### 3.2 LiDAR：Livox MID-360

选择理由：
- 360° 水平视场，垂直视场 -7° 至 +52°；
- 典型 10 Hz，首回波约 200,000 points/s；
- 100 klx 条件下标称 40 m @ 10% 反射率；
- Ethernet、PTPv2/GPS 同步、内置 IMU、IP67；
- 云深处 Lite3 Venture 公开扩展示例明确使用 MID-360/MID-360s。

本项目选 MID-360 原版作为 baseline，以对应配置和驱动版本固定。采购时确认 MID-360 与 MID-360s 型号差异、供电线束、固件和 SDK 版本，不混用配置文件。

### 3.3 两台普通 RGB 相机：See3CAM_50CUG_CXXCC_BX_H01R1（Color，无镜头）

选择理由：
- Sony IMX264 5MP 彩色传感器；
- Global Shutter，减少机器狗步态运动中的滚动快门几何失真；
- USB 3.2 Gen1、UVC/Linux 支持；
- GPIO 触发/闪光灯信号，彩色 SKU 控制功能列出 Trigger；
- 5MP 模式最高约 35 fps。

采购规格：
- 精确预选 SKU：`See3CAM_50CUG_CXXCC_BX_H01R1`，Color / no lens；两台相同批次优先。
- 两台同型号、同版本、同镜头；
- 优先采用无镜头 SKU + 两个匹配 C-mount 镜头，避免默认镜头 26.1° 水平视场过窄；
- 镜头先以约 70°–80° 水平视场作为需求指标，由供应商按 2/3-inch IMX264 成像圈和畸变数据推荐；
- 左右相机由相同触发源驱动，先验证硬件同步延迟和曝光同步；
- 初始双目基线可试 100–120 mm，最终按有效近/中/远距离、遮挡、视差误差和视野重叠确定。

局限：
- 这两台相机只是普通 RGB 相机，需项目自行做 stereo calibration/rectification/disparity/depth/cloud；
- 彩色相机对照明、纹理、透明/反光物体和遮挡较敏感；
- 相机页面有不同 lens SKU 与触发控制说明，采购前必须让制造商确认选定 Color SKU 的 GPIO 触发脚位、电平和同步模式。

### 3.4 RTK：ArduSimple simpleRTK2B Budget

选择理由：
- u-blox ZED-F9P，多频 GNSS；
- 搭配 ANN-MB 多频天线；
- 高性能配置可达到最高约 10 Hz；
- 可通过 USB/UART 接入 Jetson；
- 入门预算较低，适合第一版室外 RTK 试验。

必须额外具备：
- NTRIP 改正服务，或一套可用基站；
- 目标区域需要有网络/差分覆盖；
- GNSS 天线固定、开阔天空视场和杆臂标定。

第一版采用单天线。机身静止时 yaw 来自 Lite3 IMU/状态估计，而非单天线 GNSS。若后续对低速航向精度要求很高，再评估双天线 heading。

### 3.5 计算平台：Jetson AGX Orin

首轮按云深处开发示例选择 Jetson AGX Orin。AGX Orin Developer Kit 具有多路 USB 3.x、Ethernet、丰富扩展接口和足够的 GPU/NPU 能力，可连接两路 USB 相机、LiDAR 网络与 RTK 接收机。64 GB 开发配置有较大余量，但其功耗可随工作模式显著变化，需实测。

**功率/机械风险：**
- AGX Orin 需其额定输入电源，不能把 Lite3 外设 24V 直接接入开发套件 DC 输入。
- 需确认 DC/DC 输出电压、电流、峰值功率、纹波和热设计。
- 若开发套件的尺寸、质量、功耗不适合最终上机，保留 AGX Orin 模块的软件/接口基线，后续可换成工业载板，但必须重新验证 Lite3 固定方式和驱动。

## 4. 软件基线决策

**更新说明（2026-11-24）：** ROS 2 Lyrical Luth 已于 2026-05 发布、支持至 2031-05；NVIDIA JetPack 7.2.2 已支持 Jetson Orin Family，L4T Ubuntu 24.04。需要注意，Jetson AGX Orin Developer Kit 用户指南的 ISO 安装入口仍以 JetPack 7.2.1 / L4T r39.2.1 为基础镜像，再通过 apt update / dist-upgrade 升级到 7.2.2；该安装路径需要按 AGX DevKit 用户指南实测。由于 Lyrical Tier 1 Ubuntu 平台是 26.04，当前 Orin 平台应优先验证 Jazzy，而不是为了追最新 ROS distro 跨到未获 JetPack 支持的系统。

云深处公开的 Lite3 Venture 感知扩展示例使用：
- Ubuntu 22.04；
- ROS 2 Humble；
- JetPack 6.1 / CUDA 12.6；
- Jetson AGX Orin；
- Livox MID-360；
- Intel RealSense D435i。

本项目保留其中本体、AGX、MID-360 和 Humble 硬件参考，按用户要求将相机改为双台普通 RGB。RTK 为新增传感器。

因此策略更新为：
1. 首轮按厂商示例复现 Humble/JetPack 6.1，核对 Lite3 SDK 和 MID-360 的供应商路径；
2. 并行在 AGX Orin DevKit 的 JetPack 7.2.1 base ISO → 7.2.2 update / Ubuntu 24.04 上验证 Jazzy，若 Lite3 bridge 和所有驱动可用，则将 Jazzy 作为机载长期运行基线；
3. 自研 ROS package 保持 Humble/Jazzy 可移植，Lite3 SDK 通过独立 bridge 隔离；
4. 不在长期运行中依赖 Humble；应在 2027-05 EOL 前完成迁移或形成有维护保障的替代策略；
5. Lyrical 仅在 Jetson Orin Ubuntu 26.04 官方 BSP 和所有外设驱动可用后再评估；
6. 不默认让不同 ROS distro 的节点共享一个 DDS domain；跨发行版时用 UDP/标准消息网关或独立网络域。

## 5. 关键技术链路

### 定位

`Lite3 leg odom + IMU → Local EKF → odom→base_link`

`RTK NavSatFix + trustworthy yaw + local odom → navsat_transform → GPS odometry → quality gate + Global EKF → map→odom`

### 避障

`MID-360 → PointCloud2 → filtering/self-filter/deskew → local costmap`

`2× RGB → sync/rectify/disparity/depth → confidence filtering/PointCloud2 → local costmap`

两类几何数据保留独立观测源；不将无效双目视差解释为空闲空间。

### Nav2

`Global Costmap + Planner + BT Navigator`

`Local rolling Costmap + Controller`

`Velocity Smoother → Collision Monitor → Lite3 command adapter → Lite3 motion SDK`

Collision Monitor 是软件附加防护，必须另有物理急停和底层 watchdog。

## 6. 载荷管理规则

Venture 标称载荷 4.5 kg。项目提出 3.6 kg 为所有外置计算/传感器/支架/线缆/电源器件总质量的内部设计目标，以保留约 20% 余量。该目标不是厂商认证值。

采购前必须称量：
- AGX 开发套件与散热件；
- MID-360；
- 两台相机、镜头和支架；
- RTK 板卡、天线、接地板/安装件；
- 交换机、DC/DC、电源分配、保险；
- 所有线缆、固定件和保护壳。

如果整机外设质量超过 3.6 kg，先优化计算平台/载板、外壳和支架；任何情况下不得突破厂商载荷和重心限制。

## 7. 采购前必须完成的 Gate

- [ ] DEEP Robotics 书面确认 Venture 电源额定、安装接口、载荷定义、SDK 版本和当前状态 topic。
- [ ] 取得 AGX 安装支架 CAD 和 Venture 上盖孔位图。
- [ ] 取得外设质量/功率总表，并满足 3.6 kg 内部目标或形成获批准的例外。
- [ ] 确认 MID-360 型号、线束和以太网配置。
- [ ] e-con Systems 确认 See3CAM_50CUG Color GPIO trigger、电气电平、同步模式、镜头规格、双机 UVC/Jetson 兼容。
- [ ] ArduSimple 确认 RTK 改正服务、天线、目标地区频段和部署方案。
- [ ] 验证 Humble 首轮镜像可构建并启动 Lite3 SDK bridge。
- [ ] 确认物理急停/遥控停止和底层 command watchdog。

## 8. 官方资料

- [DEEP Robotics Lite3 Product Page](https://www.deeprobotics.us/products/lite-3/)
- [DEEP Robotics Lite3 Perception Algorithm Development Example](https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/)
- [DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS)
- [DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)
- [Livox MID-360 Specs](https://www.livoxtech.com/mid-360/specs)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [e-con Systems See3CAM IMX264](https://www.e-consystems.com/sony-imx264-camera.asp)
- [ArduSimple simpleRTK2B Basic Starter Kit](https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/)
- [NVIDIA Jetson AGX Orin Developer Kit](https://developer.nvidia.com/embedded/jetson-agx-orin-developer-kit)
- [ROS REP-2000](https://www.ros.org/reps/rep-2000.html)
