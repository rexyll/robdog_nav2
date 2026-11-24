# 硬件与软件接口确认清单

该清单与根目录 [TaskPlan.md](../../TaskPlan.md) 一起维护。**TaskPlan.md 是唯一进度台账**；本文件保留具体规格和接口确认结果。

## 0. 已选型候选总览

| 类别 | 技术预选 | 状态 |
|---|---|---|
| 四足本体 | DEEP Robotics Lite3 Venture | 已预选，未采购，待厂商确认现行 SKU/接口 |
| LiDAR | Livox MID-360 | 已预选，待采购和驱动验证 |
| RGB 相机 | 2 × e-con Systems See3CAM_50CUG Color | 已预选，待确认 GPIO 同步、镜头和 Linux Jetson 兼容 |
| RTK | ArduSimple simpleRTK2B Budget / u-blox ZED-F9P + ANN-MB | 已预选，待确定 NTRIP / Base |
| 机载计算 | NVIDIA Jetson AGX Orin Developer Kit（64GB 开发配置） | 已选参考配置，待核算安装、供电和质量 |
| 软件首轮基线 | Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble | 对齐云深处 Lite3 Venture 官方扩展示例；需实机复现 |
| 软件长期目标 | ROS 2 Jazzy | 自研包可移植；Humble EOL 前完成迁移 |

详细选型依据见 [ADR-0002](../decisions/ADR-0002-hardware-platform.md)。

## 1. 机器狗本体：DEEP Robotics Lite3 Venture

| 项目 | 当前资料/预选 | 需确认内容 | 状态 |
|---|---|---|---|
| 型号 | Lite3 Venture | 当前销售 SKU/版本/配件清单 | 待供应商确认 |
| 本体公开质量 | 约 12.2 kg | 交付批次与选装件质量 | 待确认 |
| 标称载荷 | 4.5 kg | 载荷定义、上装重心/动态运动限制 | 待确认 |
| 外部通信 | 2 × Ethernet | 端口用途、速度、IP/UDP 通讯方式 | 待确认 |
| 外部供电 | 24V/12V/5V | 每路持续/峰值电流和总功率限制 | **阻塞项** |
| 固件/SDK | Lite3 SDK / sdk_deploy | 当前支持版本、授权、维护状态 | 待确认 |
| ROS 驱动 | Lite3_ROS / lite3_transfer | Humble 编译和 Jazzy 迁移状态 | 待实测 |
| IMU/leg odom | /imu/data、/leg_odom2（官方示例接口） | 频率、时间戳、frame、covariance、IMU 是否已融合 | 待实测 |
| 运动命令 | /cmd_vel Twist 示例 | 最大速度、加速度、横移语义、死区和模式 | 待确认 |
| 安全 | 机器人 stop/遥控急停 | API、watchdog、通信断开默认行为 | 待确认 |
| 安装 | Venture + AGX 安装组件 | CAD、孔位、允许载荷分布和线缆路径 | 待获取 |

## 2. LiDAR：Livox MID-360

| 项目 | 预选资料 | 实施确认 |
|---|---|---|
| 输出 | PointCloud2 / Livox CustomMsg（依驱动配置） | 固定使用的消息类型和 topic |
| 视场 | 水平 360°，垂直 -7° 至 +52° | 安装姿态、遮挡和机身俯仰后的覆盖 |
| 点率/帧率 | 约 200k points/s、典型 10 Hz | CPU/网络吞吐与实际更新率 |
| 同步 | PTPv2/GPS | 主机时钟同步方案 |
| 供电 | 9–27V；Lite3 官方扩展示例展示 Venture 24V XT30 供电 | 电流、线束、保险、接口额定 |
| 驱动 | Livox SDK2 + livox_ros_driver2 | 锁定 commit，Humble/Jazzy 编译 |
| 安装位姿 | 待测 | lidar_link 相对 base_link 的 xyz/rpy |

## 3. 两台普通 RGB 相机：See3CAM_50CUG Color

| 项目 | 预选 | 待确认 |
|---|---|---|
| 型号 | 两台同型号 See3CAM_50CUG Color | 完整 SKU、批次和固件 |
| 传感器 | Sony IMX264，5MP，Global Shutter | 最终分辨率/FPS和图像格式 |
| 接口 | USB 3.2 Gen1/UVC | Jetson 双相机同时运行带宽、USB控制器拓扑 |
| 同步 | GPIO Trigger/Strobe | 触发引脚、逻辑电压、电平转换、触发延迟、曝光同步 |
| 镜头 | 同型号 C-mount 镜头 | 2/3 inch image circle、HFOV 70°–80° 候选 |
| 默认视场 | 页面列出的配套镜头 HFOV 约 26.1° | 不默认采用；需要按机器人需求重新选镜头 |
| 基线 | 初始试验值 100–120 mm | 结合近远距离误差和遮挡做最终定型 |
| 标定 | ROS CameraInfo、stereo calibration | 保存原始样本、内外参、rectification 和质量报告 |
| ROS 驱动 | UVC camera driver + image_pipeline | 选包并固定版本，验证 Humble/Jazzy |

双目相机不输出天然深度；需要 stereo rectify、disparity、depth、confidence 和 PointCloud2 处理流水线。

## 4. RTK：simpleRTK2B Budget / ZED-F9P

| 项目 | 预选 | 待确认 |
|---|---|---|
| GNSS 模块 | u-blox ZED-F9P | 板卡硬件 revision 和固件 |
| 天线 | ANN-MB 多频天线 | 线长、安装高度、地板/接地板要求 |
| 数据 | NavSatFix + UBX 原始状态 | ROS 驱动、状态码、协方差 |
| 更新率 | 最高性能设置可达 10Hz | 实际串口/USB配置和 CPU 流量 |
| RTK 改正 | NTRIP 或自建 Base | 目标场地覆盖、服务费、网络可用性 |
| 航向 | 单天线无静止 yaw | 初版采用 Lite3 IMU/odom；后续可双天线升级 |
| 杆臂 | 天线相对 base_link | 真实安装坐标和测量误差 |

## 5. 计算平台与供电

| 项目 | 预选 | 待确认 |
|---|---|---|
| 计算机 | Jetson AGX Orin Developer Kit | 最终内存版本、载板接口和库存 |
| 功率 | 模块/开发套件随工作模式变化 | GPU/CPU 满载功率、散热和环境温度 |
| USB | 两路 USB3 相机 | 是否共享同一 USB controller、总带宽 |
| Ethernet | MID-360 + Lite3 | 交换机拓扑、IP 网段、UDP 延迟 |
| RTK | USB/UART | 电气隔离/串口电平/接地 |
| 供电 | Lite3 外设口或独立供电 | 24V→AGX 所需电压的 DC/DC 额定、保险和浪涌 |
| 质量预算 | Venture 4.5kg 标称载荷 | 项目外设目标 ≤3.6kg，称量每个子件与线缆 |
| 结构 | 上装安装板、减振和天线支架 | CAD、质心、机身振动与跌倒冲击 |

## 6. ROS 2 / 软件兼容

| 组件 | 首轮基线 | 长期策略 |
|---|---|---|
| Lite3 SDK / bridge | Ubuntu 22.04 + Humble | 隔离在 bridge，迁移 Jazzy |
| MID-360 driver | 官方 Livox SDK2 / livox_ros_driver2 | Humble/Jazzy 分别构建测试 |
| Camera driver | UVC camera driver | Humble/Jazzy API兼容 |
| Stereo | image_pipeline / 自选 stereo matcher | 固定版本与算法模型 |
| Localization | robot_localization + navsat_transform | 标准 NavSatFix/Odometry 接口 |
| Navigation | Nav2 | Jazzy 为长期版本目标 |
| NTRIP | GNSS correction client/service | 与厂家协议和地区服务匹配 |

ROS 2 Humble 计划于 2027 年 5 月结束支持，首轮对齐厂商公开示例，但必须在 TaskPlan 中跟踪 Jazzy 迁移。

## 7. 场景与验收

- 运行环境：室内/室外混合，待明确。
- 首轮地面：平整硬质地面。
- 目标障碍：人、箱体、墙、桌椅、货架、细杆、玻璃和低矮物。
- 测试指标：RTK 精度/降级；local odom 连续性；双目深度误差；障碍物召回率；感知时延；停车距离；系统功耗和续航。
- 所有指标阈值以根目录 TaskPlan A9 为准。
- 实机自主运动前必须确认急停/遥控停止和底层 watchdog。

## 8. 阶段 A 确认结果

- 预选本体：Lite3 Venture。
- 预选 LiDAR：Livox MID-360。
- 预选双目：2 × See3CAM_50CUG Color。
- 预选 RTK：simpleRTK2B Budget / ZED-F9P。
- 预选计算：Jetson AGX Orin Developer Kit。
- 未采购、未测量、未上机、未完成构建。
- 后续所有阶段状态统一记录在根目录 TaskPlan.md。
