# 传感器时钟域与全系统时间同步架构审计

- 文档状态：N1 设计/证据审计，非实现
- 更新日期：2026-11-25
- 适用硬件预选：Livox MID-360；2 × e-con Systems See3CAM_50CUG_CXXCC_BX_H01R1；ArduSimple simpleRTK2B Budget / u-blox ZED-F9P；Jetson AGX Orin Developer Kit
- 关键边界：设备均为预选，未采购/未完成实机接线；Jetson NIC/PHC 能力、相机固件/触发电气参数、F9P 实际 SKU 与固件均待目标硬件证据

## 1. 审计结论

导航系统必须建立“时钟域 + 转换关系 + 不确定度 + 生命周期”的显式时间合同。以下对象不能仅因为都以纳秒或 ROS Time 表示就认为可直接比较：

1. GNSS 连续系统时间（GPST）及其 PPS 秒边沿标签；
2. PTP/gPTP Grandmaster 时间、NIC PHC（/dev/ptpN）；
3. Linux CLOCK_REALTIME（可被 NTP/PTP 校正，可能 step/slew）；
4. Linux CLOCK_MONOTONIC（用于本机时长、超时和 watchdog，不代表 UTC）；
5. ROS Time（真实运行时通常跟随 SystemTime；启用 use_sim_time 后跟随 /clock）；
6. MID-360 设备时间（依 time_type 区分未同步开机时间、PTP/gPTP、GPS）；
7. 相机传感器曝光时刻、相机内部计时器、USB 传输/主机接收时间；
8. 机器人本体 IMU、足端/关节控制器的设备时钟。

**禁止项：**
- 禁止把 ROS header.stamp、主机接收时间或 CLOCK_REALTIME 直接当成所有设备的采样时刻。
- 禁止把 CLOCK_MONOTONIC 与 GPST/UTC/PHC 的绝对时间直接比较。
- 禁止把 PTP、gPTP、GPS timestamp type 视为同一 epoch；必须先读出设备报告的同步模式和具体时间域。
- 禁止在运行过程中无记录地切换时间源、step 时钟或复用旧 offset。
- 禁止把仿真 /clock 回退当成真实硬件时钟回退，或把 ROS bag 回放时间用于硬件 watchdog。

## 2. 标准时间域分层

| ID | 时间域 | 典型来源 | 用途 | 不可直接推断 |
|---|---|---|---|---|
| TD-GNSS | GPST / GNSS 系统时间 | ZED-F9P NAV-PVT、GPS week/iTOW、PPS 标签 | GNSS 历元、外部时间基准 | 不能不经 UTC/GPS mapper 就当 UTC |
| TD-PHC | PTP Hardware Clock / GM | NIC PHC、PTP Grandmaster | 网络传感器硬件同步 | 不能假定每个 NIC 都支持硬件 timestamp |
| TD-WALL | Linux CLOCK_REALTIME | chronyd、phc2sys、系统 RTC | 系统日历时间、常规 ROS 实时时钟回退 | 可能被校正；不适合作为单调时长源 |
| TD-MONO | Linux CLOCK_MONOTONIC | 内核单调时钟 | timeout、watchdog、采样间隔、延迟测量 | 不带 GNSS/UTC epoch |
| TD-ROS | ROS Time | SystemTime 或 /clock | ROS 消息、TF、仿真和 bag | use_sim_time 改变来源；可能暂停/回退 |
| TD-LIDAR-0 | MID-360 boot-relative | time_type=0 | 未同步诊断/短时设备内部顺序 | 不能跨重启或跨设备比较 |
| TD-LIDAR-1 | MID-360 PTP/gPTP master time | time_type=1 | 网络同步点云/IMU | 必须确认 GM、PTP domain、PHC 路径 |
| TD-LIDAR-2 | MID-360 GPS time | time_type=2 | GNSS/PPS 同步点云/IMU | 不能无条件与 UTC wall time 比较 |
| TD-CAM | 相机 sensor/exposure/driver time | 传感器、trigger/strobe、UVC/V4L2 驱动 | 双目配对、VIO、动态障碍感知 | 驱动 timestamp 语义必须实测/查证 |
| TD-ROBOT | 本体 IMU/关节/足端时钟 | 机器人控制器或驱动 | 腿式里程计、状态估计 | 控制器时钟是否同步/是否重启未知 |

所有时间值必须同时携带 clock_domain_id、epoch_id、source_id、timestamp_semantics 和 uncertainty_ns。无法确认域或 epoch 时，时间戳状态为 UNMAPPED，不能参与精确传感器融合。

## 3. ROS 2 与 Linux 时钟职责

### 3.1 ROS Time 不是硬件同步协议

ROS 2 设计区分 SystemTime、SteadyTime、ROSTime。未启用 ROS Time Source 时，ROSTime 与 SystemTime 一致；use_sim_time 激活后，ROSTime 读取 /clock。/clock 尚未初始化时零值表示未初始化；回放/仿真可暂停、加速、减速或回退，回退需要清理时间相关状态。

因此：
- 驱动与硬件交互超时、串口/USB 超时、设备断连 watchdog：使用 steady/monotonic time。
- 对外发布 ROS 消息：按当前运行模式将采样时间映射到 ROS Time，同时保留原始设备时间和映射元数据。
- 真实机器人运行：所有节点必须统一 use_sim_time=false，并确保 wall/system clock 有明确校时源。
- 仿真/rosbag：所有参与仿真时间语义的节点统一 use_sim_time=true；启动 gate 等待 /clock 非零；时间回退时清理 TF、message_filters、积分器、scan/IMU 队列、滤波器状态及时间差缓存。
- 不得让真实硬件 watchdog、急停链路、通信租约依赖可暂停/回退的 ROS Time。

### 3.2 PHC、系统时钟与网络时间戳

Linux PTP infrastructure 可将 NIC PHC 暴露为用户空间时钟，并支持读取/校正频率/时间、外部事件 timestamp、PPS 相关功能；是否存在 PHC、是否支持硬件时间戳取决于实际网卡/驱动/板卡。

Linux SO_TIMESTAMPING 区分：
- RX software timestamp：数据进入内核接收栈附近；
- RX hardware timestamp：由网卡产生；
- TX hardware/software timestamp：不同处理位置；
- RAW_HARDWARE 读取原始硬件 timestamp。

因此“主机收到 UDP 包的时刻”不是 LiDAR 点云采样时刻。软件接收时间可用于网络延迟/抖动诊断，但不能覆盖设备采样 timestamp。

chrony 支持 PPS 和 PHC reference clock；PHC 若使用 TAI 而不是 UTC，配置必须明确 TAI/UTC offset 或 tai 语义。严禁只看 chronyc tracking 就推断传感器硬件 timestamp 与系统 UTC 已对齐。

**目标板卡验收命令（仅作为未来实机检查合同，不表示已执行）：**
- ls -l /dev/ptp*
- ethtool -T <lidar_nic>
- phc_ctl <nic> get
- pmc / ptp4l 状态检查
- chronyc tracking、chronyc sources -v
- timedatectl show
- 对照 CLOCK_REALTIME 与 CLOCK_MONOTONIC 的采样关系、PHC cross timestamp 能力和校时状态

若 Jetson AGX Orin 目标 NIC 不支持硬件 timestamp/PHC，必须重新评估外部 PTP NIC、GNSS timing board 或 PPS 直连方案；不允许默认以软件 PTP 达到硬件同步精度。

## 4. MID-360 时间同步合同

### 4.1 官方协议已确认字段

Livox MID-360 通信协议当前页面版本历史截至 2026-09-21 的 v1.4.12。点云 UDP 包头包含：
- time_interval：单位 0.1 μs，表示该包内最后点与首点时间差；
- time_type：时间戳类型；
- timestamp：该包第一个点云的 timestamp；
- 包内 N 个点云采样时间等间隔，总时间间隔由 time_interval 描述。

time_type：
- 0：无同步源，timestamp 是 LiDAR 上电后的设备时间，uint64 ns；
- 1：gPTP/PTP master clock 时间，uint64 ns；
- 2：GPS time，uint64 ns。

MID-360 支持 GPS 与 PTP 网络同步；PTP 支持 IEEE 1588v2.0 UDP/IP，gPTP 支持 L2。GPS 同步时间范围被协议限制为 2000-01-01 至 2037-12-31。协议注明 PTP 不支持 IEEE 1588v2.1，并不推荐 IEEE1588v2.0 与 gPTP 同时存在。

Livox 时间同步说明还明确：PTP 与 GPS 同时可用时，设备优先使用 PTP；网络不支持多个 master clock。故配置了 PPS/GPS 并不能证明 LiDAR 当前正在用 GPS time，必须检查 time_type/状态。

### 4.2 MID-360 诊断字段

设备信息 push 可读取：
- 0x8009 local_time_now：设备本地时间；
- 0x800A last_sync_time：最近一次同步 master 时间；PTP 时为 Follow_Up 中 t1，GPS 时为同步命令中的 PPS 上升沿时间；
- 0x800B time_offset：设备本地时间 t0 减同步源时间 t1，单位 ns；
- 0x800C time_sync_type：0 无同步，1 PTP，2 GPS；
- 0x800E lidar_diag_status：通信/时间同步模块诊断状态包含在状态位域内。

注意：time_sync_type 的状态字段将 1 描述为 PTP，而点云 time_type=1 描述为 gPTP/PTP；不能从这个状态字段单独区分 PTP 与 gPTP。

### 4.3 对现有驱动合同的约束

继续遵循 doc/Navigation/09：
- Livox CustomMsg：header.stamp 与 timebase 同源；每点时间 timebase_ns + offset_time_ns。
- PointXYZRTLT：逐点 timestamp 是另一种数据合同，不得与 CustomMsg 的相对 offset 混用。
- 驱动将设备时间写入 ROS header 时，必须说明该时间如何映射到当前 ROS Time 域。若 use_sim_time=true，不可将真实设备 epoch 的 timestamp 原样视为仿真 /clock epoch。
- 必须同时记录原始 time_type、设备 time_sync_type、last_sync_time、time_offset、固件版本和驱动版本；丢失这些状态时不能把该帧标为同步可信。

## 5. 双相机时间同步合同

### 5.1 已确认的产品能力

e-con 官方 See3CAM_50CUG 产品页将预选型号 See3CAM_50CUG_CXXCC_BX_H01R1 列为 5MP IMX264 Global Shutter、USB 3.2 Gen 1、UVC compliant，并列出外部 trigger 控制与 6-pin GPIO（I2C、Strobe、Trigger）。产品页还列出 Linux 支持及 QtCAM/OpenCView 等软件工具。

这足以把“外部触发”列为可研究的双目同步路径，但不足以证明：
- 两台具体 H01R1 样机在同一外部脉冲下可同时进入 sensor exposure；
- Trigger 是 sensor-level trigger 还是 ISP/GPIO/软件控制路径；
- 触发输入电压、极性、脉宽、最小周期、延迟和 jitter；
- strobe 输出是否精确对应 exposure start/end；
- UVC/V4L2 timestamp 是 exposure timestamp、frame-start timestamp 还是主机/驱动接收时间；
- 两台相机在 Jetson 上通过当前固件/SDK 可同时稳定采集 5MP@35fps。

### 5.2 必须向供应商确认并在实机验证

1. 对两台精确 SKU、硬件 revision、固件版本分别取得对应 Datasheet、Getting Started/User Manual、Extension Unit SDK API Manual。
2. 要求 e-con 书面确认两台 See3CAM_50CUG 是否支持共同外部 trigger 的连续视频同步（不是只支持 still capture/软件触发），以及 sensor exposure 的同步误差。
3. 取得 Trigger/Strobe 的电气规格：输入电压、绝对最大额定值、阈值、上升沿/下降沿、脉宽、隔离/共地、连接器针脚、输出驱动能力。未取得前禁止将 ZED-F9P PPS 或其他 GPIO 直接接入相机。
4. 设计公共硬件 trigger fanout；两相机各自 strobe 回采到逻辑分析仪/时间采集器，测量触发到 exposure 的固定延迟与 jitter、左右相机相对曝光偏差。
5. 分别验证 USB 帧到达时间和 sensor exposure 时间；USB 到达抖动只作传输诊断，不能替代曝光时间。
6. 若无法证明硬触发模式下的连续同步能力，则双目只能标为 SOFTWARE_TIMESTAMP_ONLY，不得作为高精度 stereo depth/VIO 输入；可保留为低速语义感知源，按实际运动模糊/时差评测。

## 6. GNSS PPS、PTP 与传感器的候选拓扑

### 方案 A：GNSS-disciplined PTP 主时钟（优先评估）

GNSS 天线/接收机提供 PPS + epoch/time label → 独立 GNSS-disciplined PTP Grandmaster → 具备硬件 timestamp 的 Ethernet/NIC/交换网络 → MID-360 PTP slave + 支持 PTP 的其他传感器；PTP PHC 与 Jetson CLOCK_REALTIME 通过受控 phc2sys/chrony 关系维护。相机若不支持 PTP，则由同一时基触发器产生同步 trigger，并采集 strobe 进行曝光时间映射。

优点：网络传感器共享主时钟；LiDAR 可使用 time_type=1；相机硬触发可形成曝光同步链。
阻塞：Jetson NIC/PHC、交换机透明/边界时钟能力、Grandmaster 产品、相机 trigger electrical/timing contract 尚未选定或实测。

### 方案 B：GNSS PPS/时间标签分配（备用评估）

ZED-F9P PPS + 对应秒标签 → 电气隔离/电平转换/扇出 timing board → MID-360 PPS + GPRMC/协议时间设置；相机公共 trigger fanout；本体 IMU/控制器按可用硬件同步输入接入。系统软件分别维护设备时间到 host/ROS 时间的映射。

Livox 官方文档给出 MID-360 GPS UART 模式使用 PPS 与 GPRMC，推荐 UART 连接；协议要求 GPS 输入为 GPRMC/GNRMC。其文档给出的 PPS 与 GPRMC 时序、电平要求必须纳入电气设计。不得将 F9P 输出直接连接到 MID-360/相机，直到确认双方引脚电压、极性、驱动能力和隔离要求。

### 方案 C：NTP/软件接收时间（仅降级）

在无 PTP/PPS/trigger 时可用 NTP/软件时间维持主机时钟与粗略跨设备时间关联。Livox 文档将 NTP 描述为毫秒级、低同步精度方案。它不能作为高动态双目深度、LIO 去畸变或紧耦合 VIO 的默认时间同步验收路径。此模式必须明确标记 DEGRADED_TIME_SYNC，并限制可用算法/速度/任务。

**尚未选定最终方案。** 方案 A 是首选研究方向，不是已批准采购/集成设计。若目标 Jetson/交换机/相机无法满足硬件能力，应在方案 A/B 之间基于实际测量作出决定。

## 7. 跨时钟域映射数据模型

每个设备到系统主时间域的映射都必须带版本与有效期，建议数据合同字段：

- source_id / device_serial / firmware_version
- source_clock_domain / target_clock_domain
- source_epoch_id / target_epoch_id
- mapping_generation（每次重启、重新锁定、时间源切换或 step 后递增）
- valid_from_source_ns / valid_until_source_ns
- offset_ns / rate_ratio（或 drift_ppb）
- uncertainty_ns / uncertainty_growth_ns_per_s
- sync_state：UNSYNCED / ACQUIRING / LOCKED / HOLDOVER / DEGRADED / LOST / INVALID
- last_sync_monotonic_ns / last_sync_source_ns
- time_source_type / grandmaster_identity / ptp_domain（适用时）
- raw_timestamp / mapped_timestamp / timestamp_semantics

允许用分段仿射模型表示 source→target 映射：
t_target = a * t_source + b，其中 a 描述频率偏差/漂移，b 描述偏移；模型只在同一 generation 与有效区间内有效。必须通过测量估计误差和不确定度；不得用一个长期固定 offset 覆盖时钟漂移或重启。

### Generation 变化触发条件

- 设备 reboot/powerup counter 变化；
- MID-360 time_type 或 time_sync_type 变化；
- PTP GM identity/domain 变化、PTP state 从 LOCKED 转为 LOST/HOLDOVER；
- PHC/system clock step 或 time source switch；
- GNSS week/iTOW/epoch reset、PPS 丢失或时间标签无法配对；
- 相机固件重启、USB 断连重连、trigger 丢脉冲或 strobe 回采失配；
- ROS /clock backward jump / zero-uninitialized；
- 本体 MCU reboot 或时钟重置。

Generation 不一致的消息禁止进入同一个精确时间配对窗口。队列应清空或隔离，滤波器按其状态模型执行 reset/reinitialize，而不是仅重排消息。

## 8. 时间同步健康状态与降级策略

| 状态 | 条件 | 允许行为 |
|---|---|---|
| LOCKED | 时钟源身份/域明确；offset、drift、uncertainty 在批准阈值内；状态持续稳定 | 允许对应融合/去畸变 |
| ACQUIRING | 已检测到时钟源但尚未满足稳定窗口 | 缓存/隔离，不参与高精度融合 |
| HOLDOVER | 外部同步丢失，设备继续自由运行 | 根据 uncertainty growth 与 holdover 上限逐步降级 |
| DEGRADED | 使用软件/NTP/host receive timestamp 或超过精度预算 | 仅允许已评估的低精度感知功能 |
| LOST | PPS/PTP/trigger 丢失或时钟源不可达 | 停止相关精确配对；按传感器冗余策略降级 |
| INVALID | 时间回退、epoch 不明、generation 不匹配、非法值或时钟域未知 | 丢弃消息/重置相关队列，不得参与融合 |

阈值不得在没有算法预算和实测依据时凭空设定。后续应分别为 LIO deskew、stereo depth、VIO、GNSS fusion、动态障碍跟踪分配最大时间误差预算，并从运动速度/角速度、视差、点云扫描周期和估计器容忍度反推阈值。

## 9. 必须补齐的测试合同

| ID | 测试 | 方法/通过条件 |
|---|---|---|
| TIME-001 | ROS clock 初始化 | /clock 未发布/零值时，仿真节点不得将零时间戳当有效采样时间 |
| TIME-002 | ROS clock 回退 | rosbag rewind/仿真回退时，时间回调完成后 TF/message_filters/LIO 队列/积分器清理；无跨 epoch 配对 |
| TIME-003 | MONOTONIC watchdog | 暂停 /clock 不得暂停硬件超时、设备断连 watchdog 和 E-stop 计时 |
| TIME-004 | PHC 能力清点 | 实机记录 /dev/ptp*、ethtool -T、PHC identity、硬件 RX/TX timestamp 能力；不支持则方案 A 不得标记可用 |
| TIME-005 | PTP GM 唯一性 | 多 GM/GM 切换/Domain 错配时检测并产生 generation 变化，禁止静默混合 |
| TIME-006 | MID-360 模式判别 | time_type=0/1/2 分别映射正确 domain；模式变化时 flush 队列并递增 generation |
| TIME-007 | MID-360 offset 诊断 | 记录 local_time_now/last_sync_time/time_offset/time_sync_type；offset 符号按设备 t0−t1 定义，不得反号 |
| TIME-008 | MID-360 GPS 优先级 | PTP 与 GPS 同时可用时核实设备采用 PTP；系统不得仅凭已接 PPS 就假定 GPS 模式 |
| TIME-009 | PPS 丢失/恢复 | 注入 PPS 丢脉冲、GPRMC 延迟/乱序、错误秒标签；检测失锁、holdover、重锁和 generation 变化 |
| TIME-010 | 相机双机触发 | 同一 trigger 输入两台相机；以 strobe/曝光测量量化左右曝光偏差和 jitter，达到由 stereo/VIO 预算确定的阈值 |
| TIME-011 | 相机 timestamp 语义 | 对比传感器 trigger/strobe 与 ROS Image header.stamp、V4L2 buffer timestamp、USB 接收时刻，确认 timestamp 来源和延迟 |
| TIME-012 | USB 负载/丢帧 | 双 5MP 相机满负载、USB bus contention、CPU/GPU 压力下测量帧丢失、乱序和时间偏差 |
| TIME-013 | 系统 clock step/slew | 对 wall clock 执行受控 step/slew 测试，确认 monotonic watchdog 稳定、设备时间映射 generation 正确变化 |
| TIME-014 | PHC↔system cross timestamp | 测量 PHC 与 CLOCK_REALTIME 映射误差、漂移和不确定度；检查校时服务竞争 |
| TIME-015 | 设备重启与重连 | LiDAR、相机、GNSS、机器人控制器分别重启；无跨 reboot epoch 的消息配对 |
| TIME-016 | rosbag 记录/回放 | 同时记录原始时间域、ROS stamp、/clock、sync diagnostics、generation；验证 replay 与 real-time 数据不混淆 |

测试阈值、目标相机 trigger 规格、目标 Jetson NIC/PHC 能力、同步拓扑和实际硬件采样均未完成。以上是待执行合同，不代表通过。

## 10. 当前证据边界与下一步

### 已由公开资料确认
- ROS 2 的三种时间抽象、use_sim_time、/clock 零值未初始化和回退行为。
- Linux PHC 与 socket timestamping 对硬件/软件时间戳的区分。
- chrony 的 PPS/PHC reference clock 能力及 PHC TAI/UTC 语义要求。
- MID-360 的 PTP/gPTP/GPS 支持、timestamp type、包内首点 timestamp、time_interval、同步诊断字段和 PTP 优先级。
- See3CAM_50CUG 产品系列为 IMX264 global shutter、USB 3.2 Gen 1，官方产品页列出外部 trigger 与 6-pin GPIO。

### 未完成/需要实物或授权
- Jetson AGX Orin 实际载板、Ethernet NIC/PHY、驱动、PHC 节点及 hardware timestamp 实测。
- PTP Grandmaster/交换机/线缆/隔离与 PPS fanout 方案选型。
- 两台目标相机的精确硬件 revision、固件、触发模式、电气参数、连续视频硬触发能力和 exposure timestamp 语义；需供应商完整手册/书面确认。
- ZED-F9P 实际 SKU、固件、PPS 输出配置、电平和精确秒标签映射。
- Lite3 本体 IMU/关节/足端时间域、控制器时钟同步能力与 reboot 行为。
- 目标硬件上的偏差、漂移、抖动、延迟、丢帧和 holdover 测试数据。

### 下一步顺序
1. 获取 See3CAM_50CUG Color 完整 Datasheet、Getting Started Manual、Extension Unit SDK API Manual，核对 trigger/strobe 电气与连续采集模式。
2. 盘点 Jetson 载板 NIC/PHY 的 PHC 与硬件 timestamp 能力；无板卡实物前不锁定 PTP 拓扑。
3. 对比 GNSS-disciplined PTP GM 与 GNSS PPS/time-label fanout 方案的器件/接口/精度/故障模式，形成 ADR 后再决定。
4. 明确 stereo/LIO/VIO 算法误差预算，推导 TIME-010 等同步阈值。
5. 实际接线后先做独立时钟/trigger 测量，再接入 ROS 2；不得直接进入全栈调参。



## 12. See3CAM_50CUG 彩色型号与 Jetson AGX Orin PTP 证据增量

### 12.1 目标相机 SKU 与手册证据边界

项目预选的精确型号是 See3CAM_50CUG_CXXCC_BX_H01R1（Color、无镜头、带 enclosure）。e-con 官方产品页将该 SKU 列在 See3CAM_50CUG 彩色型号变体中；同页说明该系列为 IMX264 5MP Global Shutter、USB 3.2 Gen 1，支持外部 Trigger，并列出 6-pin GPIO（I2C、Strobe、Trigger）。产品页同时列出 Color 版本的 Datasheet、Getting Started Manual 和 Extension Unit SDK API Manual 下载入口。

但是本轮实际可读取的完整电气 Datasheet 是 See3CAM_50CUGM（Monochrome）Revision 1.1，2023-03-28，且其机械型号表包含 H02R1。它不是项目预选的 Color H01R1。该单色手册只可作为同系列 GPIO 设计参考，**不能把其中的电气阈值/针脚细节当作 Color H01R1 的正式额定值**。

单色手册中可读到的参考参数（仅限 50CUGM，不可直接用于彩色 SKU）：
- Trigger 输入：VIL 0–0.63 V；VIH 最低 1.17 V。
- Strobe 输出：VOL 最大 0.4 V；VOH 最低 1.4 V。
- GPIO 推荐输入电压 1.8 V；绝对最大 DC 输入 3.6 V。
- GPIO 表列 CN6 pin 1 TRIG、pin 5 STROBE、pin 6 GND；TRIG 描述为 1.8 V external trigger input，STROBE 描述为 1.8 V sensor strobe output。

上述数据表明即使同系列单色型号也不是可随意接收 3.3 V/5 V 的通用 GPIO。由于彩色型号电气手册尚未取得，禁止据此设计或接线。**在拿到 Color H01R1 对应 revision 的正式 datasheet 前，不得连接 F9P PPS、Jetson GPIO 或第三方 trigger fanout 到相机。**

### 12.2 软件触发与连续视频边界

e-con 产品页 FAQ 对 software trigger 的描述是：相机进入该模式后，在通过 HID interface 收到触发命令前不会 stream frames，且可控制捕获帧数。该模式是命令驱动的 capture 行为，不等价于可持续运行的双相机硬件外触发同步流。

因此：
- 软件 trigger 仅作为单帧/有限帧控制功能单独评估；
- 不得把软件 trigger 当作高动态 stereo/VIO 的连续同步机制；
- 产品页列出 external trigger 只证明产品系列提供该能力入口，不证明两台目标 H01R1 在目标固件/SDK 下可连续同步曝光；
- 必须从 Color H01R1 Datasheet + Getting Started Manual + Extension Unit SDK API Manual 核实 trigger mode、continuous stream 行为、曝光相位、strobe 与曝光窗口关系、触发延迟/jitter、帧率上限和双机使用限制；
- 对 e-con 提出书面问题：两台相同 H01R1 是否可由同一硬件脉冲持续触发；sensor exposure start 的左右偏差最大值/典型值；是否有 exposure active strobe；strobe 极性、输出驱动、电气时序；trigger 最小脉宽/周期、输入阈值和最大频率；是否需要 SDK/XU 固件配置。

### 12.3 Jetson AGX Orin 的 PTP 硬件时间戳能力

NVIDIA 官方 Developer Guide 对 AGX Orin 平台说明：系列可配置最多一个 MGBE (XFI) 与一个 RGMII 端口；P3737 carrier board 设计并验证了一个 MGBE 端口；自定义 carrier 的接口启用和 PHY 配置需要单独适配。NVIDIA 开发者论坛中 AGX Orin 32GB Developer Kit 用户给出 ethtool -T 结果，显示 hardware-transmit、hardware-receive、hardware-raw-clock 和 PTP Hardware Clock: 0；另一 AGX Orin Developer Kit 讨论也展示 eth1 对应 PHC 0 的硬件时间戳能力。NVIDIA 论坛工程人员确认 AGX Orin MGBE 路径支持硬件 PTP；Orin NX/Nano 的第三方 PCIe Ethernet 路径不应假设具备相同支持。

据此更新架构判断：
- 方案 A（GNSS-disciplined PTP GM）在 Jetson AGX Orin Developer Kit 上具备可行的公开硬件证据，不再是完全未知项；
- 但证据不能替代本项目目标板实测，尤其不能外推到任意第三方载板、外接 USB/PCIe 网卡或不同 Ethernet interface；
- 需要在目标板上逐个接口执行 ethtool -T <iface>，记录 interface driver、carrier board ID、JetPack/L4T、kernel、PHY、/dev/ptpN 绑定关系，并用 ptp4l 确认端口进入 SLAVE/MASTER/UNCALIBRATED 等正确状态；
- phc2sys 应按 ptp4l 的 PHC 设备关系配置；不可简单把 interface 名当作 /dev/ptpN 参数使用，须核对本机 linuxptp 版本的参数语义和 -w/-s/-c 方向；
- 需测量 PHC↔CLOCK_REALTIME cross timestamp、GM 切换、link flap、PHC reset 和 holdover；仅 ethtool -T 显示能力不等于 PTP servo 已锁定，更不等于 MID-360 与 Jetson 的端到端偏差达到算法预算。

NVIDIA Jetson Linux 还提供 HTE（Hardware Timestamp Engine）文档，说明可对部分 LIC IRQ 和 AON GPIO 状态变化进行硬件时间戳采集。该能力可作为“相机 Strobe 回采”的候选诊断路径，但目标引脚是否为可用 AON GPIO、pinmux/HTE 驱动是否启用、外部电压兼容性、timestamp clock domain 和中断延迟均需目标板实测；不能把 HTE 视为自动可用的相机曝光时间戳接口。

### 12.4 新增验收合同

| ID | 测试 | 方法/通过条件 |
|---|---|---|
| TIME-017 | Color H01R1 电气合同 | 获取与 See3CAM_50CUG_CXXCC_BX_H01R1 硬件 revision 匹配的正式 datasheet；核实 Trigger/Strobe pinout、VIL/VIH、VOH/VOL、绝对最大额定值、输入脉宽/频率、驱动能力；未取得时禁止接线 |
| TIME-018 | Color H01R1 双机连续触发 | 使用两台同 SKU/固件，验证硬件 trigger continuous stream、左右 exposure active/strobe 相位差、触发到曝光延迟/jitter、丢脉冲/恢复和 USB 负载；用外部逻辑分析仪/时间采集器测量，不以 USB arrival time 代替曝光时间 |
| TIME-019 | AGX Orin PHC 绑定 | 目标载板逐接口记录 ethtool -T、driver、/dev/ptpN、carrier EEPROM/JetPack/L4T；验证 PHC 与网口映射和硬件 TX/RX timestamp |
| TIME-020 | AGX Orin PTP servo 与故障 | 对照 GM identity/domain、ptp4l port state、offset/path delay、phc2sys 方向及 PHC↔system cross timestamp；注入 GM/link/PHC reset，验证 generation 变化和恢复 |
| TIME-021 | HTE Strobe 回采可行性 | 确认目标 pin 的 AON GPIO/HTE 支持、pinmux、电压与输入保护；对相机 Strobe 与外部基准同时采集，测量硬件 timestamp 与 trigger/exposure 的关系；无实测不作为生产时间源 |

## 13. 本轮结论修订

1. **相机：** Color H01R1 产品级 external trigger/GPIO 能力已由 e-con 官方产品页确认；对应彩色型号电气 datasheet 和完整 trigger/stream API 尚未读取。50CUGM 单色 datasheet 仅作为系列参考，参数不得迁移到 Color H01R1。
2. **Jetson：** AGX Orin Developer Kit 的 MGBE/PHC 硬件 PTP 有 NVIDIA 平台文档与官方论坛实测支持，方案 A 获得正向可行性证据；自定义 carrier/实际网口/软件版本仍必须验收。
3. **双机同步：** 未证明连续双机 exposure synchronization，当前仍为 CAPABILITY_LISTED / CONTINUOUS_STEREO_SYNC_UNVERIFIED。
4. **实现边界：** 不接线、不实现驱动、不宣称 PTP lock，不执行硬件测试；等待确切彩色相机手册与目标计算板硬件信息。


## 14. 来源
- ROS 2 Clock and Time design：https://design.ros2.org/articles/clock_and_time.html
- Linux PTP Hardware Clock infrastructure：https://docs.kernel.org/driver-api/ptp.html
- Linux socket timestamping：https://docs.kernel.org/networking/timestamping.html
- chrony chrony.conf(5)：https://chrony-project.org/doc/latest/chrony.conf.html
- Livox MID-360 Communication Protocol：https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
- Livox Time Synchronization Instructions：https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html
- e-con See3CAM_50CUG Color/Monochrome product page and SKU/manual listing：https://www.e-consystems.com/sony-imx264-camera.asp
- e-con See3CAM_50CUGM Monochrome Datasheet Rev 1.1 (series reference only; not Color H01R1)：https://www.e-consystems.com/Downloads/See3CAM_50CUGM/e-con_See3CAM_50CUGM_Datasheet.pdf
- NVIDIA Jetson AGX Orin Platform Adaptation and Bring-Up：https://docs.nvidia.com/jetson/archives/r39.2.2/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonAgxOrinSeries.html
- NVIDIA Jetson Linux Hardware Timestamp Engine (HTE)：https://docs.nvidia.com/jetson/archives/r38.4/DeveloperGuide/SD/Kernel/GenericTimestampEngine.html
- NVIDIA Developer Forum — AGX Orin 32GB ethtool -T PHC evidence：https://forums.developer.nvidia.com/t/does-the-jetson-agx-orin-32gb-support-hardware-timestamping-for-ptp/337720
- NVIDIA Developer Forum — AGX Orin Developer Kit PTP/PHC interface evidence：https://forums.developer.nvidia.com/t/tsc-edge-out-functionality-on-jetson-agx-orin-developer-kit/335812

未修改生产代码；未运行构建、单元测试、仿真或实机测试。
