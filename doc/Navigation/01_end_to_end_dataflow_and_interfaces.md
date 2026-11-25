# 01 — 端到端数据流与接口审计框架

## 1. 目标

建立从物理观测到导航决策、运动命令、实际执行反馈的完整 Producer/Consumer 图。每条边必须有可追踪的接口契约，不能只画节点而忽略时间、坐标、质量、超时和失败传播。

## 2. 数据链路矩阵（首轮系统级清单）

| 链路 | Producer | 中间处理/转换 | Consumer | 数据/接口候选 | 必须核验 |
|---|---|---|---|---|---|
| LiDAR | MID-360 | Livox driver、CustomMsg decode、deskew、滤波、地面/障碍处理 | 定位、感知、Costmap | CustomMsg/PointCloud2 | timebase、offset_time、frame、QoS、点频、丢包 |
| Stereo | 左右相机 | trigger、rectify、disparity/depth、置信度过滤、点云生成 | 感知、Costmap | Image/CameraInfo/DisparityImage/PointCloud2 | 硬同步、内外参、曝光、深度误差、USB带宽 |
| IMU | MID-360/本体 IMU | driver、轴向转换、bias/温漂、时间对齐 | LIO/VIO/EKF | sensor_msgs/Imu | frame、单位、协方差、频率、硬件时间 |
| GNSS | ZED-F9P | UBX decode、四路历元 join、RTK quality gate、navsat_transform | global estimator | UBX/NavSatFix/Odometry | iTOW、GPS week、Fixed标志、天线杆臂、协方差、PPS |
| Leg odom | 厂商运动控制器/SDK | vendor adapter、freshness、状态估计质量检查 | local estimator/Controller | nav_msgs/Odometry（待厂商确认） | 真实性、frame、stamp、covariance、速度语义 |
| Joint/contact | 厂商 SDK | 解码、关节映射、接触状态估计 | leg odom、terrain、diagnostics | JointState/自定义消息 | 单位、符号、足端定义、接触置信度 |
| Localization | EKF/LIO/VIO/SLAM | map alignment、GNSS fusion、relocalization | TF、Nav2、地图管理 | Pose/Odometry/TF | 单一 TF authority、连续性、covariance |
| Mapping | SLAM/map server | 地图过滤、版本、坐标注册 | localization/planner | OccupancyGrid/PointCloud/elevation | 地图 frame、分辨率、更新/保存 |
| Obstacle perception | LiDAR/stereo/camera | segmentation、tracking、classification | Costmap/traversability | PointCloud2/Detection3D/Tracks | false negative、动态时效、置信度 |
| Terrain | ground/elevation pipeline | slope/roughness/step/drop/foot placement | planner/controller/gait | grid_map/custom terrain | 空洞处理、风险模型、更新率 |
| Global planning | Route/Planner Server | graph/search/kinodynamic constraints | BT/controller | Path/Route | frame、规划代价、无解策略 |
| Local control | Controller/trajectory optimizer | collision/kinematic constraints | command arbiter | Twist/TwistStamped/trajectory | units、limits、latency、stability |
| Safety | safety supervisor/Collision Monitor/watchdog | arbitration、limits、timeout、E-stop state | platform adapter | safe command/stop | 独立性、故障默认态、响应时间 |
| Platform command | Lite3 adapter | protocol conversion、mode gate | vendor locomotion | vendor command | SDK authority、单位、坐标、ack、timeout |
| Execution feedback | vendor controller | decode、state estimation | controller/diagnostics | odom/status/joints/contact | actual vs commanded velocity、fault、stop latency |
| Runtime health | ROS diagnostics/OS/Jetson | metrics aggregation | supervisor/operator | DiagnosticStatus/metrics | CPU/GPU/temp/memory/network/clock |

以上为待核验的接口地图；消息类型是候选，不代表设备或驱动已经提供该接口。

## 3. 每条接口必须记录的字段

| 类别 | 字段 |
|---|---|
| 身份 | producer node/package/version、consumer node/package/version、interface name |
| 类型 | ROS message/action/service type、schema version、自定义字段语义 |
| 坐标 | frame_id、child_frame_id、轴向、单位、正负方向、姿态约定 |
| 时间 | header stamp 来源、clock domain、sync method、offset、drift、arrival time |
| 传输 | topic/action/service、QoS reliability/durability/history/depth、频率、带宽、队列 |
| 质量 | covariance、confidence、valid flags、calibration version、health/freshness |
| 行为 | timeout、drop policy、out-of-order、duplicate、reconnect、reset/generation |
| 安全 | stale-data policy、degraded mode、stop condition、command authority |
| 证据 | source URL/commit、测试设备、数据集、日志、验收层级 |

## 4. 从传感器到运动的逐级追踪

### 4.1 LiDAR 主链
MID-360 原始采样 → 设备时间戳/timebase → Livox SDK/ROS driver → CustomMsg → 点级 offset_time deskew → 自体过滤/地面分割 → obstacle cloud → costmap/elevation → Planner/Controller。验证 PointCloud2 转换是否保留时间/强度/环号/来源信息；若信息丢失，明确发生在哪一层。

### 4.2 双目辅助链
左右图像 + CameraInfo + 硬件触发 → 同步配对 → rectify → disparity/depth → 深度置信度/视差范围过滤 → 点云/目标检测 → 动态障碍与语义层 → Costmap/traversability。无效深度不能默认视为空闲空间。

### 4.3 定位链
IMU + leg odom/LIO/VIO + GNSS → 时间和外参统一 → quality gate → estimator → odom→base_link 与 map→odom → Nav2/地图/规划。禁止同一传感器信息被重复融合；GNSS 普通 STATUS_FIX 不可当 RTK Fixed。

### 4.4 规划到执行链
任务目标 → task manager/BT → route/global planner → path → local controller → velocity/trajectory intent → arbiter → safety gate/watchdog → Lite3 adapter → vendor locomotion → 实际机身/足端反馈 → estimator/controller。每个转换记录限幅、模式、ack、超时和拒绝原因。

## 5. 时钟、坐标和 QoS 专项

- 分别识别设备时钟、主机 monotonic、system wall clock、ROS time、simulation time。
- 不得把 host receive timestamp 当成 sensor acquisition timestamp，除非明确声明并量化不确定度。
- GNSS GPS week/iTOW、PPS、LiDAR timebase、相机 trigger、IMU 时间、ROS /clock 必须建立可转换关系。
- 对每个 TF edge 明确唯一 authority；禁止静态/动态重复发布。
- QoS 必须与驱动默认和消费者订阅配置一起核验，不能只抄文档示例。
- bag replay 必须记录 use_sim_time、/clock、TF buffer 和消息时间戳策略。

## 6. 失效传播矩阵

| 故障 | 直接检测 | 下游影响 | 默认策略（待风险分析定量） |
|---|---|---|---|
| LiDAR stale/断流 | 数据龄期/驱动诊断 | 障碍物缺失、LIO失效 | 降级或停止；不可静默继续 |
| Stereo sync/depth invalid | timestamp差/有效像素率 | 近场障碍漏检 | 隔离该源，按剩余传感器能力降级 |
| IMU/odom stale | freshness/covariance | TF/控制器失真 | 禁止依赖该定位继续运动 |
| GNSS quality drop | UBX flags/RTK gate | 全局位置精度下降 | 切换局部定位模式，限制任务范围 |
| TF missing/jump | lookup/continuity monitor | Costmap/规划错误 | 停止相关更新并触发安全策略 |
| Costmap stale/ghost | layer age/clearing monitor | 规划阻塞或穿障 | 重建/清图/停车 |
| Controller oscillation | progress/oscillation monitor | 无法稳定跟踪 | recovery 或停止 |
| Command/SDK timeout | ack/freshness/watchdog | 指令未执行/失控风险 | 独立停止/驱动禁能 |
| CPU/GPU/thermal throttling | system metrics | 端到端延迟增加 | 降频率/限速/停止 |

## 7. 后续工作

本文件是数据流审计模板和首轮系统清单。N1 阶段逐条补充具体 package/node/class、源码路径、接口定义、参数默认值、运行线程、真实频率、时延、QoS 和验证证据；未查明项保留待核验，不得编造 topic 名或消息字段。
