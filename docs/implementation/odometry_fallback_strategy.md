# 本体里程计缺失时的替代方案与 LIO/VIO 评估

- 日期：2026-11-24
- 状态：**架构预案完成；Point-LIO 为首选备用 POC，尚未在 Jetson AGX Orin + MID-360 实机验证**
- 唯一进度台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 相关决策：[ADR-0004](../decisions/ADR-0004-local-odometry-fallback.md)
- 相关时间同步设计：[time_sync_and_calibration.md](time_sync_and_calibration.md)

## 1. 为什么需要这个方案

Nav2 需要连续的局部坐标系和本体变换，通常至少需要可信的 `odom → base_link`。当前 Lite3 新版 `sdk_deploy` 公开 DRDDS 消息包括 IMU、关节和 RobotStatus，但公开 RobotStatus 并不是完整的 `nav_msgs/Odometry`。旧版 `Lite3_ROS` Foxy 示例有 `/leg_odom2`，但不能据此推断当前 Venture 固件一定提供同样接口。

因此，不能把“Lite3 自带腿式里程计”作为无条件成立的前提。项目必须同时准备：
- 首选：厂商提供并经过审计的腿式里程计/状态估计；
- 备用：MID-360 + 其 IMU 的 LiDAR-Inertial Odometry（LIO）；
- 第二备用：双目/视觉惯性里程计（VIO），在 LiDAR LIO 无法满足场景时评估。

## 2. 当前建议的定位数据链

### 路线 A：厂商腿式 odometry 可用且通过验证

```text
Lite3 legged odometry + Lite3 IMU / contact state
             |
      validate timestamp / frame / covariance
             |
      Local state estimator (only if needed)
             |
         odom -> base_link
             |
RTK -> navsat_transform -> Global EKF -> map -> odom
```

- 厂商估计器若已融合 IMU，不得将同一惯性信息当作统计独立观测重复融合。
- LIO 作为离线/在线对照与故障监测来源，只有通过相关性分析、时间同步和 covariance 验证后才考虑融合。
- 厂商 odometry 在足滑、急转、蹦跳、坡面和台阶场景必须验证，不可仅凭平地 demo 通过。

### 路线 B：厂商没有可用腿式 odometry（首选备用方案）

```text
Livox MID-360 point cloud (per-point time)
             +
MID-360 built-in IMU
             |
       PTP/GPS time validation
             |
       Point-LIO (first POC)
             |
  lidar/imu odometry + registered cloud
             |
   calibrated imu_link -> base_link
             |
         odom -> base_link
             |
RTK -> navsat_transform -> Global EKF -> map -> odom
```

**为什么首先测试 Point-LIO：**
- Point-LIO 是逐点更新的 LiDAR-Inertial Odometry，不必等整帧扫描结束；上游论文/README 强调高带宽输出、对 IMU 饱和和强振动/剧烈运动的鲁棒性。
- Intel Robotics AI Suite 2026.3 文档将 Point-LIO 列为 ROS 2 Humble/Jazzy 已移植的 LIO，并说明其适合高动态 humanoid/quadruped 平台；其 ROS 2 Jazzy port 含 Livox ROS Driver 2 和 MID-360 配置。
- Intel 文档的端到端验证实际使用 UrbanLoco + Velodyne 数据集，且执行性能验证的平台是 Intel Core Ultra，不是 Jetson AGX Orin，也不是本项目的真实 MID-360；因此这里只能证明 ROS 2/Jazzy 集成路径有参考实现，不能证明 Jetson 上性能已通过。
- Point-LIO 上游许可为 BSD-3-Clause。仍需审查选定 ROS 2 fork/patch 的每个依赖与许可证。

**备选对照：FAST-LIO2**
- FAST-LIO2 对 Livox 非重复扫描和原始点云直接配准有成熟参考，Intel 2026.3 文档给出 Ubuntu 24.04 / ROS 2 Jazzy + Livox ROS Driver 2 的 build flow。
- Intel 该 FAST-LIO2 分支的实际 replay 验证采用 Velodyne/UrbanLoco 数据，未证明 Jetson + MID-360 实机通过。
- 上游 FAST-LIO 许可是 GPLv2（其 package.xml 中 BSD 声明与 LICENSE 不一致，以 GPLv2 为准）；商用交付需先完成许可评估或联系作者取得其他许可。
- FAST-LIO2 作为对照 benchmark，不先定为产品首选。

### 路线 C：LIO 失败时评估双目 VIO

```text
left/right synchronized images + CameraInfo
             +
validated IMU stream + camera/IMU time offset
             |
      stereo visual-inertial odometry
             |
         odom -> base_link
             |
RTK -> global position constraint -> map -> odom
```

- 两台 See3CAM 普通 RGB 相机需要准确的内参/双目外参、共触发和与 IMU 的时间关系；只做左右图同步并不意味着 camera-to-IMU 已同步。
- 透明、反光、无纹理、弱光、运动模糊和遮挡会导致视觉 tracking 退化。
- VIO 不作为本项目第一备用方案，因为 MID-360 已提供更稳定的几何测距和可用的内置 IMU；先验证 LIO 更直接。
- VIO 候选软件需要分别审查 ROS 2 Jazzy 支持、Jetson ARM64/GPU 性能、双目输入接口和 GPL/商业许可；不在未核验前锁定具体算法。

## 3. MID-360 内置 IMU 的用法和限制

MID-360 产品资料列出内置 IMU；Livox ROS Driver 2 源码中存在 IMU 消息发布路径，并将设备 IMU 时间戳转换为 ROS stamp。Point-LIO/FAST-LIO2 的 MID-360 示例配置常见话题为 `/livox/lidar`、`/livox/imu`，点云使用 Livox CustomMsg 携带每点时间偏移。

实施规则：
1. LIO 第一版优先使用 MID-360 自己的 LiDAR+IMU 时间域，避免将 Lite3 IMU 未经同步直接混入 LIO。
2. 点云必须保留逐点采样时间（Livox CustomMsg/offset_time）；仅一帧 header stamp 不足以完成逐点去畸变。
3. IMU 和 LiDAR 必须同步。MID-360 可配置 PTPv2/GPS 时间，但仍需在目标固件、驱动、主机网卡上测量 offset/drift 和消息时间戳语义。
4. 不要开启自动 `time_sync_en` 或使用默认 `time_offset_lidar_to_imu=0` 就宣称同步已完成；需按算法说明先校准/验证。
5. 测量并标定 `imu_link ↔ lidar_link` 外参；Point-LIO 配置中平移/旋转外参的坐标定义必须按实际源版本核验。
6. MID-360 内置 IMU 的饱和量程、输出率、单位和轴向必须来自当前设备配置/数据，不能照抄其他平台参数。
7. 若使用 MID-360 内置 IMU 作为 LIO 输入，Lite3 IMU仍可作为独立诊断/姿态参考；只有时间、坐标和误差相关性被验证后才考虑融合。

## 4. 与 robot_localization / RTK 的组合

建议结构：
- LIO 发布局部连续 `nav_msgs/Odometry` 和 `odom → imu_link/base_link`（实际 child frame 由配置确定）。
- 使用经过标定的静态外参将 LIO 的 body pose 正确表达为 `odom → base_link`；不能重复发布同一 TF。
- `navsat_transform_node` 使用可信 local odom、经验证的 yaw 和 RTK NavSatFix，产生 Cartesian GNSS odometry。
- Global EKF 融合 LIO 的局部连续运动与 GNSS 绝对位置，发布 `map → odom`。
- 如果 LIO 本身已经输出平滑且有效的 Odometry，不需要为了使用 robot_localization 而重复滤波；可先直接将 LIO 作为 Nav2 local odom 输入，再用 global EKF 处理 GNSS。
- covariance 必须是真实/合理估计。零矩阵、未填充 covariance 或把固定值随意设为极小值都会制造错误置信度。
- 单天线 RTK 不提供静止 yaw；如果没有可靠 heading，继续使用通过验证的 LIO yaw，或后续评估双天线 GNSS heading。
- RTK 只修正全局位置/地图参考，不作为近距离避障输入。

## 5. 算法候选对比

| 候选 | 输入 | ROS 2 资料 | 与四足/MID-360 的相关性 | 风险/许可 | 当前定位 |
|---|---|---|---|---|---|
| 厂商 legged odometry | Lite3 estimator | 需厂商确认当前 sdk_deploy 输出 | 若存在并通过动态验证，最贴合本体步态状态 | 新 SDK 公开 RobotStatus 不等于完整 Odometry | 首选，待确认 |
| Point-LIO | MID-360 + 内置 IMU | Intel 2026.3 有 Humble/Jazzy port 与 Livox Driver 2 build flow | 上游强调强振动/高速角运动；Intel ROS2 port 有 MID-360 配置 | Intel 验证平台不是 Jetson，实际数据集验证不是 MID-360；需要 Orin POC | **首选备用 POC** |
| FAST-LIO2 | LiDAR + IMU | 有 ROS 2 Jazzy build reference | 直接点云配准，适合 Livox 非重复扫描 | GPLv2，上游 metadata license 有矛盾；产品化许可须先审查 | 对照 POC |
| FAST-LIVO2 | LiDAR + IMU + camera | Intel 有 Humble/Jazzy patch/demo | 可利用视觉约束走廊等退化环境；已公开 Mid-360 + D415 profile | 公开 profile 用 RealSense，不是 See3CAM stereo；GPLv2；同步复杂 | 第二阶段评估 |
| Stereo VIO | 双目 + IMU | 多个 VIO 开源项目有 ROS 2 ports，需分别验证 | 无需主动深度，视觉可补充 | 纹理/光照敏感，camera-IMU 同步和 ARM64 运行待测 | 备选比较 |

## 6. Point-LIO POC 验收计划

### 6.1 先在公开数据和仿真验证

- [ ] 固定 Point-LIO 上游/ROS 2 Jazzy port commit、Intel patch commit、Livox ROS Driver 2 commit；
- [ ] 检查 Point-LIO BSD-3-Clause、patch 和依赖许可证；
- [ ] 使用公开带真值的 rosbag 回放，测试启动、odom 输出、TF、重置和长时间稳定；
- [ ] 确认输入点云包含每点时间；验证 /livox/imu 和 /livox/lidar 时间戳的关系；
- [ ] 输出 evo APE/RPE、轨迹漂移、频率、消息年龄和 CPU/RAM。

### 6.2 Jetson AGX Orin + MID-360 台架验证

- [ ] 在目标 Orin JetPack 7.2.2 / Ubuntu 24.04 / ROS 2 Jazzy 上 build；
- [ ] 连接真实 MID-360，仅采集不移动，验证点云 CustomMsg、/livox/imu、PTP offset 和每点时间；
- [ ] 将传感器安装到 Lite3 上，但机器狗保持静止/抬起/受控低速，先检验静态 TF、振动对点云和 IMU 的影响；
- [ ] 使用地面真值/全站仪/固定标志对静止、直行、转弯、加减速、原地旋转和步态切换测试；
- [ ] 比较 Point-LIO、FAST-LIO2 与厂商 odom（若有），不要只看 RViz 轨迹；
- [ ] 测量 P50/P95/P99 odom latency、输出频率、CPU/GPU、内存、温度和丢帧；
- [ ] 在动态障碍物、坡地和不同地面分别测漂移和失败情况。

### 6.3 准入指标

具体阈值由阶段 A 的任务速度和安全需求确定。最少评估：
- 相对位置漂移与航向误差；
- odom 连续性和 TF 100% 可查询性；
- LIO update rate / latency；
- GNSS 加入/丢失/恢复时 map→odom 修正连续性；
- 机身冲击/足滑/地面坡度条件下的失效次数；
- 计算资源和热稳定；
- 失败/重启时 Nav2 是否停机而非使用 stale odometry。

不满足门槛时，保留 vendor odom 或评估 FAST-LIO2/stereo VIO；不得为了接通 Nav2 而放宽真实安全指标。

## 7. 参考资料

- [Intel Robotics AI Suite 2026.3 — LIO/LIVO Pipelines](https://developer.robotics.intel.com/2026.3/development-stack/components/navigation/lio-livo-pipelines/index.html)
- [Intel Robotics AI Suite 2026.3 — Point-LIO Jazzy port and MID-360 config](https://developer.robotics.intel.com/2026.3/development-stack/software_references/humanoid/sample_pipelines/point_lio_demo/index.html)
- [Intel Robotics AI Suite 2026.3 — FAST-LIO2 Jazzy build reference](https://developer.robotics.intel.com/2026.3/development-stack/software_references/humanoid/sample_pipelines/fast_lio2_demo/index.html)
- [Point-LIO upstream](https://github.com/hku-mars/Point-LIO)
- [FAST-LIO upstream](https://github.com/hku-mars/FAST_LIO)
- [FAST-LIVO2 upstream](https://github.com/hku-mars/FAST-LIVO2)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [Livox MID-360 Ethernet protocol and timestamp modes](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)
- [Point-LIO ROS 2 MID-360 example fork](https://github.com/Thetrafficlights/Point-LIO-ros2)
