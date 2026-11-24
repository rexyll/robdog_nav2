# ADR-0004：本体 odometry 缺失时的 LIO/VIO 备用定位方案

- 状态：**Proposed — Point-LIO 首选备用 POC；未在 Jetson AGX Orin + MID-360 实测**
- 日期：2026-11-24
- 关联：[TaskPlan.md](../../TaskPlan.md)、[odometry fallback strategy](../implementation/odometry_fallback_strategy.md)

## 背景

当前 sdk_deploy 的公开 DRDDS IMU/joint/RobotStatus 消息不是完整 nav_msgs/Odometry。若 Lite3 Venture 当前固件未提供可用的腿式 odometry/body velocity，Nav2 仍需要连续 local odom。因此需评估使用现有 MID-360 + 内置 IMU 构建 LIO 的方案。

## 决策

1. 第一优先仍是向云深处确认正式 legged odometry，若质量/时间戳/covariance 经实测通过，则作为本体状态主要来源。
2. 若无可靠 legged odometry，首选开展 Point-LIO + MID-360 + 内置 IMU 的 POC。
3. FAST-LIO2 作为 LIO 对照基线；Stereo VIO / FAST-LIVO2 作为后续备选，不进入第一版主链。
4. LIO 使用 MID-360 点云逐点时间戳和同设备 IMU；不得未经同步将 Lite3 IMU 混合输入。
5. LIO 的里程计经过外参变换后提供连续 odom→base_link；RTK 通过 navsat_transform + global EKF 提供 map→odom 全局位置修正。
6. 若 LIO 的时间戳、协方差、TF、Orin 计算性能或四足动态环境精度不通过，不能用它驱动 Nav2；重新评估厂商估计接口/其他算法。

## 为什么 Point-LIO 先于 FAST-LIO2

- Point-LIO 的算法设计为逐点更新，对高速旋转、强振动和 IMU 饱和有明确针对性，符合四足机身运动特点；
- Intel Robotics AI Suite 2026.3 提供 Point-LIO ROS 2 Jazzy port 和 Livox Driver 2 构建流程，并包含 MID-360 配置；
- 其 BSD-3-Clause 许可比 FAST-LIO2 的 GPLv2 更便于后续许可审查；
- 但 Intel 的真实硬件验证使用 Velodyne/Intel Core Ultra 平台，尚未验证本项目的 MID-360/Jetson Orin，所以只列为首选 POC，不能直接认定通过。

## 关键风险

- MID-360 与内置 IMU 的时钟和采集时间必须验证；Livox PTP/GPS 时间不是自动证明 driver/host/IMU 全链路同步。
- FAST-LIO2 上游是 GPLv2，若用于商业交付需要评估替代许可。
- FAST-LIVO2 上游也是 GPLv2，且公开 Mid-360 profile 使用 RealSense D415/D435i，不是本项目 See3CAM 双目。
- POC 必须在实际 Jetson Orin 上做 colcon build 和性能测试；Intel PC 的通过结果不能替代 Orin。

## 验收条件

- LIO 输出有效 nav_msgs/Odometry、连续 TF 和合理 covariance；
- 使用 MID-360 CustomMsg 保留每点时间戳；
- 通过同步/外参标定和动态运动测试；
- 在 Orin 上达到阶段 A 锁定的更新率/延迟/资源门槛；
- 与独立真值比较轨迹误差，且通过 RTK loss/recovery 测试；
- 故障或 stale odometry 时 Nav2 安全停止。

