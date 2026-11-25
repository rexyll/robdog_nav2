# 05 — 硬件与 Nav2 周边软件模块

## 1. 硬件角色

| 硬件 | 导航职责 | 需确认接口 |
|---|---|---|
| Lite3 Venture 候选 | 运动执行、姿态稳定、关节/足端反馈 | 速度/步态命令、base odom、IMU、joint/contact、急停、watchdog、模式 |
| Livox MID-360 | 3D 几何感知、LIO 输入 | Ethernet、时间同步、CustomMsg/PointCloud2、诊断、电源 |
| 2× See3CAM RGB | 被动双目深度、RGB 语义 | USB3、GPIO trigger、CameraInfo、同步、曝光 |
| ZED-F9P RTK | 室外全球位置约束 | UBX 质量消息、NavSatFix、RTCM/NTRIP、PPS、天线杆臂 |
| Jetson AGX Orin | ROS、感知、定位、Nav2 计算 | ARM64/BSP/JetPack、热设计、供电、网络/USB |
| Switch/trigger/DC-DC | 网络、同步、电源管理 | 带宽、时延、保护、隔离、EMI、温升 |

以上是候选和待确认项，不代表已采购或实机验证。

## 2. 软件模块地图

### 设备接入
- Livox-SDK2 + livox_ros_driver2：MID-360 发现、数据接收、ROS 发布。
- V4L2/USB camera driver：图像、CameraInfo、触发状态。
- u-blox ZED-F9P driver：UBX/NMEA/RTCM；保留原始 UBX 质量字段，不能只依赖未经审计的 NavSatFix。
- Lite3 vendor SDK/DRDDS 或 UDP/TCP sidecar：读取状态/odometry并发送运动命令；host/distro 待厂商确认。

### 机器人描述与时间
- URDF/Xacro：机身、传感器安装、质量/惯量、关节和 frame。
- robot_state_publisher：依据 URDF/joint_states 发布 link TF。
- static_transform_publisher：仅用于固定外参；外参需测量/标定。
- ros2_control（如厂商支持）：标准硬件接口/controller manager；不能假设 Lite3 必然支持。
- chrony/PTP/PPS/硬件 trigger：时钟同步。设备时钟不可仅因数值格式相同就视为同一时钟域。

### 定位与感知
- Point-LIO / FAST-LIO2：MID-360 + IMU 局部定位候选。
- robot_localization：EKF/UKF 融合。
- navsat_transform_node：GNSS 地理坐标投影。
- image_pipeline / stereo_image_proc：校正、视差、深度。
- pointcloud_to_laserscan：仅在适用时把 3D cloud 投影成 2D scan；会丢失高度信息。
- PCL/Open3D/自研 ROS nodes：地面分割、自体过滤、聚类、动态目标提取。
- slam_toolbox / AMCL：2D 地图构建/定位；需评估与四足姿态和 3D LiDAR 的适配。

### 导航与安全
nav2_bringup、nav2_map_server、nav2_planner、nav2_controller、nav2_behaviors、nav2_bt_navigator、nav2_costmap_2d、nav2_velocity_smoother、nav2_collision_monitor、nav2_waypoint_follower；项目自研 robodog_lite3_bridge/cmd_vel adapter；diagnostics、rosbag2、RViz2、Groot（按版本支持）。

## 3. 计算与通信预算

Orin 同时运行点云预处理、LIO、双目深度、目标检测、Costmap、Planner/Controller 会竞争 CPU/GPU/内存/USB/Ethernet 带宽。需测量端到端延迟和峰值资源，而不是只看平均 CPU。绘制网络/USB 拓扑，标记真实接口、线速、供电和共享总线。

## 4. 时钟语义

区分传感器采样时刻、设备内部时钟、驱动接收时刻、ROS header.stamp、host monotonic receive time、GNSS GPS week/iTOW、仿真 /clock。deskew、双目匹配、IMU-LiDAR 融合和 GNSS 历元配对依赖不同时间语义。驱动需记录 timestamp provenance、clock mapping、offset、drift、jitter、丢包和重启 epoch。

## 5. 软件基线

首轮参考 Ubuntu 22.04 + JetPack 6.1 + ROS 2 Humble；Orin 长期目标 JetPack 7.2.2 / Ubuntu 24.04 / ROS 2 Jazzy（以 TaskPlan 为准）。x86_64 build 不代表 ARM64；通用 ARM64 不代表 Jetson BSP build；仿真不代表真实传感器验证。Lite3 vendor Foxy 与 Jazzy 不默认共享 DDS domain，跨发行版使用明确协议边界/sidecar。

