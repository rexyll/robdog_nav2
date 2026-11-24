# 参考资料与技术依据

本文件记录架构制定时参考的官方/一手资料。正式实现前仍需按锁定的 ROS 2 发行版和具体包版本复核参数、插件名和接口。

## ROS 2 / Nav2

1. [Nav2 Jazzy — Mapping and Localization](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization.html)  
   Nav2 costmap 的 Static、Obstacle、Voxel、Inflation 等层；ObstacleLayer/VoxelLayer 支持 LaserScan 和 PointCloud2 观测源。

2. [Nav2 — GPS Navigation Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)  
   Nav2 GPS 航点流程和 map→odom→base_link 定位结构；RTK/GPS 与 robot_localization 组合。

3. [robot_localization — navsat_transform_node](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/navsat_transform_node.rst)  
   navsat_transform_node 使用 odometry、IMU heading 和 NavSatFix，将地理数据转换到机器人的笛卡尔 world frame；包含 UTM/Local Cartesian 相关参数说明。实现时应切换到所使用 ROS 发行版对应的包文档/源码版本。

4. [Nav2 Jazzy — Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/collision_monitor/configuring_collision_monitor_node.html)  
   软件级碰撞监测，可使用 LaserScan、PointCloud2、Range 等输入；文档说明该节点不提供硬实时/安全认证保证。

5. [Nav2 — Setting Up Transformations](https://docs.nav2.org/setup_guides/transformation/setup_transforms.html)  
   map、odom、base_link 和传感器 frame 的职责与关系。

6. [ROS 2 image_pipeline](https://github.com/ros-perception/image_pipeline)  
   ROS 2 图像处理/双目立体视觉相关软件包。需按目标发行版验证 stereo_image_proc 的可用版本和接口。

## ROS 2 发行版

7. [ROS 2 Jazzy Jalisco Released — Open Robotics](https://www.openrobotics.org/blog/2024/5/ros-jazzy-jalisco-released)  
   官方发布信息：Jazzy 是 LTS，支持至 2029 年 5 月；Ubuntu 24.04 amd64/arm64 为 Tier 1 平台。

## 四足导航相关参考

8. [ROS 2-Driven Navigation and Sensor Platform for Quadruped Robots](https://www.mdpi.com/2218-6581/15/4/70)  
   四足平台采用 Nav2 的系统案例，描述了 global/local costmap、Planner/Controller、Legged odometry、LiDAR/point cloud 和 velocity command 接口的组合。论文方案属于案例参考，不直接等于本项目的已验证实现。

9. [Unitree B2 ROS Navigation Documentation](https://docs.quadruped.de/projects/b2/html/ros.html)  
   厂商/生态示例，展示四足机器人使用 Nav2 与 LiDAR/SLAM 的集成方式。具体接口只适用于对应产品与版本，不能直接套用到其他机器狗。

## 备注

- 网页资料只用于形成初始架构，不替代设备手册、代码版本锁定和实测。
- rolling 分支文档不得直接作为 Jazzy 参数配置；锁定发行版后应使用对应 tag/branch 文档。
- 每个配置文件应记录 ROS_DISTRO、包版本、驱动 commit、传感器固件和标定版本。


## 已选硬件平台一手资料（2026-11-24 选型核验）

10. [DEEP Robotics Lite3 产品规格](https://www.deeprobotics.us/products/lite-3/)  
    Venture 公开规格：约 12.2 kg、4.5 kg payload、2× Ethernet、24V/12V/5V 外设电源；与 Basic/Pro/LiDAR 型号对比。具体地区 SKU 和供电电流须采购前向厂商确认。

11. [DEEP Robotics Lite3 感知算法开发示例](https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/)  
    官方公开组合：Lite3 Venture、Jetson AGX Orin、Livox Mid-360、Ubuntu 22.04、ROS 2 Humble、JetPack 6.1；示范 MID-360 24V XT30 供电、Ethernet 连接和外扩安装。

12. [DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS)  
    公开 transfer 接口列出 `/leg_odom`、`/leg_odom2`、`/imu/data`、`/joint_states` 和 `/cmd_vel`；真实版本/频率/frame/控制语义需实机核对。

13. [DeepRoboticsLab/sdk_deploy](https://github.com/DeepRoboticsLab/sdk_deploy)  
    Lite3 SDK deploy 与 ROS 2 UDP message transfer；包含状态与控制接口说明。SDK 使用许可、固件版本和命令保护需单独审查。

14. [Livox MID-360 规格](https://www.livoxtech.com/mid-360/specs)  
    360° 水平 FOV、-7° 至 +52° 垂直 FOV、200k points/s、典型 10 Hz、Ethernet、PTPv2/GPS 同步、IP67 等规格。最终型号 MID-360/MID-360s 不可混用配置。

15. [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)  
    驱动 README 包含 Ubuntu 24.04 / ROS 2 Jazzy 构建说明；本项目仍需固定 commit 并在目标设备编译验证。

16. [e-con Systems See3CAM IMX264 / See3CAM_50CUG](https://www.e-consystems.com/sony-imx264-camera.asp)  
    官方产品页面列出彩色无镜头 SKU `See3CAM_50CUG_CXXCC_BX_H01R1`；5MP、Global Shutter、USB 3.2 Gen1、Linux/UVC、GPIO Trigger/Strobe、最高 35 fps。外部触发电气规格与双机硬件同步仍需制造商书面确认。

17. [ArduSimple simpleRTK2B Basic Starter Kit](https://www.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/)  
    ZED-F9P + ANN-MB 多频天线，可接收 NTRIP/基站改正，最高性能配置最高约 10 Hz；RTK Fix 需要有效差分改正。

18. [NVIDIA Jetson AGX Orin Developer Kit](https://developer.nvidia.com/embedded/jetson-agx-orin-developer-kit)  
    AGX Orin 计算平台官方规格与开发资源。模块/载板、内存、供电和散热必须按具体 SKU 核对。

19. [ROS 2 Humble to Jazzy 支持期说明](https://docs.autoware.org/main/home/roadmap/timelines/ros-2-distributions/)  
    汇总 ROS 2 Humble 于 2027 年 5 月结束支持、Jazzy 于 2029 年 5 月结束支持的时间线；发行版维护策略以 ROS REP-2000/官方发布信息为准。


## 2026-11-24 ROS / Jetson 版本更新核验

20. [ROS 2 Lyrical Luth Released — Open Robotics Discourse](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021)  
    ROS 2 Lyrical 于 2026-05-22 发布，是 LTS，支持至 2031-05；Tier 1 平台包括 Ubuntu 26.04 amd64/arm64。因 Jetson Orin 当前 JetPack 系统基线为 Ubuntu 24.04，本项目暂不将 Lyrical 定为 Orin 运行基线。

21. [ROS 2 Release List](https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases.rst)  
    当前发行版 EOL 日期：Humble 2027-05、Jazzy 2029-05、Lyrical 2031-05。

22. [NVIDIA JetPack 7.2.2 Downloads and Notes](https://developer.nvidia.com/embedded/jetpack/downloads)  
    JetPack 7.2.2 支持 Jetson Orin Family；Jetson Linux 39.2.2 / L4T Ubuntu 24.04。需按具体 AGX Orin 模块/开发套件确认实际兼容性和驱动。

23. [NVIDIA Jetson Linux Archive](https://developer.nvidia.com/embedded/jetson-linux-archive)  
    官方 Jetson Linux 39.x 版本表列出 AGX Orin / Orin NX / Orin Nano 支持情况。


## 2026-11-24 多传感器同步补充资料

24. [DEEP Robotics Lite3 — Perception Algorithm Development Example](https://developer.deeprobotics.cn/docs/lite3/examples/sdk-dev/perception-algorithm-example/)  
    Lite3 Venture 扩展感知示例使用 MID-360/MID-360s + IMU + RealSense + FAST-LIVO2；说明了特定 Mid-360s/相机组合的时钟偏移校正、软同步方案和同步失败风险。本项目只借鉴“区分时钟域并实测”的原则，不复制该示例对 RealSense 或 Mid-360s 的专有偏移常数。

25. [e-con Systems See3CAM_50CUG product specifications](https://www.e-consystems.com/sony-imx264-camera.asp)  
    彩色无镜头 SKU、外部 Trigger、Strobe、GPIO 接口和 UVC/USB 产品信息。触发电压、脉宽、边沿与双机同步时序必须以手册/供应商回复为准。

26. [Livox MID-360 specifications](https://www.livoxtech.com/mid-360/specs)  
    MID-360 外部数据端口为 100BASE-TX Ethernet，支持 IEEE 1588-2008 PTPv2/GPS 时间同步；内置 ICM40609 IMU。产品规格不能证明该 IMU 与所有驱动时间戳自动同步。


## ROS 2 CI references

27. [ros-tooling/setup-ros](https://github.com/ros-tooling/setup-ros)  
    Official setup-ros action documentation warns that APT update can be flaky on bare-metal GitHub Actions Linux workers and recommends Docker/container-based builds.

28. [ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci)  
    Official ROS 2 workspace build/test action. Its documentation recommends prebuilt setup-ros-docker images, with Ubuntu Jammy for Humble and Ubuntu Noble for Jazzy.


## 2026-11-24 机载质量与电气预算资料

29. [NVIDIA Jetson AGX Orin Developer Kit User Guide — Power and USB](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/howto.html)  
    官方用户指南说明开发套件可由 DC jack 或 USB-C PD 供电；USB-C PD sink profiles 包括 20V/4.5A、15V/3A、14.8V/3A、9V/3A、5V/3A。

30. [NVIDIA Jetson AGX Orin Developer Kit Quick Start / BSP](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/quick_start.html)  
    AGX Orin Developer Kit 当前用户指南采用 JetPack 7.2.1 / L4T r39.2.1 ISO 作为 DevKit 更新入口。

31. [NVIDIA JetPack SDK Downloads and Notes — JetPack 7.2.2](https://developer.nvidia.com/embedded/jetpack/downloads)  
    JetPack 7.2.2 / Jetson Linux 39.2.2 支持 Jetson Orin Family，使用 L4T Ubuntu 24.04；NVIDIA 指引先下载 7.2.1 image，再 apt update / dist-upgrade。

32. [NVIDIA Jetson AGX Orin Developer Kit Reviewer’s Guide](https://developer.nvidia.com/sites/default/files/akamai/Jetson_AGX_Orin_Developer_Kit_RG_0.pdf)  
    Developer Kit 模块功率范围列为 15–60W，开发套件提供多路 USB 3.x、RJ45、40-pin 扩展接口；最终供电按实际载板/输入方式核对。

33. [NVIDIA Developer Forum — AGX Orin Developer Kit weight](https://forums.developer.nvidia.com/t/jetson-agx-orin-hw-faq/249100)  
    NVIDIA 论坛硬件 FAQ 回答 Developer Kit 重量为约 872.5g；本项目仅把它作为预算估算，采购到货后必须实物称重。

34. [e-con Systems See3CAM_50CUG electrical/mechanical specifications](https://www.e-consystems.com/sony-imx264-camera.asp)  
    官方产品规格给出单台典型功耗范围 0.44–1.225W、无镜头外壳尺寸 40×40×45.23mm；未在公开页面确认质量，需采购前询问/到货称重。

35. [ArduSimple simpleRTK2B receiver specifications](https://www.ardusimple.com/simplertk2b-receivers/)  
    simpleRTK2B Budget 板卡质量约 19.5g，典型电流约 80mA @ 5V；ANN-MB 天线质量约 173g、LNA 典型电流约 15mA @ 3V。

36. [Livox MID-360 specifications](https://www.livoxtech.com/cn/mid-360/specs)  
    官方列出平均功耗 6.5W、9–27V DC、质量 265g、PTPv2/GPS 同步和 100BASE-TX Ethernet。


## 2026-11-24 Lite3 ROS SDK source audit

37. [DeepRoboticsLab/sdk_deploy — pinned main commit](https://github.com/DeepRoboticsLab/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d87d)  
    Current Lite3 Venture SDK deployment path. lite3_transfer uses DRDDS messages and has SDK/E-stop services. Cross-host documentation warns Foxy and non-Foxy ROS 2 distributions may not communicate correctly over DDS.

38. [sdk_deploy Lite3 SDK deployment guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/Lite3_sdk_deploy/README.md)  
    Describes rl_deploy, command_state, /rl_deploy/cmd_vel, effective_cmd_vel, and cross-host AGX Orin/JetPack 6.1 example. The published speed and timeout values are examples and must not be treated as this project's approved limits.

39. [sdk_deploy lite3_transfer UDP interface](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d/src/lite3_transfer/interface/robot/udp_interface.hpp)  
    Source declares /IMU_DATA, /JOINTS_DATA, /JOINTS_CMD, /SDK_MODE, /EMERGENCY_STOP and /EMERGENCY_STOP_SIGNAL. These are DRDDS custom messages/services, not standard Nav2 odometry interfaces.

40. [sdk_deploy DRDDS message definitions](https://github.com/DeepRoboticsLab/sdk_deploy/tree/20279073bc43522aa06b0abf611bb2daaa53d/src/drdds/msg)  
    ImuDataValue includes Euler angles, angular velocity and linear acceleration; RobotStatus includes joints, base RPY, base angular velocity, command state and telemetry freshness. Public RobotStatus definition does not itself provide a complete nav_msgs/Odometry record.

41. [Legacy Lite3_ROS Jetson2Motion source](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/src/transfer/src/Jetson2Motion.cpp)  
    The legacy Foxy UDP transfer publishes standard ROS messages including /leg_odom2 and /imu/data. It uses steady-clock timestamps in the inspected code path; child_frame_id/covariance require audit before localization use.

42. [sdk_deploy lite3_sdk_service deployment guide](https://github.com/DeepRoboticsLab/sdk_deploy/blob/20279073bc43522aa06b0abf611bb2daaa53d87d/src/lite3_sdk_service/README.md)  
    Describes the motion host's current Ubuntu 20.04/Foxy workflow, service commands and warning about cross-distribution ROS 2 communication and DDS discovery traffic.
