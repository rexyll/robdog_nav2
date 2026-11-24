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
