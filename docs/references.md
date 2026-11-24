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
