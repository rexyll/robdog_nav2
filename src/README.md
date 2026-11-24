# ROS 2 source workspace

当前尚未创建 ROS package。硬件与软件接口确认后，按 [ROS 2 workspace 与 package 规划](../docs/implementation/ros2_workspace_and_packages.md) 建立 ament/colcon 工作区。

计划 package：
- robodog_description
- robodog_bringup
- robodog_localization
- robodog_stereo_perception
- robodog_obstacle_perception
- robodog_nav2_bringup
- robodog_cmd_adapter
- robodog_diagnostics
- robodog_interfaces（仅在确有自定义消息需求时）

不将厂商 SDK、未授权模型或自动下载的二进制文件直接纳入源码树。
