# Launch files

预留给可组合的 ROS 2 launch files。规划入口见 [package layout](../docs/implementation/ros2_workspace_and_packages.md)。

launch 至少支持：
- sensors only
- description/TF only
- localization (indoor / outdoor RTK modes)
- perception
- Nav2
- safety / command gateway
- full robot bringup

所有硬件 topic、frame、namespace、use_sim_time 和 config path 都应参数化；不要将设备型号假设硬编码到通用入口。
