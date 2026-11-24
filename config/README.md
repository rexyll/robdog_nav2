# Configuration

本目录预留给已验证的 ROS 2 YAML 配置。当前不提交可直接驱动未知硬件的虚构参数。

计划分层：
- sensors/
- localization/
- perception/
- nav2/
- safety/

配置必须绑定 ROS_DISTRO、包版本、传感器型号/序列号和标定版本。实机配置与仿真配置分开管理。数值参数必须有来源和测试记录。
