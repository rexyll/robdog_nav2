# Livox MID-360：ROS 2 Jazzy 驱动锁定与接入基线

- 日期：2026-11-24
- 状态：官方驱动与 SDK release/commit 已锁定；x86_64 CI 的 SDK2 构建已通过一次，driver ROS 初始化步骤曾因工作流缺少 ROS 环境而失败；修正后 CI 正在复测，Orin ARM64 仍未验证
- 唯一进度台账：[TaskPlan.md](../../TaskPlan.md)
- 目标硬件：Livox MID-360（不是 MID-360S / MID-360L）

## 1. 版本选择

| 组件 | 锁定候选 | 证据 | 状态 |
|---|---|---|---|
| ROS driver | livox_ros_driver2 tag 1.2.6，commit 13eb05e4e6dd7a765b934d0c5fd6236676a57b49 | 官方 1.2.6 release 明确列出 Ubuntu 24.04 / ROS 2 Jazzy 支持；1.2.6 CHANGELOG 覆盖 MID-360 系列兼容基线 | 候选 pin，待 build |
| SDK | Livox-SDK2 tag v1.3.1，commit f5d9375f84efe2b15bc0a052d3e18482ed13adf4 | 官方 release 记录 v1.3.1 于 2026-04-15 发布；Intel Point-LIO Jazzy 手动复现明确使用 v1.3.1，并指出 GCC 13 需 `-include cstdint` | 固定候选，需与 driver CI 复测 |
| ROS | ROS 2 Jazzy | 项目 AGX Orin Ubuntu 24.04 长期基线 | 目标环境，未在 Orin 验证 |
| Driver license | MIT（driver 仓库 LICENSE） | 官方 LICENSE.txt | 按 MIT 要求保留声明；SDK 和二进制另行核对 |

为什么锁定 1.2.6 而不是直接跟随 master：1.2.6 是官方 release，且包含 Ubuntu 24.04/Jazzy 支持说明；当前 master 后续增加了 Avia2、MID-360L 等功能，但项目使用标准 MID-360，现阶段优先使用明确发布版本以降低滚动更新风险。SDK2 不再跟随滚动 master，改 pin 到 v1.3.1，与 Livox Driver 1.2.6 的时间窗口相邻。Intel Point-LIO Jazzy 参考构建同样使用该 SDK tag；GCC 13 下为 SDK2 增加 `-include cstdint` 兼容编译参数。此参考不替代本项目重新构建验收。

来源：
- [Livox ROS Driver2 1.2.6 release](https://github.com/Livox-SDK/livox_ros_driver2/releases/tag/1.2.6)
- [Livox ROS Driver2 1.2.6 exact commit](https://github.com/Livox-SDK/livox_ros_driver2/tree/13eb05e4e6dd7a765b934d0c5fd6236676a57b49)
- [Livox SDK2 v1.3.1 release](https://github.com/Livox-SDK/Livox-SDK2/releases/tag/v1.3.1)
- [Livox SDK2 v1.3.1 exact commit](https://github.com/Livox-SDK/Livox-SDK2/tree/f5d9375f84efe2b15bc0a052d3e18482ed13adf4)
- [Intel Point-LIO Jazzy reference build using SDK2 v1.3.1](https://developer.robotics.intel.com/2026.3/development-stack/software_references/humanoid/sample_pipelines/point_lio_demo/index.html)
- [Livox ROS Driver2 LICENSE](https://github.com/Livox-SDK/livox_ros_driver2/blob/1.2.6/LICENSE.txt)

## 2. 官方 Jazzy build flow

官方 livox_ros_driver2 README 的 Jazzy 步骤为：

1. 在 ROS 2 workspace 的 src/ 下 clone livox_ros_driver2；
2. 按 Livox SDK2 README 编译安装 Livox-SDK2；
3. source ROS Jazzy；
4. 从驱动仓库执行 ./build.sh jazzy；
5. source workspace install 并使用 ROS 2 launch file 启动 driver。

本项目实际 build 时还必须固定：
- Ubuntu 24.04 + JetPack/L4T 精确版本；
- aarch64 Jetson GCC/CMake/PCL/OpenCV/ROS package 版本；
- SDK2 shared library 路径与 ABI；
- colcon build 输出、ldd 依赖结果、CPU/RAM 使用和启动日志。

官方说明将 Driver2 定位为调试用途，不建议未经优化直接用于量产；本项目先将其用于原型验证。量产前需要做性能、丢包、恢复、watchdog 和长期运行审查。

2026-11-24 CI 根因补充：livox_ros_driver2 1.2.6 源仓库同时包含 ROS1 默认 package.xml 与 ROS2 package_ROS2.xml。action-ros-ci 在依赖解析前读取默认 ROS1 manifest，导致 ROS2 构建依赖 ament_cmake_auto 未安装，CMake 在 find_package(ament_cmake_auto) 失败。独立 x86_64 workflow 已改为先将 package_ROS2.xml 复制为 package.xml，再显式安装 ros-jazzy-ament-cmake-auto，执行 rosdep install 和 colcon build。修复版 workflow 在 run 45335285873 及后续重跑中验证；当前仍在 ROS Jazzy/依赖安装阶段，尚未宣称 Driver2 build pass。

## 3. Point-LIO 与 Nav2 的点云格式策略

Driver2 有两种与本项目相关的输出格式：

| xfer_format | 输出 | 时间信息 | 使用建议 |
|---|---|---|---|
| 1 | livox_ros_driver2/CustomMsg | timebase + 每点 offset_time | **Point-LIO 首选输入**，能保留 Livox 每点相对采样时间，供 deskew 使用 |
| 0 | sensor_msgs/PointCloud2 的 PointXYZRTLT | 每点字段包含 float64 timestamp（driver 文档定义） | RViz/通用点云工具与下游消费；使用前确认字段名、单位和下游保留情况 |
| 2 | PCL 标准 PointXYZI | 不提供 Point-LIO 所需的完整每点时间语义 | 不作为 LIO 输入；可作为不做运动去畸变的简单显示/几何用途 |

项目配置约束：
- Point-LIO 主输入保持 CustomMsg，不得先转成 PointXYZI 丢失 per-point time；
- 如 Nav2 costmap 需要标准 PointCloud2，使用独立转换节点输出，不能覆盖/替代 LIO 原始带时间 topic；
- 每个采样点时间戳必须单调且处于对应 frame 扫描窗口；记录 timebase、offset_time 的单位与转换；
- 点云 header stamp、per-point time、IMU stamp 必须分别保留；确认 Livox driver 对 PTP/GPS 时钟的转换语义，禁止只看 header stamp 就宣称 LiDAR/IMU 同步；
- 不在无硬件时固定猜测 time_offset_lidar_to_imu 或强制开启自动 time sync。对照 Point-LIO 选定 fork 的配置语义实测。

**CustomMsg 时间字段语义（v1.2.6 源码核验）：**
- Driver2 `lddc.cpp` 将 SDK `pkg.base_time` 写入 `CustomMsg.timebase` 和 ROS 2 `header.stamp`（`rclcpp::Time(timestamp)`）；该时间值使用纳秒。
- 每个 `CustomPoint.offset_time` = SDK 点采样时间 - `pkg.base_time`，是相对 base time 的纳秒偏移。
- Point-LIO Livox CustomMsg 分支将 `offset_time / 1e6` 转成毫秒作为内部 per-point time。
- Point-LIO 的 `preprocess.timestamp_unit` 仅作用于标准 PointCloud2 输入分支；对 Livox CustomMsg 分支不执行该通用换算。

因此：保持 `xfer_format=1`、CustomMsg 和 offset_time 原值完整；不要把 offset_time 先转成秒/毫秒再交给 Point-LIO，也不要通过 timestamp_unit 去补偿该路径。真实设备时钟/PTP 与点内偏移仍必须以 bag 验证。

Driver2 的 IMU 路径也在 `InitImuMsg()` 中使用 `ImuData.time_stamp` 作为时间戳，并在 ROS 2 分支通过 `rclcpp::Time(timestamp)` 填入 IMU header。和 LiDAR `base_time` 一样，源码中以 ns 传递；实际 MID-360 PTP/GPS 时钟状态与主机 offset/drift 仍需实测。

源码： [Driver2 lddc.cpp](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/src/lddc.cpp) · [CustomMsg](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/msg/CustomMsg.msg) · [CustomPoint](https://github.com/Livox-SDK/livox_ros_driver2/blob/13eb05e4e6dd7a765b934d0c5fd6236676a57b49/msg/CustomPoint.msg)

## 4. 网络/时间配置验收

MID-360 使用 Ethernet 连接；本项目网络配置需由设备实物标签/供应商确认，以下配置值不能直接复制官方示例 IP：
- LiDAR IP；
- AGX Orin Ethernet interface IP；
- command/data/IMU ports；
- MTU、交换机 VLAN/IGMP（如需要）、网口速率与丢包计数；
- PTPv2 profile / grandmaster / NIC hardware timestamp 能力；
- 与 GNSS PPS 的联合时间方案。

必须输出：
1. 固定版 network diagram/IP table；
2. driver launch yaml/json 和实际设备 IP；
3. 点云/IMU topic 频率、消息 age、丢包率；
4. LiDAR PTP offset/drift 与 host clock offset；
5. 点云每点时间有效率与 deskew 检查；
6. rosbag2 录制/回放一致性与长时运行报告。

## 5. 构建与测试计划（B4 / B11）

- [x] Ubuntu 24.04 + ROS 2 Jazzy x86_64：Livox SDK2 v1.3.1 与 livox_ros_driver2 v1.2.6 的构建和安装已通过 [run 45337301420](https://github.com/rexyll/robdog_nav2/actions/runs/45337301420)。SDK2 独立 CMake build/install 成功；Driver2 在显式 ROS2 manifest、ament_cmake_auto、rosdep/apt 更新后完成 colcon build，安装 prefix/executable 检查通过。此证据只覆盖 x86_64，不覆盖 Jetson Orin ARM64 或真实 MID-360 运行。
- [ ] 在 JetPack 7.2.2 / Ubuntu 24.04 / ROS 2 Jazzy / aarch64 Orin 上完成同样 build；
- [ ] 核验 SDK2 v1.3.1 与 driver 1.2.6 的 API/ABI 兼容；
- [ ] 接入真实 MID-360，确认设备发现、点云和 IMU topic；
- [ ] 验证 CustomMsg 中 timebase/offset_time 与实际时间单位，检查每点时间戳窗口；
- [ ] 使能并验证 PTPv2/GPS 时间同步，测量 offset、drift 和恢复；
- [ ] 运行 Point-LIO 公共 bag 回放和真实 MID-360 bag 回放，对比轨迹与 CPU/RAM/温度；
- [ ] 连续运行 60 分钟，验证丢包、节点重启、设备掉线恢复与 rosbag 可读。

两条 Jazzy build workflow 已设置同分支 concurrency（cancel-in-progress），且 docs-only push/PR 不再触发构建；当前无真实 MID-360 或 Orin，因此硬件相关验收仍全部未完成。仅完成 commit/文档审计不能标记 B4 或 B11 完成。
