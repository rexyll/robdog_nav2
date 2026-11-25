# 07 — 验证、调试与实施路线

## 1. 验证分层

| Level | 验证 | 能证明 | 不能证明 |
|---|---|---|---|
| L0 静态/单元 | 参数、插件配置、算法 helper、契约 | 纯逻辑 | ROS graph/实时性/硬件 |
| L1 ROS 集成 | 节点、Lifecycle、Topic/Action/TF、QoS | ROS 接口连通 | 真实传感器质量 |
| L2 仿真 | Gazebo 导航场景 | 模型内任务和控制 | 真实地面/传感器 |
| L3 数据回放 | rosbag2 定位/感知/Nav2 回放 | 真实记录离线处理 | 在线实时性/真实运动 |
| L4 台架只读 | 真实传感器、状态、时钟/带宽/温升 | 硬件接入和数据质量 | 运动控制安全 |
| L5 封闭场地受控运动 | 低速限定场地监督测试 | 受控闭环性能 | 开放环境安全认证 |
| L6 HIL/正式实机 | 完整软硬件闭环、故障注入 | 具体覆盖范围内的证据 | 未覆盖场景的普遍安全 |

报告必须写明层级、设备、软件 SHA、配置、数据、场景、重复次数、指标和失败日志。合成 CI 不得标记为真实 bag replay。

## 2. 诊断命令

    ros2 node list
    ros2 topic list -t
    ros2 action list -t
    ros2 service list -t
    ros2 topic info /points --verbose
    ros2 topic hz /points
    ros2 topic echo /odom --once
    ros2 interface show nav2_msgs/action/NavigateToPose
    ros2 run tf2_tools view_frames
    ros2 run tf2_ros tf2_echo map base_link
    ros2 run tf2_ros tf2_echo odom base_link
    ros2 lifecycle nodes
    ros2 lifecycle get /controller_server
    ros2 param dump /controller_server
    ros2 topic echo /diagnostics
    ros2 bag info <bag_dir>
    ros2 bag play <bag_dir> --clock

不同发行版 CLI 选项可能不同，先用 --help 核验。回放时 use_sim_time、/clock 和 TF 时间必须一致。

## 3. RViz 检查

- TF：map/odom/base/sensor frame 的方向和位置。
- PointCloud2：是否随机器人运动、地面是否合理、是否有重影/拖影。
- Costmap：marking、clearing、inflation、ghost obstacles。
- Global Path：是否穿障、贴边或不符合运动能力。
- Local Costmap/trajectory：Controller 是否看到新障碍、是否振荡。
- Collision Monitor：polygons/state、source timeout。
- Odometry：速度、姿态、协方差、延迟和 frame。

## 4. 量化指标

定位：ATE/RPE（仅 GT provenance 合格时）、local odom drift、heading/speed error、延迟、跳变、协方差一致性、GNSS Fixed 占比和恢复时间。

感知/Costmap：召回率/误检率、首次发现距离、点云延迟、障碍距离/高度误差、marking/clearing 延迟、ghost obstacle 时间、更新率和数据龄期。

导航：任务成功率、路径长度/规划时间、横向跟踪误差、最小障碍间距、重规划次数、卡住时间、恢复成功率、速度跟踪误差、停车距离和急停响应。

资源：CPU/GPU、内存、温度/降频、网络/USB 带宽、丢包、executor 延迟、topic jitter、端到端延迟。

阈值由风险、速度、制动距离和硬件能力定义，不直接套默认参数。

## 5. 常见故障排查顺序

1. 先查时钟和 TF，再查传感器内容。
2. 查 topic type、QoS、频率、数据年龄、frame_id。
3. 查 Costmap source、height/range、marking/clearing、persistence。
4. 查 footprint、inflation、unknown-space policy。
5. 查 Planner 是否找到路径，Controller 是否拿到 Path/Costmap/odom。
6. 查 cmd_vel remap、Twist vs TwistStamped、频率、限幅、timeout。
7. 最后查底层步态/控制模式；不要用调 Planner 掩盖 odom/命令链路错误。

## 6. 实施路线

- N0 理论与接口冻结：读完本目录，固定 ROS 发行版，列出 node/topic/action/service/TF。
- N1 Nav2 最小闭环（仿真）：标准 URDF、odom、TF、map、障碍源，完成 NavigateToPose。
- N2 真实数据只读：MID-360、双目、IMU、GNSS；完成时间、TF、QoS 和质量 Gate。
- N3 定位与 Costmap：local odom、map→odom；先 LiDAR 后 Stereo，验证 marking/clearing。
- N4 算法对照：NavFn/Smac 2D；DWB/RPP；对照 MPPI。
- N5 安全命令台架：Smoother → Collision Monitor → adapter → watchdog，先无动力/厂家认可台架。
- N6 封闭场地低速：供应商安全资料齐备后，测试停车距离、动态障碍、窄通道和故障注入。
- N7 四足地形：加入 elevation/traversability、动态跟踪、落足规划接口。

## 7. 当前 Gate

Lite3 可信腿式 odometry/运动命令能力未确认；传感器未完成目标 Orin 全链路实测；Dynamic03 RTK Fixed/GT 精度未认证；Nav2 尚未完成项目级仿真 E2E、HIL 或真实机器狗验收。未完成物理急停、底层 watchdog、制动距离和控制权确认前，不发送真实运动命令。
