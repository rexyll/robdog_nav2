# ROS 2 Workspace 与软件包规划

## 1. 软件包拆分原则

采用 ament/colcon 标准 workspace。第一阶段只建立接口边界，不急于生成未经硬件验证的 driver wrapper 或 nav2 参数。

建议初始结构：

```text
robdog_nav2/
├── README.md
├── TaskPlan.md
├── docs/
├── src/
│   ├── robodog_description/
│   ├── robodog_bringup/
│   ├── robodog_lite3_bridge/       # 项目自有标准消息适配层；当前只有 mock/诊断骨架
│   ├── robodog_lite3_vendor_gateway/ # 未来 Foxy/DRDDS 侧非 DDS 网关；需厂商确认后再创建
│   ├── robodog_lio_bringup/          # Point-LIO/FAST-LIO2 fallback for local odometry POC\n│   ├── robodog_localization/
│   ├── robodog_stereo_perception/
│   ├── robodog_obstacle_perception/
│   ├── robodog_nav2_bringup/
│   ├── robodog_cmd_adapter/
│   ├── robodog_diagnostics/
│   └── robodog_interfaces/            # 仅在确有自定义消息需求时创建
├── config/
│   ├── sensors/
│   ├── localization/
│   ├── perception/
│   ├── nav2/
│   └── safety/
├── launch/
├── maps/
├── bags/                              # 大文件默认不入 Git
├── models/                            # 模型权重默认外部管理/版本化登记
└── test/
```

## 2. 软件包职责

| Package | 职责 | 主要接口 |
|---|---|---|
| robodog_description | Xacro/URDF、link/joint、sensor frame、碰撞几何 | robot_description、TF |
| robodog_bringup | 启动 robot_state_publisher、传感器、状态估计和诊断 | launch |
| robodog_lite3_bridge | 项目自有标准消息适配层；消费经过验证的非 DDS vendor gateway 消息，并输出标准 IMU/joint/odom/status；当前只有 mock/诊断实现 | 标准 ROS 消息 |
| robodog_lite3_vendor_gateway | 未来独立的 vendor-Foxy/DRDDS 侧 gateway，将厂商专用数据封装成版本无关协议；仅在供应商确认主机拓扑后创建 | 非 DDS UDP/TCP application protocol |

| robodog_lio_bringup | 启动/配置 Point-LIO 或 FAST-LIO2 POC，统一输出局部 odom；最终算法需在 Jetson Orin + MID-360 实测选型 | PointCloud2/CustomMsg、Imu、Odometry、TF |\n| robodog_localization | local/global EKF、navsat_transform 参数、GNSS gate | Odometry、NavSatFix、TF |
| robodog_stereo_perception | 左右图同步、校正、视差、深度/点云 | Image、CameraInfo、DisparityImage、PointCloud2 |
| robodog_obstacle_perception | LiDAR/点云过滤、地面分割、障碍物提取和跟踪 | LaserScan、PointCloud2、目标消息 |
| robodog_nav2_bringup | Nav2 参数、地图、BT、planner/controller/costmap launch | NavigateToPose、cmd_vel |
| robodog_cmd_adapter | Nav2 Twist/TwistStamped 到厂商批准命令接口的安全适配、限幅、状态机检查、watchdog | 命令与底层状态 |
| robodog_diagnostics | 传感器健康、时间同步、GNSS 状态、TF 和命令链路监控 | DiagnosticArray/状态 |
| robodog_interfaces | 项目确需跨节点共享的专用状态/目标消息 | 自定义 msg/srv/action |

## 3. 外部驱动策略

- 优先使用厂商维护且支持目标 ROS 发行版的官方驱动。
- 通过 topic remap、参数和标准消息接入，不要为统一命名而复制厂商驱动。
- 只有官方驱动缺失必要功能时，才编写 adapter 或维护 fork；记录 upstream URL、许可证、固定 commit 和升级策略。
- 机器狗 SDK 可能是专有许可，应单独记录使用/分发边界；不要将授权不明的 SDK 或模型权重直接提交到公共仓库。

## 4. 配置分层

- `config/sensors/`：相机、LiDAR、GNSS/RTK、IMU 驱动 wrapper 参数。
- `config/localization/`：local/global EKF、navsat_transform、GNSS gate。
- `config/perception/`：stereo、过滤、地面/障碍物提取。
- `config/nav2/`：planner/controller、BT、global/local costmap、velocity smoother。
- `config/safety/`：Collision Monitor、命令 watchdog、限速状态。
- 实机配置应按设备序列号/平台型号分层，不在同一个 YAML 中混合仿真和实机参数。

## 5. Launch 规划

建议分别提供：
- `sensors.launch.py`：仅启动设备驱动。
- `description.launch.py`：robot_state_publisher/TF。
- `localization.launch.py`：local EKF + GNSS/global EKF（支持禁用 GNSS 的室内模式）。
- `perception.launch.py`：双目与 LiDAR 预处理。
- `nav2.launch.py`：Nav2 servers、BT、costmaps。
- `safety.launch.py`：Collision Monitor 和命令安全链。
- `robot_bringup.launch.py`：完整系统组合，支持仿真/实机参数选择。
- `diagnostics.launch.py`：健康状态和日志。

每个 launch 必须提供 namespace、use_sim_time、参数路径和组件开关，避免 launch 文件里硬编码设备 topic。

## 6. 参数与版本管理

每次实机参数变更记录：
- ROS_DISTRO、Ubuntu、RMW；
- Nav2 / robot_localization / image_pipeline 版本；
- 驱动仓库及 commit；
- 机器狗固件/SDK 版本；
- 传感器序列号、固件、安装外参；
- 标定文件 hash；
- 地图、行为树、模型和配置版本。

## 7. 尚不创建实机 package 的原因

已建立 [Lite3 bridge 接口契约](lite3_bridge_interface.md) 和 [厂商 SDK 集成评估](lite3_vendor_sdk_integration_assessment.md)。当前公开 SDK 路径包含 Foxy/DRDDS，不能假定其与 Nav2 Jazzy 通过 DDS 互通。项目自有 bridge 先保持标准消息接口和 mock 测试；vendor-side gateway 需在厂家确认 motion-host/AGX host 拓扑后再创建。不得把厂商 SDK 或其二进制直接复制进公共仓库，除非完成 license/SBOM 审核。
