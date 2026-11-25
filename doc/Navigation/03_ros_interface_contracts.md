# 03 — ROS 2 核心接口契约与数据语义（Jazzy 源码快照）

更新日期：2026-11-25。接口定义来自 ROS 2 Jazzy 分支源码固定快照。本文区分“消息结构明确规定的语义”与“本项目仍待设备/节点确认的实际发布语义”。

## 1. 固定源码锚点

- ros2/common_interfaces Jazzy：a941f14bb318d8d904505ed935ccbb97f24a70a4
- ros-navigation/navigation2 Jazzy：c7046550547d382a5186d106b0895eda02485d7c

源码链接：
- https://github.com/ros2/common_interfaces/tree/a941f14bb318d8d904505ed935ccbb97f24a70a4
- https://github.com/ros-navigation/navigation2/tree/c7046550547d382a5186d106b0895eda02485d7c

## 2. 传感器和状态消息

### 2.1 sensor_msgs/Imu

源码：common_interfaces/sensor_msgs/msg/Imu.msg

字段：
- std_msgs/Header header
- geometry_msgs/Quaternion orientation
- float64[9] orientation_covariance
- geometry_msgs/Vector3 angular_velocity
- float64[9] angular_velocity_covariance
- geometry_msgs/Vector3 linear_acceleration
- float64[9] linear_acceleration_covariance

接口语义：
- 线加速度单位为 m/s²，不是 g；角速度单位为 rad/s。
- covariance 按 x/y/z 轴行优先存储。
- 某一估计量不可用时，该估计量 covariance[0] 应设为 -1；消费者必须检查并忽略对应估计量。
- 全零 covariance 表示 covariance 未知，不能解释为零噪声或完美测量。
- Header.frame_id、stamp、轴向、重力是否已去除、orientation 来源、bias/温漂和数据频率仍由具体 driver 说明。

对本项目的要求：
- 分别核验 MID-360 内置 IMU 与 Lite3 本体 IMU；不能混用未经对齐的 frame/time。
- LIO/VIO/EKF 需要明确 IMU 是否输出 orientation、是否含重力、噪声参数和协方差来源。
- 不得仅因消息类型为 sensor_msgs/Imu 就认为数据质量合格。

### 2.2 nav_msgs/Odometry

源码：common_interfaces/nav_msgs/msg/Odometry.msg

字段：
- std_msgs/Header header
- string child_frame_id
- geometry_msgs/PoseWithCovariance pose
- geometry_msgs/TwistWithCovariance twist

接口语义：
- pose 使用 header.frame_id 指定的坐标系。
- twist 使用 child_frame_id 指定的坐标系。
- pose/twist 均携带协方差。
- 该消息类型本身不保证来源是真实轮式/腿式里程计、LIO、VIO 或融合估计，也不保证时间戳/协方差可信。

对本项目的要求：
- 目标 local odometry 通常表达 odom→base_link，但 frame 命名和 authority 需要在 TF 设计中冻结。
- 需验证 pose 与 twist 的坐标语义、单位、正方向、时间戳、协方差、连续性和 freshness。
- Lite3 公开 RobotStatus 不能自动转换成完整 Odometry；缺失 xyz/linear velocity/covariance 时不得补零伪造。
- 确保同一传感器信息不被重复融合；明确 estimator 的输入、输出和 TF 发布者。

### 2.3 sensor_msgs/PointCloud2

源码：common_interfaces/sensor_msgs/msg/PointCloud2.msg

字段：
- Header header（注释语义为传感器采集时间和 3D 点坐标 frame）
- height、width
- PointField[] fields
- is_bigendian、point_step、row_step
- uint8[] data
- is_dense

接口语义：
- fields 描述二进制 data blob 中的字段布局。
- 点云可为二维组织结构或一维无序结构；深度相机可输出类似图像的组织点云。
- is_dense 声明是否包含无效点，但仍需对实际数据做校验。

对本项目的要求：
- 对 Livox CustomMsg 转 PointCloud2 的字段映射逐项核验：x/y/z、intensity、时间偏移、line/tag/return 等实际可用字段。
- 若 deskew 需要点级时间，必须证明时间字段被保留或在转换前完成 deskew。
- 明确点云是否已去畸变、是否去地面/自体、坐标 frame、点云时间戳、频率、点数、空帧策略。
- stereo cloud 需要记录深度置信度、无效深度处理、组织结构和相机内外参。

### 2.4 geometry_msgs/TwistStamped

源码：common_interfaces/geometry_msgs/msg/TwistStamped.msg

字段：
- std_msgs/Header header
- geometry_msgs/Twist twist

接口语义：
- TwistStamped 为带时间戳和参考 frame 的 Twist。
- Twist 自身包含 linear 与 angular 向量；单位和坐标语义需按接口约定明确。

对本项目的要求：
- 不能默认所有 Nav2 插件、速度平滑器、Collision Monitor 和 Lite3 adapter 使用同一种 Twist/TwistStamped 类型。
- 需核验目标 Nav2 Jazzy 配置是否启用 stamped velocity、各节点订阅/发布类型及 remap。
- Lite3 adapter 需定义参考 frame、单位、限幅、失联超时、命令仲裁和 ACK/反馈；禁止把高层 Twist 直接映射成关节目标。

## 3. Nav2 任务 Action

### 3.1 nav2_msgs/action/NavigateToPose

源码：navigation2/nav2_msgs/action/NavigateToPose.action

Goal：
- geometry_msgs/PoseStamped pose
- string behavior_tree

Result：
- uint16 error_code（NONE=0）
- string error_msg

Feedback：
- PoseStamped current_pose
- Duration navigation_time
- Duration estimated_time_remaining
- int16 number_of_recoveries
- float32 distance_remaining

含义：
- 上层通过目标 PoseStamped 和可选 BT XML 路径/标识请求导航。
- Feedback 是导航任务状态，不是底层关节执行反馈。
- Result error_code/error_msg 应与 action 状态一起记录；不能只检查客户端 future 是否返回。

### 3.2 nav2_msgs/action/FollowPath

源码：navigation2/nav2_msgs/action/FollowPath.action

Goal：
- nav_msgs/Path path
- string controller_id
- string goal_checker_id
- string progress_checker_id

Result：
- result（Empty）
- error_code / error_msg
- 错误码包括 INVALID_CONTROLLER、TF_ERROR、INVALID_PATH、PATIENCE_EXCEEDED、FAILED_TO_MAKE_PROGRESS、NO_VALID_CONTROL、CONTROLLER_TIMED_OUT 等。

Feedback：
- float32 distance_to_goal
- float32 speed

含义：
- FollowPath 是路径跟踪 action，允许选择 controller/goal checker/progress checker ID。
- Controller Server 内部的 path tracking 与外部任务层 NavigateToPose 是不同粒度的接口。
- 必须研究 cancellation/preemption、controller failure、TF error、timeout 和 progress checker 的传播方式。

## 4. 当前已确认的接口语义与未确认项

| 数据 | 标准消息可确定 | 本项目仍需确认 |
|---|---|---|
| IMU | 单位、协方差 sentinel、frame/header 结构 | 设备时钟、轴向、噪声、重力处理、频率、bias |
| Odometry | pose/twist frame 语义、协方差字段 | Lite3 是否真实提供、来源、连续性、covariance可信度 |
| PointCloud2 | 二进制布局、fields、header、组织结构 | MID-360/stereo 实际字段、deskew、时间语义、频率 |
| TwistStamped | 带时间戳/frame 的 Twist 结构 | Nav2 上下游是否 stamped、Lite3 adapter 是否兼容 |
| NavigateToPose | goal/result/feedback 字段 | 本项目 BT XML、任务策略、超时和错误处理 |
| FollowPath | path/controller/checker IDs 和错误码 | 实际 controller 插件、命令输出类型和实时约束 |

## 5. N1 下一步源码追踪

1. 从 Nav2 bringup launch 和参数文件确认节点、命名空间、remap、生命周期管理和实际接口类型。
2. 追踪 BtNavigator 的 NavigateToPose server、Navigator plugin、BT tick 和 blackboard 数据。
3. 追踪 Planner Server 的 ComputePathToPose、Costmap/TF 依赖和 Path 输出。
4. 追踪 Controller Server 的 FollowPath、controller plugin、odom/TF/Costmap 输入和速度输出。
5. 追踪 Velocity Smoother、Collision Monitor、平台 adapter 的消息类型、超时和安全行为。
6. 追踪实际底层命令接受/拒绝、执行反馈和停止链路；Lite3 部分在厂商资料未到位前标为待确认。

## 6. 证据限制

本文直接依据固定版本 .msg/.action 源文件整理结构与注释语义。它不证明任何实际设备正在发布这些消息，也不证明项目的 topic 名称、QoS、频率、延迟、协方差质量或执行链已接通。
