# 02 — 数据生产者、消费者与端到端流转

## 1. 核心原则

每条数据都必须明确：生产者、消息类型、时间戳语义、frame、消费者、失效策略。Topic 名称可被 namespace/remapping 改写；以下是常见接口和项目建议名，不是已经冻结的运行配置。

## 2. 生产者—消费者矩阵

| 数据 | 建议接口 | 类型 | 生产者 | 消费者 | 关键 Gate |
|---|---|---|---|---|---|
| 原始 LiDAR | /livox/lidar | livox_ros_driver2/CustomMsg | MID-360 driver | 转换/deskew | timebase、offset_time、丢包、frame |
| 几何点云 | /points、/points_filtered | PointCloud2 | LiDAR preprocessing | Local Costmap、Collision Monitor、RViz | stamp、TF、range/height、自体过滤 |
| 左右图像 | /left/image_raw、/right/image_raw | Image | camera drivers | image_proc/stereo_image_proc、检测器 | 硬同步、曝光、encoding、CameraInfo |
| 相机内参 | /left/camera_info、/right/camera_info | CameraInfo | driver/calibration | rectify、stereo matching | 标定与图像尺寸一致 |
| 双目视差/深度 | /stereo/disparity、/stereo/depth | DisparityImage / Image | stereo pipeline | 点云/感知 | 左右同步、基线、有效视差 |
| 视觉点云 | /stereo/points | PointCloud2 | depth-to-cloud | Local Costmap / Collision Monitor | confidence、范围、遮挡、TF |
| IMU | /imu/data | Imu | 本体/外置 IMU driver | LIO、robot_localization | 轴向、单位、bias、stamp、covariance |
| 腿式 odom | /leg_odom（候选） | Odometry | 厂商估计器/adapter | Local EKF、Nav2 | frame、速度定义、协方差、接触/滑移 |
| LIO/VIO odom | /odometry/lio | Odometry | Point-LIO/VIO | Local EKF 或 Nav2 | drift、时间对齐、frame、协方差 |
| local odom | /odometry/filtered/local | Odometry | Local EKF | Nav2、navsat_transform | 连续性、低延迟、唯一 TF authority |
| GNSS 原始位置 | /gps/fix | NavSatFix | ZED-F9P driver/adapter | navsat_transform、quality gate | fix type、covariance、iTOW/epoch、天线 |
| GNSS 投影 odom | /odometry/gps | Odometry | navsat_transform_node | Global EKF | datum、heading、lever arm、covariance |
| global odom | /odometry/filtered/global | Odometry | Global EKF | 地图/全局监控 | GNSS quality、跳变、frame |
| TF | /tf、/tf_static | TFMessage | EKF、SLAM、robot_state_publisher | Costmap、Controller、感知、RViz | 唯一 authority、时序、连通性 |
| 静态地图 | /map | OccupancyGrid | map_server / slam_toolbox | Global Costmap StaticLayer | map frame、resolution、origin |
| 全局路径 | Action result / Path topic | Path | Planner Server | BT、Controller、RViz | frame、可行性、有效性 |
| 导航命令 | cmd_vel_nav | Twist 或 TwistStamped | Controller/Behavior | Velocity Smoother | 版本、速度限制 |
| 平滑命令 | cmd_vel_smoothed | Twist 或 TwistStamped | Velocity Smoother | Collision Monitor | timeout、accel/decel |
| 安全命令 | cmd_vel_safe | Twist 或 TwistStamped | Collision Monitor | 四足 adapter | source freshness、zone、stop |
| 底层运动命令 | vendor-specific | 厂商接口 | 四足 adapter | locomotion controller | mode、步态、限速、watchdog |
| 底层反馈 | /joint_states、/robot_status | JointState/厂商消息 | Lite3 SDK | robot_state_publisher、diagnostics | freshness、control ownership |
| 诊断 | /diagnostics | DiagnosticArray | 各模块 | supervisor、监控面板 | stale/error |

## 3. 完整数据链

### LiDAR → Costmap
MID-360 packet/CustomMsg → 驱动解包 → 时间戳/逐点时间检查 → deskew（依赖运动估计和同步）→ 距离/地面/自体/噪声过滤 → PointCloud2 → TF 到 costmap frame → ObstacleLayer 或 VoxelLayer → Inflation → Planner/Controller。

原始 CustomMsg 不等于所有 Nav2 插件可直接消费的 PointCloud2。必须明确转换节点和字段映射。逐点 offset_time 不等于 header.stamp；deskew 要使用正确时钟模型。

### Stereo → Costmap
左右 Image + CameraInfo + 同步 → rectification → disparity → depth → PointCloud2 → 置信度/范围/高度/自体过滤 → TF → Costmap。透明、反光、弱纹理、弱光和运动模糊会造成错误深度。

### GNSS / IMU / Odom → TF
IMU + 经验证的腿式 odom 或 LIO/VIO → Local EKF → 连续 odom→base_link；GNSS 经质量门控、时间历元配对、天线杆臂和坐标投影 → GPS odom → Global EKF/定位器 → map→odom。Global correction 不应破坏 local odom 连续性。

### Navigation → Actuation → Feedback
Goal → BT → Planner Path → Controller（Path + local costmap + odom）→ cmd_vel → smoother → collision monitor → adapter → gait controller → robot motion → IMU/odom/joint/contact feedback → estimator → TF/costmap/controller。

## 4. 接口类别

常见 Actions：NavigateToPose、NavigateThroughPoses、ComputePathToPose、ComputePathThroughPoses、FollowPath、SmoothPath、FollowWaypoints，以及 Spin/BackUp/Wait/DriveOnHeading 等行为 Action。名称和可用性须按已安装 nav2_msgs 与 BT plugin 核验。

Services 常用于 Costmap 清理、地图加载/保存、Lifecycle 状态切换和参数操作。Topic 用于持续流；Parameter 用于运行配置。导航 Action 不是底盘速度命令。

## 5. TF 与数据契约

最低 TF：map → odom → base_link → sensor frames。常见配置 global_frame=map、local_frame=odom、robot_base_frame=base_link。base_footprint 是常见二维投影 frame，但四足机身俯仰/横滚明显，必须明确定义语义，不能随意替代 base_link。

每个输入源登记：topic/type/QoS；header.stamp 来源；frame/单位/轴约定/外参来源；频率/数据年龄/延迟/jitter；covariance 生成方法和未知值语义；丢包/乱序/重复/无效值处理；消费者/timeout/失效动作；测试 bag 和验收证据。

Dynamic03 研究表明，消息数量和时间戳高度吻合不自动证明 GNSS RTK Fixed 精度。Nav2 不会替你判断 GNSS 是否 RTK Fixed，也不会自动修复时间同步或 TF 错误。
