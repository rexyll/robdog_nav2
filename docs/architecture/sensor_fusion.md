# 传感器接入与融合设计

## 1. 总体原则：融合分层、保留来源

LiDAR、双目 RGB、RTK GNSS、IMU 和腿式里程计的数据不能直接做同一种数学融合。Lite3 的当前 sdk_deploy 不应假定等同于 Legacy Lite3_ROS 的 /leg_odom2；可信本体 odometry 是阶段 A 必须向厂商确认的数据契约。

| 融合层 | 数据 | 目标 | 主要机制 |
|---|---|---|---|
| 本体状态估计 | IMU + 经验证的腿式 odometry/接触状态；如无则评估 LiDAR/stereo odometry | 平滑姿态、速度和局部位置 | robot_localization EKF 或经验证的状态估计器；无可信局部 odom 时不能闭环 |
| 全局定位 | local odom + RTK GNSS | map 坐标位置约束 | navsat_transform + global EKF |
| 几何障碍感知 | LiDAR + stereo PointCloud2 | 局部空间占据 | Nav2 ObstacleLayer/VoxelLayer 多观测源 |
| 语义融合 | RGB 检测结果 + 深度/点云 | 目标类别、距离、轨迹 | 目标检测、深度关联与多目标跟踪 |
| 地形融合 | 3D 点云 + IMU 姿态 | 地面坡度、台阶、粗糙度、负障碍 | 地面分割、elevation/traversability grid |

## 2. Lite3 状态输入前置条件

- Legacy Lite3_ROS 的 /leg_odom2 是 Foxy 标准消息路径，但源码中 steady-clock stamp、child_frame_id 和 covariance 需要审查。
- Current sdk_deploy 输出 DRDDS /IMU_DATA、/JOINTS_DATA 和 RobotStatus；公开 RobotStatus 不等于完整机身 Odometry。
- Foxy 与 Jazzy DDS 不能假设兼容；厂商侧和 Nav2 侧应隔离，通过版本无关的非 DDS gateway 传递数据。
- 在厂家确认完整 legged odometry 之前，local EKF 不得把 IMU-only 的位置积分当成可信 odom。候选补充来源是经验证的 LiDAR odometry/SLAM 或 stereo visual odometry。
- 厂商路径、接口、commit 和安全注意事项见 [Lite3 SDK 集成评估](../implementation/lite3_vendor_sdk_integration_assessment.md)。

## 3. 本体局部里程计与 LIO 备用路线

### 3.1 首选与备用顺序

1. 首选厂商当前固件提供的可信 legged odometry/body velocity；先确认时间戳、frame、covariance，以及该估计是否已融合 Lite3 IMU。
2. 如果 current sdk_deploy 不提供可信 odometry，第一备用是 **Point-LIO + MID-360 点云 + MID-360 内置 IMU**，在 Jetson AGX Orin 上先做独立 POC。
3. FAST-LIO2 作为对照算法；其上游 GPLv2 许可需要单独审查。
4. Stereo VIO/FAST-LIVO2 放在后续评估，不能在双目/IMU 时空外参未校验前进入主定位链。

### 3.2 Point-LIO 输入契约

- LiDAR 输入使用 Livox ROS Driver 2 的 Livox CustomMsg，必须保留每个点的时间偏移字段；不要用丢失逐点时间信息的仅帧级 PointCloud2 代替 LIO 输入。
- IMU 输入优先使用 MID-360 内置 IMU 的驱动输出（例如驱动配置映射后的 `/livox/imu`），先确认采样率、单位、轴向、量程和时间戳。
- Point-LIO 上游明确要求 LiDAR 与 IMU 同步；要核验 MID-360 的 device timestamp/PTP/GPS 时间模式，测 offset/drift，不可把 `time_sync_en` 或默认 offset 参数当作同步已完成。
- 完成 `imu_link ↔ lidar_link` 外参标定，核对 Point-LIO 源码配置对 extrinsic_T/extrinsic_R 的坐标定义。
- LIO 第一个 POC 不同时融合 Lite3 IMU，以避免两套未经校准的 IMU 时间域污染估计。Lite3 IMU 可并行录制用于交叉验证。
- 输出 odometry 的 frame、child_frame、covariance 和 TF 发布者必须核实；若 Point-LIO 已发布 `odom → imu_link`，通过标定静态变换得到 base_link，不允许重复发布同一 TF。
- 选择 Point-LIO 的理由是其逐点更新、抗强振动/IMU 饱和的设计特点；Intel Robotics AI Suite 的 Jazzy port 提供 MID-360 参数文件，但其公布的端到端运行验证使用 Velodyne 和 Intel Core Ultra，不代表本项目 Orin + MID-360 已通过。
- FAST-LIO2 Jazzy build 也有参考；但 Intel 文档指出其上游实际 LICENSE 为 GPLv2，不能因 package.xml 声明 BSD 就误判许可证。

### 3.3 与 RTK / global EKF 的关系

- 先让厂商 legged odometry 或 LIO 单独提供连续 local odom。
- 若该里程计输出本身质量足够，直接作为 Nav2 local costmap 的 `odom` 来源；不必为使用 robot_localization 而再次对同一 LIO 输入做重复滤波。
- Global EKF 融合 local odometry 与经质量门控的 RTK Cartesian odometry，输出 `map → odom`。
- 若另加 robot_localization local EKF，必须确认不同输入的信息不是同一底层 IMU/里程计的重复估计，并填充可信协方差。
- RTK 只提供全局位置约束；静止单天线 GNSS 不能替代 yaw 观测。

### 3.4 不适用和失败处理

- Point-LIO/FAST-LIO2 的 build pass 不代表 Jetson 性能通过；必须在 AGX Orin 上测 CPU、内存、温度、odom 更新率、P95/P99 latency 和 bag/现场轨迹误差。
- MID-360 的 IMU 与点云时间戳、安装外参、点云逐点 timestamp 任一不合格，LIO 输出不得用于 Nav2。
- 如果 LIO 在步态冲击/足滑/动态场景下漂移过大，保留厂商 legged odometry（若可用）并将 LIO 作为监测/备用；再评估 stereo VIO 或其他状态估计器。
- 详细评估见 [odometry fallback strategy](../implementation/odometry_fallback_strategy.md) 和 [ADR-0004](../decisions/ADR-0004-local-odometry-fallback.md)。

## 4. LiDAR 接入

### 4.1 消息形式

根据设备选择，不预先假设：
- 2D LiDAR：通常发布 `sensor_msgs/msg/LaserScan`（例如 `/scan`）。
- 3D LiDAR：通常发布 `sensor_msgs/msg/PointCloud2`（例如 `/lidar/points`）；如有需要，可通过投影节点生成某个扫描平面的 LaserScan。
- 设备还可能发布原始 packet、diagnostics、时间同步状态和 IMU。

### 4.2 预处理

1. 时间戳必须对应采集时刻，而不是回调处理时刻。
2. 按 range/intensity/有效性去除异常测量。
3. 根据 URDF/TF 做机器人本体自体过滤。
4. 3D LiDAR 需评估地面分割和高度裁剪。
5. 对旋转扫描，检查逐点/逐包时间信息，必要时用 IMU/odom deskew。
6. 四足机身倾斜时，不能简单认为 2D scan 永远水平；需明确扫描平面在 map/odom 中的姿态。
7. 保留原始 topic，输出预处理 topic，便于 rosbag 复查。

### 4.3 对 Nav2 的接口

- 2D scan 进入 ObstacleLayer 的 observation source。
- 3D points 可进入 ObstacleLayer 或 VoxelLayer 的 PointCloud2 source。
- 若使用 VoxelLayer，配置 z resolution、voxel count、mark threshold、min/max obstacle height。
- `marking` 用于标记障碍；`clearing` 只有在射线清除语义和传感器视场验证后才开启。
- 对 2D LiDAR 高度盲区、悬空障碍和地面负障碍要单独评估。

## 5. 双目普通 RGB 相机

### 5.1 硬件与标定前提

两台普通 2D 相机组成被动双目，至少要求：
- 刚性固定，基线在运行中不变化；
- 左右相机视场有充分重叠；
- 分别完成内参和畸变标定；
- 完成左右相机外参/基线标定；
- 图像尽量硬件同步；动态场景中异步采集会把物体运动误判为视差；
- 快速运动优先全局快门，并限制曝光导致的运动模糊；
- 提供准确 CameraInfo、frame_id 和采集时间戳。

### 5.2 软件流水线

```text
left/image_raw + left/camera_info
right/image_raw + right/camera_info
                 |
           approximate/exact synchronization
                 |
           stereo rectification
                 |
             disparity map
                 |
       disparity confidence / invalid mask
                 |
           depth reconstruction
                 |
          PointCloud2 generation
                 |
  confidence/range/height/ROI/self-filter/downsample
                 |
           Nav2 local costmap
```

ROS 2 可评估 `image_pipeline` / `stereo_image_proc`、OpenCV StereoSGBM 或经过测评的深度学习立体匹配方法。最终选型要考虑 ARM/GPU 支持、算力、帧率、端到端延迟、许可和部署复杂度。

### 5.3 深度原理与误差

理想针孔双目模型：

\[
Z = \frac{fB}{d}
\]

其中 (Z) 为深度，(f) 为像素单位焦距，(B) 为基线，(d) 为左右对应点视差。

近似误差：

\[
\sigma_Z \approx \frac{Z^2}{fB}\sigma_d
\]

因此距离越远，深度误差通常以近似平方关系放大；增加基线、有效分辨率或亚像素匹配精度可以改善测距，但更大基线会增加近距离遮挡并缩小重叠范围。

### 5.4 双目适用边界（前序讨论结论）

**适合优先评估的场景：**
- 室内移动导航、仓储、行人和较大障碍物检测；
- 墙壁、箱子、桌椅、货架等有足够纹理且双目同时可见的物体；
- RGB 目标检测与距离估计联合使用；
- 户外环境中，作为被动可见光系统，不需要主动投射红外纹理。

**不能作为唯一可靠深度来源的场景：**
- 透明玻璃、透明塑料、镜面和高反光金属；
- 大面积无纹理单色表面；
- 极暗环境、强逆光或严重曝光变化；
- 快速移动且左右相机不同步；
- 细电线、细杆、极小物体、深度边界和强遮挡；
- 极近距离或机械臂/足端精细操作，除非精度经严格验证。

注意：主动 RGB-D/ToF/结构光也各有材质和环境限制，并非所有困难场景都能由 RGB-D 完美解决。

### 5.5 视觉结果进入 Costmap 的规则

- 以独立的 `/stereo/points` 或经过过滤的 `/perception/obstacle_points` 接入，不覆盖原始图像/视差。
- 过滤无效视差、低置信度、近远边界异常、地面误检、自体点和动态模糊帧。
- 深度点云适合作为 LiDAR 盲区补充，不应未经实测取代 LiDAR 的稳定测距。
- 若 PointCloud2 作为 obstacle source，需验证传感器原点、射线 clearing、范围和遮挡逻辑；不能把“没有点”误解为“空间空闲”。
- 语义检测与跟踪应发布独立目标消息/marker 或自定义接口；语义类别不直接决定 free space。

## 6. RTK GNSS 接入

### 6.1 输入

优先使用驱动发布：
- `sensor_msgs/msg/NavSatFix`：经纬度、高度、status、position covariance；
- 原始接收机状态：RTK fixed/float、差分龄期、卫星数、DOP、基线状态等；
- 如设备支持，发布双天线航向或独立 heading；
- 接收机时间戳、PPS 或主机时间同步状态。

`NavSatFix` 一般不包含足以评估所有 RTK 状态的完整原始诊断，所以应保留厂商状态消息并建立质量门控。

### 6.2 坐标转换和融合

参考架构：
1. local EKF 提供连续 `odometry/filtered/local`。
2. `navsat_transform_node` 接收 local odom、可信 IMU 航向和 `gps/fix`。
3. 输出 `odometry/gps`（以当前 world frame 为参考的 Cartesian 坐标）。
4. global EKF 融合 local odom 与 GNSS odometry，发布 `map → odom`。
5. Nav2 的 global frame 使用 map；local costmap 通常使用 odom。

`navsat_transform_node` 的输入/输出和 UTM/Local ENU 选项以所锁定的 ROS 发行版文档为准。局部场地可考虑 local ENU；跨较大区域或跨 UTM zone 时必须单独处理坐标连续性。

### 6.3 质量门控

不能只按 NavSatFix 有消息就融合。需结合：
- RTK fixed/float/single 状态；
- covariance 与厂商估计精度；
- correction age、卫星质量和 DOP；
- 观测时间戳/数据年龄；
- 与当前 local odom 的创新量/位置跳变；
- 天线遮挡、多路径、反射环境；
- 失去改正后何时冻结、降权、恢复。

需由阶段 A 定义数值阈值。阈值不能从示例直接照搬。

### 6.4 航向与天线杆臂

- 单天线 RTK 提供位置，不意味着机器人静止时有可信 yaw。
- 运动方向 course-over-ground 在低速/静止时不可靠，不能无条件当作机身航向。
- 优先使用经过标定的 IMU/magnetometer，或双天线 GNSS heading。
- RTK 天线与 `base_link` 原点存在杆臂偏移；在 URDF/TF 和 navsat 参数中一致处理。
- 磁偏角、IMU yaw offset、ENU/NED 约定和天线朝向必须显式记录。

## 7. IMU 与腿式里程计

腿式机器人机身状态估计通常受到足端接触、足滑、腾空、地面软硬、机身冲击和步态影响。需要确认厂商 odometry 内部是否已经融合 IMU，以防重复计入同一测量信息。

输入质量要求：
- IMU：单位、坐标轴、ENU/NED、orientation/angular_velocity/linear_acceleration covariance、bias 和 timestamp 正确。
- 腿式 odometry：明确 pose/twist 是 world/body frame、covariance 是否真实、运动中足滑如何处理。
- 若能输出 contact state、foot velocity、slip confidence，应保留并供状态估计/地形模块使用。

## 8. 推荐 Topic 契约（暂定名称）

实际名称以硬件驱动和机器人命名空间为准。

| Topic | 消息 | 用途 |
|---|---|---|
| `/lidar/scan` | LaserScan | 2D LiDAR 原始扫描 |
| `/lidar/points` | PointCloud2 | 3D LiDAR 原始点云（如适用） |
| `/stereo/left/image_rect` | Image | 左校正图像 |
| `/stereo/right/image_rect` | Image | 右校正图像 |
| `/stereo/disparity` | DisparityImage | 视差 |
| `/stereo/points` | PointCloud2 | 双目重建点云 |
| `/gps/fix` | NavSatFix | RTK 地理坐标 |
| `/gps/status` | 厂商/诊断消息 | Fixed/Float、改正龄期等 |
| `/imu/data` | Imu | IMU |
| `/legged/odom` | Odometry | 机器狗里程计 |
| `/odometry/filtered/local` | Odometry | local EKF 输出 |
| `/odometry/gps` | Odometry | navsat 转换输出 |
| `/odometry/filtered/global` | Odometry | global EKF 输出 |
| `/stereo/objects` | 自定义/vision_msgs | 语义目标/跟踪（阶段后续） |

## 9. 时间同步和 QoS

- 首选 GNSS PPS/PTP/硬件触发等可验证的共同时间基准。
- 双目左右图像同步误差必须按运动速度和精度预算量化。
- LiDAR 扫描应使用采集时间；如果消息只有整帧时间，需评估机身运动造成的误差。
- 传感器数据常使用 SensorDataQoS（best effort、有限队列），但必须与实际驱动发布端兼容；静态 TF/地图类数据的 QoS 另行配置。
- 所有处理节点要检查数据年龄；任何过期数据不能继续被当成实时观测。
- rosbag 记录原始数据和处理后数据，便于离线重放对比。

## 10. 明确的实现顺序

1. 先验证驱动、原始消息与 TF。
2. 做 URDF/外参和时间同步。
3. 先只用腿式 odom + IMU 跑通 local EKF。
4. 再加 RTK、navsat_transform 和 global EKF。
5. 单独验证 LiDAR costmap。
6. 单独验证双目深度与 costmap。
7. 两种障碍源并行接入，并测试冲突/遮挡/过期数据。
8. 最后引入语义目标跟踪和地形可通行性。
