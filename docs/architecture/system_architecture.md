> **当前硬件基线（技术预选，未采购）：** DEEP Robotics Lite3 Venture + Livox MID-360 + 2 × See3CAM_50CUG Color（被动双目）+ ArduSimple ZED-F9P RTK + Jetson AGX Orin。首轮软件参考 Ubuntu 22.04 / JetPack 6.1 / ROS 2 Humble，长期迁移 Jazzy。项目进度只以根目录 [TaskPlan.md](../../TaskPlan.md) 为准。

# 系统总体架构

## 1. 设计目标

用三类外部传感器（LiDAR、RTK GNSS、两台普通 RGB 相机）加机器狗原生 IMU/腿式里程计，形成从传感器到 Nav2 再到底层四足运动控制的闭环。此架构区分五类信息：

1. **几何测量：** LiDAR、双目深度。
2. **语义信息：** RGB 目标检测、类别、目标跟踪 ID。
3. **本体运动：** IMU、腿式里程计、足端接触/滑移状态。
4. **全局约束：** RTK GNSS、已知地图/地理航点。
5. **执行约束：** 四足运动学、步态状态、速度/加速度限制和停止状态。

不要将这五类信息无差别拼接。它们的误差模型、时间特性和故障模式不同。

## 2. 逻辑模块

| 模块 | 输入 | 输出 | 职责 |
|---|---|---|---|
| Sensor Drivers | LiDAR/cameras/RTK/IMU | 原始 ROS 消息 | 设备驱动、状态、时间戳 |
| Time & TF | 设备时间、URDF、标定 | TF、同步状态 | 时间统一与坐标变换 |
| Legged State Estimator | IMU、腿式 odom、足端状态 | local odom、姿态 | 提供连续的本体运动估计 |
| GNSS Localization | NavSatFix、IMU heading、local odom | GNSS odom、map→odom | 地理坐标投影和全局约束 |
| LiDAR Perception | LaserScan/PointCloud2 | 几何障碍观测 | 主几何障碍源 |
| Stereo Perception | 左右图像、CameraInfo | disparity/depth/PointCloud2 | 视觉深度及补盲 |
| Semantic Perception | RGB 图像、几何深度 | 目标类别/轨迹 | 行人/车辆等类别和跟踪 |
| Nav2 | 地图、TF、costmaps、目标 | 路径、速度命令 | 全局规划、局部控制和任务恢复 |
| Safety/Command Gateway | Nav2 cmd、传感器、底盘状态 | 限幅 cmd、stop | 命令过滤、watchdog、状态闭环 |
| Legged Locomotion | cmd_vel/步态状态 | 电机/关节命令 | 四足步态、落足和底层稳定控制 |

## 3. 核心数据流

### 定位数据流

```text
IMU + Legged Odometry (+ contact/slip)
            |
            v
       Local EKF
            |
            +----> /odometry/filtered/local
            |             |
            |             +----> odom -> base_link
            |             |
            |             +----> navsat_transform_node
            |                              ^
            |                              |
RTK /gps/fix ------------------------------+
                                           |
                                           v
                                  /odometry/gps
                                           |
                                           v
                            GNSS quality gate / Global EKF
                                           |
                                           +----> map -> odom
```

local EKF 负责连续性，global EKF 负责全局位置修正。GNSS 修正应进入 `map→odom`，不应让局部控制依赖跳变的全球经纬度。

### 障碍物数据流

```text
LiDAR driver -> /scan or /points -> time/TF/filter/self-filter ---+
                                                                    +--> local costmap
Left RGB + Right RGB -> stereo rectify/disparity/depth -> cloud ----+
                                                                    |
RGB -> object detector/tracker -> semantic tracks -----------------+
```

每个传感器都应保留独立 topic 和诊断状态，costmap 层可以融合多个 observation source。除非证明时间、坐标、密度、噪声和清除射线行为一致，否则不把 LiDAR 和双目点云预先合并成单一 PointCloud2。

## 4. 为什么采用分离的局部与全局定位

- `odom`：短期连续、允许缓慢漂移；Nav2 local costmap/controller 使用它。
- `map`：长期/全局参考，可以由 GNSS、SLAM、地图定位或它们的组合提供。
- `map→odom`：吸收 GNSS/地图对齐产生的全局修正。
- `odom→base_link`：由 local state estimator 唯一发布，保持平滑。

这样可以避免 RTK 从 Fixed 变 Float 或受多路径影响时，机器狗的局部坐标瞬间跳变，导致控制器产生不合理速度命令。

## 5. Nav2 costmap 分工

### Global costmap

根据任务选择：
- 室内：SLAM Toolbox/已知 OccupancyGrid 的 StaticLayer。
- 室外：RTK 地理航点/地理路线转换到局部 ENU 或 UTM；全局地图可选道路/禁行区/地理栅格。
- 混合：地图提供静态背景，动态障碍物由实时传感器更新。

### Local costmap

- rolling window，以 `odom` 为 global_frame。
- LiDAR 作为核心观测源。
- Stereo point cloud 用作补充观测源，重点补足不同高度/类别或 LiDAR 盲区。
- VoxelLayer 适用于必须区分垂直空间占据的场景；仅需二维障碍投影时可用 ObstacleLayer。
- InflationLayer 按机器狗外形和安全余量扩张障碍物。
- 所有传感器源必须明确高度阈值、范围、超时和 TF。

## 6. 四足特有的系统约束

### 6.1 机身姿态变化

四足机身会周期性俯仰、横滚和升降。固定在机身上的相机和 LiDAR 也随之运动：
- 点云应使用正确采集时刻的 TF。
- 3D LiDAR 或逐点时间戳可进行运动畸变补偿。
- 2D LiDAR 扫描期间机身旋转/俯仰可能使扫描不再是平面；需要 IMU 辅助补偿，或评估换用 3D LiDAR。
- 立体相机需要刚性基线；机架振动或两相机相对位移会破坏标定。

### 6.2 四足可通行性不是普通二维避障

Nav2 2D costmap 将环境压到地面平面，不天然表达：
- 脚可踩但机身不可穿越的高度差；
- 可跨越台阶与不可跨越台阶；
- 坡度、粗糙度、松软地面、沟槽和悬崖；
- 足端落点稳定性和步态执行器约束。

因此，第一版可先做平整地面导航；涉及复杂地形时必须增加 terrain/traversability 表示，并与底层 locomotion planner 协作。

### 6.3 命令约束

Nav2 的速度输出必须经过 adapter：
- 将 Twist 或 TwistStamped 映射到底层真实支持的平移/旋转维度；
- 限制速度、加速度和 jerk；
- 处理站立/行走/转向步态状态；
- 命令过期即停止；
- 控制器断线或定位/传感器失效进入明确的安全状态。

## 7. 关键设计原则

1. 不让 RTK 参与近距离避障。
2. 不让视觉语义输出替代原始几何观测。
3. 不允许一个 TF child 有多个实时 publisher。
4. 不允许用过时点云/Scan 清除新障碍。
5. 相机时间同步不足时，不计算可信双目深度。
6. 定位协方差、GNSS 状态、感知数据龄期和控制链路状态要可观测。
7. 任何安全距离、最大速度和停止距离都由实测和风险分析锁定。
