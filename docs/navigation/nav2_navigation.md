# Nav2 导航、Costmap 与实时避障

## 1. Nav2 在系统中的定位

Nav2 负责：
- NavigateToPose / NavigateThroughPoses 等任务入口；
- 全局路径规划；
- 局部路径跟踪和速度控制；
- global/local costmap；
- Behavior Tree 任务组织、重规划和恢复行为；
- 通过插件扩展 planner/controller/costmap layer。

Nav2 不是四足步态控制器，也不是完整地形/落足规划器。它给出底盘级速度命令，必须通过机器狗专用 adapter 转成底层控制器可执行的命令。

## 2. 建议节点链

```text
Mission / Waypoint Client
          |
      BT Navigator
          |
 Planner Server ------ Global Costmap
          |
 Controller Server --- Local Costmap
          |
    Velocity Smoother
          |
   Collision Monitor
          |
   cmd_vel adapter
          |
  Quadruped Controller
          |
   gait / foot planner / actuators
```

实际节点 topic/remap 应在 ROS 发行版、Nav2 默认参数和机器狗接口确认后锁定。特别留意 Jazzy 及更新版本之间 Twist/TwistStamped 相关参数差异。

## 3. Global Costmap

根据运行场景选用：
- 静态地图：StaticLayer 读取 OccupancyGrid。
- SLAM：SLAM Toolbox 或其他经验证的建图/定位方案。
- 户外 GNSS 航点：将经纬度目标统一投影到本地 ENU/UTM 坐标后发送 Nav2，不把经纬度直接当作 map 的 x/y 米坐标。
- 静态禁行区：地图层或 keepout filter（需评估使用版本）。

global costmap 负责长距离规划，不应直接用高频原始点云无限扩张全局地图。

## 4. Local Costmap（实时避障核心）

建议采用 rolling window，以 `odom` 为 global_frame。

### 观测源

| Source | 数据 | 用途 |
|---|---|---|
| LiDAR | LaserScan 或 PointCloud2 | 首要几何障碍源 |
| Stereo | 过滤后的 PointCloud2 | 补充高度/视场/视觉可见障碍 |
| 可选语义层 | 行人/动态目标 track | 动态障碍扩展、速度相关安全余量 |

### ObstacleLayer 与 VoxelLayer

- **ObstacleLayer：** 适合将 LaserScan/PointCloud2 观测投影到二维 costmap。
- **VoxelLayer：** 维护局部三维体素占据，再投影到二维导航代价；适用于区分障碍高度的场景。
- 两者不是简单的“谁更好”，取决于 LiDAR 是 2D 还是 3D、Stereo 点云质量、机器狗高度包络和计算预算。
- 对点云 source，验证 marking、clearing、raytrace_range、obstacle_range、min/max_obstacle_height。
- 相机深度数据中“没有点”可能是无纹理/透明/遮挡/无效深度，不一定表示自由空间；谨慎启用 clearing。
- 对每个 source 设置 expected_update_rate/source timeout 和有效 range，过期源应被视为失效。

### Inflation 与 footprint

- 机器人 footprint 需要包含机身和所有会在移动中侵入的结构；四足步态时外廓可能变化。
- inflation 半径需结合定位不确定度、感知误差、控制延迟、停止距离和机身安全间隙计算。
- 不直接复制 Nav2 示例的 robot_radius/inflation_radius。
- 若机器狗在转弯时外廓明显变化，可评估动态 footprint，但必须保证发布频率和坐标正确。

## 5. Planner / Controller 选型

不要预先锁定单个 planner/controller。建议对候选项做实测：
- Planner：NavFn/Smac 2D 适用于相应的栅格/运动约束；Smac Hybrid-A* 更适合有明确非完整约束的运动模型。
- Controller：Regulated Pure Pursuit 可作为简单基线；MPPI 可对复杂约束和多维速度采样进行评估。
- 四足机器人可能支持前进、侧移、原地转向或受限横移，实际能力要由底层控制器确定。
- Nav2 的 holonomic 参数、速度边界、最小速度、加速度和 jerk 必须与四足真实可达范围匹配。
- 若四足执行器只接受 gait command 或 twist 有专用语义，应由 adapter 完成，不应直接让 Nav2 输出绕开安全层进入电机控制。

## 6. Collision Monitor：快速软件级减速/停止

在 controller/costmap 路径之外，使用新鲜 LiDAR/深度观测做减速和停车区域监测：
- 输入可用 LaserScan、PointCloud2 等；
- 定义 stop polygon 和 slowdown polygon；
- 可按机器人当前速度使用 velocity polygon；
- 检查数据 source timeout、transform tolerance、base frame 和 odom frame；
- 点云源可以限制 min/max height，过滤地面回波和机器人自身结构；
- 输出只进入底层命令 gateway，确保不存在绕过 Collision Monitor 的第二条 Nav2 命令路径。

**关键限制：** Nav2 Collision Monitor 是 CPU 级软件功能，不是安全认证的 scanner/controller，也不保证硬实时。真实安全急停仍需评估独立硬件急停、底层 watchdog、制动和安全扫描设备。

## 7. 实时避障闭环

1. LiDAR/双目驱动发布带采集时间戳的数据。
2. TF 将观测转换到 costmap frame。
3. local costmap 更新障碍物。
4. controller 根据路径和局部障碍生成速度。
5. velocity smoother 限制命令变化率。
6. Collision Monitor 使用独立的实时传感器源做 stop/slowdown。
7. cmd_vel adapter 再按四足运动学、步态、机身状态和命令有效期限幅。
8. 底层控制器执行；没有新命令或通信异常时 watchdog 触发停止。
9. 发布实际速度/步态/故障状态，必要时反馈 Nav2 行为树。

## 8. 四足地形扩展

在平整地面第一版通过后，再评估：
- 3D 点云地面分割；
- 高程网格、坡度、粗糙度、台阶高度；
- 正/负障碍物检测（坑、沟、路沿、台阶下坎）；
- 可通行性代价图；
- 自定义 Nav2 costmap plugin；
- 与底层足端规划器/步态选择器的接口。

Nav2 local costmap 可以限制障碍物避让，但无法独立保证每只脚落在稳定地面上。复杂地形需要四足控制器提供可通行性约束和落足安全检查。

## 9. 建议的 Nav2 配置分层（不是可直接部署参数）

```yaml
# config/nav2/robot_nav2.yaml 的概念结构
# 数值与具体 topic、frame、参数类型必须在硬件确认后填写。
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  plugins: [static_layer, obstacle_layer, inflation_layer]

local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  rolling_window: true
  plugins: [voxel_or_obstacle_layer, inflation_layer]
  # observation_sources:
  #   - lidar_scan (LaserScan) and/or lidar_points (PointCloud2)
  #   - stereo_points (PointCloud2)

collision_monitor:
  base_frame_id: base_link
  odom_frame_id: odom
  # cmd_vel input/output topics are explicitly remapped
  # observation_sources: lidar and validated stereo cloud
  # polygons: slowdown and stop
```

## 10. Nav2 验收用例

- 静态箱体、墙壁、桌椅、货架；
- 障碍物突然放入机器人路径；
- 行人横穿/迎面接近；
- 低矮障碍、悬空障碍和扫描平面高度外障碍；
- 玻璃、反光、无纹理、弱光、逆光；
- 点云/scan 过期、丢帧、TF 断开；
- GNSS Fixed/Float/Lost；
- 原地旋转、侧移、倒退（底层支持时）；
- 急停、cmd_vel 超时、底层断连；
- 地面坡度/台阶/沟坎（只在阶段 F 定义能力边界后执行）。

每项要量测首次发现距离、检测延迟、停止距离、最小障碍间隙、误停率和漏检率。
