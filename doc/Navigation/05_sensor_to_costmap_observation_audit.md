# 05 — 传感器观测到 Nav2 Costmap 的输入链审计（N1）

更新日期：2026-11-25。固定上游源码：ros-navigation/navigation2 @ c7046550547d382a5186d106b0895eda02485d7c（Jazzy）。本文审计 Nav2 Costmap 如何消费 LaserScan/PointCloud2；不代表项目已完成 MID-360/双目运行接入。

## 1. 证据锚点
- [ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_costmap_2d/plugins/obstacle_layer.cpp)
- [VoxelLayer](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_costmap_2d/plugins/voxel_layer.cpp)
- [ObservationBuffer](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_costmap_2d/src/observation_buffer.cpp)
- [Nav2 default params](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_bringup/params/nav2_params.yaml)

文件 SHA：ObstacleLayer 4bdec66f1c6a944f085d71d2dc1bee586dada017；VoxelLayer af271c0da0a2344c5a1c487c372fa790ec769089；ObservationBuffer 8a06823df38428b3b1d515a4f67eefb6171d49bc。

## 2. ObstacleLayer 输入契约
ObstacleLayer::onInitialize() 对每个 observation source 声明/读取 topic、sensor_frame、observation_persistence、expected_update_rate、data_type、min/max obstacle height、marking、clearing、obstacle_min/max_range、raytrace_min/max_range。data_type 仅接受 LaserScan 或 PointCloud2；inf_is_valid 只适用于 LaserScan。

因此 Nav2 ObstacleLayer 原生支持 PointCloud2，MID-360 不需要伪装成 LaserScan。但必须符合 PointCloud2 字段契约；源码使用 PointCloud2ConstIterator<float> 读取 x/y/z。项目必须确保字段存在、类型匹配、坐标 frame 正确、数据有限且时间戳可用于 TF 变换。

### 2.1 TF 与消息过滤
ObstacleLayer 对 LaserScan 和 PointCloud2 创建 message_filters Subscriber，并用 tf2 MessageFilter 将消息过滤到 Costmap global_frame。订阅默认使用 sensor-data QoS profile，depth 设置为 50；TF tolerance 来自 Costmap transform_tolerance。若传感器 frame 到 global_frame 的变换在消息时间戳处不可用，消息无法正常进入 observation callback。

不能只检查 ros2 topic hz。还必须监控 TF lookup 成功率、MessageFilter 丢弃/排队、传感器时间戳是否落在 TF buffer 可用区间，以及队列积压引入的数据年龄。

### 2.2 Marking 与 Clearing
- marking：将观测到的点写入障碍代价。
- clearing：从传感器 origin 沿观测射线 raytrace free space。
- 同一个 source 可同时 marking 和 clearing，但两者的 range 参数和输入质量都必须审查。
- clearing 的前提是射线确实代表传感器观测到的自由空间；遮挡、稀疏点云、视场外、无回波、无效深度和动态物体不能被错误解释为空闲。

对 MID-360，不能把所有无点区域清成 free。必须验证点云是否有可靠射线/回波语义，点云预处理是否保留可用于清障的射线信息；若只有 hit points 而无可靠 free-space ray，考虑 marking-only 或独立清障观测策略，不能凭空构造 clearing rays。

### 2.3 高度和距离过滤
ObstacleLayer 按 min/max obstacle height 过滤点，并按 obstacle range、raytrace range 约束写入和清障。高度判断在 Costmap global frame 变换后的点坐标上执行，因此参数高度基准依赖 global_frame 的 z 轴定义与地面参考。

项目必须明确 base_link / lidar frame / odom / map 的高度基准；机器人姿态变化时地面高度如何估计；机身 pitch/roll、地面坡度、台阶和悬空点是否导致错误高度过滤；地面点过滤发生在 ObstacleLayer 之前还是之后；低矮障碍、悬空障碍和负障碍分别由哪个组件负责。

## 3. ObservationBuffer 新鲜度机制
ObservationBuffer 将 observation 放入缓存，并提供 isCurrent() 检查：
- expected_update_rate == 0 时直接返回 current=true，即不做更新频率超时检查。
- 非零时检查当前时间与 last_updated 的差是否不超过 expected_update_rate。
- observation_persistence 控制缓存 observation 的保留时间；它不等于端到端 freshness，也不能替代数据源健康监控。
- Costmap layer 的 current 状态由 marking/clearing observation buffers 的状态组合决定。

关键风险：source 使用默认 expected_update_rate=0 时，即使传感器长时间不更新，ObservationBuffer 的 isCurrent() 也不会仅凭该参数判 stale。项目必须显式配置期望更新周期，并在外层独立监测消息 age、丢包、驱动状态和 TF 状态。source_timeout、expected_update_rate、observation_persistence 是不同机制，不能互相替代。

## 4. VoxelLayer 能力边界
VoxelLayer 继承 ObstacleLayer 的传感器输入与 observation 管理，并增加有限高度方向的 voxel grid。源码参数包括 z_voxels、origin_z、z_resolution、unknown_threshold、mark_threshold 和 publish_voxel_map。

它处理 PointCloud2 的 x/y/z，将点投到体素结构，再将体素状态投影/组合到二维 Costmap。它不是完整三维地形地图：
- Z 方向层数有限（源码定义的位宽限制为 16）；分辨率和 origin_z 决定可表示的高度区间。
- 最终 Nav2 planner/controller 仍消费二维 Costmap 代价。
- 不表达地形表面连续高度、不确定度、坡度/粗糙度、台阶可跨越性、沟槽宽度、足端可落区域或机身稳定性。
- 对四足机器人，VoxelLayer 适合做一层 2.5D 障碍投影/高度分层辅助，不能替代 elevation/traversability pipeline。

## 5. Nav2 默认参数与项目传感器错配
固定 nav2_bringup/params/nav2_params.yaml 示例：
- local_costmap 使用 VoxelLayer，但 observation source 为 /scan、data_type=LaserScan。
- global_costmap 使用 ObstacleLayer，source 同样为 /scan、data_type=LaserScan。
- Collision Monitor 示例也使用 scan source。
- local/global Costmap 使用 2D footprint/robot_radius 近似机器人几何。

因此默认 YAML 没有直接接入 MID-360 PointCloud2，也没有接入 See3CAM 双目深度点云。尽管 ObstacleLayer/VoxelLayer 支持 PointCloud2，仍需项目单独配置 topic、source、height/range、TF、QoS、更新率、清障策略和转换管线。不能仅把 data_type 改为 PointCloud2 就认为完成适配。

## 6. 本项目建议的观测分流架构（设计建议，未实现）
```
MID-360
  ├─ 原始 Livox CustomMsg（保留 timebase + per-point offset）
  │    └─ LIO / deskew 专用链（不经过会丢点时间的转换）
  └─ 独立 PointCloud2 obstacle adapter
       └─ 自体过滤 / ROI / 地面分离 / 高度与有效点检查
            ├─ Nav2 local VoxelLayer / ObstacleLayer（PointCloud2 marking）
            ├─ 独立 free-space / clearing 证据链（有可靠射线时才启用）
            └─ 3D elevation + traversability map（保留高度/不确定度）

双目左右图像 + CameraInfo + hardware trigger
  └─ rectify / disparity / depth confidence / invalid-depth mask
       ├─ 近场几何 obstacle cloud（与 LiDAR source 独立）
       ├─ semantic detection/tracking（动态物体层）
       └─ traversability / negative-obstacle support
```

设计原则：
1. Point-LIO 继续使用 Livox CustomMsg 原始时间字段；Costmap 消费独立 PointCloud2，不覆盖 LIO 原始输入。
2. LiDAR 与 stereo 保持独立 observation source，分别监控 freshness、质量、TF 和数据 age；融合层保留 source provenance。
3. 地面点不应全部标记为障碍；但地面估计失败也不能把未观测区域当作 free。
4. 动态障碍应维护 tracking/prediction 和 observation age；静态 Costmap clearing 不能代替动态目标跟踪。
5. 对未知/遮挡/无效深度区域保留 unknown 或风险状态；不可默认 free。
6. elevation/traversability 输出应保留高度、坡度、粗糙度、置信度/不确定度和更新时间，再由明确的 adapter 映射到 planner cost 或约束。

## 7. N1 接口矩阵增量
| Producer | Consumer | 接口 | 当前源码证据 | 项目待核验 |
|---|---|---|---|---|
| MID-360 driver | LIO | Livox CustomMsg | 项目已有 v1.2.6 / SDK2 v1.3.1 源码审计 | 实际 Orin/设备时间同步和 topic |
| MID-360 adapter | ObstacleLayer/VoxelLayer | PointCloud2 x/y/z | Nav2 源码支持 PointCloud2 | adapter、字段、frame、QoS、时延 |
| Stereo depth pipeline | ObstacleLayer/VoxelLayer | PointCloud2 x/y/z | Nav2 源码支持 PointCloud2 | 双目标定、深度置信度、无效像素与近场性能 |
| ObservationBuffer | Costmap layer | observation cache/current | expected_update_rate=0 时不检查 stale | 显式 freshness gate 与 stale 行为 |
| ObstacleLayer | Costmap master | marking/clearing | 点标记、射线清障和高度/距离过滤源码 | 是否有真实 free-space 证据 |
| VoxelLayer | Costmap master | bounded voxel projection | 有限 z_voxels 的体素更新 | 与 terrain/elevation map 的分工 |
| Elevation/traversability | 四足 planner | 高程/风险/可通行性 | 非 Nav2 核心默认输入 | 项目接口和规划器消费方式未实现 |

## 8. 下一步
1. 继续追踪 Costmap2DROS update loop、layer plugin 更新顺序、Costmap current 状态如何传到 Planner/Controller。
2. 核验 PointCloud2 的消息字段布局、NaN/Inf、organized/unorganized、时间戳、点云 frame 与 TF 对 Costmap 的具体约束。
3. 对比 MID-360、双目近场深度、动态目标跟踪、地形高程四类观测的职责，形成独立质量门控。
4. 继续研究非 Nav2 的 elevation/traversability 实现与四足地形规划接口；N1 完成后转入 N2/N3/N4 专题，但不提前把未完成任务标为完成。

本报告是固定源码 E2 审计，不包含项目节点运行、PointCloud2 实际接入、Costmap 运行图或 Lite3 实机证据。