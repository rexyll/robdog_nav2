# MID-360 点云到高程地图与四足可通行性链路审计

- 专项分支：`nav`
- 审计日期：2026-11-25
- 证据类型：固定上游源码/官方仓库文档 E2；论文/项目文档 E1；项目自身集成与运行仍未验证
- 本文目标：把 MID-360 的原始输出、运动畸变处理、高程地图、地形特征与四足可通行性连接起来，并形成可落地的项目接口边界

## 1. 本轮核心结论

1. Livox ROS Driver 2 提供两种对本项目有意义的输出：
   - Livox CustomMsg：`timebase + per-point offset_time + xyz + reflectivity/tag/line`；
   - Livox PointCloud2 PointXYZRTLT：`xyz + intensity + tag + line + per-point timestamp`。
   这两种格式保留了逐点时间信息，适合后续 LIO/deskew；不要在定位前处理链路中降级为只有 XYZ 的 PointCloud2。
2. Nav2 ObstacleLayer/VoxelLayer 适合消费已完成时间补偿、坐标变换和质量筛选的 PointCloud2；它们不承担 Livox CustomMsg 逐点时间解码或运动去畸变。
3. 本轮审计的 ROS 2 Jazzy `elevation_mapping_cupy` 在 `pointcloud_callback` 中将 PointCloud2 解析为 XYZ（可选额外 channels），按整帧 header stamp 查一次 TF，并把 XYZ 送入地图融合。默认 XYZ 路径不会保留 Livox per-point timestamp/tag/line；也没有在该 callback 内逐点 deskew。
4. 因此，进入 elevation mapping / Nav2 前应先形成明确的 deskewed cloud contract：点坐标已补偿到约定参考时刻，header stamp 对应该参考时刻，frame_id 和 TF 语义一致；原始 CustomMsg/带逐点时间 PointCloud2 必须另行保留用于 LIO、诊断与回放。
5. `grid_map` 是多层 2.5D 网格容器，可承载 elevation、variance、surface normal、friction、foothold quality、traversability 等不同层；它不是任意多层 3D voxel map。
6. `elevation_mapping_cupy` 是当前更值得进入 ROS 2 Jazzy 候选验证的高程地图实现（CUDA 12 + CuPy，MIT）；但 Jetson Orin AGX + MID-360 的 FPS 单调下降问题曾被报告且仍显示 open。v2.2.0 公布的性能验证是在 RTX 4090，不可外推为 Orin 性能保证。
7. 传统 `leggedrobotics/traversability_estimation` 明确是 ROS Noetic / Ubuntu 20.04 项目，没有单元测试；可用于算法基线与特征定义，不能直接视为 Jazzy 可用包。它将 slope、roughness、step 和 surface normal 作为独立滤波层，并提供 footprint/path 检查概念。
8. 对四足机器人，仅用坡度/粗糙度/台阶几何分数仍不够：需要逐步纳入机器人特定的姿态/步态能力、落足可行性、接触/滑移、材质与不确定性；这些是项目扩展要求，不是上述传统过滤器已经具备的功能。

## 2. 上游固定版本与适用范围

| 上游项目 | 固定版本 | 适用范围 | 证据边界 |
|---|---|---|---|
| Livox ROS Driver 2 | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` | MID-360 输出格式/launch | 官方 README 和源码审计；厂商明确称 driver 是调试工具，不建议直接用于量产 |
| elevation_mapping_cupy | `d9e1c9fe42ea3d463519333a443da99f680adde8`（ROS2 branch） | ROS 2 Jazzy、CuPy/GPU 高程地图 | 当前分支源码审计；未在本项目 Orin 上构建或运行 |
| grid_map | `d9614992e36c66d3fe67d9125ca105a0a47676fe`（jazzy branch） | 多层 2.5D grid map | 官方 ROS2 Jazzy 分支源码/README；未在本项目构建 |
| traversability_estimation | `14d24c059e1c43466aadf328280adf6394d78039`（master） | 传统 rough-terrain 特征过滤参考 | README 声明 Noetic/Ubuntu 20.04；不作为 Jazzy 依赖承诺 |
| Navigation2 | `c7046550547d382a5186d106b0895eda02485d7c` | 2D costmap 消费端 | 见本目录 04–06 审计 |

许可证/维护状态：
- Livox ROS Driver 2：MIT；README 将其定位为调试/测试 driver，建议量产按需求优化。
- elevation_mapping_cupy：MIT；ROS2 `ros2` branch 主打 Jazzy + CUDA 12 + NVIDIA GPU，当前有持续提交。
- grid_map：BSD-3-Clause；维护 ROS2 分支（Humble/Iron/Jazzy/Rolling），但 Jazzy branch README 的开发说明仍包含 ROS2 rolling 依赖工作流，实际 Jazzy 构建仍需 CI/目标机确认。
- 传统 traversability_estimation：仓库 README 标注 ROS Noetic，且“暂无单元测试”；不直接纳入 Jazzy 软件 BOM。

## 3. MID-360 数据输出与时间信息

上游：`Livox-SDK/livox_ros_driver2`

固定源码：
- README：`https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/README.md`
- MID360 launch：`https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/launch_ROS2/msg_MID360_launch.py`
- 数据分发：`https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp`

### 3.1 CustomMsg 路径

`livox_ros_driver2/msg/CustomMsg` 关键字段：
- `header`
- `timebase`：首点时间基准
- `point_num`
- `lidar_id`
- `points[]`，每个 CustomPoint 有 `offset_time`、`x/y/z`、`reflectivity`、`tag`、`line`。

逐点采样时刻可按消息定义由 timebase 与 offset_time 组合恢复；必须核实单位、回绕、时钟域和设备时间到 ROS time 的映射，不能假设只看 Header stamp 就等于每点采样时刻。

### 3.2 PointCloud2 PointXYZRTLT 路径

Driver README 列出的字段：
- `float32 x/y/z`（米）
- `float32 intensity`
- `uint8 tag`
- `uint8 line`
- `float64 timestamp`（逐点 timestamp）

launch `xfer_format=0` 选择 Livox PointCloud2(PointXYZRTLT)；`xfer_format=1` 选择 CustomMsg。ROS2 的 `xfer_format=2` PCL PointXYZI 格式不支持，源码会报错并返回。

**工程决策建议：** 原始定位链保留 CustomMsg（或 PointXYZRTLT）与 IMU；另输出独立的 deskewed PointCloud2 给高程地图和 Nav2。不能只记录 deskew 后 XYZ 而丢弃原始逐点时间与原始消息。

### 3.3 时间补偿的接口约束

每个点需要关联采样时刻对应的传感器/机器人位姿，补偿到统一参考时刻（例如 scan end 或 scan midpoint，具体策略由 LIO 定义）。必须明确：
- driver 设备时钟、ROS header stamp、逐点 timestamp/offset_time 的时钟域与换算；
- IMU 与点云时钟同步（硬件同步/PPS/PTP 或经测量的 offset）；
- LiDAR-IMU 外参、LiDAR-base 外参及其版本；
- deskew 输出参考时刻与输出 frame；
- 丢失 IMU、时间戳异常、TF 缺失、队列积压时的拒绝/降级行为。

## 4. elevation_mapping_cupy 输入端源码审计

固定版本：`leggedrobotics/elevation_mapping_cupy @ d9e1c9fe42ea3d463519333a443da99f680adde8`

关键文件：
- `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_node.py`
- `elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping.py`
- `elevation_mapping_cupy/config/core/core_param.yaml`
- `elevation_mapping_cupy/config/core/plugin_config.yaml`

### 4.1 PointCloud2 解析

`_pointcloud2_xyz_f32()`：
- 要求 little-endian；
- 要求 x/y/z 都为 FLOAT32、count=1；
- 按 PointCloud2 的 point_step、row_step、field offset 解析组织点云；
- 检查 row_step 与 data buffer 长度；
- 非 dense 时过滤 NaN/Inf；
- 返回 (N,3) XYZ 数组。

若配置额外 channels，使用 ros2_numpy 读取 PointCloud2 并将 channels 拼入点数组。**默认 XYZ 路径会忽略 intensity、tag、line、逐点 timestamp 等其他字段**；除非专门配置/扩展，否则这些字段不会进入 ElevationMap 的几何融合算法。

### 4.2 TF 与时间语义

`pointcloud_callback()`：
1. 解析 XYZ；
2. 读取 `msg.header.frame_id`；
3. 以 `msg.header.stamp` 查询一次 `map_frame ← sensor_frame` transform；
4. 将同一个 R/t 应用于整帧点云并调用 `input_pointcloud()`；
5. 只有处理成功后才更新 `_last_t`。

这表明该节点自身没有用 Livox per-point time 做逐点去畸变。若输入是机器人运动期间采集、尚未 deskew 的旋转/扫描点云，整帧一个 transform 不能补偿扫描期间机身运动。

### 4.3 QoS 与发布

- PointCloud2 subscription 使用 ROS 2 sensor_data QoS（BEST_EFFORT）。
- publisher 以配置 FPS 定时发布 GridMap；无订阅者时跳过 map 序列化/发布工作。
- GridMap header stamp 取最近成功处理的点云时间；map 可能按定时器重复发布同一份观测结果。
- 因此消费端不能把“GridMap 消息刚刚发布”误认为“底层点云刚刚更新”。必须检查 header stamp 和 per-cell observation freshness。

### 4.4 当前版本的 observation_age

ROS2 branch 在 2026-10-03 的 commit 增加 `observation_age`：
- 以每个 cell 最后被接受的原始点云观测时间为依据；
- 不把 visibility/time timer、插值、语义图像、map load、patch 或新暴露的 shift padding 当作原始观测证据；
- 缺失、零、重复或回退的源时间戳不能授予新的 freshness；
- map 导出为 GridMap 时将内部 float64 age 保守转换为 float32。

这比单纯的 GridMap Header stamp 或传统 `time` layer 更适合作为传感器新鲜度证据。但它仍需与全局消息 age、source transport health、TF health、地图有效性共同使用；不能单独代表地形估计准确度。

## 5. grid_map：地图容器能力与边界

固定版本：`ANYbotics/grid_map @ d9614992e36c66d3fe67d9125ca105a0a47676fe`（jazzy）

核心能力：
- 一个二维栅格可携带任意数量的同尺寸 layer；
- circular buffer 支持机器人中心移动时高效 reposition；
- 可承载 elevation、variance、color、friction coefficient、foothold quality、surface normal、traversability；
- ROS 接口支持 GridMap、PointCloud2、OccupancyGrid、GridCells 等转换；
- grid_map_filters 支持阈值、法向量、平滑、方差、插值/修补和矩阵卷积；
- grid_map_costmap_2d 提供与 Nav2 costmap_2d 类型之间的转换包。

边界：
- 这是 2.5D 表达：每个 (x,y) cell 通常以一个或若干属性层描述地表；无法在同一平面位置无损表示多层地面、完整垂直结构、复杂悬垂物和任意多高度占据体。
- 对负障碍/悬崖，NaN/unknown 不能自动当作 free 或 unsafe 的唯一判断；需要 ray visibility、地面连续性、可观测性与风险策略共同判定。
- grid_map_costmap_2d 的转换只解决数据表示/接口互操作，不会自动把坡度、台阶、落足风险变成正确的 Nav2 cost；需要明确映射公式、unknown policy、lethal/inscribed/inflation semantics。

## 6. Elevation Mapping 与 Traversability 的候选比较

### 6.1 传统 elevation_mapping（ANYbotics）

官方 README 明确标注“不再积极维护”，是 ROS1/catkin 研究代码；输入 PointCloud2、机器人 PoseWithCovarianceStamped 与 TF，输出带 variance 的 robot-centric elevation map。它的核心研究价值是：
- 地图随机器人移动；
- 融合机器人位姿不确定性；
- 按点云时间从 pose cache 取对应机器人位姿；
- raw/fused map 分离。

**结论：** 保留为算法论文/历史实现参考，不作为本项目 ROS 2 Jazzy 主候选。

### 6.2 elevation_mapping_cupy（当前 ROS 2 候选）

官方 ROS2 branch 文档目标为 Ubuntu 24.04 + ROS 2 Jazzy + NVIDIA GPU + CUDA 12.x；Python/CuPy GPU 路径，支持点云/图像与语义层、ray visibility cleanup、drift compensation、插件处理与 GridMap 发布。

v2.2.0 release（2026-07-20）报告 RTX 4090 上的 callback 与 GridMap 构建性能优化，并有 77 项测试和 TF/GridMap integration 等验证。该测试证据不等于 Jetson AGX Orin 性能。

**Jetson 风险证据：** issue #137（2026-02-03 创建，2026-07-20 更新，仍 open）报告在 AGX Orin 32GB + MID-360 + Ubuntu 22.04 + ROS2 Humble 环境下，map FPS 从初始约 10–15 降到 10 分钟后低于 2，且延迟增加。该 issue 没有给出已确认根因/修复结论。它不证明 Jazzy v2.2.0 必然有同一缺陷，但足以将 Orin 长时间 soak test 设为硬 Gate。

**候选结论：** 将 elevation_mapping_cupy 作为 Jazzy/Orin 重点候选，但先做目标机 build、GPU memory/CPU/温度/功耗观测、10–60 分钟以上持续 rosbag replay、FPS/p95 latency/observation_age 监测；未通过不得冻结为正式依赖。

### 6.3 traversability_estimation（传统几何过滤参考）

固定版本 README 声明 ROS Noetic/Ubuntu 20.04，并明确无单元测试。算法层包含：
- Surface Normals：每格表面法向量；
- Slope：坡度代价；
- Roughness：局部高程变化/粗糙度；
- Step：台阶高度相关代价；
- footprint/path check：检查单个 footprint 或一串 footprint 的可通行性。

它从 elevation map 读取地图并通过滤波链生成 traversability map。ROS2 Humble 社区移植存在，但没有证据证明其与当前 Jazzy、grid_map Jazzy 和本项目接口直接兼容。

**结论：** 用作 baseline 算法与对照，不直接加入 Jazzy BOM；先将特征公式、unknown policy、footprint 语义抽取为项目独立算法接口，再决定移植或重写。

## 7. 项目建议的数据流与接口

```
MID-360
  ├─ CustomMsg / PointXYZRTLT + IMU（保留逐点时间）
  │      └─ LIO / state estimator
  │            ├─ odom + covariance + TF
  │            └─ deskewed PointCloud2（约定 reference stamp/frame）
  │                    ├─ Obstacle extraction / dynamic object pipeline
  │                    │      └─ Nav2 ObstacleLayer / VoxelLayer / Collision Monitor
  │                    └─ Elevation Mapping
  │                           └─ GridMap layers:
  │                                elevation / variance / observation_age
  │                                normals / slope / roughness / step
  │                                traversability / unknown / risk
  │                                      ├─ Nav2 cost projection (2D conservative layer)
  │                                      └─ Quadruped terrain planner / foothold interface
  │
  └─ Stereo RGB + CameraInfo + hardware trigger
         ├─ rectification / depth / confidence / invalid mask
         ├─ dynamic + semantic detection/tracking
         └─ optional depth/semantic fusion into elevation/semantic map
```

### 7.1 统一数据契约（项目设计建议，待 ADR 冻结）

| 输出 | 类型建议 | 关键语义 |
|---|---|---|
| raw lidar | `livox_ros_driver2/CustomMsg` + IMU | 原始采样时间与 LiDAR 原始观测留档 |
| deskewed cloud | `sensor_msgs/PointCloud2` | XYZ 已补偿到明确参考时刻；header stamp 是该参考时刻；frame_id 明确 |
| state | `nav_msgs/Odometry` + TF | pose/twist/covariance、frame 与时间一致 |
| elevation | `grid_map_msgs/GridMap` | elevation + variance + validity + observation_age |
| terrain features | `grid_map_msgs/GridMap` | normal/slope/roughness/step/friction/semantic/uncertainty |
| Nav2 projection | `sensor_msgs/PointCloud2` 或 costmap layer input | 只提供 obstacle/free-space 有证据的投影，不把未知地形清空 |
| quadruped terrain query | 项目自定义 service/action（后续定义） | footprint、heading、姿态/步态限制、风险阈值和置信度 |

### 7.2 建议的地图层最小集合

第一版先实现并记录：
- `elevation`：地表高度估计；
- `variance`：高度不确定度；
- `observation_age`：最近一次被接受的原始点云观测年龄；
- `is_valid` / `observability`：该 cell 是否存在有效几何与视线观测；
- `normal_x/y/z`、`slope`；
- `roughness`；
- `step_up`、`step_down` 或等价局部高度差；
- `traversability_geometry`；
- `traversability_robot`（机器人特定模型输出）；
- `risk` 与 `unknown`。

层名称和公式必须后续统一，不可直接照搬示例阈值。未知、遮挡、动态污染和观测陈旧应单独表达，不得把 NaN 通过 inpainting 后的值当作传感器直接观测。

## 8. 四足可通行性算法分层

### A. 几何层（与机器人弱耦合）
- 地面/障碍候选、局部平面与表面法向；
- 坡度、粗糙度、局部高度方差；
- 上台阶/下台阶、沟槽/落差、悬崖/负障碍候选；
- 地形边界与观测置信度。

### B. 机器人能力层（与本体强耦合）
- 最大安全 pitch/roll、机身 clearance、腿长/工作空间；
- 步态与速度模式、横移/倒退/原地转向能力；
- 单步可跨高度、可跨沟宽度、落足区域尺寸；
- 支撑多边形、接触置信度、足端滑移/冲击、摩擦需求；
- 速度/方向相关风险（同一坡面上坡、下坡、横坡风险不对称）。

### C. 学习/经验层（后续研究）
- 用真实执行反馈估计滑移、接触力、能耗、冲击、失败概率；
- 预测值必须输出 epistemic/aleatoric uncertainty；
- 学习模型只作为有界风险证据，不得覆盖硬安全限制；
- 训练/评测按机器人型号、步态、地形和环境域分离。

### D. 规划交接
- 全局路线使用低频、较大范围地形风险/方向代价；
- 局部 planner 使用新鲜局部高程和动态障碍；
- 足端/步态规划器负责最终 foothold feasibility；
- Nav2 的 2D costmap 作为一份保守投影，不承担 foothold planner 的职责。

## 9. 本项目下一步决策与验证 Gate

1. [ ] 固定 MID-360 Driver 的实际 ROS2 Jazzy commit/tag、SDK2 依赖、license/SBOM、ARM64 build 与网络/时钟配置。
2. [ ] 定义 CustomMsg → deskewed PointCloud2 adapter 的时间语义、参考时刻、frame、字段、QoS 和错误行为。
3. [ ] 选择 LIO 基线并证明其消费逐点时间 + IMU、输出 deskew cloud 与 odometry/TF；比较 Point-LIO/FAST-LIO2 等候选。
4. [ ] 在 Jetson AGX Orin 上验证 elevation_mapping_cupy v2.2.x：编译、GPU 兼容、内存池、热稳定、持续 FPS、端到端延迟和观测龄期。
5. [ ] 对照 grid_map Jazzy 2.2.2，确认 GridMap ROS2 message ABI、costmap conversion 包、filters plugins 和依赖版本。
6. [ ] 将传统 traversability_estimation 的 slope/roughness/step/normal/footprint 作为算法基线，先用 bag 离线复现，再实现项目自己的 ROS2 Jazzy 适配。
7. [ ] 设计负障碍/悬崖与 unknown-space 专项测试，不允许以普通 NaN inpainting 替代风险识别。
8. [ ] 双目路径后续加入：硬件同步、内外参、rectification、depth confidence、遮挡与动态物体处理；未完成前不将双目深度用于安全清障。

## 10. 证据来源

- Livox ROS Driver 2 README（固定 commit）：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/README.md
- Livox MID360 ROS2 CustomMsg launch（固定 commit）：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/launch_ROS2/msg_MID360_launch.py
- Livox driver LDDC source（固定 commit）：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp
- elevation_mapping_cupy ROS2 branch source：https://github.com/leggedrobotics/elevation_mapping_cupy/tree/d9e1c9fe42ea3d463519333a443da99f680adde8
- elevation_mapping_cupy Jazzy v2.2.0 release：https://github.com/leggedrobotics/elevation_mapping_cupy/releases/tag/v2.2.0
- elevation_mapping_cupy Orin + MID-360 FPS issue：https://github.com/leggedrobotics/elevation_mapping_cupy/issues/137
- grid_map Jazzy branch：https://github.com/ANYbotics/grid_map/tree/d9614992e36c66d3fe67d9125ca105a0a47676fe
- legacy elevation_mapping maintenance status：https://github.com/ANYbotics/elevation_mapping
- traversability_estimation ROS1 baseline：https://github.com/leggedrobotics/traversability_estimation/tree/14d24c059e1c43466aadf328280adf6394d78039
- Quadruped soft uneven terrain state estimation/traversability study (2023): https://doi.org/10.1002/rob.22175
- Elevation Mapping for Locomotion and Navigation using GPU (IROS 2022): https://arxiv.org/abs/2204.12876
- MEM: Multi-Modal Elevation Mapping for Robotics and Learning (IROS 2023): https://arxiv.org/abs/2309.16818

## 11. 证据边界

本轮是上游固定版本源码/README/issue 与论文资料审计。没有在本项目 ROS 2 工作区编译这些依赖；没有在 Jetson AGX Orin 上执行；没有接入 MID-360 实物；没有完成 deskew adapter、双目同步、LIO、elevation map、traversability node 或 Nav2 cost projection。所有参数示例仅用于解释上游设计，不作为本项目实机参数。尤其 elevation_mapping_cupy 在 RTX 4090 上的性能结果不能外推为 Orin 目标机性能。
