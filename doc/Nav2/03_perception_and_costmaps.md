# 03 — 感知输入与 Costmap 环境表示

## 1. Costmap 概念

Nav2 Costmap 2D 是规则二维网格，每个 cell 存储代价值，常见语义包括 unknown、free、occupied/lethal 和 inflation costs。Planner 在全局网格搜索路径，Controller 在局部网格评估轨迹/控制动作。Layer plugin 将静态地图和传感器观测合成统一环境表示。它不是原始三维世界模型，也不是四足落足地图。

## 2. Global 与 Local Costmap

| 项目 | Global | Local |
|---|---|---|
| 用途 | 全局路线搜索 | 近场控制/避障 |
| 常见 frame | map | odom |
| 窗口 | 全局/静态范围 | rolling window |
| 输入 | 静态地图、禁行区、长期障碍 | 实时 LiDAR、Stereo、动态障碍 |
| 重点 | 稳定一致 | 低延迟、高更新率 |
| 消费者 | Planner | Controller、部分 Behavior |

参数需由地图尺度、速度、传感器量程、制动距离和算力确定。

## 3. 常见 Layer

- StaticLayer：消费 OccupancyGrid（通常 /map），映射静态地图。
- ObstacleLayer：消费 LaserScan/PointCloud2，执行 obstacle marking 与 raytracing clearing；配置 source、topic、data_type、marking/clearing、min/max obstacle height、range、observation persistence、expected update rate。
- VoxelLayer：维护带高度维度的 voxel 表示，再投影到二维 Costmap。适合需要垂直占据区分的 3D LiDAR/深度数据，但不等于完整三维规划。
- InflationLayer：把障碍物周围扩展成代价梯度，为 footprint 留安全余量。
- Keepout / Speed Filter：表达禁行区或速度区域。
- 自定义 Layer：可将坡度、粗糙度、动态风险、地形可通行性映射为代价，但必须定义输入和标定依据。

## 4. Marking 与 Clearing

Marking 将传感器命中点写入障碍层；Clearing 沿观测射线清除已观测为空闲的区域。只有 marking 可能留下 ghost obstacle；只有 clearing 可能漏标。PointCloud2 的 raytracing、origin、frame 和 range 限制必须验证。不得使用过期点云清除新障碍，timeout 与 persistence 要匹配真实频率和延迟。

## 5. MID-360 处理链

1. Driver 发布原始 CustomMsg 和诊断。
2. 明确 CustomMsg→PointCloud2 转换与字段映射。
3. 按时间模型 deskew；时钟未对齐或运动估计缺失时，不把畸变点云当高可信输入。
4. 过滤机身自体、地面/近地噪声和远距离低置信点；保留细杆/低矮障碍召回测试。
5. 分别发布 Costmap cloud 和 Collision Monitor cloud，或明确复用策略及数据失效语义。
6. 先接 Local Costmap，验证 marking/clearing、footprint、inflation 后，再评估 Global Costmap 实时观测。

## 6. Stereo 处理链

左右相机必须同步、刚性安装、完成内外参标定和 rectification；之后计算 disparity/depth 并生成点云。过滤无效视差、低置信度、距离范围、反光/透明区域和机身自体。Costmap 常见输入是 PointCloud2，而不是未经处理的 RGB 图像。Stereo 宜作为独立 observation source，便于诊断；未经验证不要与 LiDAR 预合并。

## 7. Footprint 与 Inflation

Footprint 应覆盖机身、支架和突出附件；robot_radius 是简化圆形模型。四足步态中支撑范围和摆动腿会变化，静态 footprint 不包含所有动态外形和落足风险。Inflation 需考虑机身宽度、附件、跟踪误差、定位误差、速度、延迟和实测停车距离。

## 8. 动态障碍与四足地形

Obstacle/Voxel layer 通常写入当前观测，不等于持久化多目标跟踪器。动态目标跟踪应由独立检测/跟踪模块产生轨迹、速度和置信度，再通过预测占据、动态 layer 或上层限速策略影响导航。

二维投影会丢失高度、坡度、台阶边缘、沟槽深度、落足面和接触稳定性。复杂地形需 elevation/traversability map、foothold planner 和 gait controller 协作。第一阶段限制在已验证的平整地面/缓坡。

参考：https://docs.nav2.org/jazzy/getting_started/navigation_concepts/environmental_representation/
