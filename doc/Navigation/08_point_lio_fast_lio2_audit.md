# Point-LIO 与 FAST-LIO2：MID-360 LIO / 时间补偿 / ROS 2 适配审计

- 专项分支：`nav`
- 审计日期：2026-11-25
- 证据类型：固定上游源码 E2、论文/README E1
- 本文不代表在项目工作区构建、数据集回放、Jetson 实测或四足实机验证
- 研究对象：
  - `ctu-mrs/point-lio @ e7ff034328b263ab92b06252d91acba1dd969b6d`（ROS 2 fork 的 `ros2` 分支）
  - `hku-mars/FAST_LIO @ 7cc4175de6f8ba2edf34bab02a42195b141027e9`（官方主仓库 main commit；ROS 1 实现）
  - Point-LIO 原始论文/官方仓库与 FAST-LIO2 论文

## 1. 本轮结论摘要

1. **Point-LIO 的核心机制是逐点状态估计。** CTU ROS 2 fork 将 Livox CustomMsg 的 `offset_time` 转成每点毫秒偏移；在 `use_imu_as_input=false` 路径中，按点时间推进/更新状态，再用该时刻状态把点变换到 world。它不是 FAST-LIO2 那种先整帧 IMU 前向传播、再反向逐点 deskew 的相同流程。
2. **FAST-LIO2 官方实现有显式整帧运动去畸变。** `UndistortPcl` 将 IMU 轨迹积分到扫描末端，并从扫描末端反向补偿各点；它需要每点时间和覆盖扫描时间区间的 IMU 数据。
3. 两者都依赖 LiDAR 与 IMU 的可靠时间关联。Point-LIO README 明确把 IMU/LiDAR 同步列为关键要求；FAST-LIO README 也强调同步和逐点 time 字段，缺失会影响前向/后向传播。
4. **Point-LIO CTU ROS2 fork 并非无依赖的通用 ROS2 包。** 它依赖 `mrs_lib`、`backward_ros`、Livox ROS Driver 2 和 MRS 风格组件；README 建议通过 `mrs-ctu/mrs_point_lio_core` 安装。其仓库 workflow 调用外部 reusable workflow，当前仓库本身未证明 ROS 2 Jazzy 构建通过。
5. **FAST-LIO2 官方仓库不是 ROS 2/Jazzy 包。** 固定版本 CMake 仍使用 catkin、roscpp、ROS1 message 类型与 Livox ROS Driver（ROS1）；其 MID360 yaml 是 ROS1 topic/驱动接口。它只能作为算法基线，若要用在 Jazzy，必须采用经过审计的 ROS2 移植或独立移植，不能直接把官方仓库列为 Jazzy 可用依赖。
6. **许可是阻断性审查项。** FAST_LIO 仓库 LICENSE 是 GPL-2.0；Point-LIO CTU fork LICENSE 同样是 GPL-2.0，但 package.xml 的 `<license>BSD</license>` 与 LICENSE 文件冲突，仓库元数据亦未明确识别 SPDX。必须先做上游许可澄清与法律审查，再决定是否纳入产品依赖或进行代码移植。
7. 当前建议：Point-LIO ROS2 fork 进入“优先验证候选”，FAST-LIO2 作为算法/基准对照；两者都不冻结为最终依赖。必须先过 license、Jazzy/ARM64 build、MID-360 + 实际 IMU bag replay、时间语义与输出 frame contract。

## 2. 固定源码与许可证据

| 项目 | 固定版本 | ROS 接口状态 | 许可审计 |
|---|---|---|---|
| CTU Point-LIO ROS2 fork | `e7ff034328b263ab92b06252d91acba1dd969b6d` | ament_cmake + rclcpp；依赖 mrs_lib/backward_ros；CI 外部 workflow | LICENSE 是 GPL-2.0；package.xml 写 BSD，冲突待澄清 |
| HKU FAST_LIO | `7cc4175de6f8ba2edf34bab02a42195b141027e9` | catkin + ROS1；非 ROS2/Jazzy | LICENSE 是 GPL-2.0 |
| Point-LIO 原始项目 | 官方仓库默认分支在审计时为 point-lio-with-grid-map | 以原始研究实现为主 | CTU fork 明确称沿用原项目许可；以随附 GPL-2.0 文件为审查对象 |

固定源码入口：
- Point-LIO ROS2 README：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/README.md
- Point-LIO ROS2 MID360 preset：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/config/presets/mid360.yaml
- Point-LIO ROS2 preprocessing：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/src/preprocess.cpp
- Point-LIO ROS2 estimator loop：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/src/laser_mapping.cpp
- Point-LIO ROS2 IMU process：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/include/imu_processing.hpp
- Point-LIO ROS2 package manifest：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/package.xml
- FAST-LIO2 README：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/README.md
- FAST-LIO2 MID360 config：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/config/mid360.yaml
- FAST-LIO2 IMU undistortion：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/IMU_Processing.hpp
- FAST-LIO2 preprocessing：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/preprocess.cpp
- FAST-LIO2 CMake：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/CMakeLists.txt
- FAST-LIO2 LICENSE：https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/LICENSE

## 3. Point-LIO ROS2 fork 源码链

### 3.1 CustomMsg 预处理与点时间

固定文件：`src/preprocess.cpp`，函数 `Preprocess::avia_handler()`

- 直接订阅 `livox_ros_driver2::msg::CustomMsg`。
- 每个点的 `offset_time` 以纳秒为源单位，源码除以 `1,000,000` 存入 PCL `curvature`，并注释该字段单位为 ms。
- 过滤 Livox tag/line，并可按 `point_filter_num` 抽点。
- 源码循环从 `i=1` 开始；这会跳过每帧第 0 个点，是需要评估的实现细节，不应直接视为算法设计要求。
- CustomMsg 的 `header.stamp` 作为 scan begin/base time；后续将 `curvature/1000` 秒加到 scan base time，得到每点估计时间。

### 3.2 IMU/LiDAR 同步

固定文件：`src/laser_mapping.cpp`

- LiDAR callback 记录 header stamp、将点云送入队列。
- IMU callback 记录 IMU header stamp，并可应用 `time_lag_imu_to_lidar`。
- `sync_packages()` 通过最大点时间推导 `lidar_end_time`，只有 IMU buffer 已覆盖到 scan end 才将该 scan 与 IMU 样本交给估计器。
- 该逻辑依赖时钟域一致、header stamp 语义正确、点 offset_time 单位正确；队列积压、时间回退、IMU 缺样和驱动时间戳跳变需要显式测试。
- 当前实现对 IMU callback 的 timestamp 检查使用修正后的 timestamp，但入队的仍是原始消息；消费端也用消息 header stamp。需要特别核对 `time_lag_imu_to_lidar` 是否真正贯穿整个同步计算，避免仅用于回退检查而未实际修正消息时间。建议把它列为代码复核/单测项，不直接假设已生效。

### 3.3 逐点估计和点云输出语义

在 `timerMain()`：
- `p_imu->Process()` 在当前 ROS2 fork 的 IMU helper 中只复制测量点云，并没有像 FAST-LIO2 那样显式反向整帧 deskew。
- 主估计循环按 `time_seq` 中的点时间分组推进滤波器；在 `use_imu_as_input=false` 的 point-by-point 路径，每个时间组执行状态预测/迭代更新，再使用该时间点状态将组内点变换到 world。
- 因而核心思想是“在各点/时间组的观测时刻估计状态并将点投到 world”，不是“先把所有点刚性补偿到扫描末端 body frame”。
- `publish_frame_world()` 发布 `feats_down_world`，header stamp 为 `lidar_end_time`，frame 为 `init_frame`；点坐标已按其各自估计时刻变换到 world，但发布时仅写 XYZ/intensity，逐点 curvature/time 不再保留。
- `publish_frame_body()` 使用 `feats_undistort` 并只做 LiDAR→IMU 外参变换；由于该 fork 的 `ImuProcess::Process()` 仅复制原始点云，这个 body topic 不能被误称为已经 deskew 到同一时刻的点云。
- Odometry 通过 `nav_msgs/Odometry` 发布；可选 TF 广播由 `_publish_fcu_tf_` 控制。必须审计具体配置的 `init_frame`、`odom_frame`、`lidar_frame`，不能仅凭 topic 类型推断 TF 树方向。

**对高程地图的影响：** Point-LIO world cloud 可以作为地图融合候选，但需要确认 world frame 与 elevation map 的 map frame 一致，header stamp 代表 scan end，且逐点时间/质量字段丢失是否可接受。若下游必须使用 scan-end body cloud，则需要另建基于 scan-end pose 的坐标转换输出，不能把 `publish_frame_body` 直接当作 deskewed cloud。

### 3.4 ROS2/Jazzy 工程适配

- `CMakeLists.txt` 使用 `ament_cmake`，属于 ROS2。
- 依赖 `rclcpp`、`rclcpp_components`、`livox_ros_driver2`、PCL、`mrs_lib`、`backward_ros` 等。
- README 推荐从 `mrs-ctu/mrs_point_lio_core` 安装，说明这不是完全独立的通用 ROS2 package。
- 仓库 workflow 仅调用 `ctu-mrs/ci_scripts` 的 reusable workflows；当前仓库 workflow 文件未直接声明 Jazzy distro、Ubuntu 24.04 或 ARM64 matrix。因此“ROS2 fork”不等于“ROS 2 Jazzy 已验证”。
- MID360 preset 有 `lidar_type: 1` 和外参示例；外参示例不能直接用于实际装配，必须现场标定/测量并冻结坐标定义。

## 4. FAST-LIO2 官方实现源码链

### 4.1 原始 MID360 输入

固定文件：`config/mid360.yaml`
- 默认话题 `/livox/lidar` 与 `/livox/imu`。
- `time_sync_en: false`，`time_offset_lidar_to_imu: 0.0`。
- `lidar_type: 1` 对应 Livox serials。
- 外参示例 `extrinsic_T/R`。
- 这些是官方 ROS1 示例配置，不是本项目实机标定值。

### 4.2 逐点时间与去畸变

固定文件：`src/preprocess.cpp` 与 `src/IMU_Processing.hpp`

- Livox CustomMsg 的 `offset_time` 被转换为毫秒并放入 `curvature`。
- `UndistortPcl()` 将点按 offset time 排序。
- 使用扫描区间内 IMU 序列做前向状态传播并保存 IMU poses。
- 扫描结束时将状态预测到 scan end。
- 再从末端反向遍历 IMU pose 区间，按点时间计算相对位姿，将每个点补偿到 scan-end LiDAR/IMU frame。
- 该函数代码中有一条作者注释指出当前补偿表达式“not accurate”，应作为数值精度/实现复核项；不能只凭“存在 deskew”就认定所有运动和外参项都无误。
- 缺失点时间会导致 README 所述 `Failed to find match for field 'time'` 问题，前向/反向传播依赖逐点时间。

### 4.3 ROS1 与移植边界

- 官方固定 commit 的 CMake 使用 `find_package(catkin)`，依赖 `roscpp`、`rospy`、ROS1 `sensor_msgs`、`tf`、`livox_ros_driver`。
- package.xml 使用 catkin，且 source 使用 ROS1 include 与 API。
- 因此该官方仓库不能直接 colcon build 于 ROS 2 Jazzy。
- 社区 ROS2 port 需要逐一审计：代码是否同步上游算法、是否保留 MID360 timestamp、是否保留 license、是否有 Jazzy/ARM64 CI 和 bag 测试；不可仅凭仓库名带 ROS2 认定可靠。

## 5. 算法与系统层比较

| 维度 | Point-LIO（CTU ROS2 fork） | FAST-LIO2（HKU 官方） |
|---|---|---|
| 状态估计粒度 | 逐点/时间组更新，Point-LIO 特征 | 整帧 scan 与 IMU 传播/滤波 |
| 运动处理方式 | 按点时间估计状态并投到 world；当前 fork helper 不做显式 scan-end deskew | IMU 前向传播 + 反向逐点 deskew 到 scan end |
| 点时间字段 | CustomMsg offset_time → curvature(ms) | CustomMsg offset_time → curvature(ms) |
| IMU 同步 | scan end 等待 IMU 覆盖；时钟/lag 细节待验证 | scan interval IMU 序列与每点时间共同驱动补偿 |
| 地图结构 | ikd-Tree | ikd-Tree |
| 输出云语义 | world-frame per-point transformed cloud；body topic 不是 deskewed | scan-end frame undistorted cloud / map cloud |
| ROS2 Jazzy | ROS2 port 存在，但 MRS 依赖和 Jazzy CI 未证明 | 官方仓库 ROS1；需移植 |
| License | GPL-2.0 文件与 package.xml BSD 声明冲突 | GPL-2.0 |
| 当前项目建议 | 优先进入 ROS2/Orin 验证候选 | 算法基准与对照候选，不直接作为 Jazzy 依赖 |

FAST-LIO2 论文报告了直接 raw-point-to-map 注册和增量 ikd-Tree；这些论文性能数字不是 Jetson Orin 实测承诺。论文参考：https://doi.org/10.1109/TRO.2022.3141876

Point-LIO 论文/原始方法参考：https://arxiv.org/abs/2209.04951

## 6. 项目接口设计：不能只输出一份 PointCloud2

建议把 LIO 输出合同拆为：

| 接口 | 类型 | 必须携带/明确的语义 |
|---|---|---|
| raw_lidar | Livox CustomMsg | 原始 header、timebase、逐点 offset_time、tag/line/reflectivity |
| imu_raw | sensor_msgs/Imu | 设备/ROS 时间域、frame、单位、饱和与同步健康 |
| state_odom | nav_msgs/Odometry + TF | pose/twist/covariance、child frame、状态时刻 |
| cloud_world | PointCloud2 | 所有点已变换到同一 world/map frame；header stamp 为参考扫描时刻；逐点时间可选保留 |
| cloud_deskewed_body | PointCloud2 | 所有点补偿到明确 scan-end/body frame；header stamp=参考时刻；字段与原始点索引映射可选保留 |
| lio_health | diagnostic_msgs/DiagnosticArray 或项目诊断消息 | IMU/LiDAR 时间差、点时间范围、scan duration、队列深度、丢包、处理延迟、估计器初始化/退化状态 |

**关键设计规则：**
- 不允许把 FAST-LIO2 的 scan-end deskew cloud 与 Point-LIO 的 per-point world cloud 混用同一个 topic 名或 frame contract。
- elevation_mapping 若接收 cloud_world，必须配置同一个 map frame，并明确地图更新 stamp；若接收 cloud_deskewed_body，必须通过 TF 使用 scan-end 时刻的 base pose。
- 任何转换不得把每点 timestamp 单位默默变成秒/毫秒；应通过显式 field name、datatype、单位与回放测试验证。
- LIO 输出的 map/world cloud 不自动等于 Nav2 的 obstacle cloud；还要处理地面、动态目标、可见自由空间和 unknown semantics。

## 7. 对本项目的候选决策（暂定）

1. **优先验证 Point-LIO ROS2 fork**，理由是已经有 rclcpp/ament 与 Livox ROS Driver 2 CustomMsg 接口，并且实现了逐点时间状态估计。
2. 不直接接受 CTU fork 为最终方案：先解决 GPL/BSD 声明冲突、MRS 依赖、Jazzy/Orin 构建、时间 lag 逻辑、点云输出语义和长时间稳定性。
3. **FAST-LIO2 作为算法对照**：它显式 deskew 机制更容易定义 scan-end body cloud contract，但官方仓库是 ROS1 + GPL-2.0，若需 ROS2 应单独评估移植成本和许可证义务。
4. 最终选择以相同 MID360 + 同一 IMU + 同一 bag + 同一外参/时间标定进行 A/B 测试；比较 ATE/RPE、重力方向/roll-pitch 漂移、点云重影、地面高度误差、动态运动鲁棒性、CPU/GPU/RAM、p50/p95 latency、队列积压和长时间掉帧。
5. 四足机器狗会产生高频机身振动与步态周期冲击；需专门采集静止、原地转向、直线走、侧移、斜坡、台阶、足端冲击/机身抖动数据，不能只用无人机/手持数据集结论代替。

## 8. 下一步验证 Gate

- [ ] 对 Point-LIO GPL-2.0 与 package.xml BSD 冲突取得上游书面澄清，形成第三方许可证审查记录。
- [ ] 确认 CTU Point-LIO fork 的完整依赖图、MRS 组件是否可从 MRS core 中剥离、Jazzy/Ubuntu 24.04/ARM64 构建结果。
- [ ] 单测验证 CustomMsg `offset_time` 纳秒→毫秒→秒链路、header stamp 参考时刻、IMU time lag 是否真正作用于消费时间戳。
- [ ] 验证 Point-LIO 的 `cloud_world` 是否每点按各自状态变换；验证 `cloud_body` 不得标记为 deskewed；决定是否增加 scan-end body 输出。
- [ ] FAST-LIO2 官方 ROS1 与候选 ROS2 port 做代码差异审计；明确移植 license 和上游同步策略。
- [ ] 用同一 MID360 + IMU rosbag 离线回放两种算法，建立 GT/RTK/外部真值独立评测；不把 LIO 自身地图当真值。
- [ ] 在 Jetson Orin 上做 1 小时/长时 soak test，记录 CPU/RAM/温度/功耗、频率、延迟、队列、时间差、状态退化。
- [ ] 与 elevation_mapping_cupy 对接时验证 cloud frame/stamp 和 map update latency；比较 cloud_world 与 cloud_deskewed_body 两种输入方案。

## 9. 证据边界

本文固定审计了公开仓库特定 commit 的源码、README、package manifest、CMake 和 LICENSE。未在本项目 ROS 2 Jazzy 工作区编译 Point-LIO；未构建 FAST-LIO2 ROS2 port；未回放任何数据集；未在 Jetson Orin 或真实 MID-360/IMU 上测试；未完成许可法律意见。所有结论限定于所列 commit，不能外推到其他分支/移植版本。
