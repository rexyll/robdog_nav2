# MID-360 时间戳合同与 Point-LIO ROS 2 集成依赖审计

- 分支：`nav`
- 日期：2026-11-25
- 证据：固定上游源码 E2；上游 README / 包元数据 E1/E2
- 本文不代表本项目已编译、录包、回放或在 Orin/实机运行

## 1. 本轮结论

1. **Livox Driver 2 固定版本中，CustomMsg 的 header.stamp 与 timebase 在生成函数中来自同一个 pkg.base_time。** ROS 2 路径用 `rclcpp::Time(timestamp)` 写入 header.stamp，`timebase=timestamp`；每点 `offset_time = point.offset_time - pkg.base_time`，类型 uint32，单位为纳秒（由驱动内部 point timestamp 与 timebase 的纳秒时基和官方字段定义共同确定）。因此，对该固定 Driver 版本，消费端用 `header.stamp + offset_time` 在语义上可重建每点时间；Point-LIO 不显式读取 timebase 本身不是错误。
2. Driver 2 的 PointXYZRTLT PointCloud2 中 `timestamp` 是每点绝对时间（float64）；CustomMsg 则为首点 timebase + uint32 offset_time。二者不能在没有显式转换的情况下互换。
3. **Point-LIO CTU fork 的 IMU time lag 实现确实存在消费链不一致。** callback 计算 `msg.header.stamp - time_lag_imu_to_lidar`，该修正值用于 last_timestamp_imu 的回退/同步门控；但实际压入 imu_deque 的仍是原始 msg_in，sync_packages 和 IMU 处理再读取原始 header.stamp。结果是 gate 使用偏移时间，积分样本仍使用未偏移时间。除非该参数保持 0，否则这是应修复/验证的问题，不能视为完整时间补偿。
4. Driver README 的 xfer_format：0=PointXYZRTLT PointCloud2，1=Livox CustomMsg，2=标准 PCL PointXYZI。Point-LIO 订阅 CustomMsg；MRS core 现有旧 launch 也显式传 `xfer_format:=1`。因此本项目驱动配置必须明确设置 1，不能依赖 Driver README 的默认示例 0。
5. **MRS Point-LIO Core 的 ros2 分支是 ROS 2 metapackage，但其 gitman 清单将 Point-LIO、Livox Driver 2、Estimator Plugin 都指向浮动的 `ros2` branch，而非固定 commit。** 不能作为可复现锁定的 BOM，需将依赖 SHA 固定。
6. MRS core 包自身是 ament_cmake，package.xml 依赖 point_lio 与 mrs_point_lio_estimator_plugin；Estimator Plugin 的 package.xml/CMake 还依赖 `mrs_uav_managers`、`mrs_uav_state_estimators`、`mrs_lib`，插件以 MRS UAV estimator pluginlib 接口注册。Point-LIO CMake 另依赖 mrs_lib、backward_ros、PCL、pcl_ros 等。MRS core 主要是集成/launch/config metapackage，不是可直接脱离 MRS 的最小化版本。
7. MRS core 仓库 README 当前简短说明其为 MRS UAV Point-LIO metapackage；较早的网页缓存/文档存在 ROS1 catkin 安装说明，不能把旧说明当成当前 ros2 分支证据。其 ros2 branch 有 ROS build workflow，但 workflow 委托外部 `ctu-mrs/ci_scripts@ros2`；必须进一步审查 reusable workflow 实际 distro/matrix 与结果，不能只凭存在 workflow 就宣称 Jazzy/ARM64 通过。
8. 该 metapackage 有独立 BSD-3-Clause 许可，不会自动覆盖其 submodules 的许可证。Point-LIO 本体 GPL-2.0 与 package.xml BSD 声明冲突仍然存在；必须分开审查各 submodule 的 LICENSE。

## 2. 固定上游版本

| 项目 | 固定版本 |
|---|---|
| Livox ROS Driver 2 | `21445540f0d100dc86a7e6df312dd70bbdb4afdf` |
| Driver 2 lddc.cpp blob | `06195df4fb616416efa980ac16997caf8daa7191` |
| CTU Point-LIO | `e7ff034328b263ab92b06252d91acba1dd969b6d` |
| MRS Point-LIO Core | `ef9de2a55b43e492aec6e20bcab113973b9390a2` |
| MRS core .gitman.yml blob | `6e275ed4d2f4b9489b364b575d45166b6afa281e` |
| MRS core package.xml blob | `c1167cb563cbd4bbe6f6b56e707172b9691796fb` |
| MRS core workflow blob | `d9ad702e022a028e70dc21a216f19d86deffc280` |

上游入口：
- Driver 2 lddc.cpp：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp
- Driver 2 README：https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/README.md
- Point-LIO callback/sync：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/src/laser_mapping.cpp
- Point-LIO preprocessing：https://github.com/ctu-mrs/point-lio/blob/e7ff034328b263ab92b06252d91acba1dd969b6d/src/preprocess.cpp
- MRS core tree：https://github.com/ctu-mrs/mrs_point_lio_core/tree/ef9de2a55b43e492aec6e20bcab113973b9390a2

## 3. Driver 2 时间戳逐行合同

### 3.1 CustomMsg

在 `Lddc::InitCustomMsg()`：
- 若点包非空，`timestamp = pkg.base_time`。
- `livox_msg.timebase = timestamp`。
- ROS2 下 `livox_msg.header.stamp = rclcpp::Time(timestamp)`。
- 所以该固定版本中 `header.stamp` 与 `timebase` 是同一个源值（ROS time 表示与 uint64 纳秒表示）。

在 `Lddc::FillPointsToCustomMsg()`：
- `point.offset_time = points[i].offset_time - pkg.base_time`。
- CustomMsg 字段为 uint32，官方定义是 relative to base time。
- 因此每点时间应按 `t_point_ns = timebase_ns + offset_time_ns` 重建。
- 禁止把 offset_time 误当微秒或毫秒；转换为秒应除以 1e9。

### 3.2 PointXYZRTLT PointCloud2

在 `InitPointcloud2Msg()`：
- header.stamp 同样来自 pkg.base_time。
- 每点字段 `timestamp` 被写入 `static_cast<double>(pkg.points[i].offset_time)`。
- 该 timestamp 字段在当前实现是每点绝对时间值，不是 CustomMsg 的相对 offset。
- 需注意 PointField 声明为 FLOAT64，但字段名/单位语义应以驱动实现和消息定义为准，adapter 不应只看字段名就擅自再次加 header.stamp。

### 3.3 IMU

`InitImuMsg()` 将 `imu_data.time_stamp` 直接转换为 ROS2 header.stamp；因此 LiDAR 与 IMU 在驱动层都使用设备时间戳来源。仍需通过实际 bag 验证同一 MID-360 数据流中二者时间域、单调性、同步偏差、重启/回绕行为。

## 4. Point-LIO 时间同步缺陷定位

固定文件：`src/laser_mapping.cpp`

`callbackIMU()`：
- 计算 `timestamp = rclcpp::Time(msg_in->header.stamp).seconds() - time_lag_imu_to_lidar`。
- 使用修正 timestamp 与 `last_timestamp_imu` 比较，以检测回退。
- 但实际执行 `imu_deque.emplace_back(msg_in)`，存入原始未修改消息。
- `last_timestamp_imu = timestamp`。

`sync_packages()`：
- 使用 `last_timestamp_imu < lidar_end_time` 作为 scan 是否已有足够 IMU 覆盖的 gate。
- 但构建 `meas.imu` 时，从队列消息的原始 `header.stamp` 读取 imu_time。
- 后续 IMU 积分同样消费这些原始消息。

因此 lag 参数目前影响 timestamp gate 和回退检查，但不影响实际 IMU sample timestamps；这会让“数据够不够”的判断与“积分使用什么时间”采用不同时间轴。必须修复为统一时间语义（例如复制消息并改 stamp 后入队，或在 buffer 内保存显式修正时间戳+原始样本），并增加正/负 lag 单测。不能在没有上游确认前自行向上游提交修复。

另需审计：
- last_timestamp_imu 的初始化/重置；
- IMU 队列空、时间回退、设备重连、bag loop；
- scan end 需要的 IMU 插值/外推边界；
- ROS use_sim_time 与设备硬件时间的关系；
- 同步参数是否在实际配置文件中默认 0。

## 5. 驱动 xfer_format 与项目适配

Driver 2 README 定义：
- 0：Livox PointCloud2 PointXYZRTLT（逐点 timestamp）
- 1：Livox CustomMsg（timebase + offset_time）
- 2：标准 PointXYZI（不适合需要逐点时间的 LIO 路径；ROS2 代码中 PCL 发布分支还存在不支持提示）

CTU Point-LIO preprocessing 订阅 CustomMsg，因此本项目若使用该实现，应固定 `xfer_format=1`。若另建 adapter 消费 PointXYZRTLT，则必须明确解析 FLOAT64 timestamp 的单位和绝对/相对语义，不得把两种格式混成同一时间合同。

MRS core 的旧 MID360 launch 明确设置 `xfer_format:=1`，这与 Point-LIO CustomMsg 输入相符；但该 launch 文件位于 `launch/old`，不能据此宣称当前 Jazzy launch 已经验证。

## 6. MRS Point-LIO Core 集成边界

固定 MRS core `ros2` branch commit：
- README 将其定义为 MRS UAV Point-LIO metapackage。
- package.xml 依赖 `point_lio` 与 `mrs_point_lio_estimator_plugin`，使用 ament_cmake。
- .gitman.yml 以浮动分支 `ros2` 引入三个 repo：`mrs_point_lio_estimator_plugin`、`ctu-mrs/point-lio`、`ctu-mrs/livox_ros_driver2`。
- 这意味着同一个 core commit 在未来 gitman update 后可能解析到不同的 submodule commits；不能视为可复现软件 BOM。
- Point-LIO CMake 的依赖包含 `mrs_lib`、`backward_ros`、`livox_ros_driver2`、PCL/pcl_ros 等；本项目若不引入整个 MRS UAV System，需先评估替换/剥离 MRS 专属 timer、diagnostics、estimator plugin 和 launch integration 的工作量。
- MRS core package 的 BSD-3-Clause 仅描述该 metapackage，不覆盖它下载的 submodules。

### 6.1 CI 证据与边界

MRS core workflow 委托 `ctu-mrs/ci_scripts/.github/workflows/ros_build_test.yml@ros2`，其下游 workflow 明确分为 `build_amd64` 与 `build_arm64`，均使用 `ctumrs/ros_jazzy:latest`。已核实的 Actions runs：

| 仓库 | Commit | Run | AMD64 | ARM64 |
|---|---|---:|---|---|
| Point-LIO | `e7ff034328b263ab92b06252d91acba1dd969b6d` | [24843451862](https://github.com/ctu-mrs/point-lio/actions/runs/24843451862) | success | success |
| Estimator Plugin | `b43af57300b43a29ab6bfa1c68a9442fb869cea1` | [21216131865](https://github.com/ctu-mrs/mrs_point_lio_estimator_plugin/actions/runs/21216131865) | success | success |
| MRS core | `0f3e329bd33aeb25d7da67c3381f0ec645de5769` | [33400579110](https://github.com/ctu-mrs/mrs_point_lio_core/actions/runs/33400579110) | success | success |

说明：Point-LIO 与 Plugin run 的 SHA 与本审计锁定 SHA 相同；MRS core run 成功的是较早提交 `0f3e329...`，不是当前审计分支头 `ef9de2a...`。因此当前已存在 Jazzy AMD64/ARM64 CI 成功证据，但仍需验证当前 core HEAD 的完整递归依赖树和最新 CI。ARM64 runner 是 GitHub `ubuntu-22.04-arm` 上通过 Jazzy 容器构建，并非 Jetson AGX Orin 硬件；不能替代 Orin GPU/内存/热/长时 soak test。

## 7. 本项目建议的时间戳标准

所有 LIO adapter 统一定义：
- `clock_domain`: sensor hardware / ROS system / ROS simulated，明确唯一来源与转换；
- `scan_start_stamp`: 首点绝对时间；
- `point_time_offset_ns`: 相对 scan_start 的 uint32/uint64 纳秒偏移；
- `point_stamp_ns`: 可选预计算绝对时间；
- `scan_reference_stamp`: 明确是 scan start / midpoint / end，不能模糊；
- `imu_stamp_ns`: 与点云在同一可比较时间域；
- `time_offset_lidar_imu_ns`: 明确符号约定（例如 corrected_imu_stamp = raw_imu_stamp + offset）；
- `timestamp_quality`: monotonicity、range、missing、fallback、clock-reset flags。

验收规则：
1. Driver 单帧 `header.stamp == timebase`（按纳秒比较）。
2. 每点 `point_stamp = timebase + offset_time`，偏移不溢出且非负；点时间范围符合设备/发布周期。
3. 同一 bag 的 IMU 与 LiDAR 时间单调，测得 offset 有统计报告；不得只看启动时静态配置。
4. 时间偏移应用于所有同步、积分、插值、状态传播与输出 stamp，不允许仅作用于 gate。
5. ROS clock 跳变、bag loop、驱动重连、丢帧均触发可观测的 reset/recovery。
6. cloud output 明确 reference stamp 和 frame，elevation mapping 使用同一时刻 TF；不得混用绝对时间和相对 offset。
7. 每种输入格式（CustomMsg / PointXYZRTLT）独立有 parser 单测与 round-trip test。

## 8. Gate 与下一步

- [ ] 获取 MID-360 实际 CustomMsg + IMU rosbag，检查 header.stamp/timebase、point offset、IMU 时间差与时钟单调性。
- [ ] 为 Point-LIO callback/sync 写最小单测，证明当前 lag 参数未贯穿 sample consumption；提出不改变算法主体的时间戳统一修复。
- [ ] 固定 MRS core、Estimator Plugin、Point-LIO、Livox Driver 2、ikd-Tree、IKFoM 等全部依赖 commit，生成 SBOM/lock manifest。
- [x] 追踪 MRS reusable CI workflow：确认 Jazzy 容器、AMD64/ARM64 jobs；Point-LIO 与 Estimator Plugin 固定审计 commit 的两架构 CI 成功，MRS core 较早 commit 的两架构 CI 成功。仍需验证当前 core HEAD 的 CI。
- [ ] 审计 estimator plugin 是否为四足输出需求所必需，评估去除 MRS UAV 专属接口后的最小 ROS2 Point-LIO package。
- [ ] 对比直接依赖 CTU fork、从算法层独立移植、采用其他 Jazzy LIO 的工程与许可证成本。
- [ ] 在 Jazzy x86_64 与 Jetson Orin ARM64 分别构建；再做 MID360+IMU bag replay 与 soak test。

## 9. 证据边界

固定源码仅能证明该版本的实现行为。尚未取得本项目真实 MID-360/IMU rosbag，未检查真实硬件时间域、偏差和丢包；已核实上游 Actions 在 Jazzy 容器中的 AMD64/ARM64 CI 成功（见 §6.1），但本项目未自行构建；MRS core 当前 HEAD 的完整依赖树仍需 CI 验证；未在 Jetson Orin 硬件验证；未完成 GPL/BSD 许可证冲突澄清。上游文档/源码审计不等于项目验证。


## 10. 原始 UDP 包、CustomMsg 与 rosbag 接收时间的证据边界

### 10.1 CustomMsg 有意保留的字段与丢失的协议字段

固定 Livox ROS Driver 2 commit `21445540f0d100dc86a7e6df312dd70bbdb4afdf` 的消息定义：
- `CustomMsg`: Header、timebase、point_num、lidar_id、reserved、points；
- `CustomPoint`: offset_time、XYZ、reflectivity、tag、line。

因此 ROS CustomMsg 没有直接保留 MID-360 原始 UDP 包头中的 `time_type`、`time_interval`、`udp_cnt`、`frame_cnt`、包级 CRC 字段。虽然可从每点 offset_time 检查相对时间顺序和重建点时间，但不能仅凭 CustomMsg 反向证明原始 UDP 包的 time_type=1、包 CRC 正确、UDP 序号连续或设备当时 PTP 锁定。CustomMsg 是驱动解码/组帧后的表示，不是原始网络报文的无损归档。

### 10.2 必须保留的双轨证据

同一采集 run 至少保留：
1. ROS bag：`/livox/lidar` CustomMsg、`/livox/imu`、驱动/设备诊断 topic、`/tf` 与 `/clock`（如存在）；记录 topic type、QoS、storage_id、rosbag2/ROS 版本和 recorder 参数。
2. 原始 LiDAR UDP pcap：保留完整 UDP payload，不仅保存过滤后的 PTP 包；记录 capture NIC、capture timestamp source、过滤表达式、snaplen、drop counter 和 SHA-256。
3. PTP pcap 与 ptp4l/pmc/phc2sys 日志；和第 18 节同一 run_id。
4. 设备诊断快照：time_sync_type、last_sync_time、time_offset、local_time_now、powerup_cnt、HMS 状态；同时保存原始响应和解析器版本。
5. 采集配置：驱动固定 commit、xfer_format=1、MID-360 固件/配置、网络拓扑、recorder 是否启用 use_sim_time、/clock 来源、主机 clock discipline 状态。

不得从点云 ROS topic 单独推断原始 UDP 的 time_type；也不得把诊断查询时刻的状态回填成整个 bag 录制期间持续锁定。

### 10.3 三种时间戳独立统计

现有 `audit_livox_rosbag_timing.py` 读取 `SequentialReader.read_next()` 的第三个返回值时，原先直接丢弃了 bag receive timestamp。本次审计工具扩展为同时统计：
- LiDAR CustomMsg.timebase / header.stamp；
- IMU header.stamp；
- rosbag2 reader 提供的 message receive timestamp；
- 每条消息 receive timestamp 减 sensor timestamp 的描述性分布和单调性。

该差值仅是 paired timestamp difference。除非已证明两者处于同一 clock domain，或具备有界误差的 PHC/system/ROS clock mapping，否则它不是网络传输延迟、驱动排队延迟或传感器时钟偏差。录制器使用 `use_sim_time` 时，rosbag2 时间戳可能来自最新收到的 `/clock`；因此必须保留 recorder 参数与 /clock 数据，不能只看 bag metadata 推断时钟域。Jazzy rosbag2 支持 sent/received timestamp 的能力与存储格式有关；当前工具只审计 reader 暴露的 receive timestamp，不声称恢复 sent timestamp。

### 10.4 离线审计工具当前边界

工具可检查 CustomMsg/header/timebase/point offset/IMU 与 bag receive timestamp 的单调性、统计分布和配对差值；不能从 CustomMsg恢复原始 UDP header、time_type、CRC 或 UDP packet counter。下一阶段若增加 pcap decoder，必须以 MID-360 protocol 固定版本实现字段偏移、字节序、长度、CRC-32、time_type 和 data_type 校验，并以官方样本/捕获包做 golden vectors；在拿到真实 pcap 前只能完成纯算法测试，不能宣称设备协议验证。

参考：
- Livox MID-360 UDP packet layout and timestamp semantics: https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
- Fixed Driver 2 CustomMsg: https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/msg/CustomMsg.msg
- Fixed Driver 2 CustomPoint: https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/msg/CustomPoint.msg
- rosbag2 Jazzy sent/received timestamp feature summary: https://roscon.ros.org/2024/talks/Jazzy_Jalisco_rosbag2_updates_and_new_features.pdf

本节与脚本改动不代表取得项目 pcap 或实际 rosbag；未执行目标设备录制、未在 ROS 2 Jazzy 环境读取真实 bag。
