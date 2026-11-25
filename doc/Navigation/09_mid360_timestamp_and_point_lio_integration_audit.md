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


## 11. MID-360 原始 UDP payload 严格解析器（协议 v1.4.12）

### 11.1 固定字段与校验合同

本项目新增纯 Python、无第三方依赖的 scripts/livox_mid360_udp_parser.py，解析边界仅为**单个 UDP application payload**，不解析 Ethernet/IP/UDP、PCAP/PCAPNG，也不解码为 ROS PointCloud2。

按 Livox 官方 MID-360 协议 v1.4.12：
- 所有多字节协议字段均为 little-endian；payload 最小头长 36 bytes，version=0，length 等于整个 UDP payload 长度。
- Header：time_interval offset 3 / uint16（单位 100 ns）；dot_num offset 5 / uint16；udp_cnt offset 7 / uint16；frame_cnt offset 9 / uint8；data_type offset 10 / uint8；time_type offset 11 / uint8；reserved offset 12–23；CRC32 offset 24–27；timestamp offset 28–35 / uint64 ns；data 从 offset 36 开始。
- time_type 当前有效枚举为 0（设备上电本地时间）、1（PTP/gPTP 主时钟时间）、2（GPS 时间）；它是时间来源类型，不是自动证明 PTP 已锁定的状态位。
- data_type：0 IMU（每 sample 6×float32，24 bytes）；1 Cartesian32（XYZ int32 + reflectivity/tag，14 bytes）；2 Cartesian16（XYZ int16 + reflectivity/tag，8 bytes）；3 spherical（depth uint32 + theta/phi uint16 + reflectivity/tag，10 bytes）。解析器按 dot_num × sample_size 校验 data 长度，不擅自解释 reserved/tag_type 扩展语义。
- 官方 CRC 表给出 CRC-32：poly 0x04C11DB7、init/xorout 0xFFFFFFFF、输入/输出反转。实现使用 Python zlib.crc32 的 CRC-32/ISO-HDLC 语义；校验覆盖 timestamp + data，即 payload bytes [28:]，报文中的 CRC 字段按 little-endian uint32 读取。

### 11.2 实现与测试边界

parse_mid360_udp_payload() 对版本、长度、点数、data_type、time_type、数据区长度和 CRC 进行严格校验；CRC 不匹配即拒绝，不提供隐式“跳过 CRC”模式。返回值保留原始 data/reserved 和已解析 header 元数据，避免将协议解析与点云坐标转换/ROS 消息适配耦合。

scripts/tests/test_livox_mid360_udp_parser.py 使用**合成报文**覆盖四种 data_type、little-endian 字段、100 ns time_interval 单位、CRC 标准 check value，以及短头、版本、长度、dot_num、data_type、time_type、data 长度和 CRC 损坏等拒绝路径。合成向量只证明 parser 对已编码合同的单元测试行为，不是 Livox 官方 golden capture，也不代表真实设备包已验证。

### 11.3 尚未实现/验证

- 尚未实现 PCAP/PCAPNG 容器读取、UDP 五元组筛选、IP 分片重组、capture timestamp 解释或丢包统计。
- 尚未从真实 MID-360 pcap 验证 CRC 字段行为、所有 data_type、IMU 点数/长度、firmware 兼容性或 time_type 分布。
- 未验证实际网络 capture timestamp 与 sensor timestamp 的时钟域映射。
- 不得把 parser 接受的合成包写成设备验收证据；后续必须使用保留 SHA-256 的原始设备 pcap 建立独立 golden fixtures，并同时记录型号/固件/配置/采集 NIC/snaplen/drop counters。

协议依据：Livox 官方 [MID-360 Communication Protocol v1.4.12](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html)，特别是协议字节序、§2 点云/IMU header、§2.2 timestamp、§2.3 data types 与 §6 CRC Algorithm。


## 12. PCAP / PCAPNG 容器读取器（抓包时间与接口元数据）

新增 scripts/pcapng_reader.py，以 Python 标准库实现流式读取，输出统一 packet record，并保留 section_index、interface_id、link_type、captured/original length、原始接口 timestamp ticks（PCAPNG EPB）和换算后的 capture_timestamp_ns。当前覆盖：
- Classic PCAP：little/big endian，microsecond/nanosecond magic，global header link type/snaplen，packet record timestamp 与截断长度检查。
- PCAPNG：Section Header Block（按 BOM 切换 endian，并在新 section 重置 interface table）、Interface Description Block、Enhanced Packet Block、Simple Packet Block；IDB if_tsresol 支持十进制与二进制分辨率，if_tsoffset 按有符号秒偏移处理。
- 对 block 长度/尾部重复长度、option padding/边界、未知 interface、captured/original length 和截断报文做结构检查。SPB 没有 packet timestamp，输出 timestamp_available=false / capture_timestamp_ns=null，不得伪造时间。

### 12.1 时间语义

Classic PCAP 的 packet timestamp 按文件 magic 指定的微秒或纳秒精度换算。PCAPNG EPB 时间戳为 64-bit ticks，必须结合该 interface 的 if_tsresol 和 if_tsoffset 换算；if_tsresol 最高位区分 base-2 与 base-10，低 7 位为 exponent。PCAPNG 的 link type 和 snaplen 是 interface-scoped 元数据。换算结果统一命名 capture_timestamp_ns，它只代表 capture file 所记录的抓包时间，不是 LiDAR timestamp、PTP clock offset 或网络单向时延。

### 12.2 解析边界和测试

本阶段没有解析 Ethernet/VLAN、Linux cooked capture、raw IP、IPv4/IPv6、UDP、IP fragmentation，也没有做 MID-360 UDP port/IP filter；reader 输出 link-layer packet bytes，下一阶段单独实现网络协议解码与 MID-360 payload dispatch。未知 PCAPNG block 按 validated block length 跳过。当前 PCAPNG 支持 SHB/IDB/EPB/SPB，未实现 obsolete Packet Block、Name Resolution Block、Interface Statistics Block、decryption/secrets、FCS 语义及自定义 option 的完整保留。

scripts/tests/test_pcapng_reader.py 使用合成容器覆盖 PCAP microsecond/nanosecond 与 endian、PCAPNG interface link type、decimal/binary if_tsresol、signed if_tsoffset、EPB truncation、SPB 无时间戳和结构错误。合成测试不等于 Wireshark/tcpdump interoperability 或真实 Orin capture 验证。

依据：IETF PCAPNG 格式草案的 IDB / EPB 定义及 if_tsresol、if_tsoffset 语义；其当前状态是 Internet-Draft，项目把它作为格式实现依据，不宣称其为已发布 RFC。


## 13. PCAP 链路层 / IP / UDP 解码与 MID-360 流量提取

新增：
- `scripts/network_packet_decoder.py`：将 PCAP/PCAPNG packet record 解码为 UDP datagram 或明确 disposition。
- `scripts/audit_mid360_pcap.py`：流式读取 classic PCAP/PCAPNG，筛选 MID-360 设备侧 UDP 端口，调用 v1.4.12 payload parser，输出 JSON 计数和少量样本。
- `scripts/tests/test_network_packet_decoder.py`、`scripts/tests/test_audit_mid360_pcap.py`：合成帧与 payload 测试。

### 13.1 支持的 capture link type

按 IANA PCAP LinkType registry：
- Ethernet (1)：EtherType IPv4/IPv6；支持 802.1Q C-tag (0x8100)、802.1ad S-tag (0x88A8) 和常见 0x9100 VLAN tag 的连续嵌套，最多 8 层。
- Linux cooked capture v1 (113) 与 v2 (276)：分别解析 16-byte / 20-byte cooked header。
- Raw IP (101)：按 IP version nibble 识别 IPv4/IPv6。
- IPv4 (228) 与 IPv6 (229)：无 link-layer header 的专用 LinkType。
- 其他 LinkType、Ethernet LLC/length frames、非 IPv4/IPv6 EtherType 返回 unsupported/non_ip，不猜测 payload 类型。

### 13.2 IPv4 / IPv6 / UDP 边界

IPv4 验证 version、IHL（含 options）、total_length 和捕获边界；IPv6 验证 base header / payload_length，解析 Hop-by-Hop、Routing、Fragment、AH、Destination Options、Mobility、HIP、Shim6 等扩展头，限制扩展头链最多 16 个。IPv6 jumbogram（payload_length=0）与 ESP opaque payload 当前不解析。

IPv4 MF/fragment offset 非零，或 IPv6 Fragment header 表明数据报被分片时，统一返回 `fragmented`，不尝试仅凭单个分片解析 UDP，更不把首片当成完整 LiDAR UDP 包。IP 分片重组尚未实现；因此此类报文不会进入 MID-360 payload/CRC 统计。UDP length 必须至少为 8 且不越过 IP packet end，payload 仅截取 UDP length 所定义的范围。

本实现不校验 Ethernet FCS、IPv4 header checksum 或 UDP checksum。NIC checksum offload、抓包点位置可能使 host capture 中的 checksum 字段看起来未完成；本阶段只做结构和长度边界审计，checksum validation 需要独立的 capture metadata / offload-aware 设计。

### 13.3 MID-360 端口筛选合同

Livox 官方 MID-360 v1.4.12 端口表规定设备侧端口：56100 control、56200 push、56300 point-cloud、56400 IMU、56500 log；56000 仅为广播设备发现端口。点云与 IMU 的 host receive port 可配置（文档示例默认 56301 / 56401），因此筛选默认依据设备侧 source port，不硬编码 host destination port。CLI 可通过 `--host-point-cloud-port` / `--host-imu-port` 显式增加 host-side port 映射，并可用 `--lidar-ip` 约束设备地址。端口映射只用于流量候选分类，不替代 MID-360 payload version/length/data_type/time_type/CRC 检验。

`audit_mid360_pcap.py CAPTURE.pcapng --lidar-ip 192.0.2.10` 输出 capture packet dispositions、截断/畸形计数、流类型、有效 payload 数、CRC/格式错误、data_type/time_type 分布、接口及少量 packet 样本。Point-cloud port 的有效 payload 不允许 data_type=0；IMU port 预期 data_type=0，违反时单独计数。控制命令（56100）不进入点云/IMU payload parser。

### 13.4 时间与证据边界

报告同时保留 `capture_timestamp_ns`（抓包文件时间）与 `sensor_timestamp_ns`（MID-360 payload 时间），并显式声明两者之差不是网络延迟。PCAPNG interface ID / section index 继续保留，防止多接口/多 section 数据混淆。抓包记录若标记 truncated，直接计数并跳过；不尝试用 Ethernet padding 补足 IP/UDP payload。

目前测试仅使用合成 Ethernet/VLAN/SLL/SLL2/raw-IP/IPv4/IPv6/UDP frames 和合成 MID-360 payload；PCAP/PCAPNG 16 项、网络解码 13 项、capture audit 3 项单测已在 GitHub Actions run 45641354918 通过。没有真实 MID-360 pcap、设备 IP/host port 配置快照、Orin NIC offload 配置、tcpdump/Wireshark 互操作或 IP fragment reassembly 验证。尚未宣称设备级验收完成。

依据：
- Livox 官方 MID-360 Communication Protocol v1.4.12（端口表、UDP 封装、点云/IMU payload）；
- IANA PCAP LinkType registry（101/228/229/276 等 link type）；
- RFC 9542（EtherType 与 VLAN tag 语义）；
- RFC 791 / RFC 8200（IPv4/IPv6 fragmentation 与 reassembly 语义）；
- tcpdump LINKTYPE_LINUX_SLL2 结构说明（20-byte cooked v2 header）。


## 14. MID-360 UDP 序号连续性、帧复位与乱序审计

新增：
- `scripts/livox_udp_sequence_audit.py`：对单条点云 UDP flow 做有界序号观察。
- `scripts/tests/test_livox_udp_sequence_audit.py`：覆盖 uint16 wrap、前向缺口、迟到补齐、重复、乱序、归零候选和大缺口内存上限。
- `scripts/audit_mid360_pcap.py` 集成 `point_cloud_sequence_audits` 与 `sequence_event_counts`，按 PCAPNG section/interface + src/dst IP/port 隔离 flow。

### 14.1 官方协议字段语义

Livox MID-360 v1.4.12 规定：
- `udp_cnt` 为 uint16 点云 UDP 包计数，每个点云 UDP 包依次加 1，点云帧起始时清零。
- `frame_cnt` 为 uint8 帧计数，每帧加 1；官方明确指出非重复扫描时该字段无效。
- 官方当前参数表将 `pattern_mode=0` 定义为 non-repetitive scan；repetitive / low-frame-rate repetitive scan 标注为当前不支持。

因此：
- 序号归零只报告 `frame_reset_candidate`，不能把 frame_cnt 当作所有配置下的有效帧边界真值。
- 若 `udp_cnt=0` 且 `frame_cnt` 同时变化，仅记录 `frame_counter_changed_at_reset` 作为旁证；若不变则记录 `frame_counter_unchanged_at_reset`，不因此否定归零候选。
- `frame_cnt` 在非零 udp_cnt 处变化，记录为观测异常，不据此自动重排/修复数据。
- uint16 从 65535 到 0 且 frame_cnt 不变按序号模数连续处理；实际帧复位则以归零候选记录，frame_cnt 仅在有效配置中辅助解释。

### 14.2 事件分类

- `in_order`：序号按模数连续。
- `forward_gap`：相对高水位出现前向缺口；统计缺失序号估计值。
- `reordered_gap_recovered`：此前缺口中的序号迟到到达，标记已补齐。
- `duplicate`：重复序号。
- `out_of_order`：较旧序号到达，但不属于当前有界 pending gap。
- `frame_reset_candidate`：非零计数之后观察到 udp_cnt=0。
- `frame_counter_changed_without_udp_reset`：frame_cnt 变化但 udp_cnt 非零。
- `sensor_timestamp_regressions_in_capture_order`：抓包遍历顺序中传感器 timestamp 回退；它可能是 UDP 乱序，不等于传感器时钟倒退。

每个 flow 独立维护状态，pending gap 和 recent duplicate window 有大小上限，避免损坏计数导致无界内存占用。超出 pending 上限的大缺口只计数，不逐个存储序号。

### 14.3 解释边界

`missing_counter_values_estimated` 是**抓包观察到的序号缺口估计**，不是网络层丢包计数。缺口可能来自网络丢包、NIC/内核/抓包工具丢包、过滤条件、抓包接口或捕获文件不完整；乱序到达也可能使缺口随后补齐。即使缺口未补齐，也不能仅凭 pcap 区分上述原因。

此外，分析只针对单个 capture file 的遍历顺序和单一 flow；多个接口、镜像口、不同 capture timestamp 精度、重复采集会改变观测结果。当前不做 IP 分片重组，也不把 IMU 的 udp_cnt 套用点云序号规则。

合成测试验证的是算法行为，不是设备实际包率、帧模式、网络丢包或 NIC 捕获性能。序号审计单测 5/5、集成 capture audit 5/5 已在 GitHub Actions run 45642338867 通过。真实 MID-360 pcap 与设备 pattern_mode 配置仍需取得后再做硬件验收。
