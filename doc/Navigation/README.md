# RobDog Navigation Module — 总体研究与工程实施目录

> **研究对象是完整机器人导航模块。Nav2 只是其中一个子系统，不等于完整导航系统。**

- 专项工作分支：nav（唯一允许开展本专项研究、文档、代码和测试的分支）
- 主线分支：main（专项完成并通过验收前，不接收本专项中间成果）
- 候选平台：DEEP Robotics Lite3 Venture + Livox MID-360 + 双 RGB 被动双目 + ZED-F9P RTK GNSS + Jetson AGX Orin
- 软件研究基线：ROS 2 Jazzy；兼顾 Humble 兼容性。最终部署版本须经过 BSP、驱动和硬件验证
- 进度唯一真源：仓库根目录 TaskPlan.md

## 1. 目标与系统边界

建设从传感器观测到四足机器人安全运动执行的完整导航能力，覆盖：

1. **任务输入与任务管理**：目标点、航点、区域/路线、巡检、跟随/伴随、暂停/恢复/取消、优先级与异常处置。
2. **传感器与设备接入**：3D LiDAR、双目/RGB-D、IMU、GNSS/RTK、足端/关节/机身状态、里程计、时间同步与硬件健康。
3. **状态估计与定位**：腿式里程计、LIO、VIO、视觉/激光/惯性融合、GNSS 融合、重定位、连续性与不确定度管理。
4. **建图与地图管理**：SLAM、静态/动态地图、地图坐标系、地图版本/切换、语义/拓扑/高程地图。
5. **环境感知**：点云/图像预处理、地面与障碍分割、动态目标检测跟踪、负障碍/悬崖/台阶检测、多传感器融合。
6. **环境表达与可通行性**：2D/3D costmap、elevation/terrain/semantic/topological map、坡度/粗糙度/台阶/落足约束和不确定区域。
7. **规划与避障**：任务/路线/拓扑层、全局路径、局部轨迹、动态避障、四足运动学/动力学/步态约束、恢复策略。
8. **导航框架与协调**：Nav2（BT、Planner、Controller、Costmap、Behavior 等）、替代/自研协调层和任务状态机。
9. **运动执行接口**：速度意图、机身速度/姿态/步态目标、厂商 SDK、平台 command adapter、执行反馈。
10. **安全**：独立急停、驱动禁能、watchdog、命令仲裁、限速/限加速度、地形风险、失联/定位失效/传感器失效降级。
11. **运行时与部署**：ROS graph、Lifecycle、Executor/QoS、DDS、网络、时钟、Jetson CPU/GPU/内存/温度/功耗、日志和可观测性。
12. **验证与运维**：数据集、rosbag、仿真、回放、台架、HIL、封闭场地、回归、故障注入、性能和安全验收。

边界原则：
- 导航模块输出受约束的运动意图/轨迹请求，不替代四足底层高频关节控制和步态闭环。
- Nav2 不自动提供传感器驱动、可靠定位、完整 3D 地形理解、落足规划、厂商控制接口或安全认证。
- 所有外围组件都必须明确责任、接口、时间语义、质量门控和失效传播。
- 所有运动命令经过独立安全链和平台适配；厂商控制权、急停、制动和安全操作规程未确认前，禁止向真实机器狗发送运动命令。

## 2. 总体分层与端到端闭环

    任务/应用层
      目标点、航点、巡检、跟随、区域任务、优先级、取消/暂停
             │ Task API / Action / mission state
             ▼
    任务协调与导航决策层
      任务状态机、行为树/状态机、路线决策、重规划/恢复、模式管理
             ├────────────── 地图与先验管理
             │                 地图服务器、SLAM、2D/3D/语义/拓扑地图
             ▼
    定位与状态估计层
      腿式里程计 / LIO / VIO / GNSS / IMU 融合
      map→odom→base_link、协方差、定位健康度
             │
             ▼
    环境理解层
      点云/图像、地面/障碍/动态目标、负障碍
      elevation / traversability / semantic
             ▼
    环境表达层
      Static/Obstacle/Voxel Costmap、3D terrain、elevation、semantic、topological graph
             ▼
    规划与避障层
      全局路线/路径规划 → 局部轨迹/控制 → 恢复行为
      足式运动学、步态、速度、坡度、台阶、落足风险约束
             ▼
    导航框架/执行协调层
      Nav2 或替代框架；BT、Planner、Controller、Behavior
             │ Twist/TwistStamped 或平台定义的运动意图
             ▼
    命令仲裁与安全层
      限幅、限速、碰撞监控、风险门控、watchdog、急停状态
             ▼
    四足平台适配层 → 厂商运动控制器 → 关节/足端 → 机器人运动
             │
             └── IMU/关节/接触/本体 odom/执行状态反馈闭环

    传感器层：LiDAR / Stereo / IMU / GNSS / 足端 / 关节
    时间同步、标定、驱动、健康诊断、数据录制，为定位/感知/地图提供输入。

## 3. 研究目录与交付物

| 文档 | 研究主题 | 必须回答的问题 |
|---|---|---|
| 00 总体架构与边界 | 全模块架构 | 子系统、责任边界、依赖、控制闭环、失效传播 |
| 01 数据流与接口总表 | Producer/Consumer | 数据生产者/消费者、类型/单位/frame/stamp/QoS/频率/质量门控/超时 |
| 03 ROS 2 核心接口契约 | ROS message/action | Imu、Odometry、PointCloud2、TwistStamped、NavigateToPose、FollowPath 字段和语义 |
| 02 传感器驱动与时间同步 | 设备接入 | 驱动、协议、时钟域、PTP/PPS/trigger、标定、健康度 |
| 03 定位、状态估计与 SLAM | 位姿来源 | 腿式 odom、LIO/VIO、GNSS、融合、重定位、漂移和协方差 |
| 04 环境感知与动态目标 | 观测理解 | 点云/深度/语义、地面/障碍/动态目标/负障碍 |
| 05 地图、Costmap 与地形表达 | 环境模型 | 2D/3D/elevation/semantic/topological map、更新和持久化 |
| 06 可通行性与四足地形分析 | 四足特有能力 | 坡度、粗糙度、台阶、沟坎、悬崖、落足区、机身姿态/步态限制 |
| 07 规划、避障与算法 | 路线到轨迹 | 全局/局部/拓扑/动态规划、预测、优化、恢复、算法比较 |
| 08 Nav2 子系统深研 | Nav2 内部机制 | 生命周期、BT、服务器、插件、costmap、消息与源码调用链 |
| 09 四足运动控制接口 | 运动执行 | cmd_vel 到 gait/body command 的映射、能力边界、反馈和拒绝策略 |
| 10 安全架构与故障降级 | 安全闭环 | E-stop、watchdog、命令仲裁、定位/传感器/网络故障、风险分析 |
| 11 ROS 2 运行时与系统部署 | 工程运行 | launch、Lifecycle、Executor、QoS、DDS、网络、资源与实时性 |
| 12 测试、数据集与验收 | 证据体系 | 单测、集成、仿真、回放、台架、HIL、实机和量化指标 |
| 13 项目集成路线与 ADR | 工程落地 | 包结构、版本锁定、部署拓扑、接口冻结、决策记录 |

每份专题至少包含：目标/非目标、术语、模块图、Producer/Consumer、接口契约、算法、软件/硬件、参数和单位、时间/frame/QoS、质量与失效处理、算力/时延/带宽、安全约束、源码/官方资料证据、未决问题和验证计划。

## 4. 每条数据链必须追踪

- Producer、Consumer、数据对象/ROS interface、Topic/Action/Service/TF（仅在已核实后填写）。
- 消息类型、字段语义、单位、坐标系、frame_id、child_frame_id、时间戳来源和时钟域。
- 频率、延迟、抖动、数据龄期、QoS、队列深度、丢包和带宽。
- 外参/内参/时间偏移、同步策略、标定版本。
- covariance/置信度/质量标志、validity gate、超时和 stale-data 行为。
- 上游失效如何传播；是否冻结、降级、重定位、重规划或安全停车。
- 观测数据与真值/评测数据隔离；记录原始输入、处理结果和版本。
- 运动命令必须追踪到实际执行反馈，不能以发布 cmd_vel 视作运动成功。

## 5. 四足导航特有研究项

- 机器人横移/原地旋转/倒退能力、速度/加速度边界、转弯半径和运动学模型。
- 机身姿态变化、摆动、足端接触、滑移、打滑、腾空/跳跃对状态估计和点云畸变的影响。
- 地形高度、坡度、粗糙度、台阶高度、沟槽宽度、负障碍、松软/湿滑/低摩擦表面。
- 2D footprint 与真实支撑多边形/足端可落区域的差异；2D costmap 的能力边界。
- 地形代价如何影响全局路线和局部轨迹；落足规划由谁负责、接口如何交接。
- 步态切换、机身高度/姿态策略、速度规划、制动距离和稳定性约束。
- 传感器遮挡/振动/运动畸变、动态行人和狭窄环境策略。

## 6. 证据与质量规则

1. 优先官方源码、API/消息定义、供应商正式资料、论文原文和可复现数据；记录 URL、版本/tag/commit、访问日期和 ROS 发行版。
2. 区分源码事实、官方文档、论文结果、项目实测、设计推断、待供应商确认；不可把推断写成事实。
3. 源码追踪记录文件路径、类/函数、入口、调用链、执行线程/executor/callback、参数、消息契约和失败路径。
4. 外部仓库锁定 tag/commit、许可证、依赖和维护状态；评估安全性、ROS 兼容、ARM64/Jetson 支持。
5. 性能结论记录硬件、软件 SHA、数据、场景、参数、重复次数和原始日志。
6. 合成测试不等于真实 rosbag；x86 build 不等于 Jetson build；仿真不等于 HIL/实机；无独立 GT 不报告定位精度。
7. Lite3 odometry、SDK 控制权、命令语义、制动、急停、供电、载荷和时序不确定项一律标待确认。
8. 未完成供应商授权、安全机制、物理急停、操作规程和封闭场地风险评估前，不向真实机器狗发送运动命令。

## 7. 专项阶段

- N0 研究基线与范围冻结：术语、系统边界、分层架构、分支策略、资料基线。
- N1 端到端数据流与接口审计：producer/consumer、消息、frame、stamp、QoS、频率、延迟、质量和失效矩阵。
- N2 传感器/时间/标定：驱动源码、设备协议、时间同步、外参、数据健康和记录链。
- N3 定位/SLAM/地图：比较腿式 odom、LIO、VIO、GNSS、融合、建图、重定位。
- N4 感知/地形/可通行性：从原始传感器到 obstacle/elevation/traversability。
- N5 规划/避障/任务协调：全局、局部、拓扑、动态目标、恢复和四足约束。
- N6 Nav2 深研：固定 Jazzy 源码版本，追踪 Action→BT→Planner→Controller→command 调用链及外围集成。
- N7 四足执行、安全和运行时：运动接口、命令仲裁、独立保护、ROS 2 runtime、Jetson 部署。
- N8 验证与系统方案收敛：仿真/回放/台架/HIL/实机分层；完成 ADR、接口冻结、集成架构和实施路线。
- N9 专项验收与回归：文档、引用、接口、风险、未决问题和实验归档；通过审查后再提出回 main 的集成方案。

## 8. 当前项目 Gate

- Lite3 Venture 未采购；SDK 控制权、腿式 odometry、速度接口、步态能力、急停、制动距离、载荷/供电/CAD 尚未完整确认。
- MID-360、双目、ZED-F9P 尚未在目标 Orin 硬件上完成完整联调。
- Dynamic03 数据中 GT 与普通 GNSS raw-fix 高度同源，且不具备 RTK Fixed 证据；不得作为独立 APE/RPE ground truth。
- Nav2 首轮文档是架构级资料，不代表节点集成、仿真闭环或实机通过。
- 本轮专项的完成仅指研究与工程基线达到验收；真实机器人功能验收仍需独立硬件 Gate。

## 9. Nav2 现有资料的定位

原 doc/Nav2/ 8 篇文档整体保留，作为本目录第 08 专题的初始材料。其架构、生命周期、Producer/Consumer、Costmap、算法插件、硬件软件模块、四足安全和验证路线必须在完整导航研究中补齐上下游依赖，不得将其误认为整个导航模块已经研究完成。

## 10. 上游研究候选入口（待逐项固定 commit）

- ROS 2 Jazzy：https://docs.ros.org/en/jazzy/
- Nav2 Jazzy：https://docs.nav2.org/jazzy/
- Navigation2：https://github.com/ros-navigation/navigation2
- robot_localization：https://github.com/cra-ros-pkg/robot_localization
- slam_toolbox：https://github.com/SteveMacenski/slam_toolbox
- Point-LIO：https://github.com/hku-mars/Point-LIO
- FAST-LIO2：https://github.com/hku-mars/FAST_LIO
- image_pipeline：https://github.com/ros-perception/image_pipeline
- vision_msgs：https://github.com/ros-perception/vision_msgs
- elevation_mapping：https://github.com/ANYbotics/elevation_mapping
- grid_map：https://github.com/ANYbotics/grid_map
- traversability_estimation：https://github.com/ANYbotics/traversability_estimation
- ros2_control：https://github.com/ros-controls/ros2_control
- ros2_controllers：https://github.com/ros-controls/ros2_controllers
- Livox ROS Driver 2：https://github.com/Livox-SDK/livox_ros_driver2

以上是候选入口，不代表已审查维护状态、许可证、接口或 Jetson 兼容性；结论以源码 pin 和专项审计为准。


## 11. 已形成的专题审计文档

- [04 — Nav2 Bringup 与 Action/命令接口审计](04_nav2_bringup_interface_audit.md)
- [05 — 传感器观测到 Nav2 Costmap 输入链审计](05_sensor_to_costmap_observation_audit.md)
- [06 — Costmap 健康状态与 Planner/Controller 消费端审计](06_costmap_health_and_consumers_audit.md)
- [07 — MID-360 点云到高程地图与四足可通行性链路审计](07_mid360_elevation_traversability_audit.md)
- [08 — Point-LIO 与 FAST-LIO2：MID-360 LIO / 时间补偿 / ROS 2 适配审计](08_point_lio_fast_lio2_audit.md)
- [09 — MID-360 时间戳合同与 Point-LIO ROS 2 集成依赖审计](09_mid360_timestamp_and_point_lio_integration_audit.md)
- [10 — Point-LIO 去 MRS 化与四足状态估计接口审计](10_point_lio_decoupling_and_state_interface_audit.md)
- [11 — GNSS generation 与跨组件 Reset Coordinator 设计](11_gnss_generation_and_reset_coordinator_design.md)
- [12 — ZED-F9P 固定驱动的 Receiver Generation 证据审计](12_zed_f9p_receiver_generation_evidence_audit.md)
- [13 — ZED-F9P UTC / GPS 周时间映射与固定驱动扩展审计](13_zed_f9p_utc_gps_time_mapping_audit.md)

上述文档均为分主题研究材料；项目进度、Gate、状态与提交证据以根目录 `TaskPlan.md` 为准。

- [14 — ZED-F9P UTC→GPS Mapper 补丁计划与纯算法测试向量](14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md)
- [15 — 传感器时钟域与全系统时间同步架构审计](15_sensor_clock_domains_and_sync_architecture.md)
- [16 — GNSS PTP Grandmaster 与 PPS 时间标签分发 ADR 候选](16_gnss_ptp_grandmaster_vs_pps_distribution_adr.md)\n- [09 §10 — MID-360 原始 UDP、CustomMsg 与 rosbag 接收时间证据边界](09_mid360_timestamp_and_point_lio_integration_audit.md#10-原始-udp-包custommsg-与-rosbag-接收时间的证据边界)\n- [17 — Orin PTP 主机/NIC 基线采集与人工判读](16_gnss_ptp_grandmaster_vs_pps_distribution_adr.md#17-目标-orin-主机nic-基线采集工具-time-045--time-052)\n- [18 — GM/交换机/Orin/MID-360 PTP 抓包与解码检查表](16_gnss_ptp_grandmaster_vs_pps_distribution_adr.md#18-gm--switch--orin--mid-360-三观察面-ptp-抓包与解码检查表-time-053--056)

- [09 §11 — MID-360 原始 UDP payload 严格解析器与合成测试合同](09_mid360_timestamp_and_point_lio_integration_audit.md#11-mid-360-原始-udp-payload-严格解析器协议-v1.4.12)
- [MID-360 原始 UDP payload 解析器](../../scripts/livox_mid360_udp_parser.py)
- [MID-360 原始 UDP payload 解析器单元测试](../../scripts/tests/test_livox_mid360_udp_parser.py)

- [09 §12 — Classic PCAP / PCAPNG 容器读取、接口元数据与抓包时间语义](09_mid360_timestamp_and_point_lio_integration_audit.md#12-pcap--pcapng-容器读取器抓包时间与接口元数据)
- [PCAP / PCAPNG 流式读取器](../../scripts/pcapng_reader.py)
- [PCAP / PCAPNG 读取器单元测试](../../scripts/tests/test_pcapng_reader.py)

- [09 §13 — Ethernet/VLAN、Linux cooked、IPv4/IPv6/UDP 与 MID-360 PCAP 流量审计](09_mid360_timestamp_and_point_lio_integration_audit.md#13-pcap-链路层--ip--udp-解码与-mid-360-流量提取)
- [MID-360 PCAP/PCAPNG 审计 CLI](../../scripts/audit_mid360_pcap.py)
- [网络包解码器](../../scripts/network_packet_decoder.py)
- [网络包解码器测试](../../scripts/tests/test_network_packet_decoder.py)
- [MID-360 PCAP 审计测试](../../scripts/tests/test_audit_mid360_pcap.py)

- [09 §14 — MID-360 UDP 序号连续性、帧复位与乱序审计](09_mid360_timestamp_and_point_lio_integration_audit.md#14-mid-360-udp-序号连续性帧复位与乱序审计)
- [MID-360 UDP 序号审计器](../../scripts/livox_udp_sequence_audit.py)
- [MID-360 UDP 序号审计测试](../../scripts/tests/test_livox_udp_sequence_audit.py)

- [09 §15 — PTPv2 / gPTP 报文识别与消息配对审计](09_mid360_timestamp_and_point_lio_integration_audit.md#15-ptpv2--gptp-报文识别与消息配对审计)
- [PTPv2 / gPTP 抓包解析与关联器](../../scripts/ptp_packet_audit.py)
- [PTP 报文解析与关联测试](../../scripts/tests/test_ptp_packet_audit.py)
- [PTP timestamp exchange / correctionField 原始证据与 Orin 运行态核验](09_mid360_timestamp_and_point_lio_integration_audit.md#155-sync--follow_updelay-与-correctionfield-时间戳关联)
- [Orin PTP host inventory 只读采集脚本](../../tools/collect_ptp_host_inventory.sh)
- [PTP Announce 候选观测、GM 声明变化与序号连续性审计](09_mid360_timestamp_and_point_lio_integration_audit.md#154-announce-候选来源变化与序号连续性观测)
- [PTP responder identity 与 MID-360 time_type 共观测审计](09_mid360_timestamp_and_point_lio_integration_audit.md#16-ptp-exchange-responder-校验与-mid-360-timebase-共观测)
- [MID-360 PTP/gPTP 混合传输与多 GM 风险检查](09_mid360_timestamp_and_point_lio_integration_audit.md#17-mid-360-ptpgptp-混合传输与多-gm-风险检查)

- [09 §18 — MID-360 控制协议时间同步诊断字段解码（0x8009–0x800E）](09_mid360_timestamp_and_point_lio_integration_audit.md#18-mid-360-控制协议时间同步诊断字段解码time-diag-001008)
- [MID-360 控制协议诊断解析器](../../scripts/livox_mid360_control_parser.py)
- [MID-360 控制协议诊断解析器单元测试](../../scripts/tests/test_livox_mid360_control_parser.py)

- [09 §19 — MID-360 HMS 故障诊断码与时间同步异常关联](09_mid360_timestamp_and_point_lio_integration_audit.md#19-mid-360-hms-故障诊断码与时间同步异常关联hms-001010)

- [09 §20 — MID-360 HMS 连续快照生命周期审计](09_mid360_timestamp_and_point_lio_integration_audit.md#20-mid-360-hms-连续快照生命周期审计hms-011016)
- [17 — ZED-F9P NAV-TIMEGPS：UTC→GPS Mapper 可选同历元交叉核验](17_zed_f9p_nav_timegps_crosscheck_audit.md)
- [18 — 固定版 ublox_dgnss 时间消息驱动实现审计](18_ublox_dgnss_fixed_revision_time_message_driver_audit.md)
- [19 — UBX / RTCM3 / NMEA 增量字节流解析器设计与测试合同](19_ubx_rtcm_nmea_stream_parser_design_and_test_contract.md)

- [独立 UBX/RTCM3 C++ stream parser（第一实现切片）](../../navigation/ubx_rtcm_stream/README.md)
- [UBX/RTCM3 parser public API](../../navigation/ubx_rtcm_stream/include/robodog_protocol/stream_decoder.hpp)
- [UBX/RTCM3 parser implementation](../../navigation/ubx_rtcm_stream/src/stream_decoder.cpp)
- [UBX/RTCM3 parser tests](../../navigation/ubx_rtcm_stream/tests/test_stream_decoder.cpp)

- [20 — UBX/RTCM3 有界 ingress queue、generation fence 与 TSan 合同](20_ubx_rtcm_bounded_ingress_queue_design_and_tsan_contract.md)

- [21 — libusb transfer 生命周期状态机与 fake-transport 验收合同](21_ubx_dgnss_libusb_transfer_lifecycle_and_fake_transport_contract.md)
- [Transfer lifecycle tracker](../../navigation/ubx_rtcm_stream/include/robodog_protocol/transfer_lifecycle.hpp)
- [Transfer lifecycle synthetic tests](../../navigation/ubx_rtcm_stream/tests/test_transfer_lifecycle.cpp)

- [22 — Fake USB transport seam 与 submit/cancel/shutdown 测试合同](22_ubx_dgnss_fake_usb_transport_seam_and_test_contract.md)
- [Fake USB transport API](../../navigation/ubx_rtcm_stream/include/robodog_protocol/fake_usb_transport.hpp)
- [Fake USB transport tests](../../navigation/ubx_rtcm_stream/tests/test_fake_usb_transport.cpp)

- [23 — libusb asynchronous I/O 官方语义核对与真实 adapter 接入合同](23_libusb_async_io_contract_and_adapter_integration_gate.md)

- [24 — 固定版 ublox_dgnss 真实 libusb adapter 源码映射与集成设计](24_fixed_ublox_dgnss_libusb_adapter_source_audit_and_design.md)

- [25 — 注入式 libusb transfer adapter 第一实现切片与测试合同](25_injectable_libusb_transfer_adapter_first_slice.md)
- [LibusbTransferTransport API](../../navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_transfer_transport.hpp)
- [Fake libusb API adapter tests](../../navigation/ubx_rtcm_stream/tests/test_libusb_transfer_transport.cpp)

- [26 — libusb-1.0 生产 API function-table binding 与编译门禁](26_libusb_1_api_binding_and_compile_gate.md)
- [libusb-1.0 API binding header](../../navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_api_binding.hpp)
- [libusb-1.0 API binding implementation](../../navigation/ubx_rtcm_stream/src/libusb_api_binding.cpp)

- [27 — 固定版 ublox_dgnss USB bulk endpoint 与 transfer 参数映射](27_fixed_ublox_dgnss_usb_endpoint_and_transfer_mapping.md)

- [28 — USB DeviceSession 状态机与 libusb transfer 长度安全合同](28_usb_device_session_design_and_length_safety_contract.md)

- [29 — libusb transfer registry 并发审计与修复记录](29_libusb_transfer_registry_concurrency_audit.md)
- [30 — libusb cancel 锁耦合与错误语义审计](30_libusb_cancel_locking_and_error_semantics.md)

- [31 — libusb DeviceSession RAII 与动态 endpoint discovery 第一切片](31_libusb_device_session_raii_and_descriptor_discovery.md)
- [LibusbDeviceSession API](../../navigation/ubx_rtcm_stream/include/robodog_protocol/libusb_device_session.hpp)
- [LibusbDeviceSession tests](../../navigation/ubx_rtcm_stream/tests/test_libusb_device_session.cpp)
