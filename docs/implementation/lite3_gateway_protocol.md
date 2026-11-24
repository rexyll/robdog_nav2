# Lite3 跨 ROS 发行版状态网关协议（设计提案）

- 状态：**Proposed v0.1；尚未获得云深处确认；未实现/未用于真实机器人**
- 项目进度唯一台账：[根目录 TaskPlan.md](../../TaskPlan.md)
- 厂商接口评估：[Lite3 vendor SDK assessment](lite3_vendor_sdk_integration_assessment.md)
- 集成决策：[ADR-0003](../decisions/ADR-0003-lite3-ros-integration.md)

## 1. 目标与非目标

目标：
- 在厂商要求的 Foxy / DRDDS 进程与 Nav2 Jazzy 进程之间建立显式消息边界；
- 不依赖跨发行版 ROS 2 DDS 发现或 CDR 兼容；
- 把 vendor 数据转换为项目自己的版本化传输结构，再转换成 Jazzy 的标准 ROS 消息；
- 保留 source timestamp、clock domain、sequence、frame、单位、协方差/质量和状态；
- 可检测丢包、乱序、超时、重启、状态机故障和无效数据。

非目标：
- 不替代 Lite3 官方 SDK，也不在网关中实现步态/关节控制；
- 不在当前版本传输或执行 Nav2 运动命令；
- 不假定 vendor IMU/joint/status 可推出完整 odometry；
- 不承诺网络或操作系统层面硬实时；
- 不在未完成 vendor approval、安全审查和测试前启用 robot motion。

## 2. 逻辑拓扑

```text
Lite3 Motion Host (vendor-supported Foxy / SDK / DRDDS)
        |
        |  vendor SDK topics/services (local DDS only)
        v
robodog_lite3_vendor_gateway
        |
        |  versioned non-DDS telemetry protocol
        |  UDP data plane + optional reliable control/status plane
        v
Jetson AGX Orin (ROS 2 Jazzy)
        |
        v
robodog_lite3_bridge
        |
        +--> /robot/imu/data
        +--> /robot/joint_states
        +--> /robot/status
        +--> /robot/odometry  (only when a trusted source exists)
        |
        v
robot_localization / Nav2
```

vendor gateway 在厂商 Foxy 侧运行；Jazzy bridge 在 AGX Orin。实际 motion-host / AGX-host 分工仍需云深处书面确认。

## 3. 传输候选

### 3.1 Telemetry data plane

首选 POC 候选：UDP unicast，每个数据包一个完整 envelope。

适用：
- IMU / joint telemetry；
- 高频 robot status；
- 可丢弃、下一个样本会更新的实时状态。

约束：
- 每个 datagram 不超过路径 MTU（默认目标约 1400 bytes），禁止 IP fragmentation；
- 每种消息有独立 sequence number；
- 允许 UDP 丢包，但必须监测缺号、重复、乱序和长时间无数据；
- 接收端不对过期消息“补发”到实时估计器；
- 不使用 UDP telemetry 搬运关节控制或安全停止请求。

可靠控制/配置平面如未来经厂商批准，再评估 TCP/TLS 或明确确认/重试的可靠 RPC；不与 telemetry datagram 混在一个未定义协议里。

### 3.2 序列化候选

- 建议评估 Protocol Buffers（有 schema/version，跨语言支持好）；
- 若 vendor SDK 环境不便生成 protobuf，可评估 CBOR；
- 不直接把 ROS 2 CDR bytes 当作跨发行版稳定协议；
- 最终选择必须基于 Foxy host 可部署性、ARM64 性能、代码许可和实际包大小测试。

当前只定义语义和 envelope，不锁定 protobuf/CBOR，也不提交可被误认为正式的 wire encoder。

仓库已在 `src/robodog_lite3_bridge/robodog_lite3_bridge/telemetry_contract.py` 实现不依赖编码格式的元数据校验、消息类型/帧要求、会话/序列跟踪、NaN/Inf 和 ROS covariance 长度/对角线基本检查；它尚未实现 socket、序列化或 vendor SDK。Covariance helper 不证明完整矩阵正半定，只做早期数据卫生检查。该纯逻辑测试已在 CI run 45328657069 的 Humble/Jammy 和 Jazzy/Noble 环境均通过，25 tests each。

## 4. Envelope 必需字段

每一条 telemetry 消息都必须携带：

| 字段 | 语义 | 要求 |
|---|---|---|
| protocol_version | 网关协议主/次版本 | 必须 |
| message_type | IMU / JointState / ContactState / RobotStatus / BaseOdometry / Heartbeat | 必须 |
| robot_id | 设备/机器人实例标识 | 必须，不能仅依赖 IP |
| session_id | gateway 每次启动产生的新会话标识 | 必须，识别进程重启 |
| sequence | 每个消息流单调递增序号 | 必须 |
| source_timestamp_ns | vendor 侧采样/生成时刻 | 若源数据提供；缺失必须明确标 no_source_time |
| source_clock_id | source timestamp 属于哪个设备/时钟域 | 必须 |
| gateway_monotonic_ns | gateway 收到/封装时的 monotonic 时间 | 必须 |
| gateway_ros_time_ns | 可选 ROS time，不能替代 source time | 可选 |
| frame_id | 数据物理坐标系 | 有空间量时必须 |
| unit_convention | 单位/轴向约定版本 | 必须 |
| validity_flags | 字段有效性、传感器饱和、时间戳质量 | 必须 |
| payload_length / checksum | 长度与完整性 | 传输层必须校验 |

source timestamp、gateway receive time、host ROS time 是三个不同概念，不能混为一谈。

## 5. 消息类型与字段契约

### 5.1 ImuState

候选 payload：
- orientation / angular_velocity / linear_acceleration；
- 每个子字段独立 valid flag；
- covariance 或由 vendor 明确提供的 quality / variance；
- unit：rad、rad/s、m/s²；
- frame_id：经核实的 imu_link；
- saturation/clipping flag；
- source timestamp 和 clock domain。

若 vendor DRDDS 的姿态角没有 quaternion 或 covariance，gateway 必须按已验证的约定转换，并在 orientation_covariance 中表达真实不确定度；不能填零矩阵假装精确。

### 5.2 JointState

候选 payload：
- names / positions / velocities / efforts；
- 每个关节的 valid flag；
- units：rad、rad/s、N·m（以厂商说明为准）；
- 明确 joint mapping version；
- 关节顺序不得仅依赖数组下标，必须映射到 URDF joint names；
- 保留原始 vendor index/name 供审计。

### 5.3 ContactState（字段待厂商确认）

候选 payload 语义：
- 每只足的稳定 foot_id / vendor index 映射；
- contact valid / sensor health 标志；
- 接触力向量或法向力标量（具体形式由目标固件接口决定，不在本项目猜测）；
- 原始单位、符号方向、坐标 frame、采样时间和质量标志；
- 若厂商提供 slip/contact confidence，作为独立字段，不从力值自行推断。

要求：
- contact_state envelope 必须带 frame_id；其 frame 与每足局部 frame/机器人公共 frame 的关系必须写入版本化 unit/frame convention；
- 不能只根据数组顺序猜测左前/右前/左后/右后，必须取得官方映射；
- 不把接触力作为机身位置/线速度，也不直接发布 nav_msgs/Odometry；
- 只有采样率、时间戳、轴向和零点校准得到确认后，才允许用于足端接触/滑移诊断。

### 5.3 RobotStatus

候选 payload：
- SDK mode；
- command state；
- safety mode；
- emergency stop active；
- telemetry_fresh；
- command timeout/rejected counters；
- gait/motion state；
- vendor error codes；
- gateway health；
- motion command enabled flag（本项目初始固定 false）。

### 5.4 BaseOdometry（可选，不能虚构）

只有在云深处确认 vendor 提供可信 base/legged odometry，或其他里程计算法 POC 通过验收后才允许发布：
- pose / twist；
- covariance；
- frame_id / child_frame_id；
- source timestamp / clock domain；
- estimation source（vendor_legged / point_lio / stereo_vio）；
- quality / slip / reset flags。

**没有可信位置/速度来源时，禁止构造 BaseOdometry。**

## 6. 数据质量与失效行为

| 条件 | 网关动作 | Jazzy bridge 动作 |
|---|---|---|
| sequence gap | 增加 drop counter | 诊断丢包率；不插值伪造 IMU/odom |
| duplicate / out-of-order | 记录并丢弃或按类型隔离 | 不倒序更新状态估计 |
| source clock reset | 新建/识别 clock epoch | 旧时间段数据作废，状态进入 recovering |
| session_id changed | 标记 gateway restart | 重新确认数据新鲜度和时间映射 |
| source timestamp missing | 显式置 no_source_time | 不用于高精度去畸变或紧耦合融合 |
| telemetry stale | heartbeat/status 报告 stale | 发布诊断；按任务策略限速或停车 |
| invalid frame / unit version | 拒绝该消息 | 不把错误坐标送入 TF/costmap |
| NaN/Inf/out-of-range | 标记 invalid 并计数 | 丢弃，不将非法数据发布为有效观测 |
| vendor E-stop active | 只读转发状态 | 导航立即取消并保持停止；不自动复位 |
| gateway link lost | 状态 stale | 命令路径保持 disabled；已有运动命令由独立底层 watchdog 处理 |

所有 freshness 门限以数据率和实测网络延迟确定，不能直接照抄示例。

## 7. 时钟映射

- vendor source clock、gateway host monotonic clock、AGX host monotonic clock、ROS time 必须分别命名。
- 使用双向时间戳测量或 PTP/PPS 建立 offset/drift 估计；如果 source timestamp 不可得，消息必须明确只带 gateway receive time。
- 不允许将 Foxy header.stamp（若实际来自 steady clock）直接解释为 AGX Jazzy ROS time。
- 为每一 source clock 设置 epoch/session；设备重启后 clock reset 不得与旧数据混合。
- 每个转换记录 offset、drift、skew 的 P50/P95/P99/max。

## 8. 安全命令平面（本版禁用）

当前协议版本不定义可执行运动命令。

未来如 vendor 同意，可单独设计：
- command lease / expiry；
- unique command ID + acknowledgement；
- approved command_state / SDK mode；
- max velocity/acceleration envelope；
- safety gate status；
- E-stop latch state；
- watchdog and fail-safe acknowledgement。

即便未来启用，也不能替代物理 E-stop、独立安全装置和底层命令 watchdog。所有 command API 必须经厂商审查与封闭测试后才能接入 Nav2。

## 9. 兼容性和版本

- major version 不兼容时拒绝通信；
- minor version 增加可选字段时保持向后兼容；
- gateway startup 必须发布 capabilities/versions；
- Jazzy bridge 必须拒绝未知 message_type / unit_convention；
- 每次录制 rosbag 记录 protocol_version、gateway commit、vendor SDK commit、firmware、ROS distro 和 clock calibration version。

## 10. 实施顺序

1. 先向云深处确认 current SDK / Foxy host / AGX host 拓扑与可读取的字段；
2. 在无运动命令条件下确认数据源、source timestamp、单位、frame、质量字段；
3. 以 mock payload 编写 encoder/decoder contract tests；
4. 在 isolated LAN 上测试 UDP loss/reorder/restart/stale/clock reset；
5. 将通过的 telemetry payload 转成 Jazzy 标准 ROS messages；
6. 只读接入 robot diagnostics 与 EKF（仅在存在可信 odometry 时）；
7. command plane 保持禁用，直到安全 Gate 全部通过。

本提案不代表云深处已批准该协议，也不代表厂商 SDK 可在任意网络路径下导出这些数据。


## 4.2 Full ROS covariance matrix validation

The bridge validator accepts flattened row-major 3x3 and 6x6 covariance arrays. Beyond length, finite-value, and non-negative diagonal checks, it also verifies:

- symmetry within a relative tolerance of 1e-9;
- positive semidefiniteness using a small-matrix Jacobi eigensolver;
- a minimum-eigenvalue tolerance of 1e-10 times the maximum absolute matrix entry to avoid rejecting harmless floating-point round-off;
- the standard sensor_msgs/Imu unknown-estimate sentinel (covariance[0] == -1) only when allow_unknown=True.

The validator does not treat an all-zero covariance as evidence of a trustworthy estimate: zero is mathematically PSD but may still mean that an upstream driver failed to populate uncertainty. Source-specific quality policy must separately decide whether zero/near-zero covariance is admissible. The PSD check is a structural consistency gate, not proof that the reported covariance is statistically calibrated or physically truthful.
