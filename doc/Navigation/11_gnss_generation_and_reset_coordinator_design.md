# GNSS Generation Controller 与跨组件 Reset Coordinator 设计

> 状态：设计规格（E3）；尚未实现。本文不定义 ZED-F9P 驱动的实际消息映射，不声明接收机重启判据已经得到实测验证。

## 1. 目标与非目标

目标是把 GNSS receiver clock/generation 判断、现有 Gate/Joiner 生命周期和导航系统跨组件 reset 事务变成可测试的 transport-independent contract，防止旧历元、旧 receiver generation 或旧 reset transaction 的数据重新进入定位链。

非目标：
- 不解析串口/USB 原始 UBX 字节，不实现 u-blox ROS subscriptions。
- 不从 iTOW 单独推断 GPS week、receiver reboot 或 boot identity。
- 不在 GNSS core 中创建 NavSatFix、Odometry、TF 或 ROS diagnostics。
- 不替代平台安全控制器、急停或底层停车 ACK。
- 不在没有许可/维护者元数据确认前提交正式 coordinator 实现。

## 2. 五种时间/epoch 身份不可混用

| 标识 | 所属生命周期 | 递增/变化条件 | 不应由什么触发 |
|---|---|---|---|
| gnss_receiver_generation | 接收机启动/时钟身份 | 有可信 reboot/session identity 变化 | 不能仅由 iTOW 回绕或 GPS week rollover 触发 |
| estimator_epoch | Point-LIO state/map 生命周期 | LIO reset、重新初始化或坐标原点不连续 | 不能由 RTK FIX→FLOAT 触发 |
| local_odom_epoch | local EKF 的连续里程计坐标系 | local odom reset/原点跳变 | 不能由 GNSS 单独降级触发 |
| map_epoch | global datum/map frame 生命周期 | datum/origin/地图坐标解释变化 | 不能由单次 GNSS dropout 触发 |
| reset_transaction_id | 一次跨组件事务 | 每个新事务唯一分配 | 不得复用；不得用作传感器测量时间 |

每个 epoch 应与稳定的 boot/session identity 关联；进程重启后不能只靠从零开始的本地计数器判断旧消息是否有效。

## 3. ReceiverClockEvidence 与 generation 决策

### 3.1 输入证据模型

纯算法层输入不可变 ReceiverClockEvidence，建议字段：

- receiver_id：已配置设备身份；缺失时不得跨设备复用 generation。
- session_id：若驱动/设备协议提供可信 session 或 boot identity，则填入；否则明确为 unavailable。
- gps_week、i_tow_ms：已解码并通过范围校验的接收机时间。
- receiver_uptime_ms：仅当来源/语义经协议审计后可用；不能把 NAV-STATUS 的易回绕计数器直接当永久 boot ID。
- host_receive_monotonic_ns：本机同一 steady clock 域采集。
- source_timestamp_ns：上游 mapper 给出的绝对接收机时间；尚未建立可信映射时为空。
- evidence_flags：协议明确提供的 reset/session/clock-valid 标志及其来源。
- previous_anchor_id：本次判断所依据的上一个已接受时间锚点。

所有字段必须区分 unavailable、invalid 和 valid；禁止用 0 代表未知。

### 3.2 决策枚举

GenerationDecision 只允许三种结果：

1. CONTINUOUS：同一 receiver session，GPS 时间严格向前；包括正常同周推进和经证据确认的连续 GPS week rollover。
2. NEW_GENERATION：有可信 receiver reboot/session identity 变化证据，或经验证的时钟重新初始化证据；分配新的 generation ID。
3. AMBIGUOUS：week/boot identity 不完整、时间不连续但缺少 reboot 证据、source mapping 冲突或 host/source 时钟无法关联。此时必须关闭 GNSS measurement validity。

### 3.3 判定规则

- iTOW 范围必须满足 0 <= iTOW < 604800000 ms。
- 同周连续推进：GPS week 不变且绝对 GPS time 严格递增；仍需检查最大允许观测间隔。
- 周回绕：GPS week 恰好增加 1、iTOW 从周末回到周初、绝对 GPS 时间连续且 boot/session identity 不变；generation 不变，source timestamp 必须保持严格递增。
- 重启：优先要求可信 session/boot identity 改变；若硬件不提供该字段，必须由已审计的多证据组合确认，不能只凭 iTOW 下降。
- GPS week 倒退、多周跳变、source timestamp 倒退、相同时间戳冲突、host receive time 倒退，均输出 AMBIGUOUS，除非另有明确协议证据。
- 阈值（最大时间间隔、周边界容忍、host/source mapping 误差）均为配置项，必须由真实接收机日志标定；不在设计文档中填写猜测值。

**关键约束：** source_clock_id 标识一个连续 receiver generation，而不是简单的 GPS week + generation 拼接值。连续 week rollover 不应改变 Joiner generation，否则会无必要地清空缓存并把 Gate recovery 重新置零。若现有驱动适配只能提供周身份、无法提供跨周连续映射，则先输出 AMBIGUOUS 并停止 GNSS 输入，不可伪装为连续。

## 4. GnssGenerationController 状态机

建议状态：
- UNINITIALIZED：尚无可信 receiver time anchor。
- ACTIVE：generation 和 source time mapping 已建立。
- AMBIGUOUS_INVALID：GNSS 输出被 fail-closed 隔离，等待新证据。
- REBOOT_PENDING：发现可能 reboot，但证据尚未达到确认条件。

输入：ReceiverClockEvidence。
输出：不可变 GenerationAssessment，至少包含 decision、current/proposed generation ID、source timestamp、reason code、evidence references、validity 和 anchor revision。

状态转移：
- UNINITIALIZED + 有效首锚点 → ACTIVE（建立首 generation）。
- ACTIVE + CONTINUOUS → ACTIVE（更新 anchor，不重置 Joiner/Gate）。
- ACTIVE + NEW_GENERATION → REBOOT_PENDING → reset transaction 成功且 mapper 建立新锚点后 ACTIVE。
- 任意状态 + AMBIGUOUS → AMBIGUOUS_INVALID，先关闭 GNSS 输出。
- AMBIGUOUS_INVALID 不能仅靠清缓存恢复；必须重新建立可信时间锚点。
- 同一 evidence/anchor revision 重复输入必须幂等，不得重复递增 generation。

generation ID 应由 receiver identity + persistent session identity 或 coordinator 持久化 generation sequence 构成；若无法保证跨进程唯一性，进程重启时必须将其标记为新 session 并重新确认，不能复用旧 ID。

## 5. ResetCoordinator 事务合同

### 5.1 输入/输出类型

- ResetTrigger：trigger_id、trigger_type、evidence、affected_components、requested_epoch_changes、source identity。
- ResetPlan：reset_transaction_id、expected current epoch tuple、proposed epoch tuple、ordered phases、required ACK set、deadline policy、invalidation scope。
- ComponentResetAck：transaction ID、component ID、component boot/session ID、component generation、phase、success/failure、observed epoch tuple、details。
- ValiditySnapshot：各数据源 validity、measurement age、epoch tuple、clock domain、health reason。
- ResetOutcome：COMMITTED 或 ABORTED_FAIL_CLOSED、实际 epoch tuple、ACK evidence、失败原因。

这些对象必须 immutable；不把 ROS message 类型放入 core。

### 5.2 状态与顺序

DETECTED → QUIESCING → INVALIDATING → DRAINING → RESETTING_COMPONENTS → REBASING → WARMING_UP → COMMITTING → COMMITTED

任何 phase 失败、超时、重复 ACK 冲突、component boot identity 改变或 epoch 不匹配：
ABORTED_FAIL_CLOSED。

执行规则：
1. 单一 coordinator actor/串行事件队列是 epoch 与 transaction 状态的唯一写入者。
2. 首先撤销受影响 validity，并关闭对应数据发布/消费入口；不能等 reset 完成后才 invalid。
3. quiesce producers，drain callbacks/queues，再 reset 对应组件。
4. 每个 ACK 必须绑定 transaction ID、component boot identity、component generation 和目标 phase。
5. 重复相同 ACK 幂等；同一 component/phase 的冲突 ACK 使事务失败。
6. ACK 不能证明物理停车；平台安全控制器的 stop request/stop confirmation 是独立接口和独立 Gate。
7. 只有所有 required ACK、epoch fence、warm-up 条件满足，才 commit 新 epoch 并重新开放 validity。
8. 事务中途进程重启时，恢复为 fail-closed；不得假设内存中 transaction 已完成。

### 5.3 组件 ownership

| 组件 | Reset owner | Reset 不应隐式触及 |
|---|---|---|
| UBX joiner / RTK gate / receiver mapper | GNSS coordinator | LIO/local odom |
| Point-LIO state/map/queues | LIO lifecycle supervisor | GNSS receiver generation |
| local EKF / odom TF | local state-estimation supervisor | global datum |
| global EKF / map→odom | global localization coordinator | local odom origin |
| static/elevation/dynamic map layers | map lifecycle coordinator | receiver clock |
| Nav2 goal/costmap consumers | navigation health supervisor | estimator internal state |
| physical stop/drive enable | platform safety controller | ROS coordinator 的软件 ACK |

### 5.4 GNSS reboot 子事务

1. 原子关闭 NavSatFix 与 quality status 发布门，撤销 GNSS validity。
2. 停止旧 generation callback 入队；等待正在执行的 callback 退出。
3. 调用 joiner.reset()，随后 gate.reset()；Joiner 的 lifetime counters 保留。
4. receiver mapper 判定 reboot/rollover/ambiguous；禁止以 reset 替代证据判定。
5. 仅当 NEW_GENERATION 且新 source-time anchor 有效时调用 joiner.begin_generation(new_id)。
6. 新数据按 exact iTOW 完成四路配对；每个 completed epoch 只允许 Gate evaluate 一次。
7. ACCEPTED observation 与 quality decision 使用同一 epoch identity 原子发布。
8. global EKF 确认旧 receiver generation 的 queue/history 已隔离，且 map→odom 重新对齐后，才恢复 global localization validity。
9. 任一 ACK/mapper/epoch 检查失败，保持 GNSS invalid；不自动重置 local odom。

## 6. EpochFence 数据消费规则

每个跨组件数据 envelope 至少带：
- producer boot/session identity；
- producer generation；
- 相关 epoch tuple（只填适用项，不使用单一全局 epoch 替代）；
- reset transaction generation / committed transaction ID；
- source measurement timestamp + clock ID；
- ROS timestamp + ROS clock epoch（若经过 ROS adapter）；
- host steady receive timestamp；
- validity/quality/covariance state。

Fence 按消费者要求校验：
- GNSS measurement：receiver generation、GPS-time mapping、quality decision、host freshness。
- LIO odometry/cloud：estimator epoch、measurement time、tracking/covariance validity。
- local odometry/TF：local_odom_epoch、同一 measurement time、唯一 TF authority。
- map/costmap observation：map/local odom epoch、原始观测 freshness、TF transform epoch。
- motion intent：local/map epoch、transaction generation、command expiry；旧 epoch 命令必须拒绝，stop 请求优先。
- reset 尚未 COMMITTED 时，除安全停车/状态 ACK 外，消费者不得接受 proposed epoch 数据。

## 7. 确定性测试矩阵

| 测试 | 输入/故障注入 | 必须满足 |
|---|---|---|
| 首次锚定 | 合法 week/iTOW/session | 建立 generation；不伪造 source time |
| 正常同周推进 | week 不变、GPS time 递增 | generation 不变，source timestamp 严格递增 |
| 连续 week rollover | week +1、iTOW 回绕、session 不变 | generation 不变；Joiner 不 reset |
| iTOW 回退但 week 缺失 | 无 boot evidence | AMBIGUOUS；GNSS invalid |
| 可信 reboot | session identity 变化 | generation 仅递增一次；进入 reset transaction |
| source timestamp 冲突 | 同 epoch 不同映射 | 拒绝并 invalid，不回退到 host time |
| duplicate evidence | 同一 anchor revision 重放 | 幂等，不重复递增 |
| old callback race | reset 后注入旧 generation callback | EpochFence 拒绝；不污染新 pending cache |
| duplicate/conflicting ACK | 相同 ACK 重放/冲突 ACK | 相同幂等；冲突 fail-closed |
| ACK timeout | 任一 required component 不响应 | ABORTED_FAIL_CLOSED，validity 不恢复 |
| process restart mid-transaction | 持久化状态恢复 | 默认 invalid；重新核验 component identity |
| GNSS downgrade only | FIXED→FLOAT | 仅撤销 GNSS validity，不 reset LIO/local odom |
| local odom reset | local epoch +1 | 旧 TF/command/map observation 被 fence |
| map datum change | map epoch +1 | 旧 map goal/layer 不得继续消费 |
| clock rewind | ROS clock epoch 改变 | flush ROS-time queues/TF；不能依赖自动参数 |
| stop separation | software ACK 成功但 platform stop ACK 缺失 | 不得宣称物理停车确认 |

所有时间阈值、恢复样本数和 ACK deadline 均使用显式配置；单元测试可使用合成数值，但生产值必须由设备/平台数据标定。

## 8. 实施前 Gate 与当前证据

- [ ] 用户/维护者确认 GNSS core 的 SPDX license。
- [ ] 用户/维护者提供真实 maintainer 名称和邮箱，替换示例占位信息。
- [ ] 固定并审计实际 ZED-F9P ROS driver/message package 的 commit、license、Jazzy/Humble 支持和原始 UBX 字段。
- [ ] 以真实 receiver log 验证 GPS week、iTOW、boot/session、NAV-STATUS uptime、USB reconnect 和 ROS/steady clock 映射。
- [ ] 确认跨进程 generation ID 的持久化/唯一性策略。
- [ ] 确认各组件 reset API、ACK 可观测性、global EKF 历史隔离和平台 stop ACK。
- [ ] 将上述纯算法测试接入 Humble/Jazzy CI，再做 bag、HIL 和硬件测试。

当前已存在的 Gate/Joiner reset 测试已在 GitHub Actions GNSS quality gate CI 的 Humble/Jammy 与 Jazzy/Noble jobs 通过（具体 run/commit 记录在根 TaskPlan v0.3.94）。该 CI 只证明当前纯算法包的构建/测试矩阵，不证明本设计已实现，也不证明 Orin、真实接收机或实机通过。

**本设计等级：E3（项目设计合同）。** 上游 API 行为与现有 reset 语义见 10_point_lio_decoupling_and_state_interface_audit.md §16–23；真实设备时间映射和跨组件 reset 仍未实现/验证。
