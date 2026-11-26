# ZED-F9P NAV-TIMEGPS：UTC→GPS Mapper 的可选同历元交叉核验

- 日期：2026-11-26
- 状态：协议源码审计与设计合同完成；未实现 parser/driver patch，未运行测试
- 专项分支：`nav`
- 固定协议基线：u-blox ZED-F9P Interface Description `UBX-18010854 R07`，protocol 27.11
- 关联文档：
  - [13 — UTC/GPS 时间映射与固定驱动扩展审计](13_zed_f9p_utc_gps_time_mapping_audit.md)
  - [14 — UTC→GPS Mapper 补丁计划与纯算法测试向量](14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md)
  - [10 — 状态估计接口审计](10_point_lio_decoupling_and_state_interface_audit.md)

## 1. 本轮审计结论

R07 §5.14.21 定义了 `UBX-NAV-TIMEGPS`（class/id `01/20`），payload 固定为 16 bytes，报告最近 navigation solution 的 GPS time、week、GPS−UTC leap seconds 和 time accuracy。它适合作为 UTC→GPS mapper 的**可选同历元交叉核验源**，但不能替代 NAV-PVT / NAV-TIMEUTC / NAV-TIMELS / NAV-STATUS 的准入合同，也不能绕过 NAV-TIMELS 当前 leap-second source policy。

关键字段：

| Offset | Wire type | Field | Unit / semantics |
|---:|---|---|---|
| 0 | U4 | iTOW | GPS time of week，ms |
| 4 | I4 | fTOW | iTOW 的 signed fractional part，ns；协议范围 ±500000 |
| 8 | I2 | week | GPS week number |
| 10 | I1 | leapS | GPS−UTC leap seconds，s |
| 11 | X1 | valid | bit 0 towValid、bit 1 weekValid、bit 2 leapSValid |
| 12 | U4 | tAcc | time accuracy estimate，ns |

R07 明确：
- `towValid` 表示 iTOW 与 fTOW 有效；
- `weekValid` 表示 GPS week 有效；
- `leapSValid` 表示 GPS leap seconds 有效；
- 三个位独立，不能把其中一个有效推断成另外两个也有效；
- 精确 TOW 为 `iTOW × 10^-3 + fTOW × 10^-9` 秒。

## 2. 在 Mapper 中的职责边界

### 2.1 必须维持的权威输入链

主路径保持 doc/Navigation/14 §4.12 的职责划分：

1. NAV-PVT：UTC civil 主候选及 validity/confirmation。
2. NAV-TIMEUTC：同 iTOW UTC 独立一致性核验与 utcStandard policy。
3. NAV-TIMELS：同 iTOW 当前 GPS−UTC offset 候选，必须通过 validCurrLs/source/freshness policy。
4. NAV-STATUS：fix 与 tow/week validity gate。
5. NAV-TIMEGPS：可选 GPS week/TOW/leapS 交叉核验，不作为前四项的替代输入。

NAV-TIMEGPS 是同一接收机的另一条输出，不是独立外部时间标准。即使它与 NAV-TIMELS 一致，也不能证明接收机 leap-second source 独立、正确或已获项目策略批准；不能用 TIMEGPS 的 leapS 绕过 `srcOfCurrLs=0/default`、255/unknown 或未批准 source 的拒绝规则。

### 2.2 Exact epoch 与 identity

TIMEGPS 记录必须携带与其他 UBX 记录相同的：
`process_session_id`、`connection_generation`、`queue_sequence`、`receive_monotonic_ns` 和原始 `iTOW`。

只有同一 session/generation 且完全相同 iTOW 的记录才可进行 cross-check。禁止 nearest-neighbor、接收时间最近、ROS header stamp 或仅凭 week/TOW 数值跨 generation 配对。TIMEGPS 的到达顺序可以不同，但旧 generation/旧 sequence 不得污染当前 mapper。

### 2.3 GPS 时间整数重建

保留原始 wire 字段，并以整数纳秒构造：

`raw_tow_ns = iTOW_ms × 1,000,000 + fTOW_ns`

`raw_absolute_gps_ns = week × WEEK_NS + raw_tow_ns`

随后以整数 `divmod` 规范化周边界，保留规范化前的 raw week/iTOW/fTOW 和规范化后的 week/TOW。不得丢弃负 fTOW，也不得用 float seconds。若 `towValid=1` 而 iTOW 超出协议周范围，或 fTOW 超出 ±500000 ns，记录结构/语义错误；不得通过归一化掩盖非法 wire 值。

由 UTC+NAV-TIMELS 计算出的 mapper GPS absolute ns，与 TIMEGPS 重建值比较时，必须采用单独批准的比较策略。不能要求两者简单逐纳秒相等，也不能自行选取容差。PVT/TIMEUTC/TIMEGPS 的 tAcc 均是接收机时间精度估计，误差相关性和统计组合方式未证明；禁止直接相加、RSS 合成或把某一 tAcc 当成完整系统误差上限。

### 2.4 Validity 组合

| towValid | weekValid | leapSValid | 允许的用途 |
|---:|---:|---:|---|
| 0 | 任意 | 任意 | 不使用 TIMEGPS TOW |
| 1 | 0 | 任意 | 仅诊断周内 TOW；不构成绝对 GPS 时间 |
| 0 | 1 | 任意 | 仅诊断 week；不构成绝对 GPS 时间 |
| 1 | 1 | 0 | 可进行 week/TOW 交叉核验；不做 leapS 核验 |
| 1 | 1 | 1 | 可进行 week/TOW 与 leapS 三项交叉核验 |
| 任意 | 任意 | 1 | leapS 只作为一致性观察；不能单独生成 mapper offset |

`weekValid` 与 `towValid` 同时为 true，仍不意味着 NAV-TIMEGPS 可单独使 mapper 进入 VALID。mapper 主路径中 PVT/TIMEUTC/TIMELS/STATUS、source policy、freshness、uncertainty、generation fence 等条件仍必须全部通过。

## 3. 冲突处理

- TIMEGPS week 与 mapper 推导 week 冲突：该 epoch 标记 `AMBIGUOUS_INVALID`，记录 `TIMEGPS_WEEK_MISMATCH`。
- TIMEGPS TOW 与 mapper 推导 TOW 超出已批准比较策略：标记 `AMBIGUOUS_INVALID`，记录 `TIMEGPS_TOW_MISMATCH`。
- `leapSValid=1` 且 TIMEGPS.leapS 与 NAV-TIMELS.currLs 不一致：标记 `AMBIGUOUS_INVALID`，记录 `TIMEGPS_LEAPS_MISMATCH`；不得选择“较新”或“更可信”的一方静默覆盖。
- TIMEGPS 缺失、未启用或相应 validity bit 为 false：按可选交叉核验缺失处理；不能因此将本来无效的主路径提升为有效。
- TIMEGPS 与 NAV-TIMELS 一致：只记录内部一致性证据，不提升 source trust level，不替代 IERS/USNO transition artifact，也不证明 GNSS receiver 的绝对时间精度。

## 4. 确定性测试合同（未实现）

| ID | 场景 | 预期 |
|---|---|---|
| TIMEGPS-RAW-001 | 16-byte payload，所有字段合法，valid=0x07 | 解析 iTOW/fTOW/week/leapS/tAcc 与三个 validity flags |
| TIMEGPS-RAW-002 | payload 长度 0/15/17 | 严格拒绝；只接受 16 bytes |
| TIMEGPS-RAW-003 | fTOW=-500000 / +500000 | signed I4 保真并接受边界 |
| TIMEGPS-RAW-004 | fTOW=-500001 / +500001 | 超出协议范围，拒绝或标记 malformed；不得静默归一化 |
| TIMEGPS-RAW-005 | valid bit 0/1/2 独立翻转 | 只改变对应字段可用性，不推导其他位 |
| TIMEGPS-RAW-006 | week 为 signed I2 边界值 | 保留 signed wire semantics；是否支持该周数由 epoch policy 决定 |
| TIMEGPS-JOIN-001 | TIMEGPS 与四路主输入同 session/generation/iTOW | 可执行可选 cross-check |
| TIMEGPS-JOIN-002 | iTOW 相差 1 ms | 不配对 |
| TIMEGPS-JOIN-003 | iTOW 相同但 generation 不同 | 拒绝跨 generation 配对 |
| TIMEGPS-MATH-001 | fTOW 为负，TOW 规范化跨周边界 | GPS absolute ns 不变；week/TOW 规范化正确 |
| TIMEGPS-MATH-002 | iTOW×1 ms + fTOW 与独立整数 oracle 比较 | 无 float、无精度丢失 |
| TIMEGPS-CHECK-001 | TIMEGPS week 与 UTC+TIMELS 推导 week 冲突 | AMBIGUOUS_INVALID |
| TIMEGPS-CHECK-002 | TIMEGPS TOW 与 UTC+TIMELS 推导 TOW 不一致 | 按冻结 tolerance fail-closed |
| TIMEGPS-CHECK-003 | TIMEGPS leapS 与 NAV-TIMELS.currLs 冲突 | AMBIGUOUS_INVALID，不允许覆盖 |
| TIMEGPS-CHECK-004 | leapSValid=false | 不执行 leapS 比较，不因此绕过主路径 |
| TIMEGPS-CHECK-005 | TIMEGPS 完全缺失 | 只失去可选 cross-check，不改变主路径有效性 |
| TIMEGPS-ACC-001 | TIMEGPS tAcc 与 PVT/TIMEUTC tAcc 不同 | 保留各自来源；未批准误差模型前不自行合成 |

## 5. 固定驱动集成前置核验

R07 证明协议定义了该消息，但不能证明固定候选驱动 `aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e` 已解析、发布或保留其 raw validity byte。后续 driver audit 必须逐项检查：
- payload exact length=16 与 class/id `01/20`；
- parser signedness/offset；
- UBX frame dispatch/container；
- ROSIDL message、publisher 和输出配置；
- raw valid byte/reserved bits 的可观测性；
- callback/executor/thread safety 与 ingress identity；
- 目标 ZED-F9P SKU、MON-VER、PROTVER 和固件实测。

不得仅因 R07 有定义就假定目标 F9P 固件/驱动输出 TIMEGPS；也不得将其他 u-blox 产品/协议版本的支持范围直接外推到目标板。

## 6. 证据等级与限制

- 协议字段、payload 长度、validity 位和 fTOW 语义：固定 R07 文档 E1/E2 规格证据。
- 本文的 exact-epoch join、状态策略、冲突处理与测试矩阵：E3 项目设计合同。
- 当前没有 TIMEGPS 目标设备原始帧、MON-VER、真实输出频率/配置、固定驱动实现审计结论、parser、ROSIDL、测试或 Orin 运行证据。
- R07 为 2019-07-10 Early Production Information；目标固件协议版本及后续修订差异仍需实际 MON-VER 与适用的正式文档确认。

## 7. 参考来源

1. u-blox, *ZED-F9P Interface Description*, UBX-18010854 R07, protocol 27.11, §5.14.21, pp.171–172. 本轮可读取副本：[PDF mirror](https://littlebirdelectronics.com.au/rails/active_storage/blobs/proxy/BAh7BkkiC19yYWlscwY6BkVUewdJIglkYXRhBjsAVGkDyQABSSIIcHVyBjsAVEkiDGJsb2JfaWQGOwBG--05a26b778874e05b88c26ad08afef7e2961fb850/PM-15136.pdf)。文档封面标识为 u-blox 原始文件号/修订号；镜像托管不等于 u-blox 官方分发链。
2. u-blox, *ZED-F9P Integration Manual*, UBX-18010802 R16：[官方 PDF](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)。用于后续核验 iTOW/receiver time validity 与配置语义；本文不把它作为 TIMEGPS wire layout 的唯一来源。

