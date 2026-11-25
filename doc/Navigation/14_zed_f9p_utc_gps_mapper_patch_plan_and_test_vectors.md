# ZED-F9P UTC→GPS Mapper：驱动补丁设计与纯算法测试向量

- 状态：设计审查稿（未实现）
- 对应分支：`nav`
- 上游驱动基线：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`（tag 0.7.6）
- 协议参考：ZED-F9P Interface Description UBX-18010854 R07 / protocol 27.11；目标设备固件版本尚未取得
- 关联审计：[13 — UTC/GPS 时间映射与固定驱动扩展审计](13_zed_f9p_utc_gps_time_mapping_audit.md)
- 证据等级：固定上游源码/协议文档 E1/E2；本文件中的接口和测试向量属于设计/合成测试规格，不是运行证据

## 1. 目的与不可越过的边界

本文件将 §13 的结论细化为可拆分、可代码审查的 patch plan 和 deterministic test vectors。目标是为已有 `robodog_gnss_quality` 纯算法核心增加可信的 UTC→GPS source-time mapping，并为固定候选驱动定义最小消息/lifecycle 扩展。

本阶段**不修改**上游 `aussierobots/ublox_dgnss`，不修改 GNSS package metadata，不新增 ROS adapter，不声称已运行测试。实现前置条件：
- 用户/项目负责人确认 `robodog_gnss_quality` 的 SPDX license 和 maintainer；当前 `UNSPECIFIED` / placeholder 仍是工程 Gate。
- 取得目标 ZED-F9P 的完整 MON-VER 输出（software version、hardware version、PROTVER），确认设备实际协议版本。
- 通过目标固件验证 NAV-PVT、NAV-TIMEUTC、NAV-TIMELS、NAV-STATUS 的输出可用性、输出率和配置持久性。
- 冻结 leap-second source allowlist、staleness policy、maximum receive skew、time accuracy threshold 和 recovery hysteresis；本文件不替项目臆造数值阈值。

禁止把 UTC validity、GNSS measurement quality、receiver generation、driver connection generation 或 reset transaction 混成一个“时间有效”布尔值。

## 2. 建议的软件分层与接口

### 2.1 Pure core（无 ROS / 无硬件依赖）

建议在现有 `src/robodog_gnss_quality/robodog_gnss_quality/` 内新增独立模块；实现命名可在代码评审时调整，但职责不可合并：

| 模块 | 职责 | 输入 | 输出 |
|---|---|---|---|
| `utc_calendar.py` | 严格校验 UTC civil fields、signed nano 规范化、leap-second 表达 | 年月日时分秒、nano | normalized UTC civil instant / invalid |
| `ubx_nav_timels.py` | 解码 NAV-TIMELS 语义与 validity/source | raw parsed fields | typed leap-second candidate |
| `gps_time.py` | UTC civil instant + 已验证 GPS−UTC offset → GPS absolute ns、week、TOW | UTC、offset | integer GPS time tuple |
| `utc_gps_mapper.py` | PVT/TIMEUTC/TIMELS/STATUS exact-epoch join、confidence、monotonicity | immutable epoch records | VALID / UNCONFIRMED / AMBIGUOUS_INVALID |
| `clock_identity.py` | source clock mapping identity 与 reset fence 元数据 | process/session/generation | immutable identity |

Pure core 不读取 system wall clock、不调用 ROS clock、不访问 USB；host receive time 由 adapter 传入 monotonic integer ns。所有时间计算用整数，禁止 float seconds 累积。

建议输出结构至少包含：
- `status`：统一采用 §4.12.4 的 `NO_FIX`、`UNCONFIRMED`、`VALID`、`DEGRADED`、`AMBIGUOUS_INVALID`、`STALE`、`RESETTING`；旧草案中的 `VALID_CONFIRMED` 对应 `VALID`，`VALID_UNCONFIRMED` 对应 `UNCONFIRMED`；
- `gps_time_ns`、`gps_week`、`gps_tow_ns`；
- `utc_civil`（保留 leap-second 标识，不强制转换成 POSIX timestamp）；
- `leap_seconds`、`leap_source`、`leap_valid`；
- `source_time_uncertainty_ns`（仅当来源字段可解释时输出；不把接收机 tAcc 与 host transport delay 混为一谈）；
- `process_session_id`、`connection_generation`、`epoch_itow_ms`、`mapping_revision`；
- `reason_codes`（例如 PVT_TIME_INVALID、TIMEUTC_MISMATCH、LEAP_SOURCE_DEFAULT、EPOCH_SESSION_MISMATCH、SOURCE_TIME_REGRESSION）。

`VALID_UNCONFIRMED` 可以供诊断显示，但默认不得向 RTK quality gate 提供可被接受的 source clock，也不得单独触发 NEW_GENERATION。是否允许特定低风险用途，须另立决策。

### 2.2 Driver ROS adapter（后续独立 patch）

固定驱动扩展应拆成可独立审查的 commits：
1. **Message definition**：新增 `UBXNavTIMELS.msg`，保留 raw `srcOfCurrLs`、signed `currLs`、`validCurrLs`、`validTimeToLsEvent`、`iTOW`、version 和原始 reserved bytes（如项目 ROS 消息策略允许）。
2. **Parser**：独立 payload parser；验证 UBX class/id、payload length=24、version=0；按协议字段 offset/signedness 解码；保留 raw validity bits；未知版本或长度不符拒绝。
3. **Dispatch/publisher**：在 `ubx_nav.hpp` 完成 message ID、FrameComms、member、constructor、accessor、dispatch；在 node 中完成 publisher 和 publish mapping；CMake 显式登记 ROSIDL message。
4. **Lifecycle contract**：引入 process session 与 connection generation；frame 入队时捕获 identity + queue sequence；attach/detach 与 UBX frame 使用同一个有序 ingress event mechanism。不能只增加一个无序 topic。
5. **Thread safety**：修复或封装 UBX queue 的 size/front/pop 锁语义；跨 callback group 共享状态采用锁保护 snapshot/atomic transaction；测试实际 executor 类型。
6. **Optional NAV-TIMEGPS**：单独 patch，作为 cross-check only；不得让它绕过 NAV-TIMELS 的可信 leap-second 来源 Gate。
7. **Build/test**：parser unit tests、ROSIDL build/install、Humble/Jazzy CI；不得把原 upstream CI 结果算作新 patch 的通过证据。

### 2.3 事件顺序合同

每个 ingress event 必须携带：
`process_session_id, connection_generation, queue_sequence, host_receive_steady_ns`。

- 新进程启动：新的 process_session_id。
- USB attach：创建新 connection_generation；不能据此恢复 GNSS time validity。
- USB detach：在同一 ingress sequence 中插入 invalidation barrier；立即使该 generation 失效。
- frame：入队时捕获当时 generation；消费者仅接受与当前 session/generation 相符且位于有效 attach barrier 之后的 frame。
- sequence gap、queue overflow、身份缺失、session mismatch：fail-closed，清除待配对 epoch。
- 当前 lifecycle snapshot 必须包含最后 event sequence，供重启 subscriber 对齐 event stream。
- receiver reboot 需要 reset evidence 与显式 generation transaction；GPS week rollover 不得被误判为 reboot。

## 2.4 Pure-core public API proposal（与现有包风格对齐）

现有包的 `ubx_nav_pvt.py` 采用 frozen dataclass + 明确异常（`EpochPairingError`）+ 无 ROS 依赖函数；新 mapper 应沿用该风格，不另建隐式全局状态。

建议对外 API（签名草案，非已实现代码）：

```python
@dataclass(frozen=True)
class UbxNavTimelsEpoch:
    i_tow_ms: int
    version: int
    src_of_curr_ls: int
    curr_ls: int                 # signed int8
    src_of_ls_change: int
    ls_change: int               # signed int8
    time_to_ls_event_s: int      # signed int32
    date_of_ls_gps_week: int
    date_of_ls_gps_day: int
    valid_curr_ls: bool
    valid_time_to_ls_event: bool
    receive_monotonic_ns: int
    process_session_id: str
    connection_generation: int

@dataclass(frozen=True)
class UtcCivilTime:
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: int                 # 0..60; 60 is explicit leap-second state
    nano: int                   # signed protocol field, normalized by helper

@dataclass(frozen=True)
class GpsTimeMapping:
    gps_week: int
    gps_tow_ns: int
    gps_absolute_ns: int
    status: str
    reason_codes: tuple
    source_clock_id: str
    source_uncertainty_ns: int | None
```

函数边界建议：
- `decode_nav_timels_payload(payload: bytes, *, receive_monotonic_ns, process_session_id, connection_generation) -> UbxNavTimelsEpoch`：只解码/校验协议结构，不自行决定 source 是否可信。
- `normalize_utc_civil(fields: UtcCivilTime) -> NormalizedUtcInstant`：校验公历日期、nano 借位和 leap-second 规则；不使用 POSIX timestamp。
- `gps_week_tow_from_utc(utc, gps_minus_utc_seconds) -> (week, tow_ns, absolute_ns)`：纯整数算术，调用方必须提供已通过策略 Gate 的 offset。
- `map_utc_epoch(pvt, timeutc, timels, status, identity, policy) -> GpsTimeMapping`：做 exact iTOW/session join、validity/source/confidence 和 monotonicity。
- 所有输入记录应 frozen/immutable；mapper 不修改传入 record，不用最近到达时间补齐缺失字段。

异常策略：结构损坏、类型越界和 payload 长度错误抛出明确 decode exception；合法但无效的接收机状态返回 AMBIGUOUS_INVALID + reason code；不得把设备状态错误伪装成 parser exception，也不得返回看似有效的零值。

## 3. 关键算法约束

1. NAV-PVT 为主 UTC candidate；要求 `valid_date && valid_time && fully_resolved`。
2. `confirmed_avail=true` 时，`confirmed_date && confirmed_time` 必须均为 true；否则拒绝该 confirmed anchor。若 `confirmed_avail=false`，只可进入 UNCONFIRMED class。
3. NAV-TIMEUTC 必须 `valid_utc=true`，`utc_std` 必须属于经项目冻结的允许集合；同 iTOW 的 PVT/TIMEUTC 经各自 nano 规范化后必须一致。
4. NAV-TIMELS 必须 `validCurrLs=true`。source=0（firmware default）和 255（unknown）拒绝作为可信当前 offset；其他 source 暂不默认全白名单，需固件/项目策略批准。
5. NAV-TIMELS 的 `validTimeToLsEvent` 只描述未来/最近 leap event 时间字段，不替代 `validCurrLs`。
6. `currLs` 是有符号秒数；不硬编码“永远 18 秒”，也不以主机当前日期推断。
7. NAV-STATUS `tow_set/wkn_set` 和 TIMEUTC `valid_tow/valid_wkn` 是 validity flags，不包含 week number 数值。
8. 同一 receiver session 中按精确 iTOW join；禁止 nearest-neighbor 复用旧 TIMEUTC/TIMELS。所有缓存有界并按 host steady time 过期。
9. GPS absolute time 使用 GPS epoch 1980-01-06 和 integer arithmetic；完整 week/TOW 从 UTC + 当前已验证 offset 计算，再与 iTOW / optional TIMEGPS 做 consistency check。
10. UTC leap second `sec=60` 必须由显式 leap-second-aware civil representation 处理；普通 POSIX/标准库归一化不能静默吞掉该秒。
11. 任一输入来源/有效性/epoch/session/monotonicity 不确定时输出 AMBIGUOUS_INVALID，不能回退到 ROS header stamp、system wall clock 或 iTOW 单独构造绝对时间。

## 4. Deterministic synthetic test vectors

以下向量是纯算法规格，所有 leap offset 都是**显式 fixture 输入**，不是对目标设备当前状态的断言。测试必须同时检查输出值和状态/reason code。

### 4.1 GPS epoch 与整数周边界

GPS epoch：1980-01-06 00:00:00 UTC，GPS week 长度 604800 s。GPS-001 是纯算法 epoch 单测，不是 NAV-PVT 可输入范围测试；R07 的 NAV-TIMEUTC UTC 年份字段范围为 1999–2099。

| ID | UTC civil input | fixture GPS−UTC | Expected GPS time | Expected week / TOW |
|---|---|---:|---|---|
| GPS-001 | 1980-01-06 00:00:00.000000000 | 0 s | 1980-01-06 00:00:00 GPS | week 0 / 0 ns |
| GPS-002 | 2024-01-07 00:00:00.000000000 | 18 s | 2024-01-07 00:00:18 GPS | week 2296 / 18,000,000,000 ns |
| GPS-003 | 2024-01-07 00:00:01.000000000 | 18 s | 2024-01-07 00:00:19 GPS | week 2296 / 19,000,000,000 ns |
| GPS-004 | 2024-01-08 00:00:00.000000000 | 18 s | 2024-01-08 00:00:18 GPS | week 2296 / 86,418,000,000 ns |

必须增加边界参数化测试：GPS week 的 TOW 恰为 604800 s 前 1 ns、恰好 604800 s、后 1 ns；输出应分别落在前一周末、下一周起点、下一周起点 +1 ns，不允许 TOW 等于一整周。

### 4.2 signed nano 与 civil calendar 借位

| ID | UTC base fields | nano | Expected normalized UTC |
|---|---|---:|---|
| UTC-001 | 2024-01-01 00:00:00 | -1 | 2023-12-31 23:59:59.999999999 |
| UTC-002 | 2024-03-01 00:00:00 | -1 | 2024-02-29 23:59:59.999999999 |
| UTC-003 | 2025-01-01 00:00:00 | -1 | 2024-12-31 23:59:59.999999999 |
| UTC-004 | 2024-12-31 23:59:59 | +1,000,000,000 | 2025-01-01 00:00:00.000000000 |
| UTC-005 | 2024-01-01 00:00:00 | -1,000,000,000 | 2023-12-31 23:59:59.000000000 |

另需验证 nano 的合法协议范围、超范围拒绝、闰年/非闰年 2 月、世纪年规则（2000 闰年、2100 非闰年）、负数整秒归一化和整数溢出防护。

### 4.3 正闰秒连续性

使用 2016-12-31 正闰秒作为历史合成 fixture：
- 23:59:59 使用 GPS−UTC=17 s → GPS 2017-01-01 00:00:16；
- 23:59:60 使用 GPS−UTC=17 s → GPS 2017-01-01 00:00:17；
- 2017-01-01 00:00:00 使用 GPS−UTC=18 s → GPS 2017-01-01 00:00:18。

期望三个 GPS instants 连续且相邻 1 s。若 parser/civil type 无法表示 `sec=60`，必须明确返回 UNSUPPORTED_LEAP_SECOND / AMBIGUOUS_INVALID；不得把 23:59:60 映射成 00:00:00 而与下一条重复。

### 4.4 NAV-TIMELS validity/source

| ID | validCurrLs | srcOfCurrLs | Expected |
|---|---:|---:|---|
| LS-001 | false | 2 (GPS) | reject offset |
| LS-002 | true | 0 (default) | reject as potentially stale |
| LS-003 | true | 255 (unknown) | reject |
| LS-004 | true | 2 (GPS) | candidate only if target firmware/source policy allows |
| LS-005 | true | 6 (aided) | candidate only if aiding provenance/freshness policy allows |
| LS-006 | true | 7 (configured) | candidate only if configured value provenance is audited |
| LS-007 | true | 2, validTimeToLsEvent=false | current offset may remain valid; future-event fields invalid |
| LS-008 | true | 2, currLs signed negative | parser preserves signed I1; mapper applies only if policy permits and consistency passes |

### 4.5 Exact epoch / lifecycle / monotonicity

| ID | Setup | Expected |
|---|---|---|
| EPOCH-001 | PVT/TIMEUTC/TIMELS same session + same iTOW | exact join candidate |
| EPOCH-002 | TIMEUTC iTOW differs by 1 ms | no nearest join; epoch incomplete/invalid |
| EPOCH-003 | same iTOW but different connection_generation | reject cross-generation mix |
| EPOCH-004 | old frame queued before detach, consumed after detach barrier | drop old frame |
| EPOCH-005 | attach event then first frame, no valid UTC anchor | connection active, GNSS time invalid |
| EPOCH-006 | sequence gap or ingress queue overflow | invalidate session and flush pending joins |
| EPOCH-007 | process_session_id changes | old cache rejected; new anchor required |
| EPOCH-008 | same session GPS week rollover with continuous GPS absolute time | no receiver generation increment |
| EPOCH-009 | same iTOW repeated with conflicting UTC fields | mark conflict; do not overwrite silently |
| EPOCH-010 | source time regresses without recognized wrap/reset evidence | AMBIGUOUS_INVALID |
| EPOCH-011 | host steady receive age exceeds configured freshness | reject; no system wall-clock fallback |
| EPOCH-012 | TIMEGPS optional cross-check disagrees with computed week/TOW | AMBIGUOUS_INVALID + diagnostic |

### 4.6 NAV-PVT / NAV-TIMEUTC validity combinations

| ID | Condition | Expected |
|---|---|---|
| VALID-001 | PVT valid_date/time/fully_resolved all true; confirmed_avail=true; both confirmed flags true | confirmed UTC candidate |
| VALID-002 | PVT valid flags true; confirmed_avail=false | UNCONFIRMED only |
| VALID-003 | confirmed_avail=true; confirmed_date=false | reject |
| VALID-004 | confirmed_avail=true; confirmed_time=false | reject |
| VALID-005 | fully_resolved=false | reject precise absolute mapping |
| VALID-006 | TIMEUTC valid_utc=false | reject |
| VALID-007 | utc_std unavailable/unknown/not allowlisted | reject |
| VALID-008 | PVT and TIMEUTC normalized UTC disagree | reject |
| VALID-009 | valid_wkn=false but UTC anchor otherwise valid | do not infer week from flag; record validity discrepancy and reject navigation-consumable absolute mapping as AMBIGUOUS_INVALID unless a separately approved policy explicitly changes the required-source contract |
| VALID-010 | NAV-STATUS tow_set/wkn_set false | do not treat iTOW/week status as valid; reject navigation-consumable absolute mapping as AMBIGUOUS_INVALID (gpsFixOK=false is separately classified as NO_FIX) |

### 4.8 NAV-TIMEUTC valid-byte masks

R07 defines the NAV-TIMEUTC payload byte 19 as:
- bit 0 / `0x01`: `validTOW`
- bit 1 / `0x02`: `validWKN`
- bit 2 / `0x04`: `validUTC`
- bit 3 / `0x08`: reserved
- bits 7..4 / `0xF0`: `utcStandard` (upper nibble)

Required independent mask vectors:
- `TIMEUTC-FLAGS-001`: byte=0x00 → no validity flags, utcStandard=0.
- `TIMEUTC-FLAGS-002`: byte=0x01 → validTOW only.
- `TIMEUTC-FLAGS-003`: byte=0x02 → validWKN only.
- `TIMEUTC-FLAGS-004`: byte=0x04 → validUTC only.
- `TIMEUTC-FLAGS-005`: byte=0x08 → reserved bit set; parser preserves/reports reserved-bit violation per strictness policy, never treats it as validUTC.
- `TIMEUTC-FLAGS-006`: byte=0x20 → utcStandard=2 (NIST), no validity flags.
- `TIMEUTC-FLAGS-007`: byte=0xF4 → utcStandard=15 (unknown), validUTC=true; semantic policy must still reject unknown standard.
- `TIMEUTC-FLAGS-008`: byte=0x74 → utcStandard=7 (NTSC), validUTC=true, validWKN=false, validTOW=false.

The bitfield decoder must preserve the raw byte and decode validity independently from the UTC-standard nibble. Whether a standard is accepted belongs to policy, not byte parsing.

### 4.7 Parser byte-level vectors still required

Before driver implementation, add byte-array fixtures copied from protocol field layout and independently reviewed:
- NAV-TIMELS payload length 24; offsets: iTOW 0..3 (U4), version 4 (U1), reserved 5..7, srcOfCurrLs 8 (U1), currLs 9 (I1), srcOfLsChange 10 (U1), lsChange 11 (I1), timeToLsEvent 12..15 (I4), dateOfLsGpsWn 16..17 (U2), dateOfLsGpsDn 18..19 (U2), reserved 20..22, valid 23 (bitfield). The two date fields describe the next/last leap event and are valid only when validTimeToLsEvent is true; they are not current GPS week/time fields.
- Verify little-endian signed decoding for `currLs` and `timeToLsEvent`; `validCurrLs` and `validTimeToLsEvent` bits are distinct.
- Byte fixtures (all unspecified bytes zero; payload offsets are zero-based):
  - `TIMELS-RAW-001`: `00 00 00 00 00 00 00 00 02 12 00 00 00 00 00 00 00 00 00 00 00 00 00 01` → iTOW=0, version=0, srcOfCurrLs=GPS(2), currLs=+18, validCurrLs=true, validTimeToLsEvent=false.
  - `TIMELS-RAW-002`: same bytes with final byte `02` → current offset invalid, event-time fields valid; must not accept currLs as current offset.
  - `TIMELS-RAW-003`: same bytes with final byte `03` → both validity flags true.
  - `TIMELS-RAW-004`: same bytes with final byte `00` → neither validity flag true.
  - `TIMELS-RAW-005`: set byte 9 to `FF` → currLs=-1 (signed I1), not 255.
  - `TIMELS-RAW-006`: set bytes 12..15 to `FF FF FF FF` → timeToLsEvent=-1 (signed I4); only meaningful when bit 1 is set.
  - `TIMELS-RAW-007`: set bytes 16..17 / 18..19 to little-endian nonzero WN/DN values; only expose as valid event date when bit 1 is set.
- Test payload lengths 0, 23, 24, 25; version unknown; reserved bytes nonzero; unknown source code; checksum failure at UBX frame layer. Add dateOfLsGpsWn/dateOfLsGpsDn validity tests gated by validTimeToLsEvent.
- R07 original bitfield graphic confirms: bit 0 = `validCurrLs` (mask `0x01`); bit 1 = `validTimeToLsEvent` (mask `0x02`); bits 2..7 reserved. Thus `valid & 0x01` controls current offset validity and `valid & 0x02` controls event-time/date validity. The flags are independent; do not require both to use a valid current offset. This is confirmed for protocol v27.11 in R07; target firmware compatibility still requires MON-VER.

## 5. Implementation order and review gates

| Patch | Scope | Exit criteria |
|---|---|---|
| P0 | User confirms package license/maintainer; acquire target MON-VER and config snapshot | governance + device capability Gate closed |
| P1 | pure UTC civil/nano arithmetic and GPS epoch/week conversion | all UTC/GPS vectors pass; no ROS dependency |
| P2 | NAV-TIMELS typed model/parser in pure core | byte-level vectors pass; signedness/flags reviewed against target protocol |
| P3 | immutable epoch model + exact join + confidence/reason codes | no nearest join; generation/session fencing tests pass |
| P4 | property/fuzz tests and monotonicity/reset edge cases | bounded memory/runtime; deterministic seeds and failure corpus |
| P5 | driver ROSIDL/parser/dispatch/publisher patch on a dedicated integration fork/branch only after approval | Humble/Jazzy build/install + parser tests pass |
| P6 | lifecycle event queue, sequence fence, thread safety | multithreaded executor and USB stress tests pass; no stale frame crosses detach |
| P7 | target receiver logging and calibration | cold/warm/hot start, reconnect, week rollover, long-run evidence collected |
| P8 | GNSS gate integration and reset coordinator transaction | mapping identity and receiver generation are independently fenced; fault injection passes |

本项目当前仅完成上游静态审计与本设计文档；P0–P8 均未因此自动完成。不得将“测试向量已定义”记录为“测试已通过”。

## 6. 验收日志模板（实施时填写）

每次 mapper test run 至少记录：
- git SHA、Python/runtime version、test command、seed；
- vector ID、input bytes/fields、expected、actual、status/reason code；
- elapsed time、cache size、queue sequence；
- host steady receive timestamps（不可用 wall time 替代）；
- process/session/generation tuple；
- failed vectors 的最小复现输入。

硬件日志另记录 MON-VER、CFG/NAV output config、天线/基站/NTRIP 状态、冷/热启动、USB 事件、receiver reset evidence、原始 UBX frame 和 host monotonic receive time。日志中不得仅保存转换后 ROS header stamp。

## 7. 固定参考

- u-blox ZED-F9P Interface Description UBX-18010854 R07：https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf
- NAV-TIMELS payload / source / validity fields：https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_UBX_NMEA_and_RTCM_protocols.pdf
- 固定候选驱动源码：https://github.com/aussierobots/ublox_dgnss/tree/9d4f5addbc72d092c57dde99b20712258e96e81e
- 相关审计：`13_zed_f9p_utc_gps_time_mapping_audit.md`

### 4.9 NAV-PVT byte-level UTC validity fixtures

R07 NAV-PVT is UBX class/id `01/07`, payload length exactly 92 bytes. The UTC-related fields used by the mapper are:
- offset 0..3: `iTOW`, U4 little-endian;
- offsets 4..10: UTC year (U2 LE), month/day/hour/minute/second (U1); second permits 60 during a leap second;
- offset 11: `valid` bitfield: bit 0 `validDate` (0x01), bit 1 `validTime` (0x02), bit 2 `fullyResolved` (0x04), bit 3 `validMag` (0x08), bits 4..7 reserved;
- offsets 12..15: `tAcc`, U4 LE nanoseconds;
- offsets 16..19: `nano`, signed I4 LE nanoseconds; preserve negative values;
- offset 22: `flags2`: bit 5 `confirmedAvail` (0x20), bit 6 `confirmedDate` (0x40), bit 7 `confirmedTime` (0x80), bits 0..4 reserved.

The payload parser must be structural only: exact length 92, explicit little-endian and signed decoding, preserve raw validity bytes, and never promote validity policy into parsing. A pure mapper applies confirmation policy after decoding. `confirmedAvail=0` means confirmation metadata is unavailable/not asserted and therefore can only yield an UNCONFIRMED candidate; when `confirmedAvail=1`, both `confirmedDate` and `confirmedTime` must be set for a confirmed candidate.

Deterministic fixtures are built from a zero-filled 92-byte payload with the following byte mutations (all offsets zero-based; common valid date fixture sets year=2024 at [4..5] as `E8 07`, month/day/hour/min/sec at [6..10] as `01 01 00 00 00`):
- `PVT-RAW-001`: offset 11 = `0x07`, offset 22 = `0xE0` → date/time/fullyResolved and confirmation available/date/time all true.
- `PVT-RAW-002`: offset 11 = `0x07`, offset 22 = `0x00` → valid UTC fields but no confirmation metadata; mapper returns UNCONFIRMED, never confirmed.
- `PVT-RAW-003`: offset 11 = `0x03` → validDate + validTime, but not fullyResolved; reject precise absolute mapping.
- `PVT-RAW-004`: offset 11 = `0x05` → validDate + fullyResolved, validTime false; reject.
- `PVT-RAW-005`: offset 11 = `0x06` → validTime + fullyResolved, validDate false; reject.
- `PVT-RAW-006`: offset 22 = `0x60` → confirmedAvail + confirmedDate, confirmedTime false; reject confirmed candidate.
- `PVT-RAW-007`: offset 22 = `0xA0` → confirmedAvail + confirmedTime, confirmedDate false; reject confirmed candidate.
- `PVT-RAW-008`: offset 16..19 = `FF FF FF FF` → nano = -1 ns, not a large positive integer.
- `PVT-RAW-009`: offset 10 = `3C` (60 seconds) → preserve explicit leap-second civil field; do not normalize it as ordinary second 0.
- `PVT-RAW-010`: offset 11 reserved bit set (e.g. `0x87`) → preserve/report reserved bits; do not reinterpret them as validity.
- Length tests: 0, 91, 92 and 93 bytes; only 92 is structurally accepted. Also test invalid calendar fields and out-of-contract nano values separately from valid-but-false flags.

### 4.10 NAV-TIMEUTC raw payload fixtures

R07 NAV-TIMEUTC is class/id `01/21`, payload length exactly 20 bytes. Layout:
- 0..3 `iTOW` U4 LE; 4..7 `tAcc` U4 LE; 8..11 `nano` signed I4 LE;
- 12..13 `year` U2 LE (1999..2099); 14 month; 15 day; 16 hour; 17 minute; 18 second (0..60); 19 `valid` bitfield.
- Byte 19 masks: `validTOW=0x01`, `validWKN=0x02`, `validUTC=0x04`, reserved `0x08`, `utcStandard=(byte >> 4) & 0x0F`.

Canonical 20-byte fixture with UTC=2024-01-01 00:00:00 and validUTC only:
`00 00 00 00 00 00 00 00 00 00 00 00 E8 07 01 01 00 00 00 04`.
Generate variants by changing only byte 19 to `00, 01, 02, 04, 08, 20, F4, 74`; expected bit interpretations are the `TIMEUTC-FLAGS-001..008` cases in §4.8. Additional raw parser checks:
- payload lengths 0, 19, 20, 21; only 20 accepted;
- `nano=FF FF FF FF` decodes to -1 ns;
- UTC year lower/upper bounds 1999 and 2099 accepted structurally; out-of-range year and invalid calendar date rejected by civil-time validation;
- raw byte 19 is always preserved, reserved bit 3 does not become a validity flag, and unknown `utcStandard` is not rejected by the byte decoder itself (policy layer rejects it);
- PVT and TIMEUTC records with equal iTOW but different normalized civil UTC must fail the mapper cross-check; equal wall-clock-looking values with different iTOW must not be joined.

These fixtures complement the NAV-TIMELS 24-byte fixtures in §4.7. They specify parser and policy test inputs; they are not executed tests or evidence of receiver output. The R07 protocol document is an early-production document, so target firmware/protocol compatibility still requires MON-VER and on-device message capture.

### 4.11 固定驱动的 NAV-PVT / TIMEUTC / STATUS 输出合同审计

审计对象固定为 aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e，不是仓库当前 main 的浮动版本。关键文件与职责：
- ublox_dgnss_node/include/ublox_dgnss_node/ubx/nav/ubx_nav_pvt.hpp：NAV-PVT payload 字段/offset 解码；
- .../ubx_nav_timeutc.hpp：NAV-TIMEUTC payload 字段/offset 解码；
- .../ubx_nav_status.hpp：NAV-STATUS payload 字段/offset 解码；
- ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_nav.hpp：NAV class/id dispatch 与对应 payload container；
- ublox_dgnss_node/src/ublox_dgnss_node.cpp：ROS publisher 创建、UBX frame 分发及字段映射；
- ublox_ubx_msgs/msg/UBXNavPVT.msg、UBXNavTimeUTC.msg、UBXNavStatus.msg：ROSIDL 输出字段。

| UBX 消息 | 固定驱动输出 | 已映射的关键字段 | 未保留/限制 |
|---|---|---|---|
| NAV-PVT (01/07) | ubx_nav_pvt / UBXNavPVT | iTOW、UTC civil、valid_date/time/fully_resolved、t_acc、signed nano、confirmed_avail/date/time | 原始 valid/flags/flags2 字节与 reserved bits 未发布；UTC mapper 可用语义 bool，但无法从 ROS topic 审计原始 reserved bits |
| NAV-TIMEUTC (01/21) | ubx_nav_time_utc / UBXNavTimeUTC | iTOW、t_acc、signed nano、UTC civil、valid_tow/wkn/utc、utc_std.id | 原始 valid byte 未发布；ROSIDL year/month/day/hour/min/sec 使用 int16/int8，而 UBX wire 为 U2/U1；正常协议值可表示，但需 adapter 验证范围 |
| NAV-STATUS (01/03) | ubx_nav_status / UBXNavStatus | iTOW、gps_fix_ok、wkn_set、tow_set、carr_soln_valid、carr_soln、msss | 没有 GPS week 数值；wkn_set/tow_set 只是 validity flags；原始 flags/fixStat/flags2 未发布 |

字段和发布路径核对结果：
1. NAV-PVT payload 从 offset 0/4/6..10/11/12/16/20/21/22 读取 iTOW、UTC civil、valid、tAcc、signed nano、fixType、flags、flags2；ROS publisher 将 valid 的三个 UTC 位与 flags2 的三个 confirmation 位逐个映射为 bool。confirmedAvail 不能被误读成 confirmedDate && confirmedTime，mapper 必须保留三位独立状态。
2. NAV-TIMEUTC payload 从 offset 0/4/8/12/14..18/19 读取 iTOW、tAcc、signed nano、UTC civil 与 valid byte；ROS publisher 将 validTOW/WKN/UTC 和 utcStandard id 分别映射。协议源中的 utcStandard 编码为 4-bit enum；策略层必须对 0、1..7、15 及保留/未知编码显式分类。
3. NAV-STATUS payload 从 offset 0/4/5/6/7/8/12 读取 iTOW、fix type、flags、fixStat、flags2、ttff、msss；ROS publisher 输出 wkn_set/tow_set。不得把这两个 bool 当成 GPS week 数值，也不得用 msss 单独断定 reboot。
4. ubx_nav.hpp 已对 PVT、STATUS、TIMEUTC 注册 frame container 和 class/id dispatch；node 侧分别创建 publisher，并在 NAV frame switch 中调用对应 publish helper。因此这三类消息的基础 transport/ROS 输出链已存在，mapper 可先作为 ROS-independent pure core 接入，而不是复制一套 USB/UBX transport。

**关键实现风险 — payload 长度检查**：
- 三个 payload 构造函数均按传入的 size resize/copy，再直接从固定 offset 调用 buf_offset<T>()；该 helper 仅执行 memcpy(buf->data()+offset, sizeof(T))，本身没有边界检查。
- 在已审计的这些固定文件/构造函数中，没有看到 NAV-PVT=92、NAV-TIMEUTC=20、NAV-STATUS=16 的 exact payload length guard。不能据此证明任意畸形帧一定能进入该路径，但一旦短 payload 到达构造函数，后续固定 offset 读取存在越界读取风险。
- P0 driver hardening：在任何 payload 字段读取前校验 class/id 与 exact payload length；不匹配时拒绝 frame、计数并记录 reason，不允许继续构造消息。还需单测短/长 payload，及合法长度但非法日历字段的分层处理。
- ROSIDL 语义字段足以完成大部分 mapper validity policy，但不能验证 reserved bits。若需求包含原始协议可观测性，应在专用 fork 的 ROSIDL 消息增加 raw validity/status bytes（例如 PVT valid/flags/flags2、TIMEUTC valid、STATUS flags/fixStat/flags2），或在同一 adapter 进程内先消费原始 UBX event；不得声称现有 ROS topic 已保留这些字节。

**NAV-TIMELS 差距**：固定版本的 ubx_nav.hpp include、payload type aliases、container、constructor、frame dispatch 均没有 NAV-TIMELS；node 也没有对应 publisher/message。故 NAV-TIMELS 仍需新增 ROSIDL、payload parser、dispatch、publisher 与配置输出路径。此项与现有 PVT/TIMEUTC/STATUS 不同，不能只写 ROS adapter 订阅配置来解决。

本节是固定源码静态审计；未运行编译、畸形帧注入、ROS topic 检查或 F9P 实机测试。后续实施必须在项目许可/维护者 Gate 通过后，对专用 fork/分支进行，不能直接改上游固定源或在本项目中复制 transport。

### 4.12 UTC→GPS source policy table 与纯算法属性测试合同（设计，未实现）

本节把前述 parser/test-vector 设计收敛为 mapper 的输入准入、跨消息配对、冲突拒绝、状态输出和生成式测试合同。它仍是 E3 设计规格：不增加生产代码、不增加依赖、不改变现有 Gate/Joiner API，也不表示任何测试已执行。

#### 4.12.1 输入源准入与优先级

主时间候选来自 NAV-PVT UTC；NAV-TIMEUTC 是同历元独立核验；NAV-TIMELS 提供当前 GPS−UTC leap-second offset；NAV-STATUS 只提供 receiver time validity / fix 状态指示。四类消息职责不同，不能互相代替。

| 输入 | 必须满足的结构/语义条件 | mapper 处理 | 缺失/不满足时 |
|---|---|---|---|
| NAV-PVT | class/id 01/07、payload 92 bytes；validDate、validTime、fullyResolved 均为 true；日期合法；signed nano 合法；若 confirmedAvail=1，必须同时 confirmedDate 和 confirmedTime | 作为 UTC 主候选；保留 valid 与 confirmation 三个位；confirmedAvail=0 只能输出 UNCONFIRMED | 结构错误/字段矛盾 → AMBIGUOUS_INVALID；UTC validity 不足 → 不产生可用绝对时间 |
| NAV-TIMEUTC | class/id 01/21、payload 20 bytes；validUTC=1；UTC civil/nano 合法；utcStandard 必须由显式 allowlist 接受 | 与 NAV-PVT 同 iTOW 严格配对并独立归一化，作为一致性核验；validTOW、validWKN 仍只是 validity bits | validUTC=0、未知/保留 utcStandard、无精确配对 → 不允许确认 PVT 候选；冲突 → AMBIGUOUS_INVALID |
| NAV-TIMELS | class/id 01/26、payload 24 bytes；validCurrLs=1；currLs 有符号解码；source code 必须由显式 allowlist 接受 | 提供当前 GPS−UTC 秒差；validTimeToLsEvent 与 validCurrLs 独立，当前 offset 不得错误要求 event bit 同时有效 | validCurrLs=0、source=0（default/hardcoded）、source=255/未知或策略未批准 → fail closed，不输出 UTC→GPS 绝对映射 |
| NAV-STATUS | class/id 01/03、payload 16 bytes；解析 gpsFixOK、towSet、wknSet | 只作 fix/time validity gate；towSet/wknSet 是标志，不能作为 iTOW/GPS week；msss 不可单独判定 reboot | 必需 validity flag 为 false 或 STATUS 不可配对 → NO_FIX 或 AMBIGUOUS_INVALID，按具体原因码区分 |

utcStandard 和 srcOfCurrLs 的 allowlist 必须是显式、可审计、按目标固件配置的策略数据；不能因某个枚举“看起来常见”就隐式接受。NAV-TIMEUTC 的标准代码 0、1..7、15 及保留/未知值均须在策略表中逐项归类。NAV-TIMELS source 1..7 的语义虽已由协议定义，但项目是否接受每一来源仍需决策；source 0/default 与 255/unknown 默认拒绝。不得在代码中内置“当前 leap seconds”常量作为 TIMELS 缺失时的静默回退。

PVT confirmation 是独立可信度维度：confirmedAvail=0 表示接收机未提供确认信息，最高只能是 UNCONFIRMED；confirmedAvail=1 但 confirmedDate/confirmedTime 不全，不能标成 confirmed。TIMEUTC 的 validUTC 只能交叉验证，不能替 PVT 补齐 confirmation bits。GNSS fix quality 与 UTC validity 也不得混成一个 bool：若 NAV-STATUS 表示 no-fix，即使日历字段看似有效，也不得将其发布为导航可用的 VALID 时间映射。

#### 4.12.2 精确历元配对与缓存隔离

1. PVT、TIMEUTC、TIMELS、STATUS 必须按完全相等的 iTOW 配对；不允许 nearest-neighbor、插值、按 ROS header stamp 猜测或以“最近到达”代替协议历元。
2. 每条记录至少携带 (process_session_id, connection_generation, receive_monotonic_ns, queue_sequence, iTOW)。同一 join key 必须属于同一 session/generation；旧 generation 的迟到回调一律丢弃。
3. iTOW 是周内毫秒计数，会在 GPS week 边界回绕；它不是全局唯一 epoch。精确相等必须再受 session/generation、接收单调时间 freshness 和缓存生命周期约束。不得让相隔一周但 iTOW 数值相同的记录跨缓存命中。
4. 缓存容量、最大接收年龄、最大 receive skew 必须配置化；在阈值未由目标设备/运行证据冻结前，配置为 unset 时 mapper 保持 fail-closed，不得臆造默认毫秒数。
5. GPS week rollover 本身不是 receiver reboot，也不自动触发 GNSS generation reset。周回绕识别必须使用可验证的连续性/周信息证据；NAV-STATUS wknSet 不是 week 数值，不能用于推导周号。
6. generation 或 process session 变化时，先推进 lifecycle fence，再清空全部待配对缓存、已退休 key 和候选输出；清理后才允许新 generation 首帧进入。任何旧 sequence / 旧 generation 数据不得污染新缓存。

#### 4.12.3 时间归一化、冲突与 fail-closed 规则

- PVT 与 TIMEUTC 各自按协议 signed nano 归一化，允许负 nano 跨秒/日/月/年边界；归一化应使用 Gregorian calendar 和明确的 leap-second-aware UTC 表示。
- 不得把 sec=60 直接交给普通 POSIX timestamp、Python datetime 或忽略 leap second 的库后再假定连续。必须保留 leap-second 显式状态，并由专用 UTC→GPS 转换器处理。
- PVT/TIMEUTC 同 iTOW 的归一化 UTC 不一致时拒绝该历元；容差、tAcc 合成方式和 maximum source uncertainty 必须来自已批准策略，未配置时不以“差不多”放行。
- TIMELS 的 validCurrLs、source allowlist、receive age 任一不满足，都不得使用上次值无限延续，也不得退回主机当前 leap-second 表；只能输出带 reason 的无效/陈旧状态。
- STATUS towSet 或 wknSet 为 false 时，不得输出绝对 GPS 时间锁定。即使二者为 true，也只证明对应时间字段被 receiver 置为有效，不提供 week number 本身。
- 当 source skew、消息 age、tAcc 门限、recovery hysteresis 尚未冻结时，不生成 VALID。策略配置缺失与输入数据错误必须使用不同 reason code，方便治理与运行诊断。

#### 4.12.4 输出状态机与可发布性

状态是 mapper 的时间映射状态，不是 RTK carrier solution 状态，也不是整个机器人 navigation health 状态。

| 状态 | 进入条件（概要） | 是否发布绝对 GPS week/TOW |
|---|---|---|
| NO_FIX | NAV-STATUS/PVT 明确表示当前无 GNSS fix（例如 gpsFixOK=false / no-fix fixType） | 否 |
| UNCONFIRMED | PVT UTC 字段有效且可与 TIMEUTC 核验，但接收机未提供完整 confirmation metadata | 默认否；只允许发布显式标注为候选的诊断数据 |
| VALID | PVT validity/confirmation 满足策略；TIMEUTC 同 iTOW 一致且标准获准；TIMELS 当前 offset 有效且来源获准；STATUS time flags 有效；所有 age/skew/accuracy 参数已配置并通过；generation fence 当前有效 | 是 |
| DEGRADED | 四类必需消息及其 validity/source/epoch 条件均满足，但可选的 NAV-TIMEGPS 等附加交叉核验缺失/不可用，且策略明确允许降级诊断 | 默认否；不得因 DEGRADED 发布导航消费的绝对 GPS 时间，除非单独 ADR 明确批准并定义独立接口 |
| AMBIGUOUS_INVALID | 同历元冲突、UTC/日历非法、未知枚举、source 不可信、join 失败、无法判定 epoch 或输入组合矛盾 | 否 |
| STALE | 任一必需源超过已批准 freshness/age，或 receive skew 超限 | 否 |
| RESETTING | detach/attach、session/generation 变化或 reset transaction 尚未完成 | 否 |

默认发布策略为 fail-closed：只有 VALID 可以进入导航消费的绝对 GPS 时间通道。NO_FIX 只表达接收机无 fix；towSet/wknSet 或 TIMEUTC validTOW/validWKN 等时间有效性标志缺失/为 false 时归入 AMBIGUOUS_INVALID，而不是 NO_FIX。DEGRADED 仅用于四类必需输入仍完整有效、但可选附加核验缺失的诊断情形；不得用它绕过 PVT/TIMEUTC/TIMELS/STATUS 的必需条件。UNCONFIRMED / DEGRADED 仅供诊断；其他状态不发布可被误认为有效的 week/TOW。每个输出同时携带稳定的 machine-readable reason_codes、source identity、generation、source uncertainty 和输入历元；状态恢复必须服从单独冻结的恢复滞回策略。状态转换不得隐式清除/重置其他 GNSS quality gate 或 estimator epoch。

#### 4.12.5 纯算法属性测试矩阵（Hypothesis 尚未引入）

本轮检查 robodog_gnss_quality 的工程元数据与现有测试：package.xml 仅声明 pytest，setup.py 的 test extra 仅包含 pytest；当前测试文件使用 unittest.TestCase。没有已声明的 Hypothesis 依赖，也没有依据擅自增加第三方测试依赖。因此第一阶段采用 Python 标准库 + unittest 的确定性生成式/property-style tests：固定 seed、边界值集合、可重复的随机序列和最小反例输出；若以后希望采用 Hypothesis，需单独评估依赖供应链、ROS distro 可用性和项目许可 Gate，再明确批准后加入 test extra。

| 属性 ID | 生成输入/扰动 | 必须始终成立的不变量 |
|---|---|---|
| UTC-NORM-PROP-001 | 生成合法日期时间与 nano（正/负、跨秒/日/月/年） | 归一化结果与独立整数纳秒 oracle 一致；不丢失符号，不发生静默溢出 |
| UTC-NORM-PROP-002 | 生成闰年/世纪年/月末/年末日历 | Gregorian 闰年规则正确；非法日期拒绝，不自动滚入下月 |
| UTC-LEAP-PROP-003 | 生成普通秒、显式 sec=60、闰秒前后相邻时刻 | leap second 被明确表示；禁止 POSIX shortcut；GPS 连续时间映射符合已核验 offset |
| UTC-LEAP-VEC-004 | 固定 2016-12-31 正闰秒三向量：23:59:59 + offset 17 → GPS 2017-01-01 00:00:16；23:59:60 + offset 17 → GPS 00:00:17；2017-01-01 00:00:00 + offset 18 → GPS 00:00:18 | 三个连续 GPS 秒严格递增且相邻差为 1 秒 |
| UTC-WEEK-PROP-005 | 在已知 GPS absolute ns 的周边界前后生成 -1 ns / 0 / +1 ns | week/TOW 满足 0 <= tow < 604800 s；绝对纳秒可由 week/TOW 无损重建；边界不多加/少加一周 |
| UTC-JOIN-PROP-006 | 随机排列 PVT/TIMEUTC/TIMELS/STATUS 到达顺序、重复、延迟、缺失、错配 iTOW | 仅同 iTOW 且同 session/generation 的完整组合可 join；任何错配均不产出 VALID |
| UTC-CONFLICT-PROP-007 | 对同 iTOW 的 PVT/TIMEUTC 注入任意 UTC/nano 差异 | 超出显式策略容差必拒绝；容差未配置时不得默许；冲突不能被后到消息覆盖 |
| UTC-SOURCE-PROP-008 | 枚举所有 utcStandard、srcOfCurrLs 值及 reserved/unknown | 只有策略 allowlist 的值被接受；0/default、255/unknown 与未登记枚举 fail-closed |
| UTC-VALID-PROP-009 | 枚举 PVT valid bits、confirmation bits、TIMEUTC valid bits、TIMELS valid bits、STATUS tow/week flags 的组合 | 各位独立解释；confirmation 不可由 TIMEUTC 代替；STATUS flags 不可被当作数值 |
| UTC-GENERATION-PROP-010 | 在任意缓存状态中插入 session/generation 变化及旧 callback | generation 变化后旧缓存为空；旧 generation/sequence 永远不能产出或污染新 generation 结果 |
| UTC-CACHE-PROP-011 | 生成重复 iTOW、跨 week wrap、超过 age/skew、超容量序列 | 缓存始终有界；过期/跨 epoch 样本不配对；周回绕不自动触发 reboot |
| UTC-STATE-PROP-012 | 生成所有输入条件组合与状态转换序列 | 只有完整策略准入的 VALID 可发布；RESETTING/STALE/AMBIGUOUS_INVALID 不得残留上一历元的有效输出 |

2016 正闰秒固定向量的解释必须统一：GPS−UTC 在闰秒前为 17 秒，闰秒后为 18 秒；sec=60 是真实的独立 UTC 标签，不能将它先折叠成下一日 00:00:00 再套用 18 秒 offset。实现必须采用能表达 leap second 的内部 civil-time 类型或等价整数时间线。

#### 4.12.6 测试执行与参数 Gate

- 第一阶段只新增纯算法 unittest，不导入 rclpy、ROS message、USB 或驱动包；所有随机生成均使用固定 seed，并在失败时打印 seed、property ID 和最小化后的输入。
- 对每条属性测试同时保留 §4.7–§4.10 的固定原始 payload fixtures；结构解码、calendar validation、mapper policy、状态机四层分开测试。
- 不在当前 Gate 未解除时新增 mapper/driver production code；本节本身也不执行测试或 CI。
- 以下参数必须由目标固件/设备日志或正式策略决策确定，未确定前保持 unset 并阻止 VALID：最大消息 age、最大 receive skew、PVT/TIMEUTC UTC 比较容差、tAcc/uncertainty 上限、TIMELS 来源 allowlist、utcStandard allowlist、恢复滞回、缓存容量/TTL。
- 目标设备 MON-VER（software/hardware/PROTVER）和真实 NAV-PVT/TIMEUTC/TIMELS/STATUS 原始帧仍未提供；不得宣称固件兼容、真实输出可用或 mapper 已验证。


### 4.13 UTC civil label → GPS integer timeline：数学定义与边界审查

本节定义 P1 pure-core 的数学合同。R07 定义 GPS epoch 为 1980-01-06，并定义 NAV-TIMELS.currLs 为 GPS 自起点以来超前 UTC 的当前秒数；iTOW 是 GPS week TOW。此处为数学规格/合成向量，不代表目标接收机实测。

#### 4.13.1 普通 UTC 标签的整数转换

区分 UTC civil label 与 GPS 连续整数时间轴。令 D 为 Gregorian 日期与 1980-01-06 的日数差，h/m/s 为时分秒，n 为规范化 signed nanoseconds，L 为同一导航历元且通过 source/validity/freshness Gate 的 NAV-TIMELS.currLs（秒）：

`nominal_utc_ns = (((D × 86400 + h × 3600 + m × 60 + s) × 10^9) + n)`

`gps_absolute_ns = nominal_utc_ns + L × 10^9`

`WEEK_NS = 604800 × 10^9`

对非负 gps_absolute_ns：

`gps_week, gps_tow_ns = divmod(gps_absolute_ns, WEEK_NS)`

不变量：0 ≤ gps_tow_ns < WEEK_NS；gps_absolute_ns == gps_week × WEEK_NS + gps_tow_ns。全程整数，禁止 float seconds。GPS epoch 之前、负绝对时间、无效 offset、超出支持日期范围或目标存储类型溢出必须明确拒绝。nominal_utc_ns 是 civil label 的名义日历坐标，不调用 POSIX timestamp API；普通秒公式不适用于 sec=60。

#### 4.13.2 闰秒标签必须独立表达

2016-12-31 正闰秒的三个相邻标签：

| UTC civil label | 当日名义秒坐标 | L | GPS 对应时刻 |
|---|---:|---:|---|
| 2016-12-31 23:59:59 | 86399 | 17 | 2017-01-01 00:00:16 |
| 2016-12-31 23:59:60 | 86400 | 17 | 2017-01-01 00:00:17 |
| 2017-01-01 00:00:00 | 次日 0 | 18 | 2017-01-01 00:00:18 |

三者在 GPS 整数时间线上严格相邻 1 秒。闰秒前与闰秒标签使用旧 offset 17；生效后的午夜使用新 offset 18。不得先将 23:59:60 折叠为次日 00:00:00 再加 18 秒，否则会与午夜标签碰撞。

sec=60 必须验证该日期确有 leap event，并能确定事件前/事件中 offset。仅有当前 currLs 不足以证明任意历史日期存在闰秒；必须有经审计的 leap transition fixture/table 或足以定位事件的有效协议证据。缺少证据时返回明确 unsupported/ambiguous reason，不生成 VALID。

#### 4.13.3 signed nano 归一化

- 先校验 Gregorian 日期及 civil fields；不得让非法日期自动滚入下月。
- 普通秒的 signed nano 用整数商余数规范化，再执行跨秒/日/月/年借位。
- sec=60 进入 leap-event-aware 分支。负 nano 可能回退到前一秒；正 nano 可能推进到事件后下一秒。跨越事件必须有 transition evidence 和前后 offset，否则拒绝。
- 保留原始字段与规范化结果；拒绝协议/固件不允许的 nano 范围、非法 civil fields 和无法解释的 leap transition。
- 测试 oracle 必须采用独立实现路径，避免与生产算法共享相同错误。

#### 4.13.4 GPS 周边界与 UBX iTOW

GPS week/TOW 由 gps_absolute_ns 整数 divmod 得到。边界前 1 ns → 前一周 WEEK_NS−1；边界正好 → 下一周 TOW=0；边界后 1 ns → 下一周 TOW=1。

UBX iTOW 是整数毫秒，不能表达 sub-ms remainder。保留 gps_tow_ns 原值，并独立计算毫秒量化值与余数；与 iTOW 的比较必须有明确精度合同，不能直接要求纳秒 TOW 等于毫秒 iTOW，也不能静默四舍五入。Exact join 仍使用原始 iTOW；iTOW 不含 GPS week number，不能单独构造绝对时间。周回绕不得仅凭 iTOW 变化判定 reboot。

#### 4.13.5 数学测试合同

| ID | 输入/条件 | 不变量/预期 |
|---|---|---|
| GPS-MATH-001 | GPS epoch 1980-01-06 00:00:00，L=0 | absolute=0，week=0，TOW=0 |
| GPS-MATH-002 | 2024-01-07 00:00:00，L=18 s | week=2296，TOW=18,000,000,000 ns |
| GPS-MATH-003/004/005 | k×WEEK_NS−1 / k×WEEK_NS / k×WEEK_NS+1 | 分别为前周末、下一周 TOW=0、下一周 TOW=1 |
| GPS-MATH-006 | UTC label + L 与独立整数 oracle | 绝对纳秒一致，无 float 路径 |
| GPS-MATH-007 | 2016 闰秒三标签，L=17/17/18 | 三个 GPS instant 相邻 1e9 ns |
| GPS-MATH-008 | 无事件证据的 sec=60 | 拒绝，不产生 VALID |
| GPS-MATH-009 | sec=60 配合 nano=-1 或 +1e9，但无 transition evidence | 拒绝跨事件归一化 |
| GPS-MATH-010 | iTOW 毫秒与 TOW 纳秒存在 sub-ms remainder | 保留 remainder，按显式精度策略比较 |
| GPS-MATH-011 | int64 边界及越界 | 边界内精确，越界显式失败，不 wrap |
| GPS-MATH-012 | GPS epoch 前日期/负绝对时间 | 按支持范围拒绝，不产生负周伪有效结果 |

#### 4.13.6 未冻结 Gate

公式中的 L 必须来自同一历元有效 NAV-TIMELS，source allowlist/freshness 仍未冻结。sec=60 需要 transition evidence API；§4.12 的 current-offset source policy 本身不足以验证历史闰秒日期。PVT/TIMEUTC nano 合法范围及字段边界需以目标协议版本/MON-VER 为准。iTOW sub-ms 比较、tAcc 合成与容差仍待 receiver evidence/策略批准。仅增加设计和合成测试合同，不增加代码/依赖，不声称测试或硬件验证。


### 4.14 Leap-transition evidence：数据模型、来源分层与冲突策略

本节补足 §4.13 的 sec=60 证据缺口。设计目标是：每个 leap transition 都能追溯到权威来源或经验证的接收机事件记录；运行时不联网、不依赖主机 leap-second 文件；缺少/冲突证据时 fail-closed。本节为设计合同，尚未实现。

#### 4.14.1 证据来源分层

| 来源 | 可提供的证据 | 角色与限制 |
|---|---|---|
| IERS Bulletin C | 对某一可能生效日期是否引入闰秒的正式公告 | 官方事件公告；适合新事件确认，不是完整历史数据表 |
| USNO tai-utc.dat | 历史 TAI−UTC 分段值及生效日期 | 历史转换数据源；需经审计的 parser 和 GPS/TAI 关系转换，不能把 TAI−UTC 数值直接当 GPS−UTC |
| 接收机 UBX-NAV-TIMELS | 当前 currLs；可选的最近/未来事件信息、事件变化量、事件日期和来源 | 同历元运行时证据/交叉核验；不自动替代完整历史表，且需目标固件验证字段语义 |
| 项目内冻结 transition artifact | 已审查、带 provenance/hash 的有限日期范围转换记录 | 运行时唯一可直接查询的历史 transition 数据；无网络依赖，超出覆盖范围拒绝 |

协议证据（u-blox F9 HPG 1.30 / UBX-21046737 R01 §3.15.26）明确：NAV-TIMELS 长度 24 bytes；srcOfCurrLs 和 currLs 描述当前差值；srcOfLsChange 和 lsChange 描述未来事件来源/变化量；timeToLsEvent 为正表示未来、零表示事件当前发生、负表示过去；dateOfLsGpsWn/dateOfLsGpsDn 在 validTimeToLsEvent=1 时描述下一次或最近一次事件的 GPS week/day；valid bit 0 是 validCurrLs，bit 1 是 validTimeToLsEvent。bit 1 不应被误用为 bit 0 的替代。

关键解释边界：
- validCurrLs=1 只允许评估当前 offset，不自动证明任意历史 civil date 的 leap event。
- validTimeToLsEvent=1 且 timeToLsEvent>0 表示未来事件信息；可用于安排未来 transition，但不得把尚未发生事件写进当前历史 offset。
- validTimeToLsEvent=1 且 timeToLsEvent<0 表示相对最近一次事件的过去信息；dateOfLsGpsWn/Dn 可作为最近事件日期的接收机侧锚点，但必须检查固件版本、字段范围、来源和与离线表的一致性。
- lsChange=0 既可能表示没有未来事件，也可能表示无信息；不能将其解释成“历史上从未发生事件”或“delta=0 的已确认事件”。
- srcOfLsChange 是事件来源，不得与 srcOfCurrLs 混用；两套 allowlist 分开维护。
- dateOfLsGpsWn/Dn 只给 GPS week/day，不应在未经协议语义和目标固件验证时推断精确秒/闰秒标签。timeToLsEvent 是相对导航历元的秒数；将其与日期字段组合前需明确整秒锚点、符号、舍入和 iTOW 对齐语义。

#### 4.14.2 Canonical transition record（规范事件记录）

每个事件记录至少包含：

| 字段 | 类型/含义 | 约束 |
|---|---|---|
| transition_id | 稳定唯一 ID | 不使用数组下标作为身份 |
| effective_utc_date | offset 生效后的 UTC 日期/午夜 | 对正闰秒，前一日 23:59:60 是独立标签；对负闰秒需单独建模缺失标签 |
| delta_gps_minus_utc_s | GPS−UTC 的变化量 | 当前标准历史事件通常为 ±1；解析层不得硬编码只接受 +1 |
| gps_minus_utc_before_s / gps_minus_utc_after_s | 事件前/后 offset | 必须满足 after = before + delta |
| effective_gps_week / effective_gps_tow_ns | 事件在 GPS 连续时间线的有效边界 | 整数；需由独立验证路径生成 |
| source_kind | IERS / USNO / UBX_RECEIVER / PROJECT_REVIEW | 多来源记录分别保留，不把来源压成单一字符串 |
| source_document_id / source_revision / source_url | 来源标识、版本和地址 | 不可缺省或伪造 |
| source_retrieved_at_utc / source_published_at_utc | 获取与发布时刻 | 区分发布时间与采集时间 |
| source_sha256 | 原始来源快照摘要 | 对下载原始字节计算；网页动态内容需归档快照 |
| parser_version | 解析器版本 | 任何解析规则变更可追溯 |
| review_status | DRAFT / VERIFIED / REJECTED | 只有 VERIFIED 记录可进入 runtime artifact |
| review_evidence | 交叉核对记录 | 至少包含第二来源或独立人工核验理由 |
| supported_from / supported_until | 数据集覆盖范围 | 查询超范围必须返回 OUT_OF_COVERAGE，不外推 |

规范表按 effective_utc_date 严格递增；相邻记录的 gps_minus_utc_after_s 必须等于下一记录的 gps_minus_utc_before_s；不得有重复生效时刻、重叠区间、offset 跳变不等于 delta、日期缺口或无法解释的来源冲突。原始来源记录与 canonical record 分开存储，禁止覆盖原始证据。

#### 4.14.3 TAI−UTC 到 GPS−UTC 的来源转换边界

USNO 历史表记录的是 TAI−UTC，不是 GPS−UTC。若项目选择它作为历史数据源，转换必须显式使用已独立审计的 TAI−GPS 常数关系（GPS 时间与 TAI 的固定 19 s 差值），而不是把表中数字直接填入 currLs。该常数关系、适用 epoch 和转换测试必须有单独来源与测试向量；本节不把未附证据的转换器视为已验收。BIPM 对 TAI−GPS 的正式关系写作 19 s + C₀，其中 C₀ 为随时间变化、量级为数十纳秒的项；因此从 USNO 的整秒 TAI−UTC 表减去 19 s，只能得到整秒 GPS−UTC leap offset，不能宣称获得纳秒级 GPS/TAI realization。若 mapper 输出整数纳秒坐标，必须把它标记为 nominal integer timeline，并另行保留 receiver tAcc/时间实现不确定度；不得把表格推导出的整秒 offset 当作纳秒精度证明。若未来需求要求纳秒级跨时间尺度溯源，必须引入经批准的 BIPM/UTC(k)−GPS C₀ 数据及其 epoch/interpolation/uncertainty 合同，当前版本不做该扩展。

IERS Bulletin C 用于确认公告日期及是否真的实施变化；USNO 历史表用于核对历史 offset 分段。若公告、历史表、接收机 event fields 三者冲突：
1. 不通过“多数表决”自动选值；
2. 将相关 transition 标记为 CONFLICT，阻止该事件及受影响日期生成 VALID；
3. 保存各来源原始内容、版本、hash 和冲突字段；
4. 需要人工审查并更新项目 artifact 后才能解除；
5. 新公告不允许自动改变已发布的运行时 artifact。

#### 4.14.4 Receiver event evidence 的校验合同

接收机 NAV-TIMELS decoder 必须保留 24-byte raw payload 和以下 typed fields：iTOW、version、srcOfCurrLs、currLs、srcOfLsChange、lsChange、timeToLsEvent、dateOfLsGpsWn、dateOfLsGpsDn、raw valid byte。不得只保留 ROS 消息中被简化的 validity bool 而丢弃 raw byte。

运行时评估分两条独立路径：
- Current-offset path：validCurrLs、srcOfCurrLs、同 iTOW 及 generation/session/freshness 检查；用于 §4.13 普通 UTC label 的 offset。
- Event-evidence path：只有 validTimeToLsEvent 为 true 才解释 event fields；验证 signed timeToLsEvent、lsChange、srcOfLsChange、GPS week/day 范围、消息版本、同 epoch 关系和目标固件行为。该路径只用于 event corroboration/未来事件计划，不能替代历史 transition artifact。
- 对已过去事件，receiver 的 last-event date/week/day 应与冻结 artifact 对应事件一致；对未来事件，receiver schedule 必须与公告/策略一致。任何不一致均为 CONFLICT，不得悄悄更新 artifact。
- 若目标驱动当前未解析/发布 NAV-TIMELS，则 receiver event evidence 当前不可用；不得假设 ROS topic 已存在。需要先完成 24-byte parser、dispatch、ROSIDL、publisher 和原始字段保真审计。

#### 4.14.5 Runtime artifact 与更新治理

建议文件格式采用 UTF-8 JSON 或 YAML，但 canonical schema 与内容 hash 必须固定；首版只覆盖项目明确支持的 GPS epoch 至当前已审查日期范围。运行时：
- 不发起 HTTP 请求，不读取操作系统隐式 leap-second 数据库，不自动从网络更新；
- artifact 缺失、schema/version 不支持、hash 不匹配、记录未 VERIFIED、查询日期超出覆盖范围或内部连续性检查失败时，返回明确 reason code 并阻止 VALID；
- artifact 更新以单独 PR/commit 完成，包含原始来源快照、source URL/revision/retrieved time/hash、解析器版本、生成差异、人工审查记录和完整回归测试；
- 禁止只改生成表而不更新 provenance；禁止无来源手工编辑历史 offset；
- artifact 的版本与 hash 纳入 mapper diagnostic output，便于 rosbag 重放复现。

#### 4.14.6 增补 transition evidence 测试矩阵

| ID | 输入/场景 | 预期 |
|---|---|---|
| LEAP-EVID-001 | VERIFIED transition artifact，日期在覆盖范围内 | 可查询唯一 offset interval 和 transition |
| LEAP-EVID-002 | sec=60 标签命中已确认正闰秒前一日 | 映射到独立 leap-second timeline slot |
| LEAP-EVID-003 | sec=60 出现在无 transition 的日期 | REJECTED，不生成 VALID |
| LEAP-EVID-004 | artifact 缺失/损坏/hash 不匹配/schema 不支持 | fail-closed，reason 可诊断 |
| LEAP-EVID-005 | 查询日期早于/晚于 artifact coverage | OUT_OF_COVERAGE，不外推 |
| LEAP-EVID-006 | 相邻记录 offset 连续性或 delta 不一致 | artifact validation failure |
| LEAP-EVID-007 | IERS 公告与 USNO historical table 冲突 | CONFLICT，需人工审查 |
| LEAP-EVID-008 | receiver past-event date 与 artifact 不一致 | CONFLICT，不覆盖 artifact |
| LEAP-EVID-009 | receiver future event 与已公告事件不一致 | CONFLICT；不自动接受未来 offset |
| LEAP-EVID-010 | validCurrLs=true、validTimeToLsEvent=false | 当前 offset 可单独评估；不得读取 event fields |
| LEAP-EVID-011 | validCurrLs=false、validTimeToLsEvent=true | event metadata 可诊断；不得用于当前 offset |
| LEAP-EVID-012 | timeToLsEvent=0 / lsChange=0 | 不能据此推断无事件或已发生 delta=0 事件 |
| LEAP-EVID-013 | srcOfCurrLs 与 srcOfLsChange 分别枚举 | 独立 allowlist；未知/默认来源 fail-closed |
| LEAP-EVID-014 | artifact 同一版本 rosbag 重放 | 结果与 source hash/artifact version 完全可复现 |

#### 4.14.7 Gate 与结论

1. 先审计并冻结 canonical schema、IERS/USNO source snapshot 归档方式及 TAI−GPS=19 s 转换来源；再生成第一版历史 artifact。
2. 单独核对 UBX NAV-TIMELS event fields 的周/日/相对秒语义，尤其 dateOfLsGpsWn/Dn 与 timeToLsEvent 的组合方式；不得凭字段名猜测。
3. 取得目标 ZED-F9P MON-VER 和原始 24-byte NAV-TIMELS 帧后，才冻结 firmware-specific event source allowlist、字段可用性和 receiver corroboration 策略。
4. 完成驱动 parser/ROSIDL/dispatch/publisher 保真后，才可进行接收机 event evidence 实测。
5. 在上述 Gate 关闭前，不实现 production mapper、不将 sec=60 接入 VALID；不添加网络运行依赖。


### 4.15 USNO TAI−UTC 历史分段解析与离线 artifact 生成规格（设计，未生成）

本节把 §4.14 的来源分层落实为可复核的解析/生成合同。当前已通过公开历史表交叉核验关键分段，但尚未取得并归档 USNO 原始 `tai-utc.dat` 字节快照；因此本节只冻结解析规则与测试向量，**不生成 VERIFIED transition artifact，不填写 source_sha256，也不宣称解析器已实现或测试通过**。

#### 4.15.1 输入格式与解析边界

USNO/IERS 历史文件常见记录形式为：

```text
<record-number> <YYYY> <MON> <DD> =JD <julian-date> TAI-UTC= <base-seconds> S + (MJD - <reference-mjd>) X <drift-seconds-per-day> S
```

实际文件可能包含注释、空行及早期格式变体。实现必须先固定具体原始文件版本及字节快照，再冻结严格 grammar；不得以网页展示文本代替原始文件 parser。字段语义如下：

- civil effective date：该 TAI−UTC 分段在该 UTC 日期 00:00:00 起生效；
- base-seconds：TAI−UTC 基值；
- reference-mjd 与 drift：早期 UTC rate adjustment 的参考 MJD 与线性漂移系数；
- JD：用于独立校验日期的冗余字段，不作为主日期来源；
- record-number：仅源文件行标识，不是 transition ID。

解析器必须保留 raw line、行号、所有原始 token 与解析结果。未知格式、重复字段、无法识别月份、非法日期、数字溢出、缺失字段或额外未解释 token 一律拒绝；不得尽力猜测或跳过无法解析的有效记录。

#### 4.15.2 支持范围与 GPS epoch clipping

- GPS epoch 为 1980-01-06 00:00:00 UTC。USNO/IERS 表中 1980-01-01 分段已给出 TAI−UTC = 19 s；该分段覆盖 GPS epoch，因此可将其作为 GPS epoch 初始基线。
- GPS epoch 当时 GPS−UTC = 0 s，因为 nominal integer conversion 使用 `TAI−UTC − 19 s`。这只定义整秒 leap offset，不构成 §4.14.3 所述纳秒级时间实现证明。
- artifact 的 `supported_from` 必须精确裁剪为 1980-01-06，而不是 1980-01-01；不得把 GPS epoch 之前的历史分段伪装成 GPS 时间线支持范围。
- 必须保留 1980-01-01 原始分段作为 epoch clipping 的来源证据，并记录其 effective date 早于支持起点。
- 任何查询早于 1980-01-06、晚于最后一个已审查分段的明确覆盖边界，均返回 OUT_OF_COVERAGE；最后一条开放区间不能无期限自动视为 VERIFIED，必须由审核记录确定 artifact 的 `supported_until`。

#### 4.15.3 固定整数分段准入规则

首版 artifact schema 仅支持 1972-01-01 之后的整秒 TAI−UTC 分段。对每条拟纳入记录：

1. 解析 base-seconds、drift 与 reference-mjd；drift 必须精确为 0（按源文件十进制 token 做 Decimal/有理数解析，不使用 float epsilon）。
2. base-seconds 必须是整数秒；否则拒绝进入本版 artifact。
3. 将 `gps_minus_utc_s = tai_minus_utc_s - 19` 作为显式转换字段，保存输入 TAI−UTC 与转换规则版本；不得覆盖或丢弃源值。
4. 每条记录的生效日期必须严格递增且唯一；日期字段必须与 JD（按源文件约定的 UTC 0h）交叉校验。
5. 相邻记录的 offset 差值必须属于本版允许的 leap transition 集合（当前首版按 ±1 秒建模）；相同 offset 的重复记录应作为来源异常审查，不得自动合并。
6. 本版遇到非零 drift、非整数 offset、非 ±1 跳变、缺失日期基线、乱序/重复日期或无法解释的连续性异常，整体生成失败，不得部分产出看似完整的 artifact。
7. 1972 之前的 rate-adjustment 历史数据明确不属于本版 GPS epoch artifact；不能简单将其 drift 记录丢入整秒表，也不能以 1972 年前的数据推断 GPS epoch offset。

#### 4.15.4 Transition 语义与 2016 样例

分段表的 effective date 是新 offset 生效后的 UTC 午夜。对正闰秒，独立的 leap label 位于前一 UTC 日的 23:59:60；这两个时间概念必须分别保存。

| UTC 分段/事件 | TAI−UTC | nominal GPS−UTC（减 19 s） | 解释 |
|---|---:|---:|---|
| 2015-07-01 至 2016-12-31 结束 | 36 s | 17 s | 事件前分段 |
| 2016-12-31 23:59:60 | 事件期间沿用 17 s | 17 s | 正闰秒独立标签 |
| 2017-01-01 00:00:00 起 | 37 s | 18 s | 新分段生效 |

因此 canonical transition 的 `effective_utc_date` 为 2017-01-01，而 `leap_label_utc`（若 delta=+1）为 2016-12-31 23:59:60。记录必须满足 before=17、after=18、delta=+1。不能把 2017-01-01 00:00:00 写成闰秒标签，也不能将 effective date 回退一天。

对于负闰秒，schema 必须保留 `delta=-1` 并另行定义缺失 civil label；不得复用正闰秒的 23:59:60 表达。首版没有负闰秒实证样例，相关转换仍须独立测试。

#### 4.15.5 生成器输出与 provenance

生成器输入：原始 USNO 文件字节、文件来源 URL/版本标识、retrieved_at、parser version、TAI−GPS conversion rule ID、review manifest。

生成器输出分为两个文件/对象：
- 原始来源快照（字节不变）及 SHA-256；
- canonical transition artifact（schema version、GPS epoch coverage、records、源行引用、生成器版本、source hash、review status）。

生成器只能输出 DRAFT；只有独立审查确认原始来源、TAI−GPS 转换、IERS 交叉核对、连续性与样例测试后，人工 review manifest 才能把记录提升为 VERIFIED。生成器不得自行把 DRAFT 标记为 VERIFIED。当前网页搜索可见的 IERS 历史表和 BIPM 表用于交叉核对关键值，不替代 USNO 原始字节快照；原始快照及 SHA-256 仍为 Gate。

#### 4.15.6 必须新增的解析/生成测试合同

| ID | 场景 | 预期 |
|---|---|---|
| USNO-PARSE-001 | 合法整秒记录，drift=0 | 精确解析所有字段并保留 raw line |
| USNO-PARSE-002 | 注释行/空行/文件头 | 只按冻结 grammar 处理，不误认数据记录 |
| USNO-PARSE-003 | 未知月份、非法日期、缺字段/额外 token | 明确拒绝并报告行号/reason |
| USNO-PARSE-004 | 重复 effective date / 非递增日期 | 整体拒绝，不自动排序或去重 |
| USNO-PARSE-005 | 非零 drift（含极小非零十进制） | 拒绝；不得使用 float epsilon 放行 |
| USNO-PARSE-006 | 非整数 TAI−UTC | 拒绝进入整秒 artifact |
| USNO-PARSE-007 | JD 与 civil date 不一致 | 拒绝并报告字段冲突 |
| USNO-PARSE-008 | 1972 前 rate-adjustment 行 | 不进入本版 artifact；若混入目标区间则拒绝 |
| USNO-GEN-009 | 1980-01-01 TAI−UTC=19，GPS epoch clipping | artifact 从 1980-01-06 开始，初始 GPS−UTC=0 |
| USNO-GEN-010 | 2015-07-01 offset 36 到 2017-01-01 offset 37 | 生成 before=17/after=18/delta=+1 |
| USNO-GEN-011 | 2016 leap label 与 effective midnight | 分别为 2016-12-31 23:59:60 与 2017-01-01 00:00:00 |
| USNO-GEN-012 | offset 差值为 0、±2 或其他非 ±1 | 首版生成失败，进入人工审查 |
| USNO-GEN-013 | 缺少 GPS epoch 前置基线记录 | 不猜初始 offset，生成失败 |
| USNO-GEN-014 | 原始文件 hash 缺失或生成器版本缺失 | 只能保持 DRAFT/拒绝发布 |
| USNO-GEN-015 | 生成结果与独立 BIPM/IERS 固定向量比较 | 所有已审查分段一致，否则 CONFLICT |
| USNO-GEN-016 | 最后分段没有明确 supported_until/review | 禁止发布开放式无限覆盖 artifact |

#### 4.15.7 本节结论与 Gate

已由 IERS 历史表与 BIPM 年报交叉确认：1980-01-01 的 TAI−UTC 为 19 s；2015-07-01 至 2017-01-01 为 36 s；2017-01-01 起为 37 s。由此推得 GPS−UTC 整秒 offset 分别为 0、17、18 秒；2016 正闰秒事件标签和生效午夜的区分如上。当前尚未获取 USNO 原始 `tai-utc.dat` 文件字节及其 SHA-256，也未实现 parser/generator、冻结 artifact 或执行测试。因此本节状态为设计完成、来源快照 Gate 未关闭、实现/测试未开始。下一步先归档原始文件和 hash，再对照 IERS/BIPM 记录做逐行审核；不需要用户手动下载，除非后续访问/授权受阻。



#### 4.15.8 原始文件实查后的格式修正（2026-11-25）

已直接打开 USNO 官方目录 `https://maia.usno.navy.mil/ser7/` 与其 `tai-utc.dat` 文本响应。目录显示文件最后修改时间为 2026-10-15 18:32、大小约 3.2K；文本响应共 41 条数据行。**实查结果修正 §4.15.1 的初始示意 grammar**：真实行没有 record-number 前缀，形式为：

```text
1961 JAN  1 =JD 2437300.5  TAI-UTC=   1.4228180 S + (MJD - 37300.) X 0.001296 S
1972 JAN  1 =JD 2441317.5  TAI-UTC=  10.0       S + (MJD - 41317.) X 0.0      S
1980 JAN  1 =JD 2444239.5  TAI-UTC=  19.0       S + (MJD - 41317.) X 0.0      S
2015 JUL  1 =JD 2457204.5  TAI-UTC=  36.0       S + (MJD - 41317.) X 0.0      S
2017 JAN  1 =JD 2457754.5  TAI-UTC=  37.0       S + (MJD - 41317.) X 0.0      S
```

因此 parser 不得假设行号字段或把 whitespace token index 当作固定列。推荐先用锚定正则解析 date、JD、TAI−UTC、reference MJD、drift 五个语义字段；正则需容忍字段间变长空格及 drift 数值与尾随 `S` 之间可有/无空格，但必须完整消费整条数据行。原始文件当前 41 行从 1961 JAN 1 到 2017 JAN 1；1961–1971 记录包含非零 drift，1972 JAN 1 起可见记录 drift 为 0.0。该实查进一步支持“GPS epoch 后固定整秒分段”的范围假设，但不代表可以忽略 1972–1979 的行：它们是核验 1980 epoch 前连续性和源文件覆盖完整性的输入。

USNO 当前发布的 `checksums.sha512`（目录标示 2026-11-24 17:17）没有列出 `tai-utc.dat`；不能把该清单中其他文件的 digest 借用给本文件。网页文本响应虽然可供逐行检查，但本运行环境无法直接下载原始字节（容器 DNS 下载失败），因此**仍没有可独立复算的原始文件 SHA-256/SHA-512**。网页目录时间/大小和行内容只作为 E1 来源可访问证据，不作为字节级快照/hash。当前状态继续为 source snapshot/hash pending、artifact DRAFT 前置 Gate 未关闭。

本轮新增实查测试要求：固定列/record-number 假设必须失败；1972–1979 分段必须保留在连续性审计输入中；正则必须完整消费行并拒绝尾随垃圾；同一语义数据在空格宽度变化下解析一致。原 §4.15.1 的示意格式应按本节更正解释。


#### 4.16 NIST machine-readable leap-second list 独立交叉核验（2026-11-25）

本轮补充检查 NIST 官方页面及其链接的纯文本 leap-seconds.list：

- NIST 页面说明该文件是机器可读的 leap-second 数据，并列出历史正闰秒日期；其当前页面记载 TAI−UTC=37 s，且说明正闰秒标签序列为 23:59:59 → 23:59:60 → 00:00:00。来源：https://www.nist.gov/pml/time-and-frequency-division/time-realization/leap-seconds。
- 文件 URL：https://tf.nist.gov/leap-seconds.list。其数据行以 NTP epoch seconds + TAI−UTC offset 表示有效区间起点；注释明确每行 offset 从该 epoch 起生效，直到下一行或无限期（文件说明行 19–30）。
- 文件列出 1972-01-01 offset 10 s、1980-01-01 offset 19 s、2015-07-01 offset 36 s、2017-01-01 offset 37 s，与本文件 §4.15 对 USNO 表的关键分段一致。注意 NIST 表的时间尺度是 TAI−UTC，不是 GPS−UTC；仍需单独处理 TAI−GPS 关系，不能直接把该列当 GPS offset。
- 文件明确说明 1972-01-01 是当前 UTC/TAI 关系定义的起点；1972 前历史时间尺度复杂，不应把本项目的整秒模型向前扩展（文件说明行 46–50）。
- 文件头记载最后一次 leap-second 值更新为 2016-07-08，但同时标记 Updated with data from IERS Bulletin C72、expiry 2027-06-28；这意味着数据表内容没有在 2016 后新增 leap-second offset，但其过期元数据已随 Bulletin C72 更新。该文件不是每次无闰秒公告都更新数据行的简单“最后修改日期”指标。
- 文件末尾提供 SHA-1 39096da8b66c33126ea48831363f817b36782978，但其计算规则忽略注释和数据行空白，并纳入最后修改时间与过期时间字段；它不是原始文件字节 SHA-256/SHA-512，也不能代替字节快照的强哈希。
- NIST 说明 IERS Bulletin C 是闰秒事件公告来源；因此建议源职责维持：IERS Bulletin C 决定是否新增事件，USNO tai-utc.dat 提供历史 TAI−UTC 分段，NIST leap-seconds.list 作为独立机器可读交叉核验与 expiry/更新状态参考，接收机 NAV-TIMELS 仅作当前接收机证据。任一冲突进入 CONFLICT，不做多数表决。

截至本次检索，USNO leap-second 产品页公告：2026 年 12 月底不引入闰秒；最后一次正闰秒仍为 2016-12-31。该公告只覆盖其明确公告窗口，不构成未来无限期保证。来源：https://maia.usno.navy.mil/products/leap-second。项目 artifact 的 supported_until 应与实际审核公告/覆盖范围绑定，不能仅因 NIST expiry 到 2027-06-28 就自动把项目 artifact 延长到该日期。

新增核验合同：
1. NIST 文件解析器必须保留 NTP epoch 原值、offset、行号和原始行；使用整数算术验证每个 NTP epoch 对应 UTC 生效午夜，不用浮点时间戳。
2. NIST 表只允许用于 1972-01-01 及之后的当前 UTC/TAI 关系；1972 前记录拒绝或明确标为不支持。
3. 比较 USNO 与 NIST 时，按生效 UTC instant 对齐 TAI−UTC 值；不比较文件行号或把两种格式的记录数视为相同。
4. 独立核对 GPS epoch：1980-01-01 TAI−UTC=19 s，GPS epoch 1980-01-06 的 GPS−UTC=0 s；同时确认 2017-01-01 起 GPS−UTC=18 s。
5. 对 #$ / #@ / #h 元数据分别解析；SHA-1 只作为来源自带的完整性提示，不满足项目强哈希 Gate。
6. Bulletin C 的“无闰秒”公告必须记录公告编号、发布日期、适用的 6 月/12 月事件窗口；不得把 NIST 文件 expiry 或旧的 #h 值解释为“没有新闰秒”的独立证明。

结论：现在已建立 USNO 原始历史表 + NIST 机器可读表 + IERS Bulletin C 的三层证据职责，但 USNO 原始字节快照/SHA-256 仍未取得，NIST SHA-1 不替代该 Gate。没有生成 VERIFIED artifact，没有实现 parser/generator，也没有执行代码测试。

#### 4.17 2026 CGPM Resolution 5：UTC 连续化制度迁移（2027-05-20）

**本节为时间尺度制度变更的架构约束，不代表接收机/软件已支持迁移。** 2026-10-15 第 28 届 CGPM 通过 Resolution 5 (2026)，规定 continuous UTC 自 2027-05-20 生效，允许长期 |UT1−UTC| 最大达到 3600 s；BIPM 负责与 IERS、ITU 等组织协作并监测/通报 UT1−UTC。权威来源：https://www.bipm.org/en/cgpm-2026/resolution-5。

该决议改变了本项目对 UTC leap-second policy 的长期假设：
- 旧假设（不得再作为无限期未来模型）：UTC 通过 IERS 每半年 Bulletin C 决定是否在 6 月/12 月末插入正/负闰秒，并依靠有限有效期公告维护未来 transition table。
- 迁移日期：2027-05-20。它是 continuous UTC 制度的生效日期，不是已公告的 GPS−UTC 数值跳变时刻；不得人为在该日期给 GPS/UTC 时间线加/减一秒。
- 新制度目标：UTC 连续，不再通过未来 leap-second step 追随 UT1；|UT1−UTC| 可在长期范围内增长至最多 3600 s。需要 UT1 的应用必须使用 IERS/EOP 或 GNSS 广播的 UT1−UTC 信息，不能再把 UTC 当作始终紧贴 UT1 的代理。
- 2026-07-06 的 Bulletin C 72 仍按旧公告格式明确：2026-12-31 不插入闰秒，且 UTC−TAI 自 2017-01-01 起为 −37 s，until further notice。该公告与 CGPM Resolution 5 并不冲突：前者明确覆盖 2026 年 12 月事件窗口；后者规定 2027-05-20 起的制度迁移。
- IERS Bulletin C Guide 对旧制度的半年节奏说明：1 月公告 6 月 30 日事件、7 月公告 12 月 31 日事件；相应文件有效期约至 12 月 28 日/次年 6 月 28 日。此节奏只作为旧制度历史/过渡期语义，不能无条件套用到 2027-05-20 之后。

对本项目 UTC→GPS mapper 的影响与保守实施规则：
1. 将时间策略版本化，至少区分 `LEAP_STEP_REGIME`（旧制度历史区间）与 `CONTINUOUS_UTC_REGIME`（2027-05-20 起）。切换依据必须是版本化、审核通过的权威 policy record，不可仅凭主机日期静默切换。
2. 2027-05-20 是制度生效日期，不是 leap event。Resolution 5 只给出日历日期，未在本次审计到的文本中指定该日内的精确时刻；transition artifact 应先记录 `regime_effective_date`，只有后续权威实施文件明确时刻后才增加 `regime_effective_instant_utc`。同时记录决议编号/版本、发布机构、原文 URL、获取时间、内容哈希、审查人/状态；不生成 `delta_gps_minus_utc_s=±1` 记录。
3. 在 continuous UTC regime，mapper 不应再期待未来 IERS leap-second events；但仍须保留历史 leap-second 表，以正确解释 1972–2017 历史 UTC 标签和 GPS−UTC offset。
4. 当前 Bulletin C 72 记载 TAI−UTC=37 s（GPS−UTC 名义整数 offset=18 s）从 2017-01-01 起 until further notice。连续 UTC 的定义要求连续且与 TAI 同速，但 Resolution 5 摘要本身没有给出接收机 NAV-TIMELS 的字段迁移/新 firmware 行为。故 `GPS−UTC=18 s forever` 暂不得作为未复核的硬编码事实；需核对 BIPM/IERS 后续实施说明、GNSS 服务提供方接口与目标 ZED-F9P 固件，再把 post-transition offset policy 标为 VERIFIED。
5. 不得因制度切换删除 `NAV-TIMELS`、`validCurrLs` 或 receiver raw evidence。接收机协议/固件可能继续提供兼容字段，也可能改变事件字段语义；以 MON-VER、固件版本和实测原始 UBX frame 建立 capability matrix。
6. GNSS GPS system time 自身连续；UTC regime 变化影响的是 GPS↔UTC 映射策略，不得重置 GPS week/TOW、LIO/EKF/ROS monotonic clock 或导航状态估计 epoch。
7. 需要天文方位/太阳位置/地球旋转等 UT1 语义的导航子模块，必须另设 EOP/UT1−UTC 数据链与 stale/conflict gate。普通机器人局部/全局定位不得误把允许的 |UT1−UTC| 增长当作 GPS/UTC 时间同步误差。
8. IERS Bulletin C 的数据模型需加入 `policy_regime`、`announcement_scope`、`valid_until`、`superseded_by_resolution`。Bulletin C 72 的 no-leap-second 结论只对应 2026-12-31，不应延伸为 2027 年 6 月或之后无期限结论。

新增验证合同：
- `UTC-REGIME-001`：围绕 Resolution 5 指定的 2027-05-20 制度生效边界，UTC/GPS 映射必须无人工一秒跳变；在权威实施文件未给出日内精确时刻前，测试不得擅自假定 00:00:00Z，而应以 policy fixture 中经审核的 effective instant 参数化；实际 offset 采用经审核 policy fixture，不由测试硬编码推断。
- `UTC-REGIME-002`：旧 regime 历史正闰秒 2016-12-31 23:59:60 仍可正确表示；不得因新制度而删除历史 leap-second support。
- `UTC-REGIME-003`：仅有 Bulletin C 72 不足以证明 2027-05-20 之后的 regime policy；缺少已审核 Resolution 5 policy record 时，mapper 对未来 regime 返回 POLICY_UNVERIFIED，而不是沿用旧表无期限外推。
- `UTC-REGIME-004`：continuous UTC 下 UT1−UTC 可非零且随地球自转变化；UT1 数据过期不得污染 GPS−UTC mapper 状态，但 UT1-dependent consumer 必须 fail closed。
- `UTC-REGIME-005`：模拟 NAV-TIMELS 在 regime 边界前后仍报告 18s、报告未知来源、或 validCurrLs=false；分别验证已审核 policy 与 receiver evidence 的一致/冲突/缺失处理，不允许 receiver 单字段静默覆盖权威政策。
- `UTC-REGIME-006`：验证新旧 policy artifact 并存、签名/哈希/审核状态/生效日期，启动时只加载唯一有效版本；重复、重叠或缺口均拒绝。

当前证据边界：Resolution 5 已正式通过，continuous UTC 生效日期为 2027-05-20；Bulletin C 72 已明确 2026-12-31 无闰秒及 UTC−TAI=−37 s。尚未审计 BIPM/IERS 的完整 operational implementation package、各 GNSS constellation 的 post-transition interface、ZED-F9P 固件兼容性和实际 UBX 输出。不得声称目标设备已支持 continuous UTC；相关硬件与服务端证据仍为 `需要实物/授权，未完成`。

#### 4.18 GPS / Galileo / BeiDou 广播 UTC 参数接口对照（首轮，2026-11-25）

本节只记录已取得官方 ICD 的可核验字段和算法边界。它不是对 2027-05-20 后星座运营策略的确认；当前检索到的 GPS L5 ICD 为 2022-08 Revision J、Galileo OS SIS ICD 为 2025-11 Issue 2.2、BeiDou B3I ICD 为 2018-02 Version 1.0，均早于 CGPM Resolution 5 (2026)。

##### 4.18.1 对照矩阵

| 星座 / 官方接口文档 | 系统时间→UTC 广播参数 | 闰秒事件字段 / 算法特点 | 独立系统时间关系 / UT1 | 证据限制 |
|---|---|---|---|---|
| GPS L5 CNAV — IS-GPS-705J (2022-08) | Message Type 33；UTC reference 明确为 UTC(USNO)；A0-n/A1-n/A2-n 多项式，ΔtLS，tot，WN_ot | ΔtLSF、WNLSF、DN；DN=1..7；参考周数 WN_ot/WNLSF 需结合截断周数恢复；参数至少每 3 天更新（控制段可上传时），否则精度随时间退化 | Message Type 32 提供 EOP；可由 UT1−GPS 与 GPS−UTC 得到 UT1−UTC；Type 33 提供 GPS↔UTC | 规范版本早于 Resolution 5；未证明 Type 33/32 已针对 continuous UTC 更新；仅覆盖 L5 CNAV，不能代表所有 GPS 民用信号代际 |
| Galileo OS SIS ICD Issue 2.2 (2025-11) | GST−UTC conversion；A0/A1 多项式、ΔtLS、t0t、WN0t | ΔtLSF、WNLSF、DN；WN0t/WNLSF 为截断 8-bit 周数；DN=1..7；闰秒边界前后 ±6h 使用单独算法分支 | 独立广播 GST−GPS（GGTO）参数 A0G/A1G/t0G/WN0G；不应把 GST 直接视作 GPS 时间 | 规范早于 Resolution 5；未证明 GST−UTC 广播字段/事件字段已切换到 continuous UTC；具体 Galileo UTC(k) realization 的长期实施细节仍需进一步权威来源核对 |
| BeiDou B3I D1/D2 — BDS-SIS-ICD-B3I-1.0 (2018-02) | BDT−UTC；A0UTC/A1UTC、ΔtLS | ΔtLSF、WNLSF（8-bit modulo 256）、DN；按闰秒生效前/事件窗口/生效后三种算法分支；ICD 声称广播参数可使 UTC 计算误差不超过 1 μs | BDT−GPS 参数 A0GPS/A1GPS 在该 ICD 标注为“Not broadcast temporarily”；B3I 文档不应被用来推断 B1C/B2a/B2b CNAV 的字段完全相同 | 文档年代较早且仅覆盖 B3I D1/D2；BDS-3 CNAV 及新版本官方 ICD 尚需单独核验；不代表 BDS 所有公开信号 |

##### 4.18.2 直接影响架构的结论

1. **必须按 constellation + signal/message type 解码。** 建议抽象为 `GnssUtcBroadcastModel`，但每种 signal/message type 保留独立 parser、字段有效性、周数展开规则、参考历元和公式版本；禁止一个通用 `A0/A1/ΔtLS/WNLSF/DN` parser 直接吞入所有星座。
2. **先转换到系统连续时间，再求 UTC。** GPS/GST/BDT 各自的 week/TOW 是原生连续时间；UTC 是带广播偏移/历史事件的映射。每个 UTC offset 记录必须带 `constellation`、`signal_id`、`message_type`、`utc_reference_realization`、`reference_week/tow`、`validity/health`、`received_monotonic_ns`、`receiver_generation`、`source_revision`。
3. **多项式不能简化成固定整数秒。** GPS CNAV 有 A0/A1/A2；Galileo OS SIS 和 BDS B3I 所审计的版本有 A0/A1。实现须保留有符号定点系数、scale factor、reference week/TOW 和明确的计算顺序，避免浮点舍入或把 polynomial bias 当成 leap count。
4. **周数截断恢复是协议专属规则。** Galileo GST−UTC 使用 8-bit WN0t/WNLSF，BDS B3I WNLSF 为 modulo 256，GPS L5 CNAV 使用 13-bit WN_ot/WNLSF。必须用同消息/同历元约束恢复 full week，拒绝歧义周数，不可借用另一星座的 rollover 窗口。
5. **闰秒字段不是连续 UTC 实施证明。** 这些 ICD 都早于 Resolution 5，仍定义 ΔtLS/ΔtLSF 事件语义。Resolution 5 生效后，旧字段可能继续为兼容/历史用途保留，也可能由后续 ICD 或服务端更新定义新语义；在官方修订/服务公告与接收机原始导航数据验证前，不能假定事件字段停止广播、永远固定，或继续按旧语义解释。
6. **UT1 必须单独建模。** GPS L5 Type 32 是明确证据：EOP/UT1 信息与 GPS−UTC 转换是相关但不同的数据面。连续 UTC 之后 |UT1−UTC| 允许增长至 3600 s，不意味着 EOP 可丢弃；普通导航时钟 mapper 不应把 UT1 缺失判成 GPS−UTC 失效，但地固/惯性变换与天文应用必须有独立 EOP freshness/conflict gate。
7. **不要跨星座多数表决。** 同一接收机同一 epoch 收到多星座 UTC offset 时，先按各自 UTC reference realization 和系统间偏差模型归一化；在参考实现/偏差未确认时，标记 `CROSS_REALIZATION_UNRESOLVED`，不得把 GPS UTC(USNO)、Galileo GST−UTC 广播所指向的 UTC、BeiDou BDT−UTC 广播所指向的 UTC 默认视为同一物理 realization；Galileo/BeiDou 对应 UTC(k) 的具体实现标识与溯源仍须由各自权威时间服务文件确认。
8. **接收机 API 能力是独立 Gate。** 本项目目标 ZED-F9P 是否提供原始 GPS/Galileo/BeiDou UTC 广播参数、保留了哪些 signal/message identity、是否提供 raw navigation page、是否在固件中融合/过滤参数，均未从本次 ICD 审计得到证明。UBX-NAV-PVT/TIMEUTC/TIMELS 是 receiver output，不等价于原始星座广播字段；后续需 MON-VER、配置快照、原始导航页和输出日志建立映射。

##### 4.18.3 建议内部数据模型（设计草案）

- `GnssTimeScale`: `GPS`, `GST`, `BDT`；携带 full week + integer TOW ns + system-time epoch/version。
- `UtcOffsetPolynomial`: signed fixed-point `A0/A1/A2`（按消息 schema 允许字段）、scale factors、reference week/TOW、evaluation interval、uncertainty、raw bits。
- `UtcLeapEventBroadcast`: current/past ΔtLS、future ΔtLSF、effective week/day、raw truncated week fields、event-valid flag、message issue/health。
- `UtcReferenceRealization`: `UTC(USNO)`（GPS L5 Type 33 明确）/ Galileo GST−UTC 对应的 UTC realization identifier / BeiDou BDT−UTC 对应的 UTC realization identifier；后两者具体 UTC(k) 名称及溯源待权威服务文件确认，不得预设为 `UTC(NTSC)` 或某个固定组合。
- `Ut1EopBroadcast`: UT1−system-time 或协议规定的 EOP fields、xp/yp、epoch、validity、source/signal identity；独立于 UTC leap event cache。
- 每个对象保留原始消息字节/页、ICD revision、接收单调时钟、receiver session/generation、解析器版本和审核状态。

##### 4.18.4 新增验证合同

- `GNSS-UTC-IF-001`：GPS L5 Type 33 解析 A0-n/A1-n/A2-n、ΔtLS、tot/WN_ot、ΔtLSF/WNLSF/DN；检查 signed two's-complement、scale、13-bit week rollover、DN 1..7、消息更新超时。
- `GNSS-UTC-IF-002`：Galileo GST−UTC parser 验证 8-bit WN0t/WNLSF、DN 1..7、ΔtLS/ΔtLSF 和 ±6h 闰秒窗口分支；独立验证 GST−GPS GGTO，不允许误把 GST 当 GPS。
- `GNSS-UTC-IF-003`：BDS B3I D1/D2 parser 验证 A0UTC/A1UTC、ΔtLS、8-bit modulo-256 WNLSF、DN、三段式事件算法；验证 ICD 所述 1 μs 误差仅适用于其广播算法合同，不外推到整机同步精度。
- `GNSS-UTC-IF-004`：同一 UTC 标签由 GPS/GST/BDT 分别转换时，先应用各自系统时间偏移与 reference realization；没有跨 realization 偏差证据时必须返回 CROSS_REALIZATION_UNRESOLVED，不做平均。
- `GNSS-UTC-IF-005`：模拟 Resolution 5 生效前后，旧 ICD 中 ΔtLSF 不变/无未来事件/字段无效三种情况；不得据此自行推断 continuous UTC offset 或合成 ±1 s leap event。
- `GNSS-UTC-IF-006`：GPS Type 32 EOP 与 Type 33 UTC 参数独立更新/过期；UT1 consumer 的 stale gate 与普通 GPS−UTC mapper 状态隔离。
- `GNSS-UTC-IF-007`：同一接收机多星座数据有不同 message issue、接收时刻、generation 或信号来源时，不跨 epoch 拼接参数。
- `GNSS-UTC-IF-008`：目标接收机无法输出原始星座广播页时，标注 receiver abstraction boundary；不得声称已验证上游 ICD 字段到 UBX 输出的逐字段映射。

##### 4.18.5 证据与后续缺口

- GPS：本节采用 USCG NAVCEN 发布的 IS-GPS-705 Revision J（2022-08），章节 20.3.3.6（Type 33）及 20.3.3.5/Type 32 EOP；必须另审 IS-GPS-200 对 L1 C/A LNAV 的当前有效版本，以及 L1C/CNAV-2 规范，不能用 L5 Type 33 代表全部 GPS 民用信号。
- Galileo：采用 European GNSS Service Centre OS SIS ICD Issue 2.2（2025-11），§5.1.7 GST−UTC 与 §5.1.8 GST−GPS；需继续审查后续正式 issue、服务公告及 Galileo 时间参考实现维护文档。
- BeiDou：采用 China Satellite Navigation Office BDS-SIS-ICD-B3I-1.0（2018-02），§5.2.4.18–5.2.4.20；需取得并审计 B1C、B2a/B2b 与 BDS-3 CNAV 的官方当前 ICD，当前不把第三方镜像当作官方修订依据。
- GLONASS 尚未纳入本节对照；需独立审查官方 ICD 的 UTC(SU) 时间标记、日历字段、闰秒/UTC 参数和 UT1/EOP 广播合同，不从 GPS/Galileo/BDS 规则推断。
- 2027-05-20 之后的实际导航电文字段/服务政策、星座控制段更新、接收机固件兼容性和 ZED-F9P 原始输出均未完成验证，标记为 `需要官方后续规范/服务公告/实物或授权，未完成`。

本节为官方 ICD 静态审计和设计合同，不是实际导航电文抓取、解码器实现或设备验证；未运行测试。

#### 4.19 GPS L1 C/A / L1C 与 GLONASS UTC 接口补充审计（2026-11-25）

##### 4.19.1 GPS L1 C/A LNAV（IS-GPS-200N）

官方 GPS.gov ICD 页面列出 IS-GPS-200N（2022-08-22）及 IRN-IS-200N-003（2024-01-19）、IRN-IS-200N-004（2026-06-16）。最新 IRN-004 的变更主题是 Civil Integrity Support Message（ISM）格式；其正文修改/新增集中于 CNAV MT-40 / ISM，不含 UTC 章节修改。因此本轮仍以 IS-GPS-200N §20.3.3.5.1.6、§20.3.3.5.2.4 作为 LNAV UTC 基线，同时将 IRN-004 作为版本组合的一部分记录。

- Subframe 4 Page 18 包含 GPS↔UTC 参数。
- Table 20-IX：A0 为 32-bit signed two's-complement，LSB 2^-30 s；A1 为 24-bit signed，LSB 2^-50 s/s；ΔtLS 为 8-bit signed seconds；t_ot 为 8-bit × 2^12 s；WN_t 与 WN_LSF 均为 8-bit；DN 为 8-bit，范围 1..7；ΔtLSF 为 8-bit signed seconds。
- 参数更新要求：控制段能够上传卫星时，至少每六天更新一次；无法上传时，广播 UTC 参数精度随时间退化。
- WN_t、WN_LSF 以及当前 WN 均存在截断/周数回卷问题；必须按 IS-GPS-200 §3.3.4(b) 恢复 full week。不得套用 L5 CNAV 的 13-bit 周数处理。
- UTC 算法包含事件前、事件窗口、事件后不同分支；事件窗口为生效时刻前后各六小时。事件后使用 ΔtLSF，且控制段后续上传需保持 UTC 时间尺度连续。
- 证据：官方 PDF IS-GPS-200N §20.3.3.5.1.6、Table 20-IX、§20.3.3.5.2.4；GPS.gov 当前 ICD 页面列出 2026-06 IRN-004。IRN-004 不是 UTC 接口修订，不能把“最新 IRN”误解为 continuous UTC 兼容确认。

##### 4.19.2 GPS L1C CNAV-2（IS-GPS-800J）

L1C 的 UTC 参数不是简单地从 L1 C/A LNAV 复用相同位宽：

- Subframe 3 Page 1 广播 UTC(USNO) 与 GPS Time 的关系，并包含未来/近期 ΔtLSF、WNLSF、DN。
- Table 3.5-3：A0-n 16-bit signed / 2^-35 s；A1-n 13-bit signed / 2^-51 s/s；A2-n 7-bit signed / 2^-68 s/s²；ΔtLS 8-bit signed；t_ot 16-bit / 2^4 s；WN_ot 与 WNLSF 各 13-bit；DN 4-bit（1..7）；ΔtLSF 8-bit signed。
- 控制段能够上传时，GPS↔UTC(USNO) 参数至少每三天更新一次。
- Subframe 3 Page 2 同时承载 GGTO 与 EOP。GGTO 是 GPS 与其他 GNSS 系统时间的偏差模型；规范要求用户还须应用各系统定义中的整数秒差。EOP 用于 ECEF↔ECI 转换。
- 因此 L1C 可以同时提供 UTC、跨星座系统时间偏差和 EOP，但它们是三个独立数据合同，不能合并为一个 offset 字段。
- 证据：官方 IS-GPS-800J（2022-08-01）§3.5.4.1.1、§3.5.4.1.1.1、Table 3.5-3、§3.5.4.2、Table 3.5-4、Table 3.5-5。该版早于 Resolution 5。

##### 4.19.3 GLONASS FDMA legacy navigation message（ICD Edition 5.1）

本轮获得 GLONASS ICD 5.1 英文版的 UNAVCO 下载入口，但 PDF 附件服务以 application/octet-stream 返回，当前检索工具不能读取原件。因此以下内容按字段级初审处理，引用的二手解析器/接收机文档只作交叉线索，不能代替官方 ICD 原文验收。

已找到的实现/设备资料交叉描述下列传统 GLONASS-M 导航字段：
- τC：GLONASS time scale 相对 UTC(SU) 的修正量（按 ICD 定义的参考日/历元解释）。
- τGPS：GPS 与 GLONASS time 的关系参数。
- N4：自 1996 年起的四年周期编号；NA：四年周期内的日历日编号。
- B1/B2：用于计算 UTC(SU) 与 UT1 的差值模型；传统字段通常以日初参数和日变化率表达。
- KP：预告当前季度末 UTC(SU) 计划进行的 ±1 s 修正。接收机厂商文档对 KP 取值的解释为 00 无本季度更新、01 季末 +1 s、11 季末 −1 s；此枚举仍须回到 ICD 原文逐字段核验。
- 传统 GLONASS 时间与 UTC(SU) 关联，UTC(SU) 本身是带闰秒阶跃的时间尺度；GLONASS 广播时间标记/历元处理与 GPS 连续时间模型不同。不得把 GLONASS time 直接当 GPS time，也不得用 GPS−UTC 整数偏移公式处理 GLONASS time。
- GLONASS CDMA General Description Edition 1.0（2016）的可检索副本进一步描述 GLONASS time scale 依赖 UT1 与 UTC(SU)，并给出向 UTC(SU)、UT1、TAI、GPS 的转换数据面；其 Appendix B/H 说明 UT1 与 UTC(SU)、UTC(SU) 与 TAI 的转换。该副本不是本轮已验证的官方分发渠道，作为后续核验线索而非最终规范证据。

**2027 continuous UTC 迁移的重大风险：**传统 GLONASS ICD 5.1 / CDMA 2016 资料定义了 UTC(SU) leap-step、KP 事件通知和相关转换语义；这些文档早于 2026 CGPM Resolution 5。不能假设 2027-05-20 后 UTC(SU)、GLONASS time、KP、τC、B1/B2 或导航时间标记会自动变成 continuous UTC 语义。必须取得 Russian Federation State Standard / UTC(SU) 维护机构、Russian Space Systems / GLONASS control segment 的正式迁移说明与新版 ICD；在此之前 GLONASS UTC 映射必须标记 POLICY_UNVERIFIED，不能作为 continuous UTC regime 的权威主时钟来源。

##### 4.19.4 架构影响与验证合同

1. GPS L1 C/A、GPS L1C、GPS L5 各自保留 signal-specific message parser 和字段位宽/scale schema；不能只按“GPS”选择一个通用 parser。
2. GPS L1 C/A 与 L1C 虽都声明 UTC(USNO)，但数据更新周期、周数字段位宽、A0/A1/A2 系数和消息结构不同；跨信号融合前需按同一 GPS epoch、issue/health 和有效时间验证。
3. GPS L1C GGTO、EOP、UTC 数据必须独立缓存和过期；EOP 不得因 UTC offset 更新而被错误刷新。
4. GLONASS 需区分 FDMA legacy GNAV 与 CDMA navigation message 两条接口；在原始 ICD 未审计前，不共用数据结构，也不推断两者字段相同。
5. UTC(SU) 与 UTC 的 realization/政策关系必须有独立 policy artifact；Resolution 5 对 UTC 的制度决定不能自动证明 UTC(SU) 的国内实现与 GLONASS control segment 已完成相同迁移。
6. GLONASS 的 UT1/EOP 不能和 GPS L5/L1C EOP 或 IERS EOP 简单多数表决；须统一参考历元、符号约定、参考 realization 和有效区间后再做一致性检查。

新增验证合同：
- GNSS-UTC-IF-009：GPS L1 C/A LNAV 的 8-bit WN_t/WN_LSF、A0/A1、ΔtLS/ΔtLSF、DN 及 ±6h event window；覆盖 GPS week rollover。
- GNSS-UTC-IF-010：GPS L1C CNAV-2 的 A0-n/A1-n/A2-n、13-bit WN_ot/WNLSF、t_ot scale、signed fields；不得误用 L1 C/A 或 L5 的位宽。
- GNSS-UTC-IF-011：GPS L1C Page 2 的 GGTO 与 EOP 独立更新/过期；系统整数秒差不能被 modulo-one-second GGTO 隐藏。
- GNSS-UTC-IF-012：IS-GPS-200N + IRN-IS-200N-004 组合验证，确认 IRN-004 的 ISM 修改不被误读为 UTC 字段更新。
- GNSS-UTC-IF-013：GLONASS GNAV τC/τGPS/N4/NA/B1/B2/KP 原始字段保存；未确认 ICD 版本/字段有效性时 fail closed。
- GNSS-UTC-IF-014：GLONASS FDMA 与 CDMA 分别解析，禁止未经规范证明的字段复用。
- GNSS-UTC-IF-015：continuous UTC policy 生效后，GLONASS UTC(SU)/KP/τC 仍缺少权威迁移证据时返回 POLICY_UNVERIFIED，不把旧 KP 事件语义推演为未来规则。
- GNSS-UTC-IF-016：跨 GPS/GLONASS 比较前验证时间尺度、UTC realization、epoch 和 UT1 符号约定；任一未确认返回 CROSS_REALIZATION_UNRESOLVED。

##### 4.19.5 证据边界与待办

- GPS L1 C/A：IS-GPS-200N（2022-08）及 IRN-IS-200N-003（2024-01）、IRN-IS-200N-004（2026-06）已取得官方发布页面；本轮读取了 IS-GPS-200N UTC 章节与 IRN-004 正文。IRN-004 仅处理 ISM 相关变更，未发现 UTC 字段修改。
- GPS L1C：IS-GPS-800J（2022-08）官方 PDF 已读取 UTC、GGTO、EOP 章节。
- GLONASS FDMA：ICD 5.1 英文版入口已定位，但原始 PDF 下载/读取失败；字段细节尚未完成官方原文逐字段复核。
- GLONASS CDMA：Edition 1.0（2016）资料目前仅作为研究线索，需取得 Russian Space Systems 官方分发原件。
- GLONASS UTC(SU) 与 CGPM Resolution 5 的国内实施协调、控制段广播字段迁移、2027-05-20 后导航电文行为均未确认。
- ZED-F9P 是否输出 GLONASS raw navigation data、KP/τC/B1/B2、GPS L1C raw page、GPS L1 C/A raw subframe，仍需目标固件 MON-VER、配置与真实 raw frame 验证。

本节是静态 ICD/公开资料审计与设计合同，不是实时导航电文采集、解码器实现、接收机验证或实机测试；以上测试合同均未运行。

##### 4.19.6 俄罗斯 2026 年国家无线电导航计划：从“未知”修正为“路线已公开、接口未验收”

本轮找到并读取《俄罗斯联邦无线电导航计划》（由俄罗斯工业和贸易部 2026-01-30 第 362 号命令批准）的公开全文副本。该计划第 3.2.2 节及 3.2.2.14.1 节对 UTC 连续化后的 GLONASS 时间架构给出比旧 ICD 更明确的国家层面路线：

- 将 GLONASS 系统时间的基准描述为俄罗斯国家 UTC(SU)，并说明传统 UTC/UTC(SU) 以整数秒校正的旧制度。
- 为兼容现有用户设备，要求同时维持两种时间尺度：一是用于现有设备向后兼容的“可校正秒”旧时间尺度；二是改变 dUT 约束后的连续 UTC 时间尺度。
- 对信号分支作了明确区分：现有 FDMA（频分）导航信号应继续传递旧的、带校正秒的时间信息；新引入的 CDMA（码分）信号及其用户设备应传递旧时间尺度和重新定义后的连续 UTC 时间尺度。
- 时间频率保障体系也应支持两种时间尺度，并建设 UTC 与 UTC(SU)、GLONASS 中央同步器时间等尺度间的比对能力。

**证据等级与时间适用性：**该文件是政府部门批准的国家无线电导航规划文本，属于国家规划/政策路线证据，不是 GLONASS 信号 ICD、控制段运行通告、CDMA 广播字段定义或接收机固件行为证据。可检索全文目前来自 Garant 法律信息平台副本；本轮未从 Минпромторг 官方网站或俄罗斯官方法律公布门户取得同一文件的原始发布页面/签章附件。因此将其记录为强政策线索（E1-policy candidate），正式来源链仍待官方门户复核。更重要的是，命令日期为 2026-01-30，早于 2026-10-15 的 CGPM Resolution 5 (2026)；它可证明俄罗斯规划文本已经预先考虑双尺度架构，但不能证明其已按 2026 Resolution 最终条款完成同步修订或已投入运行。

该发现修正 §4.19.3 的过宽表述：不能再说“俄罗斯尚无公开的 UTC 连续化迁移路线”；准确结论应为“国家无线电导航计划公开提出 FDMA 保留旧可校正尺度、CDMA 双尺度并行的兼容路线，但广播字段、实施时序、UTC(SU) 国内 realization、旧/新时间尺度的精确映射和接收机部署仍未由 ICD/正式运行文件证实”。因此：
- 对 GLONASS FDMA legacy GNAV：继续将 KP、τC 等解释限定在旧 leap-step/兼容尺度合同内；不得把 KP=00 解释为连续 UTC 无事件，也不得将旧 τC 直接当作连续 UTC 偏差。
- 对 GLONASS CDMA：架构上预留 legacy-corrected 与 continuous-UTC 两套独立 typed time-scale records / validity / epoch / source / version；在新 ICD 字段映射取得前，parser 状态必须为 UNSUPPORTED_SPEC 或 POLICY_UNVERIFIED，不允许从 FDMA 字段猜测 CDMA 字段。
- 对接收机：ZED-F9P 的旧 FDMA 输出与未来 CDMA 输出能力必须分别核实；不能因芯片支持某 GLONASS 信号频段就推断其输出两套时间尺度，也不能推断固件会暴露原始导航页。
- 对跨星座：continuous UTC 与 GPS/Galileo/BeiDou 系统连续时间映射仍需各自广播参数和 realization provenance；GLONASS legacy time 不能加入 UTC 主时钟融合/投票。

新增验证合同：
- GNSS-UTC-IF-017：FDMA legacy-only。输入旧 GNAV/KP/τC，只能生成 LEGACY_STEP_REALIZATION；不能自动构造 continuous UTC。
- GNSS-UTC-IF-018：CDMA dual-scale contract。只有经版本化 ICD 明确映射并验证的双尺度字段，才可分别生成 LEGACY_STEP_REALIZATION 与 CONTINUOUS_UTC_REALIZATION；缺少任一字段或有效标志时分别降级/拒绝。
- GNSS-UTC-IF-019：dual-scale continuity。构造 2027-05-20 前后测试向量，验证连续 UTC 支路无 ±1 s 人工跳变，旧尺度支路按兼容合同独立处理；不假定该日期的具体广播生效瞬间。
- GNSS-UTC-IF-020：receiver capability fence。MON-VER、信号跟踪状态、raw navigation page availability、解码字段版本和时间尺度 source 必须全部满足能力合同；不得仅凭型号或频段宣称 dual-scale support。

**本轮来源与限制：**
- 俄罗斯工业和贸易部命令 2026-01-30 №362 及所附《Радионавигационный план Российской Федерации》公开全文副本（Garant 法律信息平台；尚待官方发布门户回读）。
- Rosstandart 官方页面确认 ГОСТ Р 8.930-2016 仍标记为“Действует”，其范围为通过 GLONASS 传递 UTC(SU)；该标准 2016 年制定，不能替代 2026 年迁移实施规范。
- GLONASS ICD L1/L2 Edition 5.1 (2008) 的 UNAVCO 镜像本轮已可读取：§3.3.3 明确 GLONASS time 与 UTC(SU) 的传统关系；§4.5 定义 τc、τGPS、B1/B2、KP；Appendix 2 定义 leap-second 期间旧/新 UTC 与旧/新星历并用的接收机行为。该副本有签署页，但 ICD 自述官方分发机构为 Russian Institute of Space Device Engineering；UNAVCO 镜像不等于官方分发链。
- 未取得 GLONASS CDMA 新版 ICD/正式 Interface Control Document，未确认 L1OC/L2OC/L3OC 的双尺度广播字段和版本号；未取得控制段实施通知；未读取 ZED-F9P 真实 raw frames。

因此本节把政策结论从“迁移政策完全未确认”细化为“国家规划已提出双尺度迁移/兼容路线，但信号级规范与运行实现未确认”。所有 GNSS-UTC-IF-017..020 仅为测试合同，未实现、未运行。
