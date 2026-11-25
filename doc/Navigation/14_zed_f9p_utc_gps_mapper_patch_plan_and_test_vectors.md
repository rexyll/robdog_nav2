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
