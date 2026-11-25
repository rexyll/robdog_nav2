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
- `status`：`VALID_CONFIRMED`、`VALID_UNCONFIRMED`、`AMBIGUOUS_INVALID`；
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
| VALID-009 | valid_wkn=false but UTC anchor otherwise valid | do not infer week from flag; derive week only from validated UTC+LS and mark flag discrepancy per policy |
| VALID-010 | NAV-STATUS tow_set/wkn_set false | do not treat iTOW/week status as valid; absolute UTC policy decides separately, but no protocol consistency claim |

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

