# UBX-NAV-TIMEGPS / NAV-TIMELS receiver time mapper

日期：2026-11-26  
协议参考：
- u-blox ZED-F9P Interface Description UBX-18010854 R06 / protocol 27.10；
- u-blox F9 HPG 1.32 Interface Description UBX-22008968，sections 3.15.25–3.15.26。

目标设备 MON-VER 和实际固件对应的 interface-description revision 尚未采集；上述两个文档只作为当前 parser profile 的结构依据，不代表目标设备版本已确认。

## 1. 单一 raw payload decoder

ubx_time_messages.py 是唯一 NAV-TIMEGPS/NAV-TIMELS raw payload decoder；ubx_time_mapper.py 不再重复解析 payload。

NAV-TIMEGPS（class 0x01, id 0x20）固定 payload 16 bytes：
- iTOW U4 @0
- fTOW I4 @4, range +/-500000 ns
- week I2 @8
- leapS I1 @10
- valid X1 @11: towValid bit0, weekValid bit1, leapSValid bit2
- tAcc U4 @12

NAV-TIMELS（class 0x01, id 0x26）固定 payload 24 bytes：
- iTOW U4 @0, version U1 @4 (version 0)
- reserved1[3] @5
- srcOfCurrLs U1 @8, currLs I1 @9
- srcOfLsChange U1 @10, lsChange I1 @11
- timeToLsEvent I4 @12
- dateOfLsGpsWn U2 @16, dateOfLsGpsDn U2 @18
- reserved2[3] @20
- valid X1 @23: validCurrLs bit0, validTimeToLsEvent bit1

Decoder 保留 raw validity/reserved bits、source IDs、signed 字段与 lifecycle context。Decoder 只做结构/范围检查，不将 validity 或 source 自动升级为 trusted clock。

## 2. Session / connection-generation fence

每个 decoder output 必须携带：
- process_session_id；
- connection_generation；
- host monotonic receive timestamp。

UbxReceiverTimeMapper.begin_session(process_session_id, connection_generation) 显式建立当前会话。add_timegps/add_timels 只接收带 context 的 immutable epoch record；context 与当前 session/generation 不一致时拒绝，不进入 pending cache。payload convenience API 也必须由调用方显式传入 callback 入口时捕获的 process_session_id/connection_generation，禁止在 callback dispatch 时用“当前 generation”给旧排队帧重新贴标签。

TIMEGPS/TIMELS 按 exact iTOW 配对；同时要求同一 process session、connection generation、receive-skew 和 pending-age 均符合限制。source_clock_id = process_session_id + connection_generation + normalized GPS week。设备 reconnect 或 GPS week rollover 后，调用方必须显式 reset/begin_session，并在 UbxRosEpochAdapter 调用 begin_generation；旧 generation 的 DDS queued callbacks 仍需要真实 ROS ingress drain/fence，纯 Python adapter 不替代该 executor 级屏障。

## 3. GPS source time

只有 TIMEGPS towValid + weekValid 都为 true 才生成 source timestamp：
- rawTowNs = iTOW_ms * 1,000,000 + fTOW_ns；
- 将 fractional boundary carry/borrow 规范化到 GPS week；
- source_timestamp_ns = normalized_week * 604800e9 + normalized_tow_ns。

该时间是从 GPS epoch 起算的连续 GPST nanoseconds，不是 host receive time，也不是 Unix UTC。source_clock_id 包含 process session、connection generation 和 normalized GPS week。

## 4. UTC leap mapping

UTC stamp 只有在以下条件全部成立时才生成：
1. TIMEGPS.leapSValid = true；
2. TIMELS.validCurrLs = true；
3. srcOfCurrLs 在应用显式 trusted_leap_sources allowlist 中；
4. TIMEGPS.leapS 与按来源归一化后的 TIMELS.currLs 一致；
5. source-adjusted GPS-UTC offset 落在 0..127 的可表示非负范围内。

trusted_leap_sources 默认为空；来源 0 是 firmware hardcoded default，代码禁止将其加入 allowlist；255 Unknown 永不允许加入 allowlist。来源 4 BeiDou 报告 BDS-UTC，协议说明其比 GPS-UTC 少 14 秒，因此比较/UTC 换算前加 14 秒。其他来源不自动加入 allowlist，必须由目标设备配置和系统证据决定。

NAV-TIMELS 与 NAV-TIMEGPS 按 exact iTOW 配对；没有 TIMELS 时不产生 mapping。NAV-TIMEUTC 不作为替代 leap source。GPS source timestamp 可以有效而 UTC stamp 无效；这种情况下 mapping.valid=true、utc_stamp_valid=false，下游可进行 GPST epoch pairing，但不得伪造 NavSatFix UTC header。

POSIX/Unix seconds 无法表达 23:59:60 这个独立 civil label。若 TIMELS validTimeToLsEvent=true，mapper 在 timeToLsEvent 为 -1/0/+1 秒的默认 guard window 内将 utc_stamp_valid 置 false；guard 可配置。若 event flag 无效，只省略 event countdown，不单独否定仍有效的 current leap offset；lsChange 的 -1/0/+1 范围仅在 validTimeToLsEvent=true 时校验。GPST source timestamp 仍可保持 valid/单调。真正的 sec=60 leap-aware civil representation 与 transition evidence API 仍需单独实现。

## 5. Bounded pairing and statistics

- TIMEGPS/TIMELS 可任意先后到达；
- exact iTOW pair、pair receive skew、pending age、LRU capacity 均受限；
- conflicting duplicate retire 整个 iTOW；
- invalid GPS week/TOW flags 不产生 mapping；
- source 0/255 不能成为 trusted source；
- RLock 串行化并发 callback；
- stats 记录 paired、invalid time、UTC valid/untrusted、expiry、capacity、duplicate conflict、pair skew、stale context rejection。

## 6. 测试与验收

测试覆盖：
- 两种 payload 的长度、offset、signed/unsigned、reserved bytes 与 version；
- process_session_id / connection_generation / monotonic receive context；
- valid flags 与 iTOW/fTOW/week 范围；
- exact iTOW pair，任意到达顺序；
- stale process session / connection generation callback 拒绝；
- reconnect flush 与 old callback 不跨 generation 配对；
- source allowlist 默认 fail-closed，source 0 不可显式 allowlist；
- current leap validity 与 event validity 独立；
- event flag 无效时不暴露 countdown，event 有效时 ±1s UTC guard；
- lsChange 仅在 event flag 有效时验证；
- TIMEGPS leapSValid、TIMELS validCurrLs 与 offset cross-check；
- BeiDou source +14 秒归一化；
- fractional TOW week boundary carry/borrow；
- GPS week rollover clock ID；
- invalid GPS flags、stale/capacity、conflicting duplicate、并发 callbacks。

此前 mapper payload decode 曾重复实现；现统一使用 ubx_time_messages.py，避免两套 offset/validity semantics 漂移。最新 source/session fence changes 已由 nav 分支 CI 验证后再标记。

## 7. 未完成项

该 mapper 目前仍由纯 Python decoded records/payload helpers 调用。尚未从 GnssUsbStream raw Frame queue 或真实 ublox_dgnss USB stream 接入；尚未验证目标 F9P 固件版本、实际 TIMELS source、输出频率、week rollover、leap event、reconnect 与接收器重启。trusted_leap_sources 必须在目标设备证据明确后配置，不得默认启用。真实 ROS MultiThreadedExecutor/DDS ingress barrier、NavSatFix publisher 和实机时间同步仍未验收。
