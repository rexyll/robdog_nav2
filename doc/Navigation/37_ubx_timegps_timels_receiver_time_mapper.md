# UBX-NAV-TIMEGPS / NAV-TIMELS receiver time mapper

日期：2026-11-26  
协议参考：u-blox F9 HPG 1.32 Interface Description, UBX-22008968, sections 3.15.25–3.15.26.

## 1. 解析合同

NAV-TIMEGPS（class 0x01, id 0x20）固定 payload 16 bytes：
- iTOW U4 @0
- fTOW I4 @4, range +/-500000 ns
- week I2 @8
- leapS I1 @10
- valid X1 @11: towValid bit0, weekValid bit1, leapSValid bit2
- tAcc U4 @12

NAV-TIMELS（class 0x01, id 0x26）固定 payload 24 bytes：
- iTOW U4 @0, version U1 @4 (version 0)
- srcOfCurrLs U1 @8, currLs I1 @9
- srcOfLsChange U1 @10, lsChange I1 @11
- timeToLsEvent I4 @12
- dateOfLsGpsWn U2 @16, dateOfLsGpsDn U2 @18
- valid X1 @23: validCurrLs bit0, validTimeToLsEvent bit1

仅当前 leap offset 的 validCurrLs 参与 UTC 转换；validTimeToLsEvent 是未来/最近 leap event 信息，不替代 current leap validity。

## 2. GPS source time

只有 TIMEGPS towValid + weekValid 都为 true 才生成 source timestamp。计算：
- rawTowNs = iTOW_ms * 1,000,000 + fTOW_ns
- 将 fractional boundary carry/borrow 规范化到 GPS week；
- source_timestamp_ns = normalized_week * 604800e9 + normalized_tow_ns；
- source_clock_id = receiver_session_id + gps-week。

该时间是从 GPS epoch 起算的连续 GPST nanoseconds，不是 host receive time，也不是 Unix UTC。week rollover 会改变 source_clock_id，调用方必须显式 begin_generation，不能让旧 week 的 pending epoch 跨代配对。

## 3. UTC leap mapping

UTC stamp 只有在以下条件全部成立时才生成：
1. TIMEGPS.leapSValid = true；
2. TIMELS.validCurrLs = true；
3. srcOfCurrLs 在应用显式 trusted_leap_sources allowlist 中；
4. TIMEGPS.leapS 与按来源归一化后的 TIMELS.currLs 一致；source-adjusted GPS-UTC offset 必须落在 0..127 的可表示非负范围内。

默认 trusted_leap_sources 为空；来源 0 是 firmware hardcoded default，官方文档明确提示可能过时，因此代码禁止将其加入 allowlist；255 Unknown 永不允许加入 allowlist。来源 4 BeiDou 报告 BDS-UTC，官方文档说明其比 GPS-UTC 少 14 秒，因此比较/UTC 换算前加 14 秒。其他来源不自动加入 allowlist，必须由设备配置和系统验证决定。

NAV-TIMELS 与 NAV-TIMEGPS 按 exact iTOW 配对；没有 TIMELS 时不产生 mapping。NAV-TIMEUTC 不作为替代 leap source。GPS source timestamp 可以有效而 UTC stamp 无效；这种情况下 mapping.valid=true、utc_stamp_valid=false，下游可进行 GPST epoch pairing，但不得伪造 NavSatFix UTC header。

UTC 时间戳使用 POSIX/Unix 秒表示，无法表达 23:59:60 这个独立 civil label。若 TIMELS validTimeToLsEvent=true，mapper 在 timeToLsEvent 为 -1/0/+1 秒的保守 guard window 内将 utc_stamp_valid 置 false；若 event flag 无效，只省略 event countdown，不单独否定仍有效的 current leap offset；lsChange 的 -1/0/+1 语义范围仅在 validTimeToLsEvent=true 时强制校验，避免无效 future-event 字段污染当前 offset。GPST source timestamp 仍可保持 valid/单调。真正的 sec=60 civil representation 与 transition evidence API 仍需单独实现。

## 4. 缓存与会话

- TIMEGPS/TIMELS 可任意先后到达；
- exact iTOW pair、pair receive skew、pending age、LRU capacity 均受限；
- conflicting duplicate retire 整个 iTOW；
- receiver_session_id 由 DeviceSession generation 提供，设备重启必须 begin_session；
- source_clock_id 包含 GPS week；week 变化要求 ROS epoch adapter 显式 begin_generation；
- time mapper 使用 RLock 串行化并发 callbacks。

## 5. 测试

新增测试覆盖：
- 两种 payload 的长度、offset、signed/unsigned 和 version；
- valid flags 与 iTOW/fTOW/week 范围；
- exact iTOW pair，任意到达顺序；
- source allowlist 默认 fail-closed，且 source 0 不可显式 allowlist；\n- current leap validity 与 event validity 独立；仅当 event validity 有效时应用 ±1 s leap-transition guard，并对无效 event countdown 输出 null；
- TIMEGPS leapSValid、TIMELS validCurrLs 与 offset cross-check；
- BeiDou source +14 秒归一化；
- fractional TOW week boundary carry/borrow；
- GPS week rollover clock ID；
- invalid GPS flags、stale/capacity、conflicting duplicate、session reset、并发 callbacks。

CI 验收：Actions run `45756478736`，head `caced3828e9c5a7116c32211d6b422d285099907`，Humble/Jammy 与 Jazzy/Noble 全部 success；两个环境各 121 tests、0 failures。该测试覆盖 synthetic payload/mapper，不是设备验收。\n\n## 6. 未完成项

该 mapper 目前由纯 Python payload bytes 调用。尚未从 GnssUsbStream raw Frame queue 或真实 ublox_dgnss USB stream 接入；尚未验证目标 F9P 固件版本、实际 TIMELS source、输出频率、week rollover、leap event 和接收器重启。trusted_leap_sources 必须在目标设备证据明确后配置，不得默认启用。
