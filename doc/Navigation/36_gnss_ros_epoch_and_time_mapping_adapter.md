# GNSS ROS 四消息历元缓冲与 receiver-time mapping adapter

日期：2026-11-26  
分支：nav

## 1. 目的

固定版 u-blox ROS driver 的四类 topic（NAV-PVT、HPPOSLLH、NAV-COV、NAV-STATUS）到达顺序不保证与 receiver iTOW 顺序一致；同时 source_timestamp_ns/source_clock_id 必须由独立 receiver-time mapper 提供。新增纯 Python UbxRosEpochAdapter，在时间映射尚未到达时有界缓存已解码 records。

## 2. 数据合同

UbxSourceTimeMapping 包含：
- i_tow_ms：精确对应的 receiver epoch；
- source_timestamp_ns：由 validated GPS week/TOW mapper 生成的 receiver-time monotonic epoch；
- source_clock_id：包含 receiver boot generation 和 GPS week 的明确时钟代次；
- valid：GPS source mapping 是否有效；
- utc_stamp_valid + utc_seconds/utc_nanoseconds：UTC 映射独立有效位；source GPS time 有效不自动代表 UTC/leap-second mapping 有效；
- receive_monotonic_ns：mapper callback 的 host monotonic receive time。

适配器只在同一 iTOW 的四条记录和 valid mapping 全部齐备时调用 UbxFourMessageEpochJoiner。输出同时携带 GnssObservation 和 time mapping。UTC stamp 无效时可用于质量评估的 source-time observation 不应被转成带伪造 ROS timestamp 的 NavSatFix。

## 3. Generation 与缓存策略

- 必须先 begin_generation(source_clock_id)；未知 generation 或未经确认的 week/boot 变化会被拒绝。
- 四类记录和 mapping 可任意先后到达。
- 每个 iTOW 的缓存数量受 max_pending_epochs 限制；按 LRU 淘汰。
- 超过 max_pending_age_ns 的 incomplete epoch 被 retire，后续碎片不能复活。
- 完全相同的重复记录/mapping 忽略；冲突重复会 retire 整个 epoch 并报错。
- invalid mapping retire 该 iTOW；不会等待后续 record 重新激活。
- joiner 最终仍验证四消息 iTOW 精确相等、receive monotonic skew、quality flags、covariance、source timestamp 严格递增。
- reset 后必须重新 begin_generation。

## 4. 测试

新增 test_ros_epoch_adapter.py 覆盖：
- records 先于 mapping；
- mapping 先于 records；
- invalid mapping fail-closed；
- 未授权 source_clock_id 变化；
- 冲突重复；
- LRU capacity 与 stale expiry；
- UTC stamp invalid 但 GPS source mapping valid 的分离语义；
- source mapping conflict；
- reset/generation fence。

## 5. 未完成项

此模块不产生 time mapping，也不实现 rclpy subscriptions、NavSatFix publisher 或 diagnostics node。固定 driver 当前没有 NAV-TIMEGPS/NAV-TIMELS，因此真实 mapper 仍缺 GPS week/leap-second source。必须先实现/接入有效 NAV-TIMEGPS 与 NAV-TIMELS 数据源，并通过 week rollover、leap-second、receiver reset 与 monotonicity tests，才能将 valid mapping 输入本 adapter。
