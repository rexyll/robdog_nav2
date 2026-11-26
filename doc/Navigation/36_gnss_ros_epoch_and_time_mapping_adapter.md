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
- NAV-PVT、NAV-HPPOSLLH、NAV-COV、NAV-STATUS 四类 immutable records 均必须携带同一 `process_session_id`、`connection_generation`、`gps_week`；四路 ROS decoder 从 callback entry 的显式上下文接收这些字段，禁止只给 PVT 打 GPS week 标签。
- source_clock_id 必须精确匹配 `{process_session_id}:connection:{connection_generation}:gps-week:{gps_week}`；四路记录与 mapping 的 context 必须与 active generation 完全一致。
- 四类记录和 mapping 可任意先后到达。
- 每个 iTOW 的缓存数量受 max_pending_epochs 限制；按 LRU 淘汰。
- 超过 max_pending_age_ns 的 incomplete epoch 被 retire，后续碎片不能复活。
- 完全相同的重复记录/mapping 忽略；冲突重复会 retire 整个 epoch 并报错。
- invalid mapping retire 该 iTOW；不会等待后续 record 重新激活。
- joiner 最终仍验证四消息 iTOW 精确相等、receive monotonic skew、quality flags、covariance、source timestamp 严格递增。
- reset 后必须重新 begin_generation。
- GPS week rollover 与 USB reconnect 都是 generation transition；旧周/旧连接 callback 即使 iTOW 相同也必须拒绝。
- 直接调用 `build_observation_from_ubx_epochs()` 的无缓存路径也执行四记录 context 一致性与 canonical source_clock_id 校验，不能绕过 joiner 的 generation fence。

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

## 5. GPS week context 具体验收

- 四路 ROS decoder 都校验非空 process session、非负 connection generation、0..32767 GPS week，并把三者写入对应 immutable epoch record。
- ROS epoch adapter 对四类记录逐条检查 active process/generation/week；旧周 callback 不能进入 pending cache。
- UbxFourMessageEpochJoiner 对每条记录校验 GPS week 与 canonical source_clock_id 一致。
- 直接四消息构造 helper 同样拒绝跨 session、跨 connection generation、跨 GPS week 与错误 source_clock_id。
- 测试覆盖四路 decoder context 输出/非法 week、四路 joiner stale-week、ROS adapter week rollover 后四路旧回调拒绝与新周记录接纳、direct helper cross-context rejection。

## 6. 未完成项

此模块不产生 time mapping，也不实现 rclpy subscriptions、NavSatFix publisher 或 diagnostics node。固定 driver 当前没有 NAV-TIMEGPS/NAV-TIMELS，因此真实 mapper 仍缺 GPS week/leap-second source。必须先实现/接入有效 NAV-TIMEGPS 与 NAV-TIMELS 数据源，并通过 week rollover、leap-second、receiver reset 与 monotonicity tests，才能将 valid mapping 输入本 adapter。

## 7. CI 与缺陷修复记录

Actions run 45755029444 与 45755029355 对应 nav head 4fbbf078bb0f83b657c78c0b87365282769d831e，Humble/Jammy 与 Jazzy/Noble 两个矩阵均全部 success。测试覆盖 10 个 epoch-adapter 场景与并发回调。

较早 run 45754968219 / 45754972709 暴露 stale expiry 测试语义不一致；分析确认 adapter 在 watermark 推进时过期 epoch 后，可能在同一 _entry 调用中重新创建相同 iTOW 的 pending entry。修复为 stale retire 后再次检查 tombstone，拒绝同一 iTOW resurrection；同时按最大 monotonic watermark 计算 expiry，并把 epoch first_receive 设为 mapping/record 中最早 callback 时间。修复后的 Humble/Jazzy 测试全部通过。


CI 验收：代码/测试 head `afd9fc290db35e9922d3c6cf503a3003d0275271` 对应 `gnss-quality-ci` run `45759750171` 和 `ros2-package-ci` run `45759750178`；Humble/Jammy、Jazzy/Noble 全部 success，gnss-quality 每个环境 141 tests、0 failures。此为纯 Python tests，不代表 ROS DDS/executor barrier 或真实 USB/F9P 实测。
