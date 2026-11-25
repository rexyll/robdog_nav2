# ZED-F9P 固定驱动的 Receiver Generation 证据审计

> 审计对象：aussierobots/ublox_dgnss tag 0.7.6，commit 9d4f5addbc72d092c57dde99b20712258e96e81e。
> 状态：固定源码审计（E1/E2）；未连接真实 ZED-F9P，未验证固件行为、USB 重连和真实时间连续性。

## 1. 结论摘要

当前固定驱动提供了严格 iTOW 配对所需的 PVT/HPPOSLLH/COV/STATUS 原始消息字段，但**没有提供足以直接建立跨重启、跨 GPS week 的持久 receiver generation identity**：

1. UBXNavStatus 包含 iTOW、wkn_set、tow_set、msss；msss 被定义为 milliseconds since Startup/Reset，是 32 位计数，不是唯一 boot ID。
2. UBXNavTimeUTC 包含 iTOW、valid_tow、valid_wkn、valid_utc、UTC calendar fields，但消息中没有 GPS week 数值。
3. 固定版本未发现 UBXNavTimeGPS.msg、NAV-TIMEGPS parser、publisher 或订阅 topic；ubx_nav.hpp 只注册 NAV-TIMEUTC、NAV-CLOCK 等相关类。
4. NAV-PVT 提供 UTC 年月日时分秒及 valid_date/valid_time/fully_resolved，但不提供 GPS week 数值。
5. driver 的 USB 输入 callback 在主机收到 USB 数据时执行 rclcpp::Clock().now()，该值被放入内部队列，之后成为 UBX ROS message header.stamp；它不是 receiver measurement clock，也不是 host steady-clock freshness。
6. USB detach/attach 在节点内部更新 device_attached_/device_readiness_state_ 并恢复参数，但没有发布 receiver generation、boot identity 或 reset transaction event。

因此，当前 ROS adapter 不能把 source_clock_id 简单定义成 “GPS week + msss”，也不能在 msss 回退时直接宣布 reboot。需要增加驱动证据/接口，或设计并验证一个能从 UTC/GPS 时间可靠恢复周编号的 mapper；在此之前时间身份不确定时必须 fail-closed。

## 2. 固定源码事实

### 2.1 NAV-STATUS

文件：
- ublox_ubx_msgs/msg/UBXNavStatus.msg
- ublox_dgnss_node/src/ublox_dgnss_node.cpp

消息字段包括：
- itow：GPS Time of Week，单位 ms；
- wkn_set、tow_set：week/time-of-week validity flags；
- msss：milliseconds since Startup/Reset，uint32；
- gps_fix_ok、carr_soln_valid、carr_soln 等质量字段。

publisher 将 payload->msss 直接复制到 ROS 消息；没有附加 boot UUID、session counter 或持久化 generation。

msss 是 uint32 毫秒计数，数值空间约 49.7 天后自然回绕。因而：
- 单调递增可作为当前连接期间的辅助连续性证据；
- 下降既可能是计数器回绕，也可能是 receiver reset/boot；
- 长时间数据缺口、USB detach/reconnect 或 ROS 节点重启都会使单凭 msss 的判定不足；
- msss 不可作为跨进程/跨设备的唯一 generation ID。

### 2.2 NAV-TIMEUTC 与缺失的 NAV-TIMEGPS

固定版本 UBXNavTimeUTC.msg 包含 itow、t_acc、nano、UTC calendar fields，以及 valid_tow、valid_wkn、valid_utc；没有 gps_week 数值字段。

固定版本 NAV-PVT 同样包含 UTC date/time 和 validity flags，但没有 GPS week 数值字段。

在 ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_nav.hpp 的 NAV message registrations 中可见 NAV-TIMEUTC、NAV-CLOCK、NAV-PVT、NAV-STATUS 等类型；未发现 NAV-TIMEGPS 对应消息、parser 或 publisher。当前 driver 不能直接向 adapter 提供 NAV-TIMEGPS week number。

这不代表无法建立绝对时间映射，但意味着现有实现没有直接提供最清晰的 GPS week 来源。若选择从 UTC calendar 恢复 GPS week，必须：
- 只使用 valid_utc/date/time/fully_resolved 等标志全部满足的 UTC；
- 明确 GPS-UTC leap-second 表版本和有效期；
- 检查 UTC 时间连续性、闰秒处理、日期回退与冷启动；
- 处理 UTC 尚未解析或 validity flags 未置位的初始化窗口；
- 与 iTOW 做跨消息同历元关联和一致性检查；
- 在设备协议/真实 bag 证实之前，不把该路线视为已实现。

优先方案是评估给固定 driver 增加 NAV-TIMEGPS 原始消息支持，显式暴露 week number、week validity、iTOW 和相关 time accuracy；这需要额外审查 UBX protocol version、目标 F9P firmware compatibility、消息 package API 和 CI。

### 2.3 USB detach/reconnect 生命周期

文件：ublox_dgnss_node/src/ublox_dgnss_node.cpp

- hotplug_detach_callback() 将 device_attached_ 置 false、device_readiness_state_ 置 UNREADY，并重置 device parameters。
- hotplug_attach_callback() 将 device_attached_ 置 true；若曾连接过，则执行 perform_usb_initialization() 恢复参数并将 readiness 置 READY。
- 当前固定源码未见这些回调发布 receiver generation ID、boot UUID、reset reason 或可供 ROS adapter 订阅的 lifecycle event。
- ublox_in_callback() 在 USB 输入回调入口使用 rclcpp::Clock().now() 生成队列时间；后续 UBX ROS publisher 将队列时间复制到 header.stamp。

因此 adapter 若只订阅 UBX message topics，可能观察到消息中断/恢复，但不能可靠获知断连事件发生的精确时刻、设备是否真正重启、内部 msss 是否延续。需要显式 connection lifecycle interface，或由外部 supervisor 提供可靠连接 generation token。

## 3. 对 generation controller 设计的修正

### 3.1 当前可接受的证据等级

| 证据 | 可用于 | 不可单独用于 |
|---|---|---|
| iTOW | 同一 GPS week 内精确配对、检查周内时间顺序 | 全球单调时间、识别 reboot |
| NAV-STATUS wkn_set/tow_set | 判断 week/TOW 信息有效标志 | 读取 week 数值 |
| NAV-STATUS msss | 当前运行期间辅助检测时间连续性/异常 | 唯一 boot ID；区分 49.7 天回绕与 reboot |
| NAV-TIMEUTC valid flags + UTC fields | 在有效时可作为 UTC 时间锚点候选 | 未处理 leap seconds/validity 时直接生成 GPS week |
| NAV-PVT UTC fields | 辅助 UTC 时间一致性检查 | 独立 receiver generation |
| USB detach/attach 内部状态 | driver 自身重连流程 | 当前 ROS adapter 可直接消费的 generation event |
| host steady receive time | freshness、topic callback skew、数据中断 | receiver measurement timestamp |
| ROS header.stamp | driver 接收时间的辅助记录（当前实现基于 rclcpp clock） | host steady freshness 或 receiver source time |

### 3.2 必须 fail-closed 的情况

- 只有 iTOW 回退，没有可信 GPS week/session/boot evidence。
- msss 回退但无法区分自然 uint32 wrap 与设备 reset。
- USB 重新连接后没有可核验的 receiver session identity。
- UTC validity 未建立、UTC 日期/时间不连续或 leap-second mapping 不明确。
- NAV-STATUS 的 wkn_set/tow_set 无效。
- ROS header stamp 与 host steady receive time 混用，或 source-time mapper 无法证明单调。
- driver/adapter 进程重启后 generation identity 可能复用。

这些情况下，不应调用 joiner.begin_generation() 来“试着恢复”；先撤销 GNSS validity，等待可信时间锚点和 generation 判定。

### 3.3 实施路径选择

A. **增强固定驱动（优先评估）**
- 增加 NAV-TIMEGPS 消息定义、parser、publisher 和协议版本兼容检查；
- 增加 connection lifecycle/status topic，携带连接 session counter 或由 driver process session UUID + monotonic connection generation 构成的 identity；
- 在每次 detach、attach、device reset 和 driver process restart 时递增 connection generation；
- 在消息 metadata 中显式标记 receiver generation；不得仅依赖 header.stamp；
- 为 NAV-TIMEGPS 与 NAV-STATUS msss 增加 rollover/reboot parser tests。

B. **不修改 driver，adapter 从现有 UTC 字段恢复时间**
- 先证明 NAV-TIMEUTC 在所选 F9P 固件上的实际发布和有效标志语义；
- 实现 GPS-UTC leap-second-aware UTC→GPS week/TOW mapping；
- 结合 msss 的模 2^32 差分、iTOW、UTC 和 host steady receive gap 形成多证据状态机；
- 任何证据冲突进入 AMBIGUOUS_INVALID；
- 该路线必须通过真实 receiver logs、week rollover、USB reconnect、cold/warm/hot start 和长期 soak test。

两条路线都不能跳过真实硬件日志验证。当前不在本仓库中猜测或实现未经验证的阈值。

## 4. 对 §11 设计规格的具体影响

- ReceiverClockEvidence 增加 gps_week_source、utc_validity、leap_table_version、driver_connection_generation、msss_raw 与 msss_wrap_candidate。
- GenerationDecision 的 CONTINUOUS 只能由已验证 GPS absolute-time mapping 和 session continuity 支持。
- NEW_GENERATION 必须记录触发证据和 confidence class；不能将“msss 下降”作为充分证据。
- AMBIGUOUS 必须使 GNSS output invalid；不能自动 reset 后重新 begin generation。
- reset_transaction_id 和 driver_connection_generation 分离；USB reconnect 事件先撤销 validity，再等待 receiver clock evidence。
- 若无法提供稳定 receiver identity，driver process restart 默认产生新 adapter session，并要求重新建立锚点。

## 5. 下一步审计与验证 Gate

- [ ] 确认所选 ZED-F9P 固件版本和 NAV-TIMEUTC/NAV-STATUS/NAV-PVT 的实际配置、频率与 validity 行为。
- [ ] 核验 NAV-TIMEGPS 是否可由目标 F9P 固件输出；若 driver 不支持，审查最小扩展 patch。
- [ ] 审查 driver USB lifecycle 回调线程与 ROS callback/executor 的同步语义，设计不会丢失 detach/attach generation 的接口。
- [ ] 采集正常运行、GPS week rollover、USB 断开重连、接收机冷/热/暖启动的原始 bag/日志。
- [ ] 验证 msss uint32 wrap 与 reboot 可区分性；若不能区分，明确依赖 NAV-TIMEGPS/UTC/connection session 的证据组合。
- [ ] 在纯算法层补充 week rollover、msss wrap、reboot、ambiguous、driver restart、late old-session callback 测试。
- [ ] 只有上述证据闭环后，才实现 generation mapper 与 ROS adapter。

## 6. 固定来源

- Driver tag/commit：[aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e](https://github.com/aussierobots/ublox_dgnss/tree/9d4f5addbc72d092c57dde99b20712258e96e81e)
- [UBXNavStatus.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavStatus.msg)
- [UBXNavTimeUTC.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavTimeUTC.msg)
- [UBXNavPVT.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavPVT.msg)
- [ubx_nav.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_nav.hpp)
- [ublox_dgnss_node.cpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/src/ublox_dgnss_node.cpp)

**证据等级：** 固定上游源码字段/代码为 E1；项目 adapter 前置条件和路线建议为 E3。没有真实设备证据，不得标记为硬件验证通过。


## 7. 后续协议审计的修正（以 §13 为准）

本节修正 §3.3 中将“增加 NAV-TIMEGPS”列为优先时间映射路线的表述。进一步核对 ZED-F9P Integration Manual R12 后，可靠绝对时间应优先使用 NAV-PVT UTC 及其有效性/确认字段；NAV-TIMEGPS 虽在 protocol 27.11 中定义 week/fTOW/leapS 和 validity flags，但不应未经论证取代 UTC 主时间来源。

当前建议主链为 NAV-PVT UTC + NAV-TIMEUTC 同历元一致性 + NAV-TIMELS 当前 leap-second offset + NAV-STATUS 时间有效性标志；固定驱动缺少 NAV-TIMELS parser/publisher，仍需扩展。NAV-TIMEGPS 保留为可选诊断交叉校验。完整字段、validity、负 nano、sec=60、leap-second source 和测试 Gate 见 [§13 ZED-F9P UTC/GPS 时间映射审计](13_zed_f9p_utc_gps_time_mapping_audit.md)。

此外，NAV-TIMEUTC 的 valid_wkn 仅为 week validity flag，不包含 week number 数值；其 valid_utc、valid_tow、utc_std 语义必须分别处理。NAV-PVT 的 confirmed_avail/confirmed_date/confirmed_time 与 valid_date/valid_time/fully_resolved 也不能混为同一标志。
