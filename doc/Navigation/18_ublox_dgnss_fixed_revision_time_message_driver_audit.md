# 固定版 ublox_dgnss 时间消息驱动实现审计

- 日期：2026-11-26
- 审计对象：`aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e`
- 目标设备：ZED-F9P（需后续 MON-VER 确认实际固件）
- 状态：固定 commit 源码审计完成；未修改上游驱动、未构建、未运行测试、未连接硬件
- 专项分支：`nav`
- 关联：`13_zed_f9p_utc_gps_time_mapping_audit.md`、`14_zed_f9p_utc_gps_mapper_patch_plan_and_test_vectors.md`、`17_zed_f9p_nav_timegps_crosscheck_audit.md`

## 1. 结论摘要

对固定 commit 的源码逐文件核验后，确认该版本驱动**没有实现 NAV-TIMEGPS，也没有实现 NAV-TIMELS**。它实现并发布 NAV-PVT、NAV-STATUS、NAV-TIMEUTC。F9P TOML 配置中包含 `CFG_MSGOUT_UBX_NAV_PVT_USB`、`CFG_MSGOUT_UBX_NAV_STATUS_USB`、`CFG_MSGOUT_UBX_NAV_TIMEUTC_USB`，但不包含 TIMEGPS/TIMELS 输出配置项。

因此，上一份 §17 的 NAV-TIMEGPS 设计只是一份协议层可选交叉核验设计，不是当前固定驱动已具备的能力。它不能被列为当前可用输入，也不能把 NAV-TIMEGPS/TIMELS 当作 Mapper 当前输入。对当前固定驱动而言，NAV-TIMELS 缺失尤其关键：没有可直接读取的 receiver current GPS−UTC leap offset 和 source flags，§4.12/§4.13 所设计的完整可信时间路径无法闭合。

**当前 Mapper 生产实现 Gate 继续保持关闭。** 不应通过把 NAV-PVT/NAV-TIMEUTC 内部的 UTC validity 当作 leap-offset source 来补齐缺口。

## 2. 固定版本证据清单

审计 ref：`9d4f5addbc72d092c57dde99b20712258e96e81e`。

| 文件 | 观察结果 | 结论 |
|---|---|---|
| `ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_msg.hpp` | 定义 NAV-PVT=0x07、NAV-STATUS=0x03、NAV-TIMEUTC=0x21；没有 NAV-TIMEGPS=0x20 或 NAV-TIMELS=0x26 常量 | TIMEGPS/TIMELS 未纳入消息 ID |
| `.../ubx/ubx_nav.hpp` | include、FrameComms、成员、构造、accessor、frame switch 均有 PVT/STATUS/TIMEUTC；无 TIMEGPS/TIMELS | 无 payload 类型/dispatch 路径 |
| `.../ubx/nav/ubx_nav_timeutc.hpp` | 解析 20-byte TIMEUTC 字段：iTOW/tAcc/nano/civil fields/valid flags | TIMEUTC parser 存在 |
| `.../ubx/nav/ubx_nav_pvt.hpp` | 解析 PVT civil time、valid flags、tAcc/nano、fix 等字段 | PVT parser 存在 |
| `.../ubx/nav/ubx_nav_status.hpp` | 解析 iTOW、gpsFix、gpsFixOK、diffSoln、wknSet、towSet 等 | STATUS parser 存在 |
| `ublox_ubx_msgs/msg/UBXNavTimeUTC.msg` | 发布 itow/t_acc/nano/civil fields/valid_tow/valid_wkn/valid_utc/utc_std | 无原始 validity byte 字段 |
| `ublox_dgnss_node/src/ublox_dgnss_node.cpp` | 创建 `ubx_nav_time_utc`、`ubx_nav_pvt`、`ubx_nav_status` publishers；TIMEUTC/PVT 字段逐项复制到 ROS message | 发布路径存在；TIMEGPS/TIMELS publisher 缺失 |
| `ublox_dgnss/config/f9p_ubx_config.toml` | include PVT、STATUS、TIMEUTC USB output config；无 TIMEGPS/TIMELS | 默认 F9P config 不请求这两路消息 |

注意：上表的“没有”是对该固定 commit 的源代码和 F9P 配置文件审计结论，不代表所有 u-blox 固件、驱动版本或其他仓库均不支持这些消息。

## 3. 当前时间数据能力矩阵

| UBX 消息 | 固定驱动解析 | ROS publisher | F9P 配置项 | Mapper 可用性 |
|---|---|---|---|---|
| NAV-PVT (01/07) | 是 | `/ubx_nav_pvt` | 是 | UTC civil 主候选；需 validity/confirmation/freshness gate |
| NAV-STATUS (01/03) | 是 | `/ubx_nav_status` | 是 | 提供 wknSet/towSet 等辅助 gate；不能独自建立绝对时间 |
| NAV-TIMEUTC (01/21) | 是 | `/ubx_nav_time_utc` | 是 | UTC 独立一致性候选；需 valid bits/source policy |
| NAV-TIMEGPS (01/20) | 否 | 否 | 否 | 当前不可用；§17 仅保留为未来扩展设计 |
| NAV-TIMELS (01/26) | 否 | 否 | 否 | 当前关键缺口；无法从该驱动取得 current leap offset/source/event evidence |

## 4. Validity 与 raw-byte 保真

### 4.1 NAV-TIMEUTC

内部 parser 把 payload 最后一个 validity byte 读入 union `valid.all`，并通过 bitfield 解读 validTOW、validWKN、validUTC、utcStandard。ROS publisher 将这几个解释后的字段复制到 ROS message，但消息定义没有 `valid_raw` 字段，也没有保留未知/保留 bit。

这意味着：驱动内部短暂持有原始 byte，但 ROS 消费者无法从 topic 原样恢复该 byte。不能把“内部 union 有 all”误报为“ROS 输出保留 raw validity byte”。

### 4.2 NAV-PVT

parser 将 validity byte 解读为 validDate、validTime、fullyResolved、validMag；publisher 逐字段输出。额外 flags2 的 confirmedAvailable/confirmedDate/confirmedTime 也被逐项输出。原始 flags byte 和所有保留 bit 没有作为 raw byte 发布。

### 4.3 NAV-STATUS

parser 将 status flags 拆为 gpsFixOK、diffSoln、wknSet、towSet 等字段；ROS message 发布解释字段，不提供原始 flags byte。其 wknSet/towSet 可用于 gating，但不替代 TIMEGPS absolute week/TOW 或 TIMELS leap offset。

**结论：** 当前 ROS topic 的语义字段足以消费部分有效性标志，但不满足“原始 UBX validity byte 全量保真、保留未知位”的审计要求。未来若要作为可信时间输入，建议在 ROSIDL 中增加 raw validity/flags byte（或 raw payload byte array），同时保持现有语义字段向后兼容。

## 5. Payload 长度边界：发现需优先修复的健壮性风险

固定版本的 `NavTimeUTCPayload`、`NavPvtPayload`、`NavStatusPayload` 构造函数接收 `payload_polled, size`，先按传入 size resize/copy，再从固定 offset 读取字段。通用 `buf_offset<T>()` 直接执行 `memcpy(&value, buf->data() + offset, sizeof(T))`，该 helper 本身没有检查 `offset + sizeof(T) <= buf->size()`。

`FrameContainer<T>::frame()` 将 frame 的 `length` 直接传给 `Payload<T>` 构造函数。就已审计的这些路径而言，未看到在进入具体 payload constructor 前针对消息类型执行 exact-length validation。

这形成一个明确的防御性编程风险：如果异常短 payload 能够到达该层，固定 offset 读取可能越过 vector 有效范围。当前源码审计尚未完整追踪 USB 接收 parser 是否在上游保证 frame length 与 buffer 完整性；因此本项定为 **高优先级待验证风险（不是已复现崩溃）**，不能直接宣称可由外部 UBX 帧触发。

修复合同：
1. 在消息类型 payload constructor 或 FrameContainer dispatch 前，验证消息 exact payload length；至少 TIMEUTC=20 bytes、STATUS=16 bytes、PVT=92 bytes（需以固定目标协议版本再核对）。
2. 不匹配时返回结构化 parse error/reason code；不得继续执行字段 memcpy。
3. `buf_offset` 增加通用边界检查，避免其他 UBX parser 重复暴露相同风险。
4. 已在 `ubx.hpp::get_polled_frame()` 发现 checksum 条件使用 `ck_a != received_a && ck_b != received_b`。该逻辑只有两个 checksum byte 都错误时才拒绝；若仅一个 byte 错误则会放行。应改为任一 byte 不匹配即拒绝（`||`），并增加 A-only/B-only/both mismatch 三组测试。此证据来自 poll-response 路径；普通异步 USB receive parser 的 checksum 行为仍需单独审计。
5. 增加 payload 长度 0、1、expected−1、expected、expected+1、最大合法值、declared/actual length 不一致等测试。
6. 通过 fuzz/property tests 检查 parser 对截断帧、拼接帧、损坏 length、错误 checksum 的 fail-closed 行为。

其中第 4 项是进一步审计目标；本轮没有对完整 USB frame parser 做到足以确认其实际运行时行为。


### 5.1 普通异步 USB receive 路径：边界与 framing 缺口

进一步追踪 `ublox_in_callback(libusb_transfer *transfer_in)`（固定 commit 的 `ublox_dgnss_node.cpp`）发现：

- 以 `transfer_in->actual_length` 作为当前输入长度；
- 仅在 `len > 2` 且当前 transfer 的前两个字节恰好为 UBX sync 时，直接将整个 transfer 复制成一个 `Frame`；
- 随即调用 `Frame::from_buf_build()`，其读取固定 header bytes [0..5]，并将 buffer 最后两个 byte 当作 checksum；
- 未见跨 USB transfer 的残帧缓存/重组，也未见单个 transfer 内连续多个 UBX frame 的逐帧拆分；
- 未见在入队前验证 `actual_length >= 8` 或 `actual_length == declared_payload_length + 8`；
- 因而短于 6 bytes 的 transfer 可能在 `from_buf_build()` 读取 header 时越界；6/7 bytes 及 declared length 与 actual length 不一致的输入也没有在该入口被明确拒绝；
- 接收路径随后在 `ubx_queue_frame_in()` 做 checksum 检查，且该处条件为 `ck_a != received_a || ck_b != received_b`，逻辑正确；这与 `get_polled_frame()` 中已发现的 `&&` 缺陷是两条不同路径，不能混为一谈。

该入口结构意味着：若 libusb transfer 边界不保证一帧一 transfer，则合法帧被拆分、合并时可能丢失或错误解释。必须通过 libusb transfer 实测/USB API 语义证据确认边界假设，不能仅凭常见设备当前输出形态认定 framing 安全。

### 5.2 UBX queue 并发访问

`ubx_timer_callback()` 读取 `ubx_queue_.size()`、复制 `ubx_queue_[0]` 时未持有 `ubx_queue_mutex_`；USB callback 则在持锁时 push_back，timer 在稍后持锁 pop_front。若 callback 与 timer 可并发运行，则 deque 的 size/index 与 push/pop 存在未同步访问风险。需要确认 executor/libusb callback 的线程归属与互斥关系；若无严格单线程保证，应在同一锁临界区内完成 front-copy/pop 或改用受控线程安全队列。RTCM queue 也应同样审计。

这属于静态源码暴露的并发风险，尚未通过 TSAN 或压力测试复现。


### 5.3 固定 USB transfer 容量与 callback 的额外边界问题

固定版本 `usb.hpp` 定义 `IN_BUFFER_SIZE = 64 * 200 = 12800`，`make_transfer_in()` 将此容量传给 `libusb_fill_bulk_transfer`。`callback_in()` 在 COMPLETED 时将 transfer 交给 node 的 `ublox_in_callback()`，然后标记完成并尝试补交下一条 IN transfer。

`ublox_in_callback()` 的 NMEA 分支在 `len>0) 时直接写 `buf[len] = 0`，但没有检查 `len < transfer buffer capacity`。当 `actual_length == IN_BUFFER_SIZE` 时该写入越过 12800-byte transfer buffer 尾端 1 byte。这是可由代码路径直接证明的越界写风险；是否实际发生取决于设备/USB transfer 是否返回满 buffer，需要边界测试或硬件 trace 验证。应改为长度受控的字符串构造，或确保 buffer 预留并验证 terminator 空间，不能写入 libusb transfer 缓冲区边界之外。

更严重的是，UBX 与 RTCM 分支仅检查 `len > 2` 后就将整个 transfer 交给 frame parser。对于 UBX，`Frame::from_buf_build()` 无条件读取 buf[0..5] 并访问 buf[size-2]、buf[size-1]；因此 3–7 bytes 的 transfer 会在解析阶段越界读取。对任意长度大于 2 的 UBX 起始 transfer，尚未验证 `len >= 8`、declared payload length <= actual bytes、以及 `len == declared_length + 8`。该问题不仅是“未实现跨 transfer 重组”的功能缺口，也包含可静态确认的短缓冲区越界读风险。

`ublox_out_callback()` 对 USB OUT transfer 使用相同的 `len>2` + `from_buf_build()` 模式，也需要相同的长度保护；虽然 OUT 数据通常由本机生成，但不能把“通常可信”作为内存安全边界。

### 5.4 IN transfer 与 USB stream framing 的证据边界

libusb IN transfer 的 buffer capacity 是 12800 bytes；bulk endpoint descriptor 的 max packet size 与 transfer buffer capacity 是不同概念。一次 bulk transfer 的完成长度是本次 transfer 的 `actual_length`，不能据此推导“一次 transfer 恰好包含一个完整 UBX/RTCM/NMEA 消息”。当前代码把一次 transfer 的起始字节作为协议判别，并把整个 transfer 当成单一消息，缺少通用增量 stream decoder。除非另有目标设备 USB trace / 协议约束证明一 transfer 一消息，否则应按 byte stream 设计：同步字搜索、长度字段解析、完整帧等待、连续帧拆分、噪声丢弃、最大帧长上限和 checksum 验证。

### 5.5 队列线程安全：修正风险定级

node 为 UBX timer 与 USB-events timer 创建了**不同的 MutuallyExclusive callback groups**。因此：
- 若运行于 SingleThreadedExecutor，两个 timer callback 不会并发执行，USB libusb callback（由 `handle_usb_events()` 驱动）与 UBX timer 对 node-level queue 的访问在 executor callback 维度被串行化；此前“必然存在并发 data race”的表述过强。
- 若运行于 MultiThreadedExecutor 或不同 executor/thread，两个 callback group 可以并行，USB callback 在 `ubx_queue_mutex_` 下 push，而 UBX timer 对 `size()`/index 读取未持锁，随后才锁住 pop；此时存在 data race / deque invalidation 风险。
- 当前固定源码审计尚未确认目标启动入口使用的 executor 类型，也未找到对所有运行方式强制 single-threaded 的约束。因此应将此项定为**executor-dependent concurrency defect / unresolved deployment risk**，不能标记为已复现 race。

此外，`usb.cpp` 的 `transfer_queue_` 在 `cleanup_transfer_queue()`、`cleanup_all_transfers()`、`queued_transfer_in_num()` 中有若干 `size()==0` 检查位于 mutex 之前；`transfer_t::completed` 在 libusb callback 中写入、在持锁队列扫描中读取，但写入方没有同一 mutex 或 atomic。若 libusb callbacks 与其他 Connection 方法并发，这些访问也需要线程安全审查。当前未通过 TSAN 验证。

## 6. 同历元与接收时序

当前发布函数使用 `f->ts` 填充 ROS Header stamp，消息本身包含 iTOW（PVT、STATUS、TIMEUTC）。这给消费者提供了接收/采集时间戳与 GNSS epoch tag 两种信息，但源码中未发现为跨 topic 组帧提供的统一 `process_session_id`、`connection_generation`、`queue_sequence` 身份字段。

因此 Mapper 不得仅凭 ROS Header stamp 相近就 join PVT/STATUS/TIMEUTC。当前可做的保守合同是：
- 以原始 iTOW 精确匹配 PVT、STATUS、TIMEUTC；
- 同时要求驱动连接 generation/session 一致；但这些 identity 字段目前未通过 ROS topic 暴露，需在驱动 ingress 层补充；
- 禁止 nearest-neighbor/接收时间最近配对；
- generation identity 未实现前，不能将跨 USB detach/reconnect 的消息放入同一个 mapper epoch buffer；
- TIMEGPS/TIMELS 缺失不允许由 iTOW join 推导出 GPS week 或 leap offset。

## 7. 对 §17 NAV-TIMEGPS 设计文档的修订解释

§17 保留其价值，但状态必须明确为“未来驱动扩展的协议/算法设计”，而非现有能力。实现顺序应调整为：

1. 先确认目标 F9P 固件/协议版本及设备实际支持 NAV-TIMELS/TIMEGPS 的情况（MON-VER + 原始帧证据）。
2. 对固定驱动新增 NAV-TIMELS 和 NAV-TIMEGPS 的 class/id、payload、FrameComms、dispatch、ROSIDL、publisher、F9P config output 参数。
3. 保留 raw payload、raw validity byte、source flags、message version、iTOW、连接 generation。
4. 先完成 exact-length parser 与 malformed-frame 防护，再让 Mapper 订阅这些新消息。
5. 加入纯 parser 单元测试、ROS publisher integration test、bag replay test、设备实测。
6. 只有 source allowlist、freshness、同历元、leap transition artifact、冲突策略和精度合同全部冻结后，才开放 Mapper VALID。

若目标 F9P 固件不支持所需 NAV-TIMELS 字段，必须重新评估接收机型号/固件或引入另一个有明确来源合同的时间输入；不得假设升级驱动就能让硬件产生不存在的消息。

## 8. 本轮审计边界与下一步

已完成：
- 固定 commit 的消息 ID、NAV dispatcher、payload parser、ROSIDL、publisher、F9P TOML 配置核对；
- 确认 TIMEGPS/TIMELS 在该固定版本未实现；
- 确认 TIMEUTC raw validity byte 未从 ROS topic 保留；
- 发现固定 offset memcpy 缺少本地边界检查的风险；
- 确认 PVT/STATUS/TIMEUTC 通过 iTOW 和 ROS header stamp 输出，但未发现统一跨重连 generation identity。

未完成：
- USB receive callback 的 transfer 边界/残帧重组/多帧拆分与 declared-vs-actual length 审计；poll-response 与 async receive checksum 路径差异审计；
- 精确定位 receive queue 的线程、锁、生命周期及 reconnect generation；UBX queue size/index 未持锁的风险依赖 executor 类型：不同 MutuallyExclusive callback groups 在 MultiThreadedExecutor 下可并发，在 SingleThreadedExecutor 下串行；executor 入口尚未确认；
- 对上游驱动做构建/测试；
- ZED-F9P MON-VER、TIMEGPS/TIMELS 原始帧与固件实测；
- 对 payload 长度风险做最小复现或 fuzz。

下一项任务应是修复 UBX/RTCM/NMEA transfer 边界与增量 framing、poll checksum 条件，并先确认 executor/Connection callback 线程模型，再修复队列和 transfer_queue 的同步；在这之前不动时间消息生产实现。若确认长度防护确实缺失，先提交最小、可独立测试的 parser hardening patch，再扩展时间消息。

## 9. 固定源码引用

以下链接均固定到审计 commit，便于复核：

- [ubx_msg.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_msg.hpp)
- [ubx_nav.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_nav.hpp)
- [ubx_nav_timeutc.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/nav/ubx_nav_timeutc.hpp)
- [ubx_nav_pvt.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/nav/ubx_nav_pvt.hpp)
- [ubx_nav_status.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/nav/ubx_nav_status.hpp)
- [ubx.hpp](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx.hpp)
- [UBXNavTimeUTC.msg](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavTimeUTC.msg)
- [F9P configuration](https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss/config/f9p_ubx_config.toml)

