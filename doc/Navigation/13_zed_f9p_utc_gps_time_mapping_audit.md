# ZED-F9P UTC / GPS 周时间映射与固定驱动扩展审计

> 固定驱动：aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e（tag 0.7.6）。
> 协议参考：u-blox ZED-F9P Interface Description UBX-18010854 R07（协议 v27.11，Early Production Information）；ZED-F9P Integration Manual UBX-18010802 R12（适用固件 HPG 1.12/1.13/1.32）。
> 证据：固定驱动源码 E1；协议/集成手册 E1/E2；项目设计建议 E3。尚无目标板卡固件信息或真实接收机日志。

## 1. 本轮结论

上一份 §12 将“给驱动增加 NAV-TIMEGPS”列为优先方案，需要修正为：

1. **主绝对时间来源优先采用 NAV-PVT UTC**，检查 valid_date、valid_time、fully_resolved；并按 confirmed_avail 决定是否要求 confirmed_date / confirmed_time。
2. 使用同一 navigation epoch 的 NAV-TIMEUTC 进行 UTC 一致性和 UTC standard 交叉核验；检查 valid_utc、utc_std。
3. UTC→GPS 连续时间换算需要可信的当前 GPS−UTC leap-second offset。优先审查并接入 NAV-TIMELS 的 currLs、validCurrLs 和 srcOfCurrLs；不能把默认、过期或未知 leap-second 数值当成可信输入。
4. NAV-STATUS 的 tow_set / wkn_set 用于判断 iTOW / week validity；iTOW 只用于严格同历元配对和周内一致性校验，不得单独构造永久单调 source timestamp。
5. NAV-TIMEGPS 协议消息确实存在且包含 week/iTOW，但 Integration Manual 对可靠绝对时间推荐使用 UTC time messages；因此将 NAV-TIMEGPS 作为可选诊断交叉校验，而不是默认唯一时间权威。
6. 即使时间映射成功，receiver boot/session identity 仍是独立问题：UTC 连续不证明设备没有重启；还需要 driver connection lifecycle generation、进程 session identity 和 reset evidence。

## 2. 固定驱动的真实接口

### 2.1 NAV-PVT 已提供的 UTC 质量字段

固定消息 UBXNavPVT.msg 含：
- iTOW；
- year/month/day/hour/min/sec；
- valid_date、valid_time、fully_resolved；
- t_acc、nano；
- confirmed_avail、confirmed_date、confirmed_time。

固定 driver 已实现 NAV-PVT parser、publisher 和 ROS message，不需要为读取这些字段新增消息类型。

推荐解释：
- valid_date && valid_time：接收机声明知道日期和日内时间；不代表它们一定正确。
- fully_resolved：UTC 秒级歧义已消除；未完全解析时可能存在约 20 秒量级不确定性，不能用于精确 source time mapping。
- confirmed_avail：确认有效性信息是否可用。
- 当 confirmed_avail=true 时，confirmed_date && confirmed_time 才提供额外独立来源确认；任一为 false，应将绝对时间锚点降级/拒绝。
- 当 confirmed_avail=false 时，不能把 confirmation flags 的 false 当作“已确认失败”；但也不能声称有独立确认。valid-but-unconfirmed 应作为单独 confidence class，不能独立触发 NEW_GENERATION。

### 2.2 NAV-TIMEUTC 的字段和边界

固定消息 UBXNavTimeUTC.msg 含 iTOW、t_acc、nano、UTC calendar fields、valid_tow、valid_wkn、valid_utc、utc_std。

重要区别：
- valid_wkn 是“week number valid”的标志，不是 week number 数值。
- NAV-TIMEUTC payload 没有 GPS week 数值字段。
- valid_tow 与 valid_wkn 必须按协议解释；不能从两个 bool 还原 GPS week。
- utc_std 需拒绝 information-not-available / unknown 等未满足项目策略的来源。
- nano 是对年月日时分秒字段的纳秒级修正，允许负值；负值可能把时间校正到前一秒、前一分钟、前一天甚至前一年。不能把年月日时分秒拼接成普通 timestamp 后简单加 nano 而不处理日历进位/借位。
- 协议允许 sec=60 表示闰秒期间；不得直接传入只支持 0–59 秒的时间解析器而静默归一化。

### 2.3 NAV-TIMELS 是当前 leap-second offset 的候选来源

u-blox Interface Description 定义 NAV-TIMELS（0x01 0x26），协议 v27.11 的 u-blox 9 支持该消息。关键字段：
- iTOW；
- srcOfCurrLs；
- currLs：当前 GPS−UTC leap seconds，GPS time ahead of UTC；
- validCurrLs；
- srcOfLsChange、lsChange、timeToLsEvent；
- validTimeToLsEvent。

validCurrLs=true 才表示当前 leap-second 数值有效。但来源字段也必须审查：
- srcOfCurrLs=0 是 firmware hardcoded default，手册明确指出可能过时；
- srcOfCurrLs=255 是 unknown；
- 其他来源（GPS/SBAS/BeiDou/Galileo/GLONASS-derived/aided/configured）也要按所选固件和项目安全策略分类，不能把“非零”自动等同于独立可信。
- validTimeToLsEvent 与 validCurrLs 是不同的有效性位；没有未来闰秒事件信息不代表当前 leap seconds 无效，反之亦然。

固定候选驱动没有 NAV-TIMELS message/parser/publisher。因此若选择 receiver-provided leap-second offset，必须给 driver 增加 NAV-TIMELS 支持；否则只能使用版本化、来源明确的外部 leap-second table，并管理表有效期和更新责任。

### 2.4 NAV-TIMEGPS 的定位修正

u-blox Interface Description 明确定义 NAV-TIMEGPS（0x01 0x20）：iTOW、fTOW、week、leapS、towValid、weekValid、leapSValid、tAcc。手册列明 u-blox 9 protocol 27.11 支持该 periodic/polled message。

但 ZED-F9P Integration Manual 的时间章节说明多 GNSS 系统时间与 receiver local time 的转换存在复杂性，并推荐使用 UTC time messages；NAV-PVT 对 UTC 日期/时间提供 validity/confirmation 字段。

因此：
- NAV-TIMEGPS 是有价值的协议交叉校验输入；若目标固件确实支持，可用于诊断 UTC→GPS 转换差异。
- 不将 NAV-TIMEGPS 未验证地定为唯一 source-time authority。
- 若启用它，必须检查 towValid && weekValid；使用 leapS 时还必须检查 leapSValid。
- 目标 F9P firmware/protocol version 未由用户提供，不能把 R07 的 protocol 27.11 能力外推到实际板卡。

## 3. u-blox 时间语义对算法的硬约束

ZED-F9P Integration Manual R12 的时间章节明确：
- iTOW 表示 GPS time of week，但在接收机缺少足够转换信息时可能无效/不可靠；不建议将 iTOW 用作其他目的。
- 同一 navigation solution 的 UBX NAV messages 可以通过相同 iTOW 关联。
- 需要可靠绝对时间时，优先使用 NAV-PVT 的 UTC 时间与其有效性/准确度字段。
- validDate / validTime 表示接收机知道当前日期/时间，但日期/时间仍可能错误。
- confirmedDate / confirmedTime 是额外独立来源确认；仅当 confirmedAvail 可用时才提供。
- fullyResolved 表示 UTC 秒级歧义已消除；若 leap-second 信息未获取，UTC 时间可能存在秒级歧义。
- UTC date/time 六个整数分量按最近 0.01 秒舍入；nano 用于修正到精确时间，可能为负并跨越日历边界。

所以，iTOW 的使用范围限定为：
1. 同一接收机输出的 NAV-PVT/HPPOSLLH/NAV-COV/NAV-STATUS/NAV-TIMEUTC/NAV-TIMELS 同历元关联；
2. 与独立绝对 UTC/GPS 时间锚点交叉检查；
3. 周内 rollover 与时间连续性诊断（前提是有效性 flags 允许）。

禁止仅靠 iTOW 构造跨周、跨重启的单调时间轴。

## 4. 推荐的绝对时间映射链

### 4.1 数据采集

每条消息回调入口记录 host steady-clock receive timestamp；不要使用 driver header.stamp 替代。保留以下原始字段：
- NAV-PVT：iTOW、UTC fields、nano、tAcc、valid/confirmed flags；
- NAV-TIMEUTC：iTOW、UTC fields、nano、tAcc、valid flags、utc_std；
- NAV-TIMELS：iTOW、currLs、srcOfCurrLs、validCurrLs；
- NAV-STATUS：iTOW、tow_set、wkn_set、msss；
- 可选 NAV-TIMEGPS：iTOW/fTOW/week/leapS/valid flags/tAcc。

必须记录每条消息的 host receive steady time、driver connection generation、driver process session identity。ROS header stamp 只作为 driver 接收路径的附加诊断数据。

### 4.2 精确 epoch join

先按 receiver connection/session 和 iTOW 建立 bounded exact join，不使用 nearest-time：
- PVT 与 HPPOSLLH/COV/STATUS 维持现有四路精确配对。
- TIMEUTC/TIMELS 若用于时间锚点，也必须匹配同一 iTOW；不匹配时不得将“最近一条”时间状态套用到当前位置 epoch。
- TIMEGPS 若启用，同样 exact-iTOW 对齐。
- 每个 epoch 必须通过各消息自己的 validity flags；只匹配 iTOW 不代表内容有效。

### 4.3 UTC 绝对时间锚点

候选转换：
1. 验证 PVT valid_date && valid_time && fully_resolved。
2. 若 confirmed_avail=true，要求 confirmed_date && confirmed_time；否则记录为 valid-but-unconfirmed，不能用作 reboot/generation 变更的唯一证据。
3. 验证 TIMEUTC valid_utc 和可接受 utc_std；其 UTC date/time 与 PVT 同历元数据经 nano 规范化后必须一致。
4. 取得同历元或经明确定义 freshness policy 允许的 NAV-TIMELS currLs，要求 validCurrLs=true，且 source 不是手册指出可能过时的默认来源或 unknown；来源白名单需经目标固件验证后冻结。
5. 将规范化 UTC + 当前 GPS−UTC leap-second offset 映射到 GPS absolute time；使用能表示 leap second 60 和负 nano 日历借位的整数时间算法，不依赖普通 POSIX timestamp 的隐式归一化。
6. 从 GPS epoch（1980-01-06）计算完整 GPS week 与 TOW；将其与 NAV-STATUS tow_set/wkn_set、TIMEUTC valid_wkn/valid_tow 和原始 iTOW 做一致性检查。
7. 只有时间有效性、leap-second 来源、epoch join、source monotonicity 全部通过，才输出 validated source timestamp 与 source clock mapping identity。

valid_wkn / wkn_set 只是有效性标志，不提供 week 数值；完整 week 由经过验证的 UTC+leap-second 计算得出。若未来接入 NAV-TIMEGPS，可将它作为独立协议交叉检查，发现不一致即进入 AMBIGUOUS_INVALID。

### 4.4 无法映射时的行为

以下任一情况进入 AMBIGUOUS_INVALID，撤销 GNSS measurement validity：
- PVT valid_date/valid_time/fully_resolved 不满足；
- confirmed_avail=true 但 confirmed_date/confirmed_time 不满足；
- TIMEUTC valid_utc=false 或 UTC standard 不可接受；
- NAV-TIMELS validCurrLs=false、来源不可信或 offset 与已验证时间不一致；
- leap second 60、负 nano、跨日期/年边界无法被正确处理；
- 同历元 UTC 结果不一致；
- iTOW 与独立 UTC/GPS 时间映射不一致；
- 时间回退、跳变、重复 epoch 冲突或跨 session 混合；
- host steady freshness 无法确认。

不得回退到 ROS header stamp、system wall clock、最近一条旧 leap-second offset 或单纯 iTOW 来伪造 source timestamp。

## 5. Receiver generation 与时间连续性仍是两条独立轴

UTC 时间映射解决的是“测量发生在绝对时间轴的哪里”，并不提供 receiver boot UUID：
- 设备重启后可能很快恢复相同 UTC 时间源，UTC 单调并不证明没有 reboot。
- msss 回退可能是 32-bit wrap，也可能是 reboot；需要 host steady elapsed、driver detach/attach generation、driver reset service/event 和 source time 共同判断。
- driver process restart 必须产生新的 process session identity。
- USB detach/attach 必须由 driver 显式发布 lifecycle event 或由 supervisor 提供不可丢失的 connection generation。
- 同一 receiver session 内的 GPS week rollover 不应触发 generation reset。
- 发生 receiver reboot/connection replacement 时先 invalid，只有新 session identity + 新时间锚点 + reset transaction 成功后才建立新 generation。

不得把 source_timestamp_ns、gnss_receiver_generation、driver_connection_generation 和 reset_transaction_id 合并成一个字段。

## 6. 固定驱动的最小扩展范围

固定驱动已支持 NAV-PVT、NAV-TIMEUTC、NAV-STATUS；新增时间映射所需的最小缺口不是简单的 NAV-TIMEGPS，而是：
1. NAV-TIMELS message definition、payload parser、frame dispatch、publisher 和 ROS message。
2. USB attach/detach lifecycle event / connection generation topic 或 service-visible state，确保 detach/attach 不丢事件。
3. driver process session identity（每次进程启动唯一）以及 reset-service acknowledgement 与 receiver generation 的关联。
4. 可选 NAV-TIMEGPS parser/message/publisher，作为协议诊断交叉校验，而非默认时间主权威。
5. 明确目标设备 MON-VER firmware/protocol version 和实际 NAV message output config；未验证前不得宣称设备支持所选消息。
6. ROS message build/install 与 Humble/Jazzy CI；parser unit tests 覆盖 payload offsets、signed fields、valid bits、invalid payload lengths 和 endian decoding。

CMakeLists 中 ublox_ubx_msgs 使用显式 rosidl_generate_interfaces(...) 消息列表；新增 msg 必须登记。driver 的 ubx_nav.hpp 需增加 payload include/type/member/constructor/accessor/frame switch；ublox_dgnss_node.cpp 需增加 include、publisher 和 publish path。不能只添加 .msg 文件就认为驱动支持。

## 7. 新增测试矩阵

| 测试 | 输入 | 预期 |
|---|---|---|
| PVT valid | validDate/time/fullyResolved true | 进入 UTC candidate |
| PVT unconfirmed | confirmedAvail false | 标记 unconfirmed；不得独立触发 NEW_GENERATION |
| PVT confirmed | confirmedAvail true 且两确认位 true | 可进入高置信 UTC anchor |
| PVT confirmation failure | confirmedAvail true 且任一确认位 false | anchor invalid |
| TIMEUTC validity | validUTC/TOW/WKN 组合变化 | 按各自语义接受，不把 validWKN 当 week number |
| UTC standard | known / unavailable / unknown | 按冻结的标准来源策略接受或拒绝 |
| NAV-TIMELS source | validCurrLs true/false，source 0/255/其他 | default/unknown 不作为可信 leap offset |
| nano negative | 负 nano 跨秒/分/日/月/年 | 正确日历借位 |
| leap second | sec=60、lsChange 边界 | 精确处理或 fail-closed，不静默归一化 |
| week mapping | UTC + current LS | 计算完整 GPS week/TOW |
| iTOW consistency | 同历元匹配/不匹配 | 一致通过，不一致 invalid |
| GPS week rollover | UTC 连续跨周、同 session | source timestamp 连续，generation 不变 |
| driver reconnect | connection generation 改变 | 旧 cache flush，旧消息 fence |
| msss wrap | uint32 自然回绕与 reboot 候选 | 仅按多证据分类，无法区分则 ambiguous |
| process restart | driver process session 改变 | 旧消息拒绝，新 anchor 重新建立 |
| stale leap seconds | TIMELS 缺失/超龄 | 不复用无限期旧 offset |

所有阈值与来源白名单须经目标 F9P firmware 和真实日志标定；单元测试不构成硬件证据。

## 8. 决策与未决 Gate

- [x] 固定驱动 NAV-PVT/NAV-TIMEUTC 已有字段与消息生成链核对。
- [x] u-blox Interface Description 中 NAV-TIMEGPS/TIMELS/TIMEUTC 字段与 validity flags 核对。
- [x] Integration Manual 对 iTOW、UTC validity、confirmedDate/Time、fullyResolved 和 nano 的语义核对。
- [ ] 用户提供或现场读取目标 ZED-F9P 的 MON-VER firmware/protocol version。
- [ ] 确认 NAV-TIMELS 在目标固件可输出，实际配置与有效来源字段。
- [ ] 固定 NAV-TIMELS source policy 与 stale age。
- [ ] 在固定 driver 增加 NAV-TIMELS + connection lifecycle generation。
- [ ] 可选增加 NAV-TIMEGPS 作交叉校验，按目标协议版本测试。
- [ ] 真实设备日志覆盖 cold/warm/hot start、USB reconnect、week rollover、leap-second validity 与长期运行。
- [ ] 完成 UTC→GPS mapper 单测、property/fuzz 和 Humble/Jazzy CI。

## 9. 固定来源

- ZED-F9P Integration Manual UBX-18010802 R12：https://device.report/m/ee4e609d58b9d7b4e732c64036f560ec18909d6e10238c818a860750f56ad4a2
- ZED-F9P Interface Description UBX-18010854 R07：https://cdn.sparkfun.com/assets/f/7/4/3/5/PM-15136.pdf
- NAV-TIMEGPS/NAV-TIMELS/NAV-TIMEUTC protocol sections：https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_UBX_NMEA_and_RTCM_protocols.pdf
- Fixed driver commit：https://github.com/aussierobots/ublox_dgnss/tree/9d4f5addbc72d092c57dde99b20712258e96e81e
- UBXNavPVT.msg：https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavPVT.msg
- UBXNavTimeUTC.msg：https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/msg/UBXNavTimeUTC.msg
- ubx_nav.hpp：https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_dgnss_node/include/ublox_dgnss_node/ubx/ubx_nav.hpp
- ublox_ubx_msgs CMakeLists.txt：https://github.com/aussierobots/ublox_dgnss/blob/9d4f5addbc72d092c57dde99b20712258e96e81e/ublox_ubx_msgs/CMakeLists.txt

**最终判断：** 当前固定 driver 已提供 NAV-PVT/NAV-TIMEUTC/NAV-STATUS，但缺 NAV-TIMELS 与 connection-generation event。UTC→GPS 映射可以设计，但在当前 driver 接口和目标 firmware 未确认前不能实现为生产级可信 source clock。NAV-TIMEGPS 是协议支持的可选交叉检查，不应覆盖 Integration Manual 的 UTC 时间建议。


## 10. 固定 driver 中的具体扩展插入点与 lifecycle 并发约束

进一步追踪固定 driver 的 NAV-TIMEUTC 发布链和 libusb event path，得到以下实现边界。

### 10.1 NAV-TIMELS parser / ROS message 插入点

固定版本不是通过单独 .cpp 为每条 NAV message 实现 parser，而是采用 header payload class + UbxNav dispatch + node publisher 的模式。新增 NAV-TIMELS 需要至少修改：

1. 新增 ublox_ubx_msgs/msg/UBXNavTIMELS.msg，并在 ublox_ubx_msgs/CMakeLists.txt 的显式 rosidl_generate_interfaces 列表登记。
2. 新增 nav payload header，定义消息 class/id、24-byte payload 字段 offset、signedness 和 validity bitfield；parser 需验证 frame payload length，不能仅 memcpy 后无条件读取。
3. 在 ubx.hpp 增加 UBX_NAV_TIMELS message ID 常量（如固定版本尚无该常量）。
4. 在 ubx_nav.hpp 增加 include、FrameComms typedef、成员、构造初始化、accessor 和 frame switch 分支。
5. 在 ublox_dgnss_node.cpp 增加 ROS message include、publisher 创建/member、frame dispatch 和 publish method；字段必须逐项映射并保留 raw source/valid flags。
6. 在 driver CI 增加消息字段、payload parser 和 ROS build/install 检查；现有 driver CI 只覆盖原固定 commit，不能证明新 patch。

NAV-TIMEGPS 若作为可选交叉校验，采用同样扩展模式，但应与 NAV-TIMELS 分开评审、分开测试；不因增加 TIMEGPS 就认为当前 leap-second source 已可信。

### 10.2 USB lifecycle callback 的实际执行路径

固定 driver：
- libusb hotplug attach/detach callback 在 usb.cpp 注册。
- node 的 handle_usb_events_callback 由 10 ms wall timer 调用 usbc_->handle_usb_events()；该 timer 位于 callback_group_usb_events_timer_（MutuallyExclusive）。
- handle_usb_events() 调用 libusb_handle_events_timeout()；libusb hotplug 和 transfer callbacks 在该 event processing 路径中被调用。
- usb::Connection::hotplug_attach_callback() 打开设备、设置 CONNECTED/attached_ 后调用 node attach callback。
- usb::Connection::hotplug_detach_callback() 关闭 device handle、设置 DISCONNECTED/attached_=false 后调用 node detach callback。
- node attach callback 在重连时同步执行 perform_usb_initialization()；detach callback 仅更新 readiness/attached 标志并 reset device parameters。
- UBX transfer callback ublox_in_callback() 以 rclcpp::Clock().now() 记时间，将原始 frame push 到 ubx_queue_；push 有 ubx_queue_mutex_ 保护。
- ubx_timer_callback() 位于另一个 MutuallyExclusive callback group；当前实现读取 queue size/front 时没有持有 ubx_queue_mutex_，pop_front 时才加锁。若组件运行在 MultiThreadedExecutor / multithreaded component container 下，不能假设 USB event callback 与 UBX consumer 自动串行。

### 10.3 generation event 不能只加一个 ROS topic

必须把 lifecycle generation 和 frame queue ordering 一起设计：

- 每次 driver process 启动创建新的 process_session_id。
- 每次成功 attach 建立新的 driver_connection_generation；detach 立即撤销当前 generation。
- 每个 UBX frame 在 USB callback 入队时捕获 process_session_id + connection_generation + queue_sequence。
- attach/detach lifecycle event 与 UBX frame 使用同一受锁保护的 ingress event queue 或同一单调 sequence allocator，避免 lifecycle event 和旧 frame 之间无可审计的先后顺序。
- detach 事件优先建立 invalid barrier；消费者看到 generation mismatch 的旧队列 frame 必须丢弃，不能在 reconnect 后重新发布为新 session 数据。
- attach 只建立新的 connection session，不代表 GNSS UTC/iTOW 已有效；必须等有效时间锚点与 exact-epoch join 后才恢复 GNSS validity。
- 不能只依赖一个易丢失的瞬时 ROS event topic。建议同时提供 latched/current lifecycle snapshot（含 event sequence、process session、connection generation、attached/readiness）和有序 event stream；subscriber 重启后先同步 snapshot，再接受新事件。
- 事件队列满、序号跳跃、callback race、driver restart、detach/attach 快速连续发生时都必须 fail-closed。

### 10.4 线程安全与队列语义 Gate

固定 node 中 device_attached_、device_readiness_state_、has_been_connected_before_ 是普通成员；USB event timer、parameter-processing timer、UBX timer 属于不同 callback groups。MutuallyExclusive 只约束同一个 callback group 内的执行，不自动串行化不同 groups。若采用多线程 executor，应将跨组共享状态改为受 mutex/atomic 保护的 snapshot/transaction，不允许多个 callback 直接读写裸 bool/enum。

此外，ubx_queue_ 的 size/front 读取与 push/pop 必须统一在同一 mutex 下完成，或改用明确的单生产者/单消费者队列；仅保护 push 和 pop 而不保护读取不能构成完整线程安全合同。

### 10.5 当前审计边界

以上是固定源码静态追踪和项目侧设计要求。未证明当前默认 deployment 使用 MultiThreadedExecutor，也未运行 ThreadSanitizer、USB disconnect stress 或实际 F9P reconnect 测试。不能把潜在并发问题标成已复现 bug；应作为多线程部署条件下的源码风险与测试 Gate。
