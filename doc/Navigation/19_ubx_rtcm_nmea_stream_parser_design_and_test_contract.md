# UBX / RTCM3 / NMEA 增量字节流解析器设计与测试合同

- 状态：总体设计合同；UBX/RTCM3 第一实现切片已落地并完成本地 ASan/UBSan 测试；NMEA、property/fuzz、ROS/libusb 集成仍未实现/验证
- 审计对象：aussierobots/ublox_dgnss@9d4f5addbc72d092c57dde99b20712258e96e81e
- 关联审计：18_ublox_dgnss_fixed_revision_time_message_driver_audit.md
- 项目分支：nav
- 目标：定义可独立单测的 USB byte-stream decoder，替换“一个 libusb transfer 就是一条协议消息”的假设。

## 0. 当前实现状态（2026-11-26）

已新增 `navigation/ubx_rtcm_stream/`，当前实现覆盖 UBX、RTCM3 与 NMEA 的纯 C++17 增量解析，不依赖 ROS/libusb。UBX 使用 CK_A/CK_B，RTCM3 使用 CRC24Q；NMEA 支持 CR/LF/CRLF、XOR checksum、Valid/Absent 状态和可配置 checksum-required 策略。三类协议共用有界 buffer、分片/拼接、noise resync、generation reset 与 frame sequence。

本地 GCC 14.2 + ASan/UBSan 构建通过，10 组测试通过；混合流执行 100 组确定性随机 chunk segmentation，与一次性输入逐帧比较一致。Clang 17 libFuzzer target 已构建，20 秒 smoke fuzz 约 405,298 次输入变异，无崩溃或 sanitizer 报告。上述均为本地容器验证；新增 CI fuzz smoke 尚待 GitHub Actions 实际运行。

尚未接入 ROS/libusb；没有修复 USB callback、UBX/RTCM 队列并发或 transfer lifecycle；没有真实 ZED-F9P capture，也没有目标设备验证。NMEA 句长默认值 1024 是解析器资源策略，不是接收机协议保证，生产配置需按实际输出配置确认。

## 1. 设计约束

1. libusb transfer 是传输层交付单元，不是 UBX/RTCM/NMEA 消息边界。
2. parser 输入为任意长度 byte span；允许空输入、单字节输入、任意分片和任意拼接。
3. parser 不访问 ROS、libusb、设备状态、系统时间或全局状态；输出完整、结构校验通过的 frame event。
4. parser 必须有明确的内存上限、单次处理预算和可观测的丢弃计数，不能因声明长度或持续噪声无限增长。
5. UBX/RTCM frame 必须在完整性校验成功后才可交付；NMEA 的校验策略须配置并在事件中保留 checksum 状态，不能将“缺少 checksum”静默等同于已验证。
6. USB detach/reconnect 必须调用 parser reset，并推进 connection generation；不允许跨连接拼接半帧。

## 2. 协议 frame 合同

### 2.1 UBX

- Sync：B5 62（以固定版本实际常量为准；代码注释中的十六进制描述需核对，禁止沿用错误注释）。
- Header：class、id、payload length（little-endian uint16）。
- 总帧长度：payload_length + 8。
- 在至少 6 bytes header 到齐之前不得读取 length；在完整长度到齐之前不得读取 checksum 或暴露 payload。
- checksum：UBX CK_A/CK_B，覆盖 class 至 payload。
- 接收长度必须精确等于声明总长；多余字节由 stream loop 继续解析为下一帧，不可并入当前 frame。
- 协议最大 payload 字段值是 65535；部署层应另行定义可接受 payload ceiling，依据目标 ZED-F9P 消息白名单及内存预算确定。不得在没有消息清单依据时随意将 4 KiB 等值写成协议上限。
- malformed length、checksum mismatch、未知 class/id 的处理策略分离：前两者拒绝；未知消息可作为“完整且校验通过的原始 UBX frame”交上层策略决定。

### 2.2 RTCM3

- Preamble：0xD3。
- 第二字节的高 6 bit 为 reserved，必须为 0；低 2 bit 是 10-bit payload length 的高 2 bit。
- payload length：((byte1 & 0x03) << 8) | byte2，范围 0–1023。
- 总帧长度：payload_length + 6（3-byte header + payload + 3-byte CRC）。
- CRC24Q 必须对 header + payload 计算，并与末尾 3 bytes 比较；CRC 不匹配不得发布。
- 不能要求第二字节严格等于 0x00；这会丢弃 payload length 256–1023 的合法 frame。
- 完整 RTCM frame 之后继续扫描同一 buffer；半帧留待后续输入；坏 CRC 的恢复策略需记录丢弃字节数并重新同步，不能盲目跳过未知长度之外的后续 frame。

### 2.3 NMEA

- 句子起始符 $；以 CR/LF 结束（接受 CRLF，明确处理单独 CR/LF）。
- 句子最大长度采用可配置上限；默认值需依据目标接收机输出配置和设备实测确定，不能直接把传统 82 字符限制当成所有 u-blox 扩展句子的保证。
- 若句子包含 *HH checksum，验证 XOR；若缺失 checksum，按配置标记为 absent，而非 valid。
- 不得对 libusb transfer 原始 buffer 写入 NUL terminator；输出拥有独立生命周期的 string/byte vector。
- 噪声及不完整句子在达到长度上限或超时/连接 reset 时有界丢弃。

## 3. 混合协议扫描优先级

在当前 offset：
1. 若匹配 UBX sync，按 UBX header/length 等待完整 frame；若 header/length 非法，执行明确的 resync 策略。
2. 若匹配 RTCM preamble，校验 reserved bits、解码长度、等待完整 frame 并验证 CRC24Q。
3. 若匹配 NMEA $，等待行终止并执行 checksum 状态判定。
4. 否则丢弃一个 byte 并增加 noise counter。

协议候选冲突与重同步规则必须有确定性单测。遇到不完整候选时保留足够尾部字节（例如单字节 UBX sync 前缀），不能提前丢弃可能构成下一次输入开头的字节。

## 4. 推荐模块边界

- StreamDecoder：维护 bounded byte buffer，接收任意 chunk，扫描并输出完整事件。
- UbxFrameView/OwnedFrame：只在 frame length 与 checksum 已验证后构造。
- Rtcm3Frame：只在 header/length/CRC24Q 验证后构造。
- NmeaSentence：独立字符串存储，checksum 状态为 valid / absent / invalid。
- DecoderStats：输入 bytes、输出帧数、noise bytes、malformed headers、length rejects、checksum rejects、buffer-limit drops、resync count。
- reset(connection_generation)：清空所有 partial frame，并绑定新的 generation。

parser 输出建议携带：protocol kind、owned raw bytes、monotonic receive sequence、connection generation、transfer receive timestamp（同一个 transfer 内拆出的多个 frame 可共享接收时间，但 sequence 必须不同）、protocol checksum/validation status。

## 5. USB / ROS 集成合同

1. libusb callback 只把 actual_length 范围内的 bytes 交给 decoder；不修改 libusb buffer。
2. 对 actual_length 小于 0 或大于 transfer capacity 的不可能状态作防御性拒绝并计数。
3. decoder 只由一个受控执行上下文调用，或内部明确串行化；不要让不同 timer callback group 并发调用同一 decoder 实例。
4. 完整 frame event 再进入 UBX/RTCM 各自的有界队列；队列消费采用锁内 front-copy/move + pop，锁外执行业务逻辑。
5. queue 满时采用显式策略（例如丢弃最旧/最新、按协议优先级），记录 drop counter；禁止无界 deque 增长。
6. detach/shutdown/reset 必须清除 partial buffer、队列旧代数据，并增加 generation fence，防止重连前后的片段被拼接。
7. ROS 发布前 UBX checksum 与 RTCM CRC24Q 必须已通过；NMEA checksum 状态必须可观测。

## 6. 单元测试合同

### 6.1 通用 stream 行为

- 空输入、单字节输入、逐字节输入；
- 一帧拆成 2、3、多个 chunk；
- 两帧及多帧拼接在单一 chunk；
- frame 尾部 + 下一帧头部拼接；
- 随机噪声前缀/帧间噪声；
- partial sync（最后一个 byte 是 UBX sync 首字节）；
- 连续 malformed frame 后跟合法 frame，必须重新同步；
- buffer 上限边界与超限输入；
- reset 后旧 partial frame 不得与新 generation 数据拼接；
- 随机 chunk segmentation 下，输出帧序列应与一次性完整输入一致（property test）。

### 6.2 UBX

- 最小 payload=0、典型 payload、payload ceiling；
- 3/7 bytes truncated input 不得越界；
- header 未完整时不得读取 length；
- 声明长度小于/大于实际 frame；
- 单独 CK_A 错、单独 CK_B 错、两个都错；
- valid frame 后附加 valid frame；
- 伪 sync、错误 sync、长度上限拒绝；
- unknown class/id 仍能安全保留原始完整帧。

### 6.3 RTCM3

- payload length 0、1、255、256、511、1023；
- reserved bits 非零；
- header 仅 1/2 bytes；
- truncated payload / truncated CRC；
- CRC24Q 正确、CRC 任一 byte 错误；
- 多帧拼接、跨 chunk 拆帧；
- bad CRC frame 后跟 valid frame，必须恢复同步；
- 第二字节为 0x01、0x02、0x03 的合法 length 高位场景。

### 6.4 NMEA

- CRLF、CR、LF 终止策略；
- checksum valid / invalid / absent；
- 超长句、无终止符、噪声 $；
- transfer buffer 边界输入，不写越界、不依赖 NUL terminator；
- 多句子在一个 chunk、句子跨多个 chunk。

### 6.5 Sanitizer / fuzz / concurrency

- ASan + UBSan 跑全部单元测试；
- libFuzzer/AFL 对 byte stream decoder fuzz，固定 seed 可重放；
- TSAN 验证队列生产/消费与 parser ownership；
- 断连、取消、shutdown 与 callback 交错压力测试；
- 目标部署的 component_container_mt 下重复运行，验证无 data race、use-after-free、double free、deadlock。

## 7. 实施顺序与验收门槛

1. 新建纯 C++ parser library 与协议独立测试，不接 ROS/libusb。
2. 完成 UBX/RTCM/NMEA parser 和 sanitizer 测试。
3. 在 node 中以 parser 替换 transfer-is-frame 分支，保持现有 ROS message API 兼容。
4. 修复 UBX/RTCM queue 的锁内消费，并加入有界队列/丢弃统计。
5. 修复 USB transfer lifecycle 与 callback cleanup，增加 fake libusb seam 或可注入 transport 测试。
6. 构建目标 ROS 2 Jazzy workspace，跑组件容器 MT 集成测试。
7. 有设备后做 F9P USB capture 对照；未做设备测试不得声称硬件验收通过。

**当前状态：以上全部为待实现合同。没有声称任何代码已修复或测试通过。**