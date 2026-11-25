# GNSS 驯服 PTP Grandmaster 与 PPS/时间标签分发：器件级比较与 ADR 候选

- 文档状态：N1 设计审计 / ADR 候选（PROPOSED，未批准、未采购）
- 更新日期：2026-11-25
- 适用预选：Livox MID-360、2 × e-con See3CAM_50CUG_CXXCC_BX_H01R1、u-blox ZED-F9P、Jetson AGX Orin Developer Kit
- 关联文档：[15_sensor_clock_domains_and_sync_architecture.md](15_sensor_clock_domains_and_sync_architecture.md)
- 证据等级：E1 厂商/标准官方文档；E2 厂商官方手册/论坛或公开产品资料；E3 本项目架构推断；E4 目标设备实测（本轮没有 E4）

## 1. 决策摘要

比较三条路线：

- **A — GNSS-disciplined PTP Grandmaster：** GNSS 天线/接收机向独立 GM 提供时间参考；GM 向以太网设备发布 PTP；Jetson 通过 NIC PHC 与系统时钟建立受控映射；MID-360 作为 PTP slave。相机若无 PTP，仍需独立公共硬件 trigger 与曝光 strobe 回采。
- **B — GNSS PPS + epoch/time-label fanout：** GNSS 接收机提供 PPS 和与该 PPS 上升沿精确关联的时间标签；隔离/缓冲/扇出模块把脉冲送往 MID-360、相机 trigger 和可同步的本体接口；主机接收标签并维护各设备时钟映射。
- **C — NTP/软件接收时间：** 仅作诊断/低精度降级，不作为 LIO deskew 或高动态 stereo/VIO 的默认验收路径。

**候选结论：A 作为系统主同步架构优先进入下一轮设计；B 保留为可独立验证的备选/局部同步路径，不直接认定为 A 的无缝冗余。** 这不是采购决定：TP4100、SecureSync 或其他 GM 均未选定；F9P、目标载板、交换机和 Color H01R1 相机的真实接口尚未验收。即使采用 A，也应把 PPS/时间标签作为独立观测与故障诊断输入，而不是未经验证地并接到设备。

## 2. 目标设备约束

### 2.1 MID-360 是协议选择的关键约束

Livox 官方文档确认 MID-360 支持：
- PTP：IEEE 1588v2.0 UDP/IP；
- gPTP：Layer 2；
- GPS：PPS + GPRMC 时间标签；
- GPS 时间范围限制为 2000-01-01 至 2037-12-31；
- 不支持 IEEE 1588v2.1；不建议在同一应用场景中混用 IEEE 1588v2.0 与 gPTP。

协议定义 time_type 0/1/2：未同步开机时钟、PTP/gPTP master 时间、GPS 时间。数据包 timestamp 是该包首个点云时间；time_interval 描述包内点时间间隔；time_offset = t0 − t1 是设备本地时间相对同步源的诊断偏移。Livox 时间同步说明指出 PTP 与 GPS 同时可用时 PTP 优先；网络存在多个 master clock 的场景不受支持。

因此本项目必须：
1. 在 MID-360 的单个运行配置中明确选用 PTP v2.0 或 GPS/PPS，不把 PTP 与 gPTP 当作可随意并存的双网冗余；
2. 采集 time_type、time_sync_type、last_sync_time、time_offset、local_time_now 和设备重启计数；
3. 对任何源切换、GM identity/domain 变化或 time_type 变化递增 mapping_generation，并清空跨 generation 的同步队列；
4. 通过设备实测确定 PTP 丢失后是否会自动转入 GPS，以及恢复时是否发生时间 step；公开优先级说明不等于已证明所有失效切换行为。

### 2.2 ZED-F9P 的能力边界

u-blox 官方产品资料列出 ZED-F9P configurable timepulse 输出（产品摘要给出 0.25 Hz 至 10 MHz），并提供 UBX-TIM-TP 消息报告下一脉冲的时间信息与时间源。Integration Manual 建议时间脉冲 time base 与实际可用 GNSS 信号匹配；对 UTC 绝对精度有要求时，建议用参考时间源校准完整用户系统，并考虑 GNSS 模式与温度变化。

这说明 F9P 可作为 GNSS 接收机/PPS 及时间标签来源，但**不能据此把 F9P 当成网络 PTP Grandmaster**。方案 A 需要独立 GM 或经验证的 PTP GM 软件/硬件平台；方案 B 需要把 F9P PPS 与其精确历元标签配对。不能只把 PPS 接入设备而忽略对应秒编号。

F9P 产品摘要中的 TCXO 和可配置 TIMEPULSE 频率不是整机 holdover 精度保证。GNSS 失锁后的 PPS 相位漂移、有效性标志、时间标签连续性及重锁行为必须在目标模块 SKU/固件上实测；本 ADR 不给 F9P 推定 holdover 时间或纳秒精度。

## 3. 两种拓扑与误差链

### 3.1 A：GNSS-disciplined PTP GM

    GNSS antenna(s)
       │ RF / antenna cable / surge protection
       ▼
    GNSS receiver + time/epoch reference
       │ 1PPS + time-of-day / reference input
       ▼
    PTP Grandmaster (oscillator + servo + source monitor)
       │ IEEE 1588-2008 PTP over isolated timing LAN
       ├──────────────► MID-360 PTP slave (time_type=1)
       ├──────────────► Jetson NIC PHC ── phc2sys/chrony ── CLOCK_REALTIME
       └──────────────► PTP-aware switch / other PTP endpoints

    Camera pair: shared trigger generator/fanout ─► camera Trigger
                 camera Strobe ─► independent capture/measurement input

**端到端时间误差项：**
- GNSS 天线位置/多径/遮挡/干扰、接收机时间解算与 PPS 输出相位；
- GM 输入接口的 PPS/ToD 对齐、GM servo、内部振荡器噪声/老化/温度；
- PTP 网络拓扑、交换机 residence time、链路不对称、队列拥塞与非 PTP-aware hop；
- GM 与 slave 的 PTP profile/domain/transport/delay mechanism 配置；
- Jetson NIC PHC 硬件时间戳、驱动、PHC servo 和 PHC↔CLOCK_REALTIME cross timestamp；
- MID-360 内部 PTP servo、设备时钟分辨率、点云 timestamp 生成语义；
- 若跨到相机：公共 trigger 分发延迟/通道偏差、传感器触发至曝光延迟、strobe 定义和采集时钟。

不可把 GM 的产品指标直接当作 MID-360 端到端误差。总误差预算应逐项测量。已知误差有相关性时不得机械 RSS；没有相关性证明时先采用保守 worst-case bound。网络 asymmetry 可能形成系统性偏差，不会因提高采样次数自动消失。

**失锁路径：**
1. GNSS 失锁但 GM 仍有有效内部振荡器：进入 HOLDOVER；持续输出不代表仍与 UTC 锁定，uncertainty 必须按振荡器规格/学习状态随时间增长。
2. GM reference 无效或可信度受损：应输出明确 alarm/source state；项目端必须停止把它标记为 LOCKED。
3. GM reboot/power loss：holdover 能否跨重启必须按具体设备说明确认；不能默认时间连续。
4. 网络断链/交换机重启/PHC reset：MID-360、Jetson 分别进入 LOST/DEGRADED；重连后必须重新确认 GM identity、domain、servo lock 和 generation。
5. GM 切换：双 GM 必须有唯一、可审计的主时钟选择；切换时记录 old/new GM、phase step、servo state 和设备端响应。

### 3.2 B：PPS + epoch/time-label fanout

    GNSS antenna
       ▼
    ZED-F9P (TIMEPULSE + UBX-TIM-TP / NAV-PVT / GPRMC)
       ├── PPS ─► isolated buffer / fanout ─► MID-360 PPS
       │                                  ├► camera trigger generator (only after H01R1 electrical approval)
       │                                  └► robot timing input (if vendor confirms)
       └── pulse-associated epoch label ─► host serial/USB or UDP timing adapter
                                          └► per-device epoch/mapping registry

    Camera Strobe ─► independent timestamp capture / logic analyzer

**端到端时间误差项：**
- GNSS receiver PPS phase/time-base uncertainty；
- PPS 上升沿与 epoch label 的精确关联、消息发布时间/串口排队、丢帧/乱序；
- PPS fanout buffer 的 channel-to-channel skew、传播延迟、隔离器延迟、阈值/边沿 slew；
- 线缆长度/传播速度/温度、连接器与终端负载；
- 每个设备 PPS 输入阈值、同步捕获器量化、内部锁相与 timestamp 生成；
- MID-360 接收 PPS 与 GPRMC/UDP 时间标签的配对窗口和失配行为；
- 相机 trigger 输入延迟、sensor exposure start 延迟、左右相机 strobe 偏差；
- 主机从串口/USB 接收时间标签的时间戳不可替代 PPS 边沿本身的采样时刻。

**关键风险：** PPS 边沿即使非常稳定，只要标签对应错一秒，绝对时间就错一秒；标签正确但延迟/乱序也可能被错误配对。必须使用明确的 pulse sequence / GNSS week+iTOW 或完整 UTC label 关联；接收时刻仅作 freshness 诊断，不能作为脉冲历元。

**失锁路径：**
1. GNSS invalid/fix lost/time invalid：停止生成新的 LOCKED epoch label；PPS 可能仍有输出，但不得因仍有方波而继续信任绝对时间。
2. GNSS PPS 丢失：fanout 输出失效状态；设备保持内部自由运行只能标 HOLDOVER/LOST，按实测漂移增长 uncertainty。
3. 标签丢失但 PPS 存在：绝对历元无法确认，进入 INVALID/UNMAPPED；禁止用上一秒标签无限延续。
4. 单路 fanout/电缆故障：只影响对应端口，但必须逐路监测/回采；无端口状态反馈的被动 Y 分线不构成可诊断冗余。
5. F9P reboot/USB 重连：receiver generation 改变；丢弃旧 label cache，重新等待可信 epoch/validity。
6. fanout board reboot：输出是否出现毛刺/额外边沿必须实测；设备侧 generation 与 pulse counter 重新对齐。

### 3.3 精度预算方法（不预填未经测量的数值）

对每个消费者 i 建立：
- E_ref：GNSS/UTC 或 GPST reference realization 与 receiver PPS 误差；
- E_source：GM servo 或 PPS 输出接口误差；
- E_distribution,i：网络 asymmetry 或 fanout/cable channel delay；
- E_capture,i：NIC/设备输入捕获量化和硬件 timestamp 误差；
- E_device,i：设备内部 servo、曝光/点云 timestamp 语义误差；
- E_mapping,i：source→target 映射估计误差与 uncertainty。

初始保守上界：
    E_i,max ≤ E_ref + E_source + E_distribution,i + E_capture,i + E_device,i + E_mapping,i

只有在误差独立性、统计定义、置信水平与测量方法都已证明后，才允许另报 RSS/统计标准差。所有指标必须注明 1σ/3σ/峰值、观测时间、温度、GNSS 模式、网络拓扑、设备固件和测量仪器校准状态。

## 4. 公开器件候选对照（不是采购推荐）

| 候选 | 已由公开资料确认 | 主要适配点 | 未确认/限制 |
|---|---|---|---|
| Microchip TimeProvider 4100 | IEEE 1588 PTP GM；PTP/NTP/SyncE；GNSS；可选 1PPS/ToD；多种 OCXO/Super OCXO/Rubidium；双机 active/standby 或 active/active 方案；产品页列出 802.1AS-2020 与电信/电力 profile | 可做高可靠外部 GM 候选；有 GNSS anti-jam/anti-spoof 选项与冗余产品路径 | 必须确认所购具体硬件/软件许可、PTP profile 与 MID-360 v2.0 UDP/IP 互通；802.1AS 支持不等于可与 MID-360 PTP v2.0 混合部署；价格、交期、功耗、尺寸、实际 holdover 型号和输出口配置待报价/数据手册 |
| Safran SecureSync 2400 / 1204-32 PTP option | SecureSync 2400 手册列 IEEE 1588-2008 PTP v2、Default/Telecom/Enterprise/Power/802.1AS-2011 profile；1204-32 卡规格列 PTP v2、30 ns 3σ master-to-slave（crossover cable 条件）、8 ns timestamp resolution；holdover 依内部 TCXO/OCXO/Rubidium 型号变化 | 可作为独立 PTP/1PPS 时间设备候选；手册详细描述 holdover/alarm | 30 ns 指定于 1204-32 卡和 crossover cable 条件，不能外推到交换网络或 MID-360；holdover 数值按振荡器与时间而变；holdover timeout 后失同步，且手册说明 holdover 不跨 reboot/power cycle；具体端口/选件/价格待确认 |
| ZED-F9P + PPS fanout board | ZED-F9P 有 configurable TIMEPULSE、UBX-TIM-TP pulse-time metadata；MID-360 有 PPS+GPRMC GPS sync 模式 | 成本/体积有机会较低；适合独立 PPS/标签路径及台架测量 | F9P 不是 PTP GM；TCXO 不等于有保证的长时间 holdover；需要自研/采购隔离扇出、标签配对和逐路诊断；F9P 实际 SKU、固件、输出电平/极性/脉宽与精度需确认 |
| GNSS receiver + industrial timing/fanout appliance（未指定型号） | 属于可采购产品类别，尚无本项目指定候选的受控规格 | 可把 GNSS 输入、PPS/ToD 输出、隔离/多路扇出、告警与冗余整合 | 未获得具体厂商型号和正式规格，当前不得列为已选器件 |

**注意：** TimeProvider 4100 产品页的 ePRTC 30 ns / 40-day holdover 属于特定 ePRTC 配置/参考链路描述，不得当成普通基础型号、单 GNSS 输入或本项目网络端到端保证。Microchip 公布的 holdover 表也明确依振荡器、预热/学习状态和温度条件变化。SecureSync 的 holdover 指标同样依振荡器类型及时间长度，不同设备条件不可直接横向排名。

## 5. 精度、接口、故障与冗余比较

| 维度 | A：PTP GM | B：PPS + time-label fanout |
|---|---|---|
| 绝对时间标签 | 由 GM PTP timescale/UTC traceability 传递；必须核对 GM 的 UTC offset、timescale、leap handling | 由 GNSS receiver 的 pulse-associated epoch label 传递；标签必须与具体 PPS edge 精确配对 |
| 设备同步接口 | Ethernet PTP；需要端点 PTP profile/transport/delay mechanism 兼容 | PPS 电气输入 + UART/NMEA/UDP label；每个设备接口均需核对 |
| 主要精度瓶颈 | GNSS→GM servo、网络 asymmetry、switch/NIC timestamp、endpoint servo | PPS 输出、fanout channel skew、线缆延迟、输入捕获、label association |
| MID-360 | 官方支持 IEEE1588v2.0 UDP/IP；可 time_type=1 | 官方支持 PPS+GPRMC；GPS 时间有 2000–2037 限制 |
| Jetson | 目标接口需 PHC + HW timestamp；AGX Orin Developer Kit 有公开能力证据，但目标板未测 | 需 PPS capture GPIO/HTE 或外置时间采集器；PHC 非必需，但系统时钟映射仍需设计 |
| 相机 | PTP 通常不能直接解决 USB 相机曝光同步；仍需 shared trigger/strobe | shared trigger 可与 PPS 同源，但相机输入电气/曝光语义先验未知 |
| GNSS 失锁 | GM 按内部 oscillator holdover，状态与 uncertainty 可集中监控 | 接收机/分配板按自身 oscillator/free-run；各设备可能分散 holdover，需逐设备监测 |
| 网络故障 | 影响 PTP 端点；可分网/双网，但双路径需验证 asymmetry 与 GM failover | PPS 物理分发不依赖 Ethernet；标签路径仍可能依赖 UART/UDP，需独立监测 |
| 单点故障 | 单 GNSS 天线/receiver/GM/交换机均可能成为 SPOF；双 GM 不自动等于独立 GNSS 冗余 | 单 GNSS receiver 或 fanout board 是 SPOF；被动分线无法检测断线/短路 |
| 冗余实现 | 双 GM + 独立 GNSS 接收/天线 + 受控 BMCA/切换；或独立备用 PTP LAN；切换相位必须测 | 双 receiver + 独立天线 + 双 fanout/交叉选择；或 GM + PPS 交叉监测；每个输出需可隔离/监测 |
| 系统复杂度 | 网络配置/PHC/servo/交换机专业度较高，集中监控较好 | 电气设计/布线/线缆延迟/标签协议/逐端口维护较复杂 |
| 采购约束 | GM 设备可能为机架/工业/电信级，体积、功耗、价格、许可/维护成本需评估 | 接收机、隔离器、扇出板、线缆、连接器、保护和校准工具需分别采购/设计 |

## 6. 故障状态机与 fail-safe 合同

所有来源都统一映射到 doc15 的 LOCKED / HOLDOVER / DEGRADED / LOST / INVALID；具体设备原生状态另行保留，不做有损压缩。

- **LOCKED：** 来源身份、domain/timescale、epoch label、servo 状态和测量 uncertainty 均有效且在未来批准预算内。
- **HOLDOVER：** 外部 reference 丢失但内部时钟继续运行；必须附带 holdover 起点、振荡器型号、温度/学习状态、uncertainty growth model 与最大允许时长。
- **DEGRADED：** 仍有可用时间顺序但无法满足高精度预算，例如 software timestamp、无 PHC、网络状态异常或只剩接收时间。
- **LOST：** 同步源不可用，设备时钟自由运行或无有效 pulse。
- **INVALID：** epoch 不明、label/pulse 错配、时间回退、GM identity/domain 未授权、非法 timescale/UTC offset、跨 generation 混合。

**Fail-closed 规则：**
1. 无法证明 pulse→epoch label 配对时，PPS 不能被解释为绝对时间。
2. GM 发生切换或 identity/domain/timescale 改变时，不得无缝继承旧映射；必须新 generation。
3. GNSS 失锁但 PTP servo 仍显示 locked 时，要区分“PTP network locked”与“GNSS UTC traceability”；两者不是同一健康状态。
4. HOLDOVER 超出经测量/规格确认的预算后，禁止用于高精度 LIO/stereo/VIO 配对；不能只依赖设备仍在输出。
5. F9P valid flags、NAV-TIMELS/current leap offset 和 pulse label 不完整时，不发布有效绝对秒标签。
6. MID-360 time_type=0 或状态字段报告不同步时，禁止把 boot-relative 时间并入 GNSS/PTP 时间线。
7. 相机 USB frame arrival timestamp 不得替代 sensor exposure timestamp；无有效 trigger/strobe 证据时，相机同步状态为 UNVERIFIED。
8. 系统 watchdog/E-stop 使用 CLOCK_MONOTONIC/硬件链，不依赖可 step 的 PTP/UTC/ROS Time。

## 7. 采购前器件级确认清单

### 7.1 PTP GM 与网络
- 明确 GM 完整型号、硬件 revision、软件/许可版本、支持 IEEE 1588 edition/profile、UDP/IPv4/IPv6/L2、E2E/P2P delay、one-step/two-step、domain 范围。
- 取得 GNSS 输入类型、天线数量、频段、抗干扰/欺骗检测、UTC traceability、leap-second/continuous UTC 适配声明。
- 取得 oscillator SKU、warm-up、holdover 表格及测试条件；明确掉电/重启是否保留时间和学习状态。
- 取得 PPS/ToD 输出电平、极性、脉宽、连接器、负载、隔离、输出 skew 与可校准能力。
- 确认双 GM active/standby、BMCA、manual priority、phase transient、告警和管理接口。
- 目标交换机是否支持 transparent/boundary clock、硬件时间戳、QoS/队列隔离、冗余电源与独立 timing VLAN。
- 对 MID-360 必须用其支持的 PTP v2.0 UDP/IP profile 做端到端互通验证；不能仅凭 GM 宣称支持 PTPv2/802.1AS 就通过。

### 7.2 PPS/时间标签分发
- F9P 精确 SKU/固件/PROTVER；TIMEPULSE time base、polarity、active edge、pulse length、frequency、locked/valid 状态输出。
- 标签采用 UBX-TIM-TP 与同历元 NAV-PVT/TIMEUTC/TIMELS 的映射规则；对 MID-360 提供的 GPRMC 或 UDP 输入，明确哪个字段标记 PPS 上升沿对应的秒。
- fanout 输入输出电气：电压、逻辑阈值、驱动电流、终端负载、隔离、ESD/surge、反接/短路保护、每路 enable/monitor。
- 逐路测量 channel-to-channel skew、绝对延迟、温度漂移、线缆传播延迟、掉电/重启时额外边沿。
- PPS 计数与 label 序号/历元需可审计；标签迟到、重复、乱序、丢失时 fail-closed。
- 禁止未经 H01R1 Color 对应 revision 正式电气手册，直接把 PPS/fanout 接到相机 Trigger；PPS 与相机曝光 trigger 可能是不同频率/相位/脉宽合同，需独立 trigger generator。

### 7.3 机器人与主机
- AGX Orin 实际载板、MGBE/RGMII/外接 NIC 的 driver、PHC index、ethtool -T、JetPack/L4T/kernel、ptp4l/phc2sys 版本。
- 记录本体控制器是否支持 PTP、PPS、外部 sync input、设备时间读回；无供应商接口不推断可同步。
- 记录电源启动时序、GNSS 天线位置/遮挡、EMI、以太网/相机 USB 线缆走线及接地/隔离。

## 8. 新增测试合同 TIME-022..032

| ID | 测试 | 方法与通过条件 |
|---|---|---|
| TIME-022 | GM→MID-360 PTP profile 互通 | 在指定 PTP v2.0 UDP/IP profile/domain 下，核对 GM announce/sync/follow_up/delay 交换、MID-360 time_type=1 与持续 offset；profile 不匹配必须显式失败 |
| TIME-023 | PTP 网络不对称与交换机影响 | 对比直连/目标交换机/不同负载下的端到端偏差；测量 residence time 与 asymmetry；不能以 GM 本机指标代替 |
| TIME-024 | GM GNSS loss / holdover | 分别注入天线断开、GNSS invalid、RF 失锁；记录 GM 原生 alarm、holdover start、phase drift、uncertainty growth 和退出条件 |
| TIME-025 | 双 GM 切换 | 验证主备故障、BMCA/优先级、GM identity 变化、phase step、MID-360/PHC 的重新锁定与 generation flush |
| TIME-026 | GM reboot/power-cycle | 验证 holdover 是否跨 reboot、启动时间、初始错误、UTC offset/epoch 恢复和设备端时间是否 backward/forward step |
| TIME-027 | F9P PPS 与 epoch label 配对 | 同时采集 PPS、UBX-TIM-TP、NAV-PVT/TIMEUTC/TIMELS、GPRMC；验证每个脉冲对应唯一 GPS week+iTOW/UTC label，丢失/乱序/重启 fail-closed |
| TIME-028 | PPS fanout 通道偏差 | 用同一高精度时间采集器同时测量每个输出端；记录通道 skew、绝对延迟、负载、线缆长度/温度和重复性 |
| TIME-029 | PPS 输入电气与脉冲完整性 | 对 MID-360、fanout、目标接收输入逐一核对电平/极性/脉宽/阈值；注入开路/短路/边沿抖动，不损坏设备且状态可诊断 |
| TIME-030 | PPS/标签单故障注入 | 分别断 PPS、断标签 UART/UDP、错一秒标签、重复/乱序标签、F9P reboot；状态转换符合 LOCKED→HOLDOVER/LOST/INVALID，禁止静默续用 |
| TIME-031 | A/B 同时观测比较 | GM PTP、F9P PPS/label 同时进入独立时间采集器；测量相位差、漂移、温度/失锁响应；不能先验假设二者独立或互为真值 |
| TIME-032 | 机器人全链路误差预算验收 | 对 MID-360、Jetson PHC/system、左右相机 exposure/strobe、本体时间接口分别测量偏差/抖动/漂移；按 LIO/stereo/VIO 反推阈值，形成批准的 per-sensor budget |

所有测试均为待执行合同；无目标设备、校准仪器和原始测量日志前不得标记通过。

## 9. 当前 ADR 状态与下一步

### 决策状态：PROPOSED / NOT DECIDED

**暂定架构优先级：**
1. A 作为网络传感器主时间链优先评估，前提是目标 GM 能以 MID-360 支持的 PTP v2.0 UDP/IP 正确工作，目标 Orin NIC/PHC 实测通过，目标网络不存在不可控不对称。
2. B 作为独立台架/备选时间链评估，尤其用于验证 MID-360 GPS/PPS 输入、相机公共 trigger 和主机外部事件采集；但它需要完整的 PPS-label association、扇出电气设计和逐端口监测。
3. C 只作为 DEGRADED 模式，不进入高精度融合验收。

### 仍需关闭的 Gate
- 目标 GM 型号/采购价格/尺寸/功耗/交期与售后支持：未报价。
- 目标 F9P SKU/固件/天线/输出配置与 PPS 精度/holdover：需要实物/授权，未完成。
- Jetson 目标载板 PHC、PTP servo、交换机拓扑：需要实物，未完成。
- MID-360 PTP v2.0 互通、GPS fallback 行为和失锁/重锁：需要实物，未完成。
- Color H01R1 Trigger/Strobe 电气规格与连续双机曝光同步：需要对应彩色 SKU 手册/供应商书面确认和实物，未完成。
- Lite3 本体时钟/外部同步输入：需要供应商授权/实物，未完成。
- LIO/stereo/VIO 时间误差预算：需根据实际运动模型和算法输入参数推导，未完成。

### 下一轮执行顺序
1. 深入核对 TP4100 与 SecureSync 的具体型号/选件手册、PTP profile、GNSS/oscillator/holdover、PPS/ToD 接口与公开采购约束；避免将系列页指标当作单一 SKU 保证。
2. 补齐 F9P PPS pulse/UBX-TIM-TP/GPRMC 的逐字段/时序合同，并形成 PPS fanout 的电气采购规格，不锁定未经验证的板卡型号。
3. 研究 PTP-aware switch / NIC PHC 的端到端 asymmetry 与故障隔离方案。
4. 结合相机 Color H01R1 手册获取状态，决定 camera trigger 是否与 PPS 同源、是否独立触发。
5. 用 TIME-022..032 构建台架验收计划，测量后才冻结最终 ADR 和采购 BOM。

## 10. 来源与证据

- Livox MID-360 supported synchronization methods：https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html
- Livox MID-360 Ethernet protocol / timestamp types / time_offset：https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
- Livox time synchronization instructions (PTP/GPS priority and mechanism)：https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html
- u-blox ZED-F9P Integration Manual UBX-18010802 R16, §3.13 time pulse / UBX-TIM-TP：https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf
- u-blox ZED-F9P Product Summary (TIMEPULSE range, interfaces and TCXO)：https://content.u-blox.com/sites/default/files/ZED-F9P_ProductSummary_UBX-17005151.pdf
- Microchip TimeProvider 4100 official product page：https://www.microchip.com/en-us/products/clock-and-timing/systems/ptp-grandmaster-clocks/timeprovider-4100
- Microchip TimeProvider 4100 oscillator/holdover application note：https://ww1.microchip.com/downloads/en/DeviceDoc/00003802.pdf
- Safran SecureSync 2400 User Manual Rev. 13.1 (2026), PTP profiles and ports：https://safran-navigation-timing.com/wp-content/uploads/2026/07/SecureSync2400_UserManual_PN_2400-5000-0050_r13.1.pdf
- Safran SecureSync PTP Grandmaster 1204-32 option specifications：https://safran-navigation-timing.com/online-manuals/2400/Content/NC_and_SS/Com/Topics/OPTCARDS/OC_List/PTP_Grandm.htm
- Safran SecureSync holdover behavior and oscillator-dependent drift：https://safran-navigation-timing.com/online-manuals/2400/Content/NC_and_SS/Com/Topics/CONFIG/Holdover.htm
- NVIDIA Jetson AGX Orin PTP/PHC platform evidence and HTE boundary：见 [doc15 §12](15_sensor_clock_domains_and_sync_architecture.md)


## 11. 具体设备规格的二次核验（2026-11-25）

### 11.1 SecureSync 2400：内置 PTP 与 1204-32 不能混为一谈

Safran SecureSync 2400 User Manual Rev.13.1（2026-06-09）明确区分两套 PTP 实现：

**SecureSync 主机内置 PTP（ETH0/ETH1）：**
- 两个 Ethernet PTP 端口，IEEE 1588-2008 PTP v2；
- 可配置为 PTP Master 或 PTP Slave；
- 10/100/1000 Mb/s；
- 文档列出 Default、Telecom G.8265.1、Power Utility 61850-9-3、Power System C37.238-2017、Enterprise、802.1AS-2011 profiles；
- Unicast、Multicast（IPv4/Ethernet）和 Hybrid 模式；
- 手册将其与 1204-32 选件的配置说明明确分开。

**1204-32 独立 PTP Grandmaster 选件：**
- 单个 SFP Ethernet 端口，PTP Ordinary Clock，只能配置为 Master；
- 独立 1PPS BNC 输出；
- IEEE 1588-2008 PTP v2；Default、Telecom、Enterprise profiles；
- 8 ns（±4 ns）packet timestamp resolution；
- 30 ns（3σ）Master-to-Slave accuracy 只注明 crossover cable 测试条件；
- 支持 one-step 或 two-step master 模式；需要核对实际配置。

**设计影响：** 采购比较表必须把“SecureSync 2400 主机内置 PTP”和“SecureSync 2400 + 1204-32”拆成两个 BOM/接口方案。不得把内置双端口 Master/Slave 能力、选件卡单口 Master-only 能力和选件卡 30 ns crossover 指标合并描述。对本项目而言，内置 PTP 方案可能更适合需要 PTP upstream/downstream 分离的实验架构；但这仍需验证具体端口角色、GNSS input/reference、profile、network asymmetry 与 MID-360 互通。1204-32 单口 Master-only 方案不能仅凭 30 ns 指标就判定更优。

### 11.2 TimeProvider 4100：已确认平台能力，未确认项目配置

Microchip TimeProvider 4100 官方产品页与 Release 2.2 application note 提供以下平台级信息：
- IEEE 1588 v2 PTP Grandmaster，具备 PTP、NTP、SyncE；
- 官方产品页列出双机 active/standby 或 active/active 软件冗余选项；
- 内置 GNSS firewall（BlueSky）抗干扰/抗欺骗能力；
- Release 2.2 资料列出 multi-band GNSS、mini OCXO / Super OCXO / Rubidium oscillator 选项；
- 标准平台列出 8 个 Ethernet 端口、2 × 1PPS/ToD 端口、2 × 1PPS/10 MHz 端口；可选 1GE/10GE expansion module；
- 资料列出多种 PTP profile 和多域 boundary clock 能力。

但这些是系列/版本/选件级公开信息，不代表任一可报价 SKU 都包含全部端口、振荡器或 license。ePRTC 30 ns 规格是特定架构等级，不是“任意单机 + 单 GNSS + 本项目以太网网络”的端到端保证。采购询价必须要求厂商给出完整 ordering code、硬件/软件 revision、profile matrix、振荡器、GNSS 模块、PTP 端口/授权、PPS/ToD 接口选件和 holdover test condition。

**对 MID-360 的重要限制：** “IEEE 1588 v2”并不能证明兼容性。必须确认 GM 能发出 MID-360 支持的 IEEE 1588-2008 UDP/IP profile/transport 与 delay mechanism；电信 PTP profile、802.1AS 或 boundary-clock 能力不能自动代替 Default profile 互通验证。

### 11.3 ZED-F9P TIMEPULSE 与 UBX-TIM-TP 的准确时序关系

本项目应以 u-blox ZED-F9P Integration Manual UBX-18010802 R16（2024-10-30）作为当前字段/时序审计依据。其 §3.13 明确：

1. UBX-TIM-TP 提供**下一次 TIMEPULSE 脉冲**的时间信息和时间来源，不是“消息到达时刻”。
2. 官方建议 measurement rate (CFG-RATE-MEAS) 与 TIMEPULSE (CFG-TP-*) 都设为 1 Hz。若 TIMEPULSE 频率高于 1 Hz，消息输出节奏不保证每个脉冲各有一条一一对应的消息。
3. 1PPS 图示中，在边沿 T1 之后发出的 UBX-TIM-TP 指向下一次边沿 T2；同一串口批次中的 NAV-PVT 可对应上一历元 T1。不得将“同一串口批次”误解为 TIM-TP 与 NAV-PVT 指向同一个脉冲。
4. CFG-TP-TIMEGRID_TP* 选择 GPS/GNSS/UTC time grid；CFG-NAVSPG-UTCSTANDARD 选择 UTC 实现。不同 GNSS UTC realization 可有数百纳秒差异，不能只存一个无来源标记的“UTC”字符串。
5. CFG-TP-ALIGN_TO_TOW 使脉冲对齐到秒边界；polarity 决定首边沿上升/下降；user delay 和 antenna cable delay 会影响实际脉冲相位，必须纳入校准。
6. 官方示例在未锁定时仍可维持 1 Hz 输出，但 TOW 对齐只有在 GPS time locked 时才能维持。若配置 locked other setting，可在 locked/unlocked 时切换脉冲行为，甚至在未锁定时禁用 pulse；项目必须明确配置该行为，不得仅凭 GPIO 有边沿就宣称有效。
7. 绝对时间应用需要校准完整用户系统；u-blox 建议固定且准确的位置，并在适用 GNSS 模式和温度下校准。多接收机相对时间应用还需一致产品家族与 cable/user delay 校准。

**推荐的标签绑定键：**
- primary epoch key：timeBase + week + TOW（若协议/固件返回字段足够）；
- validity：保存 TIM-TP flags、UTC availability、time source/reference、receiver time validity、GNSS fix/time validity；
- pulse edge identity：本机单调递增 pulse_sequence，由硬件边沿采集器计数；
- transport metadata：消息接收 monotonic timestamp、serial port、CRC/checksum、receiver boot generation；
- calibration metadata：antenna cable delay、user delay、fanout channel delay、capture channel delay、温度/设备 SKU。

如果固件/协议消息中没有可用的 week/TOW、UTC standard 或 validity 字段，不能由 host receive time 推算补齐；应标记 UNMAPPED/INVALID，再通过对应 Interface Description 和目标固件验证字段是否支持。

### 11.4 TIME-027..030 测试合同加严

TIME-027 必须按下列步骤执行：
1. 固定目标 F9P SKU、FWVER、PROTVER、配置快照；显式设置 CFG-RATE-MEAS=1 Hz、CFG-TP 的 1 Hz、time grid、align-to-TOW、polarity、pulse width、locked/unlocked behavior。
2. 同步采集 TIMEPULSE 电气边沿（硬件 timestamp/逻辑分析仪）、原始 UBX 字节流、UBX-TIM-TP、NAV-PVT、NAV-TIMEUTC、NAV-TIMELS、MON-VER/状态字段。
3. 按手册图示验证每条 TIM-TP 是“next pulse”标签，检查 T1 边沿之后的消息指向 T2；明确 NAV-PVT 与 pulse 的历元关系，不按串口消息邻近性配对。
4. 在冷启动、热启动、GPS locked、GNSS time invalid、天线断开、GNSS interference、receiver reboot、UART 丢字节/缓冲延迟下重复测试。
5. 证明 time grid / UTC standard / validity / pulse polarity / edge alignment；若 1 Hz pulse 仍输出但 TOW 不再锁定，系统必须进入 HOLDOVER/LOST/INVALID，不得维持 LOCKED。
6. 对实际 fanout 每路测量从 F9P pin 到 consumer input 的延迟和 skew；不能仅测 receiver pin。

新增测试条目：
- **TIME-033 — TIM-TP 与 NAV-PVT 的 next-pulse 关联：** 用硬件边沿采集与原始 UBX stream 证明 TIM-TP 标签对应下一 pulse；检查跨秒 off-by-one、week rollover、GNSS/UTC time grid 和消息乱序；失败必须 INVALID。
- **TIME-034 — TIMEPULSE unlocked behavior：** 分别验证持续输出、locked-other-setting、禁用输出三种配置；注入 GNSS invalid/失锁，证明 1 Hz 边沿存在但 TOW 不可信时不会继续发布 LOCKED absolute time。
- **TIME-035 — SecureSync PTP implementation separation：** 对内置 ETH0/ETH1 PTP 与 1204-32 选件分别建立硬件/端口/角色/profile/one-step-two-step/accuracy 证据；禁止跨实现复用指标。
- **TIME-036 — GM full ordering-code procurement audit：** 对 TimeProvider 4100 与 SecureSync 2400 候选，核对完整订货编码、选件卡、软件许可、振荡器、GNSS 模块、PTP profiles、PPS/ToD 端口、holdover 与质保/交期报价；只有书面 SKU 证据才能进入 BOM。

## 12. 本轮研究结论修订

- SecureSync 2400 不能再以一个泛化“PTP GM”条目比较；内置 PTP 与 1204-32 作为两种独立实现分别评估。
- TimeProvider 4100 已取得 Release 2.2 的官方平台/接口/振荡器摘要；具体 SKU、选件和 profile 仍未确认。
- F9P 的 UBX-TIM-TP 是 next-pulse 标签，不能按消息接收时刻或同批 NAV-PVT 简单配对；TIMEPULSE 在失锁后可能仍有脉冲但不再保持 TOW 对齐。
- ADR 仍为 PROPOSED / NOT DECIDED。暂时没有足够证据批准采购，也没有把 A/B 任何一路认定为已验证冗余。

未采购设备；未修改生产代码；未接线；未运行构建、测试、仿真或实机测试。
