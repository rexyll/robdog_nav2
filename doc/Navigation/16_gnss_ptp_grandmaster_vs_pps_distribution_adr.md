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


## 13. PTP 网络交换设备与 NIC/端点兼容性审计（2026-11-25）

### 13.1 交换机不能只按“支持 PTP”筛选

PTP 交换设备至少区分：
- **普通二层转发（Forward）：** 不修正驻留时间；网络负载、队列拥塞和链路不对称直接进入端到端误差。
- **E2E Transparent Clock（TC）：** 在 PTP event message 中修正设备驻留时间；端点仍使用 end-to-end delay request 机制。
- **P2P Transparent Clock：** 逐链路测量 peer delay；需要 GM、交换机和端点的 delay mechanism/profile 相容。
- **Boundary Clock（BC）：** 交换机自身作为下游从时钟，再向下游作为主时钟重新分发；每一跳有本地 servo/clock 误差和额外收敛行为。
  
因此“交换机支持 IEEE 1588-2008”不等于所有 clock mode、profile、transport、delay mechanism、one-step/two-step 和 VLAN 组合都可用。

### 13.2 Cisco Catalyst IE3x00 / IE3400 / IE3500 / IE9300：公开工业交换机候选

Cisco 2026-06-26 PTP 配置指南列出这些平台的 PTP 行为和限制：
- 仅支持 multicast PTP messages；
- BC 模式只支持 two-step clock operation，不支持 one-step；TC 模式会把收到的 one-step Sync 转换为 two-step；
- switch 与 GM 必须配置相同 PTP domain；
- Default Profile 模式只支持 delay_request mechanism；Power Profile 模式只支持 peer_delay；
- Power Profile 还要求 Announce TLV（Organization_extension、Alternate_timescale），否则可能丢弃 Announce；
- QinQ 不支持；Boundary 模式只处理配置的 PTP VLAN；TC 模式的 VLAN/端口约束需按目标拓扑确认。
  
Cisco 公开冗余/网络指南给出的示例指标：
- IE9300 Rugged：Default/Power 2011/2017，BC 约 200 ns/hop，TC 约 50 ns/hop；
- IE3500 Rugged：Default/Power 2011/2017，BC 约 200 ns/hop，TC 小于 200 ns/hop；
- 这些是平台/模式/拓扑条件下的 per-hop 参考，不是整网绝对误差保证；Cisco 还明确指出特定 GLC-T 铜 SFP 可能降低 PTP 精度。
  
**本项目适配风险：**
- 已有 Livox 官方资料明确：MID-360 的 IEEE 1588v2.0 UDP/IP PTP 路径采用 two-step Sync/Follow_Up，并使用 E2E Delay_Req/Delay_Resp；设备作为 PTP Slave。因此 Cisco Default Profile 的 delay_request 机制可作为候选，Power Profile 的 peer_delay 机制不能直接配套这一 PTP 路径。
- 仍未证实的是 PTP 报文本身的 multicast/unicast 寻址、PTP domain/profile、PTP UDP 端口、VLAN tag/PCP、TTL，以及 Announce/Sync/Delay interval。Cisco multicast-only 限制只有在抓包确认 MID-360 PTP 实际采用 multicast 后，才能判定兼容。
- Livox 的点云与 IMU 数据流端口默认 UDP Unicast，且可配置 Multicast；这只是传感器数据流的传输合同，不能推断 PTP 同步报文采用同样的单播/组播方式。
- 若实际 PTP 报文为 UDP/IP multicast，且 domain/profile/VLAN/interval 与目标配置匹配，可进入 Cisco Default Profile + E2E TC/BC 验证；若设备发送 unicast 或存在 Cisco 不支持的组合，则该 Cisco 配置不兼容。Power Profile / 802.1AS 不应为了交换机功能而替代已确认的 PTP v2.0 端点合同。

### 13.3 Microchip LAN9668 / EVB-LAN9668：开发台架候选，不是现成采购定案

Microchip 官方将 LAN9668 标为量产中的 8-port TSN Gigabit Ethernet switch。EVB-LAN9668 用户指南说明：
- PTP interfacing 经所有网络端口可用，时间戳由 LAN8814 PHY 提供；
- 板载 DPLL、TCXO（可替换 OCXO）与 1PPS SMA 输入/输出；
- LAN9668 可配置 Boundary Clock、Transparent Clock、PTP Master 或 PTP Slave；
- EVB-LAN9668 是评估板，8 × 1G copper ports，带外部 DPLL 和系统同步连接器。
  
它适合作为实验室比较 TC/BC、PHC/硬件 timestamp、PPS 相位和链路延迟的开发平台候选；不能直接视为机器人量产网络交换机，也不能将评估板的外部 DPLL/板级性能外推到任意 LAN9668 商用板卡。正式使用需要确定 switch firmware/SDK、PTP profile/delay mechanism、PHY 型号、板卡时钟树、驱动、管理能力、EMC/供电/温度和工业连接器。

### 13.4 端到端拓扑建议（待验证）

**最小验证拓扑：**
    GNSS-disciplined GM
       │ dedicated isolated timing LAN (VLAN tagging only after endpoint compatibility is verified)
       ▼
    candidate PTP switch (Forward vs E2E TC vs BC)
       ├── MID-360 PTP endpoint
       └── Jetson AGX Orin NIC with PHC

并行观测：
- 独立 PPS reference + calibrated time interval counter；
- GM 1PPS output 与 switch/Jetson 可采集硬件事件；
- Jetson NIC PHC 与 CLOCK_REALTIME cross timestamp；
- MID-360 time_type / time_offset / last_sync_time；
- 同步采集 PTP packet capture；当前 PTP v2.0 预期消息序列为 Sync/Follow_Up 与 E2E Delay_Req/Delay_Resp。Announce、Pdelay 或其它消息只有在目标固件/配置的抓包中实际观察到时才记录，不预设设备支持。

**网络工程约束：**
1. PTP VLAN 与相机 USB、控制器、普通 DDS 流量隔离；若 MID-360 PTP 与 LiDAR data 共用物理口/链路，必须测量高负载下队列拥塞与 timestamp jitter。
2. 不把普通交换机的 QoS 优先级当作 TC/BC；QoS 只能减少拥塞风险，不能补偿 residence time 或路径 asymmetry。
3. 尽量避免 GM 到 MID-360 的路径上存在未审计的交换机、媒体转换器、非对称光电模块和不同速率链路。
4. PTP 网络主备路径切换必须验证路径不对称变化与设备重锁；链路冗余不自动等于时间冗余。
5. Jetson 上分别验证每个 NIC 的 hardware TX/RX timestamp、PHC index、驱动、link speed、EEE、IRQ/CPU affinity、PTP servo；不能将某个 Orin DevKit 的 ethX/PHC0 结论外推到其它 carrier/NIC。
6. 对每条 PTP path 记录 GM identity、domain、profile、transport、delay mechanism、one/two-step、VLAN、link rate、switch model/firmware/port、SFP SKU、cable type/length、temperature 与 traffic load。

### 13.5 新增测试合同 TIME-037..040

- **TIME-037 — MID-360 PTP packet addressing/profile capture：** 在已知 IEEE 1588v2.0 UDP/IP、two-step、E2E Delay_Req/Delay_Resp 的前提下，通过 packet capture 与设备状态确认 PTP 报文实际 multicast/unicast、源/目的地址与端口、domain/profile、Announce/Sync/Delay interval、VLAN tag/PCP/TTL；逐项对照 GM/交换机，禁止用点云/IMU 的 UDP 地址模式代替 PTP 报文证据。
- **TIME-038 — Switch forwarding/TC/BC 对比：** 同一 GM、同一端点、同一线缆，分别使用普通转发、E2E TC、BC；在空载/高负载/链路速率变化时测端到端 offset、jitter、residence correction、收敛时间；明确每 hop 误差。
- **TIME-039 — VLAN、SFP 与路径不对称：** 对铜/光 SFP、链路长度、VLAN tag、交换机端口/速率、单向路径差、REP/冗余路径切换做对照；所有路径变化均记录 GM/servo/generation，不能用平均 offset 掩盖方向性误差。
- **TIME-040 — Jetson PHC 与 PTP path qualification：** 对实际载板每个 NIC 记录 ethtool -T、驱动/固件、PHC index、ptp4l 状态、phc2sys 方向、offset/path delay、cross timestamp、IRQ/CPU 负载、EEE 与 link flap；硬件 timestamp 不存在或不稳定则该接口不通过 PTP 高精度 Gate。

## 14. PTP 网络器件初步结论

- **不批准任何交换机采购。** 已确认 MID-360 的 PTP v2.0 是 UDP/IP、two-step、E2E Delay_Req/Delay_Resp；Cisco Default Profile + E2E 可以进入兼容验证，但前提是实测确认 PTP 报文的 multicast 寻址及其它 profile/domain/VLAN 参数。Cisco Power Profile 的 peer_delay 不匹配当前 MID-360 PTP 路径。
- **LAN9668/EVB-LAN9668** 可作为可编程/可观测的实验台架候选，用于 TC/BC 与硬件 timestamp 研究；不是现成量产交换机定案。
- Livox 要求单一 master clock；任何冗余均应先采用单活主时钟与受控切换方案，并在设备端验证切换瞬态、失锁、自动 fallback 与重锁。不可部署两个互相竞争的 active GM。
- 下一步围绕实际 MID-360 packet capture、Jetson 目标载板 PHC 运行状态与 HMS 告警建立同步健康门控；不能反过来用交换机功能定义传感器同步协议。

未采购设备；未修改生产代码；未接线；未运行构建、测试、仿真或实机测试。

## 15. MID-360 PTP 报文合同、设备诊断与 Jetson PHC 实际可用性审计（2026-11-25）

### 15.1 MID-360 PTP v2.0 已确认的报文/时钟角色

Livox 官方 MID-360 时间同步说明与通信协议共同确认：

1. **PTP v2.0 路径：** IEEE 1588v2.0 over UDP/IP；MID-360 是 PTP Slave，网络中由一个 PTP Master 提供参考时间。
2. **同步报文：** 采用 two-step Sync/Follow_Up；设备使用 E2E Delay_Req/Delay_Resp 测量路径延迟，并通过四时间戳 t1/t2/t3/t4 计算 offset/delay。不能把 P2P Peer_Delay / Pdelay 当作该路径的默认机制。
3. **版本边界：** MID-360 不支持 IEEE 1588v2.1。PTP v2.0 与 gPTP 同时存在时，官方明确不建议在同一应用场景混用。
4. **主时钟边界：** Livox 官方说明多 master clock 环境不受支持。工程上应维护单一活动 GM；冗余必须通过受控 active/standby 切换实现，并且需在 MID-360 端验证切换时的 step、失锁、fallback 和重锁行为。
5. **gPTP 是另一条独立模式：** 基于 IEEE 802.1AS/Ethernet Layer 2，Livox 文档描述了不同于 PTP v2.0 的消息/BMCA行为。设备状态接口无法区分 PTP 与 gPTP；不可只凭设备 time_type=1 判定当前网络跑的是哪一种协议。
6. **时间源优先级：** PTP 优先于 GPS；二者同时可用时优先使用 PTP。公开说明并未完整保证断开 PTP 后自动切换到 GPS 的具体时刻、相位连续性或恢复策略，必须故障注入验证。

**当前仍未从公开资料锁定的 PTP 网络字段：** PTP event/general 报文的源/目的 IP、multicast 或 unicast、UDP 319/320 实际使用情况、domain number、profile、VLAN tag/PCP、TTL、Announce/Sync/Delay interval、GM identity 和设备固件/配置依赖。以上不得从点云/IMU 数据流推断，也不得仅根据交换机能力猜定。

### 15.2 传感器数据流与 PTP 控制流必须分开核验

MID-360 通信协议将点云、IMU、命令和日志列为不同 UDP 数据端口。点云与 IMU 数据流默认使用 UDP Unicast，且数据流可以配置 Multicast；控制/日志接口也有各自的传输约定。这些端口说明的是 Livox 数据/控制数据面，并不声明 IEEE 1588 PTP event/general message 的地址、端口或 multicast 行为。

因此，Cisco Catalyst IE 系列的 multicast-only 限制不能通过“点云默认 Unicast”直接判为不兼容，也不能通过“点云可 Multicast”直接判为兼容。必须在交换机与 MID-360 之间抓取真正的 PTPv2 报文，并将 packet capture 与设备配置/状态、GM 侧报文关联。

### 15.3 设备端同步状态与 HMS 故障诊断

MID-360 提供的诊断字段可组成可观察性链：

| 字段 | 已知语义 | 项目使用约束 |
|---|---|---|
| **time_sync_type** | 0=未同步；1=IEEE 1588v2.0 PTP；2=GPS | 作为所选同步机制状态，不单独证明精度/锁定健康 |
| **last_sync_time** | 最近同步时间；PTP 路径下对应 Follow_Up 携带的时间信息，GPS 路径下对应 PPS 上升沿 | 关联时间源、报文序号和设备启动 generation |
| **time_offset** | t0 - t1，t0 为设备本地时刻，t1 为同步源时刻，单位 ns | 正值表示设备本地钟领先于 master；必须与采集时刻及来源状态一起解释 |
| **local_time_now** | 设备本地当前时间 | 仅作设备时钟观测，不能当作 UTC/GPST |
| **powerup_cnt** | 上电计数 | 用于识别设备重启/时钟 generation 变化 |
| **lidar_diag_status** bit 12–15 | 通信模块（含时间同步）诊断位 | 用作粗粒度健康信号，需配合 HMS 明细 |

MID-360 HMS 公开诊断码中与时间同步直接相关的项目包括：
- **0x0402**：PTP 时间同步停止或时间间隔过大；
- **0x0403**：PTP 版本错误，设备不支持 IEEE 1588v2.1；
- **0x0405**：时间同步异常；
- **0x0406**：时间同步精度偏低；
- **0x040A**：同时检测到 PTP 与 gPTP 信号，应只使用一种。
PPS/GPS 诊断还包括 **0x0404**、**0x0407**、**0x0408**、**0x0409**。采集时必须保留 HMS 原始码、severity/level、时间和设备 generation，不能仅把码值汇总成一个布尔量。

**建议项目健康状态映射（设计，不是已实现行为）：**
- **LOCKED**：时间源类型符合配置；持续观察到有效 two-step/E2E PTP 交互；time_offset 与独立参考误差预算相符；没有 0x0402/0403/0405/0406/040A 等活动告警。
- **DEGRADED**：offset 超出正常阈值但在暂定降级范围内，或出现可恢复的低精度/链路告警；停止宣称高精度同步，并增加 uncertainty。
- **LOST/INVALID**：PTP 交换停止、版本不匹配、PTP 与 gPTP 并存、设备重启后 generation 未重新建立、时间源类型与配置不一致；禁止把本地时间当作全局绝对时间继续输入定位 deskew。
- **HOLDOVER**：只有实际测得设备/GM 在失去网络/参考后保持的时间漂移及有效期，才允许定义；不能把“仍在输出点云”或“时间戳单调递增”当成 holdover 证据。

健康判定需联合 PTP packet capture、time_sync_type、last_sync_time、time_offset、powerup_cnt、HMS 与点云 timestamp；timestamp_type=1 或存在时间戳只说明数据采用某种时间基准，不代表满足本项目精度门限。

### 15.4 Jetson AGX Orin PHC：区分芯片能力、载板能力与可运行性

NVIDIA Jetson AGX Orin 平台资料说明 MGBE/RGMII 接口受模块、UPHY lane 分配、PHY 与载板实现影响；P3737 Developer Kit carrier 是官方验证的一种实现。采用第三方载板时必须核验 device tree、PHY driver/firmware、端口映射与时钟能力，不能将 P3737 的结论外推。

公开开发者论坛报告分别呈现了不同套件/接口环境：
- 一份 AGX Orin 32GB Developer Kit 记录显示网卡 ethtool -T 暴露 hardware TX/RX timestamp 与 PHC 0；这是特定板卡/驱动的社区实测，不是对所有 Orin 网口的保证。
- 一份 AGX Orin 64GB Developer Kit 报告也展示了 PHC 能力字段，但其 ptp4l TX timestamp polling 出现 timeout，测试者报告部分硬件时间戳结果异常。该贴是单一环境的故障报告，不能推导为 64GB 平台普遍缺陷；它说明 ethtool -T 声明能力不等于 PTP 实际可运行。

LinuxPTP 的 ptp4l 支持 OC/BC/TC；LinuxPTP 文档给出的配置语义中，E2E 是默认 delay mechanism，UDPv4 是常用传输，twoStepFlag=1 表示 two-step，硬件 timestamping 需要 NIC/driver/PHC 真正可用。结合 MID-360 的已确认协议，**UDPv4 + E2E + two-step + hardware timestamp** 可作为 Orin 端 PTP v2.0 的实验起始配置，但不是已经验证的最终配置。仍需确认 domain、GM identity/priority、PTP 地址、VLAN、announce/sync interval 和 ptp4l port role。

目标平台的逐接口验收必须记录：
1. 模块/载板型号与 revision、JetPack/L4T、Linux kernel、PHY 型号与固件、实际网口名和 driver；
2. ethtool -T 全量能力、/dev/ptpN、PHC index 与 NIC 接口映射；
3. 实际 Sync/Follow_Up/Delay_Req/Delay_Resp 硬件 TX/RX timestamp 是否持续成功，不能只有 ethtool 声明；
4. ptp4l 的 port state、GM identity、domain、offset、mean path delay、servo state 与 log；分别在空载/点云满负载/ROS DDS 高负载下观察；
5. phc2sys 的方向与 source/destination，建立 PHC→CLOCK_REALTIME 映射；测量 cross timestamp 和 host system clock offset，禁止把 PHC 与系统时钟当同一时钟；
6. link flap、switch reboot、Jetson reboot、GM outage、PHC reset、主从切换后的 generation、重锁和时间 step。

NVIDIA 官方平台资料证明的是平台接口与载板支持边界；论坛帖子只提供特定实测案例。当前未在本项目实际 64GB 目标开发套件、最终网口或未来定制载板上运行 ethtool / ptp4l / phc2sys，因此目标接口状态仍为 **需要实物/授权，未完成**。Orin 应作为强制 client-only PTP Ordinary Clock，而不是 Livox Wiki 附录中作为示例 master 的 PC；系统时钟映射应使用后续 §16 中的 phc2sys 自动模式，避免硬编码 UTC offset。

### 15.5 新增验证合同 TIME-041..046

- **TIME-041 — MID-360 PTP transport/address capture：** 抓取真实 PTP event/general packets，确认源/目的 IP、UDP 端口、multicast/unicast、domain/profile、VLAN/PCP/TTL、消息间隔与 GM identity；将抓包与 firmware/config/设备状态一起归档。
- **TIME-042 — E2E two-step message conformance：** 在 GM、交换机和 MID-360 三端验证 Sync/Follow_Up 与 Delay_Req/Delay_Resp 时序、t1/t2/t3/t4、时间戳完整性和 offset；拒绝把 Pdelay/P2P 作为默认合同，除非正式设备资料/实测证明目标固件采用另一模式。
- **TIME-043 — MID-360 HMS/time-state health gate：** 联合解析 time_sync_type、last_sync_time、time_offset、powerup_cnt、lidar_diag_status 和 HMS 0x0402/0403/0405/0406/040A；注入 PTP 停止、版本不匹配、双协议并存、低精度和设备重启，验证 LOCKED→DEGRADED/LOST/INVALID 的门控。
- **TIME-044 — PTP/GPS priority and failure transition：** 同时接入合规 PTP 与 GPS/PPS，验证 PTP 优先级、PTP master 消失后的实际 fallback、恢复时 phase step、generation 更新和重锁；确认单活动 master，不预设无缝切换。
- **TIME-045 — Orin NIC PHC operational qualification：** 对目标 P3701/P3737 64GB Developer Kit 或正式载板的每个目标接口核验 ethtool -T、driver/PHY、PHC device/index，并证明 PTP event messages 的硬件 TX/RX timestamps 和 ptp4l slave servo 连续工作。
- **TIME-046 — PHC↔system clock and network stress：** 验证 phc2sys 方向、PHC→CLOCK_REALTIME cross timestamp、offset/drift、点云/DDS 高负载、IRQ/CPU、EEE、链路 flap、交换机/GM 重启和重新锁定；为每个 generation 记录误差分布与失效窗口。

### 15.6 来源与证据边界

1. Livox-SDK 官方 Wiki，MID-360 Ethernet Communication Protocol：<https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md>。
2. Livox-SDK 官方 Wiki，Time Synchronization：<https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/common/time_sync.rst>。
3. Livox-SDK 官方 Wiki，MID-360 HMS Error Codes：<https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/hms_code_mid360.rst>。
4. LinuxPTP ptp4l documentation 与 default configuration：<https://www.linuxptp.org/documentation/ptp4l/>；<https://www.linuxptp.org/documentation/default/>。
5. NVIDIA Jetson AGX Orin series platform bring-up：<https://docs.nvidia.com/jetson/archives/r39.2.2/DeveloperGuide/HR/JetsonModuleAdaptationAndBringUp/JetsonAgxOrinSeries.html>。
6. NVIDIA Developer Forum, AGX Orin 32GB hardware timestamp report：<https://forums.developer.nvidia.com/t/does-the-jetson-agx-orin-32gb-support-hardware-timestamping-for-ptp/337720>；AGX Orin 64GB ptp4l TX timestamp timeout report：<https://forums.developer.nvidia.com/t/jetson-agx-orin-64gb-ptp4l-service-timed-out-while-polling-for-tx-timestamp/316653>。

本轮 Web 访问日期为 2026-11-25。Livox Wiki 源链接当前使用 master，本次未建立不可变 commit pin，后续必须保存源码 commit SHA/文件快照或供应商协议版本号。本节是上游规格与公开案例审计，不是目标板或网络实测；无设备抓包、无目标 Jetson 命令输出、无代码实现、无构建/测试/仿真/实机验证。


## 16. LinuxPTP / Jetson 客户端配置、时钟域映射与可执行验证流程（2026-11-25）

### 16.1 PTP 角色分离：外部唯一 GM，Orin 强制 client-only

本项目方案 A 中的角色固定为：
- **外部 GNSS-disciplined GM：** 提供可追溯的时间源与唯一活动 PTP Grandmaster；其 GNSS reference、oscillator、holdover、PTP profile、domain 与输出能力必须由具体设备 SKU 和实测确认。
- **Jetson AGX Orin：** NIC PHC 作为 PTP Ordinary Clock；运行 `ptp4l` 的 `clientOnly=1`，只接收外部 GM，不参与争抢 GM 角色。即使外部 GM 暂时消失，也不能自动把 Orin 本地系统时间升格为网络主时钟。
- **MID-360：** Livox 自身 PTP Slave；与 Orin 分别跟踪同一个 GM。Orin 不通过 ROS 或 DDS 再向 MID-360“转发时间”，除非未来明确设计并验证另一个 BC/GM 拓扑。
- **交换机：** 初始台架先用单一物理交换路径，之后才比较普通 Forward、E2E TC 或 BC。不要在验证阶段同时引入多个活动 GM、PTP/gPTP 混跑或未审计的冗余路径。

Livox 官方时间同步说明仍包含一段 Linux PC `ptp4l` master 的示例；这是用于说明如何搭建/检查 PTP 网络，不是本项目 Orin 的生产角色。该上游文件还推荐过 linuxptp v3.1.1，但当前 JetPack/L4T 仓库中的包版本尚未采集，不能据此直接覆盖系统包或强行编译旧版本。目标机必须记录 `ptp4l -v`、`phc2sys -v` 和发行版包版本。

### 16.2 Orin PTP Slave 的配置基线

根据 LinuxPTP 官方手册与默认配置，UDPv4、E2E、硬件 timestamp 是默认配置；`clientOnly` 默认值为 0，因此必须显式打开。建议在抓包确认目标 GM 的 domain 后，建立专用文件，例如 `jetson-ptp-slave.cfg`：

```ini
[global]
clientOnly 1
network_transport UDPv4
delay_mechanism E2E
time_stamping hardware

# Domain must match the external GM.
# LinuxPTP defaults to 0; set domainNumber only after packet capture
# or the GM configuration confirms the actual value.
```

说明：
1. `clientOnly 1` 是关键防护；不要配置 `serverOnly 1`，也不要让 Orin 成为 BMCA 中可被选为 GM 的时钟。
2. `network_transport UDPv4`、`delay_mechanism E2E` 与 MID-360 当前已确认的 PTP v2.0 UDP/IP + E2E 合同一致；不要用 `delay_mechanism Auto`，因为 LinuxPTP 的 Auto 可在收到 peer-delay request 后切换到 P2P。
3. `time_stamping hardware` 要求目标 NIC/driver/PHC 真正支持硬件 TX/RX timestamp；若 PHC 不存在或 TX timestamp polling timeout，必须判定该接口未通过高精度 PTP Gate，不能静默退化为 software timestamp。
4. Domain number 不从示例或交换机默认配置推断。LinuxPTP 默认 domain 为 0，但若 GM 的 Announce/管理数据和配置确认的是其它值，必须使用实际值。
5. `twoStepFlag` 是本机作为 PTP Server/GM 发出 Sync 时的模式配置。Orin 已被强制为 client-only，不应把该字段理解为“要求远端 GM 发 two-step”；远端 Sync 的 two-stepFlag 与 Follow_Up 必须通过抓包确认。
6. 本配置没有擅自设定 PTP profile、priority、UTC offset、Announce/Sync interval、delay asymmetry 或 latency correction。只有当 GM/交换机配置和链路参考面已取得书面/实测证据后，才填入非默认值。

运行前先确认接口与 PHC：

```bash
IFACE=enP2p1s0   # Replace with the actual target NIC interface.
ip -brief link
ip -details link show dev "$IFACE"
sudo ethtool -i "$IFACE"
sudo ethtool -T "$IFACE"
ptp4l -v
phc2sys -v
ls -l /dev/ptp* 2>&1 || true
systemctl is-active chrony systemd-timesyncd 2>/dev/null || true
chronyc tracking 2>/dev/null || true
```

注意：`IFACE` 只是占位；不得将 `enP2p1s0` 写成项目固定网口。必须把 NIC 的 driver/firmware、PHY、实际 `/dev/ptpN`、载板 revision、JetPack/L4T、kernel 和 ethtool 完整能力一并归档。若 `chronyd` 或 `systemd-timesyncd` 正在独立调整 `CLOCK_REALTIME`，必须先明确唯一时钟纪律路径；不能让它们与直接调整系统时钟的 `phc2sys` 无协调地竞争。若决定由 chrony 统一纪律，应采用 LinuxPTP 提供的 SOCK/SHM reference-clock 集成方式，而不是两个 daemon 同时直接调系统时钟。

### 16.3 可复现的启动与运行命令

先在独立终端抓包，再启动 `ptp4l`。对 UDP PTPv2 和 IEEE 802.1AS/L2 分别捕获，避免把点云/IMU UDP 数据流误作 PTP：

```bash
sudo tcpdump -ni "$IFACE" -nn -s 0 -w ptp-udp.pcap \
  '(udp port 319 or udp port 320)'

sudo tcpdump -ni "$IFACE" -nn -s 0 -w ptp-l2.pcap \
  'ether proto 0x88f7'
```

如交换机已启用 VLAN，还需记录 VLAN tag/PCP 并确保抓包过滤器覆盖 VLAN encapsulation；抓包中没有看到报文时，不能立即宣称网络无 PTP，先确认 NIC VLAN offload、mirror/SPAN、过滤器和采集位置。

启动 Orin 为 client-only 的 PTPv2 UDPv4 E2E hardware-timestamp client：

```bash
sudo ptp4l -i "$IFACE" -f ./jetson-ptp-slave.cfg \
  -H -4 -E -s -m
```

其中 `-s` 打开 client-only，`-H` 选择硬件 timestamp，`-4` 选择 UDPv4，`-E` 选择 E2E；配置文件中的 `clientOnly 1` 再提供持久化保护。不要使用 `-A` 自动 E2E/P2P 切换，也不要用 `-2` 切换为 Layer-2 transport 来“匹配” gPTP；MID-360 的 PTP v2.0 与 gPTP 是两种独立模式。

当本地 `ptp4l` 服务已经运行后，使用 `pmc` 查询状态（将 DOMAIN 替换为实际 domain）：

```bash
PTP_DOMAIN=0   # Replace only after verifying the GM domain.
sudo pmc -u -d "$PTP_DOMAIN" "GET DEFAULT_DATA_SET"
sudo pmc -u -d "$PTP_DOMAIN" "GET PARENT_DATA_SET"
sudo pmc -u -d "$PTP_DOMAIN" "GET PORT_DATA_SET"
sudo pmc -u -d "$PTP_DOMAIN" "GET CURRENT_DATA_SET"
sudo pmc -u -d "$PTP_DOMAIN" "GET TIME_PROPERTIES_DATA_SET"
sudo pmc -u -d "$PTP_DOMAIN" "GET TIME_STATUS_NP"
sudo pmc -u -d "$PTP_DOMAIN" "GET PORT_HWCLOCK_NP"
```

采集结果至少包括 port state、parent/GM identity、domain、stepsRemoved、offsetFromMaster、meanPathDelay、time/frequency traceability、timeProperties 与 PHC association。某些 `*_NP` 管理项为 LinuxPTP 扩展；若当前发行版不支持，应保存返回结果并记录版本，不把“不支持查询项”误判为同步失败。

当 Orin 的 PTP port state 和 GM parent 数据符合预期后，再启动 PHC→系统时钟映射：

```bash
sudo phc2sys -a -r -m
```

LinuxPTP 的 `-a` 从正在运行的 `ptp4l` 读取时钟与 port state，并随状态变化自动选择同步方向；单个 `-r` 将 `CLOCK_REALTIME` 纳入同步目标。不要使用 `-rr`，因为双 `-r` 会允许系统时钟作为时间源；对于本项目 client-only Orin，不需要这个反向角色。也不要手工添加 `-O 0` 或硬编码 `-O 37`，应由 `ptp4l` 的 PTP time properties 与 `phc2sys` 自动获得正确 offset。给 `ptp4l` 和 `phc2sys` 使用不同配置文件/参数来源，避免共用文件时非预期的参数继承。

### 16.4 PTP/PHC/UTC/ROS 时钟域映射约束

LinuxPTP 文档明确区分 PTP timescale 与 UTC：硬件 timestamping 的 PHC 跟随 PTP timescale；主机 `CLOCK_REALTIME` 通常表达 UTC；`phc2sys` 根据 PTP time properties 维护两者偏移。因此：
- NIC PHC、`CLOCK_REALTIME`、`CLOCK_MONOTONIC` 和 ROS Time 不能视作同一时钟。
- `phc2sys` 只建立 PHC↔系统时间映射，不会自动证明 ROS use_sim_time、传感器 header.stamp 或 MCU 时间等于 UTC。
- PTP time scale 与 UTC offset 必须作为显式属性存储；不要把 PTP timestamp 直接标注为 UTC/GPS，亦不要把 PTP 的连续时间尺度当作自动遵守历史闰秒/未来 continuous UTC 政策的 UTC realization。
- `CLOCK_MONOTONIC` 仍作为本机超时、缓存龄期与执行时长的基准，禁止通过 NTP/PTP 的 wall-clock step 来回调 monotonic freshness 计时。
- 任何 GM identity/domain、PTP time scale/UTC offset、PHC device/index、NIC、系统时间 step、设备重启或时间源切换都建立新的 `mapping_generation`；清空跨 generation 的 LiDAR deskew、GNSS epoch join、camera trigger pairing 和融合缓冲区。
- 启动时优先让 ptp4l/phc2sys 达到受控同步状态，再启动依赖绝对时间的 ROS 传感器/定位节点。若在导航运行期间发生 clock step、GM identity 变化或 servo 失锁，则立即撤销高精度时间健康状态，并按上游状态机进入 DEGRADED/LOST/INVALID；在 generation 更新和重新收敛前不得跨界拼接历史数据。

LinuxPTP 默认配置的 first-step threshold 为 20 μs；是否发生 startup step、何时校正 PHC 和系统时钟必须以实际启动日志及外部参考测量确认。不要未经风险评估就把 threshold 改为 0：禁用 step 可能增加初次收敛时间。项目验收需分别记录首次启动 step、运行中频率调整、GM 中断后的重锁 step，并证明 ROS 导航运行期间没有未管理的 wall-clock discontinuity。

### 16.5 TIME-041..046 的测量步骤与结果字段

| 测试 | 执行步骤 | 必须保存的证据 | 失败判据 |
|---|---|---|---|
| TIME-041 PTP address/profile | 双端同步抓包，解析 IPv4/IPv6、319/320 或 L2、multicast/unicast、domain、VLAN/PCP、TTL、GM identity、消息 interval | pcap、Wireshark decode、GM 配置、交换机配置、MID-360 firmware/status | 无法确认实际 PTP flow；GM 与 endpoint transport/domain 不匹配 |
| TIME-042 two-step/E2E | 逐个验证 Sync/Follow_Up、Delay_Req/Delay_Resp、t1..t4 时间戳关联；确认无 Pdelay 取代 E2E | GM/Orin/MID-360 三方抓包、ptp4l state/log、offset/path-delay 时序 | 缺失 Follow_Up/Delay_Resp、错误 delay mechanism、持续 TX timestamp timeout 或非预期单步/双步 |
| TIME-043 MID-360 HMS gate | 对 PTP stop/version mismatch/low precision/PTP+gPTP 并存和 powerup_cnt 变化分别采集设备状态 | 原始 HMS uint32、abnormal ID/level、time_sync_type、last_sync_time、time_offset、powerup_cnt、timestamp_type | 告警存在却仍报告 LOCKED；恢复后没有 generation 变化/确认 |
| TIME-044 PTP/GPS transition | 分别运行 PTP only、GPS/PPS only、两者同时、PTP outage/recovery；不同时启用两个 master | GM/source identity、HMS、PTP pcap、PPS capture、phase offset、generation 日志 | 源优先级/切换未按设备观测；无记录 phase step；双 GM 同时 active |
| TIME-045 Orin PHC qualification | 对目标 NIC 逐口跑 ethtool/driver/PHC 枚举，运行 ptp4l clientOnly，观察 hardware TX/RX timestamp 与 port state | 板卡/模块/JetPack/kernel/driver/PHY、ethtool -T、/dev/ptpN、ptp4l/pmc 日志 | 无硬件 timestamp/PHC；端口无法进入正确 client state；持续 timeout/Faulty；Orin 意外发出 GM Announce |
| TIME-046 PHC-system stress | 运行 phc2sys -a -r；记录 cross timestamp/offset/drift，并叠加点云、DDS、高 CPU、链路 flap、交换机/GM reboot | phc2sys log、system clock offset、PHC data、CPU/IRQ/EEE、链路与重启事件、mapping_generation | 未协调的时钟 step、竞争式 system-clock discipline、失锁后仍 LOCKED、跨 generation 缓冲拼接 |

所有测试的数值通过/失败阈值必须由时间误差预算和各消费模块（LIO deskew、stereo/VIO、GNSS fusion、ROS TF）允许的数据龄期反推。本轮不擅自填入纳秒/微秒门限。每次 run 使用唯一 run_id，并保存启动参数、配置文件 SHA-256、软件版本、原始 pcap、日志、温度/负载、设备序列号（不公开对外）和结论。

### 16.6 新增验证合同 TIME-047..052

- **TIME-047 — Orin client-only role fail-safe：** 无 GM、GM 被拔线、GM reboot 和另一 GM 出现时确认 Orin 始终不发出 Announce/Sync 作为 GM；port state 与 parent GM 变化必须可观测。
- **TIME-048 — LinuxPTP config/packet conformance：** 在同一实际 domain/profile/transport 下，验证 clientOnly、UDPv4、E2E、hardware timestamp 与 Livox two-step/E2E 协议相容；显式证明配置文件中的 twoStepFlag 不是远端 two-step 证据。
- **TIME-049 — PHC servo and management readback：** 使用 ptp4l/pmc 获取 Default/Parent/Port/Current Data Set、Time Status、PHC association；将日志中的 port state、GM identity、offset 与 pcap 时间对齐。
- **TIME-050 — PHC↔UTC discipline ownership：** 验证 phc2sys -a -r 的方向与 PTP time properties 转换；审计 chrony/systemd-timesyncd，确保系统时钟只有一个直接 servo owner，或采用经过验证的 SOCK/SHM reference integration；禁止硬编码 UTC offset。
- **TIME-051 — startup/step/re-lock generation：** 测量 ptp4l 与 phc2sys 初次 step、运行中调整、GM outage/recovery 的系统 wall-clock discontinuity；确保导航栈启动顺序正确，并在 step/GM identity/domain/PHC reset 后更新 mapping_generation、flush all temporal queues。
- **TIME-052 — evidence reproducibility/version pin：** 每次目标板 run 保存 JetPack/L4T、kernel、linuxptp、ethtool、NIC/PHY firmware、完整 config、SHA-256、pcap 与原始日志；Livox upstream source fixed at commit and blob SHA，禁止只记录 mutable master URL。

### 16.7 上游文件版本固定与参考来源

本轮从 Livox-SDK 官方仓库读取并在 commit `6b030b73a7fa1e035e44bf1b262d7a9cbebe0728`（2026-09-21，add Mid 360L product information）回读确认以下文件 blob SHA 与当前 master 内容一致：
- `source/tutorials/new_product/common/time_sync.rst` — blob `bbf34d60df4c2fbb83bca989b08dbfc50d6006ea`：IEEE 1588v2.0 two-step/E2E 交互与 master/slave 模型；同一文档单独描述 gPTP L2/AUTOSAR 变体；推荐单一 master；记录旧版 LinuxPTP 3.1.1 示例，应视为历史参考，不直接覆盖当前 Orin distro 包。
- `source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md` — blob `d797ea8512ccdac6a6c1868b7b538f6c79dc4ec3`：MID-360 通信协议发布版本 v1.4.12（2026-09-21）。
- `source/tutorials/new_product/mid360/hms_code_mid360.rst` — blob `0bc60df1bd8ba094cad2ebc256cc70c8120a1adf`：HMS 码组成、severity level 和 MID-360 时间同步告警表。
  
LinuxPTP 在线手册为滚动文档，目标 JetPack 上实际 linuxptp 版本必须用 `ptp4l -v` / `phc2sys -v` 与 dpkg 记录；本节只固定当前可访问手册的语义，不把滚动文档当成目标机安装版本：
1. `ptp4l(8)`：<https://www.linuxptp.org/documentation/ptp4l/>。
2. `phc2sys(8)`：<https://www.linuxptp.org/documentation/phc2sys/>。
3. `pmc(8)`：<https://www.linuxptp.org/documentation/pmc/>。
4. default configuration：<https://www.linuxptp.org/documentation/configs/default-cfg/>。
5. default parameters：<https://www.linuxptp.org/documentation/default/>。

本节为目标机配置与验证 runbook，不代表命令已在 AGX Orin 执行。实际 interface、PTP domain/profile、GM、PHC index、UTC offset、step/holdover 行为和精度阈值仍需实物/授权验证；无本项目设备操作、无生产代码、无构建/测试/仿真/实机结果。
