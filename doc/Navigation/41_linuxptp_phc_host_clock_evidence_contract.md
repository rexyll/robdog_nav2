# LinuxPTP / PHC → SystemTime evidence contract

日期：2026-11-26  
分支：nav  
状态：上游协议/工具接口审计完成；目标 Orin 拓扑与实机证据未取得；不实现猜测式 producer。

## 1. 目的与边界

Chrony producer 只证明 Chrony 对自身 NTP clock 与 host SystemTime 的 tracking 估计满足配置预算。它不能证明 Jetson NIC 的 PHC 已锁定 PTP grandmaster，也不能证明 PHC→CLOCK_REALTIME 的 phc2sys 链正常。

LinuxPTP 路径必须拆成至少两条独立边：

1. PTP network → ptp4l servo → NIC PHC：检查 GM/port/servo、master offset、path delay、PTP time properties。
2. PHC/PTP time scale → phc2sys → host CLOCK_REALTIME：检查 phc2sys 是否实际配置为 SystemTime sink、当前 offset/servo 统计、UTC/PTP offset 规则及服务存活。

不得把 `pmc TIME_STATUS_NP.master_offset` 当成 host SystemTime−UTC offset。它描述 ptp4l 本地时钟相对 master 的状态；硬件时间戳时本地时钟通常是 PHC。即使 PHC 对 GM 同步，也不能推出 host CLOCK_REALTIME 已被 phc2sys 同步。

## 2. 上游工具接口核验

### 2.1 pmc（只读 GET）

LinuxPTP `pmc` 是 IEEE 1588 management client；支持 GET/SET/CMD。监测器只允许 GET，禁止 SET/CMD。通过 Unix domain socket 查询时可使用 `-u`，socket 默认值受部署配置影响；不得假设固定 `/var/run/ptp4l` 路径或默认 domain。

待目标机确认 socket/domain/interface 后，可执行以下只读命令（示例仅为接口，不是目标机已验证命令）：

```bash
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET TIME_STATUS_NP'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET CURRENT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET PORT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET PARENT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET TIME_PROPERTIES_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET DEFAULT_DATA_SET'
```

关注字段与职责：

| Management dataset | 关键字段 | 能证明什么 | 不能单独证明什么 |
|---|---|---|---|
| TIME_STATUS_NP | master_offset、gmPresent、gmIdentity、ingress_time | ptp4l 对 GM 的状态快照 | PHC→SystemTime 已同步、GM 的 UTC traceability |
| CURRENT_DATA_SET | stepsRemoved、offsetFromMaster、meanPathDelay | 当前时钟数据集与路径测量 | host CLOCK_REALTIME offset |
| PORT_DATA_SET | portState、delayMechanism、log intervals | PTP port 当前状态与 delay mode | 整个时钟链端到端准确度 |
| PARENT_DATA_SET | parentPortIdentity、grandmasterIdentity | parent/GM identity | GM 的 UTC 真实性 |
| TIME_PROPERTIES_DATA_SET | currentUtcOffset、currentUtcOffsetValid、ptpTimescale、timeTraceable、leap flags | PTP 时间尺度及 UTC 属性声明 | 这些声明在上游 GNSS/GM 设备上的真实性 |
| DEFAULT_DATA_SET | clockIdentity、numberPorts、twoStepFlag、clockQuality、domainNumber | 本地 PTP clock 身份与 domain | 当前被选择的 GM（须结合 parent/time status） |

必须校验响应来自目标 ptp4l socket、目标 domain 和目标 port；若多 port、多 domain、多实例，逐一分离，禁止跨端口拼接。管理响应缺失、超时、重复字段、未知枚举或不匹配的 GM/clock identity 均 fail-closed。

### 2.2 ptp4l 与硬件时间戳

LinuxPTP 文档说明 `ptp4l -H` 为硬件时间戳模式，端口需关联到同一 PHC；`-S` 是软件时间戳，不能把两者混为一谈。目标 NIC/interface 到 `/dev/ptpN` 的映射必须由运行主机的 sysfs、ethtool 与驱动证据确认，不能写死 `/dev/ptp0`。

PTP profile、domain、transport、delay mechanism（E2E/P2P）、two-step、transportSpecific、timestamping mode 与安全认证配置必须来自目标机真实配置/状态。不要从 MID-360 数据流 PTP 观察推断 Jetson NIC PTP 已锁定。

### 2.3 phc2sys：PHC → CLOCK_REALTIME 是独立门

LinuxPTP 文档明确：
- `phc2sys` 通常用于将 system clock 同步到由 ptp4l 同步的 PHC。
- 自动模式 `-a` 读取 ptp4l 的 clock graph 并跟随 port state；但默认不包含 CLOCK_REALTIME，必须配置 `-r` 才同步系统时钟。
- `-w` 等待 ptp4l 到达同步状态，并在未显式 `-O` 时按 ptp4l 的 currentUtcOffset 与方向维护 sink/source offset。
- 硬件时间戳下，PHC 通常跟随连续 PTP timescale，而系统时钟跟随 UTC；二者 offset 由 phc2sys 维护。错误的 UTC offset 可造成整秒偏差。

因此不能仅看到 ptp4l port state 为 SLAVE/CLIENT 或 TIME_STATUS_NP 有 GM，就把 HostTimeSyncStatus.state 设为 synchronized。还必须有可审计的 phc2sys source/sink 方向、运行状态、PHC index 与 CLOCK_REALTIME sink 配置、PTP/UTC offset 来源及 freshness，以及 phc2sys 实际 offset/error bound。

`phc_ctl cmp` 可用于人工比较 PHC 与 CLOCK_REALTIME，但不得在自动 monitor 中调用任何会修改 PHC 的 `set`/`adj`/`freq`/`phase` 命令。自动化只允许经审核的只读采样路径。

## 3. HostTimeSyncStatus 统一语义

最终的 host UTC evidence 必须表示 **CLOCK_REALTIME 相对 UTC** 的 offset 与 uncertainty，而不是 PTP master offset 或 PHC offset。

如 PTP timescale 为 TAI，必须明确 currentUtcOffset 有效位、当前 offset 值、leap flags 与时间尺度；不能假定固定 TAI−UTC 秒数，不能把 TAI stamp 直接作为 UTC stamp。需要同时保留来源链：
- GM identity / traceability 声明；
- ptp4l PHC 对 GM 的 offset/path-delay/error；
- phc2sys PHC↔CLOCK_REALTIME 的 offset/error；
- PTP-to-UTC scale conversion metadata；
- 同一 Linux boot ID、CLOCK_MONOTONIC sample time、sequence。

若任一层无法提供经过定义的误差上界，不能把两层偏差简单相加后宣称完整 UTC error bound。只有所有层的误差预算、采样时刻与不确定度组合规则经过系统 timing/integrity budget 审核后，才能进入统一 policy gate。

## 4. 互斥时钟管理与 Chrony

不得默认 Chrony 与 phc2sys 同时调整 CLOCK_REALTIME。必须先审计 chronyd source 是否来自 NTP server、PPS/refclock、SOCK/SHM，或 PHC 派生输入；再审计 phc2sys 是否以 CLOCK_REALTIME 为 sink。若两者都控制同一 clock，须有明确架构说明，否则视为冲突并 fail-closed。

本项目当前 Chrony producer 只采集 `chronyc -n tracking`；它不会自动探测 PTP 服务，也不证明 PTP/PHC 链。source_id allowlist 不能仅因 CI 通过而自动批准。

## 5. 目标 Orin 取证清单（待用户/实机执行）

必须先收集并保存原始输出、时间、软件版本和主机 boot ID：

```bash
cat /proc/sys/kernel/random/boot_id
uname -a
chronyc -n tracking
chronyc -n sources -v
timedatectl show
ethtool -T <ptp-interface>
readlink -f /sys/class/net/<ptp-interface>/device/ptp
ls -l /dev/ptp*
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET TIME_STATUS_NP'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET CURRENT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET PORT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET PARENT_DATA_SET'
pmc -u -s <verified-ptp4l-uds-path> -d <verified-domain> -t <verified-transport-specific> -b 0 'GET TIME_PROPERTIES_DATA_SET'
systemctl status ptp4l phc2sys chrony --no-pager
journalctl -u ptp4l -u phc2sys -u chrony --since '15 min ago' --no-pager
```

占位符 `<ptp-interface>` 必须替换为目标机真实接口；服务名、socket、domain 也必须先核验。该清单不要求用户现在执行；进入实机阶段前先由项目确认目标 Orin/NIC/服务部署。采集器 `robodog_linuxptp_host_evidence` 同样要求显式传入 interface、domain、transportSpecific 和 UDS socket；不使用 LinuxPTP 的 domain 0 / transportSpecific 0 / 默认 socket 作为项目部署假设。

## 6. 实施 Gate

现有 transport-independent `linuxptp_pmc_parser.py` 包含三层接口：`parse_pmc_response()` 保持原有 dataset dictionary API；`parse_pmc_response_envelope()` 在解析字段之外保留 response clock identity、port number 和 management sequence ID；`parse_pmc_snapshot()` 从同一份组合输出提取六种 GET dataset（TIME_STATUS_NP、CURRENT_DATA_SET、PORT_DATA_SET、PARENT_DATA_SET、TIME_PROPERTIES_DATA_SET、DEFAULT_DATA_SET），并要求每种恰好一条响应、clock identity 一致、sequence ID 不重复；PORT_DATA_SET.portIdentity 与响应头 clock/port identity 一致，DEFAULT_DATA_SET.clockIdentity 与响应头 clock identity 一致；当 TIME_STATUS_NP.gmPresent=true 时，必须与 PARENT_DATA_SET.grandmasterIdentity 一致；PORT_DATA_SET 响应 port number 必须落在 DEFAULT_DATA_SET.numberPorts 声明范围内。新增 `validate_pmc_snapshot_domain(snapshot, expected_domain_number)`，调用方必须显式提供部署期预期 domain（uint8），不隐式假设 domain 0。PARENT/DEFAULT 的 hex fields 按十六进制整数解析，domainNumber/clockIdentity/GM identity 等保留为独立证据字段。Snapshot 不强制四个 response 的 port number 相同（不同 dataset 的管理范围可能不同），也不赋予共同时间戳或 simultaneity 保证；未来 collector 必须记录命令前后 host CLOCK_MONOTONIC 样本，并由单独 policy 判断采样窗口/freshness。解析器严格校验 header/required fields/types/duplicates/unknown fields；其中 `TIME_STATUS_NP.cumulativeScaledRateOffset` 与 `CURRENT_DATA_SET.offsetFromMaster/meanPathDelay` 使用有限 Decimal 保真，`PORT_DATA_SET` 的 signed log intervals、delayMechanism、peerMeanPathDelay 和 version/timeout 按上游 formatter 的整数语义解析，并验证 portIdentity 与响应头一致。上游 LinuxPTP `pmc.c` 将这些字段分别用 `%.9f`、`%d`、`%hhd`、`%hhu`、`PRId64` 等格式输出（https://github.com/richardcochran/linuxptp/blob/master/pmc.c）；不得把 PORT_DATA_SET.peerMeanPathDelay 误解析为浮点秒。当前已增加 `linuxptp_host_evidence.py` 只读主机证据采集器与 `robodog_linuxptp_host_evidence` CLI：要求显式传入 Linux interface、PTP domain、pmc UDS socket 与 transportSpecific 值；不假设 `eth0`、domain 0 或 `/var/run/ptp4l`。它以 argv + `shell=False` 执行 `timedatectl show`、`ethtool -T`、`chronyc -n tracking/sources`，并只在用户明确配置 service unit 时执行 `systemctl status`、`systemctl show`（只读取 Id/LoadState/ActiveState/SubState/ExecStart/FragmentPath/DropInPaths）与 `journalctl`；对 pmc 只发送固定六条 GET，不接受用户自定义管理命令，不使用 `-f` 配置文件，不执行 SET/CMD 或任何时钟调整。经独立证据确认 portIdentity 时，允许可选的本地 `TARGET <portIdentity>` client selector。每条命令保存原始 stdout/stderr、退出状态、CLOCK_MONOTONIC 与 CLOCK_REALTIME 起止采样；报告还保存 boot ID、`/dev/ptp*` 与接口 sysfs PHC 映射。pmc 输出尝试六数据集解析与 expected-domain 对比，但采集结果始终标记 `host_time_sync_claim=not_evaluated`。采集器当前限定 UDS transport；如果目标实际采用 UDP/L2、多个 PTP port 或多个 ptp4l 实例，必须先扩展经审查的配置/响应选择合同，不得假设单一 response 一定唯一。`--transport-specific` 当前按 PTP header 的 4-bit nibble 限定 0..15；LinuxPTP `pmc(8)` 文档列出 0..255，但上游 `pmc.c` 将该配置值左移 4 位写入 header byte，因此必须保留这项上游文档/实现差异并在锁定的目标版本上复核，不允许把 16..255 默默接受为有效 profile 值。报告新增 `collection_status`（`complete`/`partial`）与 `collection_issues`：只有 boot ID 可读、所有显式请求的 host/PTP 命令均以 exit code 0 完成、且六数据集 pmc snapshot 解析与 expected-domain 校验有效时才标记 `complete`；任一命令 unavailable/timeout/OS error/non-zero、boot ID 缺失或 pmc snapshot 未运行/无效均标记 `partial`，并列出稳定的 issue code。此状态只表达证据采集完整性，不代表 PTP 已锁定、PHC 已同步或 host UTC 可信。当前采集器测试为 mock subprocess 合成测试，不是目标 Orin 证据；仍不推断 PHC/UTC，不发布 HostTimeSyncStatus。`systemctl show` 的 `ExecStart` 对 phc2sys 审计尤其重要：必须从真实 service argv 确认 source/sink、`-r`、`-a`、`-w`、`-O` 等配置，而不是从服务名或 active state 推断它正在同步 CLOCK_REALTIME。证据 JSON 可能包含本机路径、命令行参数和日志；分享前应人工审阅并移除密钥、token 或其他敏感值。目标机原始证据到手后，先审计 socket/domain/port/profile/服务方向，再决定是否开发 producer；禁止任何 clock adjustment side effect。

### 6.1 多端口目标选择（TARGET）

LinuxPTP `pmc(8)` 文档说明默认 management command 会发往所有 port，`TARGET` 可将后续请求限定到一个 clock/port。上游 `pmc_common.c` 的 help 明确语法为 `TARGET [portIdentity]` 或 `TARGET *`；`pmc_do_command()` 将 TARGET 交给本地 `parse_target()`，后者只更新 pmc client 的目标 `PortIdentity`，不是发往远端 clock 的 SET/CMD 管理动作。上游 `util.c:str2pid()` 规定 portIdentity 格式为 `xxxxxx.xxxx.xxxxxx-portNumber`，portNumber 解析为 uint16。

当前 collector 增加可选 `--target-port-identity`，只接受已验证的 clockIdentity-portNumber（port 1..65535），并在六条 GET 之前加入单个 `TARGET <portIdentity>` argv。未提供时仍保持默认 all-ports 查询；若返回多个同类 response，parser fail-closed 并保留原始输出。若显式配置 TARGET，采集器还会把返回的 `PORT_DATA_SET.portIdentity` 与目标值做大小写不敏感的精确比较；不匹配时 snapshot 标记 invalid。禁止自动选择第一个端口，也不使用 `TARGET *` 伪装为单端口选择。只有目标 interface/ptp4l 实例的 portIdentity 已由配置或设备证据确认时，才允许传入该参数；不因 CI fixture 中的 identity 推断目标机 identity。

上游源码：
- https://github.com/richardcochran/linuxptp/blob/master/pmc_common.c （TARGET help、local target dispatch）
- https://github.com/richardcochran/linuxptp/blob/master/util.c （str2pid exact portIdentity grammar）

## 7. 上游参考

- LinuxPTP `ptp4l(8)`: https://www.linuxptp.org/documentation/ptp4l/
- LinuxPTP `phc2sys(8)`: https://www.linuxptp.org/documentation/phc2sys/
- LinuxPTP `pmc(8)`: https://www.linuxptp.org/documentation/pmc/ (GET/SET/CMD、socket/domain/transportSpecific 参数与默认值)
- LinuxPTP `pmc(8)` Debian manpage: https://manpages.debian.org/unstable/linuxptp/pmc.8.en.html
- LinuxPTP `pmc.c` response field formatting: https://github.com/richardcochran/linuxptp/blob/master/pmc.c

本文件是上游接口审计与待实机证据合同，不构成目标 Orin 已安装 LinuxPTP、NIC 支持 PHC、PTP GM 已锁定或 UTC 准确度验收。

## 8. 采集窗口、主机时钟连续性与快照原子性

主机证据采集报告同时记录整体采集窗口，以及每条子命令的
`CLOCK_MONOTONIC` / `CLOCK_REALTIME` 起止读数。采集结束时再次读取
`/proc/sys/kernel/random/boot_id`，并与开始时的 boot ID 比较；开始或结束
boot ID 不可读、或两者不同，均使 `collection_status=partial`。boot ID
变化表示采集期间主机身份连续性无法成立，不应拼接为同一次启动周期的证据。

`collection_clock_consistency` 暴露以下原始量：

- monotonic elapsed：整体窗口的单调时钟耗时；
- realtime elapsed：整体窗口的墙钟耗时；
- realtime-minus-monotonic elapsed：两者耗时差。

当前状态固定为 `measured_no_threshold`，不设未经 timing budget 论证的
step/slew 阈值，也不自动分类为 step 或 slew。该差值会受到采样调用开销影响；
系统时钟 slew、step 或其他墙钟调整都可能影响观测。它不是当前
CLOCK_REALTIME−UTC offset，不是 PHC offset，也不构成 PTP lock 或 host UTC
同步结论。采集完整性、采集期间时钟连续性诊断与时间同步质量必须分别解释。

六类 `pmc GET` 在一个 pmc 进程中按顺序请求并返回。每条 response 保留
自身 management sequence / clock / port identity，但标准输出没有为每个
dataset 提供可供本采集器验证的共同 sample timestamp。因此报告将
`pmc_snapshot_observation.atomicity` 标记为 `not_guaranteed`，
`simultaneity` 标记为 `not_proven`，并只用该 pmc 命令自身的
monotonic 起止时间界定整体观测窗口。不得把六类 dataset 描述为同一时刻的
原子快照；也不得仅凭 response sequence 连续就推断采样同时或间隔有界。
若未来需要逐 dataset freshness / GM 切换时刻审计，必须引入可验证的逐响应
时间戳或独立采样方案，再定义明确的窗口和一致性策略。

## 9. PTP identity 的采集窗口边界比较

collector 在主机命令采集前执行一次完整的 GET-only 六数据集身份基线，
并在主机命令采集后保留原有六数据集快照。两次观测均独立解析、校验显式
domain；配置了 TARGET 时也分别校验返回的 PORT_DATA_SET.portIdentity。

`pmc_identity_boundary_comparison` 对比以下字段：

- local clockIdentity；
- PORT_DATA_SET.portIdentity 与 portState；
- DEFAULT_DATA_SET.domainNumber；
- TIME_STATUS_NP.gmPresent 与 gmIdentity；
- PARENT_DATA_SET.grandmasterIdentity。

如果前后两次观测均有效但上述任一字段不同，报告标记
`ptp_identity_changed_between_collection_boundaries`，并将
`collection_status` 置为 `partial`。前置身份基线不可用或无效，同样
不能把采集报告标为 complete。该比较仅描述边界观测，不自动判定 PTP
失锁、主时钟质量下降或 host UTC 不可信。

**边界相同不代表窗口内部没有变化。** GM 可能在两次观测之间切换后又切回，
portState 也可能短暂抖动；当前 pmc 输出没有可验证的逐响应采样时刻，因此
无法从两端一致推导窗口内连续稳定。ptp4l/phc2sys/chrony 的 systemd
properties、ExecStart 和 journal 仍是各自命令执行时的单次只读观察，不是
事务性配置快照，也没有自动前后 diff。未来若需证明窗口内无瞬态切换，必须
使用有时间戳的事件流或连续采样，并定义采样周期、丢样检测与时钟域；不得
把边界比较包装成连续监控保证。



## 10. ptp4l journald 事件证据与覆盖性边界

采集器对显式配置的服务单元改用 journalctl -u <unit> --since <window>
--no-pager --output=json --all，保留原始 stdout/stderr。由于新增结构化事件证据字段，报告 schema_version
提升为 2，并额外生成 journal_event_evidence。--output=json 让每条日志成为独立 JSON 对象；
--all 请求完整字段值，避免 journalctl 默认将超过 4096 字节的字段输出为
null。解析器保留 _BOOT_ID、__MONOTONIC_TIMESTAMP、
__REALTIME_TIMESTAMP、__CURSOR、_SYSTEMD_UNIT 和 MESSAGE。

时间戳解释严格区分：
- __MONOTONIC_TIMESTAMP 是 journald 收到该 entry 时的
  CLOCK_MONOTONIC 微秒值；必须与 _BOOT_ID 配对。仅当它与本次 collector
  的 boot ID 相同，才可将微秒乘以 1000，与 collector 的
  CLOCK_MONOTONIC 纳秒窗口比较。
- __REALTIME_TIMESTAMP 是 CLOCK_REALTIME 微秒 epoch 值；保留用于原始
  关联，不将它当成 monotonic 时间，也不把它与采集器 monotonic 绝对值相减
  来制造精准映射。
- boot ID 不匹配、缺少/非法时间戳、无效 JSON、日志顺序 monotonic 回退或
  没有任何有效 entry 均使该 service 的 journal evidence 为 partial；
  collector 顶层 collection_status 同步降为 partial。

### 10.1 当前已核实的 LinuxPTP message patterns

基于 LinuxPTP 上游 port.c / clock.c 源码中的格式字符串，解析器只识别
下列明确 pattern，并始终保留原始 MESSAGE：

| 事件类型 | 上游 message 格式 | 解释 |
|---|---|---|
| gm_candidate_observed | new foreign master <portIdentity> | 收到一个 foreign-master 候选，不代表它已被 BMC 选为 GM |
| gm_selected | selected best master clock <clockIdentity> | clock.c 报告选中的 best master |
| local_clock_selected_as_best_master | selected local clock <clockIdentity> as best master | 本地 clock 被选为 best master |
| port_state_transition | <port>: <state> to <state> on <event> | port.c port_show_transition() 输出的状态迁移 |
| master_announce_transmit_timeout | master tx announce timeout | ptp4l master 的 announce 发送定时器；不是 announce 接收超时，也不能据此判定 GM 丢失 |

port.c 的 port_show_transition() 将状态和触发 event 写入同一条 notice，
因此可记录 from_state、to_state 与 trigger；例如 trigger 为
ANNOUNCE_RECEIPT_TIMEOUT 时，是一条有明确来源的 port 状态迁移记录。
解析器不会把 master tx announce timeout 错归类为接收端 announce timeout。

本轮审计的 clock.c servo SERVO_JUMP 路径调用 clockadj_step() 执行时钟
step，但该调用点没有相应的专用 ptp4l journal message。故当前事件解析器
**不声称能从 ptp4l journal 完整识别 clock step**，也不将日志中没有
clock step 字样解释为没有发生 step。若未来要覆盖此类事件，需基于目标
LinuxPTP 版本和内核时钟调整证据另行定义事件源、字段与时钟域，不能仅添加
未经源码核验的宽泛正则。

上游源码格式依据：
- port.c：new foreign master、port_show_transition()、master tx announce timeout：
  https://github.com/richardcochran/linuxptp/blob/master/port.c
- clock.c：selected best master clock、selected local clock ... as best master、
  SERVO_JUMP / clockadj_step()：
  https://github.com/richardcochran/linuxptp/blob/master/clock.c

### 10.2 查询结果不等于连续日志覆盖

journal_event_evidence.coverage_status 当前固定为 unknown。即使查询成功、
JSON 全部可解析、返回 0 个匹配事件，也只能说“返回的 entry 中没有识别到
匹配事件”，不能说“采集窗口内没有 GM 切换/portState 变化/clock step”。

单次 journalctl 成功退出不能证明：
- 查询范围内所有历史 journal files 都仍保留；
- 日志未被 vacuum、rotation、磁盘空间策略或 rate limit 丢弃；
- --since 所覆盖的时间段没有缺口；
- ptp4l 进程在整个窗口持续运行并持续输出；
- 日志中未出现的状态变化没有发生。

当前 collector 没有把 journalctl --verify 或 --list-boots 当作窗口连续性
证明；这些命令即使成功，也不能单独证明目标窗口没有被删除或漏记。解析器
只检查本次返回 entries 的结构、boot identity 与 monotonic 顺序，不推断
journal retention 完整性。事件列表的 collection_window_relation 只将
entry 的 monotonic 时间与 collector 整体 monotonic 起止值比较，不代表
journal 查询窗口被完整覆盖。

因此，coverage_status=unknown 不因 event_count=0 改变；若需证明事件流
连续，需要独立的 journald retention/sequence/cursor gap 策略、丢样/限流
证据以及持续采集机制，并定义可验证的窗口覆盖合同。现阶段 journal 证据
只能用于发现已保留且已输出的事件，不能用作“无事件”证明。

systemd 字段语义依据：
- journalctl --output=json、--all 和 JSON 字段编码说明：
  https://www.freedesktop.org/software/systemd/man/latest/journalctl
- _BOOT_ID、__MONOTONIC_TIMESTAMP、__REALTIME_TIMESTAMP、__CURSOR 字段说明：
  https://www.freedesktop.org/software/systemd/man/latest/systemd.journal-fields



## 11. journald 保留/丢失上下文与时钟调整事件源审计（schema v3）

schema v3 将 journal query 的时间语义与 collector 命令执行时间分开：
- 每个 service journal query 保留 since_argument、显式 until_argument=null
  （即没有传入 --until）、journalctl 调用的 realtime/monotonic 起止值，
  并标记 human_since_expression_normalized=false。当前默认 --since 15 min ago
  是交给 journalctl 解释的相对时间表达式；collector 不把它伪装成精确的
  CLOCK_MONOTONIC 窗口。
- 每条 entry 的 journal_command_execution_relation 只表示该 entry 的
  __MONOTONIC_TIMESTAMP 是否落在执行该条 journalctl 命令的窄时间区间内。
  它不是 --since 查询窗口内/外判断，更不是 collector 全部任务的窗口。
- journal_storage_context 另行保留只读诊断命令原始输出：合并后的
  systemd/journald.conf（systemd-analyze cat-config）、journalctl
  --disk-usage、journalctl --list-boots，以及每个显式目标 service 的 systemctl show logging-specific properties（LogNamespace、
  LogRateLimitIntervalUSec、LogRateLimitBurst、LogLevelMax、LogFilterPatterns、
  StandardOutput/StandardError 和 unit path/drop-in metadata；不读取完整 unit 文件，
  避免为此诊断额外采集 Environment 等可能包含敏感值的配置）。
  若 LogNamespace 非空，当前 service journal query 仍未自动附加 --namespace，
  因此该 service 的日志命名空间尚未被完整覆盖；必须把它标为 namespace evidence
  gap，后续按该 namespace 查询 systemd-journald@NAMESPACE.service。这些诊断命令为可选上下文；不可用会令该 context 标为
  partial，但不单独使核心 PTP inventory 失败。若没有显式配置任何目标 service，
  不执行这些 journald 诊断命令，journal_storage_context.status=not_requested。
  systemd-analyze cat-config 只提供配置文件/ drop-in 合并视图，不等同于读取
  journald 当前运行时内部状态，也不能自动补齐编译时默认值；journalctl --disk-usage
  是当前占用统计，不是历史查询窗口内每个 journal file 的保留证明。

### 11.1 丢失与限流通知

systemd message catalog 为以下事件定义稳定 MESSAGE_ID：
- a596d6fe7bfa4994828e72309e95d61e：service 日志因 rate limit 被抑制；
  解析器额外提取 Suppressed N messages from <unit> 中的计数和受影响 unit。
- e9bf28d037fff184e6ae669a370e694725：journal 报告 kernel messages missed；
  解析器保留 N_DROPPED（若该字段存在）。

采集器单独查询 systemd-journald.service 的 suppression records，避免只查询
ptp4l/phc2sys 时漏掉 journald 自身产生的限流通知。出现上述记录是明确的
丢失/抑制证据；没有出现并不证明没有丢失，因为通知自身也可能不在查询范围、
已轮转/清理或被其他机制丢弃。全局配置、service unit drop-in、disk usage、
boot inventory 和 suppression events 都只是诊断上下文，coverage_status
仍固定为 unknown。Storage=persistent 也不保证无限保留；容量、保留期限、
可用磁盘、轮转和 service 级 LogRateLimitIntervalSec= /
LogRateLimitBurst= 都会影响可用日志。systemd 还说明 service 级限流设置
覆盖 journald 全局默认值。

### 11.2 CLOCK_REALTIME / PHC step 的独立证据路线

LinuxPTP phc2sys.c 的 update_clock() 在 SERVO_JUMP 时调用
clockadj_step(clock->clkid, -offset)；clockadj.c 以
ADJ_SETOFFSET | ADJ_NANO 调用 clock_adjtime()。clkid 可以代表
CLOCK_REALTIME，也可以是通过 /dev/ptpN 文件描述符构造的动态 PHC clock ID。
clockadj_step() 成功路径本身不输出一条专用“step succeeded”日志；因此
不能仅从 phc2sys journal 的普通 servo 状态行或没有 failed-to-step 日志，
反推是否发生 step 或目标 clock 是什么。

后续可靠事件捕获候选：
1. 在目标内核实际提供的 syscall tracepoint 上，成对记录
   clock_adjtime / adjtimex / clock_settime / settimeofday 的 entry 和
   exit；采集 syscall 时间戳、PID/TGID、comm、clockid、返回值和 boot identity。
2. 对 clock_adjtime，若要区分 ADJ_SETOFFSET、ADJ_OFFSET、
   ADJ_FREQUENCY，还必须在 syscall entry 时安全读取并记录用户态 struct timex
   的 modes 及相关字段；仅记录 syscall 参数中的用户指针不够。
3. 对动态 PHC clock ID，必须在事件时刻把 clockid 关联回实际 fd 和
   /dev/ptpN（并处理 fd close/reuse）；不可只根据数值猜 PHC index。
4. 记录 syscall exit 的 errno/返回值，以区分请求尝试、内核接受和失败。
   记录调用者身份、可执行文件、内核版本、tracepoint format/BTF 与丢事件统计。
5. 对 PHC adjtime / adjfine 与 CLOCK_REALTIME 的修改分别归档；不要把
   PHC step、system wall-clock step、frequency slew 和 phase correction 混为一类。

Linux audit syscall 规则可以作为调用者与 syscall 参数的辅助审计，但它不能
直接解引用 struct timex * 并完整记录 modes / time 内容，因此单靠 audit
记录不足以区分 ADJ_SETOFFSET 与频率/相位调整。eBPF syscall entry/exit
探针可作为进一步验证的候选，但需针对目标 Orin kernel 配置、BTF/tracepoint
可用性、用户内存读取失败、ring-buffer/perf-buffer 丢事件和 PHC fd 映射做
明确的完整性合同。当前 v0.3.249 尚未启用任何 tracing、audit rule 或
eBPF 程序；不改变主机状态，只记录证据设计和后续验收条件。

### 11.3 上游依据

- systemd journald.conf：Storage、size/retention 与全局 RateLimit 配置：
  https://www.freedesktop.org/software/systemd/man/latest/journald.conf.html
- systemd exec：service 级 LogRateLimitIntervalSec /
  LogRateLimitBurst 覆盖规则：
  https://freedesktop.org/software/systemd/man/latest/systemd.exec.html
- systemd D-Bus Service properties（LogNamespace、LogRateLimitIntervalUSec、LogRateLimitBurst 等）：
  https://www.freedesktop.org/software/systemd/man/latest/org.freedesktop.systemd1.html
- systemd catalog：rate-limit suppression 与 missed kernel message 的 MESSAGE_ID：
  https://github.com/systemd/systemd/blob/main/catalog/systemd.catalog.in
- LinuxPTP phc2sys.c 与 clockadj.c：
  https://github.com/richardcochran/linuxptp/blob/master/phc2sys.c
  https://github.com/richardcochran/linuxptp/blob/master/clockadj.c
- Linux kernel event tracing：可用 tracepoints 由目标 kernel 的
  available_events / event format 文件决定，不能假定每个内核都有同一事件集合：
  https://www.kernel.org/doc/html/latest/trace/events.html



## 12. 目标内核 tracing 能力只读盘点（v0.3.250）

为决定后续是否能在 Orin 上可靠捕获 CLOCK_REALTIME / PHC 调整事件，新增
`linux_kernel_trace_capabilities.py` 只读盘点器，并由 LinuxPTP host evidence
collector 输出 `clock_step_trace_capabilities`。盘点器仅读取现有 procfs/sysfs
文件，不运行 mount、trace-cmd、perf、bpftool 或 BPF loader，也不写 tracefs
控制文件。

### 12.1 采集范围

- 记录运行内核 release、machine、system，以及盘点自身的
  CLOCK_MONOTONIC / CLOCK_REALTIME 起止时间。
- 读取 `/proc/config.gz` 与 `/boot/config-<uname -r>`（若存在），仅摘录
  FTRACE、FTRACE_SYSCALLS、TRACEPOINTS、HAVE_SYSCALL_TRACEPOINTS、BPF、
  BPF_SYSCALL、BPF_EVENTS、DEBUG_INFO_BTF、IKCONFIG、IKCONFIG_PROC、PERF_EVENTS、KPROBE_EVENTS 等
  相关配置。配置文件缺失、权限不足和读取错误分别保留状态。
- 读取 `/proc/self/mountinfo`，识别
  `/sys/kernel/tracing` 与 `/sys/kernel/debug/tracing` 的挂载信息。
- 识别 tracefs 或 debugfs 的父挂载关系；读取每个候选 tracefs 的 `available_events`，仅针对实际列出的
  `syscalls:sys_enter/exit_clock_adjtime`、`adjtimex`、
  `clock_settime`、`settimeofday` 读取各自 `format` 文件；报告实际字段
  名和原始 format，不把不同内核/架构的字段布局写死。
- 对 `/sys/kernel/btf/vmlinux` 只检查存在性、可读性和文件大小，不加载或
  解析 BTF。collector 另执行只读 `systemctl --version` 并保留原始输出。

### 12.2 状态与证据边界

- `available` 仅表示事件名出现在该内核的 `available_events` 且其
  `format` 文件可读；`not_listed` 表示在成功读取的列表中未出现；
  `unknown` 表示列表本身不可读。不得将三者混为“内核不支持”。
- `/proc/config.gz` 是构建内核时使用的配置快照，仅当
  `CONFIG_IKCONFIG_PROC` 启用时才会提供；`/boot/config-<release>`
  可能缺失或与当前运行内核不匹配。配置内容不等同于运行时 tracepoint
  可用性，实际事件仍以目标内核 `available_events` 与 `format` 为准。
- BTF 文件存在/可读不证明 BTF 内容有效、BPF verifier 会接受目标程序、
  当前进程具有加载权限或用户态内存读取一定成功。
- tracefs 未挂载、权限不足或事件未列出时，不尝试自动挂载、不提升权限，
  不写入 enable/set_event/filter/trigger；也不据此排除 kprobe、perf 或其他
  厂商专用 instrumentation。
- 此次盘点没有开始 tracing，因此没有 event buffer、lost-event counter、
  syscall entry/exit 配对、timex 用户内存读取、PID/fd 生命周期或 PHC
  fd→`/dev/ptpN` 解析证据。它只为下一阶段方案选型提供能力边界。
- `systemctl --version` 是可选诊断命令；其缺失/失败只令该项 unavailable，不单独降低核心 PTP inventory 的 collection_status。它只记录 systemd 版本信息，不证明 journald
  runtime namespace、日志保留或 service 配置正确；具体 service 的
  `LogNamespace` 等仍按显式配置 service 的既有采集路径获取。
- 本轮 GitHub Actions 合成测试不是 Orin 实机验证。目标 Orin 的内核版本、
  config、BTF、tracefs、syscall event format、systemd 版本及访问权限仍需
  用户提供原始报告/现场执行结果，状态为“需要实物/授权，未完成”。

### 12.3 上游依据

- Linux kernel Event Tracing：`available_events` 与 event `format` 是目标
  内核实际事件/字段的发现依据：
  https://www.kernel.org/doc/html/latest/trace/events.html
- Linux man-pages `proc_config.gz(5)`：`/proc/config.gz` 提供构建当前内核
  所用配置，并依赖 `CONFIG_IKCONFIG_PROC`：
  https://man7.org/linux/man-pages/man5/proc_config.gz.5.html
- Linux kernel `init/Kconfig`：`IKCONFIG` 与 `IKCONFIG_PROC` 配置定义：
  https://github.com/torvalds/linux/blob/master/init/Kconfig
- Linux kernel BTF 文档：BTF 是 BPF 类型/调试信息格式，不能以文件存在
  代替 BPF 程序加载/验证：
  https://docs.kernel.org/bpf/btf.html
- libbpf overview：运行内核 BTF 暴露路径
  `/sys/kernel/btf/vmlinux`：
  https://docs.kernel.org/next/bpf/libbpf/libbpf_overview.html
