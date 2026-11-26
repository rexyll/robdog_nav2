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

