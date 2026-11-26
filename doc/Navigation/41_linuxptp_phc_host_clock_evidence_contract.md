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
pmc -u -b 0 'GET TIME_STATUS_NP'
pmc -u -b 0 'GET CURRENT_DATA_SET'
pmc -u -b 0 'GET PORT_DATA_SET'
pmc -u -b 0 'GET PARENT_DATA_SET'
pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
pmc -u -b 0 'GET DEFAULT_DATA_SET'
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
pmc -u -b 0 'GET TIME_STATUS_NP'
pmc -u -b 0 'GET CURRENT_DATA_SET'
pmc -u -b 0 'GET PORT_DATA_SET'
pmc -u -b 0 'GET PARENT_DATA_SET'
pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
systemctl status ptp4l phc2sys chrony --no-pager
journalctl -u ptp4l -u phc2sys -u chrony --since '15 min ago' --no-pager
```

占位符 `<ptp-interface>` 必须替换为目标机真实接口；服务名、socket、domain 也必须先核验。该清单不要求用户现在执行；进入实机阶段前先由项目确认目标 Orin/NIC/服务部署。

## 6. 实施 Gate

现有 transport-independent `linuxptp_pmc_parser.py` 包含三层接口：`parse_pmc_response()` 保持原有 dataset dictionary API；`parse_pmc_response_envelope()` 在解析字段之外保留 response clock identity、port number 和 management sequence ID；`parse_pmc_snapshot()` 从同一份组合输出提取六种 GET dataset（TIME_STATUS_NP、CURRENT_DATA_SET、PORT_DATA_SET、PARENT_DATA_SET、TIME_PROPERTIES_DATA_SET、DEFAULT_DATA_SET），并要求每种恰好一条响应、clock identity 一致、sequence ID 不重复；PORT_DATA_SET.portIdentity 与响应头 clock/port identity 一致，DEFAULT_DATA_SET.clockIdentity 与响应头 clock identity 一致；当 TIME_STATUS_NP.gmPresent=true 时，必须与 PARENT_DATA_SET.grandmasterIdentity 一致；PORT_DATA_SET 响应 port number 必须落在 DEFAULT_DATA_SET.numberPorts 声明范围内。新增 `validate_pmc_snapshot_domain(snapshot, expected_domain_number)`，调用方必须显式提供部署期预期 domain（uint8），不隐式假设 domain 0。PARENT/DEFAULT 的 hex fields 按十六进制整数解析，domainNumber/clockIdentity/GM identity 等保留为独立证据字段。Snapshot 不强制四个 response 的 port number 相同（不同 dataset 的管理范围可能不同），也不赋予共同时间戳或 simultaneity 保证；未来 collector 必须记录命令前后 host CLOCK_MONOTONIC 样本，并由单独 policy 判断采样窗口/freshness。解析器严格校验 header/required fields/types/duplicates/unknown fields；其中 `TIME_STATUS_NP.cumulativeScaledRateOffset` 与 `CURRENT_DATA_SET.offsetFromMaster/meanPathDelay` 使用有限 Decimal 保真，`PORT_DATA_SET` 的 signed log intervals、delayMechanism、peerMeanPathDelay 和 version/timeout 按上游 formatter 的整数语义解析，并验证 portIdentity 与响应头一致。上游 LinuxPTP `pmc.c` 将这些字段分别用 `%.9f`、`%d`、`%hhd`、`%hhu`、`PRId64` 等格式输出（https://github.com/richardcochran/linuxptp/blob/master/pmc.c）；不得把 PORT_DATA_SET.peerMeanPathDelay 误解析为浮点秒。它不执行 pmc、不连接 socket、不推断 PHC/UTC、不发布 HostTimeSyncStatus。由于目标机拓扑与版本输出仍未确认，暂不新增 LinuxPTP ROS producer。取得目标 Orin 原始样本后，先核验 socket/domain/port/profile 与真实字段差异，再建立 fail-closed state model 和只读 collector；禁止任何 clock adjustment side effect。

## 7. 上游参考

- LinuxPTP `ptp4l(8)`: https://www.linuxptp.org/documentation/ptp4l/
- LinuxPTP `phc2sys(8)`: https://www.linuxptp.org/documentation/phc2sys/
- LinuxPTP `pmc(8)`: https://manpages.debian.org/unstable/linuxptp/pmc.8.en.html
- LinuxPTP `pmc.c` response field formatting: https://github.com/richardcochran/linuxptp/blob/master/pmc.c

本文件是上游接口审计与待实机证据合同，不构成目标 Orin 已安装 LinuxPTP、NIC 支持 PHC、PTP GM 已锁定或 UTC 准确度验收。
