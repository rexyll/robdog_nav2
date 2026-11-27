# Linux CLOCK_REALTIME / PHC 调整事件采集设计与完整性合同

- 状态：DESIGN_ONLY / NOT_IMPLEMENTED / NOT_HARDWARE_VERIFIED
- 专题阶段：N1 — 端到端数据流与接口审计
- 适用平台候选：Jetson AGX Orin（目标 kernel 尚未取得实机能力报告）
- 关联文档：[41 — LinuxPTP PHC/主机时钟证据合同](41_linuxptp_phc_host_clock_evidence_contract.md)
- 设计日期：2026-11-26

## 1. 目的与非目标

目标是在目标内核实际提供且经 capability inventory 确认的 syscall tracepoint 上，建立 CLOCK_REALTIME / PHC 调整请求的可审计事件链：

1. 记录 syscall entry 与 exit，能够按线程正确配对；
2. 尽可能读取用户态请求结构中的操作模式，区分 step 请求、phase/frequency 调整和只读查询；
3. 记录 syscall 返回值、时钟目标、调用者与 boot-relative 时间；
4. 显式计量内核事件提交失败、用户态消费失败、entry/exit 配对缺失和用户内存读取失败；
5. 在证据不足时输出 unknown/partial，不把“未观察到事件”解释成“没有发生时钟调整”。

本设计不执行 tracing，不加载 eBPF，不启用 tracepoint，不安装 audit rule，不修改 PHC 或 CLOCK_REALTIME，不配置 ptp4l/phc2sys。也不在本阶段宣称已经实现时钟 step 检测或达到零丢失。

## 2. 证据分层与术语

必须分开保存以下层次，禁止将它们合并成一个含义模糊的 clock_step=true：

| 层 | 证据 | 可以支持的结论 | 不可以支持的结论 |
|---|---|---|---|
| E0 能力 | kernel release、available_events、format、BTF、权限 | 目标机存在候选观测入口 | 程序可加载或事件不会丢失 |
| E1 调用尝试 | syscall entry | 某线程发起了某类 syscall | 请求被内核接受或时钟已改变 |
| E2 请求内容 | 分 E2a（sys_enter 时用户内存快照）与 E2b（copy_from_user 后内核副本） | E2a 仅是预调度快照；E2b 是传给内核 handler 的归一化请求 | 不能把 E2a 快照当成内核实际消费内容，也不能把 E2b 当成已生效 |
| E3 syscall 返回观察 | 配对 exit 与原始 return value | handler 返回非负状态或负 errno 的观测 | 单凭负值（尤其 -EFAULT）不能证明副作用没有发生 |
| E4 目标解析 | 明确识别 CLOCK_REALTIME，或经过 fd 生命周期追踪确认 PHC 设备 | 请求目标的 clock domain / PHC 设备 | 仅凭 dynamic clockid 数值推断 /dev/ptpN |
| E5 独立效果 | 独立 clock sample、PHC/系统时钟观测及其误差/时间域说明 | 观测窗口内的时钟变化 | 单凭 syscall 成功断言整个同步链路正确 |

建议最终事件分类采用枚举而非单一布尔值：

- `operation_class=direct_step_request`
- `operation_class=phase_adjust_request`
- `operation_class=frequency_adjust_request`
- `operation_class=read_only_or_no_adjustment`
- `operation_class=clock_set_request`
- `operation_class=unknown_due_to_unreadable_args`
- `operation_class=unsupported_or_unclassified_mode`

另外分别记录 `syscall_outcome=nonnegative_return|negative_return_effect_ambiguous|unpaired|unknown`、`clock_target=realtime|dynamic_clock_unresolved|other|unknown` 与 `effect_observation=not_measured|observed|inconclusive`。不得将 E1/E2/E3 当成 E5。

## 3. 候选 syscall 覆盖矩阵

| syscall | entry 必需参数 | exit 必需参数 | 请求内容要求 | 目标解释 |
|---|---|---|---|---|
| clock_adjtime | clockid、tx 指针 | ret | 尽可能读取 timex.modes；其他字段仅在 ABI/offset 已验证时解码 | CLOCK_REALTIME 或 dynamic clock ID；后者默认 unresolved |
| adjtimex | tx 指针 | ret | 尽可能读取 timex.modes | Linux 系统时钟调整接口；仍需区分只读查询与调整请求 |
| clock_settime | clockid、timespec 指针 | ret | 目标 ABI 验证后读取 timespec | 仅当 clockid 已明确识别时分类目标 |
| settimeofday | timeval 指针及可选 timezone 指针 | ret | 目标 ABI 验证后读取 timeval | 系统 wall clock 请求；timezone 参数不是时钟步进量 |

覆盖边界：
- 不假定四类 syscall 在所有 kernel/architecture 上都有相同 tracepoint 字段。以目标机 `available_events` 和每个 `format` 文件为唯一字段依据。
- 不把 `clock_adjtime` / `adjtimex` 的每次调用都视为调整；它们也可用于查询。
- `clock_adjtime` 的 dynamic clock ID 编码可能来自 PHC fd，但 syscall tracepoint 中的数值不是 /dev/ptpN 身份证明。
- 未纳入的 syscall、内核内部路径、厂商私有 ioctl 或其他 instrumentation 不因本合同而被排除。

## 4. eBPF 事件采集器结构（设计）

### 4.1 事件入口

优先评估目标机已列出的 `syscalls:sys_enter_*` / `syscalls:sys_exit_*` tracepoint。加载前必须将 capability report 中的事件名、format、kernel release、machine、BTF 状态和程序构建 ABI 作为输入门禁。不能从通用 x86_64 fixture 推断 Orin ARM64 可加载。

每条 entry 生成内部关联键 `pid_tgid`（TGID 与 TID 组合），而不是只用 PID；在有界 in-flight map 中保存 syscall kind、entry timestamp、clockid（若有）、请求参数读取状态和最小必要字段。exit 仅与相同线程且 syscall kind 匹配的 entry 配对。线程退出、超时、context stack 溢出、map 插入失败和 orphan exit 均单独计数。相同线程的嵌套同名 syscall 必须保留外层上下文；不得将第二次 entry 当作覆盖。正式 producer 应为每次 syscall 分配 invocation ID，并在 exit 原样回传；内部 handler 还应携带 parent syscall invocation ID。具体字段与生命周期见 Navigation 57。

不保存或输出原始用户指针。指针仅在 syscall entry 的本次 BPF 程序调用内用于安全读取，随后丢弃。

### 4.2 用户内存读取与 ABI 门禁

`clock_adjtime` / `adjtimex` 的 tracepoint `tx` 字段是用户态 `struct timex *`。必须使用受支持的安全用户内存读取 helper（例如 `bpf_probe_read_user`），并将读取失败作为显式事件状态与计数器，不能用零值代替失败。注意：Linux syscall entry tracepoint 在实际 syscall handler 前触发；此处读取用户指针只得到 E2a pre-dispatch snapshot。另一个线程可能并发修改共享用户内存，使该样本与稍后 kernel copy_from_user 实际复制的字节不同。若必须观察内核消费的规范化请求，应另行评估 copy_from_user 之后的内部 kernel function hook（E2b），见专题 45。

BTF/CO-RE 不能自动解决任意用户态 UAPI 结构布局：libbpf 的 `BPF_CORE_READ_USER` 仍要求被访问类型存在于 kernel/kernel-module BTF；用户态 UAPI 类型若不在目标 BTF 中，不能据此假设 CO-RE relocation 可用。因此：
- 首个实现切片最多只解码经目标 ABI 明确验证的 `modes` 字段；
- 必须记录 architecture、word size、endianness、compat syscall/ABI 情况、UAPI header 版本、结构大小和字段 offset 来源；
- 32-bit compat task、未知 ABI、字段布局不匹配或用户内存读取失败时，operation_class 必须为 unknown；
- offset/freq/time 等后续字段只有在独立 ABI fixture、边界测试和目标机验证完成后才允许启用；
- 禁止根据常见 struct 布局硬编码跨架构偏移。

syscall exit 必须保留原始返回值：非负值可能是 CLOCK_REALTIME 路径返回的 TIME_* 状态码，不一定是 0。特别地，Linux clock_adjtime/adjtimex 包装器在执行时钟处理后还会 copy_to_user；若回传失败可返回 -EFAULT，即使时钟处理可能已经执行。因此负 -EFAULT 不能证明没有副作用；所有返回值都不能单独证明独立时钟效果或误差预算合格。详见专题 43。

### 4.3 clockid 与 PHC 设备身份

- `CLOCK_REALTIME` 是明确的系统 wall-clock 目标。
- `adjtimex` 的默认时钟域按 Linux API 语义记录为系统时钟接口，但仍需结合 modes 与 syscall 结果。
- `clock_adjtime` 的 dynamic clock ID 不得仅按数值反推 PHC index。
- 若后续需要将 dynamic clock ID 解析为 `/dev/ptpN`，必须独立设计同进程 fd 生命周期追踪，覆盖 open/openat/openat2、close、dup/dup2/dup3、fcntl duplication、fork/clone、exec、fd reuse、继承及可能的 descriptor passing；若无法覆盖某条路径，则映射保持 unresolved。
- `/proc/<pid>/fd` 的事后快照只能作为旁证，存在 fd close/reuse 竞态，不能替代事件时刻的映射。

第一版不得为了得到 PHC 名称而引入不完整的 fd 映射，并把猜测写成事实。

### 4.4 时间戳与时钟域

BPF 事件使用 `bpf_ktime_get_ns()` 记录 boot-relative monotonic 时间。该 helper 对应 CLOCK_MONOTONIC 语义且不计入系统 suspend 时间；事件报告必须同时记录 boot ID，避免跨 boot 拼接。

- 事件时间戳不使用 CLOCK_REALTIME，因为本采集器正要观测 CLOCK_REALTIME 可能发生的跳变。
- 用户态启动/停止时可采集 CLOCK_MONOTONIC 与 CLOCK_REALTIME 配对样本，但它们是有采集区间的关联观测，不是原子映射；不得用简单插值制造高精度 UTC traceability。
- suspend/resume 会影响 monotonic 与 boottime 的关系；若采集窗口跨 suspend，必须显式标记窗口不可直接连续比较，或额外采集 CLOCK_BOOTTIME 作为独立时间域。
- 跨 CPU 事件的严格全序不能只依赖时间戳或 per-CPU 序号。每 CPU 序号只检测本 CPU stream gap；线程迁移时必须使用 producer-side syscall/handler invocation ID 与 parent ID 配对。时间戳用于 duration/sanity check，不作为身份键。详见 Navigation 57。

## 5. 事件完整性与丢失计量

采用有界 ring buffer（或经目标 kernel 验证的 perf-event buffer）传递事件。Linux BPF ring buffer 在空间不足时 reservation 会失败且不会阻塞，因此 BPF 程序必须显式统计 reserve failure，不能静默丢弃。

至少维护以下计数器，优先使用 per-CPU map，避免热路径全局锁竞争：

| 计数器 | 说明 |
|---|---|
| syscall_entry_seen | 观察到的 entry 数 |
| syscall_exit_seen | 观察到的 exit 数 |
| entry_pair_created | 成功建立 in-flight 关联 |
| exit_pair_completed | 成功配对的 exit 数 |
| orphan_exit | 没有匹配 entry 的 exit |
| syscall_context_stack_overflow | 嵌套上下文栈达到固定深度，未覆盖外层项 |
| inflight_map_update_failed | map 容量/更新失败 |
| timex_user_read_failed | timex 用户内存读取失败 |
| timespec_user_read_failed | timespec 用户内存读取失败 |
| timeval_user_read_failed | timeval 用户内存读取失败 |
| ringbuf_reserve_failed | ring buffer 预留失败 |
| event_submit_attempted | 尝试提交的事件数 |
| userspace_event_received | 用户态实际收到的事件数 |
| userspace_decode_failed | 用户态解码失败数 |
| userspace_write_failed | 持久化输出失败数 |

每个内核事件包含 per-CPU sequence number、CPU id、event kind、pid/tgid、boot ID 对应的窗口标识和 monotonic timestamp。序号缺口、内核 reserve-failure 计数、用户态接收数和解析/写盘失败数应交叉核对。停止时先停止/摘除 probes，再读取最终 per-CPU counters 与 map 状态；在关闭消费者之前 drain ring buffer。若无法建立稳定的 stop/drain 顺序，报告状态不得为 complete。

建议报告同时给出：
- `kernel_capture_status=complete|partial|unavailable`
- `userspace_drain_status=complete|partial|unknown`
- `loss_counters`（所有计数及读取状态）
- `per_cpu_sequence_gaps`
- `inflight_remaining_at_stop`
- `event_reconciliation_status=balanced|mismatch|unknown`

即使所有可见计数均为零，也只能说明没有观察到这些已计量的丢失条件；不能绝对证明硬件/内核/程序链路零丢失。任何计数器不可读、map 更新失败、sequence gap、消费者异常或无法完成 drain 都必须 fail closed 为 partial。

## 6. 运行控制与最小权限

正式实现前必须具备：
- 明确的用户启动动作、采集时长/停止条件和输出目录；
- 记录 kernel release、machine、BTF identity、程序 SHA、libbpf/loader 版本、capability report SHA、boot ID；
- 明确所需 CAP_BPF/CAP_PERFMON/CAP_SYS_ADMIN 或发行版实际安全策略，禁止默认 sudo 全权运行；
- 只 attach 已批准的四类 syscall tracepoint，不启用全局 trace；
- 不使用 `bpf_printk` 作为事件通道；
- 有界 map/ring buffer、明确资源上限、退出清理和异常 detach；
- 运行前后保存 tracepoint enable 状态；不改变其他服务已启用的 tracing 配置；
- 对输出 JSON 做敏感信息审查，不输出 raw pointer、环境变量、命令行全量参数或无关进程内容。

本设计没有授权在用户目标 Orin 上自动加载程序。实际加载/采集须在目标内核能力报告、权限审查和安全评估完成后另行批准。

## 7. 测试合同

### 7.1 纯逻辑与 ABI 测试
1. syscall entry/exit 同线程正确配对；
2. 不同 TID 同 PID 并发调用不串线；
3. orphan exit、重复 entry、entry overwrite、in-flight timeout；
4. 每个 syscall kind 的字段缺失、格式变化和 unknown ABI；
5. timex modes 只读查询、ADJ_SETOFFSET、ADJ_OFFSET、ADJ_FREQUENCY、未知组合；
6. 用户内存读取成功/失败/部分可读/无效地址；
7. 32-bit compat ABI 与 64-bit ABI 隔离；
8. CLOCK_REALTIME 与 dynamic clock ID 区分，PHC 名称未确认时保持 unresolved；
9. syscall 返回成功/失败分别分类，不把 success 误报为已测量的 clock effect。

### 7.2 丢失与关闭测试
1. ring buffer reserve failure 注入并计数；
2. per-CPU sequence gap；
3. in-flight map 满、更新失败与遗留项；
4. 用户态 consumer 慢读、异常退出、JSON 解码失败、磁盘写入失败；
5. detach 与 final counter snapshot 顺序；
6. drain 前后事件数 reconciliation；
7. 计数器读取失败或不一致必须得到 partial；
8. 多 CPU 并发压力与有界内存测试。

### 7.3 集成与目标机验证
- 先在可控 VM/开发板执行 synthetic clock-adjust test；不得先对生产机器人运行。
- 用受控测试进程分别产生只读 adjtimex、phase/frequency 调整、ADJ_SETOFFSET、clock_settime(CLOCK_REALTIME) 与 settimeofday 请求；只有测试授权允许时才执行真实时钟修改，并在隔离环境中验证。
- PHC 测试使用独立可控 PHC/测试时钟，禁止对机器人运行中的同步主时钟进行破坏性测试。
- 将采集事件与 LinuxPTP phc2sys/ptp4l 原始日志、PMC 快照、PHC 设备状态和独立 clock samples 对齐；所有证据保留原始时钟域与 boot ID。
- 必须验证目标 Orin kernel/ARM64 ABI、BTF、tracepoint format、helper 可用性、verifier load、权限、buffer 压力和 stop/drain 行为。
- 合成测试、x86 CI、VM 测试均不能替代目标 Orin 实机验证。

## 8. 实施 Gate 与当前结论

Producer-side invocation identity、task lifetime discriminator、parent syscall context 与跨 CPU ordering 设计见 [57 — invocation identity and ordering contract](57_linux_clock_adjustment_invocation_identity_and_ordering.md)。当前 schema-v2 reducers 的 LIFO 只用于无 cookie 的合成/旧数据，不可作为最终生产采集身份方案。

当前只完成 capability inventory 和本设计合同，尚未实现 BPF 程序、loader、consumer、ABI decoder、loss accounting 或 clock effect correlator。

进入实现前的 Gate：
1. 用户提供目标 Orin 的 `robodog_kernel_trace_capabilities.json`；
2. 固定目标 kernel release、ARM64 ABI、tracepoint format、BTF 与可用 helper；
3. 明确权限模型、采集时长、性能预算和允许的测试时钟；
4. 完成 ABI-safe user-memory read 设计与 fixtures；
5. 完成 per-CPU loss counters、ring-buffer reconciliation 和 stop/drain 设计评审；
6. 完成最小权限与隐私评审。

未满足以上条件，不实现或加载 tracing 程序。当前状态保持：
- 源码/设计审计：完成本合同首版；
- 合成测试：本合同尚未形成代码测试；
- ROS 2 CI：不适用本设计文档；
- VM/Orin/PHC 实测：未完成，需要实物/授权；
- N1/N9：未验收。

## 9. 上游依据

- Linux `adjtimex(2)` / `clock_adjtime(2)`：`struct timex`、modes、offset/frequency 与 syscall 返回语义：
  https://man7.org/linux/man-pages/man2/adjtimex.2.html
- Linux BPF helper `bpf_probe_read_user`：用户内存读取会返回错误，必须显式处理：
  https://man7.org/linux/man-pages/man7/bpf-helpers.7.html
- libbpf `BPF_CORE_READ_USER`：用户内存 CO-RE 读取仍要求类型存在于 kernel/kernel-module BTF：
  https://github.com/libbpf/libbpf/blob/master/src/bpf_core_read.h
- Linux BPF ring buffer：reservation 可能因空间不足失败；API 不阻塞，调用方需处理失败：
  https://www.kernel.org/doc/html/v6.6/bpf/ringbuf.html
- Linux kernel tracepoints：probe 在 tracepoint 被触发时执行，字段布局来自具体 tracepoint 定义：
  https://kernel.org/doc/html/next/trace/tracepoints.html
- Linux BPF `bpf_ktime_get_ns`：boot-relative monotonic 时间，不计 suspend：
  https://docs.ebpf.io/linux/helper-function/bpf_ktime_get_ns/
- Linux kernel timekeeping：CLOCK_MONOTONIC、CLOCK_BOOTTIME 与 CLOCK_REALTIME 的时钟域差异：
  https://docs.kernel.org/core-api/timekeeping.html

---
本文件描述下一阶段的设计合同，不构成实现、部署或实机验收声明。
