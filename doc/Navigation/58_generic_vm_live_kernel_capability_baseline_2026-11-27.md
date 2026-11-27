# Generic VM live kernel capability baseline — 2026-11-27

- Status: OBSERVED / READ_ONLY / PARTIAL
- Source artifact: user-provided `kernel-trace-capabilities.json`
- Collection mode: `live_host` (not a rootfs fixture)
- Scope: the kernel and process from which the collector ran; this is not an Orin target report.

## 1. Kernel identity

| Field | Observed value |
|---|---|
| OS | Linux |
| Release | `7.0.0-44-generic` |
| Architecture | `x86_64` |
| BTF path | `/sys/kernel/btf/vmlinux` |
| BTF status | `present_readable` |
| BTF size | 7,134,362 bytes |
| BTF SHA-256 | `b46a7adb3d4d3738fce0038feb494055ca43ed398cad4148c52954d3e68e7825` |
| bpftool | `v7.7.0` |
| libbpf | `1.7` |

The BTF digest pins the observed byte content only. It does not prove source equivalence, symbol stability, attachability or verifier acceptance.

## 2. Kernel configuration

Observed in `/boot/config-7.0.0-44-generic`:

- `CONFIG_BPF=y`
- `CONFIG_BPF_EVENTS=y`
- `CONFIG_BPF_SYSCALL=y`
- `CONFIG_DEBUG_INFO_BTF=y`
- `CONFIG_DEBUG_INFO_BTF_MODULES=y`
- `CONFIG_FTRACE=y`
- `CONFIG_FTRACE_SYSCALLS=y`
- `CONFIG_HAVE_SYSCALL_TRACEPOINTS=y`
- `CONFIG_KPROBE_EVENTS=y`
- `CONFIG_PERF_EVENTS=y`
- `CONFIG_TRACEPOINTS=y`

`/proc/config.gz` was not found. Config values are build-time evidence, not runtime permission or feature probes.

## 3. BTF function prototypes

The collector resolved BTF FUNC → FUNC_PROTO and parameter/return type chains:

| Function | Linkage | Prototype |
|---|---|---|
| `do_clock_adjtime` | static | `int do_clock_adjtime(const clockid_t which_clock, struct __kernel_timex *ktx)` |
| `do_adjtimex` | static | `int do_adjtimex(struct __kernel_timex *txc)` |

This confirms that the two named FUNC records and their prototypes exist in this kernel's BTF. It does **not** establish that fentry/fexit can attach to either function. Both have static linkage, and the independent function-tracing symbol inventory returned `permission_denied`; symbols remain operationally unknown. No attach was attempted.

## 4. Tracefs and syscall tracepoints

- tracefs mount exists at `/sys/kernel/tracing` with `rw,nosuid,nodev,noexec,relatime`.
- Reading `available_events` and event `format` files returned `permission_denied`.
- `/sys/kernel/debug/tracing` was also not readable.
- Consequently every requested syscall entry/exit event, including native and compat `clock_adjtime*` and `adjtimex*`, is `unknown`, not absent and not available.

The candidate event names in the report are a probe list only. They are not observed event availability.

## 5. Runtime policy snapshot

| Field | Value |
|---|---|
| Lockdown | `none` |
| Effective capabilities | `0000000000000000` |
| Bounding capabilities | `000001ffffffffff` |
| `NoNewPrivs` | `0` |
| Seccomp | `0` |
| `perf_event_paranoid` | `4` |
| `unprivileged_bpf_disabled` | `2` |
| `kptr_restrict` | `1` |

The process has no effective capabilities, and unprivileged BPF is disabled by policy. This is a strong reason not to expect an unprivileged BPF load to succeed, but no BPF syscall was attempted; do not turn this preflight into a definitive load-denied result. No sudo/root escalation or sysctl modification is authorized by this observation.

## 6. Design implications

1. BTF-based type inspection is available to the current read-only collector.
2. A BTF-driven fentry/fexit design remains a candidate, not a selected path: target function attachability and verifier behavior are untested, and static linkage plus inaccessible function tracing inventory must be investigated.
3. Syscall tracepoint fallback remains unresolved because tracefs event inventory and formats could not be read.
4. Kernel build configuration is favorable but cannot override runtime security policy.
5. Do not attempt BPF load/attach, mount tracefs, enable events, alter sysctls, or change clocks as part of this baseline.
6. Repeat this report under an explicitly authorized privileged diagnostic context only if the user chooses; preserve the current unprivileged baseline as a separate observation rather than overwriting it.

## 7. Acceptance boundary

This report does not establish:
- syscall tracepoint availability;
- fentry/fexit or kprobe attachability;
- BPF map/helper availability for the proposed task-context stack;
- verifier acceptance or ring-buffer delivery;
- target Orin kernel/BSP capabilities;
- real clock adjustment or PHC behavior.

No BPF object was loaded, no probe attached, no tracepoint enabled, no trace buffer read, and no clock changed.
