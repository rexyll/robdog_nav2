# Linux BPF runtime-policy preflight inventory

- Status: READ_ONLY_POLICY_INVENTORY_IMPLEMENTED / SYNTHETIC_TESTS_ADDED / BPF_LOAD_NOT_ATTEMPTED
- Date: 2026-11-27
- Related: Navigation 46, 50, 51

## 1. Purpose

Navigation 51 inventories kernel config, tracefs syscall events, handler function-tracing visibility, and BTF file presence. This increment adds read-only observations of runtime policy and the security context of the process running the probe. This addresses a separate question: a kernel may expose tracing interfaces while the current process is restricted by sysctl policy, lockdown, capabilities, or seccomp.

## 2. Observed files

The report reads:
- `/proc/sys/kernel/unprivileged_bpf_disabled`
- `/proc/sys/kernel/perf_event_paranoid`
- `/proc/sys/kernel/kptr_restrict`
- `/sys/kernel/security/lockdown`
- selected fields from `/proc/self/status`: Uid, Gid, CapEff, CapBnd, NoNewPrivs, Seccomp, Seccomp_filters.

Unavailable files are reported as unavailable; the probe does not infer permissive defaults. Lockdown active mode is extracted only when the kernel exposes the bracketed active mode.

## 3. Interpretation limits

These values are preflight evidence, not an authorization oracle:
- `unprivileged_bpf_disabled` describes unprivileged BPF policy, not whether a privileged process can load a particular program;
- `perf_event_paranoid` is relevant to perf access and is not a universal BPF attach verdict;
- lockdown, LSM policy, user/network namespaces, capabilities, seccomp, kernel version, BPF command type, map/program type, and verifier rules interact;
- `CapEff` is specific to the probe process at collection time;
- a rootfs fixture's `/proc/self/status` is only representative if it is the actual target process namespace, not an arbitrary copied file.

The probe does not call `bpf()`, `perf_event_open()`, `prctl()`, or any privilege-changing API. It does not attempt to elevate privileges, alter sysctls, alter lockdown, or change tracefs state.

## 4. Decision use

A restrictive value can explain why a later controlled attach attempt may be denied, but a permissive-looking report does not authorize or guarantee attachment. The next attach gate must be a separate, explicitly approved, bounded experiment on the intended kernel, with exact kernel release, BTF and program type recorded. Do not make clock-adjustment syscalls as part of collector bring-up.

## 5. Validation

Synthetic tests cover readable policy files, bracketed lockdown mode, selected process status fields, and missing-file fail-closed behavior. CI validates parser behavior only. No live VM policy report or BPF attach result is included by this repository change.
