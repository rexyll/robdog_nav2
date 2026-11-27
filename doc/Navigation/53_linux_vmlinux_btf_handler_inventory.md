# vmlinux BTF handler-function inventory

- Status: OPTIONAL_BPFTOOL_BTF_FUNC_INVENTORY_IMPLEMENTED / SYNTHETIC_TESTS_ADDED / ATTACH_NOT_TESTED
- Date: 2026-11-27
- Related: Navigation 46, 51, 52

## 1. Purpose

Navigation 51 reports whether the vmlinux BTF artifact is present and whether candidate handler symbols appear in function-tracing inventory. File presence and function-tracing visibility do not establish that the target functions have BTF FUNC records. This increment adds an optional read-only inspection using the host's bpftool executable.

## 2. Operation

When bpftool is available and the selected vmlinux BTF path exists, the probe runs:

```bash
bpftool btf dump file /sys/kernel/btf/vmlinux format raw
```

It searches the raw dump for FUNC records named `do_clock_adjtime` and `do_adjtimex`. The command has a 20-second timeout. Missing bpftool, missing BTF, timeout, execution failure, and nonzero command exit are distinct statuses; failed inspection leaves symbol status unknown.

This operation reads the BTF file. It does not invoke the BPF syscall, load a program, attach a hook, or change kernel state. It does not require or imply root privileges for the file read itself, though host file permissions and bpftool availability apply.

## 3. Interpretation

A `func_record_present` result establishes only that a FUNC record with that name occurs in the inspected BTF dump. It does not establish:
- a usable FUNC_PROTO argument graph or the exact argument types expected by a chosen BPF program;
- fentry/fexit support or verifier acceptance;
- kprobe registration success;
- function stability or semantic equivalence across vendor kernel releases;
- BPF load/attach authorization under the current LSM, lockdown, namespace, capability, and seccomp context.

A missing FUNC record is not proof that no kprobe or alternate instrumentation route exists. The bpftool executable is from the running host PATH; when inspecting an arbitrary rootfs fixture, it is not necessarily a tool built for that target. Therefore run the probe on the target VM itself for authoritative runtime evidence.

## 4. Validation and next gate

Synthetic tests cover present/absent FUNC records, missing bpftool, and command failure. CI validates parser behavior only. Next, collect the report on the generic Linux VM, retain the exact kernel release and BTF artifact identity, then inspect the referenced FUNC_PROTO records before designing a minimal attach test. Repeat independently on the exact Orin BSP.

No real VM report, BPF load/attach, clock-setting syscall, or clock modification is included in this change.
