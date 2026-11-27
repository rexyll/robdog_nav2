# Linux clock-adjustment syscall tracepoint ABI coverage

- Status: UPSTREAM_SYSCALL_NAMES_AUDITED / TARGET_TRACEFS_UNVERIFIED
- Date: 2026-11-27
- Scope: Linux v6.12 source-family; x86 i386/compat and generic ARM32 time32/time64 mappings. Native x86-64/AArch64 mappings must still be checked against the exact target kernel syscall tables.
- Related: Navigation 54–55

## 1. Finding

The capability probe previously inventoried only `sys_enter/exit_clock_adjtime` and `sys_enter/exit_adjtimex`. That is insufficient for kernels exposing legacy 32-bit time ABI syscall implementations as separately named syscall tracepoints.

The syscall tracepoint names are derived from the syscall implementation names (the `SYSCALL_DEFINE*` names), not simply from every userspace ABI alias in a syscall table. Therefore the inventory now includes:

- `clock_adjtime` (also the implementation used by i386/ARM time64 syscall-table aliases such as `clock_adjtime64`);
- `clock_adjtime32` (legacy 32-bit timex conversion path);
- `adjtimex` (native `struct __kernel_timex` path);
- `adjtimex_time32` (legacy 32-bit timex conversion path).

The probe reports each entry/exit event independently and computes a separate pair status for each implementation name. An absent event remains `not_listed`; an unreadable event inventory remains `unknown`.

## 2. Source-backed ABI mapping

### x86 i386 syscall table

The Linux x86 i386 syscall table maps:

- syscall 124 `adjtimex` to `sys_adjtimex_time32`;
- syscall 343 `clock_adjtime` to `sys_clock_adjtime32`;
- syscall 405 `clock_adjtime64` to `sys_clock_adjtime`.

This means the implementation-level tracepoint names to look for are `adjtimex_time32`, `clock_adjtime32`, and `clock_adjtime`, respectively. The time64 syscall-table name does not imply a separate `clock_adjtime64` tracepoint when it targets the `sys_clock_adjtime` implementation.

### ARM32 syscall table

The ARM syscall table maps syscall 372 `clock_adjtime` to `sys_clock_adjtime32`, and syscall 405 `clock_adjtime64` to `sys_clock_adjtime`. This is the same implementation-name distinction as x86 i386. The exact syscall table and configuration of a target kernel remain authoritative.

### Kernel implementation

In `kernel/time/posix-timers.c`, `SYSCALL_DEFINE2(clock_adjtime32, ...)` converts `old_timex32` into a kernel `__kernel_timex`, then calls `do_clock_adjtime`; the ordinary `SYSCALL_DEFINE2(clock_adjtime, ...)` path copies the native kernel timex and calls the same dispatcher.

In `kernel/time/time.c`, `SYSCALL_DEFINE1(adjtimex_time32, ...)` converts the old structure and calls `do_adjtimex`; the native `SYSCALL_DEFINE1(adjtimex, ...)` path directly copies the native structure and calls `do_adjtimex`.

## 3. Collector changes

The read-only capability inventory now checks these additional tracepoint names:

- `sys_enter_clock_adjtime32` / `sys_exit_clock_adjtime32`
- `sys_enter_adjtimex_time32` / `sys_exit_adjtimex_time32`

The existing `clock_adjtime` and `adjtimex` pairs remain unchanged. No syscall number is hard-coded into the generic probe: syscall numbers and ABI aliases are architecture-specific, while tracepoint presence is determined from the target's `available_events` and `format` files.

No event is enabled, no trace buffer is read, no BPF program is loaded, and no clock-changing syscall is executed.

## 4. Validation and limits

Synthetic tests cover listed time32 entry/exit pairs and ensure pair status is computed independently. These tests validate parser behavior only. They do not prove that a particular VM or Orin kernel was built with these syscall implementations, that the tracepoints are listed, or that events can be captured without loss.

On the target VM, preserve the raw capability JSON and inspect the four implementation pairs. If the VM is x86-64 with IA32 emulation disabled, time32 compat tracepoints may legitimately be absent. Do not infer absence of compat support from an architecture name alone; verify kernel configuration and the live syscall table. Orin/AArch64 is a separate target and must be inventoried independently.

## 5. Upstream references

- Linux x86 i386 syscall table: https://github.com/torvalds/linux/blob/v6.12/arch/x86/entry/syscalls/syscall_32.tbl
- Linux ARM syscall table: https://github.com/torvalds/linux/blob/v6.12/arch/arm/tools/syscall.tbl
- Linux POSIX clock syscall implementations: https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-timers.c
- Linux adjtimex syscall implementations: https://github.com/torvalds/linux/blob/v6.12/kernel/time/time.c
- Linux syscall tracepoint metadata is generated from syscall implementation declarations; the target tracefs `available_events` and `format` files remain the final runtime evidence.

No target-host ABI coverage is claimed until the live VM report is reviewed. N1/N9 remain unaccepted; changes stay on `nav`.
