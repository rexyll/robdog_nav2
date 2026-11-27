# Syscall tracepoint versus post-copy kernel request evidence

- Status: SOURCE_AUDITED / DESIGN_ONLY / TARGET_HOOKS_NOT_VERIFIED
- Date: 2026-11-27
- Reference kernel: Linux v6.12 source tag (not an Orin BSP claim)
- Related: [42 — Linux clock adjustment event capture](42_linux_clock_step_event_capture_contract.md)
- Related: [43 — phc2sys source-to-syscall mapping](43_phc2sys_clockadj_source_to_syscall_mapping.md)
- Related: [44 — userspace event reducer](44_linux_clock_adjustment_event_reducer_contract.md)

## 1. Finding: syscall entry tracepoints are pre-handler observations

Linux v6.12 `kernel/entry/common.c` calls `trace_sys_enter(regs, syscall)` from `syscall_trace_enter()`. This happens before the architecture syscall dispatcher invokes the selected syscall implementation.

The generic `include/trace/events/syscalls.h` defines:
- `sys_enter`: syscall ID plus six `unsigned long` argument slots populated by `syscall_get_arguments()`;
- `sys_exit`: syscall ID and raw return value.

The tracepoint does not dereference a pointer argument. For `clock_adjtime`, the syscall argument is a userspace pointer to the timex request. An eBPF program attached to `sys_enter_clock_adjtime` that calls `bpf_probe_read_user()` on that pointer samples user memory before the syscall handler performs its own `copy_from_user()`.

This creates an important evidence distinction:
- The tracepoint proves a syscall entry and supplies the raw pointer argument.
- A successful `bpf_probe_read_user()` at syscall entry proves only that a user-memory sample was readable at the probe instant.
- It does not prove that the bytes sampled are exactly the bytes subsequently copied by the kernel.

A different thread in the same process may modify shared memory between the tracepoint probe and the syscall handler's `copy_from_user()`. Therefore the captured `modes` value is a **pre-dispatch user-memory snapshot**, not definitive evidence of the kernel-consumed request. A user-memory read failure is also not evidence that the syscall will fail; the kernel may subsequently read the pointer successfully, or vice versa.

Pinned source:
- `kernel/entry/common.c`: https://github.com/torvalds/linux/blob/v6.12/kernel/entry/common.c
- `include/trace/events/syscalls.h`: https://github.com/torvalds/linux/blob/v6.12/include/trace/events/syscalls.h
- `kernel/trace/trace_syscalls.c`: https://github.com/torvalds/linux/blob/v6.12/kernel/trace/trace_syscalls.c

## 2. Stronger request evidence: observe the kernel copy after copy_from_user

The Linux v6.12 `clock_adjtime` wrapper performs:

1. `copy_from_user(&ktx, utx, sizeof(ktx))`;
2. `do_clock_adjtime(which_clock, &ktx)`;
3. if the handler result is nonnegative, `copy_to_user(utx, &ktx, sizeof(ktx))`.

The internal `do_clock_adjtime()` function receives the kernel-owned `struct __kernel_timex` after the user request has been copied. A target kernel that exposes a usable fentry/kprobe/BTF attach point at this function therefore offers a stronger source for the normalized request fields than re-reading the user pointer at syscall entry.

Compat path:
- `clock_adjtime32` first converts the old 32-bit userspace structure to `struct __kernel_timex` via `get_old_timex32()`, then calls the same `do_clock_adjtime()`.
- A probe at the post-conversion internal function can observe the normalized kernel request without assuming the userspace compat layout.

System clock adjtimex path:
- The native `adjtimex` wrapper performs `copy_from_user()`, then calls `do_adjtimex(&txc)`, then copies the result back.
- The compat `adjtimex_time32` wrapper converts the old ABI structure and then calls `do_adjtimex()`.
- A probe at `do_adjtimex()` is therefore another candidate post-copy request observation point.

These internal functions are candidate hooks, not guaranteed target instrumentation points. Before using them, the target BSP must establish:
- function presence and exact signature for its kernel release;
- BTF function/type availability or supported kprobe attachment;
- compiler/LTO/inlining behavior and security/permission constraints;
- whether the same internal function has non-syscall callers that require separate attribution.

Pinned source:
- `kernel/time/posix-timers.c`: `do_clock_adjtime()`, `clock_adjtime()`, `clock_adjtime32()`:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-timers.c
- `kernel/time/time.c`: `adjtimex()`, `adjtimex_time32()`:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/time.c

## 3. A paired handler return can disambiguate copy-back EFAULT

For `clock_adjtime()`, if the post-copy handler `do_clock_adjtime()` is observed returning a nonnegative value but the outer syscall exit returns `-EFAULT`, the v6.12 wrapper's source order identifies a copy-back failure after the handler returned. The pre-handler `copy_from_user()` failure path cannot produce that observed nonnegative internal handler return because the handler would not have been called.

Likewise for `adjtimex()`, a nonnegative `do_adjtimex()` return followed by outer syscall `-EFAULT` identifies the later copy-to-user failure in the pinned wrapper.

This is stronger than syscall entry/exit alone, but it still does not prove an exact physical clock effect:
- a nonnegative handler return is a kernel handler result, not an independent PHC/CLOCK_REALTIME measurement;
- a device driver's callback may return after accepting a request without guaranteeing the requested physical phase/frequency change is observable at the requested time;
- error/rollback behavior is driver- and kernel-specific;
- a kretprobe/fentry-return observation can itself be unavailable or lost.

Recommended separate fields:
- `syscall_entry_observed`
- `pre_dispatch_user_snapshot_status`
- `pre_dispatch_user_modes` (never label as kernel-consumed)
- `post_copy_kernel_request_status`
- `post_copy_kernel_modes`
- `kernel_handler_return`
- `outer_syscall_return`
- `copyback_failure_inferred_from_pinned_wrapper` (only when the internal handler return and outer syscall return satisfy the source-derived condition)
- `clock_effect_status=not_measured`

## 4. Target-specific clock implementation still matters

For `CLOCK_REALTIME`, the v6.12 POSIX clock callback routes to `do_adjtimex()`.

For a dynamic POSIX clock, `do_clock_adjtime()` resolves the dynamic clock ID to a registered clock operation. The PTP implementation then dispatches modes to its clock/driver callbacks.

A generic post-copy observation at `do_clock_adjtime()` establishes the normalized request and raw clock ID, but does not by itself identify the final driver callback. If the goal is to distinguish PHC `adjtime`, `adjfine`, or `adjphase` handler execution, a second target-specific hook may be needed in the registered PTP/driver path. This hook is not portable by assumption and must be source-pinned to the actual driver revision.

Relevant reference:
- PTP mode dispatch in Linux v6.12: https://github.com/torvalds/linux/blob/v6.12/drivers/ptp/ptp_clock.c
- Dynamic POSIX clock dispatch: https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-clock.c

## 5. Revised evidence model

The event contract should distinguish the following:

| Evidence | Label | Meaning |
|---|---|---|
| sys_enter record | E1 | syscall invocation observed before handler |
| userspace pointer read at sys_enter | E2a | pre-dispatch snapshot, subject to shared-memory TOCTOU |
| internal kernel request after copy-in | E2b | normalized kernel-owned request passed to handler |
| internal handler return | E3a | handler return result observed |
| outer syscall exit | E3b | final user-visible syscall return |
| pair E3a + E3b | E3c | may identify post-handler copyback failure under pinned wrapper ordering |
| independent clock/PHC sample | E5 | separate effect observation with its own time/uncertainty model |

Neither E2b nor E3a/E3b is equivalent to E5.

## 6. Consequences for implementation order

Without Orin, the project can:
1. keep the current syscall event reducer independent of the capture source;
2. extend the event envelope to represent E2a versus E2b observations distinctly;
3. add synthetic cases for a differing E2a/E2b modes sample, successful post-copy handler plus outer -EFAULT, negative handler result, and missing internal hook;
4. keep the final result partial/unknown whenever internal-hook availability or pairing is not established.

Still hardware/BSP gated:
- target Orin BTF/fentry/kprobe availability and exact symbol signatures;
- target kernel syscall tracepoint format and ARM64 calling conventions;
- actual handler paths and PHC driver callbacks;
- measured event loss, runtime overhead and independent clock effect.

No kernel hook is implemented or attached by this document. It does not enable tracing or alter any clock.

---
Source audit only; Linux v6.12 is a reference snapshot, not a deployment claim.
