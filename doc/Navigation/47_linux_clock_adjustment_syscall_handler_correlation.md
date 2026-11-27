# Linux clock-adjustment syscall / post-copy handler correlation

- Status: PURE_LOGIC_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / CAPTURE_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: [44 — syscall event reducer](44_linux_clock_adjustment_event_reducer_contract.md)
- Related: [45 — syscall versus post-copy request evidence](45_syscall_tracepoint_vs_postcopy_kernel_request.md)
- Related: [46 — kernel-handler observation reducer](46_linux_clock_adjustment_kernel_observation_reducer.md)

## 1. Purpose

Navigation 44 reduces E1/E3b outer syscall events; Navigation 46 reduces E2b/E3a internal-handler events. This joiner correlates their completed intervals while retaining the two independent capture/sequence domains.

Public function: `correlate_clock_adjustment_observations(syscall_report, handler_report, wrapper_contract_id=None)`.

## 2. Matching rules

- Exact identity match on `(boot_id, pid_tgid)`.
- The handler interval must be fully contained in the outer syscall interval using CLOCK_MONOTONIC.
- Source-derived mapping: `clock_adjtime` and `clock_adjtime32` → `do_clock_adjtime`; `adjtimex` and `adjtimex_time32` → `do_adjtimex`.
- If multiple nested syscall intervals contain one handler interval, select only a unique narrowest interval. Equal-width candidates are ambiguous.
- If multiple handler intervals claim one syscall interval, none is joined. Missing/ambiguous matches remain explicit issues and make the result partial.

Interval containment is not proof of call-stack ancestry. A same-thread nested syscall may exist. A future collector should add a shared invocation cookie or explicit parent context if it can do so safely.

## 3. Copy-back EFAULT inference

Inference is disabled by default. It is enabled only by explicitly passing `linux_v6.12_clock_adjtime_adjtimex_wrapper_order`, selecting the source-pinned Linux v6.12 wrapper ordering audited in Navigation 45.

Inference is true only if the joined handler returned nonnegative and the outer syscall returned `-EFAULT` (`-14`). An internal handler returning `-EFAULT` does not qualify. This contract identifies the Linux v6.12 reference source only; it does not claim that an Orin BSP uses that source or ordering. Pin target BSP source/version before enabling this inference in deployment.

## 4. Output and limitations

The result includes joined pairs, unjoined handler indices, unmatched syscall indices, stable issue codes and partial/consistent status. Upstream partial status propagates. Every joined record retains `clock_effect=not_measured`.

This is pure Python over already reduced reports. It does not attach eBPF, prove event coverage, identify PHC callbacks, measure loss, or measure CLOCK_REALTIME/PHC effect. Interval matching is a conservative heuristic, not call-stack proof.

## 5. Tests and remaining gates

Synthetic tests cover native/compat mapping, thread/boot identity, full containment, nested selection, ambiguous ties, many-to-one collision, upstream partial propagation, malformed reports, and opt-in EFAULT inference.

Remaining: producer-side ordering/drop accounting and stop/drain snapshot; shared invocation cookie; target kernel/BSP source pin; BTF/fentry/kprobe validation; generic Linux VM then Orin/ARM64 and independent clock/PHC measurement.

No kernel hook, tracing session, clock modification or hardware validation is performed.
