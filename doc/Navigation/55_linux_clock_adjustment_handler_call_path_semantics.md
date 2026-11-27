# Linux clock-adjustment handler call-path semantics

- Status: UPSTREAM_CALL_PATH_AUDITED / LIVE_TARGET_UNVERIFIED
- Date: 2026-11-27
- Scope: Linux v6.12 source-family semantics; target VM and Orin BSP must be checked independently
- Related: Navigation 45–54

## 1. Purpose

Navigation 45–47 distinguish syscall-entry user-pointer evidence, post-copy kernel request evidence, and syscall/handler correlation. This document pins down the semantic difference between the two candidate internal handlers. They are not interchangeable, and observing either one does not by itself prove a successful clock change.

## 2. `clock_adjtime` path

The upstream `kernel/time/posix-timers.c` implementation has this sequence:

1. `SYSCALL_DEFINE2(clock_adjtime, which_clock, utx)` receives a userspace `struct __kernel_timex *`.
2. It copies the complete request into a kernel-local `struct __kernel_timex ktx` using `copy_from_user()`. A copy fault returns `-EFAULT` before the internal handler.
3. It calls `do_clock_adjtime(which_clock, &ktx)`.
4. `do_clock_adjtime` resolves `which_clock` to a `struct k_clock`; invalid clock IDs return `-EINVAL`, clocks without `clock_adj` return `-EOPNOTSUPP`, otherwise the selected callback is invoked.
5. For the ordinary `CLOCK_REALTIME` clock, `posix_clock_realtime_adj` calls `do_adjtimex(ktx)`.
6. If the callback returns non-negative, the syscall copies the updated kernel structure back to userspace; a copy-back fault changes the syscall result to `-EFAULT`.

Consequences:

- `do_clock_adjtime` entry is post-copy evidence for the kernel request structure, and includes the selected `clockid_t`.
- It is a dispatch point, not proof that `do_adjtimex` will execute. Other clock IDs may resolve to other callbacks or fail before a callback.
- `do_adjtimex` is expected on the ordinary realtime callback path, but should not be inferred from every `do_clock_adjtime` event.
- A successful internal callback does not guarantee userspace copy-back success.

## 3. `adjtimex` path

The syscall implementation in `kernel/time/time.c` separately copies the userspace timex structure into a kernel-local object, calls `do_adjtimex()`, then copies the updated object back. This path does not pass through `do_clock_adjtime`.

Consequences:

- A collector that observes only `do_clock_adjtime` misses direct `adjtimex` calls.
- A collector that observes `do_adjtimex` can cover both the direct `adjtimex` path and the ordinary realtime `clock_adjtime` callback path, subject to target-kernel implementation and attachability.
- Syscall-entry correlation must retain syscall identity; do not assume every `do_adjtimex` invocation has a preceding `clock_adjtime` syscall event.

## 4. Evidence levels and event fields

Keep these distinctions in the event schema:

| Evidence | Meaning | Must not claim |
|---|---|---|
| syscall entry | syscall number, scalar arguments and userspace pointer value observed | kernel has read the pointed-to structure |
| `do_clock_adjtime` entry | kernel-local request is available after `copy_from_user`; clock ID available | realtime timekeeper path necessarily invoked |
| `do_adjtimex` entry | kernel-local timex request reaches the timekeeping adjustment handler | operation succeeds or wall clock changes |
| handler exit | internal return value observed | userspace copy-back succeeded |
| syscall exit | final syscall return observed | physical clock/PHC effect independently measured |

For `do_clock_adjtime`, capture `which_clock` and handler invocation identity. For `do_adjtimex`, capture a bounded, versioned snapshot of selected scalar request fields only when the BPF program can safely read the kernel pointer under verifier rules. Never dereference the userspace pointer from generic syscall tracepoints.

The event should also retain `pid_tgid`, boot/session identity, monotonic timestamp, CPU, per-stream sequence, syscall correlation key, handler kind, and return value where applicable. Do not serialize raw kernel pointers.

## 5. Correlation and pairing consequences

- One `clock_adjtime` syscall can produce a `do_clock_adjtime` invocation and, for the realtime callback, a nested `do_adjtimex` invocation.
- One direct `adjtimex` syscall can produce `do_adjtimex` without `do_clock_adjtime`.
- Pair entry/exit by task invocation state, not per-CPU stack, because tasks can migrate. A task-keyed map must include nesting depth or invocation cookie; map update failure, overflow, unmatched exit, stale state and task-exit cleanup require explicit counters.
- Do not join only by PID and a broad time interval when a stronger invocation cookie/parent context can be recorded. The existing interval joiner remains a weaker fallback and must label its ancestry limitation.
- Compat `clock_adjtime32` also reaches `do_clock_adjtime` after compat conversion; syscall-number coverage must be architecture/ABI-specific and verified against target source.

## 6. Probe selection policy

No hook is selected by this source audit alone.

1. Prefer a verified BTF fentry/fexit attachment only after target VM report confirms the exact FUNC/FUNC_PROTO and a minimal load/attach test succeeds.
2. If fentry is unavailable, evaluate kprobe/kretprobe only after target symbol visibility, kernel restrictions, registration success and ABI are verified.
3. Keep syscall tracepoints as the outer evidence stream. Their generic entry payload is not a safe source for dereferencing userspace timex pointers.
4. Treat attach success, event delivery, loss accounting and shutdown drain as separate gates.
5. Never run a real `clock_adjtime`/`adjtimex` request merely to test instrumentation. Use benign syscall tracepoint tests and synthetic handler fixtures; any future live handler test must use a strictly read-only request proven not to mutate clock state, and must be reviewed before execution.

## 7. Source evidence and limitations

- Upstream `kernel/time/posix-timers.c`: `posix_clock_realtime_adj`, `do_clock_adjtime`, `SYSCALL_DEFINE2(clock_adjtime)`, and `SYSCALL_DEFINE2(clock_adjtime32)`. The source shows copy-in before dispatch and conditional copy-back after the callback.
- Upstream `kernel/time/timekeeping.c`: `do_adjtimex(struct __kernel_timex *)` validates the kernel request, performs timekeeping/NTP work and may call `clock_was_set(CLOCK_SET_WALL)`.
- Upstream `kernel/time/time.c`: direct `adjtimex` syscall copies in, calls `do_adjtimex`, then copies out.
- The source audit establishes upstream call-path semantics only. It does not prove a downstream vendor kernel retains the same functions, signatures, inlining behavior, BTF records, symbol visibility, attachability or permissions.
- The user's actual Linux VM report is still required. Orin BSP source/config/BTF and attachability remain a separate hardware-platform gate.
