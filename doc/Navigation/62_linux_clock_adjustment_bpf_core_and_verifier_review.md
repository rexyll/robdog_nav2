# BPF task-context producer: CO-RE structure and verifier-oriented review

- Status: PSEUDOCODE_REVIEW / NO_ATTACH_SECTION_SELECTED / NOT_COMPILED / NOT_VERIFIED
- Date: 2026-11-27
- Related: Navigation 50, 57, 59, 60, 61
- Scope: code-level layout review only. This is not an implementation and deliberately omits an attach section because the VM tracepoint inventory and handler symbol availability remain unknown.

## 1. Map topology

The producer needs four map roles:

1. **Task Storage map** — one fixed-size BTF value per current task, with `BPF_MAP_TYPE_TASK_STORAGE`, `BPF_F_NO_PREALLOC`, `max_entries=0`, integer key type and a BTF-described `struct task_context` value.
2. **Shared global counter ARRAY** — one map object shared by syscall and handler programs. Keys represent next event ID, next task-instance ID, task-context reservations, and allocator failure counters.
3. **Per-CPU sequence ARRAY** — keyed by stream kind (syscall or handler), with per-CPU values. This preserves separate per-CPU sequence domains.
4. **Ring buffer** — fixed-size records using the common wire ABI in Navigation 60.

The loader must prove that both producer program groups reference the same global counter map ID. A separate counter map per program group would permit event-ID collisions across streams.

## 2. BTF-described value structures

Proposed fixed-size task-local frame:

```c
struct invocation_frame {
    __u64 invocation_seq;
    __u64 parent_syscall_invocation_seq;
    __u64 entry_monotonic_ns;
    __u32 subject_id;
    __u8  emit_enabled;
    __u8  parent_context_status;
    __u16 reserved;
};

struct task_context {
    __u64 task_instance_seq;
    __u64 next_invocation_seq;
    __u32 syscall_depth;
    __u32 syscall_overflow_depth;
    __u32 handler_depth;
    __u32 handler_overflow_depth;
    struct invocation_frame syscall_stack[SYSCALL_DEPTH];
    struct invocation_frame handler_stack[HANDLER_DEPTH];
};
```

The event record is the fixed-width ABI in Navigation 60. No kernel pointer, `task_struct *`, `__kernel_timex *`, or user pointer may be serialized into the ring buffer. Only the scalar values successfully read at the probe are copied.

The exact depth constants are not selected yet. Compute value size at compile time, keep it below the exact target kernel's local-storage value bound, and test the max/max+1 transitions. Do not copy the entire task_context to the BPF stack; access the map-value pointer in place.

## 3. Helper and CO-RE access plan

For a current-task event program, the candidate sequence is:

1. Call `bpf_get_current_task_btf()` to obtain a trusted BTF `task_struct *`.
2. Call `bpf_task_storage_get()` with the Task Storage map and CREATE only after bounded task-slot admission.
3. Call `bpf_get_current_pid_tgid()` for the TGID/TID composite.
4. Call `bpf_get_smp_processor_id()` and `bpf_ktime_get_ns()` for CPU and monotonic timestamp.
5. Read only the required handler arguments/return values through verifier-approved BTF arguments or kernel memory-read helpers.

The two helpers have different license and privilege rules in upstream v7.0. `bpf_get_current_task_btf()` is marked `.gpl_only=true`, so a program using it needs a GPL-compatible BPF license. `bpf_task_storage_get()` is marked `.gpl_only=false`; it is not itself a GPL-only helper. However, creating a `BPF_MAP_TYPE_TASK_STORAGE` map is a privileged map operation in v7.0 and requires CAP_BPF (or an applicable BPF token). With `unprivileged_bpf_disabled=2`, unprivileged BPF map/program creation is blocked unless the relevant capability is delegated. Tracepoint, kprobe and tracing program types are also in the perfmon class and require CAP_PERFMON in addition to the BPF capability path. The user's VM report has CapEff=0, so the current unprivileged process cannot be expected to create this map or load these tracing programs. No syscall/load attempt was made. An administrator-created BPF token may delegate specific commands, map types, program types and attach types; its existence and policy are unknown. The exact Ubuntu 7.0.0-44 source package must be compared with upstream v7.0 before treating these source checks as target-kernel proof.

Do not read task start-time fields merely to create a task ID when Task Storage is used. The task-object association already provides lifetime isolation; task_instance_seq is allocated by the producer. If a HASH fallback is selected, its task-lifetime key must be separately proven against exact target BTF.

## 4. Verifier-oriented stack access

For every push:
- test `overflow_depth != 0` before touching the fixed stack;
- test `depth >= ARRAY_SIZE(stack)` before indexing;
- allocate invocation sequence only after capacity is known;
- write the complete frame before publishing the incremented depth;
- never overwrite an outer frame;
- on overflow or ID-allocation failure, increment overflow depth and suppress both sides of that invocation.

For every pop:
- if overflow depth is nonzero, decrement it and do not touch the tracked stack;
- otherwise require nonzero depth;
- read the top frame, validate subject ID, emit the matching exit only if `emit_enabled`, then clear the frame and decrement depth;
- on mismatch, increment a mismatch counter and mark the capture partial; do not search lower frames.

All dynamic indices must be dominated by explicit bounds checks visible to the verifier. Keep loops bounded/unrolled. Avoid holding a BPF spin lock across helper calls; whether any lock is needed for the selected hook/reentrancy context remains a separate proof obligation.

## 5. Event emission order

The required order is fixed by Navigation 48/60:

1. allocate global event ID using bounded 64-bit CMPXCHG;
2. allocate per-CPU stream sequence;
3. attempt ring-buffer reservation;
4. if reservation fails, increment the reserve-failure counter and mark the invocation frame emit-disabled;
5. otherwise fill the record and submit.

If entry emission fails at any stage, the frame's emit_enabled flag is false and the matching exit is suppressed. If exit emission alone fails, the entry remains unmatched and the strict reducer must report partial. Never fabricate a compensating exit.

## 6. Task-exit hook candidate

A candidate lifecycle observation is `sched_process_exit`, but the exact attachment type and helper availability are not yet proven. The hook must:
- retrieve the exiting current task's Task Storage value;
- count tracked and overflowed syscall/handler frames;
- increment task-exit-with-pending counters;
- subtract abandoned depths from active invocation gauges;
- release the task-context reservation exactly once;
- return before `free_task()` invokes automatic Task Storage cleanup.

If the hook cannot access Task Storage or is not guaranteed to run before cleanup, the design must change; do not infer abandoned depth from map deletion.

## 7. Verifier and runtime gates

Before any load:
- retrieve exact Ubuntu source package metadata and compare Task Storage/helper implementation with upstream v7.0;
- verify helper prototypes and program-type allowlists for the chosen attach type;
- verify GPL-only helper licensing;
- verify Task Storage map creation and BTF value-size checks;
- compile with pinned clang/LLVM and inspect emitted atomic instructions;
- run static BPF object inspection (BTF, map definitions, relocations) before load;
- obtain explicit authorization for a minimal no-clock-change load/attach test;
- prove detach, final counter snapshot, ring-buffer drain and map destruction ordering.

The current live VM report does not satisfy these gates. Pinned upstream v7.0 source references for the privilege/license distinction:
- Task Storage map creation and CAP_BPF check: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/syscall.c#L1449-L1492
- BPF program load and CAP_PERFMON check: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/syscall.c#L2921-L2944
- `bpf_task_storage_get` is not GPL-only: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/bpf_task_storage.c#L239-L248
- `bpf_get_current_task_btf` is GPL-only: https://github.com/torvalds/linux/blob/v7.0/kernel/trace/bpf_trace.c#L766-L776
- BPF token delegation model: https://docs.kernel.org/userspace-api/ebpf/syscall.html

This document intentionally does not specify `SEC("tp/...")`, `SEC("fentry/...")` or `SEC("kprobe/...")` sections because the actual attach points are unresolved.

No BPF source was compiled, no object was loaded, no helper invoked, no map created, no probe attached and no clock changed.
