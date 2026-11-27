# Linux clock-adjustment capture: invocation identity and cross-CPU ordering contract

- Status: DESIGN_PROPOSAL / NOT_IMPLEMENTED / TARGET_KERNEL_UNVERIFIED
- Date: 2026-11-27
- Related: Navigation 42 (capture contract), 44 (syscall reducer), 46 (handler reducer), 47 (syscall-handler joiner)
- Scope: producer-side identity/order requirements; no BPF program or loader is implemented by this document.

## 1. Problem statement

The pure reducers currently use LIFO stacks keyed by boot ID, pid_tgid and syscall/handler name. This preserves ordinary nested same-thread invocations, but LIFO is only a heuristic when records are lost or delivered out of order. Per-CPU sequence numbers cannot establish a total order for a thread that migrates between CPUs.

A production capture producer must provide identity at event creation time. The userspace reducer must not reconstruct invocation ancestry solely from timestamp proximity or arrival order.

## 2. Required identity fields

The current Navigation 50 event-envelope validator is schema v1 and does not yet carry these fields. This document specifies a schema-v2 extension; until that validator and the payload reducers are upgraded, invocation IDs are a design requirement only and must not be fabricated by the userspace pipeline.

Every emitted event must include:

- `boot_id`: boot identity captured once per collection session;
- `pid_tgid`: kernel TGID/TID composite;
- `task_instance_id`: stable task-lifetime discriminator, not merely a reusable PID/TID;
- `event_id`: unique within boot/session, assigned before ring-buffer submission;
- `cpu`: CPU executing the probe;
- `cpu_sequence`: monotonically increasing sequence within the declared producer stream and CPU;
- `monotonic_ns`: `bpf_ktime_get_ns()` timestamp;
- `event_kind`: syscall entry/exit or named handler entry/exit.

Syscall events additionally require `syscall_invocation_id`, assigned once at syscall entry and repeated unchanged at the matching exit. Handler events require `handler_invocation_id`, assigned at handler entry and repeated unchanged at return.

When a handler executes inside a tracked syscall context, it must also carry `parent_syscall_invocation_id`. If the producer cannot prove a parent context, it must emit an explicit state such as `parent_context_status=unavailable`; it must not guess from interval containment.

## 3. Task and invocation identity lifecycle

A proposed task-context map is keyed by pid_tgid and contains:
- task_instance_id;
- next_syscall_invocation_counter;
- bounded active syscall-context stack;
- stack depth and overflow counter.

On syscall entry, the producer allocates a new invocation ID, pushes it, and emits the entry event. On syscall exit, it emits the exit with the top invocation ID and pops the matching context. Nested same-thread syscalls (including signal-handler syscalls) therefore receive distinct IDs while preserving the interrupted outer context.

The task_instance_id must prevent PID/TID reuse from reusing an old invocation namespace. Candidate sources include a verified task start-time field from target BTF or a producer-maintained task lifecycle generation. The exact source is a target-kernel Gate; do not assume a particular `task_struct` field is CO-RE-readable until the target BTF and verifier prove it.

The stack must have a fixed, documented maximum depth. Overflow must not overwrite an outer context: increment a dedicated counter, mark the nested context unavailable, and emit an event with `parent_context_status=stack_overflow`. Task exit cleanup should delete per-task state; missing cleanup must be detected by stale-context accounting rather than silently reused.


## 3A. Task-local storage option (Linux v6.12 source audit)

A bounded per-task context is a stronger candidate than a plain hash keyed only by pid_tgid:

- Linux v6.12 defines `BPF_MAP_TYPE_TASK_STORAGE` and `bpf_task_storage_get()`.
- The task-storage implementation stores local data through the task object and frees it from `free_task()` via `bpf_task_storage_free()`; this avoids stale state being silently inherited by a later task that reuses the same numeric PID/TID.
- The v6.12 task-storage map operations deliberately use a `get_next_key` implementation that returns `-ENOTSUPP`. Userspace cannot enumerate all task-local entries using ordinary map iteration. A final `pending_syscalls=0` claim therefore cannot be derived by dumping the task-storage map.
- The tracing helper dispatcher exposes `bpf_task_storage_get`; the exact helper/program-type/attach combination must still be confirmed against the target kernel and verifier. Helper availability is not proof that a particular program can load.

Potential design: store the fixed-depth syscall context stack and per-task counter in task storage, while maintaining separate producer-wide active/pending counters and explicit task-exit accounting. At task exit, a dedicated lifecycle observation must account for abandoned active invocation depth before task storage is eventually freed. If that observation is unavailable or its counter cannot be reconciled, final capture status is partial. Do not infer zero pending work from automatic memory cleanup.

This is a candidate, not a selected implementation. Compare task storage with a bounded hash map keyed by a task-lifetime token, including memory pressure, map enumeration, task-exit ordering, helper support, and target verifier behavior.

Pinned source references:
- Linux v6.12 task storage implementation: https://github.com/torvalds/linux/blob/v6.12/kernel/bpf/bpf_task_storage.c
- Linux v6.12 task storage release from task freeing: https://github.com/torvalds/linux/blob/v6.12/kernel/fork.c#L609-L615
- Linux v6.12 tracing helper dispatch: https://github.com/torvalds/linux/blob/v6.12/kernel/trace/bpf_trace.c#L1524-L1529
- Helper program-type availability reference: https://docs.ebpf.io/linux/helper-function/bpf_task_storage_get/

## 4. Handler parent context

At handler entry, the producer reads the current task context:
- `parent_syscall_invocation_id` is the top active syscall invocation when present;
- `handler_invocation_id` is allocated independently for every handler entry;
- the handler exit carries the same handler ID and parent syscall ID.

For `clock_adjtime` on the CLOCK_REALTIME callback path, `do_clock_adjtime` and nested `do_adjtimex` events should carry the same parent syscall invocation ID. Direct `adjtimex` should produce a `do_adjtimex` event with the direct syscall invocation ID. A handler invoked without a tracked syscall parent remains valid but is marked unparented.

This identity relationship is stronger than interval containment and allows a one-to-many syscall→handler relation without confusing a nested signal-handler syscall.

## 5. Ordering and loss accounting

Per-CPU sequence numbers remain useful for detecting gaps within each CPU stream, but they do not establish cross-CPU total order. The producer must therefore:
- assign invocation IDs before submitting events;
- include the same invocation ID on entry and exit;
- include parent invocation IDs on handler events;
- retain monotonic timestamps for duration and sanity checks;
- report ring-buffer reservation failures and event submission attempts separately;
- report map insertion/update failures and context-stack overflow separately;
- expose final per-CPU sequence/counter snapshots after probes are detached;
- drain the ring buffer before the userspace consumer closes.

A consumer may sort by monotonic timestamp for presentation, but sorting is not a substitute for invocation identity. If two events have equal timestamps or delivery order differs across CPUs, identity fields still determine pairing.

## 6. Reducer migration policy

The current session envelope schema v1 carries capture/session/stream identity and per-CPU sequence only. It cannot validate or preserve invocation identity. Schema v2 must add the fields in Section 2 before any reducer may claim strict ID-based pairing. The current syscall/handler reducers and joiner remain schema v2 LIFO/interval reducers and must retain their explicit limitations.

The existing schema-v2 reducers remain valid for synthetic fixtures and legacy captures without invocation IDs. They must continue to label their policy as LIFO-without-cookie.

A future schema-v3 reducer should:
1. pair syscall entry/exit by `(boot_id, task_instance_id, syscall_invocation_id)`;
2. pair handler entry/exit by `(boot_id, task_instance_id, handler_invocation_id)`;
3. associate handler to syscall by exact `parent_syscall_invocation_id`;
4. reject duplicate invocation IDs, mismatched parent IDs, orphan exits, and IDs reused within a task lifetime;
5. retain LIFO only as a diagnostic cross-check, never as the primary identity mechanism;
6. propagate producer loss, stack overflow, map failures, and incomplete drain into `reduction_status=partial`.

Do not silently upgrade old captures by fabricating IDs from timestamps or sequence numbers.

## 7. Required producer counters

At minimum:
- `syscall_context_push_failed`;
- `syscall_context_stack_overflow`;
- `syscall_context_pop_mismatch`;
- `task_context_missing`;
- `task_context_cleanup_missing`;
- `handler_parent_context_unavailable`;
- `handler_context_push_failed`;
- `handler_context_stack_overflow`;
- `handler_context_pop_mismatch`;
- `event_id_allocation_failed` (if applicable);
- `ringbuf_reserve_failed`;
- `event_submit_attempted`;
- `userspace_event_received`;
- `userspace_decode_failed`;
- `userspace_write_failed`.

Each counter needs an explicit read status and a final snapshot after detach. A missing counter is unknown, not zero.

## 8. Implementation gates

Before a producer is implemented:
1. Obtain the generic VM live capability report and target Orin report independently.
2. Pin exact kernel release, architecture, BTF identity and source/BSP revision.
3. Confirm target BTF exposes the required task identity fields, or select a lifecycle-generation alternative.
4. Confirm bounded map types, atomic/sequence allocation primitives, verifier support and resource limits.
5. Define syscall ABI normalization for native and compat syscall tracepoint names.
6. Define stop/detach, final counter snapshot and ring-buffer drain ordering.
7. Add synthetic schema-v3 fixtures before any BPF load.
8. Obtain explicit authorization before attaching probes or performing clock-changing tests.

Current status remains design-only. No BPF object is loaded, no probe is attached, no tracepoint is enabled, and no clock is changed.
