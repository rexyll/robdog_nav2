# BPF clock-adjustment producer wire layout and bounded identity allocator pseudocode

- Status: STATIC_DESIGN / NO_BPF_C_IMPLEMENTATION / NO_COMPILATION / NO_VERIFIER_RUN
- Date: 2026-11-27
- Related: Navigation 42, 50, 57, 59
- Scope: fixed-width event ABI, shared counters, per-task invocation stacks and failure accounting.

## 1. Producer-to-userspace ABI

The BPF ring-buffer record must contain only fixed-width scalar fields; never kernel pointers, variable-length strings or raw task_struct addresses. Userspace maps numeric IDs to schema-v2 opaque strings.

Proposed common header:
- `u16 schema_version`;
- `u16 event_type` (syscall_enter, syscall_exit, handler_enter, handler_exit);
- `u32 cpu`;
- `u32 cpu_sequence` (or u64 if the selected producer contract retains u64);
- `u64 event_id`;
- `u64 monotonic_ns`;
- `u64 pid_tgid`;
- `u64 task_instance_seq`;
- `u64 invocation_seq`;
- `u64 parent_syscall_invocation_seq` (zero only when parent status says unavailable);
- `u16 subject_id` (normalized syscall/handler enum);
- `u16 parent_context_status`;
- `u32 raw_clockid` (signed interpretation for clock_adjtime handlers);
- `u32 raw_modes`;
- `s64 return_value`;
- `u32 payload_validity_flags`.

The actual C struct must use explicit alignment and compile-time `static_assert(sizeof(...))` / `offsetof(...)` checks. Signed clockid must be converted to/from a signed 32-bit value without sign loss. `return_value` is signed 64-bit. For entry records, return_value is invalid; for exit records, request-only fields are invalid. A validity bitmask must make those distinctions explicit.

The userspace serializer emits:
- `task_instance_id`: fixed-width hex encoding of task_instance_seq;
- `event_id`: fixed-width hex encoding of event_id;
- `invocation_id`: fixed-width hex encoding of invocation_seq;
- `parent_syscall_invocation_id`: fixed-width hex encoding of parent_syscall_invocation_seq only when parent_context_status=matched;
- `parent_context_status`: mapped to matched/unavailable/stack_overflow/no_active_context.

The same invocation sequence allocator is shared by syscall and handler frames within one task, so invocation_seq is unique for the task lifetime. The task_instance_id remains part of the reducer key. The event ID counter must be shared by all syscall and handler programs through the same BPF map instance; two separately-created counter maps would violate the cross-stream uniqueness contract.

## 2. Shared counter maps

Use one small `BPF_MAP_TYPE_ARRAY` for capture-global counters:
- key 0: next event ID;
- key 1: next task-instance ID;
- key 2: active task-context reservations;
- key 3: event-ID allocator contention/failure count;
- key 4: task-ID allocator contention/failure count.

All producers must reuse the exact same map object/FD (or pinned map identity), not create private copies. The loader must verify the map ID/identity used by each program before attach.

Use separate per-CPU sequence maps for syscall and handler streams, so the sequence scope remains `(capture_session_id, boot_id, producer_id, stream_id, cpu)`. Sequence numbers are local stream-gap evidence only. They do not order records across CPUs.

## 3. Bounded 64-bit allocator

Candidate pseudocode for a map-value counter initialized to 1:

```c
/* Static design only; not compiled or verifier-tested. */
static __always_inline int alloc_u64(__u64 *counter, __u64 *out)
{
#pragma unroll
    for (int attempt = 0; attempt < MAX_CAS_RETRIES; attempt++) {
        __u64 current = READ_ONCE(*counter);
        if (current == 0 || current == U64_MAX)
            return ID_ALLOC_EXHAUSTED;

        __u64 observed = __sync_val_compare_and_swap(
            counter, current, current + 1);

        if (observed == current) {
            *out = current;
            return ID_ALLOC_OK;
        }
    }
    return ID_ALLOC_CONTENTION;
}
```

The bounded loop avoids unbounded verifier paths. A counter at `U64_MAX` fails closed rather than wrapping to 0 and reusing IDs. The exact compiler output must be inspected to ensure it emits the intended 64-bit `BPF_CMPXCHG`; the verifier/JIT must accept it on the target architecture. Atomic instruction availability in upstream v7.0 is source evidence, not target runtime acceptance.

A task-local invocation sequence is incremented only in its task-storage value. Check for 0/U64_MAX before increment; on exhaustion do not wrap. Because invocation IDs are paired with task_instance_id, they need only be unique within that task lifetime.

## 4. Task-context reservation and initialization

The Task Storage map has no map-level max_entries. Enforce a separate `max_tracked_tasks` cap with a bounded CAS reservation on key 2:

1. Look up current task storage without CREATE.
2. If it exists and is initialized, reuse it.
3. If absent, atomically reserve one active task slot only while count < max_tracked_tasks.
4. Call task_storage_get with CREATE and initialize the task_instance_seq and all stack depths.
5. If storage creation fails, release the reservation and increment create_failed.
6. If the cap is reached, increment task_context_capacity_rejected and suppress the affected invocation events; do not substitute pid_tgid as a task identity.
7. On task exit, decrement the reservation exactly once using an initialized/active flag before free_task() destroys storage.

A missed exit hook leaves the task-context reservation conservatively elevated and can cause later capacity rejection. A nonzero reservation at session stop is not itself a failure because live tasks may still own idle storage; the decisive quiescence gauges are active syscall/handler invocations and in-flight callbacks. If an exit hook misses a task that still has pending frames, those invocation gauges remain nonzero and the report is partial.

## 5. Syscall and handler stack state machine

Each task-local value holds:
- a fixed syscall frame array and syscall depth;
- a syscall overflow-depth counter;
- a fixed handler frame array and handler depth;
- a handler overflow-depth counter;
- next invocation sequence;
- task identity and lifecycle flags.

### Syscall entry

1. Resolve/create task context; if unavailable/capacity-rejected, count and suppress.
2. Increment total active syscall gauge.
3. If an overflow depth is already active or the fixed stack is full, increment syscall overflow depth/counter and suppress this invocation's entry/exit event pair. Never overwrite an outer frame.
4. Allocate invocation sequence. On allocator failure, route this invocation through overflow accounting and suppress its event pair.
5. Push frame with invocation sequence, normalized syscall ID, entry timestamp and emit-enabled flag.
6. Emit schema-v2 entry only if ring-buffer reservation, event ID and per-CPU sequence allocation all succeed; otherwise set emit_enabled=false and count the exact failure.

### Syscall exit

1. If syscall overflow depth > 0, decrement it and active syscall gauge; do not pop a tracked outer frame.
2. Otherwise pop the top tracked syscall frame and verify the normalized syscall ID matches.
3. If emit_enabled=true, emit exit with the exact invocation sequence from the frame.
4. Decrement active syscall gauge. A pop mismatch, missing frame or negative gauge is a partial-capture condition.

### Handler entry/exit

Handler frames use a separate fixed stack and the same per-task invocation sequence allocator. At handler entry:
- if syscall overflow depth > 0, parent status is stack_overflow and parent ID is absent;
- else if syscall depth > 0, copy the exact top syscall invocation sequence and mark parent matched;
- else mark no_active_context;
- push the handler frame with its own invocation sequence and the captured parent ID/status.

Handler exit reuses both handler invocation sequence and parent identity from its frame. The exact joiner only associates a handler when parent_context_status=matched. Unavailable parent context remains explicit and makes the strict join partial.

If handler stack capacity is exceeded, use handler overflow-depth accounting and suppress that invocation pair. Never fabricate a handler ID or let the exit consume an outer handler frame.

## 6. Ring-buffer and sequence ordering

For every event attempt, record distinct outcomes:
- ringbuf_reserve_failed;
- event_id_allocation_failed or contention;
- per_cpu_sequence_allocation_failed;
- event_submit_success;
- event_discarded_after_reservation.

Preserve Navigation 48's required sequence ordering: allocate the per-CPU sequence before attempting ring-buffer reservation. The recommended event path is:
1. allocate the global event_id; if it fails, increment event_id_allocation_failed and do not allocate a CPU sequence;
2. allocate/increment the per-CPU sequence;
3. attempt ring-buffer reservation;
4. on reserve failure, increment ringbuf_reserve_failures; the CPU sequence and event ID are both unused;
5. on successful reservation, fill the already-validated fixed record and submit; increment ringbuf_submit_successes.

This preserves Navigation 48's equation: sequence_allocated = ringbuf_reserve_failures + ringbuf_submit_successes. Do not introduce a post-reservation discard path unless a separate terminal counter is added and Navigation 48's schema/equations are revised. Event-ID gaps are not loss proof; reconcile them with event-ID allocation failures and per-CPU reserve/submit counters.

All counter snapshots must be taken after producers quiesce and detach. Ring-buffer drain and userspace queue/writer drain follow; final report checks all counter equations. Event IDs provide uniqueness, not a cross-CPU timestamp order.

## 7. Task exit and stop protocol

A task-exit observation (candidate: a verified sched_process_exit attachment) reads the current task-local depths before free_task() invokes automatic Task Storage cleanup. It increments:
- task_exit_observed;
- task_exit_with_pending_syscalls/handlers;
- abandoned invocation counts.

It then decrements the task-context reservation exactly once. This reservation is a capacity gauge, not a required-zero final gauge. If the hook is unavailable or cannot be verified, the producer cannot claim abandoned-frame accounting for exiting tasks complete.

At stop:
1. set stop_requested and stop admitting new target invocation frames;
2. allow in-flight frames to exit; do not detach while active syscall/handler gauges are nonzero;
3. wait for BPF callback count and active invocation gauges to reach zero;
4. detach all producers;
5. read final counters and per-CPU sequence snapshots;
6. drain ring buffer, consumer queue and writer;
7. flush/close writer and record final snapshot;
8. destroy session maps.

Any stack overflow, allocation failure, event suppression, task-exit abandonment, counter read failure, active invocation remaining or incomplete drain makes the capture partial.

## 8. Static validation checklist

Before compiling or loading:
- check C struct sizes/offsets and enum values against the Python serializer;
- check every event_type's required/invalid payload fields;
- check all IDs are nonzero and never wrap;
- check syscall/handler programs reference the same global ID map;
- check per-CPU sequence maps are stream-specific;
- check stack overflow never overwrites or pops an outer frame;
- check a failed entry emission suppresses its corresponding exit emission;
- check task-exit decrements task reservation exactly once;
- check stop cannot report complete while active invocation gauges are nonzero.

This document is a static design. No BPF C source was added, no object was compiled, no verifier was run, and no probe or map was loaded.
