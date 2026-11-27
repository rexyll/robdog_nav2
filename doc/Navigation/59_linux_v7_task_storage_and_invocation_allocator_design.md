# Linux v7.0 BPF task-context storage and invocation-ID allocator design

- Status: DESIGN_PROPOSAL / UPSTREAM_V7.0_SOURCE_AUDITED / UBUNTU_KERNEL_SOURCE_NOT_PINNED / NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: Navigation 50, 57, 58
- Scope: bounded producer-side task context, ID allocation and lifecycle accounting. No BPF object or loader is added.

## 1. Evidence boundary

The uploaded live report identifies the VM kernel as `7.0.0-44-generic`, x86_64, with readable BTF. This document audits upstream Linux tag `v7.0` as a near-version reference. The exact installed Ubuntu source package revision and its patch set have not yet been read from the VM, so none of the upstream implementation details below are claimed as verified against the running Ubuntu binary. Public Noble package metadata lists linux-image-7.0.0-44-generic as version 7.0.0-44.44~24.04.1; the associated buildinfo metadata identifies linux-hwe-7.0 as its source package. This is a strong candidate mapping, not proof of the VM's installed binary package version.

The VM report also shows an unprivileged process with `CapEff=0`, `unprivileged_bpf_disabled=2`, and no observed BPF load attempt. The current report therefore does not establish map creation, helper availability, verifier acceptance or attach permission.

## 2. Linux v7.0 Task Storage facts

The upstream source establishes the following:

1. `BPF_MAP_TYPE_TASK_STORAGE` uses the local-storage allocator. Its map allocation check requires `BPF_F_NO_PREALLOC`, `max_entries=0`, `key_size=sizeof(int)`, nonzero value size, and BTF key/value type IDs. The value size is bounded by `BPF_LOCAL_STORAGE_MAX_VALUE_SIZE`.
2. The task-storage map operations set `map_get_next_key` to `notsupp_get_next_key`, which returns `-ENOTSUPP`. Ordinary userspace map iteration cannot enumerate all task-local entries.
3. `bpf_task_storage_get` accepts a BTF-typed `task_struct *`; the helper prototype identifies the argument as the kernel task BTF type.
4. `free_task()` calls `bpf_task_storage_free(tsk)` before freeing the task structure. Storage lifetime is therefore tied to task lifetime.
5. In `bpf_base_func_proto()`, `BPF_FUNC_task_storage_get` is returned only after the `CAP_BPF` token-capability check. Tracing program helper dispatch delegates to this base helper table.
6. `bpf_get_current_task_btf()` returns the current task as a trusted BTF pointer and is marked GPL-only. A tracing program using this helper must satisfy the GPL-compatible BPF license requirement.

Pinned upstream references:
- Task-storage map ops, non-enumerable `get_next_key`, helper prototype: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/bpf_task_storage.c#L194-L248
- Task storage release from `free_task()`: https://github.com/torvalds/linux/blob/v7.0/kernel/fork.c#L548-L558
- `CAP_BPF` helper gate and task-storage helper dispatch: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/helpers.c#L2051-L2172
- Tracing program delegation to base helper table: https://github.com/torvalds/linux/blob/v7.0/kernel/trace/bpf_trace.c#L1259-L1270
- Current task BTF helper and GPL-only flag: https://github.com/torvalds/linux/blob/v7.0/kernel/trace/bpf_trace.c#L766-L776
- Task-storage allocation requirements and value-size bound: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/bpf_local_storage.c#L788-L801
- Local-storage value-size bound: https://github.com/torvalds/linux/blob/v7.0/include/linux/bpf_local_storage.h#L110-L117
- Hash map iteration implementation: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/hashtab.c#L892-L940 and map ops assignments near lines 2356–2581.

## 3. Candidate comparison

| Property | Task Storage | Bounded HASH keyed by task-lifetime token |
|---|---|---|
| Task lifetime binding | Native task-object lifetime | Must construct and maintain token |
| Automatic free on task destruction | Yes, through `free_task()` | No; explicit exit cleanup required |
| Userspace enumeration | No; `get_next_key` unsupported | Yes; hash map implements `get_next_key` |
| Hard `max_entries` | No; task-storage map requires zero | Yes |
| Value size | Local-storage per-value bound | Hash-map value-size / memory constraints |
| PID reuse isolation | Inherent in task object association | Only if key includes a verified lifetime discriminator |
| Final pending-state audit | Global counters and task-exit accounting; cannot dump all task contexts | Can enumerate remaining entries, but stale entries and cleanup races still need accounting |
| Main risk | No enumeration and no map-level task-count cap | PID/TID reuse, exit cleanup races, key provenance and capacity eviction policy |

### Decision

Task Storage is the preferred first candidate for a process-context syscall/handler stack because it binds the state to the task object and frees it with that task. This is a design preference, not a final implementation selection.

A bounded HASH map remains a fallback if a hard enumerable task-context capacity is mandatory and a reliable task-lifetime key can be verified from the exact target BTF/source. Do not use an LRU hash map for active invocation contexts: silent eviction could destroy a pending entry and make a later exit appear orphaned.

## 4. Proposed task-local value layout

The BPF map value should be a fixed-size, BTF-described structure containing only bounded state:

- `initialized`, `task_instance_seq`, `next_invocation_seq`;
- syscall stack depth and a fixed array of syscall frames;
- handler stack depth and a fixed array of handler frames;
- per-task overflow/mismatch counters;
- a task-context generation/state flag.

A syscall frame contains the normalized syscall ID/name, its invocation sequence, and entry metadata needed to pair the exit. A handler frame contains handler ID/name, its own invocation sequence, the parent syscall invocation sequence copied at entry, and parent-context status.

Do not allocate variable-length arrays or overwrite an outer frame. The maximum syscall and handler depths must be compile-time constants, included in the BPF value-size calculation, and tested at depth 0, max-1, max, and max+1. Select actual depths only after checking expected signal nesting and target workload; no depth is claimed as validated yet.

For the process-context clock-adjustment hooks, one task cannot execute simultaneously on two CPUs. The implementation may therefore evaluate a lock-free task-local update path, but it must still account for nested probe execution and verifier constraints. Do not assume this is safe until a dedicated concurrency/reentrancy test proves the update ordering.

## 5. Identity allocation proposal

The wire format should use fixed-width integer fields in the BPF event struct and serialize them as opaque strings in the userspace schema-v2 envelope:

- `task_instance_id`: a globally allocated per-task lifetime sequence, assigned once when task storage is created;
- `invocation_id`: a per-task monotonically increasing invocation sequence, shared by syscall and handler invocation frames;
- `parent_syscall_invocation_id`: the syscall frame's exact invocation sequence copied into the handler frame at handler entry;
- `event_id`: a capture-session-global event sequence shared by syscall and handler producers.

The userspace serializer may encode these as fixed-width hexadecimal strings. It must not derive them from timestamps, CPU IDs or PID/TID.

### Event ID allocator

Use a one-element global `BPF_MAP_TYPE_ARRAY` counter shared by all event-producing programs. Allocate IDs with a bounded 64-bit compare-and-swap loop; detect counter exhaustion and fail closed. A plain per-CPU counter is insufficient for the schema's cross-stream uniqueness scope `(capture_session_id, boot_id)`.

Allocate `event_id` before ring-buffer submission. A reservation failure therefore creates an unused ID; this is expected and must be separately accounted. Do not interpret event-ID gaps alone as lost records. Preserve per-CPU sequence numbers for local stream-gap detection and reconcile them with allocation, reserve-failure, submit-success and userspace-receive counters.

The task-local `next_invocation_seq` is incremented for each syscall or handler entry. The syscall exit reuses the ID stored in the matching syscall frame. Handler entry copies the top syscall ID as its parent and allocates its own handler ID; handler exit reuses the handler frame ID and parent ID. No userspace timestamp matching is allowed as fallback.


## 5A. Atomic allocator source audit and Ubuntu package provenance

Upstream Linux v7.0 UAPI defines `BPF_ATOMIC`, `BPF_FETCH`, `BPF_XCHG` and `BPF_CMPXCHG`. The v7.0 verifier has explicit atomic RMW validation and accepts `BPF_ADD`, `BPF_ADD | BPF_FETCH`, and `BPF_CMPXCHG` in its atomic-op switch. This supports a design using a 64-bit compare-and-swap loop on a shared ARRAY value, but does not prove that the selected compiler emits the intended instruction, that the target architecture JIT accepts it, or that the program passes verification under the VM policy.

References:
- BPF atomic instruction definitions: https://github.com/torvalds/linux/blob/v7.0/include/uapi/linux/bpf.h#L16-L53
- Verifier atomic RMW validation and opcode dispatch: https://github.com/torvalds/linux/blob/v7.0/kernel/bpf/verifier.c#L8034-L8190

Ubuntu package provenance research:
- The public Noble package listing maps `linux-image-7.0.0-44-generic` to version `7.0.0-44.44~24.04.1`: https://www.ubuntuupdates.org/package/core/noble/main/updates/linux-image-7.0.0-44-generic
- The matching `linux-buildinfo-7.0.0-44-generic` listing identifies source package `linux-hwe-7.0` and the same version: https://www.ubuntuupdates.org/package/core/noble/main/updates/linux-buildinfo-7.0.0-44-generic
- Ubuntu's `linux-source-7.0.0` package is described as Linux 7.0.0 source with Ubuntu patches: https://packages.ubuntu.com/noble/linux-source-7.0.0

To pin the actual VM package metadata, collect this read-only output:
```bash
dpkg-query -W -f='${binary:Package}\t${Version}\t${source:Package}\t${source:Version}\n' \
  "linux-image-$(uname -r)" \
  "linux-modules-$(uname -r)" \
  "linux-headers-$(uname -r)" 2>&1

apt-cache policy linux-source-7.0.0
apt-cache showsrc linux-hwe-7.0 2>&1 | sed -n '1,45p'
```

Do not download or install the source package merely to identify its version. The dpkg metadata should first establish the exact installed binary/source version; only then retrieve the matching source artifact and record its `.dsc` and SHA-256.

## 6. Task Storage lifecycle and bounded capacity

Because Task Storage has no `max_entries` and cannot be enumerated, enforce a producer-wide `max_tracked_tasks` reservation counter before creating a new task-local value:

1. Check whether the current task already has storage.
2. If absent, reserve one slot using an atomic bounded counter.
3. If the configured cap is reached, do not create an untracked context silently; increment `task_context_capacity_rejected` and mark the capture partial.
4. Create task storage and initialize the task instance sequence. If creation fails, release the reserved slot and increment `task_context_create_failed`.
5. On `sched_process_exit`, inspect the exiting task's context before `free_task()` destroys it. Account abandoned syscall/handler stack depth and decrement the active-task gauge exactly once.
6. At session stop, wait for in-flight BPF callbacks and active invocation gauges to reach zero, detach producers, take final counter snapshots, drain ring buffers and userspace queues, then close writers and destroy the maps.

The active-task gauge need not be zero at session stop: tasks may remain alive with idle Task Storage entries. The quiescence requirement is zero active syscall/handler invocations, zero in-flight callbacks, reconciled task-exit accounting, zero ring-buffer/queue work and a complete final snapshot. Destroying the session map releases storage for still-live tasks.

If the task-exit hook misses a task with pending invocations, the active invocation gauges remain nonzero and final capture is partial. A missed idle-task exit can conservatively retain a reservation and cause later capacity rejection; it must not silently overwrite another task's state. If any required counter is unreadable, capacity is exceeded, or any invocation remains active at final quiescence, report `partial`. Do not infer zero pending work from automatic memory cleanup.

## 7. Required counters and failure states

Minimum producer counters:
- `task_context_create_attempted`, `task_context_create_failed`, `task_context_capacity_rejected`;
- `task_context_active`, `task_exit_observed`, `task_exit_with_pending_syscalls`, `task_exit_with_pending_handlers`;
- `syscall_context_stack_overflow`, `syscall_context_pop_mismatch`;
- `handler_context_stack_overflow`, `handler_context_pop_mismatch`;
- `active_syscall_invocations`, `active_handler_invocations`;
- `event_id_allocation_failed`, `invocation_id_allocation_failed`;
- per-CPU sequence allocated, ring-buffer reserve failures, submit successes;
- userspace records received, decode failures and write failures.

Each counter must have an explicit read status and final snapshot after detach. A missing counter is unknown, not zero. Overflow, allocation failure, pop mismatch, abandoned task context or failed drain makes the capture partial.

## 8. Implementation gates

Before implementation:
1. Obtain the exact Ubuntu source package revision for `7.0.0-44-generic` and compare its task-storage/helper code with upstream v7.0.
2. Confirm the BPF token/CAP_BPF policy and GPL-compatible BPF license requirement; the current unprivileged report does not satisfy this gate.
3. Verify helper availability for the chosen tracing program type and attach point; a helper prototype in source is not verifier acceptance.
4. Verify `BPF_MAP_TYPE_TASK_STORAGE` map creation, BTF value layout, per-value size, memory accounting and the configured task cap.
5. Verify task-exit observation ordering relative to `free_task()` and probe detach.
6. Add synthetic tests for nested syscall/handler frames, overflow, exit with pending frames, task ID reuse isolation, event ID exhaustion, ring-buffer reservation failure and cross-CPU event delivery reordering.
7. Only then implement a minimal no-clock-change BPF prototype. Clock-changing tests require separate explicit authorization.

Current status remains design-only. No BPF object was loaded, no helper was called, no map was created, no probe was attached, no tracepoint was enabled, and no clock was changed.
