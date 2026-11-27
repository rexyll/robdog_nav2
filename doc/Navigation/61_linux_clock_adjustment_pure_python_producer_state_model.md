# Pure-Python producer state-machine reference model

- Status: REFERENCE_MODEL_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / NOT_A_BPF_PRODUCER
- Date: 2026-11-27
- Related: Navigation 48, 50, 57, 59, 60

## 1. Purpose and boundary

`linux_clock_adjustment_producer_state.py` is a pure-Python reference model for the proposed task-local syscall/handler stack and event emission ordering. It is intended to make state transitions and failure semantics testable before BPF C exists.

It does not model kernel scheduling, BPF verifier constraints, helper availability, map allocation, ring-buffer implementation, or true concurrent execution. Passing these tests is not evidence that a BPF program compiles, loads, attaches or captures live events.

## 2. Modeled behavior

- One-task transition model plus a session-wide allocator model for multiple task lifetimes, with stable task_instance_id and per-task invocation sequence.
- A fixed-depth syscall stack and independent fixed-depth handler stack.
- Separate overflow-depth counters so an overflowed nested call cannot pop/overwrite an outer tracked frame.
- Handler parent identity copied from the exact top syscall frame at handler entry.
- Global event ID allocation shared by syscall and handler streams.
- Stream-specific per-CPU sequence allocation before ring-buffer reservation, preserving Navigation 48's terminal accounting equation.
- Event-ID allocation failure suppresses sequence allocation; reserve failure consumes a sequence and is accounted as a ring-buffer reserve failure.
- A failed entry emission marks the frame emit-disabled, so its later exit is also suppressed rather than creating an orphan exit.
- Task exit accounts pending frames, releases the task-context reservation, and leaves the producer state eligible for explicit detach only after active invocation gauges are zero.

## 3. Synthetic test coverage

The reference model was added in `linux_clock_adjustment_producer_state.py`; `ClockAdjustmentProducerSessionAllocator` models session-global IDs and multi-task admission separately from the single-task nested stack simulator. Tests exercise:
1. Nested syscall entry/exit pairing by invocation ID.
2. Syscall stack overflow and recovery without outer-frame corruption.
3. Ring-buffer reserve failure and matching exit suppression.
4. Event-ID allocation failure before CPU sequence allocation.
5. Exact handler parent syscall identity and strict join.
6. Nested handler overflow without consuming the outer handler frame.
7. Explicit no-active-parent status.
8. Task exit with pending invocation accounting.
9. Cross-stream event-ID uniqueness.
10. Refusal to detach with active invocations and refusal of duplicate task exit.
11. Two-task interleaving with globally unique event IDs and task-scoped invocation IDs.
12. Task-context capacity rejection, create failure, task-ID allocation failure, CAS contention and 64-bit exhaustion.
13. Interleaved two-task invocation gauges and task-exit cleanup without clearing another task's active invocation; pending depths are attributed to the exiting task before global gauges are decremented.
14. Post-detach allocation, invocation and task-exit mutations are rejected.
13. Global snapshot reconciliation through the producer runtime counter auditor, including live idle Task Storage at detach.

The model's emitted envelopes are also fed into the real schema-v2 invocation reducers and exact parent-ID joiner, so this is a cross-layer synthetic contract test rather than an isolated mock.

## 4. Remaining gaps

- Full multi-task scheduling and concurrent BPF execution are not modeled; the session allocator models interleaved task identities and capacity accounting only.
- Global allocator contention/CMPXCHG retries and integer exhaustion are represented only by injected success/failure flags.
- BPF callback in-flight concurrency and ring-buffer queue/drain races are not modeled.
- Task exit hook ordering relative to `free_task()` is not executed.
- No native/compat syscall tracepoint or fentry/fexit producer exists.

Before BPF implementation, extend the model to multiple task contexts and allocator contention, then translate the tested transition table into fixed-size BPF maps and structs. Keep BPF-specific verifier checks separate.

No BPF object was compiled or loaded; no map or probe was created; no clock was changed.
