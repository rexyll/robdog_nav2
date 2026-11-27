# Linux clock-adjustment capture integrity and shutdown drain contract

- Status: PURE_REDUCER_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / PRODUCER_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: Navigation 42, 44, 46 and 47

## 1. Purpose

This contract audits whether a dedicated, unfiltered per-CPU event stream reconciles with producer-side counters and whether shutdown completed all declared drain phases. It is not a kernel collector.

## 2. Producer accounting contract

For each `(producer_id, boot_id, cpu)`, the producer must:
1. Increment `sequence_allocated` before attempting ring-buffer reservation.
2. For every allocated sequence, increment exactly one terminal counter: `ringbuf_reserve_failures` or `ringbuf_submit_successes`.
3. Put the allocated sequence into every successfully submitted event.
4. Never reset counters during a capture session; use a new boot ID/session identity after restart.
5. Expose an initial snapshot before capture and a final snapshot only after producer quiescence and all buffers are drained.

Therefore, between snapshots:

`sequence_allocated_delta = ringbuf_reserve_failures_delta + ringbuf_submit_successes_delta`

The consumer input must be a dedicated, unfiltered stream. If records are filtered, sampled, or merged from unrelated streams, this reconciliation is invalid and must not be used.


## 2A. Global allocator and task-context snapshots (schema v2)

In addition to the existing per-CPU sequence snapshots, a producer may include a `global` object in both initial and final snapshots. The object contains identical `capture_session_id` / `boot_id`, cumulative `counters`, and `gauges`. The final global snapshot must declare `snapshot_after_producer_detach=true`.

Required cumulative counters:
- event ID allocation attempts/successes/failures;
- task-instance ID allocation attempts/successes/failures;
- task-context admission attempts/admitted/capacity-rejected;
- task-context create attempts/successes/failures;
- task-context initialization attempts/successes/failures;
- syscall and handler invocation-ID allocation attempts/successes/failures;
- CPU sequence allocation failures;
- syscall/handler stack overflow and pop mismatch;
- handler parent-context unavailable;
- task exits observed and task exits with pending syscall/handler frames.

Required gauges:
- `active_syscall_invocations`;
- `active_handler_invocations`;
- `inflight_bpf_callbacks`;
- `task_context_reservations`.

The global audit checks:
- each allocation/admission/create/init attempt equation;
- task-context init attempts equal successful storage creates and task-instance ID allocation attempts;
- task-context reservation delta equals admitted reservations minus create failures, init failures and observed task exits;
- all counters are monotonic and session/boot identity is stable;
- initial active gauges are zero;
- final active syscall/handler invocation and in-flight callback gauges are zero;
- final snapshot was taken after producer detach.

A nonzero final `task_context_reservations` is allowed: live tasks may still own idle Task Storage entries. The session map is destroyed after detach/drain. This gauge is a capacity reservation count, not a final quiescence requirement.

Any allocator failure, capacity rejection, stack overflow, pop mismatch, unavailable handler parent, task exit with pending frames, unreadable counter, equation mismatch, or nonzero final invocation/callback gauge makes runtime integrity partial. Task exit observations are not compared with total created contexts as a zero-sum equation because live tasks may remain at session end.

## 3. Consumer reconciliation

Each received record carries only the identity envelope needed here: `producer_id`, `boot_id`, `cpu`, `sequence`. Sequences are one-based and scoped to producer + boot + CPU.

The reducer checks:
- initial/final per-CPU snapshot identity and monotonic counter deltas;
- counter equation above;
- duplicate and out-of-window event sequences;
- received record count versus producer submitted count;
- missing sequence count versus producer ring-buffer reserve failures.

A reserve failure should create an explainable sequence hole because sequence allocation precedes reserve. Any mismatch, unreadable counter, counter regression, missing snapshot or malformed event makes the result partial. This contract deliberately does not guess that every missing event is a ring-buffer reserve failure.

## 4. Stop and drain protocol

The final snapshot is valid only after all phases are true:
1. `stop_requested`: controller requested capture stop.
2. `producer_quiesced`: no producer can allocate or submit additional records.
3. `ring_buffer_drained`: all successfully submitted kernel ring-buffer records were consumed.
4. `consumer_queue_drained`: userspace work queue is empty and no callbacks remain in flight.
5. `writer_flushed`: output writer flushed all accepted records.
6. `writer_closed`: writer closed successfully.
7. `final_snapshot_after_drain`: counters were sampled after the above phases.

At final snapshot, `queue_depth`, `inflight_callbacks`, `pending_syscalls`, and `pending_handlers` must all be zero. Nonzero pending syscall/handler pairs mean the evidence stream ended mid-operation; do not silently discard them or call capture complete.

The controller should first disable new event production, then wait for producer quiescence, drain the kernel ring buffer, drain callback/consumer queues, reconcile pending reducers, flush and close output, and finally read immutable producer/loss counters. A timeout or failed phase leaves status partial.

## 5. Output semantics

`integrity_status=complete` means only that the supplied snapshots, dedicated event records, and drain declarations reconcile under this contract. It does not prove the producer counters are truthful, that instrumentation covered every relevant kernel path, or that a clock changed. Capture coverage and physical clock effect remain independent evidence gates.

The reducer never echoes raw pointers or arbitrary event payloads. When the optional global schema-v2 snapshot is present, it also returns a separate global runtime audit; absence of both global snapshots preserves the legacy per-CPU-only mode, while presence of only one is partial.

## 6. Validation

Synthetic tests cover clean stream, explained and unexplained sequence gaps, reserve/submitted counter equation, missing consumer records, out-of-window/duplicate sequence, per-CPU isolation, counter regression, all drain phases, nonzero outstanding work, malformed input and missing snapshots.

No eBPF is loaded, no tracepoint is enabled, no kernel event is captured, and no clock/PHC is modified.
