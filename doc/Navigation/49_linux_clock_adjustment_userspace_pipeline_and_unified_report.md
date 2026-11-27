# Linux clock-adjustment userspace pipeline accounting and unified report

- Status: PURE_REDUCERS_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / LIVE_COLLECTOR_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: Navigation 44–48

## 1. Scope

Navigation 48 reconciles producer-side ring-buffer sequence accounting with records delivered to a dedicated userspace stream. This document extends the audit across decode, bounded queue admission, and output writer stages, then composes the existing syscall, post-copy handler, joiner, producer-integrity, and userspace-pipeline reducers into one offline report.

This is an offline evidence reducer. It does not implement eBPF, tracepoint/fentry attachment, a live consumer, or a writer.

## 2. Snapshot identity and counters

Both pipeline snapshots must carry identical non-empty `capture_session_id` and `boot_id`. All counters are cumulative, nonnegative integers, monotonic within that session, and must not reset between snapshots.

Required counters:
- `consumer_records_read`
- `decode_successes`, `decode_failures`
- `queue_enqueued`, `queue_admission_drops`
- `writer_records_written`, `writer_write_failures`

The final snapshot additionally requires `snapshot_after_writer_close=true`, `final_queue_depth=0`, and `final_inflight_writer_records=0`.

## 3. Stage accounting equations

For the deltas between initial and final snapshots:

`consumer_records_read = decode_successes + decode_failures`

`decode_successes = queue_enqueued + queue_admission_drops`

`queue_enqueued = writer_records_written + writer_write_failures`

Queue admission drops and writer failures are separate terminal outcomes. No implicit “other” sink is permitted: if quarantine, filtering, retry exhaustion, or another terminal path is introduced, it must have its own counter and a revised versioned contract. Writer failure means the record was consumed from the queue but not successfully written; it is never counted as written.

The final zero queue/in-flight gauges are required so the last equation cannot hide outstanding work.

## 4. Unified offline report

`build_clock_adjustment_capture_report()` runs:
1. producer sequence/counter and shutdown-drain audit (Navigation 48);
2. userspace pipeline stage accounting;
3. E1/E2a/E3b syscall reducer (Navigation 44);
4. E2b/E3a kernel-handler reducer (Navigation 46);
5. conservative syscall/handler interval join (Navigation 47).

Each gate remains separately visible. Producer global counters are nested in the Navigation 48 initial/final snapshots; the unified report exposes the global runtime audit independently from per-CPU ring-buffer integrity. The unified report also requires producer `received_records` to equal pipeline `consumer_records_read`, and requires the pipeline boot ID to equal the producer snapshot boot-ID set. An eighth mandatory gate audits the ordered capture lifecycle and final quiescence snapshot (Navigation 50). When global schema-v2 producer counters are supplied, an additional `producer_runtime_counters` gate checks allocator equations, task-context reservation lifecycle and zero active invocation/callback gauges. A separate `producer_loss_free` gate requires at least one audited per-CPU stream and zero known ring-buffer reserve failures; an accounting-complete stream with explained drops is still not accepted as loss-free. Strict schema-v2 invocation mode requires the global producer runtime snapshot gate. When schema-v2 invocation streams are supplied, additional gates check syscall invocation reduction, handler invocation reduction, exact parent-ID join and cross-stream event-ID uniqueness. Overall status is `consistent` only if every gate applicable to the supplied evidence passes; omitted optional evidence is reported as not supplied, not as a pass. An empty per-CPU snapshot set cannot pass producer integrity. The report does not collapse missing evidence into a single “clock adjustment succeeded” claim.

## 5. Evidence and limitations

- A pipeline-consistent result only proves supplied counters reconcile; it does not prove counter honesty or semantic validity.
- Producer completeness still depends on Navigation 48's dedicated unfiltered stream and producer accounting contract.
- The syscall/handler join still proves interval containment only, not call-stack ancestry.
- No gate proves instrumentation covers every relevant kernel path.
- No report proves CLOCK_REALTIME or PHC changed; independent E5 measurement remains required.
- No live collector, kernel hook, target Orin/BSP validation, or hardware capture is included.

## 6. Synthetic validation

Tests cover all three stage equations, distinction between queue drops and writer failures, session/boot identity changes, counter regression, outstanding queue/in-flight work, writer-close snapshot ordering, invalid input, unified-report fail-closed behavior, and producer-to-userspace record-count/boot-identity cross-checks.
