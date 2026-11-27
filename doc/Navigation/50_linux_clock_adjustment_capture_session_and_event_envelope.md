# Linux clock-adjustment capture session and event envelope

- Status: SCHEMA_V1_COMPATIBLE_AND_V2_VALIDATOR_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / INVOCATION_ID_REDUCER_IMPLEMENTED / LIVE_COLLECTOR_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: Navigation 42, 44, 46–49

## 1. Purpose and boundary

Navigation 50 makes the planned runtime contract machine-checkable without claiming to implement a runtime collector. It defines a versioned event envelope and an exact capture-session lifecycle reducer. Existing syscall and handler reducers remain the payload-specific validators after envelope validation.

No eBPF program, tracepoint/fentry attachment, ring-buffer consumer, worker queue, writer, or runtime orchestration is implemented here.

## 2. Event envelope schema v1

Each record carries:
- `schema_version=1`
- `capture_session_id`, `boot_id`
- `producer_id`, `stream_id`, `stream_kind` (`syscall` or `kernel_handler`)
- `cpu`, `sequence`, `monotonic_ns`
- `payload`: stream-specific event fields

The envelope owns identity, ordering and time fields. Payloads cannot override these reserved fields. The validator rejects unknown schema versions, invalid integer types (including bool), zero sequence, invalid stream kind, and event types that do not match the declared stream kind. It returns a sanitized legacy reducer event plus explicit stream identity; it does not echo arbitrary envelope content.

Sequence scope is `(capture_session_id, boot_id, producer_id, stream_id, cpu)`. Syscall and kernel-handler observations must use separate stream IDs/sequence domains. Do not combine or filter a global sequence and then interpret resulting gaps as producer loss.

## 2A. Event envelope schema v2

Schema v2 is now accepted by the pure validator while schema v1 remains supported as an explicit legacy format.

Every v2 record adds:
- `task_instance_id`: opaque task-lifetime identity string;
- `event_id`: producer-assigned event identity string;
- `invocation_id`: syscall invocation ID on syscall streams, handler invocation ID on handler streams.

Handler-stream records additionally require `parent_context_status`:
- `matched`: requires a non-empty `parent_syscall_invocation_id`;
- `unavailable`, `stack_overflow`, `no_active_context`: parent ID must be absent.

Syscall-stream records reject handler-parent fields. All new fields are envelope-owned and cannot be overridden by payload. The validator preserves the fields in the sanitized event and reports schema version and identity in its result.

The validator checks each record's shape only. The new `reduce_clock_adjustment_invocation_events` reducer performs cross-record checks: event_id uniqueness within session/boot, one entry and one exit per invocation, stable subject/PID, monotonic entry-before-exit, and stable handler parent context. It pairs across CPU migration and arrival reordering by exact invocation ID. Schema v1 remains valid for legacy consumers but is rejected by the strict invocation reducer; no IDs are fabricated for it.

## 3. Capture-session lifecycle

The required ordered lifecycle is:

`created → producers_attached → running → stop_requested → producers_quiesced → ring_buffer_drained → consumer_queue_drained → writer_flushed → writer_closed → finalized`

Every transition carries the same `capture_session_id` and `boot_id`, and a non-decreasing `CLOCK_MONOTONIC` timestamp. Missing, repeated, reordered, identity-changing or time-regressing transitions make the lifecycle partial.

The final snapshot must match the session/boot identity, explicitly state `snapshot_after_writer_close=true`, and report zero for:
- `queue_depth`
- `inflight_callbacks`
- `pending_syscalls`
- `pending_handlers`
- `inflight_writer_records`

This is deliberately stricter than an orderly process exit: a writer close without producer quiescence and all drain stages is not a complete capture.

## 4. Runtime counter ownership

The runtime implementation should expose cumulative monotonic counters, scoped to session and boot identity:

| Owner | Required counters |
|---|---|
| Kernel producer, per producer/CPU | sequence allocated, ring-buffer reserve failures, submit successes |
| Consumer | records read, decode successes, decode failures |
| Bounded queue | enqueued, admission drops, current depth |
| Writer | records written, write failures, in-flight records |
| Lifecycle/dispatcher | in-flight callbacks, pending syscalls, pending handlers |

The exact reconciliation equations and terminal outcome rules remain in Navigation 48–49. Counter snapshots must be captured before start and after final drain; counters must not reset within a session. If retry, quarantine, filtering, or another terminal path is added, give it a separate counter and revise the schema/contract.

## 5. Reducer integration

When both `syscall_envelopes` and `handler_envelopes` are supplied to the unified report builder, it runs strict invocation reducers for each stream and an exact parent-ID join. The report adds three mandatory gates: syscall invocation reduction, handler invocation reduction, and invocation parent join. Supplying only one stream or a non-list value fails closed. Omitting both keeps the report in explicit legacy mode and does not claim schema-v2 verification.

The exact join key is `(capture_session_id, boot_id, task_instance_id, parent_syscall_invocation_id)`. It additionally checks PID equality, supported syscall/handler mapping and interval containment; a syscall may own multiple handler observations. It never falls back to nearest timestamp when parent identity is missing or ambiguous.

## 5A. Legacy and strict paths

The strict invocation reducer is an additional schema-v2 path; it does not replace the schema-v1 LIFO reducers yet. Its pairing key includes session, boot, producer, stream, task lifetime and invocation ID. It reports duplicate event IDs, duplicate entry/exit sides, unmatched sides, subject mismatches, parent-context mismatches and timestamp reversals as partial.

For each envelope:
1. validate schema and session/stream identity;
2. route `payload` to the syscall or kernel-handler reducer selected by `stream_kind`;
3. preserve independent sequence domains;
4. feed producer counters and userspace counters into Navigation 48–49 audits;
5. run this lifecycle audit on the ordered transition log and final snapshot; the unified report exposes it as a mandatory eighth gate.

The validator does not itself prove that a producer emitted all events, that a consumer read every submitted record, or that a writer persisted bytes durably. Those remain separate counter and storage-integrity contracts.

## 6. Tests and remaining work

Synthetic tests cover v1 compatibility, v2 identity projection, handler parent-context states, schema/version/type rejection, payload attempts to override envelope fields, stream-kind/event-type mismatch, malformed unhashable values, lifecycle ordering, identity continuity, monotonic-time regression, and final quiescence.

Remaining implementation gates:
- migrate existing syscall/handler payload reducers to consume strict invocation pairs without losing legacy report fields;
- generate producer-side invocation IDs and parent IDs from the target-kernel context stack;
- define the actual eBPF/perf/ring-buffer transport and target-kernel attachment points;
- implement producer counters and a dedicated unfiltered stream for each event class;
- implement consumer decode/queue/writer counters and durable flush semantics;
- integrate the seven Navigation 49 gates with lifecycle status into one top-level capture report;
- run a generic Linux VM capture and verify actual event delivery/drain;
- pin and validate the target Orin BSP kernel/BTF before enabling target hooks;
- independently measure CLOCK_REALTIME and PHC effects (E5).

No clock-setting syscall was issued and no clock was modified during these tests.
