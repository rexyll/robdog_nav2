# Linux clock adjustment event envelope and userspace reducer contract

- Status: PURE_LOGIC_IMPLEMENTED / GNSS_RUNTIME_CI_PASSED / KERNEL_CAPTURE_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: [42 — event capture design](42_linux_clock_step_event_capture_contract.md)
- Related: [43 — phc2sys source-to-syscall mapping](43_phc2sys_clockadj_source_to_syscall_mapping.md)

## 1. Purpose and boundary

The pure userspace reducer provides a deterministic place to validate synthetic syscall event records before any kernel tracing program is considered. It can pair entry/exit records, preserve request-level timex mode decoding, report visible sequence gaps, and fail closed on malformed or incomplete input.

It does not capture kernel events, load eBPF, read tracefs, prove event-stream coverage, infer a PHC device from a dynamic clock ID, or prove that a requested clock adjustment changed a physical clock.

Implementation:
- Module: src/robodog_gnss_time/robodog_gnss_time/linux_clock_adjustment_events.py
- Tests: src/robodog_gnss_time/test/test_linux_clock_adjustment_events.py
- Public function: reduce_clock_adjustment_events(events, max_pending=4096)

## 2. Input event envelope

Every accepted record must be a mapping with:

| Field | Type | Meaning |
|---|---|---|
| event_type | string | syscall_enter or syscall_exit |
| boot_id | non-empty string | Kernel boot identity partition |
| monotonic_ns | non-negative integer | Boot-relative CLOCK_MONOTONIC timestamp |
| pid_tgid | uint64 integer | TGID/TID composite from the capture source |
| syscall | non-empty string | Observed syscall name; preserves native/compat spelling |
| cpu | non-negative integer | CPU that emitted the record |
| sequence | uint64 integer | Per-CPU event sequence |

Entry-only optional fields:
- raw_clockid: raw syscall clock ID, never interpreted as a PHC device identity;
- modes_read_status: for example ok, user_read_failed, unsupported_abi, or not_requested;
- raw_modes: uint32 value, included only when modes_read_status=ok.

Exit-only required field:
- return_value: signed 64-bit syscall result.

Raw user pointers, struct addresses, command lines, environment variables and unrelated process data are not part of the projected output. Unrecognized input fields are not echoed.

## 3. Correlation identity

The reducer pairs records by the exact tuple:

(boot_id, pid_tgid, syscall)

This deliberately includes:
- boot_id, to prevent records from different kernel boots being joined;
- pid_tgid, to prevent two threads in one process being cross-paired;
- syscall name, to prevent unlike syscalls on one thread being joined.

The reducer does not infer clock target from raw_clockid. It does not collapse native and compat syscall names; the event source must preserve the actual name.

Entry/exit timestamp order is checked using monotonic_ns. An exit earlier than its matching entry is not accepted as a pair.

## 4. Bounded pending entries and malformed input

The default pending capacity is 4096. Callers may supply a smaller positive integer for deterministic tests. A capacity overflow drops the new pending entry from the active map, records it as unpaired with reason inflight_capacity_exceeded, increments a counter, and marks the reduction partial.

A second entry with an identical correlation key before an exit:
- marks the previous pending entry superseded_by_duplicate_entry;
- increments inflight_overwrite;
- stores the new entry as the active candidate;
- marks the reduction partial.

Malformed records are counted and skipped. Invalid values do not get silently coerced to zero. The reducer rejects a bool as an integer for all numeric fields. event_type must be a string before it is checked against the accepted event set; this prevents an unhashable JSON list/object from raising inside the reducer. raw_clockid, when supplied, must fit signed 32-bit clockid_t; modes_read_status must be a bounded string; raw_modes is accepted only as uint32 when the read status is ok. Invalid typed fields reject the record instead of leaking arbitrary nested objects into output.

## 5. Per-CPU sequence accounting

Sequence state is partitioned by (boot_id, cpu). The first observed sequence for a partition establishes its local baseline; no missing sequence is inferred before that first event.

For subsequent records:
- forward gap: records the missing sequence count and marks partial;
- repeated sequence: reports duplicate_per_cpu_sequence;
- sequence regression: reports per_cpu_sequence_regression.

A sequence gap only means that the supplied event stream has a visible counter gap. Its relationship to kernel reserve failures, userspace delivery, and capture completeness depends on how the eventual BPF producer increments and exports its counters. The reducer alone cannot identify where an event was lost.

## 6. Return-value interpretation

The paired record retains raw_return_value and reports only a conservative interpretation:

| Return | Interpretation | Forbidden conclusion |
|---|---|---|
| non-negative | nonnegative_handler_return_not_effect_proof | Do not assume return value must be zero or that physical clock effect has been measured |
| -EFAULT (-14) | negative_efault_copy_stage_ambiguous | Do not assume no effect; copy_to_user can fail after clock handler execution |
| other negative | negative_errno_effect_unresolved | Do not assume no partial hardware/driver side effect without target-specific evidence |

Every pair sets clock_effect=not_measured and clock_target_resolution=not_inferred. The mode decoder is included only when raw_modes was successfully read and passed uint32 validation.

## 7. Output and fail-closed status

The output includes:
- counts for input/accepted/malformed events, entry/exit observations, completed pairs, orphan exits, in-flight overwrite and capacity drops;
- pairs with monotonic duration, raw clockid, mode decode, raw return value and conservative return interpretation;
- unpaired_entries and orphan_exits;
- per-CPU sequence_gaps;
- stable issues with machine-readable issue codes.

reduction_status is consistent only when the supplied records contain no detected reducer issues. Any malformed record, sequence gap/regression, duplicate entry, orphan exit, unmatched entry, capacity drop or invalid pair ordering makes it partial.

Two fields are intentionally fixed:
- capture_coverage_status=not_assessed
- clock_effect_status=not_measured

A consistent reduction is not a complete kernel capture. Empty input is not evidence that no syscall occurred.

## 8. Synthetic tests and remaining work

Tests cover:
1. matching entry/exit;
2. non-negative return interpretation;
3. -EFAULT ambiguity;
4. other negative return values;
5. orphan exit;
6. entry without exit;
7. duplicate entry overwrite;
8. per-CPU sequence gap;
9. per-CPU sequence partitioning;
10. thread identity separation;
11. boot identity separation;
12. malformed record handling and raw-pointer non-echo;
13. failed user-memory modes read;
14. pending-capacity overflow;
15. exit timestamp preceding entry;
16. invalid capacity arguments.

The tests are synthetic pure-Python tests. They do not emulate the Linux scheduler, BPF verifier, tracepoint ABI, kernel/user copy faults, or actual event loss.

Remaining non-Orin work:
- harden event field validation against arbitrary malformed JSON values;
- add multi-process/thread concurrency fixtures and randomized property-style sequences;
- define producer-side per-CPU sequence increment semantics before claiming gap/loss reconciliation;
- add explicit stop/drain and final counter snapshot contract to any future collector.

Target kernel ABI, BTF, eBPF load, permissions, real PHC identity and clock effect remain hardware/BSP gates.
