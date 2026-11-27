# Linux clock-adjustment post-copy kernel-handler observation reducer

- Status: PURE_LOGIC_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / KERNEL_HOOK_NOT_IMPLEMENTED
- Date: 2026-11-27
- Related: [42 — event capture design](42_linux_clock_step_event_capture_contract.md)
- Related: [44 — syscall event reducer](44_linux_clock_adjustment_event_reducer_contract.md)
- Related: [45 — syscall entry versus post-copy kernel request](45_syscall_tracepoint_vs_postcopy_kernel_request.md)

## 1. Evidence stages

This reducer handles a distinct stream of candidate internal-handler observations:

- E2a: syscall-entry user-memory snapshot, observed before the syscall handler performs its own copy_from_user.
- E2b: normalized kernel-owned request observed at the candidate internal handler entry after copy-in / compat conversion.
- E3a: return value from the internal handler.
- E3b: outer syscall return, reduced separately by Navigation 44.

The reducer only pairs E2b entry and E3a return observations. It deliberately reports `outer_syscall_correlation=not_joined`; it does not guess that an internal handler invocation belongs to a particular outer syscall.

## 2. Event envelope

Common required fields:
- `event_type`: `kernel_handler_enter` or `kernel_handler_exit`;
- `handler`: `do_clock_adjtime` or `do_adjtimex`;
- `boot_id`: non-empty boot identity;
- `pid_tgid`: uint64 TGID/TID composite;
- `monotonic_ns`: non-negative CLOCK_MONOTONIC timestamp;
- `cpu`: non-negative CPU number;
- `sequence`: uint64 sequence scoped to this handler-observation stream and CPU.

Entry fields:
- `raw_modes`: required uint32 copied from the kernel-owned normalized timex structure;
- `raw_clockid`: required signed 32-bit clockid_t for `do_clock_adjtime`;
- `raw_clockid` is omitted for `do_adjtimex`, which is the system adjtimex handler path.

Exit field:
- `return_value`: signed int64 internal handler result.

No user pointer, kernel pointer, struct address, command line, or unrelated process data is accepted into projected output. Unknown input keys are discarded.

## 3. Pairing and bounded state

Pair key is exactly `(boot_id, pid_tgid, handler)`. This prevents pairing across boots, threads, or unlike handler paths. A duplicate entry supersedes the previous pending entry but marks the result partial. Orphan returns, unmatched entries, per-CPU sequence gaps/regressions/duplicates, timestamp inversion, malformed records and capacity overflow are explicit issues. Default pending capacity is 4096.

This reducer expects the handler-observation stream to be supplied in its own sequence domain. Do not feed it a filtered subset of a global per-CPU sequence and then interpret the resulting gaps as loss; the producer must define a dedicated sequence or provide the complete stream to a future unified reducer.

## 4. Return interpretation

- Non-negative handler return: handler result observed; not proof of physical clock effect.
- `-EFAULT`: handler result only; not proof of outer copy-back failure.
- Other negative errno: effect remains unresolved.

Only the combined source-pinned observation of a non-negative internal handler return and an outer syscall `-EFAULT`, under the verified wrapper order documented in Navigation 45, can identify a post-handler copy-back failure. Even then it does not prove a physical clock change.

## 5. Implementation and limitations

Implementation:
- `src/robodog_gnss_time/robodog_gnss_time/linux_clock_adjustment_kernel_observations.py`
- Tests: `src/robodog_gnss_time/test/test_linux_clock_adjustment_kernel_observations.py`
- Public function: `reduce_kernel_clock_adjustment_observations(events, max_pending=4096)`

This is a pure Python reducer for synthetic/future captured records. It does not attach fentry/kprobe, load eBPF, validate target BTF/signatures, capture syscall context, prove event coverage, or measure CLOCK_REALTIME/PHC effect. Target kernel/BSP and Orin evidence remain hardware gates.
