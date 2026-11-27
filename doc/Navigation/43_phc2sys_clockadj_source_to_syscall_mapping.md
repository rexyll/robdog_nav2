# phc2sys clock adjustment to Linux syscall source mapping

- Status: SOURCE_AUDITED / PURE_MODE_DECODER_IMPLEMENTED / TARGET_KERNEL_NOT_VERIFIED
- Date: 2026-11-27
- Related contract: [42 — Linux CLOCK_REALTIME / PHC adjustment event capture](42_linux_clock_step_event_capture_contract.md)
- Related host evidence: [41 — LinuxPTP PHC / host clock evidence](41_linuxptp_phc_host_clock_evidence_contract.md)

## 1. Scope and pinned references

This audit maps the upstream LinuxPTP phc2sys implementation to the Linux clock-adjustment syscall interface. It is a source-level contract, not evidence that a particular Orin kernel, PHC driver, or process configuration uses the same code.

Pinned LinuxPTP snapshot:
- Repository: https://github.com/richardcochran/linuxptp
- Commit: 227bbd4715bf4cb735be4947e987032f41457585 (2026-08-09)
- phc2sys.c: https://github.com/richardcochran/linuxptp/blob/227bbd4715bf4cb735be4947e987032f41457585/phc2sys.c
- clockadj.c: https://github.com/richardcochran/linuxptp/blob/227bbd4715bf4cb735be4947e987032f41457585/clockadj.c
- clockadj.h: https://github.com/richardcochran/linuxptp/blob/227bbd4715bf4cb735be4947e987032f41457585/clockadj.h

Pinned generic Linux reference:
- Repository: https://github.com/torvalds/linux
- Tag: v6.12
- UAPI timex mode definitions: https://github.com/torvalds/linux/blob/v6.12/include/uapi/linux/timex.h
- POSIX clock syscall dispatch: https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-timers.c
- Dynamic POSIX clock dispatch: https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-clock.c
- PTP clock implementation: https://github.com/torvalds/linux/blob/v6.12/drivers/ptp/ptp_clock.c
- System clock adjustment implementation: https://github.com/torvalds/linux/blob/v6.12/kernel/time/ntp.c
- adjtimex syscall implementation: https://github.com/torvalds/linux/blob/v6.12/kernel/time/time.c

Linux v6.12 is a reviewable reference snapshot only. It is not a claim about the currently deployed or future Orin BSP kernel. The target kernel and driver source remain unpinned until hardware/BSP selection.

## 2. phc2sys call-chain findings

### 2.1 Startup and read-only clock queries

In the pinned phc2sys.c, servo_add() calls clockadj_init(), then clockadj_get_freq(). The latter zero-initializes struct timex and calls clock_adjtime(clkid, &tx) with modes=0; this is a read/query path, not a frequency adjustment request.

For a system clock, servo_add() also calls sysclk_set_leap(0), which uses ADJ_STATUS, and clockadj_max_freq(), which also uses zero modes to query the clock's frequency tolerance. For a PHC, the maximum adjustment is obtained through phc_max_adj(), not by assuming that all system-clock modes exist on the PHC.

Source locations:
- phc2sys.c: servo_add(), approximately lines 133–166.
- clockadj.c: clockadj_get_freq(), lines 78–91; clockadj_max_freq(), lines 130–160.

### 2.2 Servo jump and frequency steering

The pinned update_clock() has the critical sequence:

1. On SERVO_JUMP, call clockadj_step(clock->clkid, -offset).
2. If the step request succeeds, execution falls through to the SERVO_LOCKED / SERVO_LOCKED_STABLE cases.
3. Those cases call clockadj_set_freq(clock->clkid, -ppb).
4. If the target clock is CLOCK_REALTIME, call sysclk_set_sync().

Consequently one servo update may generate more than one clock_adjtime syscall: a direct time-offset request followed by a frequency request, and possibly a system-clock status/maxerror request. A syscall event stream must not assume one adjustment syscall per servo sample.

Exact source:
- phc2sys.c update_clock(): https://github.com/richardcochran/linuxptp/blob/227bbd4715bf4cb735be4947e987032f41457585/phc2sys.c#L625-L680
- clockadj_step() and clockadj_set_freq(): https://github.com/richardcochran/linuxptp/blob/227bbd4715bf4cb735be4947e987032f41457585/clockadj.c#L54-L129

### 2.3 Other clock_adjtime requests

The pinned clockadj.c contains these request builders:

| Helper | timex.modes | Request-level meaning | Important boundary |
|---|---:|---|---|
| clockadj_step() | ADJ_SETOFFSET | ADJ_NANO = 0x2100 | Add a signed time offset at nanosecond resolution | Direct offset/step request; does not itself prove successful application |
| clockadj_set_freq() | ADJ_FREQUENCY plus conditional ADJ_TICK | Frequency steering request, optionally tick setting | ADJ_TICK is included for system clocks when nominal tick information is available |
| clockadj_get_freq() | 0 | Read/query | Must not be counted as frequency mutation |
| clockadj_max_freq() | 0 | Read/query | Must not be counted as frequency mutation |
| clockadj_set_phase() | ADJ_OFFSET | ADJ_NANO = 0x2001 | Offset/phase adjustment request | Helper exists in clockadj.c; no call site was found in the pinned phc2sys.c |
| sysclk_set_leap() | ADJ_STATUS = 0x0010 | System clock status / leap-state request | Not a time step |
| sysclk_set_tai_offset() | ADJ_TAI = 0x0080 | TAI offset request | Not a UTC wall-clock step |
| sysclk_set_sync() | ADJ_STATUS | ADJ_MAXERROR = 0x0014 | Status and maximum-error metadata request | Not a time step |

The helper clockadj_set_phase() must not be attributed to the pinned phc2sys runtime path merely because it is present in the linked helper file. Future LinuxPTP versions or other binaries may call it; their source must be audited separately.

Leap and TAI paths:
- phc2sys.c calls sysclk_set_leap() only for CLOCK_REALTIME when kernel leap handling is enabled.
- phc2sys.c calls sysclk_set_tai_offset() for CLOCK_REALTIME when the PTP UTC offset is traceable and changes.
- These events are important time-scale metadata, but neither is equivalent to ADJ_SETOFFSET.

## 3. Linux clock-target dispatch is material

The syscall name clock_adjtime plus a modes bitmask is not enough to infer the effective clock operation.

In the Linux v6.12 reference source:

1. do_clock_adjtime(which_clock, ktx) resolves a k_clock implementation from which_clock and calls that implementation's .clock_adj callback.
2. The CLOCK_REALTIME k_clock callback routes to do_adjtimex(), which runs the system NTP/timekeeping adjustment path.
3. A dynamic POSIX clock ID derived from a file descriptor is dispatched through kernel/time/posix-clock.c to the registered dynamic clock's .clock_adjtime operation.
4. The PTP clock implementation in drivers/ptp/ptp_clock.c has its own mode dispatch. In the v6.12 reference it checks ADJ_SETOFFSET, then ADJ_FREQUENCY, then ADJ_OFFSET, and accepts modes=0 for a query. Its offset operation can call the PHC driver's .adjphase callback; the frequency and direct offset paths call driver callbacks.

Consequences:
- ADJ_OFFSET must be called an offset/phase request, not universally a gradual slew. The system-clock NTP implementation and a PHC driver's phase callback are different paths.
- ADJ_TICK may have meaning in the system-clock path but is not an independent PHC callback in the v6.12 PTP dispatch.
- Status, maxerror, TAI, and other modes must not be assumed to have the same effect on a dynamic PHC as on CLOCK_REALTIME.
- Dynamic clock ID alone is not a verified /dev/ptpN identity. It remains unresolved unless fd lifecycle evidence establishes the mapping.

Relevant source:
- posix-timers.c: posix_clock_realtime_adj(), do_clock_adjtime(), and SYSCALL_DEFINE2(clock_adjtime).
- posix-clock.c: dynamic clock fd resolution and clock_adjtime operation dispatch.
- ptp_clock.c: ptp_clock_adjtime() mode-specific callback dispatch.
- ntp.c: __do_adjtimex() and process_adjtimex_modes().

## 4. Syscall return value is not a side-effect oracle

The Linux v6.12 clock_adjtime syscall wrapper performs the following sequence:

1. copy_from_user() the timex request;
2. call do_clock_adjtime(), which may modify the clock;
3. if the handler returned nonnegative, copy_to_user() the updated timex result;
4. return the handler value, or -EFAULT if the result copy fails.

Therefore -EFAULT is ambiguous: it can occur before dispatch while copying the request, or after the clock handler has already executed while copying the response back. A syscall exit tracepoint records the final return value but does not identify which copy operation faulted.

The adjtimex wrapper follows the same broad copy-in → execute → copy-out pattern. Thus:
- a nonnegative result is a nonnegative handler/status result, not necessarily literal zero;
- system-clock do_adjtimex may return a TIME_* status code;
- a negative return is an error observation, but -EFAULT does not prove that no side effect happened;
- syscall exit alone does not prove the physical PHC or CLOCK_REALTIME value changed by the requested amount.

The event model must retain the raw return value. Use an interpretation such as nonnegative_handler_return, negative_errno_return_effect_ambiguous, or no_exit_observed; do not use a boolean applied.

## 5. Linux UAPI mode decoder

A pure Python decoder is added at:
- src/robodog_gnss_time/robodog_gnss_time/linux_timex_modes.py
- src/robodog_gnss_time/test/test_linux_timex_modes.py

It uses the values pinned in Linux v6.12 include/uapi/linux/timex.h and:
- accepts only a uint32 integer (rejecting bool, negative and out-of-range values);
- handles the exact special values ADJ_OFFSET_SINGLESHOT=0x8001 and ADJ_OFFSET_SS_READ=0xa001 before treating values as a bitmask;
- separates operation flags from ADJ_MICRO / ADJ_NANO resolution modifiers;
- preserves unknown mode bits;
- reports modes=0 as a read-only query;
- labels ADJ_SETOFFSET as a direct time-offset request and ADJ_OFFSET as an offset-adjustment request without claiming a universally identical slew behavior;
- never infers the target clock, driver callback, syscall result, or clock effect.

The decoder is suitable for deterministic unit tests and future userspace event reduction. It does not read user memory, depend on a target UAPI struct layout, attach eBPF, or interact with any clock device.

## 6. Event-capture implications

The eventual collector should store these as separate fields:

- syscall_name, entry/exit, raw_clockid, raw_modes, mode_decode;
- raw_return_value and its conservative interpretation;
- clock_target_resolution (CLOCK_REALTIME, verified PHC identity, dynamic/unresolved, or other);
- event_timestamp_monotonic_ns, boot_id, pid, tid;
- request user-memory read status, without persisting raw user pointers.

A complete event should still not set clock_effect=applied. Independent PHC/system clock observations are a separate evidence plane.

## 7. Remaining non-Orin work and hardware gates

Can continue without Orin:
- extend the pure request-mode decoder with additional upstream-defined special cases only when verified against the pinned UAPI;
- implement a generic userspace event-envelope validator and entry/exit reducer using synthetic records;
- test mode combinations, unknown bits, status-code returns, EFAULT ambiguity, duplicate/orphan exits, event sequence gaps, and malformed input;
- compare additional stable Linux kernel reference tags without claiming target compatibility.

Still requires a target device/BSP:
- actual Orin kernel release and ARM64 syscall/compat tracepoint formats;
- whether compat32 execution and corresponding tracepoints are present/enabled;
- actual BTF/helper/verifier/permission capabilities;
- PHC device and driver callback semantics, firmware, NIC timestamping and phc2sys service configuration;
- real syscall event capture, loss behavior under target load, PHC identity and independent clock-effect correlation.

No Orin report is assumed or fabricated. No eBPF program has been loaded, no tracing state has been changed, and no system clock/PHC has been adjusted by this project work.

## 8. Upstream source anchors

- LinuxPTP pinned source commit:
  https://github.com/richardcochran/linuxptp/commit/227bbd4715bf4cb735be4947e987032f41457585
- Linux v6.12 UAPI timex definitions:
  https://github.com/torvalds/linux/blob/v6.12/include/uapi/linux/timex.h
- Linux v6.12 clock_adjtime dispatch and copy-in/copy-out:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-timers.c
- Linux v6.12 dynamic clock fd dispatch:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/posix-clock.c
- Linux v6.12 PTP clock implementation:
  https://github.com/torvalds/linux/blob/v6.12/drivers/ptp/ptp_clock.c
- Linux v6.12 system time adjustment:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/ntp.c
- Linux v6.12 adjtimex syscall wrapper:
  https://github.com/torvalds/linux/blob/v6.12/kernel/time/time.c
- Linux man-pages clock_adjtime/adjtimex mode semantics:
  https://man7.org/linux/man-pages/man2/clock_adjtime.2.html

---
This document is a pinned source audit and request-level contract. It is not a deployment baseline or hardware validation report.
