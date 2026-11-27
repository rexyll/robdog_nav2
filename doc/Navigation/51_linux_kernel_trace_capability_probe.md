# Linux kernel trace capability probe for clock-adjustment evidence

- Status: READ_ONLY_CAPABILITY_INVENTORY_IMPLEMENTED / SYNTHETIC_TESTS_ADDED / LIVE_ATTACH_NOT_TESTED
- Date: 2026-11-27
- Related: Navigation 42, 45–50

## 1. Purpose

The existing `linux_kernel_trace_capabilities.py` inventory already reads kernel config, BTF presence, tracefs mounts, syscall tracepoint availability, event format fields, and event enable state. This increment adds visibility for the two internal handler candidates identified by Navigation 46: `do_clock_adjtime` and `do_adjtimex`.

The probe is strictly read-only. It does not mount tracefs/debugfs, write event controls, enable tracepoints, create kprobe events, load BPF, attach fentry/kprobe programs, or change clock state.

## 2. Added observations

For each conventional tracefs location:
- read `available_filter_functions`;
- report each handler symbol as `listed`, `not_listed`, or `unknown` when the inventory itself is unavailable;
- retain the source's interpretation boundary.

A listed function is only visible in the kernel's function-tracing inventory. It does not establish:
- a BTF `FUNC` record for that exact symbol;
- fentry/fexit attachability or verifier acceptance;
- kprobe registration success;
- symbol stability across vendor kernel builds;
- BPF privilege, lockdown, LSM, or runtime policy permission.

BTF file presence/readability is still only an artifact observation, not a BTF parse or attach test. Kernel config values likewise remain configuration evidence, not runtime proof.

## 3. Candidate selection rules

| Evidence | Permitted conclusion | Not permitted |
|---|---|---|
| syscall event listed and format readable | syscall tracepoint schema is exposed | event can be enabled or captured losslessly |
| handler symbol listed in available_filter_functions | function is visible to function tracing | fentry/kprobe attach will succeed |
| vmlinux BTF file readable | BTF artifact exists and can be opened | target function has usable BTF FUNC/type graph |
| relevant CONFIG_* set to y | kernel config declares feature enabled | feature works under current runtime policy |
| all observations present | candidate is worth a controlled attach test | production readiness |

For the current E2b target, `do_clock_adjtime` and `do_adjtimex` remain separate candidates. Do not assume the vendor Orin kernel exports the same symbols or uses the same wrapper implementation as upstream Linux v6.12.

## 4. Invocation

From a checkout with the ROS package on PYTHONPATH:

```bash
python3 -m robodog_gnss_time.linux_kernel_trace_capabilities \
  --output kernel-trace-capabilities.json
```

For a mounted target filesystem or synthetic fixture:

```bash
python3 -m robodog_gnss_time.linux_kernel_trace_capabilities \
  --root /path/to/rootfs \
  --kernel-release '<target-kernel-release>' \
  --output kernel-trace-capabilities.json
```

The report includes the running Python host's uname metadata even when a fixture root is supplied; a fixture report must not be represented as a live target-host observation.

## 5. Tests and next gates

Synthetic tests cover listed handler symbols, unavailable symbol inventory, and explicit non-attachability interpretation. Existing tests continue to cover syscall tracepoint formats, tracefs mount topology, config sources, BTF readability, and read-only behavior.

Next:
1. collect this report on the generic Linux VM and retain the exact kernel release/config/BTF/tracefs evidence;
2. inspect target-specific BTF for the two internal handler function IDs and argument types using a parser/toolchain pinned to that kernel;
3. perform a separate, explicitly authorized minimal attach test with bounded duration and loss counters;
4. validate syscall/handler event pairing and shutdown drain before any clock operation;
5. repeat on the exact Orin BSP kernel before claiming platform support.

No live host capability report or attach test is supplied by this repository change. No syscall was issued and no CLOCK_REALTIME or PHC clock was modified.
