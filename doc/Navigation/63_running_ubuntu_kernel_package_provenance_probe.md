# Running Ubuntu kernel package provenance probe

- Status: READ_ONLY_TOOL_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / VM_NOT_YET_RUN
- Date: 2026-11-27
- Related: Navigation 58, 59, 62
- Scope: identify installed binary/source package metadata for the current kernel; no package installation, apt update, source download, BPF operation or privileged access.

## 1. Purpose

The live capability report identifies the running kernel release as `7.0.0-44-generic`, but it does not contain the installed dpkg binary package version or source package version. Public Ubuntu package metadata suggests `linux-image-7.0.0-44-generic` version `7.0.0-44.44~24.04.1` from source package `linux-hwe-7.0`; this is only a candidate until the VM's installed package database confirms it.

The probe records:
- kernel release and architecture;
- installed candidate kernel image/modules/headers/buildinfo packages;
- dpkg binary package version, source package, source version and status;
- dpkg owners for `/boot/vmlinuz-<release>`, `/boot/config-<release>` and `/usr/lib/modules/<release>/vmlinuz`.

## 2. Invocation

```bash
python3 src/robodog_gnss_time/robodog_gnss_time/linux_kernel_package_provenance.py \
  --output "$HOME/kernel-package-provenance.json"
```

No sudo is required. The script only invokes `uname` and `dpkg-query`. It does not invoke apt, read package indexes, install/remove packages, change kernel configuration, mount tracefs, load BPF or modify clocks.

## 3. Completeness rules

The report is `observed` only if:
1. at least one candidate kernel package is installed;
2. dpkg reports an owner for the running kernel image path;
3. at least one installed package supplies a source package and source version;
4. installed candidate packages do not disagree on source package/version identity.

Otherwise the result is `partial`. A partial result is useful evidence; it must not be filled using a public package candidate.

## 4. Interpretation boundary

The source package identity enables the next audit to fetch the exact Ubuntu source package and compare:
- Task Storage map allocation constraints;
- `bpf_task_storage_get` helper registration and CAP_BPF gate;
- `bpf_get_current_task_btf` GPL-only flag;
- BPF program-type capability checks (CAP_BPF/CAP_PERFMON);
- task-exit cleanup ordering;
- BTF and fentry/fexit implementation details.

The package metadata alone does not prove that the running kernel binary is byte-identical to the source package build, nor does it prove verifier acceptance or attachability.

No BPF program was compiled or loaded; no map or probe was created.
