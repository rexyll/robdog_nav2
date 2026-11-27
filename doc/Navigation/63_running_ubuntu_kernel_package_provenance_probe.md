# Running Ubuntu kernel package provenance probe

- Status: READ_ONLY_TOOL_IMPLEMENTED / SYNTHETIC_TESTS_SUBMITTED / VM_NOT_YET_RUN
- Date: 2026-11-27
- Related: Navigation 58, 59, 62
- Scope: identify installed binary/source package metadata for the current kernel; no package installation, apt update, source download, BPF operation or privileged access.

## 1. Purpose

The live capability report identifies the running kernel release as `7.0.0-44-generic`, but it does not contain the installed dpkg binary package version or source package version. Public Ubuntu package metadata shows a distinction: the signed binary `linux-image-7.0.0-44-generic` at version `7.0.0-44.44~24.04.1` is associated with packaging source `linux-signed-hwe-7.0`, while kernel modules/headers/buildinfo are published from `linux-hwe-7.0` at the same version. These are different source package identities with different roles; the signed image source must not be mistaken for the kernel code source. Both remain candidates until the VM's installed dpkg database confirms its exact packages.

The probe records:
- kernel release and architecture;
- installed candidate kernel image/modules/headers/buildinfo packages;
- dpkg binary package version, source package, source version and status;
- dpkg owners for `/boot/vmlinuz-<release>`, `/boot/config-<release>` and `/usr/lib/modules/<release>/vmlinuz`.
- separate signed-image source identities from kernel source identities (modules, unsigned image, headers, buildinfo).

## 2. Invocation

```bash
python3 src/robodog_gnss_time/robodog_gnss_time/linux_kernel_package_provenance.py \
  --output "$HOME/kernel-package-provenance.json"
```

No sudo is required. The script only invokes `uname` and `dpkg-query`. It does not invoke apt, read package indexes, install/remove packages, change kernel configuration, mount tracefs, load BPF or modify clocks.

## 3. Completeness rules

The report is `observed` only if:
1. at least one candidate kernel package is installed;
2. dpkg reports an owner specifically for `/boot/vmlinuz-<release>`;
3. that exact owner matches an installed signed or unsigned kernel-image binary package (ownership of `/boot/config-*` or `/usr/lib/modules/*/vmlinuz` alone is insufficient);
4. at least one installed kernel-code package (modules, headers, buildinfo or unsigned image) supplies a source package and source version;
5. the kernel-code packages agree on one source package/version identity. The signed-image package may legitimately have a different source package identity.

Otherwise the result is `partial`. The report retains both `kernel_source_package_identity` and `signed_image_source_package_identities`; a partial result is useful evidence and must not be filled using a public package candidate.

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


## 5. Public package metadata cross-check (candidate only)

- Signed image binary package metadata: https://www.ubuntuupdates.org/package/core/noble/main/updates/linux-image-7.0.0-44-generic
- Ubuntu Noble source package linux-hwe-7.0 and its binary package family: https://packages.ubuntu.com/source/noble/linux-hwe-7.0
- Published linux-hwe-7.0 package pool: https://security.ubuntu.com/ubuntu/pool/main/l/linux-hwe-7.0/

These sources establish public package-family relationships, not what is installed on the user's VM.


## 6. Provenance owner-matching hardening

The collector compares the owner list for `/boot/vmlinuz-<release>` only against installed image binary packages. It strips a possible `:architecture` suffix from dpkg package identifiers before comparison, while retaining the original identifiers in the raw package/owner records. A match on the config file or modules-tree image does not prove ownership of the running `/boot/vmlinuz` image.

The collector also fails closed if any installed kernel-code package (modules, extra modules, unsigned image, headers or buildinfo) lacks either source package or source version metadata. A valid identity from a different installed package does not mask that missing identity.
