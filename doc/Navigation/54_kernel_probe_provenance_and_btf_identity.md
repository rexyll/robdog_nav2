# Kernel probe provenance and BTF artifact identity

- Status: HOST_FIXTURE_PROVENANCE_FENCED / BTF_SHA256_ADDED / SYNTHETIC_TESTS_ADDED
- Date: 2026-11-27
- Related: Navigation 51, 52, 53

## 1. Purpose

The capability probe can inspect either the live host or a supplied rootfs fixture. Previously, collector-host uname metadata could be mistaken for target-kernel metadata when `--root` pointed to a fixture. This increment makes the distinction explicit and records the identity of the inspected vmlinux BTF bytes.

## 2. Report provenance

The report now carries `schema_version=2` and `collection_mode`:
- `live_host`: root resolves to `/`; uname release/machine/system describe the same running host whose procfs/sysfs are inspected.
- `rootfs_fixture`: root is another directory; collector host metadata is kept under `collector_host` and explicitly not attributed to the fixture target. Target release remains unknown unless supplied using `--kernel-release`.

The target kernel section records the boot ID when readable. For a fixture without an explicit target release, `/boot/config-<release>` is not guessed or probed. Collection clock fields name CLOCK_MONOTONIC and CLOCK_REALTIME.

## 3. BTF artifact identity

When readable, `/sys/kernel/btf/vmlinux` is recorded with byte size and SHA-256. The hash is an artifact identity aid; it does not validate BTF semantics, prove correspondence to a separately running kernel, or establish attachability.

The optional bpftool inventory also records `bpftool version` output when the version command succeeds. The BTF dump remains read-only and bounded by the existing timeout.

## 4. Validation and limitations

Synthetic tests cover fixture/collector metadata separation and BTF SHA-256. CI validates only synthetic fixture behavior. A real report must be generated on the intended Linux VM; do not copy an unrelated `/proc/self/status` or uname result into a fixture and label it as target evidence. Orin BSP evidence must be collected separately.

The probe still does not load BPF, attach fentry/kprobe, enable tracepoints, or change system configuration.
