import subprocess

from robodog_gnss_time import linux_kernel_package_provenance as provenance


def _completed(stdout="", returncode=0, stderr=""):
    return subprocess.CompletedProcess(
        args=[], returncode=returncode, stdout=stdout, stderr=stderr
    )


def test_package_provenance_reports_exact_installed_source_identity(monkeypatch):
    release = "7.0.0-44-generic"
    image = f"linux-image-{release}"
    def fake_run(argv, check, capture_output, text, timeout):
        if argv[:2] == ["dpkg-query", "-W"]:
            package = argv[-1]
            if package == image:
                return _completed(
                    f"{image}\t7.0.0-44.44~24.04.1\tlinux-hwe-7.0\t"
                    "7.0.0-44.44~24.04.1\tii \n"
                )
            return _completed(returncode=1, stderr="not installed")
        if argv[:2] == ["dpkg-query", "-S"]:
            path = argv[-1]
            if path == f"/boot/vmlinuz-{release}":
                return _completed(f"{image}: {path}\n")
            return _completed(returncode=1, stderr="no path owner")
        if argv[0] == "uname":
            return _completed("#44-Ubuntu SMP PREEMPT_DYNAMIC\n")
        raise AssertionError(argv)

    monkeypatch.setattr(provenance.subprocess, "run", fake_run)
    report = provenance.collect_linux_kernel_package_provenance(release)
    assert report["status"] == "observed"
    assert report["kernel"]["release"] == release
    assert report["source_package_identities"] == [{
        "source_package": "linux-hwe-7.0",
        "source_version": "7.0.0-44.44~24.04.1",
    }]
    image_row = next(row for row in report["packages"] if row["package_query"] == image)
    assert image_row["status"] == "installed"
    assert image_row["binary_version"] == "7.0.0-44.44~24.04.1"
    assert image_row["source_package"] == "linux-hwe-7.0"
    assert report["side_effects"]["bpf_loaded_or_attached"] is False


def test_package_provenance_is_partial_without_kernel_file_owner(monkeypatch):
    release = "7.0.0-44-generic"
    image = f"linux-image-{release}"
    def fake_run(argv, check, capture_output, text, timeout):
        if argv[:2] == ["dpkg-query", "-W"] and argv[-1] == image:
            return _completed(
                f"{image}\t7.0.0-44.44~24.04.1\tlinux-hwe-7.0\t"
                "7.0.0-44.44~24.04.1\tii \n"
            )
        if argv[:2] == ["dpkg-query", "-W"]:
            return _completed(returncode=1, stderr="not installed")
        if argv[:2] == ["dpkg-query", "-S"]:
            return _completed(returncode=1, stderr="no owner")
        if argv[0] == "uname":
            return _completed("build\n")
        raise AssertionError(argv)

    monkeypatch.setattr(provenance.subprocess, "run", fake_run)
    report = provenance.collect_linux_kernel_package_provenance(release)
    assert report["status"] == "partial"


def test_package_provenance_detects_conflicting_source_versions(monkeypatch):
    release = "7.0.0-44-generic"
    def fake_run(argv, check, capture_output, text, timeout):
        if argv[:2] == ["dpkg-query", "-W"]:
            package = argv[-1]
            if package in (f"linux-image-{release}", f"linux-modules-{release}"):
                source_version = "7.0.0-44.44~24.04.1" if package.startswith("linux-image") else "7.0.0-44.45~24.04.1"
                return _completed(
                    f"{package}\t{source_version}\tlinux-hwe-7.0\t"
                    f"{source_version}\tii \n"
                )
            return _completed(returncode=1, stderr="not installed")
        if argv[:2] == ["dpkg-query", "-S"]:
            path = argv[-1]
            if path == f"/boot/vmlinuz-{release}":
                return _completed(f"linux-image-{release}: {path}\n")
            return _completed(returncode=1, stderr="no owner")
        if argv[0] == "uname":
            return _completed("build\n")
        raise AssertionError(argv)

    monkeypatch.setattr(provenance.subprocess, "run", fake_run)
    report = provenance.collect_linux_kernel_package_provenance(release)
    assert report["status"] == "partial"
    assert len(report["source_package_identities"]) == 2


def test_command_wrapper_handles_missing_dpkg_query(monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError("dpkg-query")
    monkeypatch.setattr(provenance.subprocess, "run", missing)
    result = provenance._command(["dpkg-query", "-W"])
    assert result["status"] == "command_not_found"
