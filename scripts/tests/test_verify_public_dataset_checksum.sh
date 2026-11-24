#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
VERIFY_SCRIPT="${REPO_ROOT}/scripts/verify_public_dataset_checksum.sh"

tmp_dir="$(mktemp -d)"
trap 'rm -rf "${tmp_dir}"' EXIT

fixture="${tmp_dir}/sample bag.bag"
printf 'sample rosbag payload\n' > "${fixture}"
expected="$(sha256sum -- "${fixture}" | awk '{print $1}')"
manifest="${tmp_dir}/checksums.txt"
printf 'fixture/sample.bag %s\n' "${expected}" > "${manifest}"

valid_output="$(PUBLIC_DATASET_SHA256_MANIFEST="${manifest}" bash "${VERIFY_SCRIPT}" fixture/sample.bag "${fixture}")"
grep -Fq 'status=verified_upstream_sha256' <<< "${valid_output}"
grep -Fq "actual_sha256=${expected}" <<< "${valid_output}"

printf 'fixture/sample.bag %064d\n' 0 > "${manifest}"
if PUBLIC_DATASET_SHA256_MANIFEST="${manifest}" bash "${VERIFY_SCRIPT}" fixture/sample.bag "${fixture}" >/dev/null 2>&1; then
  echo "ERROR: checksum mismatch was incorrectly accepted" >&2
  exit 1
fi

if PUBLIC_DATASET_SHA256_MANIFEST="${tmp_dir}/missing.txt" bash "${VERIFY_SCRIPT}" fixture/sample.bag "${fixture}" >/dev/null 2>&1; then
  echo "ERROR: missing checksum manifest was incorrectly accepted" >&2
  exit 1
fi

echo "External dataset checksum verifier tests passed."
