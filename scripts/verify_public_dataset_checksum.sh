#!/usr/bin/env bash
# Verify an externally downloaded dataset file against a pinned upstream SHA256.
# Usage: bash scripts/verify_public_dataset_checksum.sh <artifact-key> <file>
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MANIFEST="${PUBLIC_DATASET_SHA256_MANIFEST:-${REPO_ROOT}/datasets/external_sha256.txt}"

usage() {
  echo "Usage: bash scripts/verify_public_dataset_checksum.sh <artifact-key> <file>" >&2
  echo "Example: bash scripts/verify_public_dataset_checksum.sh m3dgr/Dynamic03.bag ~/Downloads/Dynamic03.bag" >&2
}

if [[ $# -ne 2 ]]; then
  usage
  exit 2
fi

ARTIFACT_KEY="$1"
FILE_PATH="$2"

case "/${ARTIFACT_KEY}/" in
  *"/../"*|*"//"*)
    echo "ERROR: invalid artifact key: ${ARTIFACT_KEY}" >&2
    exit 2
    ;;
esac

if [[ ! -f "${MANIFEST}" ]]; then
  echo "ERROR: checksum manifest not found: ${MANIFEST}" >&2
  exit 2
fi
if [[ ! -f "${FILE_PATH}" ]]; then
  echo "ERROR: dataset file not found: ${FILE_PATH}" >&2
  exit 2
fi

EXPECTED_SHA256="$(awk -v key="${ARTIFACT_KEY}" '
  /^[[:space:]]*#/ || NF == 0 { next }
  $1 == key { value = $2; count += 1 }
  END {
    if (count != 1) exit 3
    print value
  }
' "${MANIFEST}")" || {
  echo "ERROR: expected exactly one checksum entry for ${ARTIFACT_KEY} in ${MANIFEST}" >&2
  exit 2
}

if [[ ! "${EXPECTED_SHA256}" =~ ^[[:xdigit:]]{64}$ ]]; then
  echo "ERROR: invalid SHA256 entry for ${ARTIFACT_KEY}" >&2
  exit 2
fi

ACTUAL_SHA256="$(sha256sum -- "${FILE_PATH}" | awk '{print $1}')"
if [[ "${ACTUAL_SHA256,,}" != "${EXPECTED_SHA256,,}" ]]; then
  echo "ERROR: SHA256 mismatch for ${ARTIFACT_KEY}" >&2
  echo "Expected: ${EXPECTED_SHA256}" >&2
  echo "Actual:   ${ACTUAL_SHA256}" >&2
  exit 3
fi

FILE_SIZE_BYTES="$(stat -c '%s' -- "${FILE_PATH}")"
printf 'status=verified_upstream_sha256\n'
printf 'artifact_key=%s\n' "${ARTIFACT_KEY}"
printf 'file=%s\n' "${FILE_PATH}"
printf 'size_bytes=%s\n' "${FILE_SIZE_BYTES}"
printf 'upstream_sha256=%s\n' "${EXPECTED_SHA256}"
printf 'actual_sha256=%s\n' "${ACTUAL_SHA256}"
