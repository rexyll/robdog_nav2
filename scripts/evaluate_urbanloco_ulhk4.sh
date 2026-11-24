#!/usr/bin/env bash
# Compare the recorded Point-LIO TUM trajectory against UrbanLoco NovAtel GT.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DATASET_DIR="${URBANLOCO_DATASET_DIR:-${REPO_ROOT}/datasets/ulhk_4}"
BAG_DIR="${URBANLOCO_ROS2_BAG:-${DATASET_DIR}/ulhk_bag}"
RESULTS_DIR="${URBANLOCO_RESULTS_DIR:-${DATASET_DIR}/results}"
GT_TOPIC="${URBANLOCO_GT_TOPIC:-/novatel_data/inspvax}"
EST_FILE="${RESULTS_DIR}/ulhk_4_est_tum.txt"
GT_FILE="${RESULTS_DIR}/ulhk_4_gt_tum.txt"
EVO_FILE="${RESULTS_DIR}/ulhk_4_evo_ape.zip"

if [[ -f "${BAG_DIR}" && "${BAG_DIR}" == *.db3 && "${URBANLOCO_DB3_PROVENANCE_CONFIRMED:-0}" != "1" ]]; then
  echo "ERROR: refusing formal P2 evaluation from an unverified standalone DB3." >&2
  echo "Confirm/document its conversion from the expected UrbanLoco ROS1 HK-Data20190117.bag first." >&2
  exit 10
fi
if [[ ! -s "${EST_FILE}" ]]; then
  echo "Missing estimated trajectory ${EST_FILE}; run scripts/run_urbanloco_ulhk4.sh first." >&2
  exit 2
fi
if [[ ! -d "${BAG_DIR}" && ! -f "${BAG_DIR}" ]]; then
  echo "Missing rosbag2 input ${BAG_DIR}; run scripts/prepare_urbanloco_ulhk4.sh first." >&2
  exit 3
fi

mkdir -p "${RESULTS_DIR}"
python3 "${SCRIPT_DIR}/extract_urbanloco_ulhk4_gt.py" \
  --bag-path "${BAG_DIR}" --topic "${GT_TOPIC}" --out "${GT_FILE}"

if ! command -v evo_ape >/dev/null 2>&1; then
  python3 -m pip install --user --break-system-packages evo
fi
export PATH="${HOME}/.local/bin:${PATH}"

echo "Computing APE with SE(3) Umeyama alignment..."
set +e
EVO_OUTPUT="$(evo_ape tum "${GT_FILE}" "${EST_FILE}" -a --save_results "${EVO_FILE}" 2>&1)"
EVO_STATUS=$?
set -e
printf '%s\n' "${EVO_OUTPUT}" | tee "${RESULTS_DIR}/evo_ape.txt"
if (( EVO_STATUS != 0 )); then
  echo "evo_ape failed; inspect ${RESULTS_DIR}/evo_ape.txt." >&2
  exit "${EVO_STATUS}"
fi

RMSE_M="$(printf '%s\n' "${EVO_OUTPUT}" | awk 'tolower($1) == "rmse" {print $2; exit}')"
if [[ -z "${RMSE_M}" ]]; then
  echo "Could not parse RMSE from evo_ape output." >&2
  exit 4
fi

BASELINE_M="2.17"
REFERENCE_TOLERANCE_PCT="${URBANLOCO_REFERENCE_TOLERANCE_PCT:-20}"
REFERENCE_LIMIT_M="$(awk -v b="${BASELINE_M}" -v p="${REFERENCE_TOLERANCE_PCT}" 'BEGIN {printf "%.6f", b*(1+p/100)}')"
WITHIN_REFERENCE="$(awk -v v="${RMSE_M}" -v lim="${REFERENCE_LIMIT_M}" 'BEGIN {print (v <= lim) ? "yes" : "no"}')"

cat > "${RESULTS_DIR}/evaluation_summary.txt" <<EOF
sequence=ulhk_4
dataset_session=HK-Data20190117
estimated_tum=${EST_FILE}
ground_truth_tum=${GT_FILE}
evo_results=${EVO_FILE}
ape_rmse_m=${RMSE_M}
published_point_lio_paper_reference_rmse_m=${BASELINE_M}
reference_tolerance_pct=${REFERENCE_TOLERANCE_PCT}
reference_upper_bound_m=${REFERENCE_LIMIT_M}
within_reference_band=${WITHIN_REFERENCE}
interpretation=offline_Velodyne_dataset_regression_only
EOF

echo
echo "APE RMSE: ${RMSE_M} m"
echo "Published paper reference: ${BASELINE_M} m (+${REFERENCE_TOLERANCE_PCT}% => ${REFERENCE_LIMIT_M} m)"
echo "Within reference band: ${WITHIN_REFERENCE}"
echo "P2 report: ${RESULTS_DIR}/evaluation_summary.txt"
echo "NOTE: paper baseline is a historical reference, not a project navigation acceptance threshold."
echo "NOTE: UrbanLoco uses Velodyne; this does not validate MID-360 CustomMsg timing or Orin performance."
