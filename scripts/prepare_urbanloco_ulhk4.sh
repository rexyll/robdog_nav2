#!/usr/bin/env bash
# Check or convert the manually downloaded UrbanLoco HK-Data20190117 bag.
# The dataset is not downloaded automatically and must not be committed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DATASET_DIR="${URBANLOCO_DATASET_DIR:-${REPO_ROOT}/datasets/ulhk_4}"
RAW_BAG="${URBANLOCO_RAW_BAG:-${DATASET_DIR}/HK-Data20190117.bag}"
BAG_DIR="${URBANLOCO_ROS2_BAG:-${DATASET_DIR}/ulhk_bag}"
LIDAR_TOPIC="${URBANLOCO_LIDAR_TOPIC:-/velodyne_points_0}"
IMU_TOPIC="${URBANLOCO_IMU_TOPIC:-/imu/data}"
GT_TOPIC="${URBANLOCO_GT_TOPIC:-/novatel_data/inspvax}"

mkdir -p "${DATASET_DIR}"

if [[ -d "${BAG_DIR}" ]]; then
  echo "ROS 2 bag directory already present: ${BAG_DIR}"
elif [[ -f "${BAG_DIR}" && "${BAG_DIR}" == *.db3 ]]; then
  echo "Using existing standalone SQLite3 rosbag: ${BAG_DIR}"
elif [[ -e "${BAG_DIR}" ]]; then
  echo "ERROR: rosbag input exists but is neither a directory nor a standalone .db3 file: ${BAG_DIR}" >&2
  exit 3
else
  if [[ ! -f "${RAW_BAG}" ]]; then
    cat >&2 <<EOF
UrbanLoco HK-Data20190117 (ulhk_4) is not present.

UrbanLoco does not provide a scriptable download for this large ROS 1 bag.
Download the HK-Data20190117 entry from the Hong Kong Dataset section and
place it at:

  ${RAW_BAG}

Mirrors listed by Intel's Point-LIO reproduction guide:
  Dropbox: https://www.dropbox.com/scl/fo/zrsmoddbq96t4go1wbxwp/AJw_DGVXng06DmLx9j9iQMs?rlkey=rk11n8tt62ejbg8mbixrm6quz&e=1&st=j7sy3izj&dl=0
  Baidu:   https://pan.baidu.com/s/1-5d8xM1tzfsSSueTiU6-MQ?pwd=sufc

Open the HK-Data20190117 file inside the shared Hong Kong folder.
The dataset license is CC BY-NC-SA 4.0. Do not commit or redistribute the bag.
EOF
    exit 2
  fi

  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u

  if ! command -v rosbags-convert >/dev/null 2>&1; then
    python3 -m pip install --user --break-system-packages rosbags
  fi
  export PATH="${HOME}/.local/bin:${PATH}"

  if [[ -e "${BAG_DIR}" ]]; then
    echo "ERROR: output path already exists but is not a rosbag2 directory: ${BAG_DIR}" >&2
    exit 3
  fi
  echo "Converting ROS 1 bag to ROS 2 sqlite3 bag..."
  rosbags-convert --src "${RAW_BAG}" --dst "${BAG_DIR}"
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
INFO_FILE="${DATASET_DIR}/ros2_bag_info.txt"
ros2 bag info "${BAG_DIR}" | tee "${INFO_FILE}"

for topic in "${LIDAR_TOPIC}" "${IMU_TOPIC}" "${GT_TOPIC}"; do
  if ! grep -Fq "${topic}" "${INFO_FILE}"; then
    echo "ERROR: expected topic ${topic} not present in bag metadata." >&2
    echo "Inspect ${INFO_FILE}; do not silently substitute another topic." >&2
    exit 4
  fi
done

if [[ -f "${BAG_DIR}" ]]; then
  DB3_FILES=("${BAG_DIR}")
else
  shopt -s nullglob
  DB3_FILES=("${BAG_DIR}"/*.db3)
  shopt -u nullglob
fi
if (( ${#DB3_FILES[@]} == 0 )); then
  echo "ERROR: P2 ground-truth extraction currently requires sqlite3 rosbag storage (*.db3)." >&2
  exit 5
fi

AUDIT_JSON="${DATASET_DIR}/bag_audit.json"
python3 "${SCRIPT_DIR}/audit_urbanloco_ulhk4_db3.py" "${BAG_DIR}" --json-out "${AUDIT_JSON}"
echo "P2 input audit: ${AUDIT_JSON}"

# Record provenance and checksums outside the bag directory. Recompute the
# raw digest on every run so replacing a same-named bag cannot retain a stale hash.
RAW_SHA256="not_available"
RAW_SIZE_BYTES="not_available"
if [[ -f "${RAW_BAG}" ]]; then
  RAW_SIZE_BYTES="$(stat -c '%s' "${RAW_BAG}")"
  sha256sum "${RAW_BAG}" > "${DATASET_DIR}/raw_bag.sha256"
  RAW_SHA256="$(awk '{print $1}' "${DATASET_DIR}/raw_bag.sha256")"
fi

DB3_SUMS="${DATASET_DIR}/ros2_bag_db3.sha256"
: > "${DB3_SUMS}"
for db3 in "${DB3_FILES[@]}"; do
  [[ -f "${db3}" ]] && sha256sum "${db3}" >> "${DB3_SUMS}"
done

cat > "${DATASET_DIR}/dataset_manifest.txt" <<EOF
sequence=ulhk_4
session=HK-Data20190117
source=UrbanLoco Hong Kong dataset; manual download/mirror documented in datasets/README.md
license=CC BY-NC-SA 4.0
acquisition_date=${URBANLOCO_ACQUISITION_DATE:-unknown}
prepared_at_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
raw_bag=${RAW_BAG}
raw_bag_size_bytes=${RAW_SIZE_BYTES}
raw_bag_sha256=${RAW_SHA256}
ros2_bag_input=${BAG_DIR}
ros2_bag_input_kind=$(if [[ -f "${BAG_DIR}" ]]; then echo standalone_db3; else echo rosbag2_directory; fi)
lidar_topic=${LIDAR_TOPIC}
imu_topic=${IMU_TOPIC}
ground_truth_topic=${GT_TOPIC}
ros2_bag_format=sqlite3
db3_checksum_file=${DB3_SUMS}
EOF

echo "UrbanLoco u l h k_4 bag is ready: ${BAG_DIR}"
echo "Dataset provenance: ${DATASET_DIR}/dataset_manifest.txt"
echo "Raw bag SHA256: ${RAW_SHA256}"
