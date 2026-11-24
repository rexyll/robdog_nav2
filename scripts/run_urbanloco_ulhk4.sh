#!/usr/bin/env bash
# Run the pinned Point-LIO ROS 2 port against UrbanLoco u l h k_4 offline.
# No real robot, LiDAR, network interface, or motion command is involved.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
WS_DIR="${POINTLIO_WS_DIR:-${HOME}/robodog_nav2_ws}"
DATASET_DIR="${URBANLOCO_DATASET_DIR:-${REPO_ROOT}/datasets/ulhk_4}"
BAG_DIR="${URBANLOCO_ROS2_BAG:-${DATASET_DIR}/ulhk_bag}"
RESULTS_DIR="${URBANLOCO_RESULTS_DIR:-${DATASET_DIR}/results}"
LIDAR_TOPIC="${URBANLOCO_LIDAR_TOPIC:-/velodyne_points_0}"
ODOM_TOPIC="${POINTLIO_ODOM_TOPIC:-/aft_mapped_to_init}"
MIN_POSES="${URBANLOCO_MIN_ESTIMATED_POSES:-100}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-223}"

if [[ -d "${BAG_DIR}" && -f "${BAG_DIR}/metadata.yaml" ]]; then
  :
elif [[ -f "${BAG_DIR}" && "${BAG_DIR}" == *.db3 ]]; then
  :
else
  echo "No rosbag2 directory (metadata.yaml) or standalone .db3 at ${BAG_DIR}; run scripts/prepare_urbanloco_ulhk4.sh first." >&2
  exit 2
fi
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} missing under /opt/ros/${ROS_DISTRO}." >&2
  exit 3
fi
if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Point-LIO workspace not built at ${WS_DIR}; use scripts/build_point_lio_jazzy.sh first." >&2
  exit 4
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WS_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID

ros2 pkg prefix point_lio >/dev/null
ros2 pkg executables point_lio | grep -Fq "pointlio_mapping"
CONFIG_FILE="${WS_DIR}/install/point_lio/share/point_lio/config/velodyne_urbanloco.yaml"
if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Intel UrbanLoco Point-LIO config missing: ${CONFIG_FILE}" >&2
  exit 5
fi

mkdir -p "${RESULTS_DIR}"
EST_FILE="${RESULTS_DIR}/ulhk_4_est_tum.txt"
ALGO_LOG="${RESULTS_DIR}/pointlio_mapping.log"
RECORDER_LOG="${RESULTS_DIR}/odometry_recorder.log"
PLAY_LOG="${RESULTS_DIR}/ros2_bag_play.log"
BAG_INFO="${RESULTS_DIR}/bag_info.txt"
rm -f "${EST_FILE}"
ros2 bag info "${BAG_DIR}" > "${BAG_INFO}"
for topic in "${LIDAR_TOPIC}" "${IMU_TOPIC}"; do
  if ! grep -Fq "${topic}" "${BAG_INFO}"; then
    echo "Bag does not contain required Point-LIO input topic ${topic}." >&2
    exit 6
  fi
done
python3 "${SCRIPT_DIR}/audit_urbanloco_ulhk4_db3.py" "${BAG_DIR}" \
  --json-out "${RESULTS_DIR}/bag_audit.json" > "${RESULTS_DIR}/bag_audit.log"

ALGO_PID=""
REC_PID=""
BAG_PID=""
stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  # Check the whole process group as well as its leader: ros2 run/play may
  # spawn children, and a dead leader must not leave a child orphan behind.
  if kill -0 "${pid}" 2>/dev/null || kill -0 -- "-${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    for _ in {1..100}; do
      if ! kill -0 "${pid}" 2>/dev/null && ! kill -0 -- "-${pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    kill -TERM -- "-${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}
cleanup() {
  trap - EXIT INT TERM
  stop_group "${BAG_PID}"
  stop_group "${REC_PID}"
  stop_group "${ALGO_PID}"
}
trap cleanup EXIT INT TERM

echo "Starting pointlio_mapping with dataset-specific Velodyne config..."
setsid ros2 run point_lio pointlio_mapping --ros-args \
  --params-file "${CONFIG_FILE}" >"${ALGO_LOG}" 2>&1 &
ALGO_PID=$!

wait_for_subscription() {
  local topic="$1"
  local attempts="${2:-60}"
  local info count
  for _ in $(seq 1 "${attempts}"); do
    info="$(timeout 3 ros2 topic info "${topic}" 2>/dev/null || true)"
    count="$(printf '%s\n' "${info}" | awk -F': *' '/Subscription count:/ {print $2; exit}')"
    if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
      return 0
    fi
    if ! kill -0 "${ALGO_PID}" 2>/dev/null; then
      echo "pointlio_mapping exited before subscribing to ${topic}; inspect ${ALGO_LOG}." >&2
      return 1
    fi
    sleep 1
  done
  echo "Timed out waiting for Point-LIO subscription to ${topic}; inspect ${ALGO_LOG}." >&2
  return 1
}

wait_for_odometry_endpoints() {
  local info pubs subs
  for _ in $(seq 1 60); do
    info="$(timeout 3 ros2 topic info "${ODOM_TOPIC}" 2>/dev/null || true)"
    pubs="$(printf '%s\n' "${info}" | awk -F': *' '/Publisher count:/ {print $2; exit}')"
    subs="$(printf '%s\n' "${info}" | awk -F': *' '/Subscription count:/ {print $2; exit}')"
    if [[ "${pubs:-0}" =~ ^[0-9]+$ && "${subs:-0}" =~ ^[0-9]+$ ]] \
      && (( pubs > 0 && subs > 0 )); then
      return 0
    fi
    sleep 1
  done
  echo "Odometry publisher/recorder subscription did not become ready; inspect ${ALGO_LOG} and ${RECORDER_LOG}." >&2
  return 1
}

wait_for_subscription "${LIDAR_TOPIC}" 90
wait_for_subscription "${IMU_TOPIC}" 90

echo "Starting TUM odometry recorder..."
setsid python3 "${SCRIPT_DIR}/record_point_lio_odometry_tum.py" \
  --topic "${ODOM_TOPIC}" --out "${EST_FILE}" >"${RECORDER_LOG}" 2>&1 &
REC_PID=$!
wait_for_odometry_endpoints

echo "Starting rosbag2 playback at recorded rate..."
setsid ros2 bag play "${BAG_DIR}" --topics "${LIDAR_TOPIC}" "${IMU_TOPIC}" \
  --rate 1.0 >"${PLAY_LOG}" 2>&1 &
BAG_PID=$!
set +e
wait "${BAG_PID}"
PLAY_STATUS=$?
set -e
BAG_PID=""
if (( PLAY_STATUS != 0 )); then
  echo "ros2 bag play failed with exit code ${PLAY_STATUS}; inspect ${PLAY_LOG}." >&2
  exit 7
fi

sleep 2
stop_group "${ALGO_PID}"
ALGO_PID=""
stop_group "${REC_PID}"
REC_PID=""

if [[ ! -s "${EST_FILE}" ]]; then
  echo "No Point-LIO odometry written; inspect ${ALGO_LOG} and ${RECORDER_LOG}." >&2
  exit 8
fi
POSE_COUNT="$(wc -l < "${EST_FILE}")"
if (( POSE_COUNT < MIN_POSES )); then
  echo "Only ${POSE_COUNT} poses recorded; expected at least ${MIN_POSES}." >&2
  exit 9
fi

cat > "${RESULTS_DIR}/run_manifest.txt" <<EOF
sequence=ulhk_4
dataset_session=HK-Data20190117
pointlio_workspace=${WS_DIR}
bag_dir=${BAG_DIR}
lidar_topic=${LIDAR_TOPIC}
imu_topic=${IMU_TOPIC}
odometry_topic=${ODOM_TOPIC}
ros_domain_id=${ROS_DOMAIN_ID}
estimated_pose_count=${POSE_COUNT}
pointlio_config=${CONFIG_FILE}
EOF

echo "P2 replay completed; estimated poses=${POSE_COUNT}"
echo "Estimate: ${EST_FILE}"
echo "Logs: ${RESULTS_DIR}"
echo "NOTE: this is the Velodyne UrbanLoco compatibility path; it does not validate MID-360 per-point timing."
