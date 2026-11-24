#!/usr/bin/env bash
# Build pinned Livox SDK2, Livox ROS Driver2 and Intel's patched Point-LIO in one Jazzy workspace.
# Usage: scripts/build_point_lio_jazzy.sh [workspace_root]
set -euo pipefail

WORKSPACE="$HOME/robodog_nav2_ws"
if [[ "$#" -gt 0 ]]; then
  WORKSPACE="$1"
fi
POINTLIO_DIR="$WORKSPACE/src/point_lio"
DRIVER_DIR="$WORKSPACE/src/livox_ros_driver2"
# Keep the standalone SDK source outside the entire colcon workspace:
# Driver2's build.sh invokes colcon from the workspace root, whose default
# package discovery also traverses sibling directories such as workspace/vendor.
SDK_SOURCE_ROOT="${WORKSPACE}_external"
SDK_DIR="$SDK_SOURCE_ROOT/Livox-SDK2"

POINTLIO_BASE="97b0042e397ec71c327966abebff9b4b7e2c154a"
DRIVER_COMMIT="13eb05e4e6dd7a765b934d0c5fd6236676a57b49"
SDK_COMMIT="f5d9375f84efe2b15bc0a052d3e18482ed13adf4"
BUILD_JOBS="$(printenv BUILD_JOBS || echo 2)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ERROR: ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 2
fi

if [[ ! -f "$POINTLIO_DIR/.git/robodog-intel-patches.lock" ]]; then
  echo "ERROR: pinned Point-LIO source/patch lock missing. Run prepare_point_lio_intel_ros2.sh first." >&2
  exit 3
fi
grep -Fxq "upstream=$POINTLIO_BASE" "$POINTLIO_DIR/.git/robodog-intel-patches.lock"
grep -Fxq "intel_repo=decbb06fc1b746b229957b66b92ab972e1f05609" \
  "$POINTLIO_DIR/.git/robodog-intel-patches.lock"
if [[ "$(git -C "$POINTLIO_DIR" rev-list --count "$POINTLIO_BASE"..HEAD)" -ne 3 ]]; then
  echo "ERROR: Point-LIO source does not contain exactly the three pinned Intel patches." >&2
  exit 4
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

mkdir -p "$WORKSPACE/src" "$SDK_SOURCE_ROOT"

if [[ ! -e "$DRIVER_DIR" ]]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "$DRIVER_DIR"
  git -C "$DRIVER_DIR" checkout --detach "$DRIVER_COMMIT"
fi
if [[ ! -d "$DRIVER_DIR/.git" ]]; then
  echo "ERROR: existing Livox driver path is not a Git checkout." >&2
  exit 5
fi
if [[ "$(git -C "$DRIVER_DIR" rev-parse HEAD)" != "$DRIVER_COMMIT" ]]; then
  echo "ERROR: Livox driver checkout does not match pinned v1.2.6 commit." >&2
  exit 6
fi

if [[ ! -e "$SDK_DIR" ]]; then
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$SDK_DIR"
fi
if [[ ! -d "$SDK_DIR/.git" ]]; then
  echo "ERROR: existing Livox SDK2 path is not a Git checkout." >&2
  exit 7
fi
git -C "$SDK_DIR" checkout --detach "$SDK_COMMIT"
if [[ "$(git -C "$SDK_DIR" rev-parse HEAD)" != "$SDK_COMMIT" ]]; then
  echo "ERROR: Livox SDK2 checkout does not match pinned v1.3.1 commit." >&2
  exit 8
fi

echo "==> Build pinned Livox SDK2 v1.3.1"
cmake -S "$SDK_DIR" -B "$SDK_DIR/build" \
  -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS=-include cstdint'
cmake --build "$SDK_DIR/build" --parallel "$BUILD_JOBS"
if [[ "$EUID" -eq 0 ]]; then
  cmake --install "$SDK_DIR/build"
  ldconfig
else
  sudo cmake --install "$SDK_DIR/build"
  sudo ldconfig
fi

# Driver2's Jazzy CMake uses the installed SDK2 library and public headers.
test -f /usr/local/lib/liblivox_lidar_sdk_shared.so
test -f /usr/local/include/livox_lidar_api.h
test ! -e "$WORKSPACE/src/Livox-SDK2"
test ! -e "$WORKSPACE/vendor/Livox-SDK2"
case "$SDK_DIR" in
  "$WORKSPACE"/*)
    echo "ERROR: SDK2 source must stay outside the entire colcon workspace." >&2
    exit 9
    ;;
esac

echo "==> Configure ROS 2 driver package manifest and resolve dependencies"
cp "$DRIVER_DIR/package_ROS2.xml" "$DRIVER_DIR/package.xml"
rosdep update
# The CI workflow removes apt lists after installing native prerequisites to
# keep the image small; refresh them before rosdep invokes apt-get.
if [[ "$EUID" -eq 0 ]]; then
  apt-get update
else
  sudo apt-get update
fi
rosdep install --from-paths "$WORKSPACE/src" --ignore-src --rosdistro jazzy -r -y

echo "==> Verify colcon package discovery excludes Livox SDK2 source"
PACKAGE_NAMES="$(cd "$WORKSPACE" && colcon list | awk '{print $1}')"
printf '%s\n' "$PACKAGE_NAMES"
printf '%s\n' "$PACKAGE_NAMES" | grep -Fxq "livox_ros_driver2"
printf '%s\n' "$PACKAGE_NAMES" | grep -Fxq "point_lio"
if printf '%s\n' "$PACKAGE_NAMES" | grep -Fxq "livox_sdk2"; then
  echo "ERROR: colcon still discovers livox_sdk2 source inside workspace." >&2
  exit 10
fi

echo "==> Build Livox ROS Driver2 and Point-LIO using its Jazzy build entrypoint"
(
  cd "$DRIVER_DIR"
  ./build.sh jazzy
)

set +u
source "$WORKSPACE/install/setup.bash"
set -u

echo "==> Verify installed package prefixes"
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix point_lio
ros2 pkg executables point_lio
