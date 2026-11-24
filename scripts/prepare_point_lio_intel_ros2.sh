#!/usr/bin/env bash
# Prepare the pinned Intel ROS 2 Jazzy Point-LIO port on a pristine upstream tree.
# Usage: scripts/prepare_point_lio_intel_ros2.sh [workspace_root]
set -euo pipefail

WORKSPACE="$HOME/robodog_nav2_ws"
if [[ "$#" -gt 0 ]]; then
  WORKSPACE="$1"
fi
POINTLIO_DIR="$WORKSPACE/src/point_lio"
UPSTREAM_URL="https://github.com/hku-mars/Point-LIO.git"
UPSTREAM_COMMIT="97b0042e397ec71c327966abebff9b4b7e2c154a"
INTEL_BASE="https://raw.githubusercontent.com/open-edge-platform/edge-ai-suites/decbb06fc1b746b229957b66b92ab972e1f05609/robotics-ai-suite/pipelines/point-lio-demo/patches"
LOCK_CONTENT="upstream=97b0042e397ec71c327966abebff9b4b7e2c154a
intel_repo=decbb06fc1b746b229957b66b92ab972e1f05609
patches=0001,0002,0003"

mkdir -p "$WORKSPACE/src"
CLONED_NEW_TREE=0
if [[ ! -e "$POINTLIO_DIR" ]]; then
  git clone --branch point-lio-with-grid-map --no-checkout "$UPSTREAM_URL" "$POINTLIO_DIR"
  git -C "$POINTLIO_DIR" checkout --detach "$UPSTREAM_COMMIT"
  CLONED_NEW_TREE=1
fi

if [[ ! -d "$POINTLIO_DIR/.git" ]]; then
  echo "ERROR: $POINTLIO_DIR exists but is not a Git checkout." >&2
  exit 2
fi

LOCK_FILE="$POINTLIO_DIR/.git/robodog-intel-patches.lock"
if [[ -f "$LOCK_FILE" ]]; then
  if [[ "$(cat "$LOCK_FILE")" == "$LOCK_CONTENT" ]]; then
    echo "Point-LIO source and pinned Intel patch series already prepared."
    git -C "$POINTLIO_DIR" log -3 --oneline
    exit 0
  fi
  echo "ERROR: existing Point-LIO patch lock does not match the requested source set." >&2
  exit 3
fi

CURRENT_HEAD="$(git -C "$POINTLIO_DIR" rev-parse HEAD)"
if [[ "$CURRENT_HEAD" != "$UPSTREAM_COMMIT" ]]; then
  echo "ERROR: expected pristine upstream $UPSTREAM_COMMIT, got $CURRENT_HEAD." >&2
  echo "Refusing to reset or overwrite an unknown Point-LIO tree." >&2
  exit 4
fi

if [[ -n "$(git -C "$POINTLIO_DIR" status --porcelain)" ]]; then
  echo "ERROR: Point-LIO checkout is dirty; refusing to apply vendor patches." >&2
  exit 5
fi

# git am needs a committer identity. Keep this scoped to the disposable
# Point-LIO clone; never alter the user's global Git configuration.
git -C "$POINTLIO_DIR" config user.name "RobDog Nav2 Patch Preparation"
git -C "$POINTLIO_DIR" config user.email "robodog-nav2-patch-prep@users.noreply.github.com"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

for patch in \
  "0001-Port-Point-LIO-to-ROS2-and-add-benchmarking-instrume.patch" \
  "0002-Reformat-sources-to-match-the-project-s-real-clang-f.patch" \
  "0003-Fix-early-loop-exit-and-a-distance-threshold-typo.patch"
do
  echo "Downloading pinned patch: $patch"
  curl --fail --location --silent --show-error \
    "$INTEL_BASE/$patch" \
    --output "$TMP_DIR/$patch"
  echo "Applying patch: $patch"
  git -C "$POINTLIO_DIR" am --keep-cr "$TMP_DIR/$patch"
done

if [[ -n "$(git -C "$POINTLIO_DIR" status --porcelain)" ]]; then
  echo "ERROR: patch application left an unexpected dirty worktree." >&2
  exit 6
fi

printf '%s\n' "$LOCK_CONTENT" > "$LOCK_FILE"
echo "Pinned Intel ROS 2 patch series applied successfully."
git -C "$POINTLIO_DIR" log -3 --oneline
