#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/check_build_path.sh"
source /opt/ros/jazzy/setup.bash
cd "$root"
# Use only this source tree; do not source an old project install as an underlay.
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
export MAKEFLAGS="-j${BUILD_JOBS:-2}"
minimal=ON
if [[ "${PX4_MSGS_FULL_BUILD:-0}" == 1 ]]; then minimal=OFF; fi
colcon build --base-paths src --symlink-install --parallel-workers "${COLCON_WORKERS:-2}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPX4_MSGS_CLICK_NAVIGATION_ONLY="$minimal" "$@"
