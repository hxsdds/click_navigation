#!/usr/bin/env bash
# Build pinned upstream sources. Never installs into the source/original project.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/check_build_path.sh"
deps="${PX4_DEPS_DIR:-$root/_deps}"
check_build_path "$deps"
if [[ "${1:-}" == --import ]]; then
    [[ $# == 2 ]] || { echo 'Usage: bash scripts/setup_px4.sh --import /path/to/px4_deps'; exit 2; }
    exec python3 "$root/scripts/import_runtime.py" --from "$2" --to "$deps"
fi
if [[ $# != 0 ]]; then echo 'Usage: bash scripts/setup_px4.sh [--import /path/to/px4_deps]' >&2; exit 2; fi
source /opt/ros/jazzy/setup.bash
mkdir -p "$deps/src"
fetch() {
    local url="$1" ref="$2" directory="$3"
    if [[ ! -d "$directory/.git" ]]; then
        git clone --branch "$ref" --depth 1 "$url" "$directory"
    fi
    [[ "$(git -C "$directory" describe --tags --exact-match HEAD)" == "$ref" ]] || {
        echo "Unexpected source version in $directory; expected $ref" >&2; exit 2;
    }
}
fetch https://github.com/eProsima/Micro-XRCE-DDS-Agent.git v2.4.3 "$deps/src/Micro-XRCE-DDS-Agent"
# Agent 2.4.3 uses streamable endpoints in logs; Ubuntu 24.04 ships fmt 9.
cmake -S "$deps/src/Micro-XRCE-DDS-Agent" -B "$deps/agent_system_build" \
    -DCMAKE_BUILD_TYPE=Release -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
    -DUAGENT_SUPERBUILD=OFF -DUAGENT_P2P_PROFILE=OFF -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON \
    -DCMAKE_CXX_FLAGS=-DFMT_DEPRECATED_OSTREAM
cmake --build "$deps/agent_system_build" -j "${BUILD_JOBS:-2}"
fetch https://github.com/PX4/PX4-Autopilot.git v1.17.0 "$deps/src/PX4-Autopilot"
px4="$deps/src/PX4-Autopilot"
# Only initialize SITL dependencies, not NuttX, other simulators or vendor SDKs.
# PX4's CMake checks initialize any further dependencies required by this target.
git -C "$px4" submodule update --init --recursive --depth 1 -- \
    src/modules/mavlink/mavlink src/drivers/gps/devices \
    src/lib/events/libevents src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
    src/lib/cdrstream/cyclonedds src/lib/cdrstream/rosidl
# The upstream model repository includes large assets for many other vehicles.
# Keep the exact gitlink commit while downloading only our two model directories.
models="$px4/Tools/simulation/gz"
model_revision="$(git -C "$px4" rev-parse HEAD:Tools/simulation/gz)"
if [[ ! -e "$models/.git" ]]; then
    git clone --filter=blob:none --no-checkout --depth 1 \
        https://github.com/PX4/PX4-gazebo-models.git "$models"
fi
git -C "$models" sparse-checkout set models/x500 models/x500_base
if ! git -C "$models" cat-file -e "$model_revision^{commit}" 2>/dev/null; then
    git -C "$models" fetch --depth 1 --filter=blob:none origin "$model_revision"
fi
git -C "$models" checkout --detach "$model_revision"
# Separate venv: PX4's Python requirements must not replace ROS's system EmPy.
python3 -m venv "$deps/px4_venv"
# Compile bytecode lazily for imported modules, avoiding thousands of extra
# small writes on WSL Windows mounts during first installation.
"$deps/px4_venv/bin/pip" install --no-compile -r "$px4/Tools/setup/requirements.txt"
(
    source "$deps/px4_venv/bin/activate"
    export PATH="$deps/px4_venv/bin:$PATH"
    cmake -S "$px4" -B "$deps/px4_build" -G Ninja -DCONFIG=px4_sitl_default
    cmake --build "$deps/px4_build" --target px4 px4_gz_plugins -j "${BUILD_JOBS:-2}"
)
runtime="$deps/px4_runtime_v1.17.0"
mkdir -p "$runtime/bin" "$runtime/rootfs/etc" "$runtime/models" "$runtime/plugins"
cp -a "$deps/px4_build/bin/." "$runtime/bin/"
cp -a "$deps/px4_build/etc/." "$runtime/rootfs/etc/"
cp -a "$px4/Tools/simulation/gz/models/x500" "$px4/Tools/simulation/gz/models/x500_base" "$runtime/models/"
find "$deps/px4_build/src/modules/simulation/gz_plugins" -name '*.so' -exec cp '{}' "$runtime/plugins/" \;
cp "$px4/src/modules/simulation/gz_bridge/server.config" "$runtime/server.config"
echo "Pinned PX4 v1.17.0 / Agent v2.4.3 ready in $deps"
