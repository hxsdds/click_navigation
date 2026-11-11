#!/usr/bin/env bash
# Build pinned upstream sources. Never installs into the source/original project.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps="${PX4_DEPS_DIR:-$root/_deps}"
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
        git clone --branch "$ref" --depth 1 --recursive "$url" "$directory"
    fi
    [[ "$(git -C "$directory" describe --tags --exact-match HEAD)" == "$ref" ]] || {
        echo "Unexpected source version in $directory; expected $ref" >&2; exit 2;
    }
}
fetch https://github.com/eProsima/Micro-XRCE-DDS-Agent.git v2.4.3 "$deps/src/Micro-XRCE-DDS-Agent"
cmake -S "$deps/src/Micro-XRCE-DDS-Agent" -B "$deps/agent_system_build" \
    -DCMAKE_BUILD_TYPE=Release -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
    -DUAGENT_SUPERBUILD=OFF -DCMAKE_BUILD_RPATH_USE_ORIGIN=ON
cmake --build "$deps/agent_system_build" -j "${BUILD_JOBS:-2}"
fetch https://github.com/PX4/PX4-Autopilot.git v1.17.0 "$deps/src/PX4-Autopilot"
px4="$deps/src/PX4-Autopilot"
# Separate venv: PX4's Python requirements must not replace ROS's system EmPy.
python3 -m venv "$deps/px4_venv"
"$deps/px4_venv/bin/pip" install -r "$px4/Tools/setup/requirements.txt"
(
    source "$deps/px4_venv/bin/activate"
    export PATH="$deps/px4_venv/bin:$PATH"
    cmake -S "$px4" -B "$deps/px4_build" -G Ninja -DCONFIG=px4_sitl_default
    cmake --build "$deps/px4_build" --target px4 px4_gz_plugins -j "${BUILD_JOBS:-2}"
)
runtime="$deps/px4_runtime_v1.17.0"
mkdir -p "$runtime/bin" "$runtime/rootfs/etc" "$runtime/models" "$runtime/plugins"
cp -a "$deps/px4_build/bin/." "$runtime/bin/"
cp -a "$deps/px4_build/rootfs/etc/." "$runtime/rootfs/etc/"
cp -a "$px4/ROMFS/px4fmu_common/init.d" "$px4/ROMFS/px4fmu_common/init.d-posix" "$runtime/rootfs/etc/"
cp -a "$px4/Tools/simulation/gz/models/x500" "$px4/Tools/simulation/gz/models/x500_base" "$runtime/models/"
find "$deps/px4_build/src/modules/simulation/gz_plugins" -name '*.so' -exec cp '{}' "$runtime/plugins/" \;
cp "$px4/src/modules/simulation/gz_bridge/server.config" "$runtime/server.config"
echo "Pinned PX4 v1.17.0 / Agent v2.4.3 ready in $deps"
