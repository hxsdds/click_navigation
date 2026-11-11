#!/usr/bin/env bash
# Start PX4 x500 and the ROS 2 XRCE agent in an already running plant world.
set -eo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${PX4_DEPS_DIR:-$project_dir/_deps}"
runtime="$deps_dir/px4_runtime_v1.17.0"
agent="$deps_dir/agent_system_build/MicroXRCEAgent"
run_rootfs="$(mktemp -d /tmp/uav_dog_px4_rootfs.XXXXXX)"
world_name="${PX4_GZ_WORLD:-chemical_plant_v4}"

if [[ ! -x "$runtime/bin/px4" || ! -x "$agent" ]]; then
    echo "PX4 runtime or XRCE agent missing in $deps_dir" >&2
    exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source "$project_dir/scripts/env.sh"
export PX4_GZ_MODELS="$runtime/models"
export PX4_GZ_WORLDS="$runtime/worlds"
export PX4_GZ_PLUGINS="$runtime/plugins"
export PX4_GZ_SERVER_CONFIG="$runtime/server.config"
set -u
if ! gz service -l | grep -q "/world/$world_name/scene/info"; then
    echo "Start the $world_name Gazebo world before PX4." >&2
    exit 2
fi
export PX4_GZ_STANDALONE=1
export PX4_GZ_WORLD="$world_name"
export PX4_SIM_MODEL=gz_x500
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:--4,-6,0.2,0,0,0}"

"$agent" udp4 -p 8888 &
agent_pid=$!
trap 'kill "$agent_pid" 2>/dev/null || true' EXIT
cp -a "$runtime/rootfs/etc" "$run_rootfs/"
cd "$run_rootfs"
"$runtime/bin/px4" -d
while pgrep -f "^$runtime/bin/px4" >/dev/null; do
    sleep 1
done
