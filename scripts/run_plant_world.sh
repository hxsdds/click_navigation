#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${PX4_DEPS_DIR:-$project_dir/_deps}"
set +u
source /opt/ros/jazzy/setup.bash
export GZ_SIM_SERVER_CONFIG_PATH="$project_dir/config/gz_server.config"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$deps_dir/px4_runtime_v1.17.0/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$deps_dir/px4_runtime_v1.17.0/plugins"
if [[ "${PX4_PLANT_GUI:-false}" == true ]]; then
    exec gz sim -r "$project_dir/worlds/chemical_plant_v4.world"
fi
exec gz sim -s -r --headless-rendering "$project_dir/worlds/chemical_plant_v4.world"
