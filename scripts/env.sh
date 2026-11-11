#!/usr/bin/env bash
# Source this file from bash. All project/dependency paths are relocatable.
CLICK_NAV_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DEPS_DIR="${PX4_DEPS_DIR:-$CLICK_NAV_ROOT/_deps}"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo 'ROS 2 Jazzy is required. See README.md.' >&2; return 2 2>/dev/null || exit 2
fi
source /opt/ros/jazzy/setup.bash
if [[ -f "$CLICK_NAV_ROOT/install/local_setup.bash" ]]; then
    source "$CLICK_NAV_ROOT/install/local_setup.bash"
fi
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$PX4_DEPS_DIR/px4_runtime_v1.17.0/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$PX4_DEPS_DIR/px4_runtime_v1.17.0/plugins"
export LD_LIBRARY_PATH="$PX4_DEPS_DIR/agent_system_build:${LD_LIBRARY_PATH:-}"
