#!/usr/bin/env bash
# Both vehicles share one visible world; one active mission at a time.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${PX4_DEPS_DIR:-$project_dir/_deps}"
if [[ "${CLICK_NAV_PORT:-8765}" == 8766 || "${CLICK_NAV_PORT:-8765}" == 8767 ]]; then
    echo 'Ports 8766 and 8767 are reserved for robot services; choose another CLICK_NAV_PORT.' >&2; exit 2
fi
if pgrep -x px4 >/dev/null || pgrep -f '^gz sim ' >/dev/null; then
    echo 'A simulator is already running; stop it before starting the shared demo.' >&2; exit 2
fi
set +u
if [[ ! -f "$project_dir/install/local_setup.bash" ]]; then
    echo 'Build this checkout first: bash scripts/build.sh' >&2; exit 2
fi
source "$project_dir/scripts/env.sh"
python3 "$project_dir/scripts/doctor.py"
export GZ_SIM_SERVER_CONFIG_PATH="$project_dir/config/gz_server.config"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}:$deps_dir/px4_runtime_v1.17.0/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}:$deps_dir/px4_runtime_v1.17.0/plugins"
cd "$project_dir"
python3 scripts/click_navigation_gateway.py --check-ports --port "${CLICK_NAV_PORT:-8765}"
python3 scripts/prepare_px4_truth_model.py "$deps_dir/px4_runtime_v1.17.0/models/x500/model.sdf"
run_dir="test_artifacts/click_navigation/shared_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
python3 scripts/prepare_click_world.py worlds/chemical_plant_v4.world "$run_dir/chemical_plant_v4.world"
pids=();nav_pids=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${nav_pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
    for attempt in {1..200}; do
        alive=false
        for pid in "${nav_pids[@]}"; do if kill -0 "$pid" 2>/dev/null; then alive=true; fi; done
        if [[ "$alive" == false ]]; then break; fi
        sleep 0.1
    done
    for pid in "${nav_pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Navigation $pid did not exit after 20 s; terminating this SITL test process." >&2
            kill -KILL -- "-$pid" 2>/dev/null || true
        fi
    done
    for pid in "${nav_pids[@]}"; do wait "$pid" 2>/dev/null || true; done
    while read -r px4_pid; do kill -INT "$px4_pid" 2>/dev/null || true; done < <(pgrep -f "^$deps_dir/px4_runtime_v1.17.0/bin/px4 -d" || true)
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid ros2 launch uav_dog_sim chemical_plant.launch.py world_file:="$project_dir/$run_dir/chemical_plant_v4.world" server_config:="$project_dir/config/gz_click_shared.config" gui:="${CLICK_NAV_GUI:-true}" rviz:=false keyboard:=false external_sensors:=false camera_sensors:=false > "$run_dir/world.log" 2>&1 &
pids+=("$!")
world_ready=false
for attempt in {1..60}; do
    if gz service -l | grep -q '/world/chemical_plant_v4/scene/info'; then world_ready=true; break; fi
    sleep 1
done
if [[ "$world_ready" != true ]]; then echo 'Gazebo did not start' >&2; exit 2; fi
# The hardware plugin discovers IMUs during initialization. Let Go2 finish
# binding its own interfaces before adding a second model with imu_sensor.
dog_control_ready=false
for attempt in {1..30}; do
    if timeout 3 ros2 control list_controllers --spin-time 0.5 2>/dev/null | grep -E '^unitree_guide_controller[[:space:]].*[[:space:]]active$' >/dev/null; then
        dog_control_ready=true;break
    fi
    sleep 1
done
if [[ "$dog_control_ready" != true ]]; then echo 'Go2 controller did not become active' >&2; exit 2; fi
setsid python3 scripts/sitl_gcs_heartbeat.py --duration 86400 > "$run_dir/gcs.log" 2>&1 &
pids+=("$!")
# Separate ground spawn points prevent initial inter-robot collision.
setsid env PX4_PARAM_CBRK_SUPPLY_CHK=894281 PX4_GZ_MODEL_POSE='-4,-4,0.2,0,0,0' bash scripts/run_px4_in_plant.sh > "$run_dir/px4.log" 2>&1 &
pids+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge '/px4/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' > "$run_dir/bridge.log" 2>&1 &
pids+=("$!")
for platform in dog drone; do
    extra=();port=8766
    if [[ "$platform" == dog ]]; then extra+=(--dog-speed "${CLICK_NAV_DOG_SPEED:-0.30}" --dog-yaw-speed "${CLICK_NAV_DOG_YAW_SPEED:-0.10}"); fi
    if [[ "${CLICK_NAV_GUI:-true}" == true ]]; then extra+=(--auto-follow); fi
    if [[ "$platform" == drone ]]; then extra+=(--require-ground-truth --origin-y -4);port=8767; fi
    if [[ -n "${CLICK_NAV_KNOWN_GRID:-}" ]]; then extra+=(--known-grid "$CLICK_NAV_KNOWN_GRID"); fi
    setsid python3 -u src/uav_dog_nav2/scripts/click_navigation.py --root "$project_dir" --platform "$platform" --shared-world --port "$port" "${extra[@]}" --ros-args -p use_sim_time:=true > "$run_dir/$platform.log" 2>&1 &
    nav_pids+=("$!")
done
setsid python3 -u scripts/click_navigation_gateway.py --port "${CLICK_NAV_PORT:-8765}" > "$run_dir/gateway.log" 2>&1 &
pids+=("$!")
echo "Open http://localhost:${CLICK_NAV_PORT:-8765} — Go2 and PX4 share the visible Gazebo world. Logs=$run_dir"
# Fail visibly if any essential process exits, rather than leaving a dead UI.
wait -n "${nav_pids[@]}" "${pids[@]}"
