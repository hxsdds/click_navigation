#!/usr/bin/env bash
# Persistent, single-platform SITL demo. Never use this launcher on hardware.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
platform="${1:-both}"
if [[ "$platform" == both ]]; then exec bash "$project_dir/scripts/start_click_navigation_both.sh"; fi
port="${CLICK_NAV_PORT:-8765}"
if [[ "$platform" != dog && "$platform" != drone ]]; then echo 'Usage: bash scripts/start_click_navigation.sh dog|drone' >&2; exit 2; fi
if pgrep -x px4 >/dev/null || pgrep -f '^gz sim ' >/dev/null; then
    echo 'A simulator is already running. Stop its launch terminal with Ctrl+C before starting this checkout.' >&2; exit 2
fi
set +u
if [[ ! -f "$project_dir/install/local_setup.bash" ]]; then
    echo 'Build this checkout first: bash scripts/build.sh' >&2; exit 2
fi
source "$project_dir/scripts/env.sh"
deps_dir="${PX4_DEPS_DIR:-$project_dir/_deps}"
cd "$project_dir"
run_dir="test_artifacts/click_navigation/start_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
pids=()
cleanup() {
    trap - EXIT INT TERM
    # Stop navigation first and allow its cancel/hold confirmation to finish.
    if [[ -n "${nav_pid:-}" ]]; then kill -INT -- "-$nav_pid" 2>/dev/null || true; wait "$nav_pid" 2>/dev/null || true; fi
    if [[ "$platform" == drone ]]; then
        # The existing PX4 runner daemonizes PX4, so it can outlive the
        # runner's process group. No PX4 existed at entry; only stop the exact
        # runtime spawned for this instance, after navigation confirms Hold.
        while read -r px4_pid; do
            kill -INT "$px4_pid" 2>/dev/null || true
        done < <(pgrep -f "^$deps_dir/px4_runtime_v1.17.0/bin/px4 -d" || true)
    fi
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
if [[ "$platform" == dog ]]; then
    python3 scripts/prepare_click_world.py worlds/chemical_plant_v4.world "$run_dir/chemical_plant_v4.world"
    setsid ros2 launch uav_dog_sim chemical_plant.launch.py world_file:="$project_dir/$run_dir/chemical_plant_v4.world" server_config:="$project_dir/config/gz_click_shared.config" gui:="${CLICK_NAV_GUI:-true}" rviz:=false keyboard:=false external_sensors:=false camera_sensors:=false > "$run_dir/world.log" 2>&1 &
    pids+=("$!")
else

    python3 scripts/prepare_px4_truth_model.py "$deps_dir/px4_runtime_v1.17.0/models/x500/model.sdf"
    setsid env PX4_PLANT_GUI="${CLICK_NAV_GUI:-true}" bash scripts/run_plant_world.sh > "$run_dir/world.log" 2>&1 &
    pids+=("$!")
    world_ready=false
    for attempt in {1..60}; do
        if gz service -l | grep -q '/world/chemical_plant_v4/scene/info'; then world_ready=true; break; fi
        sleep 1
    done
    if [[ "$world_ready" != true ]]; then echo 'Gazebo world did not become ready' >&2; exit 2; fi
    setsid python3 scripts/sitl_gcs_heartbeat.py --duration 86400 > "$run_dir/gcs.log" 2>&1 &
    pids+=("$!")
    setsid env PX4_PARAM_CBRK_SUPPLY_CHK=894281 bash scripts/run_px4_in_plant.sh > "$run_dir/px4.log" 2>&1 &
    pids+=("$!")
    setsid ros2 run ros_gz_bridge parameter_bridge '/px4/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry' > "$run_dir/bridge.log" 2>&1 &
    pids+=("$!")
fi
extra=()
if [[ "${CLICK_NAV_GUI:-true}" == true ]]; then extra+=(--auto-follow); fi
if [[ "$platform" == drone ]]; then extra+=(--require-ground-truth); fi
if [[ -n "${CLICK_NAV_KNOWN_GRID:-}" ]]; then extra+=(--known-grid "$CLICK_NAV_KNOWN_GRID"); fi
setsid python3 -u src/uav_dog_nav2/scripts/click_navigation.py --root "$project_dir" --platform "$platform" --port "$port" "${extra[@]}" --ros-args -p use_sim_time:=true > "$run_dir/navigation.log" 2>&1 &
nav_pid=$!
echo "Open http://localhost:$port — platform=$platform; logs=$project_dir/$run_dir"
wait "$nav_pid"
