#!/usr/bin/env bash
# Ubuntu 24.04 with the official ROS 2 Jazzy apt repository already configured.
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo 'Supported base: Ubuntu 24.04 (native or WSL2). See README.md.' >&2; exit 2
fi
sudo_cmd=()
if [[ $EUID != 0 ]]; then sudo_cmd=(sudo); fi
"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y curl ca-certificates gnupg
# ROS's repository alone does not provide the unprefixed libgz development packages.
key=$(mktemp)
trap 'rm -f "$key"' EXIT
curl --fail --location --retry 3 https://packages.osrfoundation.org/gazebo.gpg -o "$key"
"${sudo_cmd[@]}" install -m 644 "$key" /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' "$(dpkg --print-architecture)" "$VERSION_CODENAME" | "${sudo_cmd[@]}" tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y \
    build-essential cmake ninja-build git pkg-config ccache libxml2-dev \
    libopencv-dev libunwind-dev rsync zip unzip file bc python3-setuptools python3-wheel \
    libxml2-utils python3-dev protobuf-compiler libprotobuf-dev cppzmq-dev \
    python3-colcon-common-extensions \
    python3-rosdep python3-numpy python3-yaml python3-pip python3-venv \
    libeigen3-dev libasio-dev libspdlog-dev libtinyxml2-dev libssl-dev \
    ros-jazzy-fastrtps ros-jazzy-fastcdr gz-harmonic \
    libgz-sim8-dev libgz-transport13-dev libgz-sensors8-dev libgz-plugin2-dev \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager ros-jazzy-controller-interface ros-jazzy-hardware-interface \
    ros-jazzy-nav2-msgs ros-jazzy-xacro ros-jazzy-kdl-parser ros-jazzy-backward-ros \
    ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 ros-jazzy-rqt-graph
echo 'System dependencies installed. Next: bash scripts/setup_px4.sh'
