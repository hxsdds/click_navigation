#!/usr/bin/env bash
# Ubuntu 24.04 with the official ROS 2 Jazzy apt repository already configured.
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
    echo 'Supported base: Ubuntu 24.04 (native or WSL2). See README.md.' >&2; exit 2
fi
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake ninja-build git pkg-config ccache libxml2-dev \
    libxml2-utils python3-dev protobuf-compiler libprotobuf-dev cppzmq-dev \
    python3-colcon-common-extensions \
    python3-rosdep python3-numpy python3-yaml python3-pip python3-venv \
    libeigen3-dev libasio-dev libspdlog-dev libtinyxml2-dev libssl-dev \
    libgz-sim8-dev libgz-transport13-dev libgz-sensors8-dev libgz-plugin2-dev \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager ros-jazzy-controller-interface ros-jazzy-hardware-interface \
    ros-jazzy-nav2-msgs ros-jazzy-xacro ros-jazzy-kdl-parser ros-jazzy-backward-ros \
    ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2 ros-jazzy-rqt-graph
echo 'System dependencies installed. Next: bash scripts/setup_px4.sh'
