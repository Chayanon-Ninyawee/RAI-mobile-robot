#!/usr/bin/env bash

set -e

cd "$(dirname "$0")"

source /opt/ros/jazzy/setup.bash

echo "==> Checking required tools..."
if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed. Rebuild the Docker image."
    exit 1
fi

echo "==> Updating APT package lists..."
apt-get update

echo "==> Checking Cartographer availability..."
if ! apt-cache show ros-jazzy-cartographer-ros >/dev/null 2>&1; then
    echo "ERROR: ros-jazzy-cartographer-ros is unavailable."
    echo "Check APT repositories and package indexes."
    exit 1
fi

echo "==> Installing ROS dependencies..."
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -r -y

echo "==> Building workspace..."
colcon build --symlink-install

echo "==> Done!"
echo "Run: source install/setup.bash"
