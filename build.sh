#!/usr/bin/env bash

set -e

source /opt/ros/jazzy/setup.bash

echo "Building workspace..."
colcon build --symlink-install

echo
echo "Build successful!"
echo "Run this to source the workspace:"
echo "source install/setup.bash"
