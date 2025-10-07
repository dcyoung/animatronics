#!/bin/zsh
set -e

echo "Building workspace..."
colcon build

echo "Sourcing workspace..."
source install/setup.zsh

echo "Launching Wheatley..."
ros2 launch wheatley start_all.launch.py