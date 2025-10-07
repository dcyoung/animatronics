# Animatronics

Ros2 testing...

## Local

```bash
# Create ros env
conda create -n ros_env -c conda-forge -c robostack-jazzy ros-jazzy-desktop
conda activate ros_env
conda install -c conda-forge compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
conda install robostack-jazzy::ros-jazzy-xacro robostack-jazzy::ros-jazzy-joint-state-publisher robostack-jazzy::ros-jazzy-joint-state-publisher-gui
```

```bash
colcon build
source install/setup.zsh

ros2 launch wheatley start_all.launch.py
```
