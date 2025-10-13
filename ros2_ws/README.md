# Animatronics

Ros2 testing...

## Local

```bash
conda create -n ros_humble -c conda-forge -c robostack-humble \
    ros-humble-desktop \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    libignition-gazebo6
# Gazebo Classic
    # ros-humble-gazebo-ros \
    # ros-humble-gazebo-ros-pkgs \
conda activate ros_humble
conda install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools rosdep
```

```bash
colcon build
source install/setup.zsh

ros2 launch wheatley start_all.launch.py
```
