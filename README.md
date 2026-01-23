## Overview
TrajectorySense is a ROS 2 workspace for simulating and evaluating a mobile robot in Gazebo. It includes the `moborobo_robot` package with robot description, meshes, controllers, and a gravel world, plus experiment scripts for running evaluation scenarios.

## Installation
```bash
git clone https://github.com/leenslf/TrajectorySense.git
```

## Build ROS2 Workspace

```bash
cd TrajectorySense
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

```bash
ros2 launch moborobo_robot minimal_gazebo.launch.py
```
