## Overview
TrajectorySense is a ROS 2 workspace for simulating and evaluating a mobile robot in Gazebo. It includes the `moborobo_robot` package with robot description, meshes, controllers.

## How This Repo Fits Together (Tested with ig_lio)
This workspace is designed to work with [ig_lio](https://github.com/leenslf/ig_lio/tree/restarting) as the local odometry source. The simulated robot publishes LiDAR and IMU data from Gazebo, ig_lio consumes those topics to produce `/lio/odometry`, and the rest of the stack builds on top of that.

Tested path:
- Gazebo simulation from `moborobo_robot`
- ig_lio running with the Velodyne config to produce `/lio/odometry`

## Setup
*Check [this](https://github.com/leenslf/ig_lio_ros2_workspace) for easy full setup*

```bash
git clone https://github.com/leenslf/TrajectorySense.git
```


```bash
cd TrajectorySense
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

```bash
ros2 launch moborobo_robot minimal_gazebo.launch.py
```

## State Estimation (LIO + GPS)

Launch the standalone state estimation pipeline (local EKF + navsat transform + global EKF):

```bash
ros2 launch state_estimation state_estimation.launch.py use_sim_time:=true
```

Inputs/outputs:
- Inputs: `/lio/odometry`, `/imu/data`, `/gps/fix`
- Outputs: `/odometry/filtered`, `/gps/odom`, `map -> odom` TF

## Nav 2 (work in progress)

Launch Nav2 with the sim params:

```bash
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true
```
