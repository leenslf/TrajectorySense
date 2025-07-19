# Gravel Simulation for TrajectorySense

This project simulates a robot navigating a gravel road in Gazebo, enabling collection and analysis of IMU (Inertial Measurement Unit) sensor data. The goal is to evaluate the effects of rough terrain on the robot's trajectory using ROS 2 Humble and Gazebo.

## Features

- **Custom gravel terrain** generated in Blender and imported into Gazebo.
- **URDF-based mobile robot** with IMU sensor.
- **Data collection** using `rosbag2` for offline IMU analysis.
- **Post-processing and plotting** of IMU data for motion analysis.

## Demo

[![Watch the video](https://img.youtube.com/vi/XXs3yhgwSZE/0.jpg)](https://www.youtube.com/watch?v=XXs3yhgwSZE)

## Installation

```bash
git clone https://github.com/MAli7319/TrajectorySense.git
cd TrajectorySense
colcon build --symlink-install
source install/setup.bash
```

## Docker
Build Docker image:
```bash
chmod +x *.sh
cd util
./build_docker.sh
```

Run container:
```bash
cd util
./run_docker.sh
```
Excute container:
```bash
docker exec -it trajectorysense-container bash
```

## Running the Simulation

Replace `{your_username}` and `{your_workspace}` with your actual user name and workspace folder.

### 1. Launch Gazebo with the Gravel World

**First terminal:**

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/home/{your_username}/{your_workspace}/src/gravel_detect/world/gravel.world
```

### 2. Start the Robot State Publisher

**Second terminal:**

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro ~/{your_workspace}/src/gravel_detect/description/robot.urdf.xacro)"
```

### 3. Spawn the Robot into the World

**Third terminal:**

```bash
ros2 run gazebo_ros spawn_entity.py \
  -topic /robot_description \
  -entity gravel_detect \
  -x 0.5 -y 1 -z 0.2
```

## IMU Data Collection & Analysis

You can use `rosbag2` to record `/imu/data` and analyze it using the provided Python scripts for extraction and visualization.

## License

MIT License
