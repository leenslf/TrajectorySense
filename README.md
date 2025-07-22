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
git clone https://github.com/leenslf/TrajectorySense.git
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

## Build ROS2 Workspace

```bash
cd TrajectorySense
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

```bash
  ros2 launch gravel_detect minimal_gazebo.launch.py
```

## IMU Data Collection & Analysis

You can use `rosbag2` to record `/imu/data` and analyze it using the provided Python scripts for extraction and visualization.

## License

MIT License
