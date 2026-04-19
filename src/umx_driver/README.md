# UM7 ROS 2 Driver Status (2026-04-19)

## Overview
This workspace includes a local ROS 2 Humble package named `umx_driver` that connects to a Redshift Labs UM7 over serial (`/dev/ttyUSB0`) and publishes standard sensor topics.

## How It Works
- Node: `umx_driver_node`
- Launch: `launch/um7_launch.py`
- Transport: `rsl_comm_py.UM7Serial` at 115200 baud
- Data flow:
  1. Opens serial connection to UM7
  2. Reads broadcast packets from the sensor
  3. Maps UM7 processed data to ROS messages
  4. Publishes `/imu/data` and `/imu/mag`

## Parameters (current launch defaults)
- `port`: `/dev/ttyUSB0`
- `baud`: `115200`
- `frame_id`: `imu_link`
- `tf_ned_to_enu`: `true`
- `mag_updates`: `true`
- `zero_gyros`: `true`

## Current Outputs
- `/imu/data` (`sensor_msgs/Imu`)
  - Orientation (quaternion)
  - Angular velocity (rad/s)
  - Linear acceleration (m/s^2)
- `/imu/mag` (`sensor_msgs/MagneticField`)
  - Magnetic field (Tesla)

## Observed Frequency
- `/imu/data`: stable around **10.8 Hz** (measured with `ros2 topic hz /imu/data`)
- Expected target in original task was 20-100 Hz; current sensor stream appears capped near 10-11 Hz in this environment.

## Important Notes
- Serial detection and permissions were validated (`/dev/ttyUSB0`, user in `dialout`).
- NED to ENU transform is enabled in-node (`tf_ned_to_enu=true`).
- Orientation is populated via quaternion stream with Euler fallback.
- The lower publish rate appears to come from current UM7 device/firmware output behavior, not ROS publish throttling.

## Quick Run
```bash
ros2 launch umx_driver um7_launch.py
```

## Quick Checks
```bash
ros2 topic list | grep imu
ros2 topic echo /imu/data --once
ros2 topic echo /imu/data --once --field header
ros2 topic echo /imu/data --once --field orientation
ros2 topic echo /imu/data --once --field angular_velocity
ros2 topic echo /imu/data --once --field linear_acceleration

ros2 topic echo /imu/mag --once
ros2 topic hz /imu/data
```
