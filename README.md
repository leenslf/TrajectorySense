## Installation
```bash
git clone https://github.com/leenslf/TrajectorySense.git
```

## Docker
(TODO: improve this)

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

