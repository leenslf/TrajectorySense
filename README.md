# Gravel Simulation for Trajectory Sense

```
conda deactivate
git clone https://github.com/MAli7319/TrajectorySense.git
colcon build --symlink-install
source install/setup.bash
```

### First terminal
```
ros2 launch gazebo_ros gazebo.launch.py world:=/home/{your_username}/{your_workspace}/src/gravel_detect/world/gravel.world
```

### Second terminal
```
ros2 run robot_state_publisher robot_state_publisher   --ros-args -p robot_description:="$(xacro ~/{your_workspace}/src/gravel_detect/description/robot.urdf.xacro)"
```

### Third terminal
```
ros2 run gazebo_ros spawn_entity.py   -topic /robot_description   -entity gravel_detect   -x 0.5 -y 1 -z 0.2
```
