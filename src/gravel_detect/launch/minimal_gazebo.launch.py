# minimal_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_gravel_detect = get_package_share_directory('gravel_detect')
    
    # World file path
    world_file = os.path.join(pkg_gravel_detect, 'world', 'gravel.world')
    
    return LaunchDescription([
        # Launch Gazebo with world
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', f'world:={world_file}'],
            output='screen'
        ),
        
        # Launch robot state publisher 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_gravel_detect, 'launch', 'robot_state_publisher.launch.py')
            ])
        ),
        
        # Spawn robot after a delay
        TimerAction(
            period=3.0,  # Wait 3 seconds for Gazebo to start
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-topic', '/robot_description',
                        '-entity', 'gravel_detect',
                        '-x', '0.5', '-y', '1.0', '-z', '0.2'
                    ],
                    output='screen'
                )
            ]
        ),
    ])