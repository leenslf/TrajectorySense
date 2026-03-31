# minimal_gazebo.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_moborobo_robot = get_package_share_directory('moborobo_robot')

    # World file argument — defaults to gravel.world, overridable via CLI:
    #   ros2 launch moborobo_robot minimal_gazebo.launch.py world_file:=/path/to/other.world
    default_world = os.path.join(pkg_moborobo_robot, 'world', 'gravel.world')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=default_world,
        description='Full path to the Gazebo world file to load'
    )
    world_file = LaunchConfiguration('world_file')

    return LaunchDescription([
        world_file_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': world_file,
                'gui': 'true',
                'server': 'true',
                'verbose': 'true'
            }.items()
        ),
        
        # Launch robot state publisher 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_moborobo_robot, 'launch', 'robot_state_publisher.launch.py')
            ])
        ),
        
        # Spawn robot after a delay
        TimerAction(
            period=5.0,  
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-topic', '/robot_description',
                        '-entity', 'moborobo_robot',
                        '-x', '0.0', '-y', '0.0', '-z', '0.2'
                    ],
                    output='screen'
                )
            ]
        ),
    ])