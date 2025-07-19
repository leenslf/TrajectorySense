from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    moborobo_description_path = PathJoinSubstitution([
        FindPackageShare('gravel_detect'),
        'description',
        'robot.urdf.xacro'
    ])

    world_path = PathJoinSubstitution([
        FindPackageShare('gravel_detect'),
        'world',
        'gravel.world'
    ])

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        moborobo_description_path
    ])

    # Launch Description
    return LaunchDescription([
        # Robot description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Include Gazebo empty world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={'world': world_path}.items()
        ),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'gravel_detect',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ],
            output='screen'
        )
    ])
