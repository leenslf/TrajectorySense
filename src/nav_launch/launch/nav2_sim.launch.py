from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_path = os.path.join(get_package_share_directory('nav_launch'), 'config', 'nav_params_sim.yaml')

    launch_nav2_declaration = DeclareLaunchArgument(
        "launch_nav2",
        default_value="false",
        description='whether to start nav2 or not'
    )
    launch_nav2 = LaunchConfiguration("launch_nav2")

   

    # costmap_compressor =Node(
    #     package='tbt_otonom_compression',
    #     executable='compressor.py',
    #     name='compressor',
    #     condition=IfCondition(launch_nav2)
    # )

    # traversability = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('tbt_otonom_traversability'), 'launch', 'voxel_filter_polar.launch.py')),
    #     condition=IfCondition(launch_nav2)
    # )
    
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav_launch'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'autostart': 'true',
                'use_sim_time': 'true', # hardcoded here!
                'params_file': params_path,
            }.items(),
            condition=IfCondition(launch_nav2)
        )

    nav2 = TimerAction(
        period=3.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        launch_nav2_declaration,
        # costmap_compressor,
        # traversability,
        nav2
    ])