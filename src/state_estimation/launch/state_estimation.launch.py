from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg_share = get_package_share_directory('state_estimation')
    navsat = os.path.join(pkg_share, 'config', 'navsat.yaml')
    ekf_global = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/imu/data'),
            ('odometry/filtered', '/lio/odometry'),
            ('odometry/gps', '/gps/odom'),
        ],
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[ekf_global, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        navsat_node,
        ekf_global_node,
    ])
