from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('nav_launch')
    config_file = os.path.join(pkg_share, 'config', 'navsat.yaml')

    gps_fusion = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[config_file],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('/gps/fix', '/gps/fix'),
            ('/imu/data', '/imu/data')
        ],
        output='screen'
    )

    return LaunchDescription([gps_fusion])

# currently this publishes → /odometry/gps  but still no map->odom tf!
# ig lio's topic has no twist info! so we use use the odom topic published by the controller. 