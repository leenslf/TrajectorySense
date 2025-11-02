from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gps_fusion = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=['config/navsat.yaml'],
        remappings=[
            ('/odometry/filtered', '/odom'),
            ('/gps/fix', '/gps/fix'),
            ('/imu/data', '/imu/data')
        ],

        output='screen'
    )

    return LaunchDescription([gps_fusion])
