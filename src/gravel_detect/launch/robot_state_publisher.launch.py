# robot_state_publisher.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from xacro import process_file

def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('gravel_detect'),
        'description',
        'robot.urdf.xacro'
    )

    robot_description_config = process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
    ])
