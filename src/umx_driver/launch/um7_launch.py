from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="umx_driver",
                executable="umx_driver_node",
                name="um7_driver",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "baud": 115200,
                        "update_rate": 100,
                        "frame_id": "imu_link",
                        "tf_ned_to_enu": True,
                        "mag_updates": False,
                        "zero_gyros": True,
                    }
                ],
                output="screen",
                respawn=True,
            )
        ]
    )
