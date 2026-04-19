from setuptools import setup

package_name = "umx_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/um7_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UM7 Integrator",
    maintainer_email="user@localhost",
    description="Minimal ROS 2 UM7 driver publishing /imu/data and /imu/mag.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "umx_driver_node = umx_driver.umx_driver_node:main",
        ],
    },
)
