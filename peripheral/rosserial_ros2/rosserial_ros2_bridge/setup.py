from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rosserial_ros2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["generated_interfaces.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sverk",
    maintainer_email="user@example.com",
    description="ROS 2 bridge for Arduino sketches that use the rosserial protocol.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "bridge_node = rosserial_ros2_bridge.bridge_node:main",
            "generate_ros_lib = rosserial_ros2_bridge.generate_ros_lib:main",
            "buffer_size = rosserial_ros2_bridge.buffer_size:main",
        ],
    },
)
