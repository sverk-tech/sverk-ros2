from setuptools import find_packages, setup

package_name = "ld19_lidar"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/ld19.launch.py",
                "launch/ld19_usb.launch.py",
                "launch/ld19_uart.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/ld19.yaml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS 2 driver for the LDROBOT LD19 2D lidar.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ld19_lidar_node = ld19_lidar.ld19_lidar_node:main",
        ],
    },
)
