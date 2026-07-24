from setuptools import find_packages, setup

package_name = "serial_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/arduino_servo.launch.py"]),
        ("share/" + package_name + "/config", ["config/arduino_servo.yaml"]),
        (
            "share/" + package_name + "/examples/serial_servo_bridge_d2",
            ["examples/serial_servo_bridge_d2/serial_servo_bridge_d2.ino"],
        ),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Reusable Python serial transport helpers and example ROS 2 bridge nodes.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arduino_servo_node = serial_bridge.arduino_servo_node:main",
        ],
    },
)
