from setuptools import find_packages, setup

package_name = "servo_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/servo_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/servo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="ROS 2 servo control node based on Raspberry Pi hardware PWM.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "servo_node = servo_control.servo_node:main",
        ],
    },
)
