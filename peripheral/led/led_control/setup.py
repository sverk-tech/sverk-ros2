from setuptools import find_packages, setup

package_name = "led_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/led.launch.py"]),
        ("share/" + package_name + "/config", ["config/led_params.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    # extras_require={"spidev": ["spidev"], "uart": ["pyserial"]},
    zip_safe=True,
    maintainer="Niklas Rousset",
    maintainer_email="byrousset@gmail.com",
    description="ROS2 node for WS2812 LED strip (PX4 event visualization, set_effect, set_leds)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "led_node = led_control.led_node:main",
        ],
    },
)
