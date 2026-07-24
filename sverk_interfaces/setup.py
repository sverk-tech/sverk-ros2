from setuptools import setup

package_name = "sverk_interfaces"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sverk",
    maintainer_email="petayyyy@gmail.com",
    description=(
        "High-level Python client library for sverk_drone: offboard_control, "
        "fmu_control and optional led_control (LED strip) ROS 2 nodes."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)

