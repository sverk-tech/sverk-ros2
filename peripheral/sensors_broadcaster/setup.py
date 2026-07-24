from setuptools import find_packages, setup

package_name = 'sensors_broadcaster'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vl53_scan_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Sensor data broadcasting (ROS 2)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multi_vl53_node = sensors_broadcaster.multi_vl53_node:main',
            'vl53_scan_fuser_node = sensors_broadcaster.vl53_scan_fuser_node:main',
        ],
    },
)
