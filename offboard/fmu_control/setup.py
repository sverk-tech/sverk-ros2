from setuptools import setup
import os
from glob import glob

package_name = 'fmu_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sverk',
    maintainer_email='petayyyy@gmail.com',
    description='PX4 calibration and quick disarm via VehicleCommand',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_control = fmu_control.calibration_control_node:main',
        ],
    },
)
