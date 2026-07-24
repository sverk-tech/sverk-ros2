from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'self_check'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sverk',
    maintainer_email='petayyyymail@gmail.com',
    description='Диагностика дрона: нода-оркестратор self_check (пассивные и интерактивные проверки, ROS action API).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_check_node = self_check.node:main',
            'selfcheck = self_check.cli:main',
        ],
    },
)
