from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_mcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'fastapi>=0.111',
        'uvicorn[standard]>=0.29',
        'pydantic>=2.7',
    ],
    zip_safe=True,
    maintainer='Petayyyy',
    maintainer_email='petayyyy@gmail.com',
    description='ROS2 MCP HTTP server for picoclaw integration',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ros2_mcp_server = ros2_mcp.server:main',
        ],
    },
)
