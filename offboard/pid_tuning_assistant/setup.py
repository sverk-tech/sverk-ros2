from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pid_tuning_assistant'


def _collect_tree(src_dir):
    """Recursively map a source tree to (install_dir, [files]) data_files entries.

    A flat ``glob('static/*')`` silently skips subdirectories, so the moment the
    web UI grows a ``static/js/`` or ``static/css/`` folder those assets would
    vanish from the installed share/ tree with no error. Walk the tree instead
    so every file is installed under the mirrored share/<pkg>/<path> layout.
    """
    entries = []
    for root, _dirs, files in os.walk(src_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        entries.append((install_dir, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'examples'), glob('examples/*')),
    ] + _collect_tree('static'),
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
        'websockets',
        'numpy',
        'scipy',
        'pandas',
        'pyarrow',
        # Mandatory: FastAPI needs it to parse multipart UploadFile bodies
        # (/api/params, /api/ulog/upload). Kept in install_requires, not extras.
        'python-multipart',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Read-only PX4 PID tuning assistant',
    license='MIT',
    extras_require={
        'ulog': ['pyulog>=0.4.0'],
    },
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_tuning_assistant = pid_tuning_assistant.main:main',
        ],
    },
)
