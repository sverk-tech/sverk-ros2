from setuptools import setup, find_packages

package_name = 'ros_uwb'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages() + find_packages(where='pyuwb/src'),
    package_dir={'': '.', 'pyuwb': 'pyuwb/src/pyuwb'},
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
    ],
    zip_safe=True,
)
