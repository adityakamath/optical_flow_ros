import os
from glob import glob
from setuptools import setup

package_name = 'pmw3901_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pmw3901', 'numpy'],
    zip_safe=True,
    maintainer='Aditya Kamath',
    maintainer_email='adityakamath@live.com',
    description='ROS 2 Node for Optical Flow Sensors - PMW3901 and the short range variant PAA5100',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optical_flow_publisher = pmw3901_ros.optical_flow_publisher:main',
            'optical_flow_node = pmw3901_ros.optical_flow_node:main',
        ],
    },
)
