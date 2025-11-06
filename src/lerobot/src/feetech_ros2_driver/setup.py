from setuptools import setup
import os
from glob import glob

package_name = 'feetech_ros2_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'feetech_interfaces'],
    data_files=[
        #('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'msg'), glob('feetech_interfaces/msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zws',
    maintainer_email='zws@example.com',
    description='ROS2 driver for Feetech smart servos with topic interface.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feetech_driver_node = feetech_ros2_driver.feetech_driver_node:main',
            'feetech_interface_node = feetech_ros2_driver.feetech_interface_node:main',
        ],
    },
)
