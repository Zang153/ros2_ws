import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dynamixel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotic',
    maintainer_email='robotic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_controller = dynamixel_control.motor_controller:main',
            'scan_dynamixel = dynamixel_control.scan_dynamixel:main',
            'visualization_node = dynamixel_control.visualization_node:main',
            'joint_control_node = dynamixel_control.joint_control_node:main',
            'vrpn_display_node = dynamixel_control.vrpn_display_node:main',
        ],
    },
)
