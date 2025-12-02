from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'wx250_bringup'
setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),
        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ishabh',
    maintainer_email='ishabh@todo.todo',
    description='Bringup + MoveIt (Move Group, RViz) + optional teleop for wx250',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_moveit_cmd = wx250_bringup.test_moveit_cmd:main',
            'servo_to_interbotix_bridge = wx250_bringup.servo_to_interbotix_bridge:main',
            'geomagic_servo = wx250_bringup.geomagic_servo:main',
            'custom_servo_node = wx250_bringup.custom_servo_node:main',
            'robot_to_geomagic_feedback = wx250_bringup.robot_to_geomagic_feedback:main',  # ← ADD THIS
        ],
    },
)
