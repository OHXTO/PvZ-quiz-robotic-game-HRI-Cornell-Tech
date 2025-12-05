from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab5'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/game.launch.py']),
        ('share/' + package_name + '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maggie Liang, Xingtai Huang, Xueer Zhang',
    maintainer_email='ml2927@cornell.edu, xh457@cornell.edu, xz946@cornell.edu',
    description='ROS2 True/False robot game',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control_architecture_node = lab5.robot_control_architecture_node:main',
            'speaker_test = lab5.speaker_test:main',
            'color_detector = lab5.color_detector:main',
            'game_controller = lab5.game_controller:main',
            'stt_listener = lab5.stt_listener:main',
            'stt_speed_controller = lab5.stt_speed_controller:main',
            'color_detector_node = lab5.color_detector_node:main',
            'color_game_node = lab5.color_game_node:main',


        ],
    },
)
