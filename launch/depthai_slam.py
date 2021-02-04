#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'depthai.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    return LaunchDescription([
            Node(
                package='camera_control',
                namespace='camera',
                executable='depthai_node',
                name='depthai_node',
                parameters=[
                    {"frequency": 1.0}
                ]
            ),
            Node(
                package='orbslam_node',
                executable='mono',
                name='mono',
                arguments=[vocabulary_file,
                           camera_config_file]
            )
        ])


