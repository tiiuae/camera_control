#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'webcam.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    return LaunchDescription([
        Node(
            package='image_tools',
            namespace='camera',
            executable='cam2image',
            name='cam2image',
            parameters=[
                {"frequency": 1.0, "width": 640, "height": 480}
            ],
        ),
        Node(
            package='orbslam_node',
            executable='ros_mono',
            name='orb_slam',
            output='screen',
            arguments=[vocabulary_file,
                       camera_config_file],
        )
    ])


