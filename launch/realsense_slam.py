#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'realsense.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'luanch',
        'rs_launch.py')

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            namespace='camera',
            executable='rs_launch.py',
            name='rs_camera',
            parameters=[
                {}
            ],
            remappings=[]
        ),
        Node(
            package='orbslam_node',
            executable='ros_mono',
            name='orb_slam',
            output='screen',
            remappings = [('/camera/image', '/camera/color/image_raw')],
            arguments=[vocabulary_file,
                       camera_config_file],
        ),
        Node(
            package='orbslam_node',
            executable='ros_mono',
            name='orb_slam',
            output='screen',
            remappings = [('/camera/image', '/camera/color/image_raw')],
            arguments=[vocabulary_file,
                       camera_config_file],
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
        #     #launch_arguments={'camera_name': 'camera'}.items(),
        # ),

    ])


