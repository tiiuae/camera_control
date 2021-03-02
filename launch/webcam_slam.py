#!/usr/bin/env python3
import os
#import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import tempfile

def to_urdf(xacro_path, parameters=None):
    """Convert the given xacro file to URDF file.
    * xacro_path -- the path to the xacro file
    * parameters -- to be used when xacro file is parsed.
    """
    urdf_path = tempfile.mktemp(prefix="%s_" % os.path.basename(xacro_path))

    # open and process file
    doc = xacro.process_file(xacro_path, mappings=parameters)
    # open the output file
    out = xacro.open_output(urdf_path)
    out.write(doc.toprettyxml(indent='  '))

    return urdf_path


def generate_launch_description():

    urdf = os.path.join(os.path.join(get_package_share_directory('camera_control'), 'urdf'), 'camera.urdf')
    rviz_config_dir = os.path.join(get_package_share_directory('camera_control'), 'config')

    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'webcam.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output = 'screen',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            name='model_node',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='',
            output='screen',
            arguments = [urdf]
        ),
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


