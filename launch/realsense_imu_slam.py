#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir
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
    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'realsense_imu.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    rviz_config_dir = os.path.join(get_package_share_directory('camera_control'), 'config')
    xacro_path = os.path.join(get_package_share_directory('camera_control'), 'urdf', 'realsense.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true', 'name': 'camera'})

    return LaunchDescription(
        [
            Node(
                package='realsense2_camera',
                namespace='camera',
                name="camera",
                executable='realsense2_camera_node',
                parameters=[{'serial_no' : 'f0210811',
                             'unite_imu_method' : 'linear_interpolation',
                             'enable_color': True,
                             'enable_gyro' : True,
                             'enable_accel' : True,
                             'enable_infra' : False,
                             'enable_infra1' : False,
                             'enable_infra2' : False,
                             'enable_depth' : False,
                             'enable_pointcloud' : False,
                             'infra_rgb' : False,
                             'color_width' : 640,
                             'color_height' : 480,
                             'color_fps' : 6.0,
                             'gyro_fps' : 100.0,
                             'accel_fps' : 100.0,
                             }],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_dir],
                remappings=[('/camera/image', '/camera/color/image_raw'),
                            ('/imu', '/camera/imu')],
                parameters=[{'use_sim_time': False}]
            ),
            Node(
                name='model_node',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='screen',
                arguments=[urdf]
            ),

            # Node(
            #     package='orbslam_node',
            #     executable='ros_mono_inertial',
            #     name='orb_slam',
            #     output='screen',
            #     remappings=[('/camera/image', '/camera/color/image_raw'),
            #                 ('/imu', '/camera/imu')],
            #     arguments=[vocabulary_file,
            #                camera_config_file],
            # ),

        ])
