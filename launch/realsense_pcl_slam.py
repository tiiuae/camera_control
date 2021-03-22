#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    rviz_config_dir = os.path.join(get_package_share_directory('camera_control'), 'config')
    xacro_path = os.path.join(get_package_share_directory('camera_control'), 'urdf', 'realsense_depth_test.urdf')
    urdf_string = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true', 'name': 'camera'})

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
                             'enable_gyro' : False,
                             'enable_accel' : False,
                             'enable_infra' : False,
                             'enable_infra1' : False,
                             'enable_infra2' : False,
                             'enable_confidence' : False,
                             'enable_depth' : True,
                             'enable_pointcloud' : True,
                             'depth_fps': 6.0,
                             'infra_rgb' : False,
                             }],
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_dir],
                remappings=[('/camera/image', '/camera/color/image_raw')],
                #parameters=[{'use_sim_time': False}]
            ),
            Node(
                name='model_node',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='screen',
                arguments=[urdf_string]
            ),

            # Node(
            #     package='orbslam_node',
            #     executable='ros_mono',
            #     name='orb_slam',
            #     output='screen',
            #     remappings=[('/camera/image', '/camera/color/image_raw')],
            #     arguments=[vocabulary_file,
            #                camera_config_file],
            # ),

        ])
