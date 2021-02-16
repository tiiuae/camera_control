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


configurable_parameters = [{'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no', 'default': '', 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id', 'default': '', 'description': 'choose device by usb port id'},
                           {'name': 'device_type', 'default': '', 'description': 'choose device by type'},
                           {'name': 'config_file', 'default': '', 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud', 'default': 'false', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method', 'default': '', 'description': '[copy|linear_interpolation]'},
                           {'name': 'json_file_path', 'default': '', 'description': 'allows advanced configuration'},
                           {'name': 'output', 'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_width', 'default': '640', 'description': 'depth image width'},
                           {'name': 'depth_height', 'default': '480', 'description': 'depth image height'},
                           {'name': 'enable_depth', 'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'color_width', 'default': '640', 'description': 'color image width'},
                           {'name': 'color_height', 'default': '480', 'description': 'color image height'},
                           {'name': 'enable_color', 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'infra_width', 'default': '640', 'description': 'infra width'},
                           {'name': 'infra_height', 'default': '480', 'description': 'infra width'},
                           {'name': 'enable_infra1', 'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2', 'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb', 'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width', 'default': '848', 'description': 'fisheye width'},
                           {'name': 'fisheye_height', 'default': '800', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1', 'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2', 'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width', 'default': '640', 'description': 'depth image width'},
                           {'name': 'confidence_height', 'default': '480', 'description': 'depth image height'},
                           {'name': 'enable_confidence', 'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps', 'default': '30.', 'description': ''},
                           {'name': 'depth_fps', 'default': '30.', 'description': ''},
                           {'name': 'confidence_fps', 'default': '30.', 'description': ''},
                           {'name': 'infra_fps', 'default': '30.', 'description': ''},
                           {'name': 'color_fps', 'default': '6.', 'description': ''},
                           {'name': 'gyro_fps', 'default': '400.', 'description': ''},
                           {'name': 'accel_fps', 'default': '250.', 'description': ''},
                           {'name': 'color_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'confidence_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'depth_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'fisheye_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'infra_qos', 'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'enable_gyro', 'default': 'false', 'description': ''},
                           {'name': 'enable_accel', 'default': 'false', 'description': ''},
                           {'name': 'pointcloud_texture_stream', 'default': 'RS2_STREAM_COLOR',
                            'description': 'testure stream for pointcloud'},
                           {'name': 'pointcloud_texture_index', 'default': '0',
                            'description': 'testure stream index for pointcloud'},
                           {'name': 'enable_sync', 'default': 'false', 'description': ''},
                           {'name': 'align_depth', 'default': 'false', 'description': ''},
                           {'name': 'filters', 'default': '', 'description': ''},
                           {'name': 'clip_distance', 'default': '-2.', 'description': ''},
                           {'name': 'linear_accel_cov', 'default': '0.01', 'description': ''},
                           {'name': 'initial_reset', 'default': 'false', 'description': ''},
                           {'name': 'allow_no_texture_points', 'default': 'false', 'description': ''},
                           {'name': 'ordered_pc', 'default': 'false', 'description': ''},
                           {'name': 'calib_odom_file', 'default': '', 'description': ''},
                           {'name': 'topic_odom_in', 'default': '', 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate', 'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename', 'default': '',
                            'description': 'A realsense bagfile to run from as a device'},
                           ]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for
            param in parameters]


def generate_launch_description():
    camera_config_file = os.path.join(
        get_package_share_directory('camera_control'),
        'realsense.yaml')

    vocabulary_file = os.path.join(
        get_package_share_directory('orbslam_node'),
        'Vocabulary',
        'ORBvoc.txt')

    rviz_config_dir = os.path.join(get_package_share_directory('camera_control'), 'config')
    xacro_path = os.path.join(get_package_share_directory('camera_control'), 'urdf', 'realsense.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true', 'name': 'camera'})

    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
            Node(
                package='realsense2_camera',
                namespace='camera',
                name="camera",
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)],
                output='screen',
                # emulate_tty=True,
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_dir],
                remappings=[('/camera/image', '/camera/color/image_raw')],
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

            Node(
                package='orbslam_node',
                executable='ros_mono',
                name='orb_slam',
                output='screen',
                remappings=[('/camera/image', '/camera/color/image_raw')],
                arguments=[vocabulary_file,
                           camera_config_file],
            ),

        ])
