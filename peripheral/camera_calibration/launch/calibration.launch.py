from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = {
        'image_topic': LaunchConfiguration('image_topic'),
        'output_dir': LaunchConfiguration('output_dir'),
        'camera_name': LaunchConfiguration('camera_name'),
        'frame_id': LaunchConfiguration('frame_id'),
        'preview_rate_hz': ParameterValue(LaunchConfiguration('preview_rate_hz'), value_type=float),
        'publish_preview_when_idle': ParameterValue(
            LaunchConfiguration('publish_preview_when_idle'), value_type=bool
        ),
        'min_charuco_corners': ParameterValue(LaunchConfiguration('min_charuco_corners'), value_type=int),
    }

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera_1/image_raw'),
        DeclareLaunchArgument('output_dir', default_value='~/camera_calibrations'),
        DeclareLaunchArgument('camera_name', default_value='calibrated_camera'),
        DeclareLaunchArgument('frame_id', default_value='camera_optical_1'),
        DeclareLaunchArgument('preview_rate_hz', default_value='3.0'),
        DeclareLaunchArgument('publish_preview_when_idle', default_value='true'),
        DeclareLaunchArgument('min_charuco_corners', default_value='10'),

        Node(
            package='camera_calibration',
            executable='camera_calibration_node.py',
            name='camera_calibration',
            output='screen',
            parameters=[params],
        ),
    ])