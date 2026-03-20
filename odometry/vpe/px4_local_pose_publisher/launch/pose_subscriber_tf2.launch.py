from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_cov_topic', default_value='aruco_map/pose_cov'),
        DeclareLaunchArgument('vehicle_local_position_topic', default_value='/fmu/out/vehicle_local_position'),

        DeclareLaunchArgument('camera_frame_id', default_value='camera_optical_1'),
        DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('aruco_map_frame_id', default_value='aruco_map_detected'),
        DeclareLaunchArgument('offset_frame_id', default_value='aruco_map'),

        DeclareLaunchArgument('data_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('offset_timeout', default_value='5.0'),
        DeclareLaunchArgument('allow_offset_timeout_reset', default_value='false'),
        DeclareLaunchArgument('allow_send_vio', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('fallback_variance', default_value='0.5'),

        Node(
            package='px4_local_pose_publisher',
            executable='pose_subscriber_tf2',
            name='pose_subscriber_tf2',
            output='screen',
            parameters=[{
                # Topics (declare_parameter in pose_subscriber_tf2)
                'pose_cov_topic': LaunchConfiguration('pose_cov_topic'),
                'vehicle_local_position_topic': LaunchConfiguration('vehicle_local_position_topic'),

                # Coordinate frames
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'aruco_map_frame_id': LaunchConfiguration('aruco_map_frame_id'),
                'offset_frame_id': LaunchConfiguration('offset_frame_id'),

                # Behavior
                'data_timeout_sec': ParameterValue(LaunchConfiguration('data_timeout_sec'), value_type=float),
                'offset_timeout': ParameterValue(LaunchConfiguration('offset_timeout'), value_type=float),
                'allow_offset_timeout_reset': ParameterValue(
                    LaunchConfiguration('allow_offset_timeout_reset'), value_type=bool),
                'allow_send_vio': ParameterValue(
                    LaunchConfiguration('allow_send_vio'), value_type=bool),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'fallback_variance': ParameterValue(LaunchConfiguration('fallback_variance'), value_type=float),
            }],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])