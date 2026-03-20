from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    params = {
        # Behavior
        'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
        'markers_topic': LaunchConfiguration('markers_topic'),
        'map_markers_topic': LaunchConfiguration('map_markers_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),

        # Outputs
        'pose_topic': LaunchConfiguration('pose_topic'),

        # TF
        'tf_parent_frame': LaunchConfiguration('tf_parent_frame'),
        'tf_child_frame': LaunchConfiguration('tf_child_frame'),
        'tf_map_frame': LaunchConfiguration('tf_map_frame'),
        'tf_markers_prefix': LaunchConfiguration('tf_markers_prefix'),
        'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
        'publish_static_markers_tf': ParameterValue(LaunchConfiguration('publish_static_markers_tf'), value_type=bool),
        'detect_individual_markers': ParameterValue(LaunchConfiguration('detect_individual_markers'), value_type=bool),
        'put_markers_count_to_covariance': ParameterValue(LaunchConfiguration('put_markers_count_to_covariance'), value_type=bool),

        # Frames
        'world_frame_id': LaunchConfiguration('world_frame_id'),
        'base_link_frame_id': LaunchConfiguration('base_link_frame_id'),
        'camera_frame_id': LaunchConfiguration('camera_frame_id'),

        # Vertical
        'known_vertical_frame': LaunchConfiguration('known_vertical_frame'),
        'flip_vertical': ParameterValue(LaunchConfiguration('flip_vertical'), value_type=bool),
        'auto_flip': ParameterValue(LaunchConfiguration('auto_flip'), value_type=bool),

        # Refinement & gating
        'refine_pose': ParameterValue(LaunchConfiguration('refine_pose'), value_type=bool),
        'gating.max_inlier_rmse_px': ParameterValue(LaunchConfiguration('gating_max_inlier_rmse_px'), value_type=float),

        # PnP/RANSAC tuning
        'pnp.iterations': ParameterValue(LaunchConfiguration('pnp_iterations'), value_type=int),
        'pnp.reprojection_error_px': ParameterValue(LaunchConfiguration('pnp_reprojection_error_px'), value_type=float),
        'pnp.confidence': ParameterValue(LaunchConfiguration('pnp_confidence'), value_type=float),
        'pnp.flags': ParameterValue(LaunchConfiguration('pnp_flags'), value_type=int),
        'pnp.min_used_markers': ParameterValue(LaunchConfiguration('pnp_min_used_markers'), value_type=int),
        'pnp.min_inlier_points': ParameterValue(LaunchConfiguration('pnp_min_inlier_points'), value_type=int),
    }

    return LaunchDescription([
        # Behavior
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('markers_topic', default_value='/markers'),
        DeclareLaunchArgument('map_markers_topic', default_value='/map_markers'),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info',
        ),

        # Outputs
        DeclareLaunchArgument('pose_topic', default_value='aruco_map/pose_cov'),

        # TF
        DeclareLaunchArgument('tf_parent_frame', default_value='main_camera_optical'),
        DeclareLaunchArgument('tf_child_frame', default_value='aruco_map_detected'),
        DeclareLaunchArgument('tf_map_frame', default_value='aruco_map'),
        DeclareLaunchArgument('tf_markers_prefix', default_value='aruco_'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_static_markers_tf', default_value='true'),
        DeclareLaunchArgument('detect_individual_markers', default_value='true'),
        DeclareLaunchArgument('put_markers_count_to_covariance', default_value='false'),

        # Frames
        DeclareLaunchArgument('world_frame_id', default_value='map'),
        DeclareLaunchArgument('base_link_frame_id', default_value='base_link'),
        DeclareLaunchArgument('camera_frame_id', default_value=''),

        # Vertical
        DeclareLaunchArgument('known_vertical_frame', default_value=''),
        DeclareLaunchArgument('flip_vertical', default_value='false'),
        DeclareLaunchArgument('auto_flip', default_value='false'),

        # Refinement & gating
        DeclareLaunchArgument('refine_pose', default_value='false'),
        DeclareLaunchArgument('gating_max_inlier_rmse_px', default_value='10.0'),

        # PnP/RANSAC
        DeclareLaunchArgument('pnp_iterations', default_value='150'),
        DeclareLaunchArgument('pnp_reprojection_error_px', default_value='4.0'),
        DeclareLaunchArgument('pnp_confidence', default_value='0.99'),
        DeclareLaunchArgument('pnp_flags', default_value='0'),  # SOLVEPNP_ITERATIVE is 0 in OpenCV
        DeclareLaunchArgument('pnp_min_used_markers', default_value='3'),
        DeclareLaunchArgument('pnp_min_inlier_points', default_value='4'),

        Node(
            package='aruco_det_loc',
            executable='aruco_loc_node',
            name='aruco_loc',
            output='screen',
            parameters=[params],
        ),
    ])
