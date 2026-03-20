from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_node(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    is_debug = (mode == 'debug')

    params = {
        # Inputs
        'map_markers_topic': LaunchConfiguration('map_markers_topic'),
        'image_topic': LaunchConfiguration('image_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),

        # parent frame for published markers / visualization
        'parent_frame_id': LaunchConfiguration('parent_frame_id'),

        # ArUco
        'marker_size': ParameterValue(LaunchConfiguration('marker_size'), value_type=float),
        'dictionary_id': ParameterValue(LaunchConfiguration('dictionary_id'), value_type=int),

        # Single-solvePnP behavior toggle
        'estimate_marker_pose': ParameterValue(
            LaunchConfiguration('estimate_marker_pose'), value_type=bool
        ),

        # solvePnP FPS cap
        'pnp_fps_cap_enabled': ParameterValue(
            LaunchConfiguration('pnp_fps_cap_enabled'), value_type=bool
        ),
        'pnp_fps_cap': ParameterValue(
            LaunchConfiguration('pnp_fps_cap'), value_type=int
        ),

        # Marker debug image + RViz visualization (mode-controlled)
        'debug_image_topic': LaunchConfiguration('debug_image_topic'),
        'publish_debug_image': is_debug,
        'publish_visualization': is_debug,

        # world-axes debug image driven by /aruco_map/pose_cov (mode-controlled)
        'publish_world_debug_image': is_debug,
        'world_debug_image_topic': LaunchConfiguration('world_debug_image_topic'),
        'pose_cov_topic': LaunchConfiguration('pose_cov_topic'),
        'world_axes_length': ParameterValue(LaunchConfiguration('world_axes_length'), value_type=float),
        'world_axes_thickness': ParameterValue(LaunchConfiguration('world_axes_thickness'), value_type=int),
        'pose_cov_is_world_to_camera': ParameterValue(
            LaunchConfiguration('pose_cov_is_world_to_camera'), value_type=bool
        ),
        'world_pose_max_age_sec': ParameterValue(
            LaunchConfiguration('world_pose_max_age_sec'), value_type=float
        ),
    }

    return [Node(
        package='aruco_det_loc',
        executable='aruco_detect_node',
        name='aruco_detect',
        output='screen',
        parameters=[params],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='normal',
            description="Run mode: 'normal' or 'debug' (debug enables marker debug image + RViz viz + world-axes debug image)",
        ),

        DeclareLaunchArgument('map_markers_topic', default_value='/map_markers'),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info',
        ),

        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='main_camera_optical',
            description="Frame id used as parent for published marker array + RViz visualization.",
        ),

        DeclareLaunchArgument('marker_size', default_value='0.5'),
        DeclareLaunchArgument('dictionary_id', default_value='3'),

        # Pose estimation
        DeclareLaunchArgument(
            'estimate_marker_pose',
            default_value='false',
            description="If true, compute per-marker pose (solvePnP) and publish it (and TF if send_tf is enabled in node).",
        ),
        DeclareLaunchArgument(
            'pnp_fps_cap_enabled',
            default_value='true',
            description="If true, limit how often solvePnP runs (global cap).",
        ),
        DeclareLaunchArgument(
            'pnp_fps_cap',
            default_value='10',
            description="Max solvePnP rate in Hz (must be > 0 when pnp_fps_cap_enabled is true).",
        ),

        # Marker debug image
        DeclareLaunchArgument('debug_image_topic', default_value='/aruco_det/debug_image'),

        # world-axes debug image
        DeclareLaunchArgument('world_debug_image_topic', default_value='/aruco_det/world_debug_image'),
        DeclareLaunchArgument('pose_cov_topic', default_value='/aruco_map/pose_cov'),
        DeclareLaunchArgument('world_axes_length', default_value='0.5'),
        DeclareLaunchArgument('world_axes_thickness', default_value='3'),
        DeclareLaunchArgument(
            'pose_cov_is_world_to_camera',
            default_value='false',
            description="false: pose_cov pose is camera pose in world (T_world_camera); true: pose is world->camera (T_camera_world)",
        ),
        DeclareLaunchArgument(
            'world_pose_max_age_sec',
            default_value='0.2',
            description="Max |image_stamp - pose_stamp| to draw world axes; <=0 disables gating",
        ),

        OpaqueFunction(function=_make_node),
    ])
