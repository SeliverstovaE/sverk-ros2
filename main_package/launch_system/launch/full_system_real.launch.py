#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
import getpass

def generate_launch_description():
    ENABLE_ARUCO = True          
    ENABLE_CAMERA = True         
    ENABLE_VIO = True            # Send ArUco to PX4
    ENABLE_MAVLINKROUTER = True  # Link to QC (mavlink-router)
    ENABLE_WEB = True          
    ENABLE_LED = True          

    # ===== CAMERA PARAMETERS =====
    CAMERA_SENSOR = 'ov5647'          # Sensor model
    CAMERA_WIDTH = 320                # Image width
    CAMERA_HEIGHT = 240               # Image height
    CAMERA_FPS = 25                   # Desired FPS
    CAMERA_ID = 1                     # Camera ID (multi-camera)
    IMAGE_TOPIC = f'/camera_{CAMERA_ID}/image_raw'          # Raw image topic
    CAMERA_INFO_TOPIC = f'/camera_{CAMERA_ID}/camera_info'  # Camera info topic
    CAMERA_FRAME_ID = f'camera_optical_{CAMERA_ID}'         # Camera frame_id for tf

    # Actions included in this launch
    actions = []

    # 1. MicroXRCEAgent
    actions.append(
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyAMA0', '-b', '921600'],
            name='micro_xrce_agent',
            output='screen',
            shell=False,
            respawn=True,
            respawn_delay=1.0,
        )
    )

    # 2. mavlink-routerd
    if ENABLE_MAVLINKROUTER:
        actions.append(
            ExecuteProcess(
                cmd=['mavlink-routerd', '/dev/ttyACM0:57600', '0.0.0.0:14550'],
                name='mavlink-routerd',
                output='screen',
                shell=False,
                respawn=True,
                respawn_delay=1.0,
            )
        )

    # 3. Camera driver
    if ENABLE_CAMERA:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('camera_ros'),
                        'launch',
                        'camera.launch.py'
                    ])
                ),
                launch_arguments={
                    'sensor': CAMERA_SENSOR,
                    'width': str(CAMERA_WIDTH),
                    'height': str(CAMERA_HEIGHT),
                    'fps': str(CAMERA_FPS),
                    'camera_id': str(CAMERA_ID),
                }.items()
            )
        )

    # 4. Aruco
    if ENABLE_ARUCO:
        # 4.1. Aruco map
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('aruco_map'),
                        'launch',
                        'aruco_map.launch.py'
                    ])
                ]),
                launch_arguments={'mode': 'debug'}.items()
            )
        )

        # 4.2. Aruco detect
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('aruco_det_loc'),
                        'launch',
                        'aruco_detect.launch.py'
                    ])
                ]),
                launch_arguments={
                    'mode': 'debug',
                    'image_topic': IMAGE_TOPIC,
                    'camera_info_topic': CAMERA_INFO_TOPIC
                }.items()
            )
        )

        # 4.3. Aruco localization
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('aruco_det_loc'),
                        'launch',
                        'aruco_loc.launch.py'
                    ])
                ]),
                launch_arguments={
                    'mode': 'debug',
                    'image_topic': IMAGE_TOPIC,
                    'camera_info_topic': CAMERA_INFO_TOPIC,
                    'tf_parent_frame': CAMERA_FRAME_ID
                }.items()
            )
        )
    
    # 5. PX4 local pose publisher
    if ENABLE_VIO:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('px4_local_pose_publisher'),
                        'launch',
                        'pose_subscriber_tf2.launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_frame_id': CAMERA_FRAME_ID
                }.items()
            )
        )   
    
    # 6. Offboard control node
    actions.append(
        Node(
            package='offboard_control',
            executable='offboard_control',
            name='offboard_control',
            parameters=[{'simulator': False}],
            output='screen'
        )
    )

    # 7. Web
    if ENABLE_WEB:
        # 7.1. Web video server
        actions.append(
            Node(
                package='web_video_server',
                executable='web_video_server',
                name='web_video',
                parameters=[],
                output='screen'
            )
        )

        # 7.2. Rosboard
        actions.append(
            Node(
                package='rosboard',
                executable='rosboard_node',
                name='rosboard',
                parameters=[],
                output='screen'
            )
        )

        # 7.3. ROS Services Bridge (HTTP API, port 9090)
        actions.append(
            Node(
                package='ros_services_bridge',
                executable='ros_services_bridge_node',
                name='ros_services_bridge',
                parameters=[],
                output='screen'
            )
        )
        
        # 7.4. Send command to FCU
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('fmu_calibration_control'),
                        'launch',
                        'calibration_control.launch.py'
                    ])
                ]),
                launch_arguments={}.items()
            )
        )

    # 8. Led control
    if ENABLE_LED:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('led_control'),
                        'launch',
                        'led.launch.py'
                    ])
                ]),
                launch_arguments={}.items()
            )
        )   
    return LaunchDescription(actions)

if __name__ == '__main__':
    generate_launch_description()