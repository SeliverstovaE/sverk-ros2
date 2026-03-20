#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import getpass

def generate_launch_description():
    # User home directory
    home_dir = os.path.expanduser(f'~{getpass.getuser()}')
    px4_dir = os.path.join(home_dir, 'PX4-Autopilot')

    # Check PX4 directory exists
    if not os.path.exists(px4_dir):
        print(f"WARNING: PX4 directory not found at {px4_dir}. Make sure it exists.")

    return LaunchDescription([
        # 1. PX4 SITL with Gazebo (separate process from PX4 tree)
        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gz_x500_mono_cam_down_aruco'],
            name='px4_sitl_gazebo',
            output='screen',
            cwd=px4_dir,
            shell=True
        ),
        
        # 2. MicroXRCEAgent (separate process)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            name='micro_xrce_agent',
            output='screen',
            shell=True
        ),

        # 3. Gazebo bridge for image
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=['/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),
        
        # 4. Gazebo bridge for camera_info
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=['/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            output='screen'
        ),
        
        # 5. Static transform for camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_publisher',
            # arguments=['0', '0', '-0.10', '-1.5707963', '0', '3.1415926', 'base_link', 'main_camera_optical'],
            arguments=['0', '0', '0.0', '-1.5707963', '0', '3.1415926', 'base_link', 'main_camera_optical'],
            output='screen'
        ),
       
        # 6. Aruco map node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_map'),
                    'launch',
                    'aruco_map.launch.py'
                ])
            ]),
            launch_arguments={
                'mode': 'debug'
            }.items()
        ),


        # 7. Aruco detect node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_det_loc'),
                    'launch',
                    'aruco_detect.launch.py'
                ])
            ]),
        ),

        # 8. Launch aruco_det_loc
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('aruco_det_loc'),
                    'launch',
                    'aruco_loc.launch.py'
                ])
            ]),
            launch_arguments={
                'mode': 'debug'
            }.items()
        ),
        
        # # 9. Launch px4_local_pose_publisher
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('px4_local_pose_publisher'),
        #             'launch',
        #             'pose_subscriber_tf2.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={}.items()
        # ),   
                
        # 10. Offboard control node
        Node(
            package='offboard_control',
            executable='offboard_control',
            name='offboard_control',
            parameters=[
                {'simulator': True}
            ],
            output='screen'
        ),     
    ])

if __name__ == '__main__':
    generate_launch_description()