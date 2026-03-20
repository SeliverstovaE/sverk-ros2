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
            cmd=['make', 'px4_sitl', 'gz_x500_lidar_2d'],
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

        # 3. Gazebo bridge for lidar data
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_bridge',
            arguments=['/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='point_bridge',
            arguments=['/world/default/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
            output='screen'
        ),
       
        # 4. Static transform for lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['0.0', '0.0', '0.2', '0', '0', '0', 'base_link', 'x500_lidar_2d_0/link/lidar_2d_v2'],
            output='screen'
        ),

        # # 5. Launch px4_local_pose_publisher
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
                
        # 6. Offboard control node
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