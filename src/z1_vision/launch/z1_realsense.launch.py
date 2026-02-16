#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Parametri camera
    camera_x = LaunchConfiguration('camera_x', default='0.10')
    camera_y = LaunchConfiguration('camera_y', default='0.0')
    camera_z = LaunchConfiguration('camera_z', default='-0.02')
    
    # Quaternione camera (180° attorno Y)
    camera_qx = LaunchConfiguration('camera_qx', default='0.0')
    camera_qy = LaunchConfiguration('camera_qy', default='1.0')
    camera_qz = LaunchConfiguration('camera_qz', default='0.0')
    camera_qw = LaunchConfiguration('camera_qw', default='0.0')
    
    parent_frame = LaunchConfiguration('parent_frame', default='link06')
    
    # Package paths
    z1_gazebo_pkg = FindPackageShare('z1_gazebo')
    realsense_pkg = FindPackageShare('realsense2_camera')
    z1_vision_pkg = FindPackageShare('z1_vision')
    
    # RViz config
    rviz_config = PathJoinSubstitution([
        z1_vision_pkg, 'rviz', 'z1_realsense.rviz'
    ])
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('camera_x', default_value='0.10',
                             description='Camera X offset from parent frame'),
        DeclareLaunchArgument('camera_y', default_value='0.0',
                             description='Camera Y offset from parent frame'),
        DeclareLaunchArgument('camera_z', default_value='-0.02',
                             description='Camera Z offset from parent frame'),
        DeclareLaunchArgument('parent_frame', default_value='link06',
                             description='Parent frame for camera'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                             description='Launch RViz'),
        
        # 1. Z1 Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([z1_gazebo_pkg, 'launch', 'empty_world.launch.py'])
            ])
        ),
        
        # 2. RealSense camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([realsense_pkg, 'launch', 'rs_launch.py'])
            ]),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
                'colorizer.enable': 'false',
            }.items()
        ),
        
        # 3. Static TF: link06 → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                camera_x, camera_y, camera_z,
                camera_qx, camera_qy, camera_qz, camera_qw,
                parent_frame, 'camera_link'
            ],
            output='screen'
        ),
        
        # 4. Surface detection node
        Node(
            package='z1_control',
            executable='realsense_surface_node',
            name='realsense_surface_node',
            parameters=[{
                'ee_frame': 'link06',
                'camera_frame': 'camera_depth_optical_frame',
                'base_frame': 'world',
                'patch_radius_px': 30,
                'min_depth': 0.10,
                'max_depth': 2.0,
            }],
            output='screen'
        ),
        
        # 5. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=LaunchConfiguration('use_rviz'),
            output='screen'
        ),
    ])
