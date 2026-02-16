#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_x', default_value='0.10'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='-0.02'),
        DeclareLaunchArgument('camera_qx', default_value='0.0'),
        DeclareLaunchArgument('camera_qy', default_value='1.0'),
        DeclareLaunchArgument('camera_qz', default_value='0.0'),
        DeclareLaunchArgument('camera_qw', default_value='0.0'),
        DeclareLaunchArgument('parent_frame', default_value='link06'),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                LaunchConfiguration('camera_x'),
                LaunchConfiguration('camera_y'),
                LaunchConfiguration('camera_z'),
                LaunchConfiguration('camera_qx'),
                LaunchConfiguration('camera_qy'),
                LaunchConfiguration('camera_qz'),
                LaunchConfiguration('camera_qw'),
                LaunchConfiguration('parent_frame'),
                'camera_link'
            ],
            output='screen'
        ),
    ])
