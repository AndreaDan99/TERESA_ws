#!/usr/bin/env python3
"""
Z1 Vision Full Launch
Avvia: YOLO Torso Tracker + RealSense Surface Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    use_surface_node = LaunchConfiguration('use_surface_node')

    config_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'impedance_control_params.yaml'
    ])

    # ── NUOVO: YAML parametri torso tracker ───────────────────────
    torso_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'z1_yolo_torso_params.yaml'
    ])

    # =========================================================
    # NODO 1: YOLO Torso Tracker
    # =========================================================
    yolo_tracker_node = Node(
        package='z1_vision',
        executable='z1_yolo_torso_tracker',
        name='z1_yolo_torso_tracker',
        parameters=[torso_params_file],   # ← carica da YAML
        output='screen'
    )

    # =========================================================
    # NODO 2: Surface Detection Node
    # =========================================================
    surface_node = Node(
        package='z1_vision',
        executable='realsense_surface_node',
        name='realsense_surface_node',
        parameters=[{
            'ee_frame':        'link06',
            'camera_frame':    'camera_depth_optical_frame',
            'base_frame':      'world',
            'patch_radius_px': 30,
            'min_depth':       0.05,
            'max_depth':       2.0,
        }],
        output='screen',
        condition=IfCondition(use_surface_node)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_surface_node',
            default_value='true',
            description='Launch surface detection node'
        ),
        yolo_tracker_node,
        surface_node,
    ])
