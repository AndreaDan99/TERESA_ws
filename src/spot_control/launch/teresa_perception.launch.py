#!/usr/bin/env python3
"""
Teresa Perception Launch
=========================
Avvia contemporaneamente spot_perception (Orbbec) e z1_perception (RealSense).

Uso:
  ros2 launch spot_control teresa_perception.launch.py
  ros2 launch spot_control teresa_perception.launch.py use_orbbec_driver:=true
  ros2 launch spot_control teresa_perception.launch.py use_surface:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Argomenti ──────────────────────────────────────────────────────
    test_mode_arg = DeclareLaunchArgument(
        'test_mode', default_value='true',
        description='Passa al laying_human_detector')
    use_orbbec_driver_arg = DeclareLaunchArgument(
        'use_orbbec_driver', default_value='false',
        description='Driver Orbbec + TF statiche. false se gia in teresa_core')
    use_tracker_arg = DeclareLaunchArgument(
        'use_tracker', default_value='true',
        description='Avvia z1_yolo_torso_tracker (RealSense)')
    use_surface_arg = DeclareLaunchArgument(
        'use_surface', default_value='true',
        description='Avvia realsense_surface_node')

    test_mode = LaunchConfiguration('test_mode')
    use_orbbec_driver = LaunchConfiguration('use_orbbec_driver')
    use_tracker = LaunchConfiguration('use_tracker')
    use_surface = LaunchConfiguration('use_surface')

    # ── Spot perception (Orbbec → WBC) ─────────────────────────────────
    spot_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('spot_perception'),
                'launch', 'spot_perception.launch.py',
            ])
        ]),
        launch_arguments={
            'test_mode':          test_mode,
            'use_orbbec_driver':  use_orbbec_driver,
        }.items(),
    )

    # ── Z1 perception (RealSense → FSM) ────────────────────────────────
    z1_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('z1_vision'),
                'launch', 'z1_perception.launch.py',
            ])
        ]),
        launch_arguments={
            'use_tracker': use_tracker,
            'use_surface': use_surface,
        }.items(),
    )

    return LaunchDescription([
        test_mode_arg,
        use_orbbec_driver_arg,
        use_tracker_arg,
        use_surface_arg,
        spot_perception_launch,
        z1_perception_launch,
    ])
