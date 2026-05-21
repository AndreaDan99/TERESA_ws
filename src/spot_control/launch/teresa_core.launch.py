#!/usr/bin/env python3
"""
TERESA Core Launch
==================
Avvia TUTTI i driver hardware + TF statiche + tf_monitor con 4 condizioni.

Include:
  - Orbbec Femto Bolt driver (dal config di spot_perception)
  - TF statiche: my_spot/body → orbbec_link → orbbec_color_optical_frame
  - TF statica: my_spot/body → world (Z1 mount, connette Spot al world frame URDF)
  - z1_realsense.launch.py (Z1 bringup + RealSense + camera TF, senza RViz)
  - tf_monitor (4 condizioni: 3 topic + 8 TF → /wbc/tf_ready)

Uso:
  ros2 launch spot_control teresa_core.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    z1_mount_x = LaunchConfiguration('z1_mount_x')
    z1_mount_y = LaunchConfiguration('z1_mount_y')
    z1_mount_z = LaunchConfiguration('z1_mount_z')

    # ── Orbbec Femto Bolt driver ──────────────────────────────────────
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('orbbec_camera'),
                'launch',
                'femto_bolt.launch.py',
            ])
        ]),
        launch_arguments={
            'camera_name':             'orbbec',
            'enable_color':            'true',
            'enable_depth':            'true',
            'color_width':             '1280',
            'color_height':            '720',
            'color_fps':               '15',
            'color_format':            'MJPG',
            'depth_width':             '1024',
            'depth_height':            '1024',
            'depth_fps':               '15',
            'depth_format':            'Y16',
            'depth_registration':      'true',
            'enable_point_cloud':      'false',
            'enable_colored_point_cloud': 'false',
            'enable_ir':               'false',
            'enable_accel':            'false',
            'enable_gyro':             'false',
            'publish_tf':              'false',
        }.items()
    )

    # ── TF statiche Orbbec ────────────────────────────────────────────
    static_tf_body_orbbec = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='spot_to_orbbec',
        arguments=[
            '0.30', '0.0', '0.15',
            '0', '0', '0',
            'my_spot/body', 'orbbec_link',
        ]
    )

    static_tf_orbbec_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='orbbec_to_optical',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'orbbec_link', 'orbbec_color_optical_frame',
        ]
    )

    # ── Z1 bringup + RealSense + camera TF (senza RViz) ──────────────
    z1_realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('z1_vision'),
                'launch',
                'z1_realsense.launch.py',
            ])
        ]),
        launch_arguments={
            'use_rviz': 'false',
            'use_camera_tf': 'false',
        }.items()
    )

    # ── TF statica Z1 mount: my_spot/body → world ─────────────────
    # world = root frame of Z1 URDF (parent of link00).
    static_tf_body_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='spot_to_world',
        arguments=[
            '--x', z1_mount_x, '--y', z1_mount_y, '--z', z1_mount_z,
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'my_spot/body', '--child-frame-id', 'world',
        ]
    )

    # ── TF statica RealSense mount: link06 → camera_link ───────────
    static_tf_link06_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link06_to_camera_link',
        arguments=[
            '0.0', '0.0', '0.05',
            '0', '0', '0', '1',
            'link06', 'camera_link',
        ]
    )

    # ── TF monitor (4 condizioni → /wbc/tf_ready) ────────────────────
    tf_monitor_node = Node(
        package='spot_control',
        executable='tf_monitor',
        name='tf_monitor',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('z1_mount_x', default_value='0.20',
            description='Z1 link00 X da my_spot/body [m]'),
        DeclareLaunchArgument('z1_mount_y', default_value='0.0',
            description='Z1 link00 Y da my_spot/body [m]'),
        DeclareLaunchArgument('z1_mount_z', default_value='0.20',
            description='Z1 link00 Z da my_spot/body [m]'),

        LogInfo(msg=['TERESA Core — driver hardware + TF statiche + monitor']),

        orbbec_launch,
        static_tf_body_orbbec,
        static_tf_orbbec_optical,

        static_tf_body_world,

        static_tf_link06_camera,

        z1_realsense_launch,

        tf_monitor_node,

        LogInfo(msg=['Core lanciato — tf_monitor in attesa...']),
    ])
