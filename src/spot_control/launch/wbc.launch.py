#!/usr/bin/env python3
"""
WBC Launch — wbc_qp_controller + wbc_coordinator + wbc_spot_navigator

PREREQUISITI:
  - spot_ros2 su SpotCore (TF odom→body)
  - teresa_core.launch.py in esecuzione (driver + tf_monitor)
  - spot_perception.launch.py in esecuzione (Orbbec + YOLO)
  - z1_vision in esecuzione (z1_ik_to_jtc, z1_FSM)

Uso:
  ros2 launch spot_control wbc.launch.py
  ros2 launch spot_control wbc.launch.py dry_run:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false',
        description='Dry run: publish debug topics only, no robot movement')

    step_mode_arg = DeclareLaunchArgument('step_mode', default_value='false',
        description='Step mode: gate automatic FSM transitions, press "n" to advance')

    skip_scan_arg = DeclareLaunchArgument('skip_perceptual_scan', default_value='false',
        description='Skip 6-pose Cartesian perceptual scan during APPROACHING (use when no RealSense)')

    params_file = PathJoinSubstitution([
        FindPackageShare('spot_control'), 'config', 'wbc_params.yaml'
    ])

    qp_node = Node(
        package='spot_control',
        executable='wbc_qp_controller',
        name='wbc_qp_controller',
        output='screen',
        parameters=[params_file,
                    {'dry_run': LaunchConfiguration('dry_run')},
                    {'skip_perceptual_scan': LaunchConfiguration('skip_perceptual_scan')}],
    )

    coord_node = Node(
        package='spot_control',
        executable='wbc_coordinator',
        name='wbc_coordinator',
        output='screen',
        parameters=[params_file,
                    {'step_mode': LaunchConfiguration('step_mode')}],
    )

    navigator_node = Node(
        package='spot_control',
        executable='wbc_spot_navigator',
        name='wbc_spot_navigator',
        output='screen',
        parameters=[params_file],
    )

    optimizer_node = Node(
        package='spot_control',
        executable='body_pose_optimizer',
        name='body_pose_optimizer',
        output='screen',
        parameters=[params_file],
    )

    exposure_node = Node(
        package='spot_control',
        executable='exposure_scanner',
        name='exposure_scanner',
        output='screen',
        parameters=[params_file],
    )

    snapshot_node = Node(
        package='spot_control',
        executable='exposure_snapshot',
        name='exposure_snapshot',
        output='screen',
        parameters=[params_file],
    )

    mux_node = Node(
        package='spot_control',
        executable='ik_goal_mux',
        name='ik_goal_mux',
        output='screen',
    )

    return LaunchDescription([
        dry_run_arg,
        step_mode_arg,
        skip_scan_arg,
        LogInfo(msg=['WBC — arm-only QP + coordinator + spot navigator + body optimizer + exposure scanner + snapshot + ik_goal_mux']),
        qp_node,
        coord_node,
        navigator_node,
        optimizer_node,
        exposure_node,
        snapshot_node,
        mux_node,
    ])
