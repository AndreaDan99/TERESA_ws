#!/usr/bin/env python3
"""
WBC Launch — static TF body→link00 + wbc_qp_controller + wbc_coordinator

PREREQUISITI:
  - spot_ros2 su SpotCore (TF odom→body)
  - spot_perception.launch.py in esecuzione (Orbbec + YOLO)
  - z1_vision in esecuzione (z1_ik_to_jtc, z1_FSM)
  - teresa_mission in esecuzione (navigazione Spot)

Uso:
  ros2 launch spot_control wbc.launch.py
  ros2 launch spot_control wbc.launch.py z1_x:=0.30 z1_z:=0.70
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # TF mount offsets — ESTIMATED, replace with measured values after mounting
    z1_x_arg = DeclareLaunchArgument('z1_x', default_value='0.20',
        description='Z1 link00 X from my_spot/body [m] (forward)')
    z1_y_arg = DeclareLaunchArgument('z1_y', default_value='0.0',
        description='Z1 link00 Y from my_spot/body [m] (left)')
    z1_z_arg = DeclareLaunchArgument('z1_z', default_value='0.20',
        description='Z1 link00 Z from my_spot/body [m] (up)')

    dry_run_arg = DeclareLaunchArgument('dry_run', default_value='false',
        description='Dry run: publish debug topics only, no robot movement')

    params_file = PathJoinSubstitution([
        FindPackageShare('spot_control'), 'config', 'wbc_params.yaml'
    ])

    # Static TF: my_spot/body → link00  (x y z yaw pitch roll parent child)
    static_tf_z1_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='spot_to_z1_base',
        arguments=[
            LaunchConfiguration('z1_x'),
            LaunchConfiguration('z1_y'),
            LaunchConfiguration('z1_z'),
            '0', '0', '0',
            'my_spot/body',
            'link00',
        ]
    )

    # NOTE: ik_goal_mux is now launched by z1_control.launch.py
    # (always needed, even in standalone mode)

    qp_node = Node(
        package='spot_control',
        executable='wbc_qp_controller',
        name='wbc_qp_controller',
        output='screen',
        parameters=[params_file,
                    {'dry_run': LaunchConfiguration('dry_run')}],
    )

    coord_node = Node(
        package='spot_control',
        executable='wbc_coordinator',
        name='wbc_coordinator',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        z1_x_arg, z1_y_arg, z1_z_arg, dry_run_arg,
        LogInfo(msg=['WBC — Spot+Z1 holistic control']),
        LogInfo(msg=['TF my_spot/body → link00']),
        static_tf_z1_mount,
        qp_node,
        coord_node,
    ])
