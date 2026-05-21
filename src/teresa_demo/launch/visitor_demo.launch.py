#!/usr/bin/env python3
"""
Visitor Demo Launch
===================
Avvia Z1 bringup + z1_ik_to_jtc + visitor_demo per demo visitatori.

Spot + Z1 si muovono contemporaneamente in pattern di searching
(senza telecamere, WBC o percezione).

Prerequisito: spot_ros2 in esecuzione su SpotCore (TF odom→body).

Uso:
  ros2 launch teresa_demo visitor_demo.launch.py
  ros2 launch teresa_demo visitor_demo.launch.py joint_offset:='0.0 0.69 0.0 0.0 0.0 0.0'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Argomenti ──────────────────────────────────────────────────────
    joint_offset_arg = DeclareLaunchArgument(
        'joint_offset',
        default_value='0.0 0.69 0.0 0.0 0.0 0.0',
        description='Joint encoder offset [rad] — joint2 folded ~40° for Spot mount')

    # ── Z1 bringup (no RViz, JTC attivo, senza gripper) ──────────────
    z1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('z1_bringup'),
                'launch', 'z1.launch.py',
            ])
        ]),
        launch_arguments={
            'sim_ignition':         'false',
            'starting_controller':  'joint_trajectory_controller',
            'with_gripper':         'false',
            'rviz':                 'false',
            'joint_offset':         LaunchConfiguration('joint_offset'),
        }.items(),
    )

    # ── z1_ik_to_jtc (IK solver) ─────────────────────────────────────
    ik_params = PathJoinSubstitution([
        FindPackageShare('z1_vision'), 'config', 'z1_ik_jtc_params.yaml',
    ])
    ik_node = Node(
        package='z1_vision',
        executable='z1_ik_to_jtc',
        name='z1_ik_to_jtc',
        parameters=[ik_params],
        output='screen',
    )

    # ── Visitor Demo node ────────────────────────────────────────────
    demo_params = PathJoinSubstitution([
        FindPackageShare('teresa_demo'), 'config', 'demo_params.yaml',
    ])
    demo_node = Node(
        package='teresa_demo',
        executable='visitor_demo',
        name='visitor_demo',
        parameters=[demo_params],
        output='screen',
    )

    return LaunchDescription([
        joint_offset_arg,

        LogInfo(msg=['Visitor Demo — Spot + Z1 simultaneous search movements']),

        z1_bringup,
        ik_node,
        demo_node,

        LogInfo(msg=['Demo avviato. Spot core esegue body_pose grid, Z1 arm pose cycling.']),
    ])
