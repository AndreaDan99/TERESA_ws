#!/usr/bin/env python3
"""
Z1 Perception + FSM Launch
Avvia: YOLO Torso Tracker + RealSense Surface Node + FSM

(Nota: IK→JTC e Impedance vengono avviati in un launch separato.)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():

    use_surface_node = LaunchConfiguration('use_surface_node')
    use_fsm = LaunchConfiguration('use_fsm')
    use_tracker_node = LaunchConfiguration('use_tracker_node')

    # ── NUOVO: YAML parametri torso tracker ───────────────────────
    torso_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'z1_yolo_torso_params.yaml'
    ])

    # ── NUOVO: YAML parametri FSM esterna ─────────────────────────
    fsm_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'z1_fsm_params.yaml'
    ])

    # ── NUOVO: YAML parametri surface node ───────────────────────
    surface_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'surface_params.yaml'
    ])

    # =========================================================
    # NODO 1: YOLO Torso Tracker
    # =========================================================
    yolo_tracker_node = Node(
        package='z1_vision',
        executable='z1_yolo_torso_tracker',
        name='z1_yolo_torso_tracker',
        parameters=[torso_params_file],   # ← carica da YAML
        output='screen',
        condition=IfCondition(use_tracker_node)
    )

    # =========================================================
    # NODO 1.5: External Torso FSM (state machine)
    # =========================================================
    z1_fsm_node = Node(
        package='z1_vision',
        executable='z1_FSM',
        name='z1_FSM',
        parameters=[fsm_params_file],
        output='screen',
        condition=IfCondition(use_fsm)
    )

    # =========================================================
    # NODO 2: Surface Detection Node
    # =========================================================
    surface_node = Node(
        package='z1_vision',
        executable='realsense_surface_node',
        name='realsense_surface_node',
        parameters=[surface_params_file],
        output='screen',
        condition=IfCondition(use_surface_node)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tracker_node',
            default_value='true',
            description='Launch YOLO torso tracker node'
        ),
        DeclareLaunchArgument(
            'use_surface_node',
            default_value='true',
            description='Launch surface detection node'
        ),
        DeclareLaunchArgument(
            'use_fsm',
            default_value='true',
            description='Launch external torso FSM (state machine)'
        ),

        yolo_tracker_node,
        z1_fsm_node,
        surface_node,
    ])
