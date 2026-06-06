#!/usr/bin/env python3
"""
Z1 Perception Launch
====================
Avvia il pipeline di percezione visiva:
  - z1_yolo_torso_tracker   : YOLO11 pose estimation + Kalman filter + FSM LOCKED
  - realsense_surface_node  : stima piano superficie torso da depth ROI (PCA)

Da lanciare DOPO z1_realsense.launch.py (robot hw + camera + TF).

Uso:
    ros2 launch z1_vision z1_perception.launch.py
    ros2 launch z1_vision z1_perception.launch.py use_surface_node:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('z1_vision')

    # ── Config files ───────────────────────────────────────────────────
    yolo_params    = PathJoinSubstitution([pkg, 'config', 'z1_yolo_torso_params.yaml'])
    nlf_params     = PathJoinSubstitution([pkg, 'config', 'nlf_torso_params.yaml'])
    surface_params = PathJoinSubstitution([pkg, 'config', 'surface_params.yaml'])

    # ── Launch arguments ───────────────────────────────────────────────
    use_tracker_arg = DeclareLaunchArgument(
        'use_tracker',
        default_value='true',
        description='Avvia il nodo YOLO torso tracker'
    )
    use_surface_arg = DeclareLaunchArgument(
        'use_surface',
        default_value='true',
        description='Avvia il nodo realsense surface (stima piano torso)'
    )

    use_tracker = LaunchConfiguration('use_tracker')
    use_surface = LaunchConfiguration('use_surface')

    perception_backend_arg = DeclareLaunchArgument(
        'perception_backend',
        default_value='nlf',
        description='Perception backend: nlf or yolo'
    )
    perception_backend = LaunchConfiguration('perception_backend')

    # ── Nodi ──────────────────────────────────────────────────────────

    # NODO 1 — YOLO Torso Tracker
    # Stima la posizione 3D del torso, filtra con Kalman, pubblica target
    # quando il tracker è in stato LOCKED.
    yolo_tracker_node = Node(
        package    = 'z1_vision',
        executable = 'z1_yolo_torso_tracker',
        name       = 'z1_yolo_torso_tracker',
        parameters = [yolo_params],
        output     = 'screen',
        condition  = IfCondition(PythonExpression([
            '"', perception_backend, '" == "yolo" and "', use_tracker, '" == "true"'
        ])),
    )

    # NODO 1b — NLF Torso Tracker (alternative to YOLO)
    nlf_tracker_node = Node(
        package    = 'z1_vision',
        executable = 'nlf_torso_tracker',
        name       = 'nlf_torso_tracker',
        parameters = [nlf_params],
        output     = 'screen',
        condition  = IfCondition(PythonExpression(['"', perception_backend, '" == "nlf"'])),
    )

    # NODO 2 — RealSense Surface Node
    # Riceve il target LOCKED, estrae la ROI depth, stima il piano
    # della superficie del torso con PCA e pubblica il frame superficie.
    surface_node = Node(
        package    = 'z1_vision',
        executable = 'realsense_surface_node',
        name       = 'realsense_surface_node',
        parameters = [surface_params],
        output     = 'screen',
        condition  = IfCondition(use_surface),
    )

    return LaunchDescription([
        use_tracker_arg,
        use_surface_arg,
        perception_backend_arg,
        nlf_tracker_node,
        yolo_tracker_node,
        surface_node,
    ])
