#!/usr/bin/env python3
"""
SpotZi Display Launch
=====================
Visualizza Spot + Z1 in RViz con slider per controllare i giunti.

Uso:
  ros2 launch spotzi_description display.launch.py
  ros2 launch spotzi_description display.launch.py with_gripper:=false
  ros2 launch spotzi_description display.launch.py z1_mount_x:=0.25 z1_mount_z:=0.18
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("spotzi_description")

    # ── Launch arguments ─────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Use joint_state_publisher_gui (sliders)"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Launch RViz2"
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "spotzi.rviz"]),
        description="RViz config file"
    )
    with_gripper_arg = DeclareLaunchArgument(
        "with_gripper", default_value="true",
        description="Include Z1 gripper"
    )
    z1_mount_x_arg = DeclareLaunchArgument(
        "z1_mount_x", default_value="0.20",
        description="Z1 mount X offset from Spot body [m]"
    )
    z1_mount_y_arg = DeclareLaunchArgument(
        "z1_mount_y", default_value="0.0",
        description="Z1 mount Y offset from Spot body [m]"
    )
    z1_mount_z_arg = DeclareLaunchArgument(
        "z1_mount_z", default_value="0.20",
        description="Z1 mount Z offset from Spot body [m]"
    )

    # ── Robot description (xacro → URDF) ─────────────────────────────────
    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([pkg_share, "urdf", "spotzi.urdf.xacro"]),
        " with_gripper:=", LaunchConfiguration("with_gripper"),
        " z1_mount_x:=", LaunchConfiguration("z1_mount_x"),
        " z1_mount_y:=", LaunchConfiguration("z1_mount_y"),
        " z1_mount_z:=", LaunchConfiguration("z1_mount_z"),
    ])

    # ── Nodes ────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("gui"), invert=True),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        rviz_config_arg,
        with_gripper_arg,
        z1_mount_x_arg,
        z1_mount_y_arg,
        z1_mount_z_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz,
    ])
