#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description=(
            'cmd_vel topic — must match spot_driver namespace. '
            'Use /my_spot/cmd_vel if spot_name="my_spot" in spot_driver config.'
        ),
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('spot_navigation'),
        'config',
        'spot_nav_params.yaml',
    ])

    navigator_node = Node(
        package='spot_navigation',
        executable='spot_goal_navigator',
        name='spot_goal_navigator',
        output='screen',
        parameters=[
            params_file,
            {'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic')},
        ],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        navigator_node,
    ])
