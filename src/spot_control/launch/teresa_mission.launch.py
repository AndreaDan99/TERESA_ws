#!/usr/bin/env python3
"""
TERESA Mission Launch
Avvia teresa_mission node (coordinatore navigazione Spot).

PREREQUISITO: spot_perception.launch.py già in esecuzione.

Uso:
  # Test geometria senza muovere Spot
  ros2 launch spot_control teresa_mission.launch.py dry_run:=true

  # Movimento reale (richiede spot_ros2 su SpotCore)
  ros2 launch spot_control teresa_mission.launch.py dry_run:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='Se true calcola goal e pubblica RViz ma non muove Spot'
    )
    preferred_side_arg = DeclareLaunchArgument(
        'preferred_side',
        default_value='auto',
        description='Lato di approccio: auto | left | right'
    )
    approach_margin_arg = DeclareLaunchArgument(
        'approach_margin',
        default_value='0.05',
        description='Distanza di sicurezza oltre il bordo bbox (metri)'
    )
    crouch_height_arg = DeclareLaunchArgument(
        'crouch_height',
        default_value='-0.10',
        description='Altezza crouch Spot all arrivo (range: -0.15 a 0.15 m)'
    )

    mission_node = Node(
        package='spot_control',
        executable='teresa_mission',
        name='teresa_mission',
        output='screen',
        parameters=[{
            'dry_run':          LaunchConfiguration('dry_run'),
            'preferred_side':   LaunchConfiguration('preferred_side'),
            'approach_margin':  LaunchConfiguration('approach_margin'),
            'crouch_height':    LaunchConfiguration('crouch_height'),
            'spot_front_offset': 0.50,
            'min_confidence':   0.6,
            'nav_timeout':      30.0,
            'goal_frame':       'my_spot/odom',
        }]
    )

    return LaunchDescription([
        dry_run_arg,
        preferred_side_arg,
        approach_margin_arg,
        crouch_height_arg,

        LogInfo(msg=['🤖 TERESA Mission — avvio coordinator']),
        LogInfo(msg=['   Topics in: /human_pose/points_3d, /human_pose/bounding_box, /human_pose/posture']),
        LogInfo(msg=['   Topics out (RViz): /teresa/body_axis, /teresa/approach_goal, /teresa/safe_zone, /teresa/fsm_state']),

        mission_node,
    ])
