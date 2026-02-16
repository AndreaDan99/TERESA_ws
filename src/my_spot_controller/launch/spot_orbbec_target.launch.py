from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    
    # Path to config files
    config_file = '/ros_ws/src/my_spot_controller/config/my_spot.yaml'
    nav2_params = PathJoinSubstitution([
        FindPackageShare('my_spot_controller'),
        'config',
        'nav2_params.yaml'
    ])
    
    return LaunchDescription([
        
        # ========================================================================
        # 1) SPOT DRIVER
        # ========================================================================
        LogInfo(msg="[0s] ========== AVVIO SPOT DRIVER =========="),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'launch',
                    'spot_driver.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file': config_file,
                'launch_rviz': 'True'
            }.items()
        ),
        
        # ========================================================================
        # 2) ORBBEC CAMERA DRIVER (solo per perception)
        # ========================================================================
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[3s] ========== AVVIO ORBBEC CAMERA (PERCEPTION) =========="),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('orbbec_camera'),
                            'launch',
                            'femto_bolt.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'enable_color': 'true',
                        'enable_depth': 'true',
                        'color_width': '1280',
                        'color_height': '720',
                        'color_fps': '15',
                        'color_format': 'MJPG',
                        'depth_width': '1024',
                        'depth_height': '1024',
                        'depth_fps': '15',
                        'depth_format': 'Y16',
                        'depth_registration': 'true',
                        'enable_point_cloud': 'true',
                        'enable_colored_point_cloud': 'true',
                        'enable_ir': 'false',
                        'enable_accel': 'false',
                        'enable_gyro': 'false'
                    }.items()
                ),
            ]
        ),
        
        # ========================================================================
        # 3) TF STATICHE: body -> camera
        # ========================================================================
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="[5s] ========== PUBBLICAZIONE TF STATICHE =========="),
                
                # TF: body -> camera_link (Orbbec)
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='spot_to_camera',
                    arguments=[
                        '0.40', '0.0', '0.55',
                        '0', '0', '0',
                        'body',
                        'camera_link'
                    ]
                ),
                
                # TF: camera_link -> camera_color_optical_frame
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='camera_to_optical',
                    arguments=[
                        '0', '0', '0',
                        '-1.5708', '0', '-1.5708',
                        'camera_link',
                        'camera_color_optical_frame'
                    ]
                ),
            ]
        ),
        
        # ========================================================================
        # 4) SPOT STARTUP NODE
        # ========================================================================
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="[6s] ========== AVVIO SPOT STARTUP NODE =========="),
                Node(
                    package='my_spot_controller',
                    executable='spot_startup_node',
                    name='spot_startup_node',
                    output='screen',
                    parameters=[{
                        'robot_name': 'my_spot'
                    }]
                ),
            ]
        ),
        
        # ========================================================================
        # 5) YOLO SKELETON NODE (Perception - usa Orbbec)
        # ========================================================================
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg="[7s] ========== AVVIO YOLO SKELETON (ORBBEC) =========="),
                Node(
                    package='my_spot_controller',
                    executable='yolo_skeleton_node_orbbec',
                    name='yolo_skeleton_node',
                    output='screen',
                ),
            ]
        ),
        
        # ========================================================================
        # 6) HUMAN POSTURE ANALYZER + BOUNDING BOX (Perception)
        # ========================================================================
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="[8s] ========== AVVIO PERCEPTION ANALYZERS =========="),
                
                # Posture Analyzer
                Node(
                    package='my_spot_controller',
                    executable='human_posture_analyzer_spot',
                    name='posture_analyzer',
                    output='screen',
                    parameters=[{
                        'frame_id': 'camera_color_optical_frame',
                        'knee_angle_stand_min': 160.0,
                        'knee_angle_sit_max': 120.0,
                        'torso_angle_lying_min': 65.0
                    }]
                ),
                
                # Bounding Box Visualizer
                Node(
                    package='my_spot_controller',
                    executable='human_bounding_box_visualizer',
                    name='bbox_visualizer',
                    output='screen',
                    parameters=[{
                        'frame_id': 'camera_color_optical_frame',
                        'safety_margin_body': 0.5,
                    }]
                ),
            ]
        ),
        
        # ========================================================================
        # 7) HUMAN-AWARE TARGET GENERATOR (Navigation planning)
        # ========================================================================
        TimerAction(
            period=9.0,
            actions=[
                LogInfo(msg="[9s] ========== AVVIO HUMAN-AWARE TARGET GENERATOR =========="),
                Node(
                    package='my_spot_controller',
                    executable='human_target_generator',
                    name='human_target_generator',
                    output='screen',
                    parameters=[{
                        'standing_approach_dist': 1.5,    # Distanza approccio persona in piedi
                        'sitting_approach_dist': 1.0,     # Distanza approccio persona seduta
                        'lying_lateral_offset': 0.8,      # Offset laterale persona sdraiata
                        'min_update_interval': 2.0,       # Non aggiornare goal troppo spesso
                        'goal_frame': 'odom',             # Frame goal
                    }]
                ),
            ]
        ),
        
        # ========================================================================
        # 8) NAV2 STACK (Controller + Costmap - usa Spot cameras)
        # ========================================================================
        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg="[10s] ========== AVVIO NAV2 STACK (LOCAL PLANNING) =========="),
                
                # Controller Server (DWB local planner)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                
                # Recoveries Server (backup, spin, wait)
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server', 
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params]
                ),
                
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': ['controller_server', 'behavior_server'] 
                    }]
                ),
            ]
        ),
        
        # ========================================================================
        # 9) NAV2 GOAL SENDER (ponte Human-Aware → Nav2)
        # ========================================================================
        TimerAction(
            period=11.0,
            actions=[
                LogInfo(msg="[11s] ========== AVVIO NAV2 GOAL SENDER =========="),
                Node(
                    package='my_spot_controller',
                    executable='nav2_goal_sender',
                    name='nav2_goal_sender',
                    output='screen',
                    parameters=[{
                        'dry_run_mode': True,             # Simulazione attiva
                        'min_goal_interval': 5.0,         # Min 5s tra i goal
                        'cancel_old_goals': True,         # Cancella goal precedenti
                        'robot_frame': 'my_spot/body',    # Frame robot
                        'odom_frame': 'my_spot/odom',     # Frame odometria
                        'use_tf_for_pose': True,          # USA TF (non topic /odom)
                        'wait_for_server_timeout': 5.0,   # Timeout action server
                    }]
                ),
            ]
        ),
        
        # ========================================================================
        # FINE
        # ========================================================================
        TimerAction(
            period=12.0,
            actions=[
                LogInfo(msg="[12s] ========== SISTEMA COMPLETO PRONTO! =========="),
            ]
        ),
    ])
