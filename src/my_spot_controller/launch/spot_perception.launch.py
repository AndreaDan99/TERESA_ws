#!/usr/bin/env python3
"""
Launch file per Human Pose Detection su Spot con RealSense
Avvia in ordine:
1. Spot startup node
2. RealSense camera
3. YOLO skeleton detection
4. Human posture analyzer
5. Static TF body->camera
6. RViz (opzionale)

PREREQUISITO: Spot driver già avviato manualmente
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # ============================================================
    # LAUNCH ARGUMENTS
    # ============================================================
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # ============================================================
    # 1. SPOT STARTUP NODE
    # ============================================================
    spot_startup_node = Node(
        package='my_spot_controller',
        executable='spot_startup_node.py',
        name='spot_startup_node',
        output='screen',
        parameters=[{
            'robot_name': 'my_spot'
        }]
    )
    
    # ============================================================
    # 2. REALSENSE CAMERA (con i tuoi parametri)
    # ============================================================
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ]),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'colorizer.enable': 'false',
            'align_depth.enable': 'true',
        }.items()
    )
    
    # ============================================================
    # 3. STATIC TRANSFORM: Spot body -> RealSense camera
    # ============================================================
    # ⚠️ MODIFICA x, y, z in base al mount fisico della RealSense su Spot
    static_tf_body_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_camera',
        arguments=['0.3', '0', '0.4', '0', '0', '0', 'body', 'camera_link']
    )
    
    # ============================================================
    # 4. YOLO SKELETON DETECTION NODE
    # ============================================================
    yolo_skeleton_node = Node(
        package='my_spot_controller',
        executable='yolo_skeleton_spot.py',
        name='yolo_skeleton_node_spot',
        output='screen',
        parameters=[{
            'model_path': 'yolo11n-pose.pt',
            'conf_thr': 0.3,
            'vel_damping': 0.6,
            'max_depth_m': 3.0
        }]
    )
    
    # ============================================================
    # 5. HUMAN POSTURE ANALYZER NODE
    # ============================================================
    posture_analyzer_node = Node(
        package='my_spot_controller',
        executable='human_posture_analyzer_spot.py',
        name='human_posture_analyzer_spot',
        output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'hip_to_base_stand': 0.35,
            'hip_to_base_sit': 0.20,
            'lying_height_max': 0.45,
            'lying_angle_min': 65.0
        }]
    )
    
    # ============================================================
    # 6. RVIZ 
    # ============================================================
    rviz_config_path = os.path.join(
        get_package_share_directory('my_spot_controller'),
        'config',
        'realsense_config.rviz' 
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    # ============================================================
    # SEQUENZA DI AVVIO CON DELAY
    # ============================================================
    return LaunchDescription([
        # Argomenti
        launch_rviz_arg,
        
        # Step 1: Spot startup (subito)
        spot_startup_node,
        
        # Step 2: RealSense dopo 3 secondi (aspetta che Spot sia in piedi)
        TimerAction(
            period=3.0,
            actions=[realsense_launch]
        ),
        
        # Step 3: Static TF dopo 4 secondi
        TimerAction(
            period=4.0,
            actions=[static_tf_body_camera]
        ),
        
        # Step 4: YOLO skeleton dopo 6 secondi (aspetta RealSense)
        TimerAction(
            period=6.0,
            actions=[yolo_skeleton_node]
        ),
        
        # Step 5: Posture analyzer dopo 7 secondi
        TimerAction(
            period=7.0,
            actions=[posture_analyzer_node]
        ),
        
        # Step 6: RViz (opzionale, parte subito se abilitato)
        rviz_node,
    ])
