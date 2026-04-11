#!/usr/bin/env python3
"""
Spot Perception Launch - Jetson
Avvia: yolo_skeleton (Orbbec), posture_classifier, laying_human_detector
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_spot',
        description='Nome robot Spot'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='frontleft',
        description='Nome camera Orbbec su Spot'
    )

    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='true',
        description='Se true pubblica approach point ma non invia goal navigazione'
    )

    robot_name = LaunchConfiguration('robot_name')
    camera_name = LaunchConfiguration('camera_name')
    test_mode = LaunchConfiguration('test_mode')

    yolo_skeleton_node = Node(
        package='spot_perception',
        executable='yolo_skeleton',
        name='yolo_skeleton',
        output='screen',
        parameters=[{
            'model_path': 'yolo11n-pose.pt',
            'conf_thr': 0.3,
            'imgsz': 640,
            'device': '0',        # GPU Jetson
            'use_half': True,     # FP16 su Jetson
            'max_depth_m': 5.0,
            'vel_damping': 0.6,
            'camera_name': camera_name,
            'target_frame': 'my_spot/body',
        }]
    )

    posture_classifier_node = Node(
        package='spot_perception',
        executable='posture_classifier',
        name='posture_classifier',
        output='screen',
        parameters=[{
            'frame_id': 'my_spot/body',
            'up_axis': [0.0, 0.0, 1.0],
            'hip_to_base_stand': 0.35,
            'hip_to_base_sit': 0.20,
            'lying_height_max': 0.45,
            'lying_angle_min': 65.0,
        }]
    )

    laying_detector_node = Node(
        package='spot_perception',
        executable='laying_human_detector',
        name='laying_human_detector',
        output='screen',
        parameters=[{
            'approach_distance': 1.0,
            'min_detection_confidence': 0.5,
            'min_valid_keypoints': 4,
            'test_mode': test_mode,
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        camera_name_arg,
        test_mode_arg,
        yolo_skeleton_node,
        posture_classifier_node,
        laying_detector_node,
    ])
