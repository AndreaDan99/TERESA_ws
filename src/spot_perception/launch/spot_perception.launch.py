#!/usr/bin/env python3
"""
Spot Perception Launch — Jetson + Orbbec Femto Bolt
Avvia: Orbbec driver, TF statiche, yolo_skeleton, posture_analyzer,
       bounding_box_visualizer, laying_human_detector
PREREQUISITO: spot_ros2 già avviato su SpotCore (via DDS)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ============================================================
    # ARGOMENTI
    # ============================================================
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='true',
        description='Se true pubblica approach point ma non invia goal navigazione'
    )

    test_mode = LaunchConfiguration('test_mode')

    # ============================================================
    # 1) ORBBEC CAMERA (Femto Bolt)
    # ============================================================
    orbbec_launch = IncludeLaunchDescription(
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
            'enable_gyro': 'false',
        }.items()
    )

    # ============================================================
    # 2) TF STATICHE: body → camera_link → camera_color_optical_frame
    # ============================================================
    # Adatta x, y, z in base al mount fisico dell'Orbbec su Spot
    static_tf_body_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='spot_to_camera',
        arguments=[
            '0.40', '0.0', '0.55',
            '0', '0', '0',
            'my_spot/body',
            'camera_link'
        ]
    )

    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'camera_link',
            'camera_color_optical_frame'
        ]
    )

    # ============================================================
    # 3) YOLO SKELETON (Orbbec)
    # ============================================================
    yolo_skeleton_node = Node(
        package='spot_perception',
        executable='yolo_skeleton_node_orbbec',
        name='yolo_skeleton_node',
        output='screen',
        parameters=[{
            'model_path': 'yolo11n-pose.pt',
            'conf_thr': 0.25,
            'vel_damping': 0.5,
            'max_depth_m': 5.0,
            'max_track_distance': 0.6,
            'track_timeout': 1.5,
            'lying_torso_angle_min': 65.0,
            'max_tracks': 5,
            'target_hysteresis_frames': 10,
        }]
    )

    # ============================================================
    # 4) HUMAN POSTURE ANALYZER
    # ============================================================
    posture_analyzer_node = Node(
        package='spot_perception',
        executable='human_posture_analyzer_spot',
        name='posture_analyzer',
        output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'knee_angle_stand_min': 160.0,
            'knee_angle_sit_max': 120.0,
            'torso_angle_lying_min': 65.0,
        }]
    )

    # ============================================================
    # 5) BOUNDING BOX VISUALIZER
    # ============================================================
    bbox_visualizer_node = Node(
        package='spot_perception',
        executable='human_bounding_box_visualizer',
        name='bbox_visualizer',
        output='screen',
        parameters=[{
            'frame_id': 'camera_color_optical_frame',
            'safety_margin_body': 0.5,
        }]
    )

    # ============================================================
    # 6) LAYING HUMAN DETECTOR
    # ============================================================
    laying_detector_node = Node(
        package='spot_perception',
        executable='laying_human_detector',
        name='laying_human_detector',
        output='screen',
        parameters=[{
            'approach_margin':          0.05,
            'spot_front_offset':        0.50,
            'preferred_side':           'auto',
            'min_detection_confidence': 0.5,
            'min_valid_keypoints':      4,
            'test_mode':                test_mode,
        }]
    )

    # ============================================================
    # SEQUENZA DI AVVIO
    # ============================================================
    return LaunchDescription([
        test_mode_arg,

        LogInfo(msg=['🤖 Spot Perception System — Jetson + Orbbec Femto Bolt']),
        LogInfo(msg=['   Topics: /camera/color/image_raw + /camera/depth/image_raw']),
        LogInfo(msg=['   Frame output: camera_color_optical_frame']),

        # Orbbec subito
        orbbec_launch,

        # TF statiche dopo 2s (aspetta Orbbec)
        TimerAction(period=2.0, actions=[
            LogInfo(msg=['[2s] TF statiche: body → camera_link → camera_color_optical_frame']),
            static_tf_body_camera,
            static_tf_camera_optical,
        ]),

        # Perception dopo 4s (aspetta TF + Orbbec warm-up)
        TimerAction(period=4.0, actions=[
            LogInfo(msg=['[4s] YOLO skeleton + Posture + BBox + Laying detector']),
            yolo_skeleton_node,
            posture_analyzer_node,
            bbox_visualizer_node,
            laying_detector_node,
        ]),

        TimerAction(period=5.0, actions=[
            LogInfo(msg=['[5s] Sistema perception PRONTO']),
        ]),
    ])
