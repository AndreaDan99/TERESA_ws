#!/usr/bin/env python3
"""
Teresa Perception Launch — Jetson + Orbbec Femto Bolt + Z1 RealSense
Avvia: Orbbec driver, TF statiche, yolo_skeleton, posture_analyzer,
       bounding_box_visualizer, laying_human_detector, z1_perception
PREREQUISITO: spot_ros2 già avviato su SpotCore (via DDS)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    use_orbbec_driver_arg = DeclareLaunchArgument(
        'use_orbbec_driver',
        default_value='false',
        description='Lancia driver Orbbec + TF statiche. false se già in teresa_core'
    )
    use_orbbec_driver = LaunchConfiguration('use_orbbec_driver')

    perception_backend_arg = DeclareLaunchArgument(
        'perception_backend',
        default_value='yolo',
        description='Perception backend: yolo (default) or nlf'
    )
    perception_backend = LaunchConfiguration('perception_backend')

    # Config files
    nlf_params_file = PathJoinSubstitution([
        FindPackageShare('spot_perception'), 'config', 'nlf_params.yaml']) 

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
            'camera_name': 'orbbec',
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
            'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false',
            'enable_ir': 'false',
            'enable_accel': 'false',
            'enable_gyro': 'false',
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(use_orbbec_driver),
    )

    # ============================================================
    # 2) TF STATICHE: body → orbbec_link → orbbec_color_optical_frame
    # ============================================================
    # Adatta x, y, z in base al mount fisico dell'Orbbec su Spot
    static_tf_body_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='spot_to_orbbec',
        arguments=[
            '0.30', '0.0', '0.15',
            '0', '0', '0',
            'my_spot/body',
            'orbbec_link'
        ]
    )

    static_tf_camera_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='orbbec_to_optical',
        arguments=[
            '0', '0', '0',
            '-1.5708', '0', '-1.5708',
            'orbbec_link',
            'orbbec_color_optical_frame'
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
        }],
        condition=IfCondition(PythonExpression(['"', perception_backend, '" == "yolo"']))
    )

    # ============================================================
    # 3b) NLF SKELETON (Orbbec) — always launched, idle until /nlf/trigger
    # ============================================================
    nlf_skeleton_node = Node(
        package='spot_perception',
        executable='nlf_skeleton',
        name='nlf_skeleton',
        output='screen',
        parameters=[nlf_params_file],
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
            'frame_id': 'orbbec_color_optical_frame',
            'knee_angle_stand_min': 160.0,
            'knee_angle_sit_max': 120.0,
            'torso_angle_lying_min': 65.0,
            'verticality_ratio_lying_max': 0.30,
            'knee_angle_lying_bonus_max': 140.0,
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
            'frame_id': 'orbbec_color_optical_frame',
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
    # 7) Z1 PERCEPTION (Realsense torso tracker + surface node)
    # ============================================================
    z1_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('z1_vision'),
                'launch',
                'z1_perception.launch.py'
            ])
        ]),
    )

    # ============================================================
    # SEQUENZA DI AVVIO
    # ============================================================
    return LaunchDescription([
        test_mode_arg,
        use_orbbec_driver_arg,
        perception_backend_arg,

        LogInfo(msg=['Teresa Perception System — Jetson + Orbbec Femto Bolt + Z1']),
        LogInfo(msg=['   Topics: /orbbec/color/image_raw + /orbbec/depth/image_raw']),
        LogInfo(msg=['   Frame output: orbbec_color_optical_frame']),

        # Orbbec subito (solo se use_orbbec_driver=true)
        orbbec_launch,

        # TF statiche dopo 2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg=['[2s] TF statiche: body → orbbec_link → orbbec_color_optical_frame']),
            static_tf_body_camera,
            static_tf_camera_optical,
        ]),

        # Perception: 4s se Orbbec parte qui, 1s se già avviato dal core
        # Sceglie backend in base a perception_backend (nlf o yolo)
        TimerAction(period=PythonExpression([
            '4.0 if "', use_orbbec_driver, '" == "true" else 1.0'
        ]), actions=[
            LogInfo(msg=['Perception backend: ', perception_backend]),
            # NLF backend (default)
            nlf_skeleton_node,
            # YOLO backend (perception_backend:=yolo)
            yolo_skeleton_node,
            posture_analyzer_node,
            bbox_visualizer_node,
            laying_detector_node,
        ]),

        # Z1 perception (Realsense torso tracker + surface node)
        z1_perception_launch,

        TimerAction(period=PythonExpression([
            '5.0 if "', use_orbbec_driver, '" == "true" else 2.0'
        ]), actions=[
            LogInfo(msg=['Sistema perception PRONTO']),
        ]),
    ])
