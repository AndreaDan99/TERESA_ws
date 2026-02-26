#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown


def generate_launch_description():
    # Launch arguments
    camera_x = LaunchConfiguration('camera_x')
    camera_y = LaunchConfiguration('camera_y')
    camera_z = LaunchConfiguration('camera_z')
    camera_qx = LaunchConfiguration('camera_qx')
    camera_qy = LaunchConfiguration('camera_qy')
    camera_qz = LaunchConfiguration('camera_qz')
    camera_qw = LaunchConfiguration('camera_qw')
    parent_frame = LaunchConfiguration('parent_frame')
    use_surface_node = LaunchConfiguration('use_surface_node')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Package paths
    z1_bringup_pkg = FindPackageShare('z1_bringup')
    realsense_pkg = FindPackageShare('realsense2_camera')
    z1_vision_pkg = FindPackageShare('z1_vision')
    
    # Custom RViz config
    rviz_config = PathJoinSubstitution([
        z1_vision_pkg, 'rviz', 'z1_realsense.rviz'
    ])

    # ============== JOINT TRAJECTORY START COMMAND ==============
    go_to_start_conf = TimerAction(
        period=5.0,   # qualche secondo dopo l'avvio del controller di traiettoria
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/joint_trajectory_controller/joint_trajectory',
                    'trajectory_msgs/msg/JointTrajectory',
                    '{header: {stamp: {sec: 0, nanosec: 0}}, '
                    "joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], "
                    'points: [{positions: [0.0, 1.0, -1.0, 0.0, 0.0, 0.0], '
                    'time_from_start: {sec: 10, nanosec: 0}}]}'
                ],
                output='screen'
            )
        ]
    )
    go_to_safe_conf = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/joint_trajectory_controller/joint_trajectory',
            'trajectory_msgs/msg/JointTrajectory',
            '{header: {stamp: {sec: 0, nanosec: 0}}, '
            "joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], "
            'points: [{positions: [0.0, 0.0, -0.1, 0.0, 0.0, 0.0], '
            'time_from_start: {sec: 5, nanosec: 0}}]}'
        ],
        output='screen'
    )

    on_shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                go_to_safe_conf
            ]
        )
    )
    
    return LaunchDescription([
        # ============== ARGUMENTS ==============
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.0',
            description='Camera X offset from parent frame [m]'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera Y offset from parent frame [m]'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.05',  # ← 5cm SOPRA
            description='Camera Z offset from parent frame [m]'
        ),
        DeclareLaunchArgument(
            'camera_qx',
            default_value='0.0',  
            description='Camera quaternion X'
        ),
        DeclareLaunchArgument(
            'camera_qy',
            default_value='0.0',
            description='Camera quaternion Y'
        ),
        DeclareLaunchArgument(
            'camera_qz',
            default_value='0.0',
            description='Camera quaternion Z'
        ),
        DeclareLaunchArgument(
            'camera_qw',
            default_value='1.0', 
            description='Camera quaternion W'
        ),
        DeclareLaunchArgument(
            'parent_frame',
            default_value='link06',
            description='Parent frame for camera mounting'
        ),
        DeclareLaunchArgument(
            'use_surface_node',
            default_value='true',
            description='Launch surface detection node'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with custom config'
        ),
        
        # ============== Z1 ROBOT (RViz DISABILITATO) ==============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    z1_bringup_pkg, 'launch', 'z1.launch.py'
                ])
            ]),
            launch_arguments={
                'sim_ignition': 'true',
                'starting_controller': 'joint_trajectory_controller',
                'with_gripper': 'false',
                'rviz': 'false',
            }.items()
        ),

        # comando di traiettoria iniziale
        go_to_start_conf,
        
        # ============== REALSENSE CAMERA ==============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    realsense_pkg, 'launch', 'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'pointcloud.enable': 'true',
                'colorizer.enable': 'false',
                'align_depth.enable': 'true',
            }.items()
        ),
        
        # ============== CAMERA TF ==============
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                camera_x, camera_y, camera_z,
                camera_qx, camera_qy, camera_qz, camera_qw,
                parent_frame, 'camera_link'
            ],
            output='screen'
        ),
        
        # ============== SURFACE DETECTION NODE ==============
        Node(
            package='z1_vision',
            executable='realsense_surface_node',
            name='realsense_surface_node',
            parameters=[{
                'ee_frame': 'link06',
                'camera_frame': 'camera_depth_optical_frame',
                'base_frame': 'world',
                'patch_radius_px': 30,
                'min_depth': 0.05,
                'max_depth': 2.0,
            }],
            output='screen',
            condition=IfCondition(use_surface_node)
        ),
        
        # ============== RVIZ CUSTOM ==============
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(use_rviz)
        ),

        on_shutdown_handler,  
    ])
