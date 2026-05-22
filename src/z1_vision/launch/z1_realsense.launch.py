#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition



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
    joint_offset = LaunchConfiguration('joint_offset')
    #use_surface_node = LaunchConfiguration('use_surface_node')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera_tf = LaunchConfiguration('use_camera_tf')
    
    # Package paths
    z1_bringup_pkg = FindPackageShare('z1_bringup')
    realsense_pkg = FindPackageShare('realsense2_camera')
    z1_vision_pkg = FindPackageShare('z1_vision')
    
    # Custom RViz config
    rviz_config = PathJoinSubstitution([
        z1_vision_pkg, 'rviz', 'z1_realsense.rviz'
    ])
    
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
            'use_rviz',
            default_value='true',
            description='Launch RViz with custom config'
        ),
        DeclareLaunchArgument(
            'use_camera_tf',
            default_value='true',
            description='Publish static TF link06→camera_link. false se gestita da teresa_core'
        ),
        DeclareLaunchArgument(
            'joint_offset',
            default_value='0.0 0.69 0.0 0.0 0.0 0.0',
            description='Joint encoder offset [rad] — joint2 folded ~40° for Spot mount'
        ),
        
        # ============== Z1 ROBOT (RViz DISABILITATO) ==============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    z1_bringup_pkg, 'launch', 'z1.launch.py'
                ])
            ]),
            launch_arguments={
                'sim_ignition': 'false',
                'starting_controller': 'joint_trajectory_controller',
                'with_gripper': 'false',
                'rviz': 'false',
                'joint_offset': joint_offset,
            }.items()
        ),
        

        
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
                'log_level': 'info',
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
            output='screen',
            condition=IfCondition(use_camera_tf)
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

    ])
