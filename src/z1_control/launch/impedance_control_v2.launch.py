from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Path al config file
    config_file = PathJoinSubstitution([
        FindPackageShare('z1_control'),
        'config',
        'impedance_control_params.yaml'
    ])
    
    # Nodo controller
    impedance_controller_node = Node(
        package='z1_control',
        executable='impedance_controller_v2',
        name='impedance_controller_v2',
        parameters=[config_file],
        output='screen'
    )
    
    return LaunchDescription([
        impedance_controller_node
    ])
