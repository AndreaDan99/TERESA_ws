from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Carica robot_description
    xacro_file = os.path.join(
        get_package_share_directory('z1_description'),
        'urdf', 'z1.urdf.xacro'
    )
    
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' name:=z1',
        ' prefix:=',
        ' with_gripper:=true',
        ' sim_ignition:=true',
        ' controllers:=', os.path.join(
            get_package_share_directory('z1_bringup'),
            'config', 'z1_controllers.yaml'
        )
    ])
    
    # Wrappa in ParameterValue
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # 👇 AGGIUNGI: Percorso al file YAML con parametri PID
    config_file = os.path.join(
        get_package_share_directory('z1_control'),
        'config',
        'z1_pid_params.yaml'
    )

    # Nodo PD con gravity
    pd_node = Node(
        package='z1_control',
        executable='z1_pd_gravity',
        output='screen',
        parameters=[
            config_file,  # 👈 AGGIUNGI: carica parametri PID da YAML
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([pd_node])
