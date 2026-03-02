from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_ik_node = LaunchConfiguration('use_ik_node')
    use_impedance_node = LaunchConfiguration('use_impedance_node')

    ik_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'z1_ik_jtc_params.yaml'
    ])

    impedance_params_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'impedance_params.yaml'
    ])

    # =========================================================
    # NODO 1: IK → JointTrajectoryController
    # =========================================================
    ik_to_jtc_node = Node(
        package='z1_vision',
        executable='z1_ik_to_jtc',
        name='z1_ik_to_jtc',
        parameters=[ik_params_file],
        output='screen',
        condition=IfCondition(use_ik_node)
    )

    # =========================================================
    # NODO 2: Impedance Controller
    # =========================================================
    impedance_controller_node = Node(
        package='z1_vision',
        executable='impedance_controller_realsense',
        name='impedance_controller_realsense',
        parameters=[impedance_params_file],
        output='screen',
        condition=IfCondition(use_impedance_node)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ik_node',
            default_value='true',
            description='Launch IK → JointTrajectoryController node'
        ),
        DeclareLaunchArgument(
            'use_impedance_node',
            default_value='true',
            description='Launch impedance controller node'
        ),
        ik_to_jtc_node,
        impedance_controller_node,
    ])
