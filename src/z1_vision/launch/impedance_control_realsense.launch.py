from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('z1_vision'),
        'config',
        'impedance_control_params.yaml'
    ])

    # =========================================================
    # NODO 1: Safe Switch
    # Si avvia per primo, congela il robot, fa lo switch,
    # poi muore da solo
    # =========================================================
    safe_switch_node = Node(
        package='z1_vision',
        executable='safe_controller_switch',
        name='safe_controller_switch',
        output='screen'
    )

    # =========================================================
    # NODO 2: Impedance Controller
    # Si avvia solo DOPO che safe_switch_node è terminato
    # =========================================================
    impedance_controller_node = Node(
        package='z1_vision',
        executable='impedance_controller_realsense',
        name='impedance_controller_realsense',
        parameters=[config_file],
        output='screen'
    )

    # Avvia il controller solo quando safe_switch ha finito
    start_impedance_after_switch = RegisterEventHandler(
        OnProcessExit(
            target_action=safe_switch_node,
            on_exit=[impedance_controller_node]
        )
    )

    return LaunchDescription([
        safe_switch_node,
        start_impedance_after_switch,
    ])
