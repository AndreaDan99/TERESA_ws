# Copyright 2025 IDRA, University of Trento
# Author: Matteo Dalle Vedove (matteodv99tn@gmail.com)

from launch.event_handlers.on_process_exit import OnProcessExit
import xacro
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_path,
)


def launch_setup(context, *args, **kwargs):


    nodes_to_start = list()
        # ------------------------------------------------------------
    # 🔴 CLOCK BRIDGE (OBBLIGATORIO SU ROS 2 JAZZY)
    # ------------------------------------------------------------
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    nodes_to_start.append(clock_bridge)
    
    xacro_file = LaunchConfiguration("xacro_file")
    robot_name = LaunchConfiguration("robot_name")
    with_gripper = LaunchConfiguration("with_gripper")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    controller_config = LaunchConfiguration("controller_config")
    starting_controller = LaunchConfiguration("starting_controller")
    sim_ignition = LaunchConfiguration("sim_ignition")

    use_sim_time = (sim_ignition.perform(context) == "true")

    is_simulation = IfCondition(sim_ignition)
    is_real = UnlessCondition(sim_ignition)

    # ------------------------------------------------------------
    # 🔴 FIX FONDAMENTALE:
    # forza use_sim_time GLOBALMENTE (vale anche per gz_ros2_control)
    # ------------------------------------------------------------
    nodes_to_start.append(SetParameter(name="use_sim_time", value=use_sim_time))

    # ------------------------------------------------------------
    # Robot description
    # ------------------------------------------------------------
    robot_description_content = xacro.process(
        xacro_file.perform(context),
        mappings={
            "name": robot_name.perform(context),
            "prefix": "",
            "with_gripper": with_gripper.perform(context),
            "controllers": controller_config.perform(context),
            "sim_ignition": sim_ignition.perform(context),
        }
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time, "set_state": "active"}],
    )

    starting_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[starting_controller.perform(context), "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time, "set_state": "active"}],
    )

    torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["torque_controller", "-c", "/controller_manager", "--inactive"],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=is_real,  # Solo per robot reale
    )

    nodes_to_start.append(torque_controller_spawner)
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config.perform(context)],
        condition=IfCondition(rviz),
    )

    rviz_delayed = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start += [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        starting_controller_spawner,
        rviz_delayed,
    ]

    # ------------------------------------------------------------
    # Real robot (unchanged)
    # ------------------------------------------------------------
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
        ],
        condition=is_real,
    )

    z1_controller_script_path = os.path.join(
        get_package_share_path("z1_hardware_interface"),
        "scripts",
        "z1_controller_process.py"
    )
    z1_controller_process = ExecuteProcess(
        cmd=["python3", z1_controller_script_path],
        condition=is_real,
        output="screen",
    )

    nodes_to_start += [
        controller_manager_node,
        z1_controller_process,
    ]

    # ------------------------------------------------------------
    # Gazebo Sim
    # ------------------------------------------------------------
    ignition_simulator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 1 empty.sdf"}.items(),
        condition=is_simulation,
    )

    ignition_spawn_z1_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "/robot_description",
        ],
        condition=is_simulation,
    )

    nodes_to_start += [
        ignition_simulator_node,
        ignition_spawn_z1_node,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    rviz_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "rviz", "z1.rviz"
    )
    xacro_file_default = os.path.join(
        get_package_share_path("z1_description"), "urdf", "z1.urdf.xacro"
    )
    controller_config_default = os.path.join(
        get_package_share_path("z1_bringup"), "config", "z1_controllers.yaml"
    )

    # ------------------------------------------------------------
    # 🔴 FIX VARIABILI AMBIENTE (gz, non ign)
    # ------------------------------------------------------------
    MDL_ENV_VAR = "GZ_SIM_RESOURCE_PATH"
    if MDL_ENV_VAR in os.environ:
        os.environ[MDL_ENV_VAR] += ":" + os.path.join(
            get_package_prefix("z1_description"), "share"
        )
    else:
        os.environ[MDL_ENV_VAR] = os.path.join(
            get_package_prefix("z1_description"), "share"
        )

    LIB_ENV_VAR = "GZ_SIM_SYSTEM_PLUGIN_PATH"
    if LIB_ENV_VAR in os.environ:
        os.environ[LIB_ENV_VAR] += ":/opt/ros/jazzy/lib"
    else:
        os.environ[LIB_ENV_VAR] = "/opt/ros/jazzy/lib"

    # ------------------------------------------------------------
    # Launch arguments (UNTOUCHED)
    # ------------------------------------------------------------
    declared_arguments += [
        DeclareLaunchArgument("xacro_file", default_value=xacro_file_default),
        DeclareLaunchArgument("robot_name", default_value="z1"),
        DeclareLaunchArgument("controller_config", default_value=controller_config_default),
        DeclareLaunchArgument("with_gripper", default_value="true"),
        DeclareLaunchArgument("starting_controller", default_value="torque_controller"),
        DeclareLaunchArgument("sim_ignition", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_config_default),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
