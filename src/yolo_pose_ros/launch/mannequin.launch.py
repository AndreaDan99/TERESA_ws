from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory("yolo_pose_ros")
    urdf_path = os.path.join(pkg_path, "urdf", "human_mannequin.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),

        Node(
            package="yolo_pose_ros",
            executable="yolo_skeleton_node_kf_mannequin",
            output="screen",
        ),
    ])
