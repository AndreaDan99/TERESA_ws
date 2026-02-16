from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # TF: body -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='spot_to_camera',
            arguments=[
                '0.40', '0.0', '0.55',  # x, y, z (MISURA QUESTI!)
                '0', '0', '0',          # yaw, pitch, roll
                'body',                 # parent frame (Spot)
                'camera_link'           # child frame
            ]
        ),
        
        # TF: camera_link -> camera_color_optical_frame
        # (rotazione standard optical frame RealSense/Orbbec)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_optical',
            arguments=[
                '0', '0', '0',
                '-1.5708', '0', '-1.5708',  # -90° pitch, -90° yaw
                'camera_link',
                'camera_color_optical_frame'
            ]
        ),
    ])
