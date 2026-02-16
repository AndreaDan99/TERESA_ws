from setuptools import setup
from glob import glob
import os

package_name = 'yolo_pose_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # <-- IMPORTANTE
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        
        # install launch files
        (os.path.join('share', 'yolo_pose_ros', 'launch'),
            glob('launch/*.py')),

        # (opzionale ma consigliato)
        (os.path.join('share', 'yolo_pose_ros', 'urdf'),
            glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea@todo.todo',
    description='YOLO Pose integration with RealSense',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'yolo_pose_node_old = yolo_pose_ros.old_nodes.yolo_pose_node:main',
            'yolo_skeleton_node_old = yolo_pose_ros.old_nodes.yolo_skeleton_node:main',
            'yolo_skeleton_node_v2 = yolo_pose_ros.old_nodes.yolo_skeleton_node_v2:main',
            'yolo_skeleton_node_smooth = yolo_pose_ros.old_nodes.yolo_skeleton_node_smooth:main',
            'yolo_skeleton_node_kf = yolo_pose_ros.old_nodes.yolo_skeleton_node_kf:main',
            'yolo_skeleton_node_kf_calib = yolo_pose_ros.old_nodes.yolo_skeleton_node_kf_calib:main',
            'yolo_smpl_mesh_node = yolo_pose_ros.old_nodes.yolo_smpl_mesh_node:main',
            'yolo_hand_node = yolo_pose_ros.old_nodes.yolo_hand_node:main',
            'debug_depth = yolo_pose_ros.old_nodes.debug_depth:main',
            'face_landmarks_node = yolo_pose_ros.old_nodes.face_landmarks_node:main',
            'yolo_skeleton_node_kf_mannequin = yolo_pose_ros.old_nodes.yolo_skeleton_node_kf_mannequin:main',
            'yolo_skeleton_node_stable = yolo_pose_ros.old_nodes.yolo_skeleton_node_stable:main',
            'yolo_smpl_mesh_node_v2 = yolo_pose_ros.old_nodes.yolo_smpl_mesh_node_v2:main',
            'yolo_skeleton_node_kf_calib_v2 = yolo_pose_ros.yolo_skeleton_node_kf_calib_v2:main',


            'yolo_skeleton_node_v1 = yolo_pose_ros.yolo_skeleton_node_v1:main',
            'human_posture = yolo_pose_ros.human_posture_analyzer_node:main',
            'yolo_skeleton_node = yolo_pose_ros.yolo_skeleton_node:main',
            'yolo_multi_node = yolo_pose_ros.yolo_multitracking_node:main',
        ],
    },
)
