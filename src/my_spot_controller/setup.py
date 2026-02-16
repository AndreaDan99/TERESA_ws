from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_spot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Launch files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Spot ROS2 controller with navigation and perception',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # NAVIGATION             
            'spot_startup_node = ' + package_name + '.navigation.spot_startup_node:main',
            'target_tracker = ' + package_name + '.navigation.target_tracker:main',
            'spot_camera_assistant = ' + package_name + '.navigation.spot_camera_assistant:main',
            'human_target_generator = ' + package_name + '.navigation.human_target_generator:main', 
            'nav2_goal_sender = ' + package_name + '.navigation.nav2_goal_sender:main',  

            #PERCEPTION
            'human_posture_analyzer_spot = ' + package_name + '.perception.human_posture_analyzer_spot:main',
            'yolo_skeleton_node_orbbec = ' + package_name + '.perception.yolo_skeleton_node_orbbec:main',
            'human_bounding_box_visualizer = ' + package_name + '.perception.human_bounding_box_visualizer:main',
        ],
    },
)
