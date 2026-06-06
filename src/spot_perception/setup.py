from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='Orbbec-based human perception for Spot',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_skeleton_node_orbbec = spot_perception.yolo_skeleton_spot:main',
            'human_posture_analyzer_spot = spot_perception.posture_classifier:main',
            'human_bounding_box_visualizer = spot_perception.human_bounding_box_visualizer:main',
            'laying_human_detector = spot_perception.laying_human_detector:main',
            'nlf_skeleton = spot_perception.nlf_skeleton:main',
        ],
    },
)
