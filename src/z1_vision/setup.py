from setuptools import setup
import os
from glob import glob

package_name = 'z1_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        
        # RViz configs
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Z1 robot with RealSense camera integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [            
            'impedance_controller_realsense = z1_vision.impedance_controller_realsense:main',
            'realsense_surface_node = z1_vision.realsense_surface_node:main',
        ],
    },
)
