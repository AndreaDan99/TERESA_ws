from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'z1_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'interactive_motion = z1_control.interactive_motion:main',
            'z1_state_debug = z1_control.state_debug_node:main',
            'z1_incremental_controller = z1_control.z1_incremental_controller:main',
            'test_home_position = z1_control.home_position:main',
            'test_joint_sweep = z1_control.test_joint_sweep:main',
            'test_sequence = z1_control.test_sequence:main',
            'z1_pd_effort = z1_control.z1_pd_effort:main',
            'z1_pd_gravity = z1_control.z1_pd_gravity_node:main',
            'tuning_logger = z1_control.tuning_logger:main',
            'impedance_controller = z1_control.impedance_controller:main',
            'impedance_test = z1_control.impedance_test:main',
            'impedance_controller_v2 = z1_control.impedance_controller_v2:main',

        ],
    },
)
