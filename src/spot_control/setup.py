from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_control'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='Spot navigation, mission coordination and WBC orchestration for TERESA',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_goal_navigator = spot_control.spot_goal_navigator:main',
            'wbc_spot_navigator  = spot_control.wbc_spot_navigator:main',
            'wbc_qp_controller   = spot_control.wbc_qp_controller:main',
            'wbc_coordinator     = spot_control.wbc_coordinator:main',
            'wbc_keyboard_node   = spot_control.wbc_keyboard_controller:main',
            'ik_goal_mux         = spot_control.ik_goal_mux:main',
            'tf_monitor          = spot_control.tf_monitor:main',
            'wbc_approach_scanner = spot_control.wbc_approach_scanner:main',
            'experiment_logger    = spot_control.experiment_logger:main',
            'exposure_scanner    = spot_control.exposure_scanner:main',
            'exposure_snapshot   = spot_control.exposure_snapshot:main',
        ],
    },
)
