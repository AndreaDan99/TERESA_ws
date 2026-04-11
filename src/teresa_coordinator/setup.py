from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teresa_coordinator'

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
    description='Mission coordinator for TERESA: Spot navigation + Z1 FAST scan orchestration',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teresa_mission = teresa_coordinator.teresa_mission:main',
        ],
    },
)
