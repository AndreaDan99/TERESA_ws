from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'teresa_demo'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrea.dantona@unife.it',
    description='Visitor demo: Spot + Z1 arm simultaneous search movements',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visitor_demo = teresa_demo.visitor_demo_node:main',
        ],
    },
)
