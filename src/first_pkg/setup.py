from setuptools import setup
import os
from glob import glob

package_name = 'first_pkg'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf/robot'), glob('urdf/robot/*.xacro')),
        (os.path.join('share', package_name, 'urdf/robot/plugins'), glob('urdf/robot/plugins/*.xacro')),
        (os.path.join('share', package_name, 'urdf/robot/sensor'), glob('urdf/robot/sensor/*.xacro')),
        (os.path.join('share', package_name, 'urdf/robot/actor'), glob('urdf/robot/actor/*.xacro')),
        (os.path.join('share', package_name, 'map'), glob('map/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dong',
    maintainer_email='dchengteng@gmail.com',
    description='ROS2 Robot Navigation and Exploration System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_node = first_pkg.main:main',
        ],
    },
)
