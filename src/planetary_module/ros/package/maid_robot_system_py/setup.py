import os
from glob import glob
from setuptools import setup

package_name = 'maid_robot_system_py'

setup(
    name=package_name,
    version='0.23.7',
    packages=[package_name],
    data_files=[
        # Add package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Add sub files.
        (os.path.join('lib', package_name, 'utils'), glob(
            os.path.join(package_name, 'utils', '*.py'))),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch/*', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akari',
    maintainer_email='developer@masiro-project.com',
    description='This package is for running Maid Robot System for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_capture_node = maid_robot_system_py.video_capture_node:main',
            'detect_ar_node = maid_robot_system_py.detect_ar_node:main',
            'mediapipe_node = maid_robot_system_py.mediapipe_node:main',
            'mediapipe_ext_node = maid_robot_system_py.mediapipe_ext_node:main'
        ],
    },
)
