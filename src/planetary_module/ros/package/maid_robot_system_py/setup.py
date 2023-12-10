import os
from glob import glob
from setuptools import setup

package_name = 'maid_robot_system_py'

setup(
    name=package_name,
    version='0.23.7',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Add sub files.
        (os.path.join('lib', package_name, 'utils'), glob(
            os.path.join(package_name, 'utils', '*.py'))),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', 'head_unit', 'input_device', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', 'head_unit', 'recognizer', 'marker', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', 'head_unit', 'recognizer', 'posture', '*.launch.py'))),
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
            'detect_ar_node = maid_robot_system_py.head_unit.recognizer.marker.detect_ar_node:main',
            'mediapipe_node = maid_robot_system_py.head_unit.recognizer.posture.mediapipe_node:main',
            'mediapipe_ext_node = maid_robot_system_py.head_unit.recognizer.posture.mediapipe_ext_node:main',
            'photo_to_video_node = maid_robot_system_py.head_unit.input_device.photo_to_video_node:main',
            'video_capture_node = maid_robot_system_py.head_unit.input_device.video_capture_node:main',
            'video_topic_to_service = maid_robot_system_py.head_unit.input_device.video_topic_to_service:main'
        ],
    },
)
