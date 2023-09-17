import os
from glob import glob
from setuptools import setup

package_name = 'dev_maid_robot_system'

setup(
    name=package_name,
    version='0.23.7',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch/*', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akari',
    maintainer_email='developer@masiro-project.com',
    description='This package is for Maid Robot System(Develop) for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'photo_to_video_node = dev_maid_robot_system.photo_to_video_node:main'
        ],
    },
)
