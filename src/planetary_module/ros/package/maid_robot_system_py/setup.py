import os
from glob import glob
from setuptools import setup

package_name = 'maid_robot_system_py'

setup(
    name=package_name,
    version='0.23.7',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # Add sub files.
        (os.path.join('lib', package_name, 'features', 'mp'), glob(os.path.join(package_name , 'features', 'mp', '*.py'))),
        (os.path.join('lib', package_name, 'features'), glob(os.path.join(package_name , 'features', '*.py'))),
        (os.path.join('lib', package_name, 'utils'), glob(os.path.join(package_name , 'utils', '*.py'))),
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
            'face_recognition_node = maid_robot_system_py.face_recognition_node:main',
            'minimal_param_node = maid_robot_system_py.python_parameters_node:main',
        ],
    },
)
