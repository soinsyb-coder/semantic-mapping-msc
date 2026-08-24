from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'semantic_perception'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laptop4',
    maintainer_email='laptop4@todo.todo',
    description='ROS 2 semantic perception and mapping package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'image_passthrough_node = '
                'semantic_perception.image_passthrough_node:main'
            ),
            (
                'yolo_detector_node = '
                'semantic_perception.yolo_detector_node:main'
            ),
            (
                'rgbd_yolo_localizer_node = '
                'semantic_perception.rgbd_yolo_localizer_node:main'
            ),
            (
                'slam_drift_diagnostic_node = '
                'semantic_perception.slam_drift_diagnostic_node:main'
            ),
        ],
    },
)
