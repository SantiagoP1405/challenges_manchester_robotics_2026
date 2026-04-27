import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_tuned_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santiago_gomez',
    maintainer_email='a01735171@tec.mx',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'puzzlebot_odometry = puzzlebot_tuned_controller.puzzlebot_odometry:main',
            'path_generator_node = puzzlebot_tuned_controller.path_generator_node:main',
            'controller_path = puzzlebot_tuned_controller.controller_path:main',
            'traffic_light_node = puzzlebot_tuned_controller.traffic_light_node:main',
        ],
    },
)
