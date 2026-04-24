import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_closeloop'

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
    maintainer='ximena',
    maintainer_email='ximena@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_sqr= puzzlebot_closeloop.controller_sqr:main',
            'puzzlebot_odometry= puzzlebot_closeloop.puzzlebot_odometry:main',
            'path_generator_node= puzzlebot_closeloop.path_generator_node:main',
            'controller_path= puzzlebot_closeloop.controller_path:main',
        ],
    },
)
