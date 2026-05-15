from setuptools import find_packages, setup

package_name = 'puzzlebot_line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'pure_pursuit_lane_controller = puzzlebot_line_follower.pure_pursuit_lane_controller:main',
            
        ],
    },
)
