import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('puzzlebot_open_loop'),
        'config',
        'params.yaml'
    )

    path_generator_node = Node(
        package='puzzlebot_open_loop',
        executable='path_generator',
        name='path_generator',
        parameters=[config],
        output='screen'
    )

    controller_node = Node(
        package='puzzlebot_open_loop',
        executable='controller',
        name='controller',
        output='screen'
    )

    return LaunchDescription([
        path_generator_node,
        controller_node
    ])