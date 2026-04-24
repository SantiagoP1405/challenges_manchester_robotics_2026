import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_closeloop')
    params_file = os.path.join(pkg_share, 'config', 'puzzlebot_params.yaml')

    odom_node = Node(
        package='puzzlebot_closeloop',
        executable='puzzlebot_odometry', 
        name='puzzlebot_odometry',
        parameters=[params_file],
        output='screen'
    )

    controller_node = Node(
        package='puzzlebot_closeloop',
        executable='controller_path',
        name='controller_path',
        parameters=[params_file],
        output='screen'
    )

    path_gen_node = Node(
        package='puzzlebot_closeloop',
        executable='path_generator_node',
        name='path_generator_node',
        parameters=[params_file],
        output='screen',

    )

    return LaunchDescription([
        odom_node,
        controller_node,
        path_gen_node
    ])