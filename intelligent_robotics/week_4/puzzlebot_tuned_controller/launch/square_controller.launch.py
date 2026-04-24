from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_tuned_controller')

    params_file = os.path.join(pkg_share, 'config', 'puzzlebot_params.yaml')

    return LaunchDescription([
        Node(
            package='puzzlebot_tuned_controller',
            executable='puzzlebot_odometry',
            name='puzzlebot_odometry',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='puzzlebot_tuned_controller',
            executable='controller_sqr',
            name='controller_sqr',
            parameters=[params_file],
            output='screen'
        )
    ])