from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import numpy as np

def generate_launch_description():
    input_node = Node(name="input",
                      package='final_challenge',
                      executable='input_node',
                      emulate_tty=True,
                      output='screen',
                      parameters=[{
                            'amplitude':  22 * (2*np.pi / 60),
                            'omega': 0.3,
                            'timer_period': 0.02,
                            'signal_type': 'sine',
                        }]
                    )
    controller_node = Node(name="controller",
                       package='final_challenge',
                       executable='controller',
                       emulate_tty=True,
                       output='screen',
                       parameters=[{
                            'Ts': 0.005,
                            #'kp': 0.90644619633467,
                            'kp': 0.0874503393459739,
                            #'ki': 2.73987747995308,
                            'ki': 2.54986519821649,
                            'kd': 0.0
                            }
                        ]
                    )
    micro_ros_agent_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyACM0',
            '-b', '115200'
        ],
        output='screen'
    )

    rqt_plot_node = Node(
    package='rqt_plot',
    executable='rqt_plot',
    name='plot',
    arguments=[
        '/set_point/data',
        '/motor_input/data',
        '/motor_w/data',
    ],
    output='screen'
    )

    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='reconfigure',
        output='screen'
    )

    return LaunchDescription([
        input_node,
        controller_node,
        micro_ros_agent_node,
        rqt_plot_node,
        rqt_reconfigure_node
    ])