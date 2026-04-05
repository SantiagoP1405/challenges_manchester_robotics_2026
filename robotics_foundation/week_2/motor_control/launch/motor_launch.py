from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node = Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       parameters=[{
                        'sample_time': 0.02,
                        'sys_gain_K': 2.16, #1.75
                        'sys_tau_T': 0.05,   #0.5
                        'initial_conditions': 0.0,
                            }
                        ]
                    )
    sp_node = Node(name="sp_gen",
                        package='motor_control',
                        executable='set_point',
                        emulate_tty=True,
                        output='screen',
                        parameters=[{
                            'amplitude': 2.0,
                            'omega': 1.0,
                            'timer_period': 0.02,
                            'signal_type': 'sine',
                        }]
                       )
    controller_node = Node(name="controller",
                       package='motor_control',
                       executable='controller',
                       emulate_tty=True,
                       output='screen',
                       parameters=[{
                            't_s': 0.02,
                            'kp': 0.002454422522133, #0.0371615460917942
                            'ki': 2.35908845044266, #0.829038652294856 
                            'kd': 0.0
                            }
                        ]
                       )
                       
    rqt_plot_node = Node(
    package='rqt_plot',
    executable='rqt_plot',
    name='plot',
    arguments=[
        '/set_point/data',
        '/motor_input_u/data',
        '/motor_output_y/data',
    ],
    output='screen'
    )

    rqt_reconfigure_node = Node(
    package='rqt_reconfigure',
    executable='rqt_reconfigure',
    name='reconfigure',
    output='screen'
    )
    
    l_d = LaunchDescription([sp_node, controller_node, motor_node, rqt_plot_node, rqt_reconfigure_node])
    #l_d = LaunchDescription([sp_node, controller_node, motor_node])

    return l_d