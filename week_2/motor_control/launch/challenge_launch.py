from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_node_1= Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace='group1',
                       parameters=[{
                        'sample_time': 0.02,
                        'sys_gain_K': 1.75,
                        'sys_tau_T': 0.5,
                        'initial_conditions': 0.0,
                            }
                        ]
                    )
    
    sp_node_1 = Node(name="sp_gen",
                        package='motor_control',
                        executable='set_point',
                        emulate_tty=True,
                        output='screen',
                        namespace='group1',
                        parameters=[{
                            'amplitude': 2.0,
                            'omega': 1.0,
                            'timer_period': 0.02,
                            'signal_type': 'sine',
                        }]
                       )
    
    controller_node_1 = Node(name="controller",
                       package='motor_control',
                       executable='controller',
                       emulate_tty=True,
                       output='screen',
                       namespace='group1',
                       parameters=[{
                            't_s': 0.02,
                            'kp': 0.0371615460917942,
                            'ki': 0.829038652294856,
                            'kd': 0.0
                            }
                        ]
                       )
    
    motor_node_2= Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace='group2',
                       parameters=[{
                        'sample_time': 0.02,
                        'sys_gain_K': 1.75,
                        'sys_tau_T': 0.5,
                        'initial_conditions': 0.0,
                            }
                        ]
                    )
    
    sp_node_2 = Node(name="sp_gen",
                        package='motor_control',
                        executable='set_point',
                        emulate_tty=True,
                        output='screen',
                        namespace='group2',
                        parameters=[{
                            'amplitude': 2.0,
                            'omega': 1.0,
                            'timer_period': 0.02,
                            'signal_type': 'sine',
                        }]
                       )
    
    controller_node_2 = Node(name="controller",
                       package='motor_control',
                       executable='controller',
                       emulate_tty=True,
                       output='screen',
                       namespace='group2',
                       parameters=[{
                            't_s': 0.02,
                            'kp': 0.0371615460917942,
                            'ki': 0.829038652294856,
                            'kd': 0.0
                            }
                        ]
                       )
    
    motor_node_3= Node(name="motor_sys",
                       package='motor_control',
                       executable='dc_motor',
                       emulate_tty=True,
                       output='screen',
                       namespace='group3',
                       parameters=[{
                        'sample_time': 0.02,
                        'sys_gain_K': 1.75,
                        'sys_tau_T': 0.5,
                        'initial_conditions': 0.0,
                            }
                        ]
                    )
    
    sp_node_3 = Node(name="sp_gen",
                        package='motor_control',
                        executable='set_point',
                        emulate_tty=True,
                        output='screen',
                        namespace='group3',
                        parameters=[{
                            'amplitude': 2.0,
                            'omega': 1.0,
                            'timer_period': 0.02,
                            'signal_type': 'sine',
                        }]
                       )
    
    controller_node_3 = Node(name="controller",
                       package='motor_control',
                       executable='controller',
                       emulate_tty=True,
                       output='screen',
                       namespace='group3',
                       parameters=[{
                            't_s': 0.02,
                            'kp': 0.0371615460917942,
                            'ki': 0.829038652294856,
                            'kd': 0.0
                            }
                        ]
                       )
                       
    # rqt_plot_node = Node(
    #     package='rqt_plot',
    #     executable='rqt_plot',
    #     name='plot',
    #     arguments=[
    #         '/group1/set_point/data',
    #         '/group1/motor_input_u/data',
    #         '/group1/motor_output_y/data',
    #     ],
    #     output='screen'
    # )

    # rqt_reconfigure_node = Node(
    #     package='rqt_reconfigure',
    #     executable='rqt_reconfigure',
    #     name='reconfigure',
    #     output='screen'
    # )
    
    # l_d = LaunchDescription([sp_node_1, controller_node_1, motor_node_1, sp_node_2, controller_node_2, motor_node_2, sp_node_3, controller_node_3, motor_node_3, rqt_plot_node, rqt_reconfigure_node])
    l_d = LaunchDescription([sp_node_1, controller_node_1, motor_node_1, sp_node_2, controller_node_2, motor_node_2, sp_node_3, controller_node_3, motor_node_3])

    return l_d