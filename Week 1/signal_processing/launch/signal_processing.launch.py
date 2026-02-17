from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    # Signal generator node
    signal_generator_node= Node(
        package='signal_processing',          
        executable='signal_generator',    
    )

    # Signal processing node
    process_node= Node(
        package='signal_processing',      
        executable='process',    
    )

    rqt_plot_node = Node(
    package='rqt_plot',
    executable='rqt_plot',
    name='plot',
    arguments=[
        '/signal/data',
        '/proc_signal/data'
    ],
    output='screen'
    )


    # Launch nodes
    return LaunchDescription([
        signal_generator_node,
        process_node,
        rqt_plot_node
    ])


