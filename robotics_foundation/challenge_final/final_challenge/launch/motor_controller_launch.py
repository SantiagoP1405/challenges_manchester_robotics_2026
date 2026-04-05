from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import numpy as np

def generate_launch_description():

    # Input Signal Node
    # Generates the reference signal (setpoint) for the motor.
    # The signal can be sine, square, or step depending on the
    # 'signal_type' parameter. In this case a sine signal is used.
    # The amplitude corresponds to the maximum motor speed
    # converted from RPM to rad/s.
    input_node = Node(
        name="input",
        package='final_challenge',
        executable='input_node',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'amplitude': 17 * (2*np.pi / 60),  # 17 RPM converted to rad/s
            'omega': 0.3,                      # Angular frequency of the input signal (rad/s)
            'timer_period': 0.02,              # Signal update period (50 Hz)
            'signal_type': 'sine',             # Type of signal generated
        }]
    )
  
    # Controller Node
    # Implements the PID motor speed controller.
    # The controller receives the reference signal and the
    # measured motor speed and computes the normalized control
    # input for the motor driver.  
    controller_node = Node(
        name="controller",
        package='final_challenge',
        executable='controller',
        emulate_tty=True,
        output='screen',
        parameters=[{
            'Ts': 0.005,                       # Controller sampling time (200 Hz)
            #'kp': 0.90644619633467,           # Previous tuning value (commented)
            'kp': 0.0874503393459739,          # Proportional gain
            #'ki': 2.73987747995308,           # Previous tuning value (commented)
            'ki': 2.54986519821649,            # Integral gain
            'kd': 0.0                          # Derivative gain (not used)
        }]
    )

    # micro-ROS Agent
    # This process bridges ROS2 running on the PC with the
    # microcontroller (ESP32) through a serial connection.
    # It allows the embedded system to communicate with the
    # ROS2 ecosystem.   
    micro_ros_agent_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyACM0',           # Serial device connected to ESP32
            '-b', '115200'                     # Serial baudrate
        ],
        output='screen'
    )
    
    # rqt_plot
    # Real-time visualization tool used to monitor key signals
    # in the control loop:
    #   - set_point      → desired speed
    #   - motor_input    → controller output (PWM command)
    #   - motor_w        → measured motor speed  
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
  
    # rqt_reconfigure
    # Graphical interface that allows dynamic modification
    # of node parameters (e.g., PID gains) during runtime.
    # Useful for controller tuning without restarting nodes.
    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='reconfigure',
        output='screen'
    )

    # Launch all nodes simultaneously    
    return LaunchDescription([
        input_node,
        controller_node,
        micro_ros_agent_node,
        rqt_plot_node,
        rqt_reconfigure_node
    ])