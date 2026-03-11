#!/usr/bin/env python3
"""
motor_identification_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ROS2 node for open-loop DC motor identification.

Publishes (plottable in rqt_plot):
  /motor/pwm_input   [std_msgs/Float64]  PWM input signal (-1 to 1)
  /motor/speed_rpm   [std_msgs/Float64]  Measured speed (RPM)
  /motor/position    [std_msgs/Float64]  Accumulated position (degrees)

Subscription:
  /motor/cmd_pwm     [std_msgs/Float64]  Optional external command

ROS2 Parameters (ros2 run ... --ros-args -p <param>:=<val>):
  port          (str)   Serial port        [default: /dev/ttyUSB0]
  baud          (int)   Baudrate           [default: 115200]
  step_pwm      (float) Step amplitude     [default: 0.5 → 50% PWM]
  step_duration (float) Step duration      [default: 3.0 s]
  signal_type   (str)   step | prbs | ramp [default: step]
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

import serial
import threading
import time
import random


class MotorIdentificationNode(Node):

    def __init__(self):
        super().__init__('motor_identification')

        # ── Declare parameters ──────────────────────────────
        self.declare_parameter('port',          '/dev/ttyACM0')
        self.declare_parameter('baud',          115200)
        self.declare_parameter('step_pwm',      1.0)
        self.declare_parameter('step_duration', 10.0)
        self.declare_parameter('signal_type',   'step')  # step | prbs | ramp

        port          = self.get_parameter('port').value
        baud          = self.get_parameter('baud').value
        self.step_pwm = self.get_parameter('step_pwm').value
        self.step_dur = self.get_parameter('step_duration').value
        self.sig_type = self.get_parameter('signal_type').value

        # ── Publishers ─────────────────────────────────────
        self.pub_pwm   = self.create_publisher(Float64, '/motor/pwm_input',  10)
        self.pub_speed = self.create_publisher(Float64, '/motor/speed_rpm',  10)
        self.pub_pos   = self.create_publisher(Float64, '/motor/position',   10)

        # ── Subscriber (optional external command) ─────────
        self.create_subscription(Float64, '/motor/cmd_pwm',
                                 self._cmd_callback, 10)

        # ── Internal state variables ───────────────────────
        self._external_cmd  = None   # None = automatic mode
        self._current_pwm   = 0.0
        self._running       = True

        # ── Open serial port ───────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)          # Wait for ESP32 reset
            self.ser.flushInput()
            self.get_logger().info(f'Serial port opened: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open {port}: {e}')
            raise

        # ── Serial reading thread ──────────────────────────
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()

        # ── Signal generator thread ────────────────────────
        self._signal_thread = threading.Thread(target=self._signal_generator, daemon=True)
        self._signal_thread.start()

        self.get_logger().info(
            f'Node started | signal: {self.sig_type} | '
            f'Step PWM: {self.step_pwm:.2f} | duration: {self.step_dur}s'
        )

    # ─────────────────────────────────────────────────────────
    #  Serial reading thread
    # ─────────────────────────────────────────────────────────
    def _read_serial(self):
        while self._running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()

                if line.startswith('DATA:'):
                    parts = line[5:].split(',')
                    if len(parts) == 3:
                        pwm_fb  = float(parts[0])
                        rpm     = float(parts[1])
                        pos_deg = float(parts[2])

                        self.pub_pwm.publish(Float64(data=self._current_pwm))
                        self.pub_speed.publish(Float64(data=rpm))
                        self.pub_pos.publish(Float64(data=pos_deg))

                elif line == 'READY':
                    self.get_logger().info('ESP32 ready.')

            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'Serial read error: {e}')

    # ─────────────────────────────────────────────────────────
    #  Send PWM command to ESP32
    # ─────────────────────────────────────────────────────────
    def _send_pwm(self, pwm_norm: float):
        pwm_norm = max(-1.0, min(1.0, pwm_norm))
        self._current_pwm = pwm_norm
        pwm_int = int(pwm_norm * 255)
        cmd = f'CMD:{pwm_int}\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    # ─────────────────────────────────────────────────────────
    #  External command callback
    # ─────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Float64):
        self._external_cmd = msg.data
        self._send_pwm(msg.data)

    # ─────────────────────────────────────────────────────────
    #  Identification signal generator
    # ─────────────────────────────────────────────────────────
    def _signal_generator(self):
        """
        Generates the excitation signal for motor identification.

        step : Step input 0 → step_pwm → 0 ... (infinite cycle)
        prbs : Pseudo-random binary sequence (useful for ARX/BJ models)
        ramp : Ramp up/down between 0 and step_pwm
        """
        time.sleep(1.0)   # Wait for ESP32 to be ready

        while self._running:
            if self._external_cmd is not None:
                # In manual mode, the generator releases control
                time.sleep(0.1)
                continue

            if self.sig_type == 'step':
                self._run_step()
            elif self.sig_type == 'prbs':
                self._run_prbs()
            elif self.sig_type == 'ramp':
                self._run_ramp()
            else:
                self.get_logger().warn(f'Unknown signal_type: {self.sig_type}')
                time.sleep(1.0)

    # ── Step input signal ───────────────────────────────────
    def _run_step(self):
        self.get_logger().info('Applying positive step input...')
        self._send_pwm(self.step_pwm)
        time.sleep(self.step_dur)

        self.get_logger().info('Motor OFF (rest period)...')
        self._send_pwm(0.0)
        time.sleep(self.step_dur)

    # ── PRBS signal ─────────────────────────────────────────
    def _run_prbs(self):
        levels = [0.0, self.step_pwm, -self.step_pwm]
        pwm    = random.choice(levels)
        self._send_pwm(pwm)
        hold   = random.uniform(0.3, self.step_dur)
        time.sleep(hold)

    # ── Ramp signal ─────────────────────────────────────────
    def _run_ramp(self):
        steps = 50
        dt    = self.step_dur / steps

        # Ramp up
        for i in range(steps + 1):
            self._send_pwm(self.step_pwm * i / steps)
            time.sleep(dt)

        # Ramp down
        for i in range(steps, -1, -1):
            self._send_pwm(self.step_pwm * i / steps)
            time.sleep(dt)

    # ─────────────────────────────────────────────────────────
    #  Node destructor
    # ─────────────────────────────────────────────────────────
    def destroy_node(self):
        self._running = False
        try:
            self.ser.write(b'STOP\n')
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MotorIdentificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()