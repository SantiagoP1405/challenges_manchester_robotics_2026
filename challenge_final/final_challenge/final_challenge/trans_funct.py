#!/usr/bin/env python3
"""
motor_identification_node.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Nodo ROS2 para identificación de motor DC a lazo abierto.

Publica (graficables en rqt_plot):
  /motor/pwm_input   [std_msgs/Float64]  Señal de entrada PWM (-1 a 1)
  /motor/speed_rpm   [std_msgs/Float64]  Velocidad medida (RPM)
  /motor/position    [std_msgs/Float64]  Posición acumulada (grados)

Suscripción:
  /motor/cmd_pwm     [std_msgs/Float64]  Comando externo (opcional)

Parámetros ROS2 (ros2 run ... --ros-args -p <param>:=<val>):
  port          (str)   Puerto serial      [default: /dev/ttyUSB0]
  baud          (int)   Baudrate           [default: 115200]
  step_pwm      (float) Amplitud escalón   [default: 0.5 → 50% PWM]
  step_duration (float) Duración escalón   [default: 3.0 s]
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

        # ── Declarar parámetros ──────────────────────────────
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

        # ── Publicadores ─────────────────────────────────────
        self.pub_pwm   = self.create_publisher(Float64, '/motor/pwm_input',  10)
        self.pub_speed = self.create_publisher(Float64, '/motor/speed_rpm',  10)
        self.pub_pos   = self.create_publisher(Float64, '/motor/position',   10)

        # ── Suscriptor (comando externo opcional) ─────────────
        self.create_subscription(Float64, '/motor/cmd_pwm',
                                 self._cmd_callback, 10)

        # ── Estado interno ────────────────────────────────────
        self._external_cmd  = None   # None = modo automático
        self._current_pwm   = 0.0
        self._running       = True

        # ── Abrir puerto serial ───────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)          # Esperar reset ESP32
            self.ser.flushInput()
            self.get_logger().info(f'Puerto serial abierto: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo abrir {port}: {e}')
            raise

        # ── Hilo de lectura serial ────────────────────────────
        self._read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._read_thread.start()

        # ── Hilo generador de señal ───────────────────────────
        self._signal_thread = threading.Thread(target=self._signal_generator, daemon=True)
        self._signal_thread.start()

        self.get_logger().info(
            f'Nodo iniciado | señal: {self.sig_type} | '
            f'PWM step: {self.step_pwm:.2f} | duración: {self.step_dur}s'
        )

    # ─────────────────────────────────────────────────────────
    #  Lectura Serial (hilo separado)
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
                    self.get_logger().info('ESP32 lista.')

            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'Error lectura serial: {e}')

    # ─────────────────────────────────────────────────────────
    #  Enviar comando PWM a ESP32
    # ─────────────────────────────────────────────────────────
    def _send_pwm(self, pwm_norm: float):
        pwm_norm = max(-1.0, min(1.0, pwm_norm))
        self._current_pwm = pwm_norm
        pwm_int = int(pwm_norm * 255)
        cmd = f'CMD:{pwm_int}\n'
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Error escritura serial: {e}')

    # ─────────────────────────────────────────────────────────
    #  Callback comando externo
    # ─────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Float64):
        self._external_cmd = msg.data
        self._send_pwm(msg.data)

    # ─────────────────────────────────────────────────────────
    #  Generador de señal de identificación
    # ─────────────────────────────────────────────────────────
    def _signal_generator(self):
        """
        Genera la señal de excitación al motor.

        step : Escalón 0 → step_pwm → 0 ... (ciclo infinito)
        prbs : Secuencia binaria pseudo-aleatoria  (útil para ARX/BJ)
        ramp : Rampa sube/baja entre 0 y step_pwm
        """
        time.sleep(1.0)   # Esperar que ESP32 esté lista

        while self._running:
            if self._external_cmd is not None:
                # En modo manual, el generador cede el control
                time.sleep(0.1)
                continue

            if self.sig_type == 'step':
                self._run_step()
            elif self.sig_type == 'prbs':
                self._run_prbs()
            elif self.sig_type == 'ramp':
                self._run_ramp()
            else:
                self.get_logger().warn(f'signal_type desconocido: {self.sig_type}')
                time.sleep(1.0)

    # ── Señal escalón ─────────────────────────────────────────
    def _run_step(self):
        self.get_logger().info('Aplicando escalón positivo...')
        self._send_pwm(self.step_pwm)
        time.sleep(self.step_dur)

        self.get_logger().info('Motor OFF (reposo)...')
        self._send_pwm(0.0)
        time.sleep(self.step_dur)

    # ── Señal PRBS ────────────────────────────────────────────
    def _run_prbs(self):
        levels = [0.0, self.step_pwm, -self.step_pwm]
        pwm    = random.choice(levels)
        self._send_pwm(pwm)
        hold   = random.uniform(0.3, self.step_dur)
        time.sleep(hold)

    # ── Señal rampa ───────────────────────────────────────────
    def _run_ramp(self):
        steps = 50
        dt    = self.step_dur / steps

        # Subida
        for i in range(steps + 1):
            self._send_pwm(self.step_pwm * i / steps)
            time.sleep(dt)
        # Bajada
        for i in range(steps, -1, -1):
            self._send_pwm(self.step_pwm * i / steps)
            time.sleep(dt)

    # ─────────────────────────────────────────────────────────
    #  Destructor
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
        node.get_logger().info('Deteniendo nodo...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()