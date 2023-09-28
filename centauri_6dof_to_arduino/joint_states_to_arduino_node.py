#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math

class JointStatesSender(Node):
    def __init__(self):
        super().__init__('joint_states_sender')
        self.serial_port = serial.Serial('/dev/ttyACM0', baudrate=9600)
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.tolerance = 0.001  # Ajusta la tolerancia según tus necesidades
        self.last_positions = None  # Almacena las últimas posiciones recibidas
        self.sent_stable_positions = False  # Bandera para verificar si se enviaron las posiciones estables

    def joint_state_callback(self, msg):
        joint_positions_radians = msg.position

        # Convertir radianes a grados
        joint_positions_degrees = [math.degrees(pos) for pos in joint_positions_radians]

        # Multiplicar las posiciones en posiciones específicas por las constantes
        multipliers = [88.889, 45.556, 200, 8.889, -40, 8.334]

        # Orden joint1, joint2, joint3, joint4, joint5

        for idx, multiplier in zip([0, 1, 2, 3, 4, 5], multipliers):
            joint_positions_degrees[idx] *= multiplier

        # Si es la primera vez, establecer las posiciones recibidas como estables
        if self.last_positions is None:
            self.last_positions = joint_positions_degrees

        # Si las posiciones están cambiando, reiniciar la bandera de envío de posiciones estables
        if not self.positions_equal(joint_positions_degrees, self.last_positions):
            self.sent_stable_positions = False

        # Si las posiciones son estables, no se han enviado aún y no están cambiando, enviar los datos
        if self.positions_equal(joint_positions_degrees, self.last_positions) and not self.sent_stable_positions:
            self.send_data(joint_positions_degrees)
            self.sent_stable_positions = True

        # Actualizar las últimas posiciones recibidas
        self.last_positions = joint_positions_degrees

    def send_data(self, positions):
        joint_positions_str = [str(pos) for pos in positions]
        message = ','.join(joint_positions_str) + ','+'0' + ',' + '\n'

        self.serial_port.write(message.encode())
        self.get_logger().info(f'Sent: {message}')

    def positions_equal(self, positions1, positions2):
        for pos1, pos2 in zip(positions1, positions2):
            if abs(pos1 - pos2) > self.tolerance:
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    joint_states_sender = JointStatesSender()
    rclpy.spin(joint_states_sender)
    joint_states_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
