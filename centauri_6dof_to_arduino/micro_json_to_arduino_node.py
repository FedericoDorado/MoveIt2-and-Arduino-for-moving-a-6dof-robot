import rclpy
import ujson as json  # Importar ujson en lugar de json estándar
import serial
from sensor_msgs.msg import JointState
import time

serial_port = None

def joint_states_callback(msg, node):
    global serial_port

    positions = list(msg.position)

    start_sequence = b'@'
    json_positions = json.dumps(positions)  # Utilizar ujson para codificar las posiciones en JSON
    data_to_send = start_sequence + json_positions.encode() + b'\n'

    try:
        serial_port.write(data_to_send)
        node.get_logger().info("JSON positions sent: %s" % json_positions)
    except Exception as e:
        node.get_logger().error("Error sending JSON positions: %s" % str(e))

def main(args=None):
    global serial_port
    rclpy.init(args=args)
    node = rclpy.create_node('joint_states_to_serial_node')
    node.get_logger().info("Joint States to Serial Node Initialized")

    serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)

    subscription = node.create_subscription(
        JointState,
        'joint_states',
        lambda msg: joint_states_callback(msg, node),
        10
    )
    subscription

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
