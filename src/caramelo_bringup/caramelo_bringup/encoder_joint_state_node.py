import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import json
import time

class EncoderJointStateNode(Node):
    def __init__(self):
        super().__init__('encoder_joint_state_node')
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=0.1)
        self.last_counts = [0, 0, 0, 0]
        self.last_time = time.time()
        self.wheel_radius = 0.05  # metros
        self.pulses_per_rev = 1024 * 2 * 28  # Exemplo: encoder quadratura * redução
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        self.position = [0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.02, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                return
            data = json.loads(line)
            counts = [data.get('enc_fl', 0), data.get('enc_fr', 0), data.get('enc_rl', 0), data.get('enc_rr', 0)]
            now = time.time()
            dt = now - self.last_time
            for i in range(4):
                delta = counts[i] - self.last_counts[i]
                # posição em radianos
                self.position[i] = counts[i] * (2 * 3.14159265359) / self.pulses_per_rev
                # velocidade em rad/s
                self.velocity[i] = (delta * (2 * 3.14159265359) / self.pulses_per_rev) / dt if dt > 0 else 0.0
            self.last_counts = counts
            self.last_time = now
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = self.position
            js.velocity = self.velocity
            js.effort = [0.0, 0.0, 0.0, 0.0]
            self.joint_state_pub.publish(js)
        except Exception as e:
            self.get_logger().warn(f'Erro ao ler/parsing serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderJointStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
