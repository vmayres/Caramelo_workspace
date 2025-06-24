import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import serial
import json
import time

class CarameloHWInterfaceNode(Node):
    def __init__(self):
        super().__init__('caramelo_hw_interface_node')
        # Assina comandos de velocidade das rodas (esperado pelo mecanum_drive_controller)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mecanum_controller/commands',
            self.wheel_command_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=2.5)
        self.max_pwm = 1023
        self.min_pwm = 0
        self.zero_pwm = 512
        self.k_pwm = 300  # ajuste conforme necessário
        # Publisher para joint_states (valores simulados)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.last_time = self.get_clock().now()
        self.sim_position = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.sim_velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]

    def wheel_command_callback(self, msg):
        # Espera-se ordem: [vel_fl, vel_fr, vel_rl, vel_rr] (rad/s)
        if len(msg.data) != 4:
            self.get_logger().warn('Comando de roda deve ter 4 valores (FL, FR, RL, RR)')
            return
        pwm_fl = int(self.zero_pwm + self.k_pwm * msg.data[0])
        pwm_fr = int(self.zero_pwm + self.k_pwm * msg.data[1])
        pwm_rl = int(self.zero_pwm + self.k_pwm * msg.data[2])
        pwm_rr = int(self.zero_pwm + self.k_pwm * msg.data[3])
        pwm_fl = max(self.min_pwm, min(self.max_pwm, pwm_fl))
        pwm_fr = max(self.min_pwm, min(self.max_pwm, pwm_fr))
        pwm_rl = max(self.min_pwm, min(self.max_pwm, pwm_rl))
        pwm_rr = max(self.min_pwm, min(self.max_pwm, pwm_rr))
        pwm_dict = {
            'pwm_fl': pwm_fl,
            'pwm_fr': pwm_fr,
            'pwm_rl': pwm_rl,
            'pwm_rr': pwm_rr
        }
        try:
            self.serial_port.write((json.dumps(pwm_dict)+'\n').encode('utf-8'))
            self.get_logger().info(f'Enviando PWM: {pwm_dict}')
        except Exception as e:
            self.get_logger().error(f'Erro ao enviar PWM: {e}')
        # Atualiza valores simulados para joint_states
        self.sim_velocity = list(msg.data)
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        for i in range(4):
            self.sim_position[i] += self.sim_velocity[i] * dt
        self.last_time = now
        self.publish_simulated_joint_states()

    def publish_simulated_joint_states(self):
        # Publica valores simulados de joint_states (apenas para visualização, NÃO são valores reais de encoder!)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.sim_position
        js.velocity = self.sim_velocity
        js.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = CarameloHWInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
