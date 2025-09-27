#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')

        # Parameters
        self.declare_parameter('input_twist_topic', 'robot_controller/cmd_vel_unstamped')
        self.declare_parameter('output_twist_stamped_topic', '/mecanum_controller/reference')
        self.declare_parameter('frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_twist_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_twist_stamped_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Subscriber para Twist "unstamped" vindo do twist_mux
        self.twist_subscriber = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10,
        )

        # Publisher para TwistStamped no tópico do controlador mecanum
        self.twist_stamped_publisher = self.create_publisher(
            TwistStamped,
            output_topic,
            10,
        )

    def twist_callback(self, msg: Twist):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id
        twist_stamped.twist = msg
        self.twist_stamped_publisher.publish(twist_stamped)

def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()