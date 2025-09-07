#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistToTwistStampedConverter(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_converter')
        
        # Subscriber para Twist simples (do teleop)
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            10
        )
        
        # Publisher para TwistStamped no tópico de referência do controlador
        # O mecanum_drive_controller (não em chained mode) escuta em <nome_do_controlador>/reference
        self.twist_stamped_publisher = self.create_publisher(
            TwistStamped,
            '/mecanum_controller/reference',
            10
        )
        
        self.get_logger().info('   Conversor Twist → TwistStamped iniciado!')
        self.get_logger().info('   Recebe: /cmd_vel (Twist)')
        self.get_logger().info('   Publica: /mecanum_controller/reference (TwistStamped)')

    def twist_callback(self, msg):
        """Converte Twist para TwistStamped no formato exato esperado"""
        # Criar mensagem TwistStamped
        twist_stamped = TwistStamped()
        
        # Timestamp atual (recomendado) e frame vazio (controlador ignora frame)
        now = self.get_clock().now().to_msg()
        twist_stamped.header.stamp = now
        twist_stamped.header.frame_id = ''
        
        # Copiar dados do Twist
        twist_stamped.twist = msg
        
        # Publicar
        self.twist_stamped_publisher.publish(twist_stamped)
        
        # Log apenas para debug (comentado para não fazer spam)
        # self.get_logger().debug(f'Convertendo: linear=({msg.linear.x:.2f}, {msg.linear.y:.2f}) angular={msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    node = TwistToTwistStampedConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destruir nó antes de shutdown
        try:
            node.destroy_node()
        except:
            pass
        
        # Shutdown apenas se ainda não foi feito
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
