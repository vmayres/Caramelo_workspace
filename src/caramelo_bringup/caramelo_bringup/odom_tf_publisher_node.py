#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class OdomTFPublisherNode(Node):
    """
    Nó responsável por publicar a transformação TF entre os frames 'odom' e 'base_footprint'.
    O frame 'odom' é o frame de referência global para odometria, sempre posicionado em (0,0,0).
    O frame 'base_footprint' é o frame base do robô no chão.
    
    Este nó é configurável via parâmetros ROS2 para permitir ajustes de posição inicial
    e taxa de publicação.
    """
    
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Declarar parâmetros com valores padrão
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.0)
        self.declare_parameter('initial_qx', 0.0)
        self.declare_parameter('initial_qy', 0.0)
        self.declare_parameter('initial_qz', 0.0)
        self.declare_parameter('initial_qw', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        
        # Obter valores dos parâmetros
        self.initial_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.initial_z = self.get_parameter('initial_z').get_parameter_value().double_value
        self.initial_qx = self.get_parameter('initial_qx').get_parameter_value().double_value
        self.initial_qy = self.get_parameter('initial_qy').get_parameter_value().double_value
        self.initial_qz = self.get_parameter('initial_qz').get_parameter_value().double_value
        self.initial_qw = self.get_parameter('initial_qw').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        
        # Broadcaster para publicar transformações TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer para publicar a transformação periodicamente
        timer_period = 1.0 / self.publish_rate  # Converter Hz para período em segundos
        self.timer = self.create_timer(timer_period, self.publish_odom_tf)
        
        self.get_logger().info(f'Odom TF Publisher Node iniciado')
        self.get_logger().info(f'Publicando TF: {self.parent_frame} -> {self.child_frame}')
        self.get_logger().info(f'Posição inicial: ({self.initial_x}, {self.initial_y}, {self.initial_z})')
        self.get_logger().info(f'Taxa de publicação: {self.publish_rate} Hz')
    
    def publish_odom_tf(self):
        """
        Publica a transformação estática do frame 'odom' para 'base_footprint'.
        Por enquanto, mantém o robô na posição inicial configurada.
        """
        try:
            # Criar mensagem de transformação
            transform = TransformStamped()
            
            # Header
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.parent_frame  # Frame pai
            transform.child_frame_id = self.child_frame    # Frame filho
            
            # Posição (configurável via parâmetros)
            transform.transform.translation.x = self.initial_x
            transform.transform.translation.y = self.initial_y
            transform.transform.translation.z = self.initial_z
            
            # Orientação (quaternion configurável)
            transform.transform.rotation.x = self.initial_qx
            transform.transform.rotation.y = self.initial_qy
            transform.transform.rotation.z = self.initial_qz
            transform.transform.rotation.w = self.initial_qw
            
            # Publicar a transformação
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Erro ao publicar transformação TF: {str(e)}')


def main(args=None):
    """Função principal do nó"""
    rclpy.init(args=args)
    
    try:
        node = OdomTFPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Erro no nó Odom TF Publisher: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
