import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import serial
import json
import time
import math

class EncoderJointStateNode(Node):
    def __init__(self):
        super().__init__('encoder_joint_state_node')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Parâmetros do robô
        self.wheel_radius = 0.05  # metros
        self.wheel_base = 0.47    # Distância entre eixos dianteiro/traseiro
        self.wheel_separation = 0.315  # Distância entre rodas esquerda/direita (31,5 cm)
        
        # Parâmetros dos encoders
        self.pulses_per_rev = 114688  # Pulsos por revolução do motor corrigido
        self.meters_per_pulse = (2 * math.pi * self.wheel_radius) / self.pulses_per_rev  # Distância linear por pulso (metros)
        
        # Estado dos joints (nomes correspondentes ao URDF)
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        self.position = [0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        
        # Estado da odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_counts = [0, 0, 0, 0]
        self.last_raw_counts = [0, 0, 0, 0]  # Para mostrar valores originais no display
        self.last_time = self.get_clock().now()
        self.counts_initialized = False
        
        # Configuração da serial com tentativas de reconexão
        self.setup_serial_connection()
        
        # Timer para leitura (50Hz)
        self.timer = self.create_timer(0.02, self.read_and_publish)
        
        # Timer para display estático (a cada 0.1 segundos = 10Hz)
        self.create_timer(0.1, self.display_static_info)
        
        self.get_logger().info('🔄 CARAMELO ENCODERS - Iniciando...')
        self.get_logger().info('Nó de encoders e odometria iniciado!')
        self.get_logger().info(f'Parâmetros: wheel_radius={self.wheel_radius}m, wheel_base={self.wheel_base}m, wheel_separation={self.wheel_separation}m')
        self.get_logger().info(f'Pulsos por revolução: {self.pulses_per_rev}')
        self.get_logger().info(f'Distância por pulso: {self.meters_per_pulse:.9f}m (calculado: 2π×{self.wheel_radius}m / {self.pulses_per_rev})')
        self.get_logger().info('Display estático será iniciado...')

    def setup_serial_connection(self):
        """Configura conexão serial com a ESP32 dos encoders"""
        # Porta exclusiva para encoders: USB1
        encoder_port = '/dev/ttyUSB1'
        
        try:
            # Fechar qualquer conexão anterior
            if hasattr(self, 'serial_port') and self.serial_port.is_open:
                self.serial_port.close()
                time.sleep(0.5)
            
            # Conectar exclusivamente na porta USB1 para encoders
            self.serial_port = serial.Serial(encoder_port, baudrate=115200, timeout=0.1)
            self.get_logger().info(f"📊 ESP32 dos encoders conectada em {encoder_port}")
            
            # Hard Reset da ESP32 para limpar memória
            self.get_logger().info("🔄 Fazendo hard reset da ESP32 dos encoders...")
            self.serial_port.dtr = False
            time.sleep(0.1)
            self.serial_port.dtr = True
            time.sleep(2.0)  # Aguarda ESP32 reiniciar completamente
            
            # Limpar buffers após reset
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            
            self.get_logger().info("✅ ESP32 dos encoders resetada e pronta!")
            self.connection_established = True
            
        except Exception as e:
            self.get_logger().error(f"❌ Falha ao conectar ESP32 dos encoders em {encoder_port}: {e}")
            self.get_logger().warn("🔍 Verifique se a ESP32 dos encoders está conectada em /dev/ttyUSB1")
            self.get_logger().warn("🔍 Verifique as permissões: sudo chmod 777 /dev/ttyUSB1")
            self.connection_established = False

    def display_static_info(self):
        """Display estático com limpeza de tela - valores atualizados no mesmo local"""
        import os
        import sys
        
        # Método mais robusto para limpar tela
        if os.name == 'nt':  # Windows
            os.system('cls')
        else:  # Linux/Mac
            os.system('clear')
        
        # Flush para garantir limpeza
        sys.stdout.flush()
        
        # Banner fixo
        print("\033[96m╔══════════════════════════════════════════════════════════════╗")
        print("║                   🔄 CARAMELO ENCODERS 🔄                    ║")
        print("║               Odometria em Tempo Real                        ║")
        print("╚══════════════════════════════════════════════════════════════╝\033[0m")
        print()
        
        # Status da conexão
        status_color = "\033[92m" if self.connection_established else "\033[91m"
        status_text = "CONECTADO" if self.connection_established else "DESCONECTADO"
        port_info = ""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            port_info = f" ({self.serial_port.port})"
        print(f"{status_color}📡 Status: {status_text}{port_info}\033[0m")
        print()
        
        # Odometria
        print("\033[95m📍 POSIÇÃO GLOBAL:\033[0m")
        print(f"   X: {self.x:8.3f} m    Y: {self.y:8.3f} m    θ: {math.degrees(self.theta):7.1f}°")
        print()
        
        # Velocidades das rodas
        print("\033[93m⚙️  VELOCIDADES DAS RODAS (rad/s):\033[0m")
        print(f"   FL: {self.velocity[0]:7.3f}    FR: {self.velocity[1]:7.3f}")
        print(f"   RL: {self.velocity[2]:7.3f}    RR: {self.velocity[3]:7.3f}")
        print()
        
        # Contadores dos encoders (corrigidos com inversão)
        print("\033[94m🔢 CONTADORES DOS ENCODERS (corrigidos):\033[0m")
        print(f"   FL: {self.last_counts[0]:8d}    FR: {self.last_counts[1]:8d} (inv)")
        print(f"   RL: {self.last_counts[2]:8d}    RR: {self.last_counts[3]:8d} (inv)")
        print()
        
        # Rodapé com instruções
        print("\033[90m" + "─" * 62 + "\033[0m")
        print("\033[90mCtrl+C para parar | Atualizando a 10Hz (0.1s)\033[0m")
        
        # Flush para garantir que a saída seja imediata
        import sys
        sys.stdout.flush()

    def read_and_publish(self):
        """Lê dados dos encoders e calcula odometria"""
        if not hasattr(self, 'serial_port') or not self.serial_port.is_open:
            return
            
        try:
            # Ler linha da serial
            line = self.serial_port.readline().decode('utf-8').strip()
            if not line:
                return
                
            # Parse do JSON
            data = json.loads(line)
            raw_counts = [
                data.get('enc_fl', 0), 
                data.get('enc_fr', 0), 
                data.get('enc_rl', 0), 
                data.get('enc_rr', 0)
            ]
            
            # CORREÇÃO: Inverter sinal das rodas direitas (FR e RR) 
            # porque os motores são instalados invertidos
            counts = [
                raw_counts[0],   # FL: normal
                -raw_counts[1],  # FR: invertido
                raw_counts[2],   # RL: normal  
                -raw_counts[3]   # RR: invertido
            ]
            
            # Timestamp atual
            now = self.get_clock().now()
            
            # Inicialização na primeira leitura
            if not self.counts_initialized:
                self.last_counts = counts[:]
                self.last_raw_counts = raw_counts[:]
                self.last_time = now
                self.counts_initialized = True
                self.get_logger().info(f'Encoders inicializados: FL={counts[0]}, FR={counts[1]} (inv), RL={counts[2]}, RR={counts[3]} (inv)')
                self.get_logger().info(f'Valores RAW: FL={raw_counts[0]}, FR={raw_counts[1]}, RL={raw_counts[2]}, RR={raw_counts[3]}')
                return
            
            # Calcular delta de tempo
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            
            # Calcular deltas dos counts e velocidades das rodas
            wheel_velocities = []
            for i in range(4):
                delta_counts = counts[i] - self.last_counts[i]
                
                # Posição em radianos (absoluta)
                self.position[i] = counts[i] * (2 * math.pi) / self.pulses_per_rev
                
                # Velocidade em rad/s
                self.velocity[i] = (delta_counts * (2 * math.pi) / self.pulses_per_rev) / dt
                
                # Velocidade linear da roda usando a distância real por pulso (m/s)
                wheel_velocities.append(delta_counts * self.meters_per_pulse / dt)
            
            # Cinemática inversa para mecanum drive
            # v_x = (v_fl + v_fr + v_rl + v_rr) / 4
            # v_y = (-v_fl + v_fr + v_rl - v_rr) / 4  
            # w_z = (-v_fl + v_fr - v_rl + v_rr) / (4 * (wheel_base/2 + wheel_separation/2))
            
            v_x = (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3]) / 4.0
            v_y = (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3]) / 4.0
            
            # Distância do centro às rodas (para cálculo da velocidade angular)
            l = (self.wheel_base + self.wheel_separation) / 2.0
            w_z = (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3]) / (4.0 * l)
            
            # Integração da odometria
            # Transformar velocidades do frame do robô para o frame global
            delta_x = (v_x * math.cos(self.theta) - v_y * math.sin(self.theta)) * dt
            delta_y = (v_x * math.sin(self.theta) + v_y * math.cos(self.theta)) * dt
            delta_theta = w_z * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
            
            # Normalizar ângulo
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Publicar JointState
            self.publish_joint_states(now)
            
            # Publicar Odometria
            self.publish_odometry(now, v_x, v_y, w_z)
            
            # Publicar TF
            self.publish_tf(now)
            
            # Atualizar estados para próxima iteração
            self.last_counts = counts[:]
            self.last_raw_counts = raw_counts[:]
            self.last_time = now
            
        except json.JSONDecodeError as e:
            self.get_logger().debug(f'Erro de parsing JSON: {e}')
        except Exception as e:
            self.get_logger().warn(f'Erro ao ler/processar dados dos encoders: {e}')
            # Tentar reconectar em caso de erro
            self.setup_serial_connection()

    def publish_joint_states(self, timestamp):
        """Publica estados dos joints das rodas"""
        js = JointState()
        js.header.stamp = timestamp.to_msg()
        js.name = self.joint_names
        js.position = self.position[:]
        js.velocity = self.velocity[:]
        js.effort = [0.0, 0.0, 0.0, 0.0]
        self.joint_state_pub.publish(js)

    def publish_odometry(self, timestamp, v_x, v_y, w_z):
        """Publica mensagem de odometria"""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientação (quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocidades
        odom.twist.twist.linear.x = v_x
        odom.twist.twist.linear.y = v_y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w_z
        
        # Covariâncias (valores padrão)
        odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        
        self.odom_pub.publish(odom)

    def publish_tf(self, timestamp):
        """Publica transformação odom -> base_link"""
        tf = TransformStamped()
        tf.header.stamp = timestamp.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        
        # Posição
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        
        # Orientação
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(self.theta / 2.0)
        tf.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf)

    def __del__(self):
        """Fechar conexão serial ao finalizar"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

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
