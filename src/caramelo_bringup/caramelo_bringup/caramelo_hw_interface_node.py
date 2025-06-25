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
        
        # Configuração da serial com reset automático
        self.setup_serial_connection()
        
        self.max_pwm = 1023
        self.min_pwm = 0
        self.zero_pwm = 512
        
        # Parâmetros do motor para cálculo do k_pwm
        # Baseado em velocidade máxima de 1.5 m/s:
        # vel_angular_roda = 1.5/0.05 = 30 rad/s = 286.5 RPM
        # rpm_motor = 286.5 * 28 = 8022 RPM
        self.max_motor_rpm = 8022.0  # RPM máximo para 1.5 m/s
        self.gear_ratio = 28.0       # Redução do motor (1:28)
        self.wheel_radius = 0.05     # Raio da roda em metros
        
        # Cálculo automático do k_pwm baseado nas características do motor
        self.k_pwm = self.calculate_k_pwm()
        
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
        
        # Timer para verificar conexão (apenas quando necessário)
        self.create_timer(30.0, self.check_connection)  # Verificar a cada 30 segundos
        self.last_successful_send = time.time()
        self.connection_established = False
        
        # Timer para display estático (a cada 0.5 segundos)
        self.create_timer(0.5, self.display_static_info)
        
        self.get_logger().info('⚡ CARAMELO PWM INTERFACE - Iniciando...')
        self.get_logger().info('Hardware Interface para ESP32 PWM iniciado!')
        self.get_logger().info(f'k_pwm calculado: {self.k_pwm:.2f}')
        self.get_logger().info('Display estático será iniciado...')
        self.last_cmd_vel = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        self.last_wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.last_pwm_values = [512, 512, 512, 512]  # FL, FR, RL, RR

    def calculate_k_pwm(self):
        """
        Calcula o fator k_pwm baseado nas características do motor e roda.
        
        Conversão: rad/s -> PWM
        Hardware: 512=parado, 0=frente_max, 1023=trás_max
        """
        # Velocidade angular máxima da roda (rad/s)
        max_wheel_rpm = self.max_motor_rpm / self.gear_ratio
        max_wheel_rad_s = (max_wheel_rpm * 2 * 3.14159265359) / 60.0
        
        # PWM disponível para velocidade (do centro até o máximo em qualquer direção)
        # De 512 até 0 (frente) = 512 unidades PWM
        # De 512 até 1023 (trás) = 511 unidades PWM
        # Usar o menor range para garantir simetria
        pwm_range = min(self.zero_pwm - self.min_pwm, self.max_pwm - self.zero_pwm)  # min(512, 511) = 511
        
        # k_pwm = PWM_range / velocidade_angular_maxima
        k_pwm = pwm_range / max_wheel_rad_s
        
        self.get_logger().info(f'Parâmetros do motor:')
        self.get_logger().info(f'  Max motor RPM: {self.max_motor_rpm}')
        self.get_logger().info(f'  Gear ratio: {self.gear_ratio}')
        self.get_logger().info(f'  Max wheel RPM: {max_wheel_rpm:.2f}')
        self.get_logger().info(f'  Max wheel rad/s: {max_wheel_rad_s:.2f}')
        self.get_logger().info(f'  PWM range: {pwm_range} (512=parado, 0=frente_max, 1023=trás_max)')
        self.get_logger().info(f'  k_pwm calculado: {k_pwm:.2f}')
        
        return k_pwm

    def setup_serial_connection(self):
        """Configura e testa a conexão serial com reset automático"""
        # Lista de portas para tentar (PWM geralmente em porta diferente dos encoders)
        pwm_ports = ['/dev/ttyUSB0', '/dev/ttyUSB2', '/dev/ttyUSB3']
        
        for port in pwm_ports:
            try:
                # Fechar qualquer conexão anterior
                if hasattr(self, 'serial_port') and self.serial_port.is_open:
                    self.serial_port.close()
                    time.sleep(0.5)
                
                # Tentar abrir nova conexão na porta
                self.serial_port = serial.Serial(port, baudrate=9600, timeout=2.5)
                
                # Reset automático da ESP32 (toggle DTR)
                self.get_logger().info(f"🔧 ESP32 PWM conectada em {port}")
                self.get_logger().info("🔄 Resetando ESP32 PWM...")
                self.serial_port.dtr = False
                time.sleep(0.1)
                self.serial_port.dtr = True
                time.sleep(2.0)  # Aguarda ESP32 reiniciar
                
                # Limpar buffer
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                
                # Teste de comunicação
                test_msg = {"pwm_fl": 512, "pwm_fr": 512, "pwm_rl": 512, "pwm_rr": 512}
                self.serial_port.write((json.dumps(test_msg)+'\n').encode('utf-8'))
                self.serial_port.flush()
                
                self.get_logger().info(f"✅ ESP32 PWM resetada e conexão estabelecida em {port}!")
                self.connection_established = True
                return  # Sucesso, sair da função
                
            except Exception as e:
                self.get_logger().warn(f"❌ Falha ao conectar em {port}: {e}")
                continue
        
        # Se chegou aqui, nenhuma porta funcionou
        self.get_logger().error("❌ Erro: Não foi possível conectar a ESP32 PWM em nenhuma porta")
        self.get_logger().warn("🔍 Verifique se a ESP32 PWM está conectada")
        self.get_logger().warn("🔍 Portas tentadas: /dev/ttyUSB0, /dev/ttyUSB2, /dev/ttyUSB3")
        self.connection_established = False

    def check_connection(self):
        """Verifica se a comunicação está funcionando"""
        # Só verificar se não houve atividade recente quando há comandos sendo enviados
        time_since_last_send = time.time() - self.last_successful_send
        
        # Se a conexão foi estabelecida mas não há atividade há muito tempo (só depois de 60s)
        if self.connection_established and time_since_last_send > 60.0:
            self.get_logger().warn('Sem comunicação há mais de 60 segundos. Verificando conexão...')
            try:
                # Tentar enviar um comando de teste
                if hasattr(self, 'serial_port') and self.serial_port.is_open:
                    test_msg = {"pwm_fl": 512, "pwm_fr": 512, "pwm_rl": 512, "pwm_rr": 512}
                    self.serial_port.write((json.dumps(test_msg)+'\n').encode('utf-8'))
                    self.serial_port.flush()
                    self.last_successful_send = time.time()
                    self.get_logger().info('Teste de conexão enviado com sucesso!')
                else:
                    self.setup_serial_connection()
            except Exception as e:
                self.get_logger().error(f'Falha no teste de conexão: {e}')
                self.setup_serial_connection()

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
        print("║                    ⚡ CARAMELO PWM INTERFACE ⚡                ║")
        print("║               Hardware Interface em Tempo Real               ║")
        print("╚══════════════════════════════════════════════════════════════╝\033[0m")
        print()
        
        # Status da conexão
        status_color = "\033[92m" if self.connection_established else "\033[91m"
        status_text = "CONECTADO" if self.connection_established else "DESCONECTADO"
        port_info = ""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            port_info = f" ({self.serial_port.port})"
        print(f"{status_color}🔌 Status: {status_text}{port_info}\033[0m")
        print()
        
        # Comando de velocidade aproximado
        print("\033[95m🎯 COMANDO DE VELOCIDADE (aproximado):\033[0m")
        print(f"   Linear X: {self.last_cmd_vel['linear_x']:7.3f} m/s")
        print(f"   Linear Y: {self.last_cmd_vel['linear_y']:7.3f} m/s") 
        print(f"   Angular Z: {self.last_cmd_vel['angular_z']:6.3f} rad/s")
        print()
        
        # Velocidades das rodas
        print("\033[93m⚙️  VELOCIDADES DAS RODAS (rad/s):\033[0m")
        print(f"   FL: {self.last_wheel_velocities[0]:7.3f}    FR: {self.last_wheel_velocities[1]:7.3f}")
        print(f"   RL: {self.last_wheel_velocities[2]:7.3f}    RR: {self.last_wheel_velocities[3]:7.3f}")
        print()
        
        # Valores PWM com direção
        print("\033[94m🔧 VALORES PWM (512=parado, <512=frente, >512=trás):\033[0m")
        
        def get_direction_simple(pwm):
            if pwm < 500:
                return "FRENTE"
            elif pwm > 520:
                return "TRÁS"
            else:
                return "PARADO"
        
        print(f"   FL: {self.last_pwm_values[0]:4d} ({get_direction_simple(self.last_pwm_values[0])})    FR: {self.last_pwm_values[1]:4d} ({get_direction_simple(self.last_pwm_values[1])})")
        print(f"   RL: {self.last_pwm_values[2]:4d} ({get_direction_simple(self.last_pwm_values[2])})    RR: {self.last_pwm_values[3]:4d} ({get_direction_simple(self.last_pwm_values[3])})")
        print()
        
        # Rodapé com instruções
        print("\033[90m" + "─" * 62 + "\033[0m")
        print("\033[90mCtrl+C para parar | Atualizando a cada 0.5s\033[0m")
        
        # Flush para garantir que a saída seja imediata
        import sys
        sys.stdout.flush()

    def wheel_command_callback(self, msg):
        # Espera-se ordem: [vel_fl, vel_fr, vel_rl, vel_rr] (rad/s)
        if len(msg.data) != 4:
            self.get_logger().warn('Comando de roda deve ter 4 valores (FL, FR, RL, RR)')
            return
        
        # Armazenar velocidades das rodas para logs informativos
        self.last_wheel_velocities = list(msg.data)
        
        # Conversão para cmd_vel (aproximação para logs) - apenas para mostrar no status
        # Para mecanum: v_x = (v_fl + v_fr + v_rl + v_rr) / 4
        # v_y = (-v_fl + v_fr + v_rl - v_rr) / 4
        # w_z = (-v_fl + v_fr - v_rl + v_rr) / (4 * (lx + ly))
        # Para simplificar os logs, vamos usar aproximações
        wheel_radius = self.wheel_radius
        v_x_approx = (msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3]) * wheel_radius / 4
        v_y_approx = (-msg.data[0] + msg.data[1] + msg.data[2] - msg.data[3]) * wheel_radius / 4
        w_z_approx = (-msg.data[0] + msg.data[1] - msg.data[2] + msg.data[3]) * wheel_radius / (4 * 0.35)  # Estimativa baseada na distância entre rodas
        
        self.last_cmd_vel = {
            'linear_x': v_x_approx,
            'linear_y': v_y_approx,
            'angular_z': w_z_approx
        }
        
        # Conversão: velocidade positiva = PWM menor que 512 (direção frente)
        # velocidade negativa = PWM maior que 512 (direção trás)
        pwm_fl = int(self.zero_pwm - self.k_pwm * msg.data[0])
        pwm_fr = int(self.zero_pwm - self.k_pwm * msg.data[1])
        pwm_rl = int(self.zero_pwm - self.k_pwm * msg.data[2])
        pwm_rr = int(self.zero_pwm - self.k_pwm * msg.data[3])
        
        # Limitar valores PWM
        pwm_fl = max(self.min_pwm, min(self.max_pwm, pwm_fl))
        pwm_fr = max(self.min_pwm, min(self.max_pwm, pwm_fr))
        pwm_rl = max(self.min_pwm, min(self.max_pwm, pwm_rl))
        pwm_rr = max(self.min_pwm, min(self.max_pwm, pwm_rr))
        
        # Armazenar valores PWM para logs informativos
        self.last_pwm_values = [pwm_fl, pwm_fr, pwm_rl, pwm_rr]
        
        pwm_dict = {
            'pwm_fl': pwm_fl,
            'pwm_fr': pwm_fr,
            'pwm_rl': pwm_rl,
            'pwm_rr': pwm_rr
        }
        
        try:
            if self.serial_port.is_open:
                self.serial_port.write((json.dumps(pwm_dict)+'\n').encode('utf-8'))
                self.serial_port.flush()
                self.last_successful_send = time.time()
                
                # Armazenar valores PWM para o display
                self.last_pwm_values = [pwm_fl, pwm_fr, pwm_rl, pwm_rr]
            else:
                self.get_logger().error("❌ Porta serial não está aberta!")
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao enviar PWM: {e}")
            self.setup_serial_connection()  # Tentar reconectar
        
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

    def __del__(self):
        """Fechar conexão serial ao finalizar"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

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
