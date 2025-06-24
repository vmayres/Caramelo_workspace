# caramelo_bringup

Pacote ROS 2 para integração do robô real Caramelo com ESP32 via serial, usando rodas mecanum e controle por PWM.

## Objetivo

- Receber comandos de velocidade (Twist) do ROS 2.
- Usar o plugin mecanum_drive_controller do ROS 2 Control para gerar comandos de roda.
- Converter comandos de roda em PWM e enviar via serial para a ESP32.
- (Opcional) Visualizar o robô no RViz.

## Passo a passo para testar o sistema completo

### 1. Grave o código na ESP32

- Use o Arduino IDE para gravar o arquivo `ESP32_PWM_writer.ino` (disponível neste pacote) na sua ESP32 responsável pelo controle dos motores.
- Ajuste os pinos se necessário conforme seu hardware.

### 2. Conecte a ESP32 ao PC via USB

- Verifique qual porta serial foi atribuída (exemplo: `/dev/ttyUSB0`).

### 3. Compile e instale o pacote ROS 2

```bash
cd /path/to/Caramelo_workspace
colcon build --packages-select caramelo_bringup
source install/setup.bash
```

### 4. Inicie o sistema de controle e comunicação com um único comando

```bash
ros2 launch caramelo_bringup bringup_with_control.launch.py
```

- Isso já inicializa o controller mecanum e o nó de hardware interface para enviar PWM para a ESP32.
- Certifique-se de que o nó está usando a porta serial correta (`/dev/ttyUSB0`).
- O nó irá receber comandos do controller e enviar PWM para a ESP32.
- **O nó também publica valores simulados em `/joint_states` apenas para visualização no RViz. Estes NÃO são valores reais de encoder!**

### 5. (Opcional) Visualize o robô no RViz

```bash
ros2 launch caramelo_bringup view_robot_rviz.launch.py
```

### 6. Envie comandos de velocidade para o robô

- Em outro terminal, envie comandos de velocidade para `/cmd_vel`:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

- O mecanum_drive_controller irá converter para comandos de roda, o nó Python converte para PWM e a ESP32 aplica nos motores.

---

## Estrutura dos arquivos

- `caramelo_hw_interface_node.py`: Nó para integração com o mecanum_drive_controller. Recebe comandos de velocidade das rodas do controller e envia PWM para ESP32. **Publica valores simulados em `/joint_states` apenas para visualização.**
- `launch/bringup_hw_interface.launch.py`: Lança o nó de hardware interface para integração com ROS 2 Control.
- `launch/view_robot_rviz.launch.py`: Abre o RViz para visualizar o robô e tópicos.

## Como usar

### 1. Compilar o pacote
```bash
cd /path/to/Caramelo_workspace
colcon build --packages-select caramelo_bringup
source install/setup.bash
```

### 2. Usar com mecanum_drive_controller (recomendado para controle avançado)
- Certifique-se de ter o mecanum_drive_controller instalado e configurado no seu sistema.
- Rode o controller normalmente (exemplo):
  ```bash
  ros2 launch caramelo_controller controler.launch.py
  ```
- Em outro terminal, rode o nó de hardware interface:
  ```bash
  ros2 launch caramelo_bringup bringup_hw_interface.launch.py
  ```
- Publique comandos de velocidade em `/cmd_vel` (Twist). O mecanum_drive_controller converte para comandos de roda, e o nó envia PWM para a ESP32.

### 3. Visualizar no RViz
```bash
ros2 launch caramelo_bringup view_robot_rviz.launch.py
```

## Explicação dos códigos

### caramelo_pwm_serial_node.py
- **Assina**: `/caramelo_cmd_vel` (Twist)
- **Cálculo**: Usa cinemática inversa mecanum para obter velocidade de cada roda.
- **Conversão**: Transforma velocidade angular de cada roda em PWM (ajuste constante `k_pwm` conforme necessário).
- **Envio**: Monta um JSON com os PWMs e envia via serial para a ESP32.

### caramelo_hw_interface_node.py
- **Assina**: `/mecanum_controller/commands` (Float64MultiArray, ordem: FL, FR, RL, RR)
- **Conversão**: Transforma velocidade angular de cada roda em PWM (ajuste constante `k_pwm` conforme necessário).
- **Envio**: Monta um JSON com os PWMs e envia via serial para a ESP32.
- **Fácil de expandir**: No futuro pode ler encoders e publicar `/joint_states`.

### Integração com ROS 2 Control
- O mecanum_drive_controller faz toda a lógica de cinemática e controle.
- O nó de hardware interface só converte comandos de roda para PWM.
- Isso facilita a implementação de controle em malha fechada no futuro.

### view_robot_rviz.launch.py
- Abre o RViz para visualizar o robô.
- Publica um static_transform se necessário para visualizar corretamente o modelo.

## Leitura dos encoders para controle fechado

- O robô possui uma segunda ESP32 dedicada à leitura dos encoders das rodas.
- O código `esp32_encoder_reader.ino` (no pacote) lê os encoders e envia via serial (USB) um JSON com as contagens de cada roda:
  ```json
  {"enc_fl": 12345, "enc_fr": 12340, "enc_rl": 12350, "enc_rr": 12347}
  ```
- No ROS, o nó `encoder_joint_state_node.py` lê esses valores, converte para posição/velocidade e publica em `/joint_states`.
- Use o launch:
  ```bash
  ros2 launch caramelo_bringup encoder_joint_state.launch.py
  ```
- Assim, o `joint_state_broadcaster` e o `mecanum_drive_controller` podem operar em malha fechada.

#### Vantagem desse fluxo
- Toda a lógica de cálculo, calibração e filtragem pode ser feita no ROS.
- O código da ESP32 fica simples e fácil de manter.
- O sistema fica flexível para upgrades e debug.

## Fluxo de dados

```
[ROS2 Topic /cmd_vel] --Twist--> [mecanum_drive_controller] --Float64MultiArray--> [caramelo_hw_interface_node.py] --PWM JSON via Serial--> [ESP32] --Motores-->
```

## Observações
- **Os valores publicados em `/joint_states` são simulados, apenas para visualização no RViz. Não são valores reais de encoder!**
- O nó pode ser facilmente adaptado para receber feedback real no futuro.
- Ajuste os parâmetros de PWM conforme o seu hardware.

## Manutenção
- Victor Oliveira Ayres
- victoroliveiraayres@gmail.com
