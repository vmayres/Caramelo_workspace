# Caramelo Robot - ROS2 Jazzy + Gazebo Harmonic

Este workspace contém os pacotes para simulação e controle de um robô omnidirecional com rodas mecanum usando ROS2 Jazzy e Gazebo Ignition Harmonic.

## Estrutura dos Pacotes

- **caramelo_description**: Contém os arquivos URDF/Xacro do robô
- **caramelo_controller**: Configuração dos controladores ROS2 Control
- **caramelo_simulation**: Arquivos de launch para simulação no Gazebo

## Dependências Necessárias

Certifique-se de que as seguintes dependências estão instaladas:

```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-mecanum-drive-controller
sudo apt install ros-jazzy-joint-state-broadcaster
sudo apt install ros-jazzy-controller-manager
sudo apt install ros-jazzy-robot-state-publisher
sudo apt install ros-jazzy-xacro
```

## Como Construir

```bash
cd /path/to/workspace
colcon build
source install/setup.bash
```

## Como Usar

### 1. Lançar apenas o robô no Gazebo (sem controladores)

```bash
ros2 launch caramelo_simulation spawn_robot.launch.py
```

### 2. Lançar simulação completa com controladores

```bash
# Terminal 1: Lançar Gazebo e spawnar o robô
ros2 launch caramelo_simulation spawn_robot.launch.py

# Terminal 2: Lançar os controladores
ros2 launch caramelo_controller controler.launch.py
```

### 3. Testar movimento do robô

```bash
# Comandar velocidade linear e angular
ros2 topic pub /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}"

# Parar o robô
ros2 topic pub /mecanum_controller/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### 4. Verificar status dos controladores

```bash
# Listar controladores disponíveis
ros2 control list_controllers

# Ver informações de um controlador específico
ros2 control list_hardware_components
```

## Parâmetros do Robô

### Dimensões das Rodas
- **Raio**: 0.05m (5cm)
- **Largura**: 0.075m (7.5cm)

### Cinemática
- **Distância entre rodas (wheelbase)**: 0.474m
- **Largura do robô (track)**: 0.3m

## Estrutura dos Controladores

O robô utiliza:
- **joint_state_broadcaster**: Para publicar estados dos joints
- **mecanum_drive_controller**: Para controle omnidirecional

## Tópicos Principais

- `/mecanum_controller/cmd_vel` - Comandos de velocidade (geometry_msgs/Twist)
- `/mecanum_controller/odom` - Odometria (nav_msgs/Odometry)
- `/joint_states` - Estados dos joints (sensor_msgs/JointState)

## Resolução de Problemas

### Controladores não carregam
1. Verifique se o Gazebo está rodando completamente
2. Verifique se o robô foi spawnado corretamente
3. Aguarde alguns segundos após spawnar o robô antes de carregar os controladores

### Robô não se move
1. Verifique se os controladores estão ativos: `ros2 control list_controllers`
2. Verifique se os nomes dos joints estão corretos no arquivo de configuração
3. Teste com comandos manuais de velocidade

### Erros de dependências
- Certifique-se de estar usando ROS2 Jazzy
- Instale todas as dependências listadas acima
- Reconstrua o workspace: `colcon build --symlink-install`

## Contato

Mantainer: victor
Email: victoroliveiraayres@gmail.com
