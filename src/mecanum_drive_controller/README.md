# Mecanum Drive Controller

Um controlador ROS 2 personalizado para robôs com rodas mecanum, especificamente desenvolvido para o robô Caramelo.

## Descrição

Este pacote implementa um controlador ros2_control para robôs com rodas mecanum, permitindo movimento omnidirecional incluindo movimentação lateral. O controlador converte comandos de velocidade (linear em x, y e angular) em velocidades individuais para cada uma das quatro rodas mecanum.

## Características

- **Movimento Omnidirecional**: Suporte completo para movimento em qualquer direção
- **Integração ros2_control**: Totalmente compatível com o framework ros2_control
- **Odometria**: Cálculo de odometria baseado na posição das rodas
- **Limitadores de Velocidade**: Suporte para limitação de velocidade, aceleração e jerk
- **Publicação em Tempo Real**: Publishers em tempo real para odometria e transformações

## Dependências

- ROS 2 (testado com Humble)
- ros2_control
- controller_interface
- hardware_interface
- geometry_msgs
- nav_msgs
- tf2

## Instalação

```bash
cd ~/Caramelo_workspace
colcon build --packages-select mecanum_drive_controller
source install/setup.bash
```

## Configuração

O controlador é configurado através do arquivo `config/robot_controllers.yaml` no pacote caramelo_controller. Os principais parâmetros são:

- `wheel_base`: Distância entre rodas dianteiras e traseiras [m]
- `wheel_separation`: Distância entre rodas esquerda e direita [m]  
- `wheel_radius`: Raio das rodas [m]
- `front_left_joint_name`: Nome da junta da roda dianteira esquerda
- `front_right_joint_name`: Nome da junta da roda dianteira direita
- `back_left_joint_name`: Nome da junta da roda traseira esquerda
- `back_right_joint_name`: Nome da junta da roda traseira direita

## Uso

1. Certifique-se de que o controlador está carregado no control_manager
2. Use o launch file para carregar os controladores:

```bash
ros2 launch caramelo_controller load_ros2_controllers.launch.py
```

3. Envie comandos de velocidade:

```bash
ros2 topic pub /mecanum_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
  frame_id: ''
twist:
  linear:
    x: 0.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0"
```

## Tópicos

### Subscritos
- `~/cmd_vel` (geometry_msgs/TwistStamped): Comandos de velocidade

### Publicados
- `~/odom` (nav_msgs/Odometry): Informações de odometria
- `/tf` (tf2_msgs/TFMessage): Transformação de odometria (se habilitado)

## Estrutura do Projeto

```
mecanum_drive_controller/
├── include/mecanum_drive_controller/
│   ├── mecanum_drive_controller.hpp
│   ├── odometry.hpp
│   ├── speed_limiter.hpp
│   └── visibility_control.h
├── src/
│   ├── mecanum_drive_controller.cpp
│   ├── mecanum_drive_controller_parameter.yaml
│   ├── odometry.cpp
│   └── speed_limiter.cpp
├── CMakeLists.txt
├── package.xml
└── mecanum_drive_plugin.xml
```

## Cinemática do Mecanum Drive

O controlador implementa a cinemática inversa padrão para rodas mecanum:

```
v_fl = (vx - vy - w*(lx + ly)) / r
v_fr = (vx + vy + w*(lx + ly)) / r  
v_bl = (vx + vy - w*(lx + ly)) / r
v_br = (vx - vy + w*(lx + ly)) / r
```

Onde:
- `v_fl, v_fr, v_bl, v_br`: velocidades das rodas
- `vx, vy`: velocidades lineares
- `w`: velocidade angular
- `lx, ly`: metade da distância entre rodas
- `r`: raio da roda

## Autor

Victor Oliveira Ayres (adaptado de Addison Sears-Collins)
Data: 23 de Junho, 2025
