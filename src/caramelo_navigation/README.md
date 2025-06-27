# Caramelo Navigation Package

Este pacote fornece funcionalidades de navegação autônoma para o robô Caramelo usando o stack NAV2 do ROS2 Jazzy.

## Funcionalidades

- **SLAM (Simultaneous Localization and Mapping)**: Mapeamento em tempo real usando SLAM Toolbox
- **Navegação Autônoma**: Navegação usando NAV2 stack
- **Compatibilidade com Mecanum Drive**: Conversão automática de cmd_vel para o controlador mecanum
- **Visualização em RViz**: Interface gráfica para monitoramento e controle

## Dependências

Este pacote depende dos seguintes pacotes ROS2:
- **NAV2 Stack**: `nav2_bringup`, `nav2_map_server`, `nav2_amcl`, `slam_toolbox`
- **Caramelo Bringup**: `caramelo_bringup` (para o conversor de cmd_vel)

## Estrutura do Pacote

```
caramelo_navigation/
├── config/
│   ├── nav2_params.yaml      # Parâmetros do NAV2
│   └── slam_params.yaml      # Parâmetros do SLAM Toolbox
├── launch/
│   ├── caramelo_navigation_launch.py  # Launch principal
│   ├── slam_launch.py        # Launch apenas para SLAM
│   └── navigation_launch.py  # Launch apenas para navegação
├── maps/
│   └── map.yaml             # Mapa exemplo
├── rviz/
│   └── nav2_default_view.rviz # Configuração do RViz
└── README.md
```

## Como Usar

### 1. Mapeamento (SLAM)

Para fazer o mapeamento do ambiente:

```bash
# Terminal 1 - Navegação com SLAM
ros2 launch caramelo_navigation caramelo_navigation_launch.py slam:=True

# Terminal 2 - Controle manual para mapeamento
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Para salvar o mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/meu_mapa
```

### 2. Navegação Autônoma

Para navegação usando um mapa já criado:

```bash
ros2 launch caramelo_navigation caramelo_navigation_launch.py slam:=False map:=/path/to/your/map.yaml
```

### 3. Apenas SLAM

Para rodar apenas o SLAM sem navegação:

```bash
ros2 launch caramelo_navigation slam_launch.py
```

### 4. Apenas Navegação

Para rodar apenas a navegação (assumindo localização externa):

```bash
ros2 launch caramelo_navigation navigation_launch.py
```

## Integração com Mecanum Drive

Este pacote inclui automaticamente o **twist_converter_node** do `caramelo_bringup` que:

- **Recebe**: `/cmd_vel` (geometry_msgs/Twist) - do NAV2
- **Converte e Publica**: `/mecanum_drive_controller/cmd_vel` (geometry_msgs/TwistStamped) - para o controlador mecanum

Isso garante compatibilidade total entre o NAV2 e o sistema de controle mecanum do robô.

## Tópicos Importantes

### Publicados pelo NAV2:
- `/cmd_vel` → Comandos de velocidade (convertidos automaticamente)

### Consumidos pelo NAV2:
- `/scan` → Dados do LiDAR
- `/odom` → Odometria do robô
- `/tf` → Transformações

### Serviços do NAV2:
- `/navigate_to_pose` → Navegar para uma pose específica
- `/navigate_through_poses` → Navegar através de múltiplas poses

## Configuração

### Parâmetros Principais (nav2_params.yaml):
- **robot_base_frame**: `base_footprint`
- **odom_frame**: `odom`
- **global_frame**: `map`
- **robot_radius**: Ajustar conforme o tamanho do robô

### Parâmetros do SLAM (slam_params.yaml):
- **odom_frame**: `odom`
- **map_frame**: `map`
- **base_frame**: `base_footprint`

## Visualização

O RViz é automaticamente iniciado com uma configuração otimizada para navegação, mostrando:
- Mapa
- Trajetória planejada
- Obstáculos
- Pose do robô
- Dados do LiDAR

## Troubleshooting

### Problemas Comuns:

1. **Robô não se move**:
   - Verificar se o twist_converter_node está rodando
   - Verificar tópicos: `ros2 topic list | grep cmd_vel`

2. **SLAM não funciona**:
   - Verificar se o LiDAR está publicando em `/scan`
   - Verificar transformações TF

3. **Navegação falha**:
   - Verificar se a odometria está sendo publicada
   - Verificar se o mapa está carregado corretamente

### Comandos Úteis:

```bash
# Verificar tópicos
ros2 topic list

# Verificar transformações
ros2 run tf2_tools view_frames

# Enviar goal manualmente
ros2 topic pub /goal_pose geometry_msgs/PoseStamped ...
```

## Exemplo de Uso Completo

1. **Preparação do ambiente**:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Caramelo_workspace
colcon build
source install/setup.bash
```

2. **Iniciar o robô** (hardware):
```bash
ros2 launch caramelo_bringup robot_bringup.launch.py
```

3. **Iniciar navegação**:
```bash
ros2 launch caramelo_navigation caramelo_navigation_launch.py
```

4. **Usar RViz** para definir goals de navegação com a ferramenta "2D Nav Goal"
