# INSTRUÇÕES PARA MAPEAMENTO COM CARAMELO

## Processo de Mapeamento (3 terminais separados)

### Terminal 1: Sistema de Encoders
```bash
cd /home/linux24-04/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup bringup_encoder.launch.py
```

### Terminal 2: Sistema PWM (Motores)  
```bash
cd /home/linux24-04/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup bringup_pwm.launch.py
```

### Terminal 3: Mapeamento (SLAM + LIDAR + RViz)
```bash
cd /home/linux24-04/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_navigation mapping_launch.py
```

### Terminal 4: LIDAR (Alternativa - comando direto)
```bash
# OU use o comando direto do rplidar_ros (se preferir):
ros2 launch rplidar_ros rplidar_s2_launch.py \
  channel_type:=serial \
  serial_port:=/dev/ttyUSB2 \
  serial_baudrate:=1000000 \
  frame_id:=laser_frame \
  inverted:=true \
  angle_compensate:=true \
  scan_mode:=DenseBoost
```

## O que acontece no Terminal 3:
- Inicia o LIDAR (rplidar)
- Inicia o SLAM Toolbox para mapeamento
- Inicia o conversor de comandos Twist → TwistStamped
- Abre o RViz com visualização do mapeamento

## Como fazer o mapeamento:
1. Execute os 3 comandos acima em terminais separados
2. No RViz, você verá:
   - Pontos vermelhos do LIDAR
   - Mapa sendo construído em tempo real
   - Robot model
   - Frames TF
3. Use teleop (teclado) para mover o robô e mapear o ambiente:
   ```bash
   # Em um 4º terminal:
   ros2 launch caramelo_bringup teleop_keyboard.launch.py
   ```

## Salvar o mapa:
```bash
cd /home/linux24-04/Caramelo_workspace/src/caramelo_navigation/maps
ros2 run nav2_map_server map_saver_cli -f meu_mapa
```

## Tópicos importantes:
- `/scan` - Dados do LIDAR
- `/map` - Mapa sendo construído
- `/cmd_vel` - Comandos de movimento
- `/mecanum_drive_controller/cmd_vel` - Comandos para o controlador
- `/odom` - Odometria do robô
