# 🔌 ORGANIZAÇÃO DAS PORTAS USB - ROBÔ CARAMELO

## Mapeamento Fixo das Portas:

| **Porta USB** | **Dispositivo** | **Função** | **Launch** |
|---------------|-----------------|------------|-----------|
| `/dev/ttyUSB0` | ESP32 PWM | Controle dos motores | `bringup_pwm.launch.py` |
| `/dev/ttyUSB1` | ESP32 Encoders | Leitura de odometria | `bringup_encoder.launch.py` |
| `/dev/ttyUSB2` | RPLidar S2 | Sensor LIDAR | `bringup_lidar.launch.py` |

## ⚠️ IMPORTANTE:

1. **Sempre conecte os dispositivos nas portas corretas**
2. **Verifique as permissões antes de usar:**
   ```bash
   sudo chmod 777 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2
   ```

3. **Para verificar dispositivos conectados:**
   ```bash
   ls -la /dev/ttyUSB*
   ```

## 🚀 Workflow de Inicialização:

### **Para Mapeamento:**
```bash
# Terminal 1 - PWM (motores)
ros2 launch caramelo_bringup bringup_pwm.launch.py

# Terminal 2 - Encoders (odometria)  
ros2 launch caramelo_bringup bringup_encoder.launch.py

# Terminal 3 - LIDAR (sensor)
ros2 launch caramelo_bringup bringup_lidar.launch.py

# Terminal 4 - SLAM (mapeamento)
ros2 launch caramelo_navigation slam_mapping.launch.py

# Terminal 5 - RViz (visualização)
ros2 launch caramelo_bringup visualization_rviz.launch.py

# Terminal 6 - Teleop (controle manual)
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

### **Para Navegação Autônoma:**
```bash
# Terminal 1 - PWM (motores)
ros2 launch caramelo_bringup bringup_pwm.launch.py

# Terminal 2 - Encoders (odometria)
ros2 launch caramelo_bringup bringup_encoder.launch.py

# Terminal 3 - LIDAR (sensor)
ros2 launch caramelo_bringup bringup_lidar.launch.py

# Terminal 4 - NAV2 (navegação)
ros2 launch caramelo_navigation navigation.launch.py map:=/path/to/map.yaml

# Terminal 5 - RViz (goals)
ros2 launch caramelo_bringup visualization_rviz.launch.py
```

## 🔧 Troubleshooting:

- **Dispositivo não encontrado?** → Verifique cabos USB
- **Permissão negada?** → Execute `sudo chmod 777 /dev/ttyUSBX`
- **Porta errada?** → Verifique com `dmesg | tail` após conectar
