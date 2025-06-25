# Caramelo Bringup - Sistema Completo de Controle

Sistema ROS 2 completo para controle do robô Caramelo com rodas mecanum, incluindo:
- 🤖 Controle de hardware via 2 ESP32s (PWM para motores + Encoders para odometria)
- 🎮 Controle via teclado (teleop) 
- 📊 Visualização em RViz
- ⚙️ Controllers para navegação mecanum

## 🚀 Quick Start - Como Ligar o Robô

### 1. 🔌 Conectar Hardware
- **ESP32 #1 (Encoders):** Conectar **OBRIGATORIAMENTE** em `/dev/ttyUSB1`
- **ESP32 #2 (PWM/Motores):** Conectar preferencialmente em `/dev/ttyUSB0`
- Verificar portas: `ls /dev/ttyUSB*`

### 2. ⚡ Configurar Permissões
```bash
sudo chmod 777 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyUSB3
```

### 3. 🔧 Compilar Workspace (primeira vez)
```bash
cd ~/Caramelo_workspace
colcon build
source install/setup.bash
```

### 4. 🎯 Ligar Sistema (2 Terminais Separados) (ctrl + shift + E//O)

**Terminal 1 - ESP32 dos Encoders:**
```bash
cd ~/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup bringup_encoder.launch.py
```
*Aguarde até ver: "ESP32 encoders reiniciada com sucesso"*

**Terminal 2 - ESP32 dos PWMs:**
```bash
cd ~/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup bringup_pwm.launch.py
```

### 5. 🎮 Controlar Robô (Terminal 3) (ctrl + shift + E//O)
```bash
cd ~/Caramelo_workspace
source install/setup.bash
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```
*Este comando inicia automaticamente o conversor Twist→TwistStamped e o controle via teclado*

**Teclas de Controle:**
- **U/I**: Frente/Trás
- **O/J**: Direita/Esquerda  
- **K/L**: Rotar Esquerda/Direita
- **M**: PARAR
- **N/<**: Aumentar/Diminuir velocidade

## ✅ Verificar se Sistema está Funcionando

```bash
# 1. Verificar nós ativos
ros2 node list | grep -E "(controller_manager|caramelo|encoder)"

# 2. Verificar tópicos de controle
ros2 topic list | grep mecanum_drive_controller

# 3. Verificar controladores
ros2 control list_controllers

# 4. Teste rápido de movimento
ros2 topic pub --once /mecanum_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**✅ Sistema OK se:**
- Aparece `/controller_manager` e `/caramelo_hw_interface_node`
- Aparece `/mecanum_drive_controller/cmd_vel` no topic list
- Controllers aparecem como "active"
- Robô se move com comando de teste

## 📡 Especificações do Hardware

### ESP32 #1 - Leitura de Encoders
- **Porta:** `/dev/ttyUSB1` (EXCLUSIVA)
- **Baudrate:** 115200
- **Função:** Lê encoders e envia contagens via JSON
- **Formato:** `{"enc_fl": 1234, "enc_fr": 5678, "enc_rl": 9012, "enc_rr": 3456}`

### ESP32 #2 - Controle de Motores  
- **Porta:** `/dev/ttyUSB0` (preferencial) ou `/dev/ttyUSB2`, `/dev/ttyUSB3`
- **Baudrate:** 9600
- **Função:** Recebe comandos PWM e controla motores
- **Formato:** `{"pwm_fl": 512, "pwm_fr": 512, "pwm_rl": 512, "pwm_rr": 512}`

### Parâmetros do Robô
- **Raio das rodas:** 5 cm
- **Distância entre eixos:** 47 cm  
- **Distância entre rodas:** 31,5 cm
- **Encoders:** 114,688 pulsos/revolução (motor)
- **PWM:** 0-1023 (512 = parado, <512 = frente, >512 = trás)
- **Motores FR e RR:** Invertidos fisicamente (código compensa automaticamente)

## 🐛 Solução de Problemas

### Problema: ESP32 não conecta
```bash
# Verificar dispositivos USB
ls -la /dev/ttyUSB*

# Verificar comunicação
sudo screen /dev/ttyUSB0 115200  # ESP32 Encoders
sudo screen /dev/ttyUSB1 9600    # ESP32 PWM
# (Ctrl+A, K para sair do screen)
```

### Problema: Robô não se move
1. **Verificar se tópico existe:**
   ```bash
   ros2 topic list | grep mecanum_drive_controller/cmd_vel
   ```

2. **Se não existe, reiniciar sistema:**
   - Parar ambos os launches (Ctrl+C)
   - Desconectar/reconectar ESP32s fisicamente
   - Reiniciar launches na ordem correta

3. **Verificar logs:**
   ```bash
   ros2 topic echo /rosout | grep -i error
   ```

### Problema: Portas USB erradas
- Editar os launches para ajustar as portas USB
- Os launches tentam automaticamente portas alternativas

## 📊 Monitoramento do Sistema

```bash
# Monitorar odometria
ros2 topic echo /odom

# Monitorar comandos PWM
ros2 topic echo /mecanum_controller/commands

# Monitorar velocidades das rodas
ros2 topic echo /joint_states

# Ver árvore de TF
ros2 run tf2_tools view_frames
```

## 📁 Arquitetura do Sistema

### Fluxo de Dados
```
[Teleop] → [/cmd_vel] → [mecanum_drive_controller] → [/mecanum_controller/commands] → [caramelo_hw_interface_node] → [ESP32 PWM] → [Motores]
                                                                                                                            ↑
[ESP32 Encoders] → [encoder_joint_state_node] → [/joint_states] → [joint_state_broadcaster] ----------------------→ [Feedback]
```

### Estrutura dos Arquivos
```
caramelo_bringup/
├── launch/
│   ├── bringup_encoder.launch.py      # 🔥 ESP32 Encoders + Odometria
│   ├── bringup_pwm.launch.py          # 🔥 ESP32 PWM + Controllers
│   └── teleop_keyboard.launch.py      # 🎮 Controle via teclado
├── caramelo_bringup/
│   ├── caramelo_hw_interface_node.py  # Interface PWM
│   ├── encoder_joint_state_node.py    # Odometria real
│   └── twist_converter_node.py        # Conversor Twist→TwistStamped
├── config/
│   └── robot_controllers.yaml         # Config mecanum controller
├── urdf/
│   └── caramelo_real.urdf.xacro       # URDF para hardware real
├── rviz/
│   └── caramelo_complete.rviz         # Config RViz
├── ESP32_PWM_writer.ino               # 📟 Código ESP32 motores
├── esp32_encoder_reader.ino           # 📟 Código ESP32 encoders
└── README.md                          # 📖 Este guia
```

### Launches Disponíveis
- **`bringup_encoder.launch.py`** - Inicia ESP32 dos encoders, odometria e TF tree
- **`bringup_pwm.launch.py`** - Inicia ESP32 dos motores, controllers e RViz
- **`teleop_keyboard.launch.py`** - 🎮 Inicia controle via teclado + conversor automático

### Sistema de Conversão de Comandos
O controlador mecanum espera mensagens `TwistStamped`, mas o teleop_twist_keyboard publica `Twist` simples. 
O launch `teleop_keyboard.launch.py` resolve isso automaticamente:

```
[teleop] → [/cmd_vel] (Twist) → [twist_converter_node] → [/mecanum_drive_controller/cmd_vel] (TwistStamped) → [controlador]
```

## 🎯 Comandos de Referência Rápida

### Sequência Completa de Inicialização:
```bash
# 1. Permissões (sempre executar)
sudo chmod 777 /dev/ttyUSB*

# 2. Workspace
cd ~/Caramelo_workspace
source install/setup.bash

# 3. Terminal 1 - Encoders
ros2 launch caramelo_bringup bringup_encoder.launch.py

# 4. Terminal 2 - PWMs (aguardar Terminal 1 conectar)
ros2 launch caramelo_bringup bringup_pwm.launch.py

# 5. Terminal 3 - Controle
ros2 launch caramelo_bringup teleop_keyboard.launch.py
```

### Verificação Rápida:
```bash
# Tudo funcionando?
ros2 topic list | grep mecanum_drive_controller/cmd_vel && echo "✅ Sistema OK"

# Controllers ativos?
ros2 control list_controllers
```

## 📞 Manutenção

**Desenvolvido por:** Victor Oliveira Ayres  
**Email:** victoroliveiraayres@gmail.com  
**Data:** Junho 2025  
**ROS 2:** Jazzy  
**Sistema:** Ubuntu 24.04  

### Próximos Passos
- ✅ **Controle básico** - Funcionando
- ✅ **Odometria** - Funcionando  
- ⏳ **LIDAR** - Integrar RPLidar para SLAM
- ⏳ **Navegação** - NAV2 + SLAM Toolbox
