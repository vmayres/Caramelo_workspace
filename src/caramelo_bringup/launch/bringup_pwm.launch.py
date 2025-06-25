#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Caminhos dos pacotes
    caramelo_bringup_path = FindPackageShare('caramelo_bringup')
    
    # ===============================================
    # 1. Arquivo de configuração dos controladores
    # ===============================================
    robot_controllers = PathJoinSubstitution([
        caramelo_bringup_path,
        "config",
        "robot_controllers.yaml"
    ])
    
    # ===============================================
    # 2. URDF/Robot Description
    # ===============================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            caramelo_bringup_path,
            "urdf",
            "caramelo_real.urdf.xacro"
        ])
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # ===============================================
    # 3. Controller Manager (OBRIGATÓRIO para ROS2 Control!)
    # ===============================================
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
        output="screen",
        respawn=True
    )
    
    # ===============================================
    # 4. Comando para limpar a memória da ESP32 PWM (reboot)
    # ===============================================
    esp_pwm_reboot_command = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'echo "Reiniciando ESP32 dos PWMs..." && '
            'python3 -c "import serial; import time; '
            'try: '
            '    ser = serial.Serial(\\\'/dev/ttyUSB2\\\', 115200, timeout=1); '
            '    ser.setDTR(False); time.sleep(0.1); '
            '    ser.setDTR(True); time.sleep(0.5); '
            '    ser.close(); '
            '    print(\\\"ESP32 PWM reiniciada com sucesso\\\"); '
            'except Exception as e: '
            '    print(f\\\"Erro ao reiniciar ESP32 PWM: {e}\\\"); '
            '    try: '
            '        ser = serial.Serial(\\\'/dev/ttyUSB3\\\', 115200, timeout=1); '
            '        ser.setDTR(False); time.sleep(0.1); '
            '        ser.setDTR(True); time.sleep(0.5); '
            '        ser.close(); '
            '        print(\\\"ESP32 PWM reiniciada com sucesso na porta USB3\\\"); '
            '    except: '
            '        print(\\\"Falha ao reiniciar ESP32 PWM em ambas as portas\\\")"'
        ],
        output='screen'
    )
    
    # ===============================================
    # 5. Hardware Interface Node (PWM)
    # ===============================================
    hw_interface_node = Node(
        package='caramelo_bringup',
        executable='caramelo_hw_interface_node',
        name='caramelo_hw_interface_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True
    )
    
    # ===============================================
    # 6. Spawner para Mecanum Drive Controller
    # ===============================================
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30"
        ],
        output="screen"
    )
    
    # ===============================================
    # 5. RViz com configuração específica (opcional)
    # ===============================================
    
    # ===============================================
    # 8. RViz com configuração específica (opcional)
    # ===============================================
    rviz_config_file = PathJoinSubstitution([caramelo_bringup_path, 'rviz', 'caramelo_complete.rviz'])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Ordem de inicialização:
        controller_manager,                      # 1. Controller Manager
        esp_pwm_reboot_command,                 # 2. Reinicia ESP32 dos PWMs
        hw_interface_node,                      # 3. Hardware Interface (PWM)
        mecanum_drive_controller_spawner,       # 4. Mecanum Controller
        rviz_node,                              # 5. RViz (opcional)
    ])
