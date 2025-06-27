#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch para RPLidar S2 do robô Caramelo.
    
    Este launch inicia:
    1. Driver do RPLidar S2 na porta USB2
    2. Filtro laser para limitar ângulo a 180° frontal
    3. Configurações otimizadas para navegação
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Nó do RPLidar S2
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',                   # Executável correto para S2
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',                # Tipo de canal
            'serial_port': '/dev/ttyUSB2',           # LIDAR sempre na porta USB2
            'serial_baudrate': 1000000,              # Baudrate correto para S2 (1M)
            'frame_id': 'laser_frame',               # Frame do LIDAR no TF tree
            'inverted': False,                       # Não inverter leituras
            'angle_compensate': True,                # Compensação de ângulo ativada
            'scan_mode': 'DenseBoost',               # Modo de scan do S2
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('scan', '/scan')                        # Publicar diretamente no /scan
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        rplidar_node,
    ])
