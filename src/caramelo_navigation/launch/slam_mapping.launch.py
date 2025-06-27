#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch para SLAM com SLAM Toolbox - Criação de mapas do robô Caramelo.
    
    Este launch inicia:
    1. SLAM Toolbox para mapeamento em tempo real
    2. Configurações otimizadas para o robô Caramelo
    3. Salvamento automático do mapa
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Caminho para os arquivos de configuração
    pkg_caramelo_navigation = get_package_share_directory('caramelo_navigation')
    slam_params_file = os.path.join(pkg_caramelo_navigation, 'config', 'slam_toolbox_params.yaml')
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('scan', 'scan'),
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        slam_toolbox_node,
    ])
