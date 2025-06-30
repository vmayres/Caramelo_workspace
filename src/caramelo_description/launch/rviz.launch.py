#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file para RViz com configuração específica do robô Caramelo.
    
    Este launch pode ser usado com:
    - Encoder + PWM + LIDAR para visualização básica
    - SLAM para mapeamento
    - Navegação autônoma
    
    Configurações RViz incluem:
    - Robot model
    - TF tree
    - Laser scan (/scan)
    - Odometria (/odom)
    - Mapa (/map) quando disponível
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declara argumentos de launch
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('caramelo_description'),
            'rviz', 'caramelo.rviz'
        ),
        description='Caminho para arquivo de configuração RViz'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        rviz_node,
    ])
