#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch para navegação autônoma do robô Caramelo com NAV2.
    
    Este launch inicia:
    1. NAV2 completo para navegação autônoma
    2. AMCL para localização
    3. Costmaps local e global
    4. Planejador de rotas
    5. Controlador de navegação
    """
    
    # Argumentos do launch
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default='')
    
    # Caminho para os arquivos de configuração
    pkg_caramelo_navigation = get_package_share_directory('caramelo_navigation')
    nav2_params_file = os.path.join(pkg_caramelo_navigation, 'config', 'nav2_params.yaml')
    
    # NAV2 Bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
        }.items(),
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file to load'
        ),
        
        nav2_bringup_launch,
    ])
