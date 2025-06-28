#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file para navegação com AMCL (Monte Carlo Localization).
    
    IMPORTANTE: Este launch é para NAVEGAÇÃO em mapa já criado!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup bringup_encoder.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup bringup_pwm.launch.py
    3. Terminal 3: ros2 launch caramelo_navigation amcl_navigation_launch.py
    
    Este launch inicia:
    - LIDAR (rplidar)
    - AMCL (Monte Carlo Localization)
    - Map Server
    - Robot Localization EKF
    - RViz para visualização
    """
    
    # Configurações
    package_name = 'caramelo_navigation'
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_params_file = LaunchConfiguration('amcl_params_file')
    
    # Declara argumentos de launch
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'maps', 'meu_mapa_20250628_123340.yaml'),
        description='Caminho completo para o arquivo .yaml do mapa')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    declare_amcl_params_file_cmd = DeclareLaunchArgument(
        'amcl_params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config', 'amcl_params.yaml'),
        description='Arquivo de parâmetros do AMCL')

    # LIDAR (rplidar)
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_bringup'),
                         'launch', 'bringup_lidar.launch.py'))
    )

    # Robot Localization EKF
    start_ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name),
                         'launch', 'ekf_launch.py'))
    )

    # Conversor Twist → TwistStamped
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen'
    )

    # Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}]
    )

    # AMCL
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file,
                    {'use_sim_time': use_sim_time}]
    )

    # Lifecycle manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    # RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name),
                                      'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Argumentos
        declare_map_cmd,
        declare_use_sim_time_cmd,
        declare_amcl_params_file_cmd,
        
        # Nós e sistemas
        start_lidar_cmd,           # LIDAR
        start_ekf_cmd,             # EKF para fusão de sensores
        twist_converter_node,      # Conversor Twist
        map_server_node,           # Map Server
        amcl_node,                 # AMCL
        lifecycle_manager_node,    # Lifecycle Manager
        start_rviz_cmd,            # RViz
    ])
