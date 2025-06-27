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
    Launch file para mapeamento do ambiente com SLAM.
    
    IMPORTANTE: Este launch NÃO inclui encoder e PWM!
    Execute separadamente em terminais diferentes:
    1. Terminal 1: ros2 launch caramelo_bringup bringup_encoder.launch.py
    2. Terminal 2: ros2 launch caramelo_bringup bringup_pwm.launch.py
    3. Terminal 3: ros2 launch caramelo_navigation mapping_launch.py
    
    Este launch inicia:
    - LIDAR (rplidar)
    - SLAM Toolbox
    - Conversor Twist → TwistStamped
    - RViz para visualização
    """
    
    # Configurações
    package_name = 'caramelo_navigation'
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declara argumentos de launch
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config', 'slam_params.yaml'),
        description='Arquivo de parâmetros do SLAM')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # LIDAR (rplidar)
    start_lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('caramelo_bringup'),
                         'launch', 'bringup_lidar.launch.py'))
    )

    # Nó conversor Twist → TwistStamped (para compatibilidade com mecanum drive)
    twist_converter_node = Node(
        package='caramelo_bringup',
        executable='twist_converter_node',
        name='twist_converter',
        output='screen'
    )

    # SLAM Toolbox
    start_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Lifecycle Manager para SLAM
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['slam_toolbox']}]
    )

    # RViz com configuração para mapeamento
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(package_name),
                                      'rviz', 'mapping_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Cria a descrição de launch
    ld = LaunchDescription()

    # Declara argumentos de launch
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Adiciona ações
    ld.add_action(start_lidar_cmd)           # LIDAR
    ld.add_action(twist_converter_node)      # Conversor de comandos
    ld.add_action(start_slam_toolbox_node)   # SLAM
    ld.add_action(start_lifecycle_manager_cmd) # Lifecycle manager
    ld.add_action(start_rviz_cmd)            # RViz

    return ld
