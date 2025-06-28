#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file para Robot Localization EKF.
    
    Este nó funde:
    - Odometria dos encoders (/odom)
    - Correções do scan matching (quando disponível)
    
    Publica:
    - /odometry/filtered - Odometria corrigida
    - TF odom->base_footprint corrigida
    """
    
    # Configurações
    package_name = 'caramelo_navigation'
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declara argumentos de launch
    declare_ekf_params_file_cmd = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=os.path.join(get_package_share_directory(package_name),
                                   'config', 'ekf_params.yaml'),
        description='Arquivo de parâmetros do EKF')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tempo de simulação')

    # Nó Robot Localization EKF
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Entrada: odometria bruta dos encoders
            ('odometry/wheel', '/odom'),
            # Saída: odometria filtrada/corrigida
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    return LaunchDescription([
        declare_ekf_params_file_cmd,
        declare_use_sim_time_cmd,
        robot_localization_node,
    ])
